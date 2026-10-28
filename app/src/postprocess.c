#include "postprocess.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define REG_MAX 16

/* Dense-frame fix: bounded candidate pool scanned before top-K selection. */
#define PP_POOL_MAX 1024

static void sort_desc(int *idx, const int *key, int n)
{
    for (int i = 1; i < n; i++) {
        int v = idx[i], j = i - 1;
        while (j >= 0 && key[idx[j]] < key[v]) { idx[j + 1] = idx[j]; j--; }
        idx[j + 1] = v;
    }
}

static float iou_xyxy(const det_t *a, const det_t *b)
{
    float xx1 = a->x1 > b->x1 ? a->x1 : b->x1;
    float yy1 = a->y1 > b->y1 ? a->y1 : b->y1;
    float xx2 = a->x2 < b->x2 ? a->x2 : b->x2;
    float yy2 = a->y2 < b->y2 ? a->y2 : b->y2;
    float iw = xx2 - xx1 > 0 ? xx2 - xx1 : 0;
    float ih = yy2 - yy1 > 0 ? yy2 - yy1 : 0;
    float inter = iw * ih;
    float aw = a->x2 - a->x1 > 0 ? a->x2 - a->x1 : 0;
    float ah = a->y2 - a->y1 > 0 ? a->y2 - a->y1 : 0;
    float bw = b->x2 - b->x1 > 0 ? b->x2 - b->x1 : 0;
    float bh = b->y2 - b->y1 > 0 ? b->y2 - b->y1 : 0;
    return inter / (aw * ah + bw * bh - inter + 1e-9f);
}

int pp_decode(const float *const *outs, const int *chs, const int *hs, const int *ws,
              int n_out, int nc, int imgsz,
              float conf_thres, float iou_thres,
              det_t *dets, int max_det)
{
    int box_idx[8], cls_idx[8], n_box = 0, n_cls = 0;
    for (int i = 0; i < n_out; i++) {
        if (chs[i] == REG_MAX * 4 && n_box < 8) box_idx[n_box++] = i;
        else if (chs[i] == nc && n_cls < 8) cls_idx[n_cls++] = i;
    }
    sort_desc(box_idx, hs, n_box);
    sort_desc(cls_idx, hs, n_cls);

    int n = n_box < n_cls ? n_box : n_cls;
    int cnt = 0;
    /* Conservative logit-domain reject. Keep a margin so the final sigmoid
       comparison below remains authoritative near the threshold. */
    const float reject_logit =
        (conf_thres > 0.0f && conf_thres < 1.0f)
            ? logf(conf_thres / (1.0f - conf_thres)) - 0.01f
            : -INFINITY;

    /* Dense-frame fix (2026-10-10): the pre-fix code stopped scanning at the
       first max_det candidates, so in dense frames a high-score target later
       in scan order was dropped before NMS.  Collect lightweight candidates
       over the full scan into a bounded pool, then decode only the global
       top-max_det by logit (sigmoid is monotonic, so logit order equals score
       order).  Frames with at most max_det candidates are unaffected; equal
       logits keep their scan order. */
    typedef struct { float logit; short gy, gx; signed char scale, cls; } pp_cand_t;
    pp_cand_t *pool = (pp_cand_t *)malloc(sizeof(pp_cand_t) * PP_POOL_MAX);
    if (!pool) return 0;
    int npool = 0;

    for (int s = 0; s < n; s++) {
        const float *cls = outs[cls_idx[s]];
        int H = hs[box_idx[s]], W = ws[box_idx[s]];
        long HW = (long)H * W;

        for (int gy = 0; gy < H; gy++) {
            for (int gx = 0; gx < W; gx++) {
                const float *cp = cls + (long)gy * W + gx;
                /* Sigmoid is monotonic: one evaluation is enough to reject
                   background cells. For surviving cells, preserve the old
                   first-class tie rule (float sigmoid may saturate to 1). */
                float best_logit = -INFINITY;
                int bc = 0;
                for (int c = 0; c < nc; c++) {
                    float v = cp[c * HW];
                    if (v > best_logit) { best_logit = v; bc = c; }
                }
                if (best_logit < reject_logit) continue;
                float best = 1.0f / (1.0f + expf(-best_logit));
                if (best < conf_thres) continue;
                for (int c = 0; c < bc; c++) {
                    float v = 1.0f / (1.0f + expf(-cp[c * HW]));
                    if (v == best) { bc = c; break; }
                }
                if (npool < PP_POOL_MAX) {
                    pool[npool].logit = best_logit;
                    pool[npool].gy = (short)gy;
                    pool[npool].gx = (short)gx;
                    pool[npool].scale = (signed char)s;
                    pool[npool].cls = (signed char)bc;
                    npool++;
                }
            }
        }
    }

    if (npool == 0) { free(pool); return 0; }
    unsigned char *taken = (unsigned char *)calloc(npool, 1);
    if (!taken) { free(pool); return 0; }
    const int want = npool < max_det ? npool : max_det;
    for (int i = 0; i < want; i++) {
        int bi = -1;
        float bv = -INFINITY;
        for (int j = 0; j < npool; j++)
            if (!taken[j] && (bi < 0 || pool[j].logit > bv)) { bv = pool[j].logit; bi = j; }
        taken[bi] = 1;
        const int s = pool[bi].scale;
        const int gy = pool[bi].gy, gx = pool[bi].gx;
        const int H = hs[box_idx[s]], W = ws[box_idx[s]];
        const long HW = (long)H * W;
        const float *bp = outs[box_idx[s]] + (long)gy * W + gx;
        const float stride = (float)imgsz / H;
        float dist[4];
        for (int k = 0; k < 4; k++) {
            float mx = -1e30f;
            for (int b = 0; b < REG_MAX; b++) {
                float v = bp[(long)(k * REG_MAX + b) * HW];
                if (v > mx) mx = v;
            }
            float sum = 0.0f, acc = 0.0f;
            for (int b = 0; b < REG_MAX; b++) {
                float e = expf(bp[(long)(k * REG_MAX + b) * HW] - mx);
                sum += e;
                acc += e * (float)b;
            }
            dist[k] = acc / sum * (float)stride;
        }
        float ax = ((float)gx + 0.5f) * (float)stride;
        float ay = ((float)gy + 0.5f) * (float)stride;
        dets[cnt].x1 = ax - dist[0];
        dets[cnt].y1 = ay - dist[1];
        dets[cnt].x2 = ax + dist[2];
        dets[cnt].y2 = ay + dist[3];
        dets[cnt].score = 1.0f / (1.0f + expf(-bv));
        dets[cnt].cls = pool[bi].cls;
        cnt++;
    }
    free(taken);
    free(pool);

    if (cnt == 0) return 0;

    char *supp = (char *)calloc(cnt, 1);
    int *order = (int *)malloc(cnt * sizeof(int));
    int *keep = (int *)malloc(cnt * sizeof(int));
    /* [AUDIT-E1 2026-10-22] NMS buffers must be checked: the decoders already
       check pool/taken, but an OOM here dereferenced NULL on the hot path. */
    if (!supp || !order || !keep) {
        free(supp); free(order); free(keep);
        return 0;
    }
    int nkeep = 0;

    for (int c = 0; c < nc; c++) {
        int m = 0;
        for (int i = 0; i < cnt; i++)
            if (dets[i].cls == c && !supp[i]) order[m++] = i;
        /* sort candidates of this class by score desc */
        for (int i = 1; i < m; i++) {
            int v = order[i], j = i - 1;
            while (j >= 0 && dets[order[j]].score < dets[v].score) { order[j + 1] = order[j]; j--; }
            order[j + 1] = v;
        }
        for (int p = 0; p < m; p++) {
            int i = order[p];
            if (supp[i]) continue;
            keep[nkeep++] = i;
            for (int q = p + 1; q < m; q++) {
                int j = order[q];
                if (!supp[j] && iou_xyxy(&dets[i], &dets[j]) > iou_thres) supp[j] = 1;
            }
        }
    }

    /* compact kept dets to front (keep[] is in per-class score order, not sorted) */
    /* [AUDIT-E1] Guard the compaction buffer (nkeep may be 0). */
    det_t *tmp = (det_t *)malloc((nkeep > 0 ? nkeep : 1) * sizeof(det_t));
    if (!tmp) {
        free(supp); free(order); free(keep);
        return 0;
    }
    for (int i = 0; i < nkeep; i++) tmp[i] = dets[keep[i]];
    memcpy(dets, tmp, nkeep * sizeof(det_t));
    free(tmp);
    free(supp); free(order); free(keep);
    return nkeep;
}
