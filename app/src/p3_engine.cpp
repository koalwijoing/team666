// ============================================================
// p3_engine.cpp —— 感知引擎接入层实现
// ============================================================
// 与 c_bench.c / c_pp_bench.c 同口径：rknn_inputs_set + rknn_run + rknn_outputs_get(want_float=1)
// 后处理复用我方已对拍交付的 postprocess.c（pp_decode）
// 单帧流程：
//   1) SBS 1280×360 → 取左目 640×360 → letterbox 640×640（NHWC uint8 RGB）
//   2) 灯模型推理 → pp_decode(nc=7)   → 框映射回 640×360
//   3) 牌模型推理 → pp_decode(nc=1)   → 框映射回 640×360
//   4) 画框到原 SBS 图的左半（可视化）
//
// 注意：本文件在板端编译，依赖 OpenCV 与 rknn_api.h（app_pcie/includes 内已有）
// ============================================================
#include "p3_engine.h"
#include "hw_osd_config.h"
#include "perf_config.h"
#include "light_classes.h"
#include "postprocess.h"
#include "rknn_api.h"
#include "atlas_v4_shadow.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>

static float g_object_ema_alpha = 0.0f;
static int g_object_track_hold = 2;
static bool g_light_track_enabled = true;
static int g_light_track_hold = 2;
static bool g_light_verify_enabled = true;
static float g_light_verify_conf = 0.50f;
static int g_light_verify_topk = 2;
// [LIGHT-ZOOM 2026-10-21] 二阶段 ROI 放大补检位掩码：
//   bit0(1)=启用中心片；bit1(2)=再加右片（两片覆盖 75% 宽）；
//   bit2(4)=每次灯推理都跑（默认只在全帧无候选时跑）。
// 中心片优先：离线 A/B 中远灯补检几乎全部来自中间那片（208/213 帧）。
static int g_light_zoom = 0;
static int g_light_zoom_log = 0;

// ---------- 工具：计时 ----------
static inline double ms_since(const std::chrono::steady_clock::time_point& t0) {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0).count();
}

// ---------- 类别名 ----------
// 灯类别顺序唯一来源: light_classes.h（与训练导出一致，见该文件头注释）。
// 2026-10-08 实测核实前曾错位为 {stop, go, warning, stopLeft, goLeft, ...}，
// 会导致 goLeft→warning、stopLeft→goLeft 等安全级错误，禁止再手写本表。
static const char* const* LIGHT_CLS = p3_light::kLightClassNames;

// [PERF-SPLIT 2026-10-22] Opt-in per-model IO split for the P1/P2 work.
// [AUDIT-E4] Line-buffered so evidence survives a kill/crash.
static FILE* perf_split_file() {
    static FILE* f = []() -> FILE* {
        const char* p = std::getenv("P3_ENGINE_SPLIT");
        FILE* fp = p && *p ? std::fopen(p, "a") : nullptr;
        if (fp) std::setvbuf(fp, nullptr, _IOLBF, 0);
        return fp;
    }();
    return f;
}

// [PERF-INT8 2026-10-22] int8 outputs: skip the driver's full want_float
// conversion and decode the quantized tensors directly with the same affine
// (q - zp) * scale formula. Equivalence: 0/251 same-frame mismatches. This is
// the shipped default; P3_INT8_IO=0 restores the want_float path for A/B.
// P3_INT8_CHECK=1 additionally decodes a fully dequantized copy with the
// frozen float decoder and logs any mismatch (validation mode).
static bool perf_int8_io() {
    static const bool on = []() {
        const char* v = std::getenv("P3_INT8_IO");
        return !v || (v[0] != 0 && v[0] != '0' && v[0] != 'f');
    }();
    return on;
}
static bool perf_int8_check() {
    static const bool on = []() {
        const char* v = std::getenv("P3_INT8_CHECK");
        return v && v[0] != 0 && v[0] != '0' && v[0] != 'f';
    }();
    return on;
}
static FILE* perf_int8_log() {
    static FILE* f = []() -> FILE* {
        const char* p = std::getenv("P3_INT8_LOG");
        FILE* fp = p && *p ? std::fopen(p, "a") : nullptr;
        if (fp) std::setvbuf(fp, nullptr, _IOLBF, 0);
        return fp;
    }();
    return f;
}

namespace {
constexpr int kRegMax = 16;
constexpr int kPoolMax = 1024;

void sort_desc_i(int* idx, const int* key, int n)
{
    for (int i = 1; i < n; i++) {
        int v = idx[i], j = i - 1;
        while (j >= 0 && key[idx[j]] < key[v]) {
            idx[j + 1] = idx[j];
            j--;
        }
        idx[j + 1] = v;
    }
}

float iou_xyxy_i(const det_t* a, const det_t* b)
{
    const float xx1 = a->x1 > b->x1 ? a->x1 : b->x1;
    const float yy1 = a->y1 > b->y1 ? a->y1 : b->y1;
    const float xx2 = a->x2 < b->x2 ? a->x2 : b->x2;
    const float yy2 = a->y2 < b->y2 ? a->y2 : b->y2;
    const float iw = xx2 - xx1 > 0 ? xx2 - xx1 : 0;
    const float ih = yy2 - yy1 > 0 ? yy2 - yy1 : 0;
    const float inter = iw * ih;
    const float aw = a->x2 - a->x1 > 0 ? a->x2 - a->x1 : 0;
    const float ah = a->y2 - a->y1 > 0 ? a->y2 - a->y1 : 0;
    const float bw = b->x2 - b->x1 > 0 ? b->x2 - b->x1 : 0;
    const float bh = b->y2 - b->y1 > 0 ? b->y2 - b->y1 : 0;
    return inter / (aw * ah + bw * bh - inter + 1e-9f);
}

// Line-by-line twin of postprocess.c::pp_decode for int8 NCHW outputs.
int pp_decode_q8(const int8_t* const* outs, const float* scales,
                 const int* zps, const int* chs, const int* hs, const int* ws,
                 int n_out, int nc, int imgsz, float conf_thres,
                 float iou_thres, det_t* dets, int max_det)
{
    int box_idx[8], cls_idx[8], n_box = 0, n_cls = 0;
    for (int i = 0; i < n_out; i++) {
        if (chs[i] == kRegMax * 4 && n_box < 8) box_idx[n_box++] = i;
        else if (chs[i] == nc && n_cls < 8) cls_idx[n_cls++] = i;
    }
    sort_desc_i(box_idx, hs, n_box);
    sort_desc_i(cls_idx, hs, n_cls);

    const int n = n_box < n_cls ? n_box : n_cls;
    int cnt = 0;
    const float reject_logit =
        (conf_thres > 0.0f && conf_thres < 1.0f)
            ? logf(conf_thres / (1.0f - conf_thres)) - 0.01f
            : -INFINITY;

    typedef struct {
        float logit;
        short gy, gx;
        signed char scale, cls;
    } pp_cand_t;
    pp_cand_t* pool = static_cast<pp_cand_t*>(
        malloc(sizeof(pp_cand_t) * kPoolMax));
    if (!pool) return 0;
    int npool = 0;

    for (int s = 0; s < n; s++) {
        const int8_t* cls = outs[cls_idx[s]];
        const float cs = scales[cls_idx[s]];
        const int czp = zps[cls_idx[s]];
        const int H = hs[box_idx[s]], W = ws[box_idx[s]];
        const long HW = static_cast<long>(H) * W;
        for (int gy = 0; gy < H; gy++) {
            for (int gx = 0; gx < W; gx++) {
                const int8_t* cp = cls + static_cast<long>(gy) * W + gx;
                int best_q = -1000;
                int bc = 0;
                for (int c = 0; c < nc; c++) {
                    const int q = cp[static_cast<long>(c) * HW];
                    if (q > best_q) {
                        best_q = q;
                        bc = c;
                    }
                }
                const float best_logit =
                    (static_cast<float>(best_q) - czp) * cs;
                if (best_logit < reject_logit) continue;
                const float best = 1.0f / (1.0f + expf(-best_logit));
                if (best < conf_thres) continue;
                for (int c = 0; c < bc; c++) {
                    const float v = 1.0f /
                        (1.0f + expf(-((static_cast<float>(
                            cp[static_cast<long>(c) * HW]) - czp) * cs)));
                    if (v == best) {
                        bc = c;
                        break;
                    }
                }
                if (npool < kPoolMax) {
                    pool[npool].logit = best_logit;
                    pool[npool].gy = static_cast<short>(gy);
                    pool[npool].gx = static_cast<short>(gx);
                    pool[npool].scale = static_cast<signed char>(s);
                    pool[npool].cls = static_cast<signed char>(bc);
                    npool++;
                }
            }
        }
    }

    if (npool == 0) {
        free(pool);
        return 0;
    }
    unsigned char* taken = static_cast<unsigned char*>(calloc(npool, 1));
    if (!taken) {
        free(pool);
        return 0;
    }
    const int want = npool < max_det ? npool : max_det;
    for (int i = 0; i < want; i++) {
        int bi = -1;
        float bv = -INFINITY;
        for (int j = 0; j < npool; j++)
            if (!taken[j] && (bi < 0 || pool[j].logit > bv)) {
                bv = pool[j].logit;
                bi = j;
            }
        taken[bi] = 1;
        const int s = pool[bi].scale;
        const int gy = pool[bi].gy, gx = pool[bi].gx;
        const int H = hs[box_idx[s]], W = ws[box_idx[s]];
        const long HW = static_cast<long>(H) * W;
        const int8_t* bp =
            outs[box_idx[s]] + static_cast<long>(gy) * W + gx;
        const float bs = scales[box_idx[s]];
        const int bzp = zps[box_idx[s]];
        const float stride = static_cast<float>(imgsz) / H;
        float dist[4];
        for (int k = 0; k < 4; k++) {
            float mx = -1e30f;
            for (int b = 0; b < kRegMax; b++) {
                const float v = (static_cast<float>(static_cast<int>(
                    bp[static_cast<long>(k * kRegMax + b) * HW])) - bzp) * bs;
                if (v > mx) mx = v;
            }
            float sum = 0.0f, acc = 0.0f;
            for (int b = 0; b < kRegMax; b++) {
                const float v = (static_cast<float>(static_cast<int>(
                    bp[static_cast<long>(k * kRegMax + b) * HW])) - bzp) * bs;
                const float e = expf(v - mx);
                sum += e;
                acc += e * static_cast<float>(b);
            }
            dist[k] = acc / sum * stride;
        }
        const float ax = (static_cast<float>(gx) + 0.5f) * stride;
        const float ay = (static_cast<float>(gy) + 0.5f) * stride;
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

    char* supp = static_cast<char*>(calloc(cnt, 1));
    int* order = static_cast<int*>(malloc(cnt * sizeof(int)));
    int* keep = static_cast<int*>(malloc(cnt * sizeof(int)));
    // [AUDIT-E1 2026-10-22] Same OOM guard as the frozen float decoder.
    if (!supp || !order || !keep) {
        free(supp);
        free(order);
        free(keep);
        return 0;
    }
    int nkeep = 0;
    for (int c = 0; c < nc; c++) {
        int m = 0;
        for (int i = 0; i < cnt; i++)
            if (dets[i].cls == c && !supp[i]) order[m++] = i;
        for (int i = 1; i < m; i++) {
            const int v = order[i];
            int j = i - 1;
            while (j >= 0 && dets[order[j]].score < dets[v].score) {
                order[j + 1] = order[j];
                j--;
            }
            order[j + 1] = v;
        }
        for (int p = 0; p < m; p++) {
            const int i = order[p];
            if (supp[i]) continue;
            keep[nkeep++] = i;
            for (int q = p + 1; q < m; q++) {
                const int j = order[q];
                if (!supp[j] && iou_xyxy_i(&dets[i], &dets[j]) > iou_thres)
                    supp[j] = 1;
            }
        }
    }
    det_t* tmp = static_cast<det_t*>(
        malloc((nkeep > 0 ? nkeep : 1) * sizeof(det_t)));
    if (!tmp) {
        free(supp);
        free(order);
        free(keep);
        return 0;
    }
    for (int i = 0; i < nkeep; i++) tmp[i] = dets[keep[i]];
    memcpy(dets, tmp, nkeep * sizeof(det_t));
    free(tmp);
    free(supp);
    free(order);
    free(keep);
    return nkeep;
}
}  // namespace
static_assert(p3_light::kLightClassCount == 7, "light class table drifts from nc=7");
static const char* SIGN_CLS[1]  = {"sign"};
static const char* COCO_CLS[80] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush"};

static bool keep_object_class(int cls)
{
    return cls == 0 || cls == 1 || cls == 2 || cls == 3 ||
           cls == 5 || cls == 7;
}

static float box_center_distance(const P3Det& left, const P3Det& right)
{
    const float left_x = (left.x1 + left.x2) * 0.5f;
    const float left_y = (left.y1 + left.y2) * 0.5f;
    const float right_x = (right.x1 + right.x2) * 0.5f;
    const float right_y = (right.y1 + right.y2) * 0.5f;
    const float dx = left_x - right_x;
    const float dy = left_y - right_y;
    return std::sqrt(dx * dx + dy * dy);
}

static float box_iou(const P3Det& left, const P3Det& right)
{
    const float x1 = std::max(left.x1, right.x1);
    const float y1 = std::max(left.y1, right.y1);
    const float x2 = std::min(left.x2, right.x2);
    const float y2 = std::min(left.y2, right.y2);
    const float intersection = std::max(0.0f, x2 - x1) *
                               std::max(0.0f, y2 - y1);
    const float left_area = std::max(0.0f, left.x2 - left.x1) *
                            std::max(0.0f, left.y2 - left.y1);
    const float right_area = std::max(0.0f, right.x2 - right.x1) *
                             std::max(0.0f, right.y2 - right.y1);
    const float denom = left_area + right_area - intersection;
    return denom > 0.0f ? intersection / denom : 0.0f;
}

void P3Engine::update_object_tracks(const std::vector<P3Det>& detections)
{
    std::vector<bool> matched(detections.size(), false);
    for (auto& track : object_tracks_) {
        int best = -1;
        float best_score = -1.0f;
        for (std::size_t i = 0; i < detections.size(); ++i) {
            if (matched[i] || detections[i].cls != track.det.cls)
                continue;
            const float iou = box_iou(track.det, detections[i]);
            const float width = std::max(track.det.x2 - track.det.x1,
                                         detections[i].x2 - detections[i].x1);
            const float height = std::max(track.det.y2 - track.det.y1,
                                          detections[i].y2 - detections[i].y1);
            const float max_dim = std::max(width, height);
            const float distance = box_center_distance(track.det, detections[i]);
            const bool near = max_dim > 0.0f && distance < max_dim * 0.75f;
            if (iou < 0.05f && !near)
                continue;
            const float score =
                iou + (max_dim > 0.0f ? 1.0f / (1.0f + distance / max_dim) : 0.0f);
            if (score > best_score) {
                best_score = score;
                best = static_cast<int>(i);
            }
        }
        if (best >= 0) {
            const P3Det& det = detections[best];
            const float old_cx = (track.det.x1 + track.det.x2) * 0.5f;
            const float old_cy = (track.det.y1 + track.det.y2) * 0.5f;
            if (g_object_ema_alpha > 0.0f) {
                const float keep = 1.0f - g_object_ema_alpha;
                track.det.x1 = track.det.x1 * keep + det.x1 * g_object_ema_alpha;
                track.det.y1 = track.det.y1 * keep + det.y1 * g_object_ema_alpha;
                track.det.x2 = track.det.x2 * keep + det.x2 * g_object_ema_alpha;
                track.det.y2 = track.det.y2 * keep + det.y2 * g_object_ema_alpha;
            } else {
                track.det.x1 = det.x1;
                track.det.y1 = det.y1;
                track.det.x2 = det.x2;
                track.det.y2 = det.y2;
            }
            track.det.score = std::max(track.det.score * 0.8f, det.score);
            track.det.label = det.label;
            // A real successful re-detection refreshes measurement identity.
            // Keeping a track's birth timestamp made every persistent car
            // stale after 400ms, even with fresh boxes on every frame.
            // Flow-only/cache/missed paths below must NOT refresh these.
            track.det.obs_run_id = det.obs_run_id;
            track.det.obs_capture_ms = det.obs_capture_ms;
            track.det.obs_infer_done_ms = det.obs_infer_done_ms;
            track.det.obs_origin = det.obs_origin;
            const float new_cx = (track.det.x1 + track.det.x2) * 0.5f;
            const float new_cy = (track.det.y1 + track.det.y2) * 0.5f;
            track.vx = track.vx * 0.5f + (new_cx - old_cx) * 0.5f;
            track.vy = track.vy * 0.5f + (new_cy - old_cy) * 0.5f;
            track.missed = 0;
            track.det.obs_age_frames = 0;
            matched[best] = true;
        } else {
            ++track.missed;
            track.det.obs_age_frames = track.missed;
        }
    }

    for (std::size_t i = 0; i < detections.size(); ++i) {
        if (!matched[i])
            object_tracks_.push_back({detections[i], 0, 0.0f, 0.0f});
    }
    object_tracks_.erase(
        std::remove_if(object_tracks_.begin(), object_tracks_.end(),
                       [](const ObjectTrack& track) {
                           return track.missed > g_object_track_hold;
                       }),
        object_tracks_.end());
}

void P3Engine::age_object_tracks()
{
    for (auto& track : object_tracks_) {
        ++track.missed;
        track.det.obs_age_frames = track.missed;
    }
    object_tracks_.erase(
        std::remove_if(object_tracks_.begin(), object_tracks_.end(),
                       [](const ObjectTrack& track) {
                           return track.missed > g_object_track_hold;
                       }),
        object_tracks_.end());
}

void P3Engine::track_object_boxes(const cv::Mat& current_gray)
{
    if (prev_object_gray_.empty() || current_gray.empty() ||
        prev_object_gray_.size() != current_gray.size() ||
        object_tracks_.empty()) {
        return;
    }

    std::vector<cv::Point2f> previous_points;
    std::vector<int> owners;
    for (std::size_t track_index = 0; track_index < object_tracks_.size();
         ++track_index) {
        const P3Det& det = object_tracks_[track_index].det;
        const int x1 = std::clamp(static_cast<int>(std::floor(det.x1)), 0,
                                  current_gray.cols - 1);
        const int y1 = std::clamp(static_cast<int>(std::floor(det.y1)), 0,
                                  current_gray.rows - 1);
        const int x2 = std::clamp(static_cast<int>(std::ceil(det.x2)), x1 + 1,
                                  current_gray.cols);
        const int y2 = std::clamp(static_cast<int>(std::ceil(det.y2)), y1 + 1,
                                  current_gray.rows);
        if (x2 - x1 < 8 || y2 - y1 < 8)
            continue;

        for (int gy = 1; gy <= 3; ++gy) {
            for (int gx = 1; gx <= 3; ++gx) {
                const float px = x1 + (x2 - x1) * gx / 4.0f;
                const float py = y1 + (y2 - y1) * gy / 4.0f;
                previous_points.emplace_back(px, py);
                owners.push_back(static_cast<int>(track_index));
            }
        }
    }
    if (previous_points.empty())
        return;

    std::vector<cv::Point2f> next_points;
    std::vector<unsigned char> status;
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(
        prev_object_gray_, current_gray, previous_points, next_points,
        status, error, cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                         20, 0.03));

    std::vector<std::vector<float>> dx(object_tracks_.size());
    std::vector<std::vector<float>> dy(object_tracks_.size());
    for (std::size_t i = 0; i < previous_points.size(); ++i) {
        if (!status[i] || error[i] > 30.0f)
            continue;
        const int owner = owners[i];
        dx[owner].push_back(next_points[i].x - previous_points[i].x);
        dy[owner].push_back(next_points[i].y - previous_points[i].y);
    }

    for (std::size_t i = 0; i < object_tracks_.size(); ++i) {
        if (dx[i].size() < 3)
            continue;
        auto median = [](std::vector<float>& values) {
            const std::size_t middle = values.size() / 2;
            std::nth_element(values.begin(), values.begin() + middle, values.end());
            return values[middle];
        };
        const float shift_x = std::clamp(median(dx[i]), -8.0f, 8.0f);
        const float shift_y = std::clamp(median(dy[i]), -8.0f, 8.0f);
        P3Det& det = object_tracks_[i].det;
        det.x1 += shift_x;
        det.y1 += shift_y;
        det.x2 += shift_x;
        det.y2 += shift_y;
        det.obs_origin = 1;  // [N1-5] 光流更新：只动坐标，不刷新证据年龄
        object_tracks_[i].vx = shift_x;
        object_tracks_[i].vy = shift_y;
    }
}

void P3Engine::update_light_tracks(const std::vector<P3Det>& detections)
{
    std::vector<bool> matched(detections.size(), false);
    for (auto& track : light_tracks_) {
        int best = -1;
        float best_score = -1.0f;
        for (std::size_t i = 0; i < detections.size(); ++i) {
            if (matched[i])
                continue;
            const float iou = box_iou(track.det, detections[i]);
            const float width = std::max(track.det.x2 - track.det.x1,
                                         detections[i].x2 - detections[i].x1);
            const float height = std::max(track.det.y2 - track.det.y1,
                                          detections[i].y2 - detections[i].y1);
            const float max_dim = std::max(width, height);
            const float distance = box_center_distance(track.det, detections[i]);
            const bool near = max_dim > 0.0f && distance < max_dim;
            if (iou < 0.02f && !near)
                continue;
            const float score =
                iou + (max_dim > 0.0f ? 1.0f / (1.0f + distance / max_dim) : 0.0f);
            if (score > best_score) {
                best_score = score;
                best = static_cast<int>(i);
            }
        }
        if (best >= 0) {
            track.det = detections[best];
            track.missed = 0;
            matched[best] = true;
        } else {
            ++track.missed;
        }
    }

    for (std::size_t i = 0; i < detections.size(); ++i) {
        if (!matched[i])
            light_tracks_.push_back({detections[i], 0, 0.0f, 0.0f});
    }
    light_tracks_.erase(
        std::remove_if(light_tracks_.begin(), light_tracks_.end(),
                       [](const ObjectTrack& track) {
                           return track.missed > g_light_track_hold;
                       }),
        light_tracks_.end());
}

void P3Engine::age_light_tracks()
{
    for (auto& track : light_tracks_)
        ++track.missed;
    light_tracks_.erase(
        std::remove_if(light_tracks_.begin(), light_tracks_.end(),
                       [](const ObjectTrack& track) {
                           return track.missed > g_light_track_hold;
                       }),
        light_tracks_.end());
}

void P3Engine::track_light_boxes(const cv::Mat& current_gray)
{
    if (prev_light_gray_.empty() || current_gray.empty() ||
        prev_light_gray_.size() != current_gray.size() ||
        light_tracks_.empty()) {
        return;
    }

    std::vector<cv::Point2f> previous_points;
    std::vector<int> owners;
    for (std::size_t track_index = 0; track_index < light_tracks_.size();
         ++track_index) {
        const P3Det& det = light_tracks_[track_index].det;
        const int x1 = std::clamp(static_cast<int>(std::floor(det.x1)), 0,
                                  current_gray.cols - 1);
        const int y1 = std::clamp(static_cast<int>(std::floor(det.y1)), 0,
                                  current_gray.rows - 1);
        const int x2 = std::clamp(static_cast<int>(std::ceil(det.x2)), x1 + 1,
                                  current_gray.cols);
        const int y2 = std::clamp(static_cast<int>(std::ceil(det.y2)), y1 + 1,
                                  current_gray.rows);
        if (x2 - x1 < 4 || y2 - y1 < 4)
            continue;
        for (int gy = 1; gy <= 3; ++gy) {
            for (int gx = 1; gx <= 3; ++gx) {
                previous_points.emplace_back(
                    x1 + (x2 - x1) * gx / 4.0f,
                    y1 + (y2 - y1) * gy / 4.0f);
                owners.push_back(static_cast<int>(track_index));
            }
        }
    }
    if (previous_points.empty())
        return;

    std::vector<cv::Point2f> next_points;
    std::vector<unsigned char> status;
    std::vector<float> error;
    cv::calcOpticalFlowPyrLK(
        prev_light_gray_, current_gray, previous_points, next_points,
        status, error, cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,
                         20, 0.03));

    std::vector<std::vector<float>> dx(light_tracks_.size());
    std::vector<std::vector<float>> dy(light_tracks_.size());
    for (std::size_t i = 0; i < previous_points.size(); ++i) {
        if (!status[i] || error[i] > 30.0f)
            continue;
        const int owner = owners[i];
        dx[owner].push_back(next_points[i].x - previous_points[i].x);
        dy[owner].push_back(next_points[i].y - previous_points[i].y);
    }

    for (std::size_t i = 0; i < light_tracks_.size(); ++i) {
        if (dx[i].size() < 2)
            continue;
        auto median = [](std::vector<float>& values) {
            const std::size_t middle = values.size() / 2;
            std::nth_element(values.begin(), values.begin() + middle, values.end());
            return values[middle];
        };
        const float shift_x = std::clamp(median(dx[i]), -8.0f, 8.0f);
        const float shift_y = std::clamp(median(dy[i]), -8.0f, 8.0f);
        P3Det& det = light_tracks_[i].det;
        det.x1 += shift_x;
        det.y1 += shift_y;
        det.x2 += shift_x;
        det.y2 += shift_y;
        det.obs_origin = 1;  // [N1-5] 光流更新：只动坐标，不刷新证据年龄
    }
}
static void refine_light_color(const cv::Mat& sbs, P3Det& detection)
{
    const int half_width = sbs.cols / 2;
    const int left = std::clamp(static_cast<int>(std::floor(detection.x1)), 0,
                                half_width - 1);
    const int top = std::clamp(static_cast<int>(std::floor(detection.y1)), 0,
                               sbs.rows - 1);
    const int right = std::clamp(static_cast<int>(std::ceil(detection.x2)), 1,
                                 half_width);
    const int bottom = std::clamp(static_cast<int>(std::ceil(detection.y2)), 1,
                                  sbs.rows);
    if (right - left < 4 || bottom - top < 4) return;

    const int inset_x = std::max(1, (right - left) / 8);
    const int inset_y = std::max(1, (bottom - top) / 8);
    const cv::Rect box(left + inset_x, top + inset_y,
                       std::max(1, right - left - 2 * inset_x),
                       std::max(1, bottom - top - 2 * inset_y));
    cv::Mat hsv;
    cv::cvtColor(sbs(box), hsv, cv::COLOR_BGR2HSV);

    int red = 0;
    int green = 0;
    int yellow = 0;
    int colored = 0;
    for (int y = 0; y < hsv.rows; ++y) {
        const cv::Vec3b* row = hsv.ptr<cv::Vec3b>(y);
        for (int x = 0; x < hsv.cols; ++x) {
            const int hue = row[x][0];
            const int saturation = row[x][1];
            const int value = row[x][2];
            if (saturation < 60 || value < 55) continue;
            ++colored;
            if (hue <= 10 || hue >= 170)
                ++red;
            else if (hue >= 35 && hue <= 95)
                ++green;
            else if (hue >= 15 && hue <= 34)
                ++yellow;
        }
    }

    // A distant lamp bulb may occupy only a few pixels inside a mostly black
    // housing. Normalize by saturated colored pixels, not by the whole box.
    if (colored < 4) return;
    const float red_ratio = static_cast<float>(red) / colored;
    const float green_ratio = static_cast<float>(green) / colored;
    const float yellow_ratio = static_cast<float>(yellow) / colored;
    const float best = std::max({red_ratio, green_ratio, yellow_ratio});
    if (best < 0.55f) return;

    p3_light::Color new_color = p3_light::COLOR_NONE;
    if (green_ratio == best && green_ratio >= red_ratio * 1.25f &&
        green_ratio >= yellow_ratio * 1.25f) {
        new_color = p3_light::COLOR_GREEN;
    } else if (red_ratio == best && red_ratio >= green_ratio * 1.25f &&
               red_ratio >= yellow_ratio * 1.25f) {
        new_color = p3_light::COLOR_RED;
    } else if (yellow_ratio == best && yellow_ratio >= red_ratio * 1.25f &&
               yellow_ratio >= green_ratio * 1.25f) {
        new_color = p3_light::COLOR_YELLOW;
    }
    if (new_color == p3_light::COLOR_NONE) return;

    // 只更新颜色、保留模型给出的方向类别：方向（圆/直行/左转）由模型
    // 决定，颜色复核仅修正颜色词。不得把箭头重写成圆灯类别。
    p3_light::Color cur_color = p3_light::COLOR_NONE;
    p3_light::Direction dir = p3_light::DIR_UNKNOWN;
    if (!p3_light::parse_label(detection.label, &cur_color, &dir))
        return;  // 已是复核产物（unknown）或非本表标签：不重复改写

    std::string new_label;
    int new_cls = -2;
    if (p3_light::resolve_recolor(cur_color, dir, new_color, &new_label,
                                  &new_cls)) {
        detection.label = new_label;
        detection.cls = new_cls;
    }
}

// ---------- SBS 左目裁切 + letterbox ----------
cv::Mat P3Engine::sbs_left_letterbox(const cv::Mat& sbs, int imgsz,
                                     float& ratio, int& dw, int& dh) {
    // 取左目：宽度一半（1280 → 640）
    cv::Mat left = sbs(cv::Rect(0, 0, sbs.cols / 2, sbs.rows));
    int h = left.rows, w = left.cols;
    ratio = std::min((float)imgsz / h, (float)imgsz / w);
    int nh = (int)std::round(h * ratio), nw = (int)std::round(w * ratio);
    dw = (imgsz - nw) / 2;
    dh = (imgsz - nh) / 2;
    cv::Mat resized;
    cv::resize(left, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    cv::Mat out(imgsz, imgsz, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(out(cv::Rect(dw, dh, nw, nh)));
    return out;   // BGR 三通道（OpenCV 默认）
}

cv::Mat P3Engine::sbs_full_letterbox(const cv::Mat& sbs, int imgsz,
                                     float& ratio, int& dw, int& dh) {
    const int h = sbs.rows, w = sbs.cols;
    ratio = std::min((float)imgsz / h, (float)imgsz / w);
    const int nh = (int)std::round(h * ratio);
    const int nw = (int)std::round(w * ratio);
    dw = (imgsz - nw) / 2;
    dh = (imgsz - nh) / 2;

    cv::Mat resized;
    cv::resize(sbs, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    cv::Mat out(imgsz, imgsz, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(out(cv::Rect(dw, dh, nw, nh)));
    return out;
}

cv::Mat P3Engine::sbs_left_gray(const cv::Mat& sbs) {
    cv::Mat left = sbs(cv::Rect(0, 0, sbs.cols / 2, sbs.rows));
    cv::Mat gray;
    cv::cvtColor(left, gray, cv::COLOR_BGR2GRAY);
    return gray;
}

void P3Engine::track_sign_boxes(const cv::Mat& current_gray) {
    if (prev_sign_gray_.empty() || current_gray.empty() ||
        prev_sign_gray_.size() != current_gray.size()) {
        return;
    }

    for (auto& det : last_sign_) {
        int x = std::max(0, static_cast<int>(std::round(det.x1)));
        int y = std::max(0, static_cast<int>(std::round(det.y1)));
        int right = std::min(current_gray.cols, static_cast<int>(std::round(det.x2)));
        int bottom = std::min(current_gray.rows, static_cast<int>(std::round(det.y2)));
        if (right - x < 8 || bottom - y < 8) continue;

        const cv::Rect previous_rect(x, y, right - x, bottom - y);
        const cv::Mat templ = prev_sign_gray_(previous_rect);
        const int margin_x = std::max(12, previous_rect.width * 3 / 4);
        const int margin_y = std::max(12, previous_rect.height * 3 / 4);
        const int search_x = std::max(0, previous_rect.x - margin_x);
        const int search_y = std::max(0, previous_rect.y - margin_y);
        const int search_right = std::min(current_gray.cols,
                                          previous_rect.x + previous_rect.width + margin_x);
        const int search_bottom = std::min(current_gray.rows,
                                           previous_rect.y + previous_rect.height + margin_y);
        const cv::Rect search_rect(search_x, search_y,
                                   search_right - search_x,
                                   search_bottom - search_y);
        if (search_rect.width < templ.cols || search_rect.height < templ.rows)
            continue;

        cv::Mat response;
        cv::matchTemplate(current_gray(search_rect), templ, response,
                          cv::TM_CCOEFF_NORMED);
        double minimum = 0.0, maximum = 0.0;
        cv::Point minimum_location, maximum_location;
        cv::minMaxLoc(response, &minimum, &maximum, &minimum_location,
                      &maximum_location);
        if (maximum < 0.45) continue;

        const int new_x = search_rect.x + maximum_location.x;
        const int new_y = search_rect.y + maximum_location.y;
        det.x1 = static_cast<float>(new_x);
        det.y1 = static_cast<float>(new_y);
        det.x2 = static_cast<float>(new_x + previous_rect.width);
        det.y2 = static_cast<float>(new_y + previous_rect.height);
        det.obs_origin = 1;  // [N1-5] 模板跟踪更新：只动坐标，不刷新证据年龄
    }
}

// ---------- 模型加载 ----------
bool P3Engine::load_model(ModelCtx& m, const std::string& path, int nc, int imgsz) {
    FILE* fp = fopen(path.c_str(), "rb");
    if (!fp) { printf("[p3] 打开模型失败: %s\n", path.c_str()); return false; }
    fseek(fp, 0, SEEK_END);
    long sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (sz <= 0) { fclose(fp); printf("[p3] 模型文件为空: %s\n", path.c_str()); return false; }
    std::vector<unsigned char> blob(sz);
    if (fread(blob.data(), 1, sz, fp) != (size_t)sz) { fclose(fp); return false; }
    fclose(fp);

    rknn_context ctx = 0;
    if (rknn_init(&ctx, blob.data(), sz, 0, nullptr) != RKNN_SUCC) {
        printf("[p3] rknn_init 失败: %s\n", path.c_str()); return false;
    }

    // [T02-D] 所有 rknn_query 必须检查返回值；失败时销毁 context 并拒绝
    // 标记 ready（禁止静默得到"输出数=0"的伪成功）。
    rknn_input_output_num io_num{};
    if (rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num)) != RKNN_SUCC) {
        printf("[p3] rknn_query IN_OUT_NUM 失败: %s\n", path.c_str());
        rknn_destroy(ctx);
        return false;
    }
    if (io_num.n_output <= 0 || io_num.n_output > 8) {
        printf("[p3] 输出数量越界 out=%u（数组容量 8）: %s\n",
               io_num.n_output, path.c_str());
        rknn_destroy(ctx);
        return false;
    }
    m.n_out = static_cast<int>(io_num.n_output);
    for (int i = 0; i < m.n_out; ++i) {
        rknn_tensor_attr attr{};
        attr.index = i;
        if (rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr)) != RKNN_SUCC) {
            printf("[p3] rknn_query OUTPUT_ATTR 失败 (i=%d): %s\n", i, path.c_str());
            rknn_destroy(ctx);
            return false;
        }
        // 输出契约：
        //   4 维 NCHW float32 → 检测头（pp_decode 的解读方式）；
        //   2 维 [1, nc] float32 → 分类头（如灯复核 light_classifier_cn）。
        // 其余维度/布局拒绝（T02-D），但不得误伤二维分类模型。
        const bool is_det_head = (attr.n_dims == 4 &&
                                  attr.fmt == RKNN_TENSOR_NCHW);
        const bool is_cls_head = (attr.n_dims == 2);
        if (!is_det_head && !is_cls_head) {
            printf("[p3] 输出 %d 布局不符 (n_dims=%u fmt=%d): %s\n", i,
                   attr.n_dims, static_cast<int>(attr.fmt), path.c_str());
            rknn_destroy(ctx);
            return false;
        }
        m.chs[i] = attr.dims[1];
        m.hs[i]  = is_det_head ? attr.dims[2] : 1;
        m.ws[i]  = is_det_head ? attr.dims[3] : 1;
        m.sizes[i] = attr.n_elems * sizeof(float);
        m.out_type[i] = static_cast<int>(attr.type);
        m.out_qnt[i] = static_cast<int>(attr.qnt_type);
        m.out_scale[i] = attr.scale;
        m.out_zp[i] = attr.zp;
        m.out_elems[i] = attr.n_elems;
        m.out_bytes[i] = attr.size;
        std::printf("[p3] out[%d] type=%d qnt=%d scale=%.9f zp=%d elems=%u "
                    "bytes=%u\n", i, static_cast<int>(attr.type),
                    static_cast<int>(attr.qnt_type), attr.scale, attr.zp,
                    attr.n_elems, attr.size);
    }
    m.ctx = (void*)ctx;
    m.nc = nc; m.imgsz = imgsz;
    m.in_size = imgsz * imgsz * 3;
    m.int8_ok = true;
    for (int i = 0; i < m.n_out; ++i) {
        if (m.out_type[i] != RKNN_TENSOR_INT8 ||
            m.out_qnt[i] != RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC ||
            !(m.out_scale[i] > 0.0f) ||
            // [AUDIT-E3 2026-10-22] Trusting a misreported byte size would
            // let the int8 path read past the buffer.
            m.out_bytes[i] < m.out_elems[i])
            m.int8_ok = false;
    }
    m.ready = true;
    printf("[p3] 载入 %s: out=%d nc=%d imgsz=%d int8=%d\n",
           path.c_str(), m.n_out, nc, imgsz, m.int8_ok ? 1 : 0);
    return true;
}

LightVerifyResult P3Engine::light_is_real(const cv::Mat& sbs, const P3Det& detection) {
    // [T02-A] 四态语义：
    //   开关关闭 → NotEvaluated（配置性跳过，不是错误）；
    //   分类器未就绪 / 推理失败 → Error（禁止当成"通过"）；
    //   裁剪过小 → Rejected；分类通过/不通过 → Accepted/Rejected。
    if (!g_light_verify_enabled)
        return LightVerifyResult::NotEvaluated;
    if (!light_filter_.ready)
        return LightVerifyResult::Error;

    const int half_width = sbs.cols / 2;
    const int box_width = std::max(1, static_cast<int>(std::ceil(
        detection.x2 - detection.x1)));
    const int box_height = std::max(1, static_cast<int>(std::ceil(
        detection.y2 - detection.y1)));
    const int margin_x = std::max(2, box_width / 5);
    const int margin_y = std::max(2, box_height / 5);
    const int x1 = std::clamp(static_cast<int>(std::floor(detection.x1)) - margin_x,
                              0, half_width - 1);
    const int y1 = std::clamp(static_cast<int>(std::floor(detection.y1)) - margin_y,
                              0, sbs.rows - 1);
    const int x2 = std::clamp(static_cast<int>(std::ceil(detection.x2)) + margin_x,
                              x1 + 1, half_width);
    const int y2 = std::clamp(static_cast<int>(std::ceil(detection.y2)) + margin_y,
                              y1 + 1, sbs.rows);
    if (x2 - x1 < 8 || y2 - y1 < 8)
        return LightVerifyResult::Rejected;

    cv::Mat rgb;
    cv::resize(sbs(cv::Rect(x1, y1, x2 - x1, y2 - y1)), rgb,
               cv::Size(light_filter_.imgsz, light_filter_.imgsz),
               0, 0, cv::INTER_LINEAR);
    cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);

    rknn_input in{};
    in.index = 0;
    in.type = RKNN_TENSOR_UINT8;
    in.size = light_filter_.in_size;
    in.fmt = RKNN_TENSOR_NHWC;
    in.buf = rgb.data;
    if (rknn_inputs_set((rknn_context)light_filter_.ctx, 1, &in) != RKNN_SUCC)
        return LightVerifyResult::Error;
    if (rknn_run((rknn_context)light_filter_.ctx, nullptr) != RKNN_SUCC)
        return LightVerifyResult::Error;

    rknn_output out{};
    out.index = 0;
    out.want_float = 1;
    if (rknn_outputs_get((rknn_context)light_filter_.ctx, 1, &out, nullptr) !=
        RKNN_SUCC) {
        return LightVerifyResult::Error;
    }
    const float* scores = static_cast<const float*>(out.buf);
    // [AUDIT-E7 2026-10-22] Derive the classifier buffer need from the loaded
    // tensor instead of a hardcoded 2-class size.
    const std::size_t verify_need =
        light_filter_.sizes[0] > 0 ? light_filter_.sizes[0]
                                   : 2 * sizeof(float);
    const bool enough = out.size >= verify_need;
    const bool is_light = enough &&
        scores[0] >= scores[1] && scores[0] >= g_light_verify_conf;
    if (getenv("P3_LIGHT_LOG")) {
        printf("[p3-light] %.2f/%.2f %s box=(%.0f,%.0f,%.0f,%.0f)\n",
               enough ? scores[0] : 0.0f, enough ? scores[1] : 0.0f,
               is_light ? "keep" : "reject",
               detection.x1, detection.y1, detection.x2, detection.y2);
    }
    rknn_outputs_release((rknn_context)light_filter_.ctx, 1, &out);
    return is_light ? LightVerifyResult::Accepted : LightVerifyResult::Rejected;
}

bool P3Engine::init(const std::string& light_rknn, const std::string& sign_rknn,
                    const std::string& object_rknn,
                    const std::string& light_filter_rknn) {
    // 灯类别表留痕（与训练导出顺序一致性由 tests/test_light_class_map.py
    // 离线断言保证；运行期打印便于日志核对与实际模型比对）。
    printf("[p3] light class map (single source light_classes.h):");
    for (int i = 0; i < p3_light::kLightClassCount; ++i) {
        printf(" %d=%s(%s,%s)", i, p3_light::kLightClasses[i].label,
               p3_light::color_name(p3_light::kLightClasses[i].color),
               p3_light::direction_name(p3_light::kLightClasses[i].dir));
    }
    printf("\n");
    bool ok = load_model(light_, light_rknn, 7, 640);
    if (!sign_rknn.empty())
        ok = load_model(sign_, sign_rknn, 1, 640) && ok;
    if (!object_rknn.empty())
        ok = load_model(object_, object_rknn, 80, 320) && ok;
    // [T02-A] 复核分类器是正式档的必需模型：加载失败必须让 init 失败，
    // 不允许静默降级为"复核开启"（原实现不把该结果计入 ok）。
    if (!light_filter_rknn.empty())
        ok = load_model(light_filter_, light_filter_rknn, 2, 224) && ok;
    return ok;
}

// ---------- 单帧主流程 ----------
std::vector<P3Det> P3Engine::run(cv::Mat& img, float light_conf,
                                 float sign_conf, float object_conf,
                                 int sign_interval, int object_interval,
                                 int light_interval) {
    // [AUDIT-E9 2026-10-22] Cheap geometry guard: the SBS letterbox and zoom
    // band math assume at least one column per eye.
    if (img.empty() || img.cols < 8 || img.rows < 32)
        return {};

    // [OSD-MASK 2026-10-23] The FPGA OSD burns its status block (L/M/R at
    // y8..24, white-on-red ALARM at y30..46, top-left 0..56 x 0..48 of each
    // 640x360 eye) into the captured stream; the light model otherwise reads
    // the red ALARM block as a red traffic light. Cover the block with a
    // Telea inpaint of the surrounding pixels (SBS frames only) so neither
    // the model nor the display ever shows it -- and no black hole appears.
    // Paired with a verified no-OSD bitstream; default preserves legacy masking.
    static const bool hw_osd = p3_hw_osd_enabled(std::getenv("P3_HW_OSD"));
    if (hw_osd && img.cols >= 1200 && img.rows >= 48) {
        const int half = img.cols / 2;
        for (int side = 0; side < 2; ++side) {
            const int x0 = side == 0 ? 0 : half;
            const int w = std::min(56, side == 0 ? half : img.cols - half);
            const int hh = std::min(48, img.rows);
            const int ph = std::min(hh + 24, img.rows);
            if (w <= 0) continue;
            const cv::Rect patch_rect(x0, 0, w, ph);
            cv::Mat patch = img(patch_rect);
            cv::Mat mask(patch.size(), CV_8UC1, cv::Scalar(0));
            mask(cv::Rect(0, 0, w, hh)).setTo(255);
            cv::Mat filled;
            cv::inpaint(patch, mask, filled, 3, cv::INPAINT_TELEA);
            filled.copyTo(img(patch_rect));
        }
    }
    auto t_all = std::chrono::steady_clock::now();
    auto t0 = std::chrono::steady_clock::now();
    const int frame = frame_idx_;

    // [N1-1] 每帧先清零模型状态与耗时：未跑/失败帧不得沿用上帧耗时或标记。
    last_light_ms = last_sign_ms = last_object_ms = 0.0;
    last_light_scheduled = last_light_attempted = false;
    last_light_succeeded = last_light_failed = false;
    last_sign_scheduled = last_sign_attempted = false;
    last_sign_succeeded = last_sign_failed = false;
    last_object_scheduled = last_object_attempted = false;
    last_object_succeeded = last_object_failed = false;

    // 牌模型降频：仅到期才真跑（牌是静态目标；实测双模型同帧 82ms→10.4FPS，
    // 降频后 ≈45ms→20+FPS）。未到期时沿用上次结果并重新画框（persist）。
    bool do_sign = sign_.ready &&
                   (sign_interval <= 1 || (frame_idx_ % sign_interval == 0));
    last_sign_scheduled = do_sign;
    bool do_object = object_.ready &&
                     (object_interval <= 1 ||
                      (frame_idx_ % object_interval == 0));
    last_object_scheduled = do_object;
    bool do_light = light_.ready &&
                    (light_interval <= 1 ||
                     (frame_idx_ % light_interval == 0));
    last_light_scheduled = do_light;
    cv::Mat current_gray = sbs_left_gray(img);

    bool light_needs_flow = g_light_track_enabled && !do_light;
    for (const auto& track : light_tracks_) {
        if (track.missed > 0) {
            light_needs_flow = true;
            break;
        }
    }
    if (light_needs_flow && !light_tracks_.empty())
        track_light_boxes(current_gray);
    // current_gray owns a fresh buffer; cv::Mat's refcount keeps it alive for
    // the next frame. All trackers read it without mutating it.
    prev_light_gray_ = current_gray;

    // Between sign-model runs, move persisted boxes with the current image
    // instead of leaving them frozen at their previous coordinates.
    if (!do_sign && !last_sign_.empty() && sign_age_ < 30)
        track_sign_boxes(current_gray);
    prev_sign_gray_ = current_gray;
    bool object_needs_flow = !do_object;
    for (const auto& track : object_tracks_) {
        if (track.missed > 0) {
            object_needs_flow = true;
            break;
        }
    }
    if (object_needs_flow && object_age_ < g_object_track_hold + 1)
        track_object_boxes(current_gray);
    prev_object_gray_ = current_gray;

    float light_ratio; int light_dw, light_dh;
    cv::Mat light_lb, light_rgb;
    if (do_light) {
        light_lb = sbs_left_letterbox(img, 640, light_ratio, light_dw, light_dh);
        cv::cvtColor(light_lb, light_rgb, cv::COLOR_BGR2RGB);
    }

    float sign_ratio = 1.0f;
    int sign_dw = 0, sign_dh = 0;
    cv::Mat sign_rgb;
    if (do_sign) {
        // 实测：牌子单裁到左眼时容易漏，而完整 SBS 帧保留了双目上下文，
        // 离线对同一禁停标志稳定命中。牌子低频运行，仅付出 1/20 帧开销。
        cv::Mat sign_lb = sbs_full_letterbox(img, 640, sign_ratio, sign_dw, sign_dh);
        cv::cvtColor(sign_lb, sign_rgb, cv::COLOR_BGR2RGB);
    }

    float object_ratio = 1.0f;
    int object_dw = 0, object_dh = 0;
    cv::Mat object_rgb;
    if (do_object) {
        cv::Mat object_lb = sbs_left_letterbox(img, 320, object_ratio,
                                               object_dw, object_dh);
        cv::cvtColor(object_lb, object_rgb, cv::COLOR_BGR2RGB);
    }
    last_pre_ms = ms_since(t0);

    std::vector<P3Det> out;
    std::vector<P3Det> object_raw;
    std::vector<P3Det> light_raw;
    double light_zoom_ms = 0.0;  // [LIGHT-ZOOM] 补检耗时，计入灯模型耗时

    struct Item { ModelCtx* m; float conf; int model_id; const char* const* names; };
    Item items[3] = {
        {&light_, light_conf, 0, LIGHT_CLS},
        {&sign_, sign_conf, 1, SIGN_CLS},
        {&object_, object_conf, 2, COCO_CLS}};

    // [N1-1] 按 model_id 分发的模型状态标记。
    auto mark_attempted = [&](int id) {
        if (id == 0) last_light_attempted = true;
        else if (id == 1) last_sign_attempted = true;
        else last_object_attempted = true;
    };
    auto mark_failed = [&](int id) {
        if (id == 0) last_light_failed = true;
        else if (id == 1) last_sign_failed = true;
        else last_object_failed = true;
    };
    auto mark_succeeded = [&](int id) {
        if (id == 0) last_light_succeeded = true;
        else if (id == 1) last_sign_succeeded = true;
        else {
            last_object_succeeded = true;
            // [C-R2] 记录整次成功观察（含 0 框）的源身份/时间。
            last_object_success_run_id = object_run_id;
            last_object_success_capture_ms = last_capture_done_ms;
        }
    };

    for (auto& it : items) {
        bool is_sign = (it.model_id == 1);
        bool is_object = (it.model_id == 2);
        if (!it.m->ready) continue;
        if (it.model_id == 0 && !do_light) continue;
        if (is_sign && !do_sign) continue;
        if (is_object && !do_object) continue;
        cv::Mat* model_rgb = is_sign ? &sign_rgb :
                             (is_object ? &object_rgb : &light_rgb);
        const float map_ratio = is_sign ? sign_ratio :
                                (is_object ? object_ratio : light_ratio);
        const int map_dw = is_sign ? sign_dw :
                           (is_object ? object_dw : light_dw);
        const int map_dh = is_sign ? sign_dh :
                           (is_object ? object_dh : light_dh);
        auto t1 = std::chrono::steady_clock::now();
        double verify_ms = 0.0;
        mark_attempted(it.model_id);  // [N1-1] 进入实际推理调用

        rknn_input in{}; memset(&in, 0, sizeof(in));
        in.index = 0;
        in.type = RKNN_TENSOR_UINT8;
        in.size = it.m->in_size;
        in.fmt = RKNN_TENSOR_NHWC;
        in.buf = model_rgb->data;
        // [T02-D] inputs_set / run / outputs_get 全部检查返回值；
        // 任一步失败：本轮该模型无结果（不访问输出、不 release 未获取的输出），
        // 由上层把它当作"本帧无成功观察"。
        if (rknn_inputs_set((rknn_context)it.m->ctx, 1, &in) != RKNN_SUCC) {
            printf("[p3] rknn_inputs_set 失败 (model %d)\n", it.model_id);
            mark_failed(it.model_id);
            continue;
        }
        const auto t_in = std::chrono::steady_clock::now();
        if (rknn_run((rknn_context)it.m->ctx, nullptr) != RKNN_SUCC) {
            printf("[p3] rknn_run 失败 (model %d)\n", it.model_id);
            mark_failed(it.model_id);
            continue;
        }
        const auto t_run = std::chrono::steady_clock::now();
        std::vector<rknn_output> outs(it.m->n_out);
        memset(outs.data(), 0, sizeof(rknn_output) * it.m->n_out);
        const bool use_int8 = perf_int8_io() && it.m->int8_ok;
        for (int i = 0; i < it.m->n_out; ++i) {
            outs[i].index = i;
            outs[i].want_float = use_int8 ? 0 : 1;
        }
        if (rknn_outputs_get((rknn_context)it.m->ctx, it.m->n_out, outs.data(),
                             nullptr) != RKNN_SUCC) {
            printf("[p3] rknn_outputs_get 失败 (model %d)\n", it.model_id);
            mark_failed(it.model_id);
            continue;  // 未成功 get：不访问 outs.buf，也不走 release
        }
        // [N1-7] 输出有效性检查：空/短 buffer 或数量不符时不得继续解码，
        // 也不得沿用旧结果伪装成功。成功 get 的缓冲必须 release 恰好一次。
        {
            bool outputs_valid = true;
            for (int i = 0; i < it.m->n_out; ++i) {
                const size_t need = use_int8 ? it.m->out_bytes[i]
                                             : it.m->sizes[i];
                if (outs[i].buf == nullptr || outs[i].size < need) {
                    outputs_valid = false;
                    break;
                }
            }
            if (!outputs_valid) {
                printf("[p3] 输出 buffer 不合法 (model %d)\n", it.model_id);
                mark_failed(it.model_id);
                rknn_outputs_release((rknn_context)it.m->ctx, it.m->n_out,
                                     outs.data());
                continue;
            }
        }
        double inf_ms = ms_since(t1);
        const auto t_out = std::chrono::steady_clock::now();
        // [N1-5] 本轮观察身份：成功 get 后才消耗 run id（失败不消耗）。
        std::uint64_t this_run_id = 0;
        if (it.model_id == 0) this_run_id = ++light_run_id;
        else if (it.model_id == 1) this_run_id = ++sign_run_id;
        else this_run_id = ++object_run_id;
        const double infer_done_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();

        auto t2 = std::chrono::steady_clock::now();
        det_t dets[64];
        int n;
        if (use_int8) {
            const int8_t* qptrs[8];
            for (int i = 0; i < it.m->n_out; ++i)
                qptrs[i] = static_cast<const int8_t*>(outs[i].buf);
            n = pp_decode_q8(qptrs, it.m->out_scale, it.m->out_zp, it.m->chs,
                             it.m->hs, it.m->ws, it.m->n_out, it.m->nc,
                             it.m->imgsz, it.conf, 0.45f, dets, 64);
            if (perf_int8_check()) {
                // Same-frame equivalence: full dequant + frozen float decoder.
                std::vector<std::vector<float>> ref(it.m->n_out);
                const float* rptrs[8];
                for (int i = 0; i < it.m->n_out; ++i) {
                    const int8_t* q = static_cast<const int8_t*>(outs[i].buf);
                    ref[i].resize(it.m->out_elems[i]);
                    for (unsigned e = 0; e < it.m->out_elems[i]; ++e)
                        ref[i][e] =
                            (static_cast<float>(static_cast<int>(q[e])) -
                             it.m->out_zp[i]) * it.m->out_scale[i];
                    rptrs[i] = ref[i].data();
                }
                det_t rdets[64];
                const int rn = pp_decode(rptrs, it.m->chs, it.m->hs, it.m->ws,
                                         it.m->n_out, it.m->nc, it.m->imgsz,
                                         it.conf, 0.45f, rdets, 64);
                bool same = (rn == n);
                for (int k = 0; same && k < n; ++k) {
                    same = rdets[k].x1 == dets[k].x1 &&
                           rdets[k].y1 == dets[k].y1 &&
                           rdets[k].x2 == dets[k].x2 &&
                           rdets[k].y2 == dets[k].y2 &&
                           rdets[k].score == dets[k].score &&
                           rdets[k].cls == dets[k].cls;
                }
                if (FILE* lf = perf_int8_log())
                    std::fprintf(lf, "%s,model=%d,rn=%d,n=%d\n",
                                 same ? "OK" : "MISMATCH", it.model_id, rn, n);
            }
        } else {
            const float* ptrs[8];
            for (int i = 0; i < it.m->n_out; ++i)
                ptrs[i] = static_cast<const float*>(outs[i].buf);
            n = pp_decode(ptrs, it.m->chs, it.m->hs, it.m->ws, it.m->n_out,
                          it.m->nc, it.m->imgsz, it.conf, 0.45f, dets, 64);
        }
        double pp_ms = ms_since(t2);

        // 解回原图坐标（去 letterbox）。灯始终为左目 640x360；
        // 牌子映射回完整 SBS 后，优先保留左目坐标，左目为空时回退右目。
        std::vector<P3Det> sign_left, sign_right;
        std::vector<P3Det> light_candidates;
        std::vector<P3Det> light_now;
        size_t det_begin = out.size();
        for (int i = 0; i < n; ++i) {
            if (is_object && !keep_object_class(dets[i].cls))
                continue;
            P3Det d;
            d.x1 = (dets[i].x1 - map_dw) / map_ratio;
            d.y1 = (dets[i].y1 - map_dh) / map_ratio;
            d.x2 = (dets[i].x2 - map_dw) / map_ratio;
            d.y2 = (dets[i].y2 - map_dh) / map_ratio;
            d.score = dets[i].score;
            d.cls = dets[i].cls;
            d.model_id = it.model_id;
            d.label = it.names[dets[i].cls];
            // [N1-5] 观察元数据（推理产生；跟踪/缓存复用时不刷新）。
            d.obs_run_id = this_run_id;
            d.obs_capture_ms = last_capture_done_ms;
            d.obs_infer_done_ms = infer_done_ms;
            d.obs_origin = 0;
            if (!is_sign && !is_object) {
                light_candidates.push_back(d);
            }
            if (is_sign) {
                const float half_width = img.cols * 0.5f;
                const float center_x = (d.x1 + d.x2) * 0.5f;
                if (center_x >= half_width) {
                    d.x1 -= half_width;
                    d.x2 -= half_width;
                    sign_right.push_back(d);
                } else {
                    sign_left.push_back(d);
                }
            } else if (is_object) {
                object_raw.push_back(d);
            }
            // [T02-A] 灯检测只进入候选列表：未经复核的候选不再无条件进入
            // 输出（原实现在此 push light_now 造成未复核候选直接放行）。
        }
        if (it.model_id == 0) {
            // [LIGHT-ZOOM 2026-10-21] 二阶段 ROI 放大补检。
            // 全帧走左目 640x360 原生分辨率，远处 5x9px 灯在压缩/运动模糊下
            // 会连续漏检（离线 A/B：漏检段最长 27 帧）。这里把地平线带切成
            // 两片 320x180 窗口，双线性放大 2x 后再跑同一灯模型，按类 IoU
            // 合并回左目坐标；复核/跟踪/Atlas/决策链完全复用现有路径。
            // 只在全帧无候选时触发（P3_LIGHT_ZOOM=1），把 NPU 预算压在
            // 漏检帧；=2 时每次灯推理都补检（实验用）。
            const std::size_t zoom_base = light_candidates.size();
            double zoom_ms_total = 0.0;
            if ((g_light_zoom & 4) ||
                ((g_light_zoom & 3) && light_candidates.empty())) {
                const int half_w = img.cols / 2;
                const int band_h = std::max(16, img.rows * 2 / 5);
                int band_y = (img.rows - band_h) / 2 - img.rows / 12;
                band_y = std::clamp(band_y, 0,
                                    std::max(0, img.rows - band_h));
                const int tile_w =
                    std::min(half_w, std::max(16, half_w / 2));
                const int tiles = (g_light_zoom & 2) ? 2 : 1;
                for (int t = 0; t < tiles; ++t) {
                    const int x0 = (t == 0)
                                       ? (half_w - tile_w) / 2
                                       : std::max(0, half_w - tile_w);
                    cv::Mat crop = img(cv::Rect(x0, band_y, tile_w, band_h));
                    float zr = 1.0f;
                    int zdw = 0, zdh = 0;
                    cv::Mat zlb = sbs_full_letterbox(crop, light_.imgsz, zr,
                                                     zdw, zdh);
                    cv::Mat zrgb;
                    cv::cvtColor(zlb, zrgb, cv::COLOR_BGR2RGB);
                    const auto zt = std::chrono::steady_clock::now();
                    rknn_input zin{};
                    zin.index = 0;
                    zin.type = RKNN_TENSOR_UINT8;
                    zin.size = light_.in_size;
                    zin.fmt = RKNN_TENSOR_NHWC;
                    zin.buf = zrgb.data;
                    if (rknn_inputs_set((rknn_context)light_.ctx, 1, &zin) !=
                        RKNN_SUCC)
                        continue;
                    if (rknn_run((rknn_context)light_.ctx, nullptr) != RKNN_SUCC)
                        continue;
                    std::vector<rknn_output> zouts(light_.n_out);
                    memset(zouts.data(), 0, sizeof(rknn_output) * light_.n_out);
                    const bool zint8 = perf_int8_io() && light_.int8_ok;
                    for (int i = 0; i < light_.n_out; ++i) {
                        zouts[i].index = i;
                        zouts[i].want_float = zint8 ? 0 : 1;
                    }
                    if (rknn_outputs_get((rknn_context)light_.ctx, light_.n_out,
                                         zouts.data(), nullptr) != RKNN_SUCC)
                        continue;
                    det_t zdets[64];
                    int zn = 0;
                    if (zint8) {
                        const int8_t* zq[8] = {};
                        bool zvalid = true;
                        for (int i = 0; i < light_.n_out; ++i) {
                            zq[i] = static_cast<const int8_t*>(zouts[i].buf);
                            if (!zouts[i].buf ||
                                zouts[i].size < light_.out_bytes[i])
                                zvalid = false;
                        }
                        zn = zvalid ? pp_decode_q8(
                                          zq, light_.out_scale, light_.out_zp,
                                          light_.chs, light_.hs, light_.ws,
                                          light_.n_out, light_.nc,
                                          light_.imgsz, it.conf, 0.45f, zdets,
                                          64)
                                    : 0;
                        if (perf_int8_check() && zvalid) {
                            std::vector<std::vector<float>> zref(light_.n_out);
                            const float* zrptrs[8] = {};
                            for (int i = 0; i < light_.n_out; ++i) {
                                const int8_t* q =
                                    static_cast<const int8_t*>(zouts[i].buf);
                                zref[i].resize(light_.out_elems[i]);
                                for (unsigned e = 0;
                                     e < light_.out_elems[i]; ++e)
                                    zref[i][e] =
                                        (static_cast<float>(
                                             static_cast<int>(q[e])) -
                                         light_.out_zp[i]) *
                                        light_.out_scale[i];
                                zrptrs[i] = zref[i].data();
                            }
                            det_t zrdets[64];
                            const int zrn = pp_decode(
                                zrptrs, light_.chs, light_.hs, light_.ws,
                                light_.n_out, light_.nc, light_.imgsz,
                                it.conf, 0.45f, zrdets, 64);
                            bool same = (zrn == zn);
                            for (int k = 0; same && k < zn; ++k)
                                same = zrdets[k].x1 == zdets[k].x1 &&
                                       zrdets[k].y1 == zdets[k].y1 &&
                                       zrdets[k].x2 == zdets[k].x2 &&
                                       zrdets[k].y2 == zdets[k].y2 &&
                                       zrdets[k].score == zdets[k].score &&
                                       zrdets[k].cls == zdets[k].cls;
                            if (FILE* lf = perf_int8_log())
                                std::fprintf(lf,
                                             "%s,model=zoom,r=%d,n=%d\n",
                                             same ? "OK" : "MISMATCH", zrn,
                                             zn);
                        }
                    } else {
                        const float* zptrs[8] = {};
                        bool zvalid = true;
                        for (int i = 0; i < light_.n_out; ++i) {
                            zptrs[i] = (const float*)zouts[i].buf;
                            if (!zouts[i].buf || zouts[i].size < light_.sizes[i])
                                zvalid = false;
                        }
                        zn = zvalid ? pp_decode(zptrs, light_.chs, light_.hs,
                                                light_.ws, light_.n_out,
                                                light_.nc, light_.imgsz,
                                                it.conf, 0.45f, zdets, 64)
                                    : 0;
                    }
                    zoom_ms_total += ms_since(zt);
                    rknn_outputs_release((rknn_context)light_.ctx, light_.n_out,
                                         zouts.data());
                    for (int i = 0; i < zn; ++i) {
                        P3Det d;
                        d.x1 = (zdets[i].x1 - zdw) / zr + x0;
                        d.y1 = (zdets[i].y1 - zdh) / zr + band_y;
                        d.x2 = (zdets[i].x2 - zdw) / zr + x0;
                        d.y2 = (zdets[i].y2 - zdh) / zr + band_y;
                        d.score = zdets[i].score;
                        d.cls = zdets[i].cls;
                        d.model_id = 0;
                        d.label = LIGHT_CLS[zdets[i].cls];
                        d.obs_run_id = this_run_id;
                        d.obs_capture_ms = last_capture_done_ms;
                        d.obs_infer_done_ms = infer_done_ms;
                        d.obs_origin = 0;
                        bool dup = false;
                        for (auto& m : light_candidates) {
                            if (m.cls != d.cls)
                                continue;
                            const float xx1 = std::max(m.x1, d.x1);
                            const float yy1 = std::max(m.y1, d.y1);
                            const float xx2 = std::min(m.x2, d.x2);
                            const float yy2 = std::min(m.y2, d.y2);
                            const float inter =
                                std::max(0.0f, xx2 - xx1) *
                                std::max(0.0f, yy2 - yy1);
                            const float au = std::max(0.0f, m.x2 - m.x1) *
                                             std::max(0.0f, m.y2 - m.y1);
                            const float bu = std::max(0.0f, d.x2 - d.x1) *
                                             std::max(0.0f, d.y2 - d.y1);
                            if (inter / (au + bu - inter + 1e-6f) > 0.45f) {
                                if (d.score > m.score)
                                    m = d;
                                dup = true;
                                break;
                            }
                        }
                        if (!dup)
                            light_candidates.push_back(d);
                    }
                }
                if (g_light_zoom_log)
                    printf("[p3-zoom] pass ms=%.1f added=%zu\n",
                           zoom_ms_total,
                           light_candidates.size() - zoom_base);
            }
            light_zoom_ms = zoom_ms_total;
            const std::size_t zoom_added =
                light_candidates.size() - zoom_base;
            if (zoom_added > 0 && g_light_zoom_log)
                printf("[p3-zoom] full=%zu added=%zu total_ms=%.1f\n",
                       zoom_base, zoom_added, zoom_ms_total);
            std::stable_sort(light_candidates.begin(), light_candidates.end(),
                             [](const P3Det& a, const P3Det& b) {
                                 return a.score > b.score;
                             });
            // Optional simulation agreement gate. Attribute failure must never
            // leave a partial or previous run available as green evidence.
            try {
                atlas_v4_shadow().observe(img, light_candidates, this_run_id,
                                          last_capture_done_ms, frame);
            } catch (const std::exception& e) {
                atlas_v4_shadow().reset();
                printf("[atlas-guard] attribute failure; release vetoed: %s\n",e.what());
            }
            // [T02-A] 对预算内的候选执行四态复核；预算外候选保留为
            // NotEvaluated 证据（不删除其方向信息，也不作为放行依据）。
            const std::size_t verify_limit = g_light_verify_enabled
                ? std::min<std::size_t>(
                      light_candidates.size(),
                      std::min<std::size_t>(
                          8, static_cast<std::size_t>(g_light_verify_topk) +
                                 zoom_added))
                : 0;
            std::vector<LightVerifyResult> verify_results;
            verify_results.reserve(verify_limit);
            const auto verify_start = std::chrono::steady_clock::now();
            for (std::size_t i = 0; i < verify_limit; ++i) {
                const std::string original_label=light_candidates[i].label;
                const LightVerifyResult vr = light_is_real(img, light_candidates[i]);
                if (vr == LightVerifyResult::Accepted) {
                    refine_light_color(img, light_candidates[i]);
                    atlas_v4_shadow().constrain(light_candidates[i],i,this_run_id,original_label);
                }
                verify_results.push_back(vr);
            }
            verify_ms = ms_since(verify_start);
            bool degraded = false;
            light_now = assemble_light_output(light_candidates, verify_limit,
                                              verify_results, &degraded);
            // [COLOR-ONLY 2026-10-21] 用户规则：颜色决定通行，箭头方向不再
            // 参与。把已复核/已约束的灯标签按颜色塌缩为圆灯类别（绿->go、
            // 红->stop、黄->warning），桥的 classify_light 与左右方向门随即
            // 变成纯颜色语义：绿一律放行、红一律禁止。unknown/off 保持原样
            // （颜色判不出时仍按 fail-closed 处理）。
            for (P3Det& d : light_now) {
                p3_light::Color c = p3_light::COLOR_NONE;
                p3_light::Direction dir = p3_light::DIR_UNKNOWN;
                if (!p3_light::parse_label(d.label, &c, &dir))
                    continue;
                const char* collapsed =
                    p3_light::label_for(c, p3_light::DIR_ROUND);
                if (collapsed) {
                    d.label = collapsed;
                    d.cls = p3_light::class_index_of(collapsed);
                }
            }
            light_verify_degraded = degraded;
            last_verify_accepted = last_verify_rejected = 0;
            last_verify_not_evaluated = last_verify_error = 0;
            for (const P3Det& d : light_now) {
                switch (d.verify_state) {
                case kLightVerifyAccepted: ++last_verify_accepted; break;
                case kLightVerifyRejected: ++last_verify_rejected; break;
                case kLightVerifyError: ++last_verify_error; break;
                default: ++last_verify_not_evaluated; break;
                }
            }
            if (degraded) {
                printf("[p3] WARN light verify degraded: verifier error "
                       "(accepted=%d rejected=%d not_eval=%d error=%d)\n",
                       last_verify_accepted, last_verify_rejected,
                       last_verify_not_evaluated, last_verify_error);
            }
            light_raw = light_now;
        }
        if (is_sign) {
            const std::vector<P3Det>& chosen =
                !sign_left.empty() ? sign_left : sign_right;
            out.insert(out.end(), chosen.begin(), chosen.end());
        }

        if (it.model_id == 0) {
            last_light_ms = inf_ms + light_zoom_ms;
            last_pp_ms = pp_ms;
            last_light_infer_done_ms = infer_done_ms + light_zoom_ms;  // [N1-5]
        } else if (it.model_id == 1) {
            last_sign_ms = inf_ms;
            // persist：保存本轮牌结果（供降频期间复用）
            last_sign_.assign(out.begin() + det_begin, out.end());
            sign_age_ = 0;
        } else {
            last_object_ms = inf_ms;
            update_object_tracks(object_raw);
            object_age_ = 0;
        }

        rknn_outputs_release((rknn_context)it.m->ctx, it.m->n_out, outs.data());
        // 注意：outs 只 release 一次（坑清单 #11 double-free）
        if (FILE* sf = perf_split_file()) {
            const double in_ms = std::chrono::duration<double, std::milli>(
                                     t_in - t1).count();
            const double run_ms = std::chrono::duration<double, std::milli>(
                                      t_run - t_in).count();
            const double out_ms = std::chrono::duration<double, std::milli>(
                                      t_out - t_run).count();
            const unsigned long long run_id =
                static_cast<unsigned long long>(
                    it.model_id == 0 ? light_run_id
                                     : (it.model_id == 1 ? sign_run_id
                                                         : object_run_id));
            std::fprintf(sf, "%llu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", run_id,
                         it.model_id, in_ms, run_ms, out_ms, pp_ms,
                         it.model_id == 0 ? light_zoom_ms : 0.0, verify_ms);
        }
        mark_succeeded(it.model_id);  // [N1-1] 全链路成功（含解码）
    }

    if (g_light_track_enabled) {
        if (do_light)
            update_light_tracks(light_raw);
        else
            age_light_tracks();
        for (const auto& track : light_tracks_)
            out.push_back(track.det);
    } else if (do_light) {
        out.insert(out.end(), light_raw.begin(), light_raw.end());
        last_light_ = light_raw;
        light_age_ = 0;
    } else if (!last_light_.empty() && light_age_ < 3) {
        // [N1-5] 缓存复用：标记 origin=cached（保留原观察身份/时间，供桥
        // 判定证据年龄；不得当作本帧新观察）。
        for (P3Det d : last_light_) {
            d.obs_origin = 2;
            out.push_back(d);
        }
        ++light_age_;
    }

    // 牌降频期间的 persist：沿用上次牌结果（超 30 帧未更新则丢弃，避免陈旧框误导决策）
    if (!do_sign && !last_sign_.empty() && sign_age_ < 30) {
        for (P3Det d : last_sign_) {  // [N1-5] cached 标记
            d.obs_origin = 2;
            out.push_back(d);
        }
        sign_age_++;
    }
    // [残影-fix] 目标模型"已调度但失败"的帧必须按"无观察"老化跟踪：原实现只
    // 在未调度帧老化（!do_object），失败帧既不更新也不老化，连续失败时旧框会
    // 被无限期重画（照抄灯路径的语义：失败="本轮无检测结果"）。
    const bool object_failed_this_frame =
        last_object_attempted && !last_object_succeeded;
    if (!do_object || object_failed_this_frame)
        age_object_tracks();
    for (const auto& track : object_tracks_)
        out.push_back(track.det);

    static const bool det_log_enabled = []() {
        const char* e = getenv("P3_DET_LOG");
        return e && atoi(e) != 0;
    }();
    if (det_log_enabled) {
        for (const auto& d : out) {
            const char* model = d.model_id == 0 ? "light" :
                                (d.model_id == 1 ? "sign" : "object");
            printf("[p3-det] frame=%d model=%s label=%s conf=%.2f "
                   "(%.0f,%.0f,%.0f,%.0f)\n",
                   frame, model,
                   d.label.c_str(), d.score, d.x1, d.y1, d.x2, d.y2);
        }
    }

    frame_idx_++;

    // ---------- 画框（画在 SBS 左半） ----------
    const cv::Scalar C_LIGHT(0, 255, 0), C_SIGN(0, 200, 255);
    const cv::Scalar C_OBJECT(255, 180, 0);
    for (auto& d : out) {
        cv::Scalar c = d.model_id == 0 ? C_LIGHT :
                       (d.model_id == 1 ? C_SIGN : C_OBJECT);
        cv::rectangle(img, cv::Point((int)d.x1, (int)d.y1), cv::Point((int)d.x2, (int)d.y2), c, 2);
        char buf[64];
        snprintf(buf, sizeof(buf), "%s %.2f", d.label.c_str(), d.score);
        cv::putText(img, buf, cv::Point((int)d.x1, std::max(12, (int)d.y1 - 4)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, c, 1);
    }
    atlas_v4_shadow().draw(img, frame, last_light_succeeded,
                           last_light_failed, last_capture_done_ms);
    last_total_ms = ms_since(t_all);
    return out;
}

void P3Engine::warmup_models() {
    // [T01] 覆盖四个实际模型：
    //   主流程三模型（灯/牌/人车）用合成帧走一次完整 run（interval=1）；
    //   复核器用固定合法尺寸 crop 直接调用 light_is_real（不依赖检测到
    //   真灯，也不通过降低真实阈值制造预热候选）。
    if (!light_.ready)
        return;
    cv::Mat dummy(360, 640, CV_8UC3, cv::Scalar(114, 114, 114));
    run(dummy, 0.99f, 0.99f, 0.99f, 1, 1, 1);
    if (light_filter_.ready) {
        cv::Mat sbs(360, 1280, CV_8UC3, cv::Scalar(114, 114, 114));
        P3Det probe{};
        probe.x1 = 320; probe.y1 = 120; probe.x2 = 400; probe.y2 = 200;
        probe.score = 0.9f; probe.cls = 0; probe.model_id = 0;
        probe.label = "go";
        const LightVerifyResult vr = light_is_real(sbs, probe);
        printf("[p3] warmup verifier: %s\n",
               vr == LightVerifyResult::Accepted ? "accepted" :
               vr == LightVerifyResult::Rejected ? "rejected" :
               vr == LightVerifyResult::Error ? "ERROR" : "not_evaluated");
    }
}

void P3Engine::reset_state() {
    atlas_v4_shadow().reset();
    // [T01] 预热/状态清零：清跟踪、持久化、审计计数与帧计数；
    // 模型与配置保持不动。
    light_tracks_.clear();
    object_tracks_.clear();
    last_sign_.clear();
    last_light_.clear();
    last_object_.clear();
    sign_age_ = light_age_ = object_age_ = 0;
    frame_idx_ = 0;
    last_verify_accepted = last_verify_rejected = 0;
    last_verify_not_evaluated = last_verify_error = 0;
    light_verify_degraded = false;
    last_capture_done_ms = 0;
    // [C-R2] 清掉上一次成功对象观察的源身份/时间。
    last_object_success_run_id = 0;
    last_object_success_capture_ms = 0.0;
}

void P3Engine::release() {
    if (light_.ctx) { rknn_destroy((rknn_context)light_.ctx); light_.ctx = nullptr; }
    if (sign_.ctx)  { rknn_destroy((rknn_context)sign_.ctx);  sign_.ctx = nullptr; }
    if (object_.ctx) { rknn_destroy((rknn_context)object_.ctx); object_.ctx = nullptr; }
    if (light_filter_.ctx) {
        rknn_destroy((rknn_context)light_filter_.ctx);
        light_filter_.ctx = nullptr;
    }
}

// ---------- 与厂商同签名的便捷入口 ----------
static P3Engine g_engine;
static bool g_inited = false;
static bool g_init_attempted = false;  // [N3-fix] init 失败不重试（防资源爆炸）
static int  g_frame = 0;
static std::vector<P3Det> g_last_detections;

// [T00] 调度/复核/跟踪参数统一来自 perf_config：
// 环境变量只在 p3_config_load_from_env 解析一次，非法值 fail-fast。
static int g_sign_interval = -1;
static int g_object_interval = -1;
static int g_light_interval = -1;
static float g_light_conf = 0.35f;  // [ARROW-RECALL] P3_LIGHT_CONF

void p3_engine_run(cv::Mat& img_display) {
    if (g_light_interval < 0) {
        const P3RuntimeConfig& cfg = p3_config();
        g_light_interval = cfg.light_interval;
        g_sign_interval = cfg.sign_interval;
        g_object_interval = cfg.object_interval;
        g_object_ema_alpha = cfg.object_ema;
        g_object_track_hold = cfg.object_track_hold;
        g_light_track_enabled = cfg.light_track;
        g_light_track_hold = cfg.light_track_hold;
        g_light_verify_enabled = cfg.light_verify;
        g_light_verify_conf = cfg.light_verify_conf;
        g_light_verify_topk = cfg.light_verify_topk;
        g_light_conf = cfg.light_conf;
        if (const char* zoom_env = std::getenv("P3_LIGHT_ZOOM"))
            g_light_zoom = std::atoi(zoom_env);
        if (const char* zlog_env = std::getenv("P3_LIGHT_ZOOM_LOG"))
            g_light_zoom_log = std::atoi(zlog_env);
        printf("[p3] scheduling from p3_config: light=%d sign=%d object=%d | "
               "verify=%d topk=%d conf=%.2f light_conf=%.2f zoom=%d | "
               "light_track=%d hold=%d | object_ema=%.2f hold=%d\n",
               g_light_interval, g_sign_interval, g_object_interval,
               g_light_verify_enabled ? 1 : 0, g_light_verify_topk,
               static_cast<double>(g_light_verify_conf),
               static_cast<double>(g_light_conf), g_light_zoom,
               g_light_track_enabled ? 1 : 0, g_light_track_hold,
               static_cast<double>(g_object_ema_alpha), g_object_track_hold);
    }
    if (!g_inited && !g_init_attempted) {
        g_init_attempted = true;  // [N3-fix] 只尝试一次：失败后禁止每帧重试
        g_inited = g_engine.init("/root/board_deploy/lisa_v4_headless.rknn",
                                 p3_config().enable_sign ? "/root/board_deploy/sign_det_v3_headless.rknn" : "",
                                 "/root/board_deploy/coco_320_headless.rknn",
                                 "/root/board_deploy/light_classifier_cn.rknn");
        printf("[p3] engine init %s\n", g_inited ? "OK" : "FAILED (no retry)");
    }
    if (!g_inited) return;
    g_last_detections =
        g_engine.run(img_display, g_light_conf, 0.35f, 0.25f,
                     g_sign_interval, g_object_interval, g_light_interval);
    const auto& dets = g_last_detections;
    // [T01/N1-2] 1 秒滑动窗口摘要：只统计本窗口内实际成功推理样本的模型耗时
    // （未跑/失败帧不把旧耗时带入均值）；输出成功次数与失败次数。
    // result_age 标签改为 current_input_age_ms：完成 T02-B 前它是"当前输入帧
    // 的出队年龄"，不是缓存检测结果的年龄，不得用于"结果新鲜"结论。
    ++g_frame;
    {
        static auto last_log = std::chrono::steady_clock::now();
        static int win_frames = 0;
        static int win_light_ok = 0, win_sign_ok = 0, win_object_ok = 0;
        static int win_light_fail = 0, win_sign_fail = 0, win_object_fail = 0;
        static double win_pre = 0, win_light = 0, win_sign = 0, win_obj = 0,
                      win_pp = 0, win_total = 0;
        ++win_frames;
        if (g_engine.last_light_succeeded) {
            ++win_light_ok;
            win_light += g_engine.last_light_ms;
        }
        if (g_engine.last_sign_succeeded) {
            ++win_sign_ok;
            win_sign += g_engine.last_sign_ms;
        }
        if (g_engine.last_object_succeeded) {
            ++win_object_ok;
            win_obj += g_engine.last_object_ms;
        }
        win_light_fail += g_engine.last_light_failed ? 1 : 0;
        win_sign_fail += g_engine.last_sign_failed ? 1 : 0;
        win_object_fail += g_engine.last_object_failed ? 1 : 0;
        win_pre += g_engine.last_pre_ms;
        win_pp += g_engine.last_pp_ms;
        win_total += g_engine.last_total_ms;
        const auto now = std::chrono::steady_clock::now();
        const double elapsed_s =
            std::chrono::duration<double>(now - last_log).count();
        if (elapsed_s >= 1.0 && win_frames > 0) {
            last_log = now;
            const double hz = win_frames / elapsed_s;
            const double now_ms = std::chrono::duration<double, std::milli>(
                                      now.time_since_epoch())
                                      .count();
            const double input_age_ms =
                g_engine.last_capture_done_ms > 0
                    ? now_ms - g_engine.last_capture_done_ms
                    : -1.0;
            auto ok_ms = [](double sum, int ok) {
                return ok > 0 ? sum / ok : 0.0;
            };
            printf("[p3] %.2fs win(%d): %.1f Hz det=%zu | pre=%.2f "
                   "light=%.2f(ok=%d fail=%d) sign=%.2f(ok=%d fail=%d) "
                   "object=%.2f(ok=%d fail=%d) pp=%.2f total=%.2f ms | "
                   "current_input_age_ms=%.0f\n",
                   elapsed_s, win_frames, hz, dets.size(),
                   win_pre / win_frames,
                   ok_ms(win_light, win_light_ok), win_light_ok, win_light_fail,
                   ok_ms(win_sign, win_sign_ok), win_sign_ok, win_sign_fail,
                   ok_ms(win_obj, win_object_ok), win_object_ok, win_object_fail,
                   win_pp / win_frames, win_total / win_frames, input_age_ms);
            win_frames = win_light_ok = win_sign_ok = win_object_ok = 0;
            win_light_fail = win_sign_fail = win_object_fail = 0;
            win_pre = win_light = win_sign = win_obj = win_pp = win_total = 0;
        }
    }
}

// [T01] 主循环在拿到输入帧后调用，记录采集完成时刻（毫秒，steady_clock
// 绝对时间），用于结果年龄统计。当前口径为"取帧返回时刻"（dequeue 近似），
// CaptureResult 携带真实采样时间戳前不得标成传感器曝光时刻。
void p3_engine_note_capture(double steady_ms)
{
    g_engine.last_capture_done_ms = steady_ms;
}

P3ModelFrameState p3_engine_last_light_state()
{
    return {g_engine.last_light_scheduled, g_engine.last_light_attempted,
            g_engine.last_light_succeeded, g_engine.last_light_failed};
}
P3ModelFrameState p3_engine_last_sign_state()
{
    return {g_engine.last_sign_scheduled, g_engine.last_sign_attempted,
            g_engine.last_sign_succeeded, g_engine.last_sign_failed};
}
P3ModelFrameState p3_engine_last_object_state()
{
    return {g_engine.last_object_scheduled, g_engine.last_object_attempted,
            g_engine.last_object_succeeded, g_engine.last_object_failed};
}

P3LampObservation p3_engine_last_light_observation()
{
    P3LampObservation o;
    o.run_id = g_engine.light_run_id;
    o.capture_done_ms = g_engine.last_capture_done_ms;
    o.infer_done_ms = g_engine.last_light_infer_done_ms;
    o.status = g_engine.last_light_succeeded
                   ? 1
                   : (g_engine.last_light_failed ? 2 : 0);
    return o;
}

P3ObjectObservation p3_engine_last_object_observation()
{
    P3ObjectObservation o;
    o.run_id = g_engine.last_object_success_run_id;
    o.capture_done_ms = g_engine.last_object_success_capture_ms;
    o.status = g_engine.last_object_succeeded
                   ? 1
                   : (g_engine.last_object_failed ? 2 : 0);
    return o;
}

void p3_engine_warmup()
{
    if (!g_inited && !g_init_attempted) {
        // 预热前确保模型已加载（与 p3_engine_run 相同的初始化路径）。
        g_init_attempted = true;
        g_inited = g_engine.init("/root/board_deploy/lisa_v4_headless.rknn",
                                 // [AUDIT-E6] Mirror p3_engine_run: a disabled
                                 // sign model must not make init() fail here.
                                 p3_config().enable_sign ? "/root/board_deploy/sign_det_v3_headless.rknn" : "",
                                 "/root/board_deploy/coco_320_headless.rknn",
                                 "/root/board_deploy/light_classifier_cn.rknn");
        printf("[p3] engine init %s\n", g_inited ? "OK" : "FAILED (no retry)");
    }
    if (g_inited)
        g_engine.warmup_models();
}

void p3_engine_reset_state()
{
    if (g_inited)
        g_engine.reset_state();
    g_frame = 0;
    g_last_detections.clear();
}

const std::vector<P3Det>& p3_engine_last_detections()
{
    return g_last_detections;
}
