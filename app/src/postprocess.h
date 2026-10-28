#ifndef PP_H
#define PP_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x1, y1, x2, y2;
    float score;
    int cls;
} det_t;

/* yolov8-headless decode: cls sigmoid prefilter -> DFL(16bin softmax) -> dist2bbox -> per-class NMS
 * outs[j]: NCHW float32 (1, chs[j], hs[j], ws[j]); 3 box tensors (64ch) + 3 cls tensors (nc ch), any order.
 * Semantics line-by-line mirrors rknn_postprocess.py (decode), same conf/iou thresholds.
 * Returns det count; dets are xyxy in input-image pixels. */
int pp_decode(const float *const *outs, const int *chs, const int *hs, const int *ws,
              int n_out, int nc, int imgsz,
              float conf_thres, float iou_thres,
              det_t *dets, int max_det);

#ifdef __cplusplus
}
#endif

#endif
