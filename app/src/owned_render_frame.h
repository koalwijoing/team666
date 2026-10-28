#ifndef P3_OWNED_RENDER_FRAME_H
#define P3_OWNED_RENDER_FRAME_H
#include <opencv2/core.hpp>

namespace p3_render {
// Captured/conversion memory is owned by hardware. CPU overlays must NEVER be
// drawn into it, including warmup: that can dirty an imported RGA target.
inline cv::Mat owned_frame(const cv::Mat& captured) {
    return captured.clone();
}
}
#endif
