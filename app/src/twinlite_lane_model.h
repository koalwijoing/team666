#ifndef TWINLITE_LANE_MODEL_H
#define TWINLITE_LANE_MODEL_H

#include <opencv2/opencv.hpp>

#include "twinlite_ego_lane_postprocess.h"
#include "lane_kalman.h"
#include "lane_pair_hysteresis.h"
#include "one_euro.h"

#include <chrono>
#include <cstdint>
#include <string>

// Visual-only TwinLiteNet+ lane-marking segmentation for the live demo.
// It deliberately does not produce lane-change or FPGA control decisions.
class TwinliteLaneModel {
public:
    bool init(const std::string& model_path);
    bool update(const cv::Mat& side_by_side_bgr,
                std::uint64_t source_delivery_id);
    void draw(cv::Mat& frame, std::uint64_t display_delivery_id,
              std::uint64_t max_age_frames) const;
    void release();

    bool ready() const { return ready_; }
    double last_ms() const { return last_ms_; }
    double postprocess_ms() const { return postprocess_ms_; }
    int lane_pixels() const { return lane_pixels_; }
    bool fresh_at(std::uint64_t display_delivery_id,
                  std::uint64_t max_age_frames) const
    {
        return ready_ && has_success_ &&
               display_delivery_id >= last_success_delivery_id_ &&
               display_delivery_id - last_success_delivery_id_ <= max_age_frames;
    }
    std::uint64_t last_success_delivery_id() const
    {
        return last_success_delivery_id_;
    }
    const EgoLanePostprocessResult& lane_result() const { return lane_result_; }

    // Caller must join the update future before reading. No cached mask is
    // returned for a failed, skipped, or different-source update.
    const cv::Mat* mask_for_success(std::uint64_t delivery_id) const {
        return fresh_at(delivery_id, 0) && !mask_.empty() ? &mask_ : nullptr;
    }

private:
    void* context_ = nullptr;
    bool ready_ = false;
    bool output_nhwc_ = false;
    int output_count_ = 0;
    double last_ms_ = 0.0;
    double postprocess_ms_ = 0.0;
    int lane_pixels_ = 0;
    std::uint64_t last_success_delivery_id_ = 0;
    bool has_success_ = false;
    cv::Mat mask_;
    EgoLanePostprocessResult lane_result_;
    // [LANE-KF 2026-10-22] Display-only smoothing of the selected pair. The
    // decision result above and the bridge never read these states.
    LaneKalman kf_left_;
    LaneKalman kf_right_;
    // [LANE-OEF 2026-10-22] Geometry-only One Euro smoothing of the selected
    // pair. P3_LANE_SMOOTH selects exactly one smoother: "oef" (default),
    // "kf" (display-only leg) or "off"; the two are never stacked.
    LaneOneEuro oef_left_;
    LaneOneEuro oef_right_;
    std::chrono::steady_clock::time_point oef_last_time_{};
    bool oef_has_time_ = false;
    // [PERF-TWIN8 2026-10-22] Output quantization. With an affine int8 output
    // and a positive per-tensor scale, the mask test class1>class0 is invariant
    // to (q-zp)*scale, so the raw int8 buffers can be compared directly
    // (P3_TWIN_INT8=1). Non-affine/float outputs keep the want_float path.
    int out_type_ = 0;
    int out_qnt_ = 0;
    float out_scale_ = 0.0f;
    int out_zp_ = 0;
    std::size_t out_elems_ = 0;
    bool out_int8_ok_ = false;
    // [PAIR-HYSTERESIS 2026-10-22] Previous pair identity anchor; a switch is
    // accepted after kPairMissFrames consecutive failed geometric matches.
    LanePairAnchor pair_anchor_;
    int pair_miss_ = 0;
};

#endif  // TWINLITE_LANE_MODEL_H
