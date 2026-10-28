#include "twinlite_lane_model.h"
#include "lane_upgrade_observe.h"
#include "lane_shared_frontend.h"
#include <cstdlib>

namespace {
bool lane_shared_enabled() {
    const char* value = std::getenv("P3_LANE_SHARED");
    return !value || std::atoi(value) != 0;   // default ON in this tree
}
// [PERF-TWIN8 2026-10-22] Raw int8 output comparison (see header). Shipped
// default; P3_TWIN_INT8=0 restores the want_float path for A/B.
bool twin_int8_enabled() {
    static const bool on = []() {
        const char* value = std::getenv("P3_TWIN_INT8");
        return !value || (value[0] != 0 && value[0] != '0' && value[0] != 'f');
    }();
    return on;
}
}  // namespace

#include "rknn_api.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

namespace {
constexpr int kInputWidth = 512;
constexpr int kInputHeight = 288;

// [LANE-SMOOTH 2026-10-22] Exactly one smoother may touch the selected pair:
//   oef (default): One Euro on the geometry coefficients, decision-visible;
//   kf           : the earlier display-only Kalman leg, kept for the A/B;
//   off          : raw observations.
enum class LaneSmoothMode { kOef, kKf, kOff };

LaneSmoothMode lane_smooth_mode()
{
    static const LaneSmoothMode mode = []() {
        const char* value = std::getenv("P3_LANE_SMOOTH");
        const char* name = "oef";
        LaneSmoothMode parsed = LaneSmoothMode::kOef;
        if (value && std::strcmp(value, "kf") == 0) {
            parsed = LaneSmoothMode::kKf;
            name = "kf";
        } else if (value && std::strcmp(value, "off") == 0) {
            parsed = LaneSmoothMode::kOff;
            name = "off";
        } else if (value && *value && std::strcmp(value, "oef") != 0) {
            name = "oef(unknown-env)";
        }
        std::printf("[lane-smooth] mode=%s (P3_LANE_SMOOTH=%s)\n", name,
                    value ? value : "unset");
        return parsed;
    }();
    return mode;
}

// [LANE-OEF-A/B] Opt-in raw-coefficient log so both smoothers can be replayed
// offline on the same observation sequence. Never enabled by default.
FILE* lane_coef_log()
{
    static FILE* file = []() -> FILE* {
        const char* path = std::getenv("P3_LANE_COEF_LOG");
        if (!path || !*path) return nullptr;
        FILE* fp = std::fopen(path, "a");
        if (fp)
            std::setvbuf(fp, nullptr, _IOLBF, 0);  // [AUDIT-E4] crash-safe
        if (fp)
            std::fprintf(fp,
                "t_s,delivery_id,left_index,right_index,"
                "lc0,lc1,lc2,rc0,rc1,rc2,an_lx0,an_lx1,an_rx0,an_rx1\n");
        return fp;
    }();
    return file;
}

void log_lane_coefs(std::uint64_t delivery_id,
                    const EgoLanePostprocessResult& result,
                    int analyzer_left, int analyzer_right)
{
    FILE* fp = lane_coef_log();
    if (!fp) return;
    const auto now = std::chrono::steady_clock::now();
    const double t_s =
        std::chrono::duration<double>(now.time_since_epoch()).count();
    const auto coef_of = [&](int index, double* out) {
        if (index < 0 || index >= static_cast<int>(result.candidates.size()))
            return false;
        const auto& coef =
            result.candidates[static_cast<std::size_t>(index)].candidate.coef;
        out[0] = coef[0];
        out[1] = coef[1];
        out[2] = coef[2];
        return true;
    };
    // [PAIR-HYSTERESIS A/B] Probes (x at z=0 and z=1) of the analyzer's own
    // pair, so one run logs both the held and the raw selection on the same
    // frames. Held probes are derivable from the coefficients.
    const auto probe_of = [&](int index, double* out) {
        if (index < 0 || index >= static_cast<int>(result.candidates.size()))
            return false;
        const auto& candidate =
            result.candidates[static_cast<std::size_t>(index)].candidate;
        out[0] = candidate.x_at(208.0);
        out[1] = candidate.x_at(276.0);
        return true;
    };
    double left[3]{};
    double right[3]{};
    const bool has_left = coef_of(result.left_index, left);
    const bool has_right = coef_of(result.right_index, right);
    double an_left[2]{};
    double an_right[2]{};
    const bool an_has_left = probe_of(analyzer_left, an_left);
    const bool an_has_right = probe_of(analyzer_right, an_right);
    std::fprintf(fp, "%.9f,%llu,%d,%d,", t_s,
                 static_cast<unsigned long long>(delivery_id),
                 result.left_index, result.right_index);
    if (has_left)
        std::fprintf(fp, "%.6f,%.6f,%.6f,", left[0], left[1], left[2]);
    else
        std::fprintf(fp, "nan,nan,nan,");
    if (has_right)
        std::fprintf(fp, "%.6f,%.6f,%.6f,", right[0], right[1], right[2]);
    else
        std::fprintf(fp, "nan,nan,nan,");
    if (an_has_left)
        std::fprintf(fp, "%.6f,%.6f,", an_left[0], an_left[1]);
    else
        std::fprintf(fp, "nan,nan,");
    if (an_has_right)
        std::fprintf(fp, "%.6f,%.6f\n", an_right[0], an_right[1]);
    else
        std::fprintf(fp, "nan,nan\n");
}

double ms_since(const std::chrono::steady_clock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - start)
        .count();
}
}  // namespace

bool TwinliteLaneModel::init(const std::string& model_path)
{
    FILE* file = std::fopen(model_path.c_str(), "rb");
    if (!file) {
        std::printf("[twinlite] cannot open model: %s\n", model_path.c_str());
        return false;
    }
    if (std::fseek(file, 0, SEEK_END) != 0) {
        std::fclose(file);
        return false;
    }
    const long file_size = std::ftell(file);
    if (file_size <= 0 || std::fseek(file, 0, SEEK_SET) != 0) {
        std::fclose(file);
        return false;
    }
    std::vector<unsigned char> blob(static_cast<std::size_t>(file_size));
    if (std::fread(blob.data(), 1, blob.size(), file) != blob.size()) {
        std::fclose(file);
        return false;
    }
    std::fclose(file);

    rknn_context context = 0;
    if (rknn_init(&context, blob.data(), blob.size(), 0, nullptr) != RKNN_SUCC) {
        std::printf("[twinlite] rknn_init failed\n");
        return false;
    }
    context_ = reinterpret_cast<void*>(context);

    rknn_input_output_num io_num{};
    if (rknn_query(context, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num)) !=
            RKNN_SUCC ||
        io_num.n_input != 1 || io_num.n_output != 1) {
        std::printf("[twinlite] unexpected I/O count\n");
        release();
        return false;
    }

    rknn_tensor_attr input_attr{};
    input_attr.index = 0;
    if (rknn_query(context, RKNN_QUERY_INPUT_ATTR, &input_attr,
                   sizeof(input_attr)) != RKNN_SUCC) {
        std::printf("[twinlite] cannot query input shape\n");
        release();
        return false;
    }
    const bool input_nhwc = input_attr.n_dims == 4 &&
                            input_attr.dims[0] == 1 &&
                            input_attr.dims[1] == kInputHeight &&
                            input_attr.dims[2] == kInputWidth &&
                            input_attr.dims[3] == 3;
    const bool input_nchw = input_attr.n_dims == 4 &&
                            input_attr.dims[0] == 1 &&
                            input_attr.dims[1] == 3 &&
                            input_attr.dims[2] == kInputHeight &&
                            input_attr.dims[3] == kInputWidth;
    if (!input_nhwc && !input_nchw) {
        std::printf("[twinlite] unsupported input dims=%d [%d,%d,%d,%d] fmt=%d\n",
                    input_attr.n_dims, input_attr.dims[0], input_attr.dims[1],
                    input_attr.dims[2], input_attr.dims[3], input_attr.fmt);
        release();
        return false;
    }
    if (!input_nhwc) {
        std::printf("[twinlite] model input is NCHW; expected NHWC RGB from board eval\n");
        release();
        return false;
    }

    rknn_tensor_attr output_attr{};
    output_attr.index = 0;
    if (rknn_query(context, RKNN_QUERY_OUTPUT_ATTR, &output_attr,
                   sizeof(output_attr)) != RKNN_SUCC ||
        output_attr.n_dims != 4 || output_attr.dims[0] != 1) {
        std::printf("[twinlite] cannot query output shape\n");
        release();
        return false;
    }
    if (output_attr.dims[1] == 2 && output_attr.dims[2] == kInputHeight &&
        output_attr.dims[3] == kInputWidth) {
        output_nhwc_ = false;
    } else if (output_attr.dims[1] == kInputHeight &&
               output_attr.dims[2] == kInputWidth && output_attr.dims[3] == 2) {
        output_nhwc_ = true;
    } else {
        std::printf("[twinlite] unsupported output dims=[%d,%d,%d,%d] fmt=%d\n",
                    output_attr.dims[0], output_attr.dims[1],
                    output_attr.dims[2], output_attr.dims[3], output_attr.fmt);
        release();
        return false;
    }

    output_count_ = static_cast<int>(io_num.n_output);
    out_type_ = static_cast<int>(output_attr.type);
    out_qnt_ = static_cast<int>(output_attr.qnt_type);
    out_scale_ = output_attr.scale;
    out_zp_ = output_attr.zp;
    out_elems_ = output_attr.n_elems;
    out_int8_ok_ = out_type_ == RKNN_TENSOR_INT8 &&
                   out_qnt_ == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC &&
                   out_scale_ > 0.0f;
    ready_ = true;
    std::printf("[twinlite] engine init OK input=[1,288,512,3] output_nhwc=%d fmt=%d int8=%d scale=%.9f zp=%d\n",
                output_nhwc_ ? 1 : 0, output_attr.fmt,
                out_int8_ok_ ? 1 : 0, output_attr.scale, output_attr.zp);
    return true;
}

bool TwinliteLaneModel::update(const cv::Mat& side_by_side_bgr,
                               std::uint64_t source_delivery_id)
{
    if (!ready_ || side_by_side_bgr.empty() || side_by_side_bgr.cols < 2 ||
        side_by_side_bgr.rows < 32)
        return false;

    const auto start = std::chrono::steady_clock::now();
    // The capture is a 2-up 1280x360 frame. Each camera view is 16:9;
    // evaluate the left camera only, matching the prior UFLD demo geometry.
    const cv::Mat left = side_by_side_bgr(
        cv::Rect(0, 0, side_by_side_bgr.cols / 2, side_by_side_bgr.rows));
    cv::Mat resized_bgr, input_rgb;
    cv::resize(left, resized_bgr, cv::Size(kInputWidth, kInputHeight), 0, 0,
               cv::INTER_AREA);
    cv::cvtColor(resized_bgr, input_rgb, cv::COLOR_BGR2RGB);

    rknn_input input{};
    input.index = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.size = static_cast<std::uint32_t>(input_rgb.total() * input_rgb.elemSize());
    input.fmt = RKNN_TENSOR_NHWC;
    input.buf = input_rgb.data;
    const rknn_context context = reinterpret_cast<rknn_context>(context_);
    if (rknn_inputs_set(context, 1, &input) != RKNN_SUCC ||
        rknn_run(context, nullptr) != RKNN_SUCC) {
        std::printf("[twinlite] inference failed at frame=%llu\n",
                    static_cast<unsigned long long>(source_delivery_id));
        return false;
    }

    std::vector<rknn_output> outputs(static_cast<std::size_t>(output_count_));
    std::memset(outputs.data(), 0, sizeof(rknn_output) * outputs.size());
    for (int index = 0; index < output_count_; ++index) {
        outputs[index].index = index;
        outputs[index].want_float =
            (twin_int8_enabled() && out_int8_ok_) ? 0 : 1;
    }
    if (rknn_outputs_get(context, output_count_, outputs.data(), nullptr) != RKNN_SUCC) {
        std::printf("[twinlite] output retrieval failed at frame=%llu\n",
                    static_cast<unsigned long long>(source_delivery_id));
        return false;
    }

    const bool use_int8 = twin_int8_enabled() && out_int8_ok_;
    // [AUDIT-E2 2026-10-22] Validate the output length before decoding: a
    // short or misreported buffer used to be an out-of-bounds read.
    const std::size_t need_bytes =
        out_elems_ * static_cast<std::size_t>(use_int8 ? 1 : 4);
    bool ok = outputs[0].buf != nullptr && outputs[0].size >= need_bytes;
    cv::Mat new_mask(kInputHeight, kInputWidth, CV_8UC1, cv::Scalar(0));
    if (ok) {
        const float* logits =
            use_int8 ? nullptr : static_cast<const float*>(outputs[0].buf);
        const std::int8_t* qlogits =
            use_int8 ? static_cast<const std::int8_t*>(outputs[0].buf) : nullptr;
        for (int y = 0; y < kInputHeight; ++y) {
            std::uint8_t* mask_row = new_mask.ptr<std::uint8_t>(y);
            for (int x = 0; x < kInputWidth; ++x) {
                const std::size_t pixel =
                    static_cast<std::size_t>(y) * kInputWidth + x;
                const std::size_t class0 = output_nhwc_ ? pixel * 2 : pixel;
                const std::size_t class1 = output_nhwc_ ? pixel * 2 + 1
                                                         : pixel + kInputWidth * kInputHeight;
                if (use_int8) {
                    // Affine int8 with positive per-tensor scale: the mask
                    // test is invariant to (q-zp)*scale.
                    mask_row[x] = qlogits[class1] > qlogits[class0] ? 255 : 0;
                    continue;
                }
                if (!std::isfinite(logits[class0]) ||
                    !std::isfinite(logits[class1])) {
                    ok = false;
                    break;
                }
                mask_row[x] = logits[class1] > logits[class0] ? 255 : 0;
            }
            if (!ok) break;
        }
    }
    rknn_outputs_release(context, output_count_, outputs.data());
    if (!ok || cv::countNonZero(new_mask) == 0) {
        std::printf("[twinlite] invalid/empty lane mask at frame=%llu\n",
                    static_cast<unsigned long long>(source_delivery_id));
        return false;
    }

    last_ms_ = ms_since(start);
    const auto postprocess_start = std::chrono::steady_clock::now();
    EgoLanePostprocessResult new_lane_result;
    cand_observe::EgoLanePostprocessResult lane_upgrade_prepared;
    double lane_upgrade_analyze_ms = 0.0;
    // [AUDIT-E8] Cache the env decision for this frame; two reads could in
    // principle disagree and feed a default-constructed observe result.
    const bool use_shared = lane_shared_enabled();
    if (use_shared) {
        const lane_shared::SharedInputs shared = lane_shared::build(new_mask, left);
        const auto shared_start = std::chrono::steady_clock::now();
        new_lane_result = analyze_twinlite_ego_lane_shared(shared);
        lane_upgrade_prepared = cand_observe::analyze_twinlite_ego_lane_shared(shared);
        lane_upgrade_analyze_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - shared_start).count();
    } else {
        new_lane_result = analyze_twinlite_ego_lane(new_mask, left);
    }
    postprocess_ms_ = ms_since(postprocess_start);
    if (use_shared)
        lane_upgrade_observe::set_from_update_result(
            lane_upgrade_prepared,
            static_cast<unsigned long long>(source_delivery_id),
            lane_upgrade_analyze_ms);
    else
        lane_upgrade_observe::set_from_update(
            new_mask, left, new_lane_result,
            static_cast<unsigned long long>(source_delivery_id));
    // [PAIR-HYSTERESIS 2026-10-22] Hold the previous pair while it still
    // matches the new candidates by geometry; accept a switch after three
    // consecutive misses. The analyzer's own "a valid pair exists" gate is
    // never bypassed.
    // [PAIR-HYSTERESIS 2026-10-22] Measured on real footage: only 1/61 frames
    // were overridden (index churn was candidate reordering, not identity
    // switching), so this is an OFF-by-default experiment, not shipped
    // behaviour. P3_PAIR_HYST=1 re-enables it for A/B work.
    static const bool pair_hysteresis_enabled = []() {
        const char* value = std::getenv("P3_PAIR_HYST");
        if (!value || !*value) return false;
        if (std::strcmp(value, "1") == 0) return true;  // rejected experiment
        std::fprintf(stderr,
                     "[pair-hyst] WARN ignored value '%s' (only \"1\" enables "
                     "the rejected experiment)\n", value);
        return false;
    }();
    constexpr int kPairMissFrames = 3;
    const int analyzer_left = new_lane_result.left_index;
    const int analyzer_right = new_lane_result.right_index;
    const bool analyzer_pair = analyzer_left >= 0 && analyzer_right >= 0;
    if (pair_hysteresis_enabled) {
        if (pair_anchor_.valid && analyzer_pair) {
            int left = -1;
            int right = -1;
            if (rematch_pair(new_lane_result.candidates, pair_anchor_,
                             &left, &right)) {
                new_lane_result.left_index = left;
                new_lane_result.right_index = right;
                new_lane_result.reason = "ego pair held by temporal match";
                pair_anchor_ = make_pair_anchor(new_lane_result);
                pair_miss_ = 0;
            } else if (++pair_miss_ >= kPairMissFrames) {
                pair_anchor_ = make_pair_anchor(new_lane_result);
                pair_miss_ = 0;
            }
        } else if (!analyzer_pair) {
            if (!pair_anchor_.valid) {
                pair_miss_ = 0;
            } else if (++pair_miss_ >= kPairMissFrames) {
                pair_anchor_ = LanePairAnchor{};
                pair_miss_ = 0;
            }
        } else {
            pair_anchor_ = make_pair_anchor(new_lane_result);
            pair_miss_ = 0;
        }
    }
    // [LANE-SMOOTH 2026-10-22] Record the raw observation before any filter
    // touches it, then apply exactly one smoother to the selected pair.
    log_lane_coefs(source_delivery_id, new_lane_result, analyzer_left,
                   analyzer_right);
    const LaneSmoothMode smooth = lane_smooth_mode();
    const auto side_valid = [&](int index) {
        return index >= 0 &&
               index < static_cast<int>(new_lane_result.candidates.size());
    };
    const auto side_coef = [&](int index) -> std::array<double, 3>& {
        return new_lane_result.candidates[static_cast<std::size_t>(index)]
            .candidate.coef;
    };
    const auto now = std::chrono::steady_clock::now();
    double oef_dt_s = 0.0;
    if (oef_has_time_)
        oef_dt_s = std::chrono::duration<double>(now - oef_last_time_).count();
    oef_last_time_ = now;
    oef_has_time_ = true;
    if (smooth == LaneSmoothMode::kOef) {
        // One Euro keeps the decision-visible geometry stable; the marking
        // classes and pair validity are untouched. Missing side -> reset, so
        // a stale line cannot survive a loss or scene change.
        if (side_valid(new_lane_result.left_index)) {
            if (oef_left_.update(side_coef(new_lane_result.left_index),
                                 oef_dt_s))
                side_coef(new_lane_result.left_index) = oef_left_.state();
        } else {
            oef_left_.reset();
        }
        if (side_valid(new_lane_result.right_index)) {
            if (oef_right_.update(side_coef(new_lane_result.right_index),
                                  oef_dt_s))
                side_coef(new_lane_result.right_index) = oef_right_.state();
        } else {
            oef_right_.reset();
        }
    } else {
        oef_left_.reset();
        oef_right_.reset();
    }
    if (smooth == LaneSmoothMode::kKf) {
        // [LANE-KF] Display-only leg: the decision result stays raw.
        // [A/B 2026-10-22] REJECTED for real data: on a 58-observation lane
        // stream the chi-square gate rejected 57/58 updates (Q/R scales did
        // not match the real coefficient magnitudes), so the line froze at
        // its first observation (right side 37.7 px off raw). Do not enable
        // "kf" until Q/R are recalibrated; "oef" is the shipped smoother.
        if (side_valid(new_lane_result.left_index))
            kf_left_.update(side_coef(new_lane_result.left_index));
        else
            kf_left_.reset();
        if (side_valid(new_lane_result.right_index))
            kf_right_.update(side_coef(new_lane_result.right_index));
        else
            kf_right_.reset();
    } else {
        kf_left_.reset();
        kf_right_.reset();
    }
    lane_result_ = std::move(new_lane_result);
    mask_ = new_mask;
    lane_pixels_ = cv::countNonZero(mask_);
    last_success_delivery_id_ = source_delivery_id;
    has_success_ = true;
    std::printf("[twinlite] frame=%llu model=%.1fms post=%.1fms total=%.1fms lane_pixels=%d candidates=%zu pair=%d/%d\n",
                static_cast<unsigned long long>(source_delivery_id), last_ms_,
                postprocess_ms_, last_ms_ + postprocess_ms_,
                lane_pixels_, lane_result_.candidates.size(),
                lane_result_.left_index, lane_result_.right_index);
    return true;
}

void TwinliteLaneModel::draw(cv::Mat& frame,
                             std::uint64_t display_delivery_id,
                             std::uint64_t max_age_frames) const
{
    if (!ready_ || !has_success_ || display_delivery_id < last_success_delivery_id_ ||
        display_delivery_id - last_success_delivery_id_ > max_age_frames ||
        frame.empty() || frame.cols < 2)
        return;

    const cv::Rect left_rect(0, 0, frame.cols / 2, frame.rows);
    cv::Mat left = frame(left_rect);
    // [LANE-KF 2026-10-22] Draw the filtered pair only; the decision copy in
    // lane_result_ is untouched.
    // [LANE-SMOOTH 2026-10-22] The display copy only diverges from the
    // decision result in the A/B "kf" mode; OEF already filtered lane_result_.
    EgoLanePostprocessResult display = lane_result_;
    if (lane_smooth_mode() == LaneSmoothMode::kKf) {
        const auto apply_filter = [&](int index, const LaneKalman& kf) {
            if (index >= 0 &&
                index < static_cast<int>(display.candidates.size()) &&
                kf.initialized())
                display.candidates[static_cast<std::size_t>(index)]
                    .candidate.coef = kf.state();
        };
        apply_filter(display.left_index, kf_left_);
        apply_filter(display.right_index, kf_right_);
    }
    draw_twinlite_ego_lane(left, display);
    lane_upgrade_observe::draw(left, display_delivery_id, max_age_frames);
}

void TwinliteLaneModel::release()
{
    if (context_) {
        rknn_destroy(reinterpret_cast<rknn_context>(context_));
        context_ = nullptr;
    }
    ready_ = false;
    output_count_ = 0;
    mask_.release();
    lane_result_ = EgoLanePostprocessResult{};
    kf_left_.reset();
    kf_right_.reset();
    oef_left_.reset();
    oef_right_.reset();
    oef_has_time_ = false;
    pair_anchor_ = LanePairAnchor{};
    pair_miss_ = 0;
    has_success_ = false;
}
