#include "road_speed_estimator.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace {
constexpr int kWidth = 320;
constexpr int kHeight = 180;
const cv::Rect kRoadRoi(72, 108, 176, 66);

double option_double(const char* name, double fallback)
{
    const char* value = std::getenv(name);
    if (!value || !*value) return fallback;
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    return end && *end == '\0' && parsed > 0.0 ? parsed : fallback;
}

double median(std::vector<double> values)
{
    if (values.empty()) return 0.0;
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const double upper = values[middle];
    if (values.size() % 2) return upper;
    const auto lower = *std::max_element(values.begin(), values.begin() + middle);
    return (lower + upper) * 0.5;
}
}  // namespace

void RoadSpeedEstimator::reset()
{
    previous_gray_.release();
    previous_points_.clear();
    previous_time_ = {};
    last_estimate_ = {};
    raw_smoothed_kmh_ = 0.0;
    smoothed_kmh_ = 0.0;
    speed_window_.reset();
    moving_streak_ = 0;
    zero_streak_ = 0;
    moving_ = false;
    // Demo camera-on-monitor calibration. It is deliberately configurable:
    // a direct camera/viewport setup will need a different ground-plane scale.
    scale_m_per_px_ = option_double("P3_SPEED_SCALE", 0.50);
}

RoadSpeedEstimator::Estimate RoadSpeedEstimator::update(const cv::Mat& sbs_bgr)
{
    Estimate result;
    if (sbs_bgr.empty() || sbs_bgr.cols < 2 || sbs_bgr.rows < 2)
        return result;

    if (scale_m_per_px_ <= 0.0)
        scale_m_per_px_ = option_double("P3_SPEED_SCALE", 0.50);

    cv::Mat left = sbs_bgr(cv::Rect(0, 0, sbs_bgr.cols / 2, sbs_bgr.rows));
    cv::Mat small;
    cv::resize(left, small, cv::Size(kWidth, kHeight), 0, 0, cv::INTER_AREA);
    cv::Mat current_gray;
    cv::cvtColor(small(kRoadRoi), current_gray, cv::COLOR_BGR2GRAY);

    const auto now = std::chrono::steady_clock::now();
    const double now_s =
        std::chrono::duration<double>(now.time_since_epoch()).count();
    if (previous_gray_.empty() || previous_gray_.size() != current_gray.size()) {
        cv::goodFeaturesToTrack(current_gray, previous_points_, 90, 0.01, 6);
        previous_gray_ = current_gray.clone();
        previous_time_ = now;
        return result;
    }

    // 显示循环可能快于摄像头帧率：同一采集帧会被送来多次。重复帧不参与
    // 光流（全零样本对会把中值流与零速滞回拖死），保持上一估计原样返回。
    cv::Mat frame_diff;
    cv::absdiff(previous_gray_, current_gray, frame_diff);
    result.mean_diff = cv::mean(frame_diff)[0];
    if (result.mean_diff < 0.3) {
        result.kmh = last_estimate_.kmh;
        result.flow_px = last_estimate_.flow_px;
        result.inliers = last_estimate_.inliers;
        result.moving = last_estimate_.moving;
        result.valid = last_estimate_.valid;
        return result;
    }

    double dt = std::chrono::duration<double>(now - previous_time_).count();
    previous_time_ = now;
    if (dt <= 0.0 || dt > 0.75) {
        reset();
        return update(sbs_bgr);
    }
    dt = std::clamp(dt, 0.02, 0.20);

    if (previous_points_.size() < 8)
        cv::goodFeaturesToTrack(previous_gray_, previous_points_, 90, 0.01, 6);
    if (previous_points_.size() < 8) {
        previous_gray_ = current_gray.clone();
        previous_points_.clear();
        return result;
    }

    std::vector<cv::Point2f> current_points;
    std::vector<unsigned char> forward_status;
    std::vector<float> forward_error;
    cv::calcOpticalFlowPyrLK(previous_gray_, current_gray, previous_points_,
                             current_points, forward_status, forward_error,
                             cv::Size(21, 21), 3,
                             cv::TermCriteria(cv::TermCriteria::COUNT |
                                                  cv::TermCriteria::EPS,
                                              30, 0.01));

    std::vector<cv::Point2f> backward_points;
    std::vector<unsigned char> backward_status;
    std::vector<float> backward_error;
    if (!current_points.empty()) {
        cv::calcOpticalFlowPyrLK(current_gray, previous_gray_, current_points,
                                 backward_points, backward_status,
                                 backward_error, cv::Size(21, 21), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT |
                                                      cv::TermCriteria::EPS,
                                                  30, 0.01));
    }

    std::vector<cv::Point2f> previous_good;
    std::vector<cv::Point2f> current_good;
    std::vector<double> flow_magnitudes;
    for (std::size_t i = 0; i < previous_points_.size(); ++i) {
        if (i >= forward_status.size() || !forward_status[i] ||
            i >= current_points.size() || i >= backward_status.size() ||
            !backward_status[i])
            continue;
        const cv::Point2f flow = current_points[i] - previous_points_[i];
        const cv::Point2f backflow = backward_points[i] - current_points[i];
        const double magnitude = std::hypot(flow.x, flow.y);
        const double back_error = std::hypot(backflow.x, backflow.y);
        if (magnitude > 18.0 || back_error > 1.5)
            continue;
        previous_good.push_back(previous_points_[i]);
        current_good.push_back(current_points[i]);
        flow_magnitudes.push_back(magnitude);
    }

    if (previous_good.size() < 8) {
        previous_gray_ = current_gray.clone();
        previous_points_.clear();
        cv::goodFeaturesToTrack(previous_gray_, previous_points_, 90, 0.01, 6);
        return result;
    }

    cv::Mat inlier_mask;
    cv::Mat affine = cv::estimateAffinePartial2D(
        previous_good, current_good, inlier_mask, cv::RANSAC, 1.5, 2000, 0.99,
        10);

    std::vector<double> inlier_flow;
    if (!affine.empty() && !inlier_mask.empty()) {
        for (int i = 0; i < inlier_mask.rows; ++i) {
            if (inlier_mask.at<unsigned char>(i))
                inlier_flow.push_back(flow_magnitudes[i]);
        }
    }
    const std::vector<double>& robust_flow =
        inlier_flow.size() >= 8 ? inlier_flow : flow_magnitudes;
    result.inliers = static_cast<int>(robust_flow.size());
    result.flow_px = median(robust_flow);

    cv::Mat difference;
    cv::absdiff(previous_gray_, current_gray, difference);
    const double mean_difference = cv::mean(difference)[0];

    cv::Mat previous_float;
    cv::Mat current_float;
    previous_gray_.convertTo(previous_float, CV_32F);
    current_gray.convertTo(current_float, CV_32F);
    cv::Mat hanning;
    cv::createHanningWindow(hanning, current_gray.size(), CV_32F);
    double phase_response = 0.0;
    const cv::Point2d phase =
        cv::phaseCorrelate(previous_float, current_float, hanning,
                           &phase_response);
    const double phase_flow = std::hypot(phase.x, phase.y);

    double motion_flow = result.flow_px;
    if (phase_response > 0.10 && mean_difference > 1.8 &&
        phase_flow > motion_flow) {
        motion_flow = phase_flow * 0.8;
    }
    result.flow_px = motion_flow;

    const bool quiet = mean_difference < 1.5 && phase_flow < 0.12 &&
                       result.flow_px < 0.08;

    double raw_kmh =
        result.flow_px * scale_m_per_px_ * (1.0 / dt) * 3.6;
    if (quiet) {
        // A measured standstill (not an invalid sample): zero immediately so
        // the median cannot hold a moving reading across a real stop.
        speed_window_.reset();
        speed_window_.push(0.0, now_s);
        raw_kmh = 0.0;
    } else {
        speed_window_.push(raw_kmh, now_s);
    }
    // [SPEED-MEDIAN 2026-10-22] Causal median of the last (<=3) valid samples
    // replaces the single-sample EMA. Invalid frames never push and never
    // backfill zeros; a window whose newest sample is stale reports no value.
    double median_kmh = 0.0;
    raw_smoothed_kmh_ =
        speed_window_.median(now_s, &median_kmh) ? median_kmh : 0.0;

    if (raw_smoothed_kmh_ < 0.7) {
        ++zero_streak_;
        moving_streak_ = 0;
    } else if (raw_smoothed_kmh_ > 1.2) {
        ++moving_streak_;
        zero_streak_ = 0;
    } else {
        zero_streak_ = std::max(0, zero_streak_ - 1);
        moving_streak_ = std::max(0, moving_streak_ - 1);
    }

    if (!moving_ && moving_streak_ >= 2)
        moving_ = true;
    if (moving_ && zero_streak_ >= 5) {
        moving_ = false;
        smoothed_kmh_ = 0.0;
    }

    if (moving_) {
        smoothed_kmh_ =
            smoothed_kmh_ * 0.68 + raw_smoothed_kmh_ * 0.32;
        if (smoothed_kmh_ < 0.7) smoothed_kmh_ = 0.0;
    } else {
        smoothed_kmh_ = 0.0;
    }

    result.kmh = smoothed_kmh_;
    result.moving = moving_;
    result.valid = true;
    last_estimate_ = result;

    previous_gray_ = current_gray.clone();
    previous_points_ = current_good;
    if (previous_points_.size() < 30 || (frame_count_++ % 15) == 0) {
        previous_points_.clear();
        cv::goodFeaturesToTrack(previous_gray_, previous_points_, 90, 0.01, 6);
    }
    return result;
}
