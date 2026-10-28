#ifndef ROAD_SPEED_ESTIMATOR_H
#define ROAD_SPEED_ESTIMATOR_H

#include <opencv2/opencv.hpp>

#include <chrono>

#include "speed_median_window.h"

class RoadSpeedEstimator {
public:
    struct Estimate {
        double kmh = 0.0;
        double flow_px = 0.0;
        int inliers = 0;
        double mean_diff = 0.0;
        bool moving = false;
        bool valid = false;
    };

    Estimate update(const cv::Mat& sbs_bgr);
    void reset();

private:
    cv::Mat previous_gray_;
    std::vector<cv::Point2f> previous_points_;
    std::chrono::steady_clock::time_point previous_time_{};
    Estimate last_estimate_{};
    double raw_smoothed_kmh_ = 0.0;
    double smoothed_kmh_ = 0.0;
    // [SPEED-MEDIAN 2026-10-22] Causal 3-sample median window (see header).
    SpeedMedianWindow speed_window_;
    double scale_m_per_px_ = 0.030;
    int moving_streak_ = 0;
    int zero_streak_ = 0;
    int frame_count_ = 0;
    bool moving_ = false;
};

#endif  // ROAD_SPEED_ESTIMATOR_H
