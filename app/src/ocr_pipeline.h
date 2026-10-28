#ifndef OCR_PIPELINE_H
#define OCR_PIPELINE_H

#include "hyper_lpr_sdk.h"
#include <opencv2/opencv.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct PlateOverlay {
    cv::Rect rect;
    std::string text;
    std::string type;
    float confidence = 0.0f;
};

struct OcrStatistics {
    std::uint64_t submitted = 0;
    std::uint64_t completed = 0;
    std::uint64_t dropped = 0;
    std::uint64_t max_pending = 0;
    double latest_ms = 0.0;
    double mean_ms = 0.0;
};

class OcrPipeline {
public:
    enum class Mode { Serial, Async };

    OcrPipeline(P_HLPR_Context context, Mode mode);
    ~OcrPipeline();

    OcrPipeline(const OcrPipeline&) = delete;
    OcrPipeline& operator=(const OcrPipeline&) = delete;

    bool submit(std::vector<cv::Mat> crops, std::vector<cv::Rect> rects,
                int display_y_offset);
    std::vector<PlateOverlay> overlays() const;
    OcrStatistics statistics() const;
    void clear_overlays();
    void wait_idle();
    void reset_statistics();

private:
    struct Task {
        std::vector<cv::Mat> crops;
        std::vector<cv::Rect> rects;
        int y_offset = 0;
    };

    std::vector<PlateOverlay> run_task(Task& task, double& elapsed_ms);
    void publish(std::vector<PlateOverlay> results, double elapsed_ms);
    void worker_loop();

    P_HLPR_Context context_;
    Mode mode_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::condition_variable idle_condition_;
    std::deque<Task> pending_;
    bool running_ = false;
    bool stopping_ = false;
    std::thread worker_;
    std::vector<PlateOverlay> overlays_;
    OcrStatistics stats_;
    double total_ms_ = 0.0;
};

#endif
