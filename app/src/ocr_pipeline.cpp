#include "ocr_pipeline.h"

#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <utility>

namespace {
const std::vector<std::string> kPlateTypes = {
    "蓝牌", "黄牌单层", "白牌单层", "绿牌新能源", "黑牌港澳",
    "香港单层", "香港双层", "澳门单层", "澳门双层", "黄牌双层"
};
}

OcrPipeline::OcrPipeline(P_HLPR_Context context, Mode mode)
    : context_(context), mode_(mode)
{
    if (!context_)
        throw std::invalid_argument("OCR context is null");
    if (mode_ == Mode::Async)
        worker_ = std::thread(&OcrPipeline::worker_loop, this);
}

OcrPipeline::~OcrPipeline()
{
    if (mode_ == Mode::Async) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_all();
        if (worker_.joinable())
            worker_.join();
    }
}

bool OcrPipeline::submit(std::vector<cv::Mat> crops,
                         std::vector<cv::Rect> rects, int display_y_offset)
{
    if (crops.size() != rects.size())
        throw std::invalid_argument("OCR crops/rectangles size mismatch");
    if (crops.empty())
        return false;

    Task task{std::move(crops), std::move(rects), display_y_offset};
    if (mode_ == Mode::Serial) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stats_.submitted++;
            running_ = true;
        }
        double elapsed_ms = 0.0;
        auto results = run_task(task, elapsed_ms);
        publish(std::move(results), elapsed_ms);
        return true;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Preserve every trigger so serial and asynchronous experiments execute
        // exactly the same OCR workload.  The finite experiment runner drains
        // this queue before stopping the wall/CPU timers.
        pending_.push_back(std::move(task));
        stats_.submitted++;
        if (pending_.size() > stats_.max_pending)
            stats_.max_pending = pending_.size();
    }
    condition_.notify_one();
    return true;
}

std::vector<PlateOverlay> OcrPipeline::run_task(Task& task,
                                                double& elapsed_ms)
{
    std::vector<PlateOverlay> output;
    const auto start = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < task.crops.size(); ++i) {
        HLPR_ImageData image{};
        image.data = task.crops[i].ptr<std::uint8_t>(0);
        image.width = task.crops[i].cols;
        image.height = task.crops[i].rows;
        image.format = STREAM_BGR;
        image.rotation = CAMERA_ROTATION_0;

        P_HLPR_DataBuffer buffer = HLPR_CreateDataBuffer(&image);
        if (!buffer)
            continue;
        HLPR_PlateResultList results{};
        const auto status = HLPR_ContextUpdateStream(context_, buffer, &results);
        if (status == HResultCode::Ok && results.plate_size > 0) {
            PlateOverlay overlay;
            overlay.rect = task.rects[i] + cv::Point(0, task.y_offset);
            overlay.text = results.plates[0].code;
            overlay.confidence = results.plates[0].text_confidence;
            const int type = static_cast<int>(results.plates[0].type);
            overlay.type = (type >= 0 && type < static_cast<int>(kPlateTypes.size()))
                               ? kPlateTypes[type]
                               : "UNKNOWN";
            output.push_back(overlay);
            std::printf("\n[OCR] %s [%s] confidence %.3f\n", overlay.text.c_str(),
                        overlay.type.c_str(), overlay.confidence);
        }
        HLPR_ReleaseDataBuffer(buffer);
    }
    elapsed_ms = std::chrono::duration<double, std::milli>(
                     std::chrono::steady_clock::now() - start).count();
    return output;
}

void OcrPipeline::publish(std::vector<PlateOverlay> results, double elapsed_ms)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        overlays_ = std::move(results);
        stats_.completed++;
        stats_.latest_ms = elapsed_ms;
        total_ms_ += elapsed_ms;
        stats_.mean_ms = total_ms_ / static_cast<double>(stats_.completed);
        running_ = false;
    }
    idle_condition_.notify_all();
}

void OcrPipeline::worker_loop()
{
    for (;;) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this] { return stopping_ || !pending_.empty(); });
            if (stopping_ && pending_.empty())
                return;
            task = std::move(pending_.front());
            pending_.pop_front();
            running_ = true;
        }

        double elapsed_ms = 0.0;
        auto results = run_task(task, elapsed_ms);
        publish(std::move(results), elapsed_ms);
    }
}

std::vector<PlateOverlay> OcrPipeline::overlays() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return overlays_;
}

OcrStatistics OcrPipeline::statistics() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

void OcrPipeline::clear_overlays()
{
    std::lock_guard<std::mutex> lock(mutex_);
    overlays_.clear();
}

void OcrPipeline::wait_idle()
{
    std::unique_lock<std::mutex> lock(mutex_);
    idle_condition_.wait(lock, [this] { return !running_ && pending_.empty(); });
}

void OcrPipeline::reset_statistics()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_ || !pending_.empty())
        throw std::logic_error("cannot reset OCR statistics while work is pending");
    stats_ = OcrStatistics{};
    total_ms_ = 0.0;
}
