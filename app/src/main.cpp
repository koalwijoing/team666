#include "../includes/yolo.h"
#include "p3_engine.h"
#include "perf_config.h"
#include "frame_prefetcher.h"
#include "../prepared_source.h"
#include "display_compose.h"
#include "disp_publish.h"   // [disp-thread] P3_DISPLAY_THREAD=1
#include "display_backend_policy.h"
#include "owned_render_frame.h"
#include "crosswalk_person.h"
#include "p3_decision_bridge.h"
#include "../main_adapter.h"
#include "experiment_metrics.h"
#include "ocr_pipeline.h"
#include "pcie_frame_source.h"
#include "road_speed_estimator.h"
#include "p3_road_safety_advisor.h"
#include "road_input_build.h"
#include "ufld_lane_model.h"
#include "twinlite_lane_model.h"
#include "decision_osd.h"  // display-only: final decision, no raw-key permission

#include <rga/RgaApi.h>
#include <rga/im2d.h>
#include <opencv2/freetype.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <algorithm>
#include <condition_variable>
#include <cstdlib>
#include <cstdint>
#include <ctime>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {
// [disp-probe] timing accumulators for the display path (diagnostic only).
double g_disp_total_ms = 0.0;
// [disp-thread] single-slot publisher; only used when P3_DISPLAY_THREAD=1.
p3_disp::Publisher g_disp_publisher;
bool g_disp_thread = false;
bool g_disp_disabled = false;   // set when no safe GUI owner exists
bool g_disp_worker_exited = true;  // only true once a worker has really returned
unsigned long g_disp_sync_completed = 0;  // completed HighGUI calls, NOT scan-out
p3_disp::HealthMonitor g_disp_health(2000);  // engineering fault threshold
std::atomic<bool> g_disp_fault_reported{false};
double g_disp_imshow_ms = 0.0;
int g_disp_frames = 0;
// Assigned on main before starting the sole compute thread, then read-only.
std::shared_ptr<p3_main_gui::DisplayChannel> g_qt_channel;
// [PERF-P3a 2026-10-22] Opt-in GUI-thread composition: the worker publishes
// the pre-compose canvas; the GUI draws OSD/lane status, prescales and adds
// the HUD. g_gui_font is written once by the compute thread before its first
// publish and read only by the GUI thread afterwards (the channel mutex
// orders the first publish after this write).
bool g_display_on_gui = false;
cv::Ptr<cv::freetype::FreeType2> g_gui_font;
std::atomic<long long> g_compute_progress_ms{0};
long long steady_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

enum class PreprocessMode { CopyCpu, MmapCpu, MmapRga };

bool env_flag(const char* name, bool fallback) {
    const char* value = std::getenv(name);
    if (!value || !*value)
        return fallback;
    return value[0] != '0' && value[0] != 'f' && value[0] != 'F';
}

// [T01] 运行标识（本地时间戳，写入 run_config.json / perf_*.csv）。
std::string make_run_id()
{
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char buf[32] = {0};
    std::strftime(buf, sizeof(buf), "run_%Y%m%d_%H%M%S", &tm);
    return std::string(buf);
}

struct Options {
    std::string device = "/dev/pango_pci_driver";
    std::uint32_t width = 1280, height = 360, frames = 0, warmup = 5;
    std::uint32_t timeout_ms = 1000, ocr_interval = 5, save_every = 0;
    int ai_mode = 0;
    bool display = true;
    bool prefetch = true;
    bool direct_display = false;
    bool fpga_decision = false;
    PreprocessMode preprocess = PreprocessMode::MmapRga;
    OcrPipeline::Mode ocr_mode = OcrPipeline::Mode::Async;
    std::string raw_csv = "experiment_frames.csv";
    std::string summary_csv = "experiment_summary.csv";
    // [T01] 新版有界性能表：独立文件、独立表头，不追加到旧格式 CSV。
    std::string perf_frames_csv = "perf_frames_v2.csv";
    std::string perf_summary_csv = "perf_summary_v2.csv";
    std::string run_config_json = "run_config.json";
    std::string save_dir = "captures";
    std::string input_dir;
    std::string manifest_csv;
    std::string accuracy_csv = "accuracy_results.csv";
    std::string accuracy_summary_csv = "accuracy_summary.csv";
    std::string accuracy_save_dir;
    bool ocr_replay = false;
    std::uint32_t replay_per_condition = 10;
    std::string replay_run_id;
    std::string replay_summary_csv = "ocr_replay_summary.csv";
    std::string pci_bdf = "0002:21:00.0";
};

struct FrameTiming {
    double capture_ms = 0, copy_ms = 0, convert_ms = 0;
    double preprocess_ms = 0, preprocess_cpu_ms = 0;
    double npu_ms = 0, frame_ms = 0;
    std::uint64_t device_ns = 0;
    std::uint32_t bytes = 0;
    // [N1-1] 逐帧模型状态（scheduled/attempted/succeeded/failed；cached=!scheduled）。
    // 供 perf_frames_v2 的模型状态列与"成功新观察 Hz"统计。
    int light_scheduled = 0, light_attempted = 0, light_succeeded = 0,
        light_failed = 0;
    int sign_scheduled = 0, sign_attempted = 0, sign_succeeded = 0,
        sign_failed = 0;
    int object_scheduled = 0, object_attempted = 0, object_succeeded = 0,
        object_failed = 0;
    int det_count = 0;
};

// [T01/N1-3] 在线运行统计（有界内存）：用于 perf_summary_v2 与旧 summary CSV。
// 模型 Hz 按"成功新观察"计数（succeeded）；失败与缓存分别统计。
// P95：帧数未超样本上限时为**精确值**（排序求 1-based rank=max(1,ceil(0.95n))）；
// 超过硬上限（kSampleLimit）后标记 overflow，退化为分桶近似并在 p95_method
// 列注明，不静默冒充精确值。
struct RunAccumulator {
    static constexpr std::size_t kSampleLimit = 200000;  // 约 5.5h@10fps，硬上限

    std::uint64_t frames = 0;
    double frame_ms_sum = 0.0, frame_ms_max = 0.0;
    double capture_sum = 0.0, copy_sum = 0.0, convert_sum = 0.0;
    double preprocess_sum = 0.0, preprocess_cpu_sum = 0.0;
    double preprocess_max = 0.0;
    std::uint64_t bytes_sum = 0, device_ns_sum = 0;
    // [N1-1/N1-2] 模型成功/失败计数（Hz 按 succeeded 计算）。
    std::uint64_t light_ok = 0, light_fail = 0;
    std::uint64_t sign_ok = 0, sign_fail = 0;
    std::uint64_t object_ok = 0, object_fail = 0;
    std::uint64_t buckets[8] = {0};
    std::uint64_t preprocess_buckets[8] = {0};
    std::vector<double> frame_samples;
    std::vector<double> preprocess_samples;
    bool sample_overflow = false;

    RunAccumulator()
    {
        frame_samples.reserve(65536);
        preprocess_samples.reserve(65536);
    }

    static void bucket_add(std::uint64_t* buckets, double value)
    {
        static const double edges[8] = {10, 20, 30, 40, 50, 75, 100, 1e18};
        for (int i = 0; i < 8; ++i) {
            if (value < edges[i]) {
                ++buckets[i];
                return;
            }
        }
    }

    // 1-based rank = max(1, ceil(0.95*n))（N1-3 要求的 rank 定义）。
    static std::size_t p95_rank(std::uint64_t n)
    {
        const std::size_t r =
            static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(n)));
        return r < 1 ? 1 : r;
    }

    static double p95_of(const std::uint64_t* buckets, std::uint64_t frames,
                         double overflow_fallback)
    {
        if (frames == 0)
            return 0.0;
        static const double edges[8] = {10, 20, 30, 40, 50, 75, 100, 1e18};
        const std::size_t target = p95_rank(frames);
        std::uint64_t cum = 0;
        for (int i = 0; i < 8; ++i) {
            cum += buckets[i];
            if (cum >= target)
                return edges[i] >= 1e17 ? overflow_fallback : edges[i];
        }
        return overflow_fallback;
    }

    // 精确 P95（对样本副本排序；样本数受 kSampleLimit 限制）。
    static double exact_p95(std::vector<double>& samples)
    {
        if (samples.empty())
            return 0.0;
        std::sort(samples.begin(), samples.end());
        std::size_t rank = p95_rank(samples.size());
        if (rank > samples.size())
            rank = samples.size();
        return samples[rank - 1];
    }

    void add(const FrameTiming& t)
    {
        ++frames;
        frame_ms_sum += t.frame_ms;
        if (t.frame_ms > frame_ms_max)
            frame_ms_max = t.frame_ms;
        capture_sum += t.capture_ms;
        copy_sum += t.copy_ms;
        convert_sum += t.convert_ms;
        preprocess_sum += t.preprocess_ms;
        if (t.preprocess_ms > preprocess_max)
            preprocess_max = t.preprocess_ms;
        preprocess_cpu_sum += t.preprocess_cpu_ms;
        bytes_sum += t.bytes;
        device_ns_sum += t.device_ns;
        light_ok += t.light_succeeded ? 1 : 0;
        light_fail += t.light_failed ? 1 : 0;
        sign_ok += t.sign_succeeded ? 1 : 0;
        sign_fail += t.sign_failed ? 1 : 0;
        object_ok += t.object_succeeded ? 1 : 0;
        object_fail += t.object_failed ? 1 : 0;
        bucket_add(buckets, t.frame_ms);
        bucket_add(preprocess_buckets, t.preprocess_ms);
        if (frame_samples.size() < kSampleLimit) {
            frame_samples.push_back(t.frame_ms);
            preprocess_samples.push_back(t.preprocess_ms);
        } else {
            sample_overflow = true;
        }
    }

    double mean(double sum) const { return frames ? sum / frames : 0.0; }
    const char* p95_method() const
    {
        return sample_overflow ? "histogram_approx_overflow" : "precise";
    }
};

// Import the two coherent mmap slots into the RGA driver once, then reuse the
// returned handles for every frame.  This keeps buffer registration and page
// table construction out of the measured per-frame conversion interval.
class RgaPreprocessor {
public:
    RgaPreprocessor(PcieFrameSource& source, cv::Mat& destination)
        : width_(source.width()), height_(source.height()), destination_(destination)
    {
        if (!destination_.isContinuous())
            throw std::runtime_error("RGA destination must be contiguous");
        try {
            for (std::uint32_t i = 0; i < source.buffer_count(); ++i) {
                const auto handle = importbuffer_virtualaddr(source.buffer(i),
                                                              source.frame_bytes());
                if (!handle)
                    throw std::runtime_error("RGA failed to import coherent mmap slot " +
                                             std::to_string(i));
                source_handles_.push_back(handle);
                source_buffers_.push_back(wrapbuffer_handle(handle, width_, height_,
                                                             RK_FORMAT_RGB_565));
            }

            destination_handle_ = importbuffer_virtualaddr(
                destination_.data, destination_.total() * destination_.elemSize());
            if (!destination_handle_)
                throw std::runtime_error("RGA failed to import RGB888 destination");
            destination_buffer_ = wrapbuffer_handle(destination_handle_, width_, height_,
                                                     RK_FORMAT_RGB_888);
        } catch (...) {
            release();
            throw;
        }
    }

    ~RgaPreprocessor() { release(); }

    RgaPreprocessor(const RgaPreprocessor&) = delete;
    RgaPreprocessor& operator=(const RgaPreprocessor&) = delete;

    void convert(std::uint32_t buffer_index)
    {
        if (buffer_index >= source_buffers_.size())
            throw std::out_of_range("RGA source buffer index");
        const IM_STATUS status = imcvtcolor(source_buffers_[buffer_index], destination_buffer_,
                                            RK_FORMAT_RGB_565, RK_FORMAT_RGB_888);
        if (status != IM_STATUS_SUCCESS)
            throw std::runtime_error(std::string("RGA conversion failed: ") +
                                     imStrError(status));
    }

private:
    void release() noexcept
    {
        if (destination_handle_) {
            releasebuffer_handle(destination_handle_);
            destination_handle_ = 0;
        }
        for (auto handle : source_handles_)
            if (handle) releasebuffer_handle(handle);
        source_handles_.clear();
        source_buffers_.clear();
    }

    int width_ = 0;
    int height_ = 0;
    cv::Mat& destination_;
    std::vector<rga_buffer_handle_t> source_handles_;
    std::vector<rga_buffer_t> source_buffers_;
    rga_buffer_handle_t destination_handle_ = 0;
    rga_buffer_t destination_buffer_{};
};

using FramePrefetcher = FramePrefetcherT<PcieFrameSource>;  // [T02-C] 见 frame_prefetcher.h
const char* preprocess_name(PreprocessMode mode)
{
    if (mode == PreprocessMode::CopyCpu) return "A_copy_cpu";
    if (mode == PreprocessMode::MmapCpu) return "B_mmap_cpu";
    return "C_mmap_rga";
}

const char* ocr_name(OcrPipeline::Mode mode)
{
    return mode == OcrPipeline::Mode::Serial ? "serial" : "async";
}

const char* ai_mode_name(int mode)
{
    if (mode == 1) return "PLATE OCR";
    if (mode == 2) return "ZEBRA+PERSON+LIGHT";
    if (mode == 3) return p3_config().enable_sign ? "LIGHT+SIGN+OBJECT" : "LIGHT+OBJECT";
    return "OFF";
}

const char* lane_state_cn(int state)
{
    if (state == LANE_DASHED) return "虚线";
    if (state == LANE_SOLID) return "实线";
    if (state == LANE_INVALID) return "无效";
    return "未知";
}

P3RoadMarking road_marking_from_twinlite(LanePaintType type);

const char* ufld_state_cn(UfldLaneState state)
{
    if (state == UfldLaneState::Dashed) return "虚线";
    if (state == UfldLaneState::Solid) return "实线";
    return "无";
}

void draw_lane_status(cv::Mat& canvas, const P3DecisionOutput& decision,
                      const cv::Ptr<cv::freetype::FreeType2>& font,
                      const RoadSpeedEstimator::Estimate& speed,
                      const UfldLaneModel* ufld)
{
    if (!font) return;
    const std::string line1 =
        std::string("车道 左：") + lane_state_cn(decision.left_lane_state) +
        "  |  右：" + lane_state_cn(decision.right_lane_state);
    char line2_buf[128];
    std::snprintf(line2_buf, sizeof(line2_buf),
                  "车道模型 %s | 数据龄 %.0f 毫秒",
                  decision.lane_fresh ? "有效" : "不可用/过期",
                  decision.lane_age_ms);
    font->putText(canvas, line1, cv::Point(24, 72), 30,
                  cv::Scalar(255, 255, 255), -1, cv::LINE_8, true);
    font->putText(canvas, line2_buf, cv::Point(24, 126), 26,
                  cv::Scalar(180, 220, 255), -1, cv::LINE_8, true);
    if (ufld && ufld->ready()) {
        std::string line3 =
            std::string("模型车道：左 ") + ufld_state_cn(ufld->left().state) +
            "  |  右 " + ufld_state_cn(ufld->right().state);
        font->putText(canvas, line3, cv::Point(24, 170), 26,
                      cv::Scalar(255, 160, 255), -1, cv::LINE_8, true);
    }
}

void draw_lane_status_compact(cv::Mat& frame, const P3DecisionOutput& decision,
                              const cv::Ptr<cv::freetype::FreeType2>& font,
                              const RoadSpeedEstimator::Estimate& speed)
{
    if (!font) return;
    const std::string line =
        std::string("Twin 左：") + lane_state_cn(decision.left_lane_state) +
        "  Twin 右：" + lane_state_cn(decision.right_lane_state) +
        (decision.lane_fresh ? "  有效  " : "  不可用/过期  ");
    font->putText(frame, line,
                  cv::Point(10, std::max(24, frame.rows - 12)), 18,
                  cv::Scalar(255, 255, 255), -1, cv::LINE_8, true);
}

std::uint32_t parse_u32(const char* option, const char* value)
{
    try {
        std::size_t used = 0;
        const unsigned long parsed = std::stoul(value, &used, 10);
        if (used != std::strlen(value) || parsed > UINT32_MAX)
            throw std::out_of_range("range");
        return static_cast<std::uint32_t>(parsed);
    } catch (...) {
        throw std::invalid_argument(std::string("invalid value for ") + option + ": " + value);
    }
}

void print_help(const char* program)
{
    std::cout << "Usage: " << program << " [options]\n"
              << "  --preprocess copy-cpu|mmap-cpu|mmap-rga\n"
              << "  --ocr serial|async       identical OCR trigger policy\n"
              << "  --ai-mode 0|1|2|3        passthrough, plate, main model, light+sign\n"
              << "  --width N --height N --frames N --warmup N\n"
              << "  --ocr-interval N --timeout-ms N --headless --no-prefetch\n"
              << "  --direct-display         show 1280x360 directly for ai-mode 3\n"
              << "  --fpga-decision          write decision LEDs/alarm to FPGA OSD\n"
              << "  --pci-bdf BDF            PCIe BDF (default 0002:21:00.0)\n"
              << "  --raw-csv FILE --summary-csv FILE\n"
              << "  --save-every N --save-dir DIR --device FILE\n"
              << "\nAccuracy dataset mode (bypasses PCIe capture):\n"
              << "  --input-dir DIR --manifest FILE\n"
              << "  --accuracy-csv FILE --accuracy-summary FILE\n"
              << "  --accuracy-save-dir DIR   save annotated result images\n"
              << "\nDeterministic OCR replay (also bypasses PCIe capture):\n"
              << "  --ocr-replay --input-dir DIR --manifest FILE\n"
              << "  --ocr serial|async --frames N --warmup N --ocr-interval N\n"
              << "  --replay-per-condition N --raw-csv FILE\n"
              << "  --replay-run-id ID --replay-summary FILE\n";
}

Options parse_options(int argc, char** argv)
{
    Options out;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&]() -> const char* {
            if (++i >= argc) throw std::invalid_argument("missing value after " + arg);
            return argv[i];
        };
        if (arg == "--preprocess") {
            const std::string mode = value();
            if (mode == "copy-cpu") out.preprocess = PreprocessMode::CopyCpu;
            else if (mode == "mmap-cpu") out.preprocess = PreprocessMode::MmapCpu;
            else if (mode == "mmap-rga") out.preprocess = PreprocessMode::MmapRga;
            else throw std::invalid_argument("unknown preprocess mode: " + mode);
        } else if (arg == "--ocr") {
            const std::string mode = value();
            if (mode == "serial") out.ocr_mode = OcrPipeline::Mode::Serial;
            else if (mode == "async") out.ocr_mode = OcrPipeline::Mode::Async;
            else throw std::invalid_argument("unknown OCR mode: " + mode);
        } else if (arg == "--ai-mode") out.ai_mode = static_cast<int>(parse_u32("--ai-mode", value()));
        else if (arg == "--width") out.width = parse_u32("--width", value());
        else if (arg == "--height") out.height = parse_u32("--height", value());
        else if (arg == "--frames") out.frames = parse_u32("--frames", value());
        else if (arg == "--warmup") out.warmup = parse_u32("--warmup", value());
        else if (arg == "--timeout-ms") out.timeout_ms = parse_u32("--timeout-ms", value());
        else if (arg == "--ocr-interval") out.ocr_interval = parse_u32("--ocr-interval", value());
        else if (arg == "--save-every") out.save_every = parse_u32("--save-every", value());
        else if (arg == "--device") out.device = value();
        else if (arg == "--raw-csv") out.raw_csv = value();
        else if (arg == "--summary-csv") out.summary_csv = value();
        // [T01] 新版性能表路径（独立文件）
        else if (arg == "--perf-frames") out.perf_frames_csv = value();
        else if (arg == "--perf-summary") out.perf_summary_csv = value();
        else if (arg == "--run-config-json") out.run_config_json = value();
        else if (arg == "--save-dir") out.save_dir = value();
        else if (arg == "--pci-bdf") out.pci_bdf = value();
        else if (arg == "--input-dir") out.input_dir = value();
        else if (arg == "--manifest") out.manifest_csv = value();
        else if (arg == "--accuracy-csv") out.accuracy_csv = value();
        else if (arg == "--accuracy-summary") out.accuracy_summary_csv = value();
        else if (arg == "--accuracy-save-dir") out.accuracy_save_dir = value();
        else if (arg == "--ocr-replay") out.ocr_replay = true;
        else if (arg == "--replay-per-condition")
            out.replay_per_condition = parse_u32("--replay-per-condition", value());
        else if (arg == "--replay-run-id") out.replay_run_id = value();
        else if (arg == "--replay-summary") out.replay_summary_csv = value();
        else if (arg == "--headless") out.display = false;
        else if (arg == "--no-prefetch") out.prefetch = false;
        else if (arg == "--direct-display") out.direct_display = true;
        else if (arg == "--fpga-decision") out.fpga_decision = true;
        else if (arg == "--help" || arg == "-h") { print_help(argv[0]); std::exit(0); }
        else throw std::invalid_argument("unknown option: " + arg);
    }
    if (out.ai_mode < 0 || out.ai_mode > 3)
        throw std::invalid_argument("--ai-mode must be 0, 1, 2, or 3");
    if (!out.width || !out.height || !out.ocr_interval)
        throw std::invalid_argument("dimensions and OCR interval must be positive");
    if (out.input_dir.empty() != out.manifest_csv.empty())
        throw std::invalid_argument("--input-dir and --manifest must be used together");
    if (out.ocr_replay && (out.input_dir.empty() || !out.frames || !out.replay_per_condition))
        throw std::invalid_argument(
            "--ocr-replay requires --input-dir, --manifest, positive --frames and "
            "--replay-per-condition");
    // [disp-thread] one bounded experiment: do the imshow on a worker thread.
    {
        const char* v = std::getenv("P3_DISPLAY_THREAD");
        g_disp_thread = v && v[0] != 0 && v[0] != '0' && v[0] != 'f' &&
                        v[0] != 'F';
    }
    if (!out.display && !out.frames && out.input_dir.empty())
        throw std::invalid_argument("--headless requires a finite --frames value");
    return out;
}

bool file_is_empty(const std::string& path)
{
    std::error_code error;
    return !std::filesystem::exists(path, error) || std::filesystem::file_size(path, error) == 0;
}

// [N1-fix] CSV 表头校验：空文件 → 需要写表头（返回 true）；非空 → 首行必须
// 与期望 schema 完全一致，否则明确报错——禁止旧 schema 文件被追加新格式行
// （防"同名文件列错位"被误读为有效数据）。expected 不含换行符。
bool csv_header_pending(const std::string& path, const char* expected)
{
    if (file_is_empty(path))
        return true;
    std::ifstream in(path);
    std::string first;
    std::getline(in, first);
    if (first.size() >= 3 &&
        static_cast<unsigned char>(first[0]) == 0xEF &&
        static_cast<unsigned char>(first[1]) == 0xBB &&
        static_cast<unsigned char>(first[2]) == 0xBF)
        first.erase(0, 3);  // 去 UTF-8 BOM
    while (!first.empty() && (first.back() == '\r' || first.back() == '\n'))
        first.pop_back();
    if (first != expected) {
        throw std::runtime_error(
            "CSV schema mismatch for " + path +
            ": existing header was not produced by the current schema; "
            "remove or rename the old file before appending new rows");
    }
    return false;
}

void ensure_parent(const std::string& path)
{
    const auto parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent);
}

P_HLPR_Context create_lpr_context()
{
    static char model_path[] = "./hyperlpr3/resource/models/r2_mobile";
    HLPR_ContextConfiguration cfg{};
    cfg.models_path = model_path;
    cfg.max_num = 5;
    cfg.det_level = DETECT_LEVEL_LOW;
    cfg.use_half = false;
    cfg.nms_threshold = 0.5f;
    cfg.rec_confidence_threshold = 0.5f;
    cfg.box_conf_threshold = 0.30f;
    cfg.threads = 2;
    P_HLPR_Context context = HLPR_CreateContext(&cfg);
    if (!context || HLPR_ContextQueryStatus(context) != HResultCode::Ok)
        throw std::runtime_error("HyperLPR context creation failed");
    return context;
}

double preprocess_frame(PreprocessMode mode, const std::uint8_t* mapped,
                        std::vector<std::uint8_t>& copy_buffer,
                        std::uint32_t width, std::uint32_t height, cv::Mat& color,
                        RgaPreprocessor* rga, std::uint32_t buffer_index,
                        double& copy_ms, double& convert_ms)
{
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    const std::uint8_t* input = mapped;
    copy_ms = 0;
    if (mode == PreprocessMode::CopyCpu) {
        const auto copy_start = Clock::now();
        std::memcpy(copy_buffer.data(), mapped, copy_buffer.size());
        copy_ms = std::chrono::duration<double, std::milli>(Clock::now() - copy_start).count();
        input = copy_buffer.data();
    }
    const auto convert_start = Clock::now();
    if (mode == PreprocessMode::MmapRga) {
        if (!rga) throw std::logic_error("RGA preprocessor is not initialized");
        rga->convert(buffer_index);
    } else {
        // [N3 修复] CPU 路径对齐 RGA 契约：板端 RGA 实测确认
        // RK_FORMAT_RGB_888 的内存字节序为 BGR（纯红 0xF800 -> bytes[0]=0,
        // [2]=248），engine 亦按 BGR 输入处理；原 BGR5652RGB 会使 CPU 路径
        // 红蓝互换。改为输出 BGR（像素断言见 tests/test_color_contract.cpp）。
        cv::Mat packed(static_cast<int>(height), static_cast<int>(width), CV_8UC2,
                       const_cast<std::uint8_t*>(input));
        cv::cvtColor(packed, color, cv::COLOR_BGR5652BGR);
    }
    convert_ms = std::chrono::duration<double, std::milli>(Clock::now() - convert_start).count();
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

struct PreparedPixels {
    cv::Mat color;
    std::vector<std::uint8_t> copy;
    std::unique_ptr<RgaPreprocessor> rga;
    PreparedPixels(PcieFrameSource& source, PreprocessMode mode)
        : color(source.height(), source.width(), CV_8UC3) {
        if (mode == PreprocessMode::CopyCpu) copy.resize(source.frame_bytes());
        if (mode == PreprocessMode::MmapRga)
            rga = std::make_unique<RgaPreprocessor>(source, color);
    }
};
using PreparedSource = p3_prep::SourceAdapter<PcieFrameSource, PreparedPixels>;
using PreparedPrefetcher = FramePrefetcherT<PreparedSource>;

void make_plate_crops(const cv::Mat& frame, const std::vector<cv::Rect>& detections,
                      std::vector<cv::Mat>& crops, std::vector<cv::Rect>& rectangles)
{
    const cv::Rect bounds(0, 0, frame.cols, frame.rows);
    for (const auto& plate : detections) {
        constexpr float expand = 0.20f;
        const cv::Rect expanded(static_cast<int>(plate.x - plate.width * expand),
                                static_cast<int>(plate.y - plate.height * expand),
                                static_cast<int>(plate.width * (1 + 2 * expand)),
                                static_cast<int>(plate.height * (1 + 2 * expand)));
        const cv::Rect safe = expanded & bounds;
        if (safe.width < 15 || safe.height < 10) continue;
        crops.push_back(frame(safe).clone());
        rectangles.push_back(plate);
    }
}

struct AccuracySample {
    std::string sample_id;
    std::string condition;
    std::string ground_truth;
    std::string relative_path;
    cv::Rect ground_truth_rect;
};

struct AccuracyCount {
    std::uint64_t samples = 0;
    std::uint64_t detected_iou50 = 0;
    std::uint64_t detected_iou70 = 0;
    std::uint64_t full_correct_iou50 = 0;
    std::uint64_t full_correct_iou70 = 0;
};

std::vector<std::string> parse_csv_row(const std::string& line)
{
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char value = line[i];
        if (quoted) {
            if (value == '"' && i + 1 < line.size() && line[i + 1] == '"') {
                field.push_back('"');
                ++i;
            } else if (value == '"') {
                quoted = false;
            } else {
                field.push_back(value);
            }
        } else if (value == ',') {
            fields.push_back(std::move(field));
            field.clear();
        } else if (value == '"') {
            quoted = true;
        } else if (value != '\r') {
            field.push_back(value);
        }
    }
    if (quoted) throw std::runtime_error("unterminated quoted CSV field");
    fields.push_back(std::move(field));
    return fields;
}

std::string csv_escape(const std::string& value)
{
    if (value.find_first_of(",\"\r\n") == std::string::npos) return value;
    std::string escaped = "\"";
    for (const char character : value) {
        if (character == '"') escaped += "\"\"";
        else escaped.push_back(character);
    }
    escaped.push_back('"');
    return escaped;
}

std::vector<AccuracySample> load_accuracy_manifest(const std::string& path)
{
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open accuracy manifest: " + path);
    std::string line;
    if (!std::getline(file, line)) throw std::runtime_error("empty accuracy manifest: " + path);
    if (line.size() >= 3 && static_cast<unsigned char>(line[0]) == 0xef &&
        static_cast<unsigned char>(line[1]) == 0xbb &&
        static_cast<unsigned char>(line[2]) == 0xbf)
        line.erase(0, 3);
    const auto header = parse_csv_row(line);
    std::unordered_map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < header.size(); ++i) columns[header[i]] = i;
    for (const char* required : {"sample_id", "condition", "ground_truth_plate",
                                 "relative_path", "bbox_x1", "bbox_y1", "bbox_x2",
                                 "bbox_y2"})
        if (!columns.count(required))
            throw std::runtime_error(std::string("manifest missing column: ") + required);

    std::vector<AccuracySample> samples;
    std::size_t row_number = 1;
    while (std::getline(file, line)) {
        ++row_number;
        if (line.empty()) continue;
        const auto fields = parse_csv_row(line);
        auto get = [&](const char* name) -> std::string {
            const std::size_t index = columns.at(name);
            if (index >= fields.size())
                throw std::runtime_error("short manifest row " + std::to_string(row_number));
            return fields[index];
        };
        const int x1 = std::stoi(get("bbox_x1"));
        const int y1 = std::stoi(get("bbox_y1"));
        const int x2 = std::stoi(get("bbox_x2"));
        const int y2 = std::stoi(get("bbox_y2"));
        AccuracySample sample{get("sample_id"), get("condition"),
                              get("ground_truth_plate"), get("relative_path"),
                              cv::Rect(cv::Point(std::min(x1, x2), std::min(y1, y2)),
                                       cv::Point(std::max(x1, x2) + 1, std::max(y1, y2) + 1))};
        if (sample.sample_id.empty() || sample.ground_truth.empty() ||
            sample.relative_path.empty())
            throw std::runtime_error("empty required value in manifest row " +
                                     std::to_string(row_number));
        samples.push_back(std::move(sample));
    }
    if (samples.empty()) throw std::runtime_error("accuracy manifest has no samples");
    return samples;
}

double intersection_over_union(const cv::Rect& left, const cv::Rect& right)
{
    const cv::Rect intersection = left & right;
    const double intersection_area = std::max(0, intersection.area());
    const double union_area = left.area() + right.area() - intersection_area;
    return union_area > 0.0 ? intersection_area / union_area : 0.0;
}

struct ReplayImage {
    std::string sample_id;
    std::string condition;
    std::string relative_path;
    cv::Mat image;
};

int run_ocr_replay(const Options& o)
{
    const auto manifest = load_accuracy_manifest(o.manifest_csv);
    const std::filesystem::path input_root(o.input_dir);
    if (!std::filesystem::is_directory(input_root))
        throw std::runtime_error("replay input directory not found: " + o.input_dir);

    std::map<std::string, std::uint32_t> selected_per_condition;
    std::vector<ReplayImage> images;
    for (const auto& sample : manifest) {
        auto& selected = selected_per_condition[sample.condition];
        if (selected >= o.replay_per_condition) continue;
        const auto image_path = input_root / std::filesystem::path(sample.relative_path);
        cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
        if (image.empty())
            throw std::runtime_error("cannot preload replay image: " + image_path.string());
        images.push_back({sample.sample_id, sample.condition, sample.relative_path,
                          std::move(image)});
        ++selected;
    }
    if (selected_per_condition.size() != 4)
        throw std::runtime_error("OCR replay requires exactly four input conditions");
    for (const auto& [condition, count] : selected_per_condition)
        if (count != o.replay_per_condition)
            throw std::runtime_error("not enough replay images for condition: " + condition);
    if (images.empty()) throw std::runtime_error("OCR replay selected no images");

    std::cout << "[OCR_REPLAY] preloaded=" << images.size()
              << " images, mode=" << ocr_name(o.ocr_mode)
              << ", frames=" << o.frames << ", warmup=" << o.warmup
              << ", interval=" << o.ocr_interval << '\n';
    for (const auto& [condition, count] : selected_per_condition)
        std::cout << "[OCR_REPLAY] condition=" << condition << " images=" << count << '\n';

    yolo_init();
    P_HLPR_Context lpr = create_lpr_context();
    OcrPipeline ocr(lpr, o.ocr_mode);

    auto process_one = [&](std::uint32_t index, bool measure,
                           std::ofstream* raw, std::vector<double>* frame_times) {
        ReplayImage& source = images[index % images.size()];
        const auto frame_start = std::chrono::steady_clock::now();
        const auto detection_start = std::chrono::steady_clock::now();
        std::vector<cv::Rect> detections;
        yolo_run_plate(source.image, detections);
        const double detection_ms = std::chrono::duration<double, std::milli>(
                                        std::chrono::steady_clock::now() - detection_start)
                                        .count();
        bool submitted = false;
        double submit_ms = 0.0;
        if ((index + 1) % o.ocr_interval == 0 && !detections.empty()) {
            std::vector<cv::Mat> crops;
            std::vector<cv::Rect> rectangles;
            make_plate_crops(source.image, detections, crops, rectangles);
            const auto submit_start = std::chrono::steady_clock::now();
            submitted = ocr.submit(std::move(crops), std::move(rectangles), 0);
            submit_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - submit_start)
                            .count();
        }
        const double frame_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - frame_start)
                                    .count();
        if (measure) {
            frame_times->push_back(frame_ms);
            *raw << index << ',' << csv_escape(source.sample_id) << ','
                 << csv_escape(source.condition) << ',' << csv_escape(source.relative_path)
                 << ',' << detections.size() << ',' << (submitted ? 1 : 0) << ','
                 << std::fixed << std::setprecision(6) << detection_ms << ',' << submit_ms
                 << ',' << frame_ms << '\n';
        }
    };

    // Warm the NPU and OCR models using the same scheduling mode, then clear all
    // counters so setup work cannot enter the formal measurements.
    for (std::uint32_t index = 0; index < o.warmup; ++index)
        process_one(index, false, nullptr, nullptr);
    ocr.wait_idle();
    ocr.reset_statistics();
    ocr.clear_overlays();

    ensure_parent(o.raw_csv);
    std::ofstream raw(o.raw_csv, std::ios::trunc);
    if (!raw) throw std::runtime_error("cannot open replay raw CSV: " + o.raw_csv);
    raw << "frame_index,sample_id,condition,relative_path,detection_count,ocr_submitted,"
           "detection_ms,submit_ms,main_frame_ms\n";

    std::vector<double> frame_times;
    frame_times.reserve(std::min<std::size_t>(o.frames, 257));
    ProcessCpuMeter total_cpu_meter;
    ProcessCpuMeter producer_cpu_meter;
    total_cpu_meter.start();
    producer_cpu_meter.start();
    const auto total_start = std::chrono::steady_clock::now();
    const auto producer_start = total_start;
    for (std::uint32_t index = 0; index < o.frames; ++index)
        process_one(index, true, &raw, &frame_times);
    const double producer_seconds = std::chrono::duration<double>(
                                        std::chrono::steady_clock::now() - producer_start)
                                        .count();
    const CpuUsage producer_cpu = producer_cpu_meter.stop();
    const auto drain_start = std::chrono::steady_clock::now();
    ocr.wait_idle();
    const double drain_seconds = std::chrono::duration<double>(
                                     std::chrono::steady_clock::now() - drain_start)
                                     .count();
    const double total_seconds = std::chrono::duration<double>(
                                     std::chrono::steady_clock::now() - total_start)
                                     .count();
    const CpuUsage total_cpu = total_cpu_meter.stop();
    const OcrStatistics stats = ocr.statistics();
    raw.close();

    ensure_parent(o.replay_summary_csv);
    const bool header = file_is_empty(o.replay_summary_csv);
    std::ofstream summary(o.replay_summary_csv, std::ios::app);
    if (!summary)
        throw std::runtime_error("cannot open replay summary CSV: " + o.replay_summary_csv);
    if (header)
        summary << "run_id,ocr_mode,source_images,frames,warmup,ocr_interval,producer_seconds,"
                   "drain_seconds,total_seconds,producer_fps,end_to_end_fps,mean_main_frame_ms,"
                   "p95_main_frame_ms,producer_cpu_process_pct,producer_cpu_machine_pct,"
                   "total_cpu_process_pct,total_cpu_machine_pct,ocr_submitted,ocr_completed,"
                   "ocr_dropped,ocr_max_pending,mean_ocr_ms\n";
    summary << csv_escape(o.replay_run_id) << ',' << ocr_name(o.ocr_mode) << ','
            << images.size() << ',' << o.frames << ',' << o.warmup << ','
            << o.ocr_interval << ',' << std::fixed << std::setprecision(6)
            << producer_seconds << ',' << drain_seconds << ',' << total_seconds << ','
            << o.frames / producer_seconds << ',' << o.frames / total_seconds << ','
            << mean(frame_times) << ',' << percentile_ms(frame_times, .95) << ','
            << producer_cpu.process_percent << ',' << producer_cpu.machine_percent << ','
            << total_cpu.process_percent << ',' << total_cpu.machine_percent << ','
            << stats.submitted << ',' << stats.completed << ',' << stats.dropped << ','
            << stats.max_pending << ',' << stats.mean_ms << '\n';
    summary.close();

    std::cout << std::fixed << std::setprecision(3)
              << "[OCR_REPLAY_RESULT] mode=" << ocr_name(o.ocr_mode)
              << " producer_fps=" << o.frames / producer_seconds
              << " end_to_end_fps=" << o.frames / total_seconds
              << " drain=" << drain_seconds << " s"
              << " CPU=" << total_cpu.process_percent << "%"
              << " submitted=" << stats.submitted
              << " completed=" << stats.completed
              << " dropped=" << stats.dropped
              << " max_pending=" << stats.max_pending
              << " mean_ocr=" << stats.mean_ms << " ms\n";

    ocr.clear_overlays();
    HLPR_ReleaseContext(lpr);
    return 0;
}

int run_accuracy_dataset(const Options& o)
{
    const auto samples = load_accuracy_manifest(o.manifest_csv);
    const std::filesystem::path input_root(o.input_dir);
    if (!std::filesystem::is_directory(input_root))
        throw std::runtime_error("accuracy input directory not found: " + o.input_dir);
    ensure_parent(o.accuracy_csv);
    ensure_parent(o.accuracy_summary_csv);
    std::ofstream result(o.accuracy_csv, std::ios::trunc);
    if (!result) throw std::runtime_error("cannot open accuracy CSV: " + o.accuracy_csv);
    result << "\xef\xbb\xbf"
              "sample_id,condition,ground_truth_plate,detected_iou50,detected_iou70,"
              "predicted_plate_iou50,full_plate_correct_iou50,full_plate_correct_iou70,"
              "plate_type,confidence,detection_count,detection_ms,"
              "max_iou,ocr_ms,total_ms,relative_path,error\n";
    result << std::fixed << std::setprecision(6);

    if (!o.accuracy_save_dir.empty())
        std::filesystem::create_directories(o.accuracy_save_dir);
    cv::Ptr<cv::freetype::FreeType2> font;
    if (!o.accuracy_save_dir.empty()) {
        font = cv::freetype::createFreeType2();
        font->loadFontData("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc", 0);
    }

    yolo_init();
    P_HLPR_Context lpr = create_lpr_context();
    OcrPipeline ocr(lpr, OcrPipeline::Mode::Serial);
    std::map<std::string, AccuracyCount> counts;

    std::size_t processed = 0;
    for (const auto& sample : samples) {
        const auto total_start = std::chrono::steady_clock::now();
        bool detected_iou50 = false;
        bool detected_iou70 = false;
        bool full_correct_iou50 = false;
        bool full_correct_iou70 = false;
        std::string predicted;
        std::string plate_type;
        std::string error;
        float confidence = 0.0f;
        double detection_ms = 0.0;
        double ocr_ms = 0.0;
        double max_iou = 0.0;
        std::vector<cv::Rect> detections;
        cv::Mat image;
        try {
            const auto image_path = input_root / std::filesystem::path(sample.relative_path);
            image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
            if (image.empty()) throw std::runtime_error("cannot read image: " + image_path.string());

            const auto detection_start = std::chrono::steady_clock::now();
            yolo_run_plate(image, detections);
            detection_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - detection_start).count();
            for (const auto& rectangle : detections)
                max_iou = std::max(max_iou,
                                   intersection_over_union(rectangle, sample.ground_truth_rect));
            detected_iou50 = max_iou >= 0.50;
            detected_iou70 = max_iou > 0.70;
            ocr.clear_overlays();
            if (!detections.empty()) {
                std::vector<cv::Mat> crops;
                std::vector<cv::Rect> rectangles;
                make_plate_crops(image, detections, crops, rectangles);
                const auto ocr_start = std::chrono::steady_clock::now();
                ocr.submit(std::move(crops), std::move(rectangles), 0);
                ocr_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - ocr_start).count();
                const auto overlays = ocr.overlays();
                if (!overlays.empty()) {
                    const auto best_any = std::max_element(
                        overlays.begin(), overlays.end(),
                        [](const PlateOverlay& left, const PlateOverlay& right) {
                            return left.confidence < right.confidence;
                        });
                    const PlateOverlay* best_iou50 = nullptr;
                    for (const auto& overlay : overlays) {
                        const double iou = intersection_over_union(
                            overlay.rect, sample.ground_truth_rect);
                        if (iou >= 0.50 &&
                            (!best_iou50 || overlay.confidence > best_iou50->confidence))
                            best_iou50 = &overlay;
                        if (iou >= 0.50 && overlay.text == sample.ground_truth)
                            full_correct_iou50 = true;
                        if (iou > 0.70 && overlay.text == sample.ground_truth)
                            full_correct_iou70 = true;
                    }
                    const PlateOverlay& selected = best_iou50 ? *best_iou50 : *best_any;
                    predicted = selected.text;
                    plate_type = selected.type;
                    confidence = selected.confidence;
                }
            }

            if (!o.accuracy_save_dir.empty()) {
                cv::rectangle(image, sample.ground_truth_rect, cv::Scalar(255, 0, 0), 2);
                for (const auto& rectangle : detections) {
                    const bool valid = intersection_over_union(
                                           rectangle, sample.ground_truth_rect) > 0.70;
                    cv::rectangle(image, rectangle,
                                  valid ? cv::Scalar(0, 255, 0)
                                        : (intersection_over_union(rectangle, sample.ground_truth_rect) >= 0.50
                                               ? cv::Scalar(0, 255, 255)
                                               : cv::Scalar(0, 0, 255)), 3);
                }
                const std::string label = sample.sample_id + "  GT:" + sample.ground_truth +
                                          "  PRED:" + (predicted.empty() ? "NONE" : predicted);
                font->putText(image, label, cv::Point(10, 35), 25,
                              full_correct_iou50 ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
                              -1, 8, true);
                const auto output_path = std::filesystem::path(o.accuracy_save_dir) /
                                         std::filesystem::path(sample.relative_path);
                std::filesystem::create_directories(output_path.parent_path());
                if (!cv::imwrite(output_path.string(), image))
                    throw std::runtime_error("cannot save annotated image: " + output_path.string());
            }
        } catch (const std::exception& exception) {
            error = exception.what();
        }
        const double total_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - total_start).count();
        auto& count = counts[sample.condition];
        count.samples++;
        if (detected_iou50) count.detected_iou50++;
        if (detected_iou70) count.detected_iou70++;
        if (full_correct_iou50) count.full_correct_iou50++;
        if (full_correct_iou70) count.full_correct_iou70++;
        result << csv_escape(sample.sample_id) << ',' << csv_escape(sample.condition) << ','
               << csv_escape(sample.ground_truth) << ',' << (detected_iou50 ? 1 : 0) << ','
               << (detected_iou70 ? 1 : 0) << ',' << csv_escape(predicted) << ','
               << (full_correct_iou50 ? 1 : 0) << ',' << (full_correct_iou70 ? 1 : 0) << ','
               << csv_escape(plate_type) << ',' << confidence << ',' << detections.size() << ','
               << detection_ms << ',' << max_iou << ',' << ocr_ms << ',' << total_ms << ','
               << csv_escape(sample.relative_path) << ',' << csv_escape(error) << '\n';
        ++processed;
        std::cout << "[ACCURACY] " << processed << '/' << samples.size() << ' '
                  << sample.sample_id << " GT=" << sample.ground_truth
                  << " PRED=" << (predicted.empty() ? "NONE" : predicted)
                  << " IoU50=" << (detected_iou50 ? 1 : 0)
                  << " IoU70=" << (detected_iou70 ? 1 : 0)
                  << " IoU=" << std::fixed << std::setprecision(3) << max_iou
                  << " correct50=" << (full_correct_iou50 ? 1 : 0)
                  << " correct70=" << (full_correct_iou70 ? 1 : 0);
        if (!error.empty()) std::cout << " ERROR=" << error;
        std::cout << '\n';
    }
    result.close();

    std::ofstream summary(o.accuracy_summary_csv, std::ios::trunc);
    if (!summary)
        throw std::runtime_error("cannot open accuracy summary: " + o.accuracy_summary_csv);
    summary << "\xef\xbb\xbf"
               "condition,samples,detected_iou50,detected_iou70,full_correct_iou50,"
               "full_correct_iou70,detection_rate_iou50_pct,detection_rate_iou70_pct,"
               "full_plate_accuracy_iou50_pct,full_plate_accuracy_iou70_pct\n";
    AccuracyCount total;
    auto write_count = [&](const std::string& condition, const AccuracyCount& value) {
        const double detection_rate50 = value.samples ? 100.0 * value.detected_iou50 / value.samples : 0.0;
        const double detection_rate70 = value.samples ? 100.0 * value.detected_iou70 / value.samples : 0.0;
        const double full_rate50 = value.samples ? 100.0 * value.full_correct_iou50 / value.samples : 0.0;
        const double full_rate70 = value.samples ? 100.0 * value.full_correct_iou70 / value.samples : 0.0;
        summary << csv_escape(condition) << ',' << value.samples << ','
                << value.detected_iou50 << ',' << value.detected_iou70 << ','
                << value.full_correct_iou50 << ',' << value.full_correct_iou70 << ','
                << std::fixed << std::setprecision(3) << detection_rate50 << ','
                << detection_rate70 << ',' << full_rate50 << ',' << full_rate70 << '\n';
    };
    for (const auto& [condition, value] : counts) {
        write_count(condition, value);
        total.samples += value.samples;
        total.detected_iou50 += value.detected_iou50;
        total.detected_iou70 += value.detected_iou70;
        total.full_correct_iou50 += value.full_correct_iou50;
        total.full_correct_iou70 += value.full_correct_iou70;
    }
    write_count("总体", total);
    summary.close();
    HLPR_ReleaseContext(lpr);
    std::cout << "[ACCURACY] results=" << o.accuracy_csv
              << " summary=" << o.accuracy_summary_csv << '\n';
    return 0;
}

void append_raw_csv(const Options& o, const std::vector<FrameTiming>& values,
                    std::size_t start_index = 0)
{
    ensure_parent(o.raw_csv);
    const bool header = file_is_empty(o.raw_csv);
    std::ofstream file(o.raw_csv, std::ios::app);
    if (!file) throw std::runtime_error("cannot open raw CSV: " + o.raw_csv);
    if (header) file << "preprocess,ocr_mode,ai_mode,width,height,frame_index,bytes,capture_ms,"
                        "copy_ms,convert_ms,preprocess_ms,preprocess_cpu_ms,npu_ms,frame_ms,"
                        "device_dma_ms\n";
    file << std::fixed << std::setprecision(6);
    for (std::size_t i = 0; i < values.size(); ++i) {
        const auto& v = values[i];
        file << preprocess_name(o.preprocess) << ',' << ocr_name(o.ocr_mode) << ',' << o.ai_mode
             << ',' << o.width << ',' << o.height << ',' << (start_index + i) << ','
             << v.bytes << ','
             << v.capture_ms << ',' << v.copy_ms << ',' << v.convert_ms << ','
             << v.preprocess_ms << ',' << v.preprocess_cpu_ms << ',' << v.npu_ms << ','
             << v.frame_ms << ','
             << v.device_ns / 1.0e6 << '\n';
    }
}

// [T01] 新版逐帧性能表：独立表头，不追加到旧格式 CSV。
// ai_pipeline_ms = 现有 npu_ms 的口径（engine+测速+车道+决策+overlay），
// 不是纯 NPU 时间；纯模型分项在后续轮次补齐（见执行汇报 T01 未覆盖项）。
void append_perf_frames_v2(const Options& o,
                           const std::vector<FrameTiming>& values,
                           std::size_t start_index, const std::string& run_id)
{
    if (values.empty())
        return;
    ensure_parent(o.perf_frames_csv);
    static const char* kPerfFramesHeader =
        "run_id,schema_version,frame_index,frame_ms,capture_ms,copy_ms,"
        "convert_ms,preprocess_ms,preprocess_cpu_ms,ai_pipeline_ms,"
        "device_dma_ms,light_scheduled,light_attempted,light_succeeded,"
        "light_failed,light_cached,sign_scheduled,sign_attempted,"
        "sign_succeeded,sign_failed,sign_cached,object_scheduled,"
        "object_attempted,object_succeeded,object_failed,object_cached,"
        "det_count";
    const bool header = csv_header_pending(o.perf_frames_csv, kPerfFramesHeader);
    std::ofstream file(o.perf_frames_csv, std::ios::app);
    if (!file)
        throw std::runtime_error("cannot open perf frames CSV: " + o.perf_frames_csv);
    if (header)
        file << kPerfFramesHeader << '\n';
    file << std::fixed << std::setprecision(6);
    for (std::size_t i = 0; i < values.size(); ++i) {
        const auto& v = values[i];
        // cached = 未调度但有旧结果可显示（本帧该模型没有成功新观察）。
        const int light_cached = (!v.light_scheduled && !v.light_failed) ? 1 : 0;
        const int sign_cached = (!v.sign_scheduled && !v.sign_failed) ? 1 : 0;
        const int object_cached =
            (!v.object_scheduled && !v.object_failed) ? 1 : 0;
        file << run_id << ",3," << (start_index + i) << ',' << v.frame_ms << ','
             << v.capture_ms << ',' << v.copy_ms << ',' << v.convert_ms << ','
             << v.preprocess_ms << ',' << v.preprocess_cpu_ms << ',' << v.npu_ms
             << ',' << v.device_ns / 1.0e6 << ',' << v.light_scheduled << ','
             << v.light_attempted << ',' << v.light_succeeded << ','
             << v.light_failed << ',' << light_cached << ',' << v.sign_scheduled
             << ',' << v.sign_attempted << ',' << v.sign_succeeded << ','
             << v.sign_failed << ',' << sign_cached << ',' << v.object_scheduled
             << ',' << v.object_attempted << ',' << v.object_succeeded << ','
             << v.object_failed << ',' << object_cached << ',' << v.det_count
             << '\n';
    }
}

void write_perf_summary_v2(const Options& o, RunAccumulator& acc,
                           double seconds, const std::string& run_id)
{
    ensure_parent(o.perf_summary_csv);
    static const char* kPerfSummaryHeader =
        "run_id,schema_version,frames,seconds,processed_fps,frame_ms_mean,"
        "frame_ms_p95,frame_ms_max,light_succeeded,light_failed,light_hz,"
        "sign_succeeded,sign_failed,sign_hz,object_succeeded,object_failed,"
        "object_hz,p95_method";
    const bool header = csv_header_pending(o.perf_summary_csv,
                                           kPerfSummaryHeader);
    std::ofstream file(o.perf_summary_csv, std::ios::app);
    if (!file)
        throw std::runtime_error("cannot open perf summary CSV: " + o.perf_summary_csv);
    if (header)
        file << kPerfSummaryHeader << '\n';
    const double fps = seconds > 0 ? acc.frames / seconds : 0.0;
    const double mean = acc.frames ? acc.frame_ms_sum / acc.frames : 0.0;
    // [N1-3] 精确 P95（样本未溢出）；溢出时用桶近似并在 p95_method 注明。
    const double p95 = acc.sample_overflow
                           ? RunAccumulator::p95_of(acc.buckets, acc.frames,
                                                    acc.frame_ms_max)
                           : RunAccumulator::exact_p95(acc.frame_samples);
    file << std::fixed << std::setprecision(4) << run_id << ",3," << acc.frames
         << ',' << seconds << ',' << fps << ',' << mean << ',' << p95 << ','
         << acc.frame_ms_max << ',' << acc.light_ok << ',' << acc.light_fail
         << ',' << (seconds > 0 ? acc.light_ok / seconds : 0.0) << ','
         << acc.sign_ok << ',' << acc.sign_fail << ','
         << (seconds > 0 ? acc.sign_ok / seconds : 0.0) << ',' << acc.object_ok
         << ',' << acc.object_fail << ','
         << (seconds > 0 ? acc.object_ok / seconds : 0.0) << ','
         << acc.p95_method() << '\n';
}

void append_summary_csv(const Options& o, RunAccumulator& acc,
                        const CpuUsage& cpu, double seconds, const OcrStatistics& ocr)
{
    // [N1-3] 旧表 P95 保持精确语义（样本未溢出时）；溢出时退化为桶近似，
    // 并在新增的 p95_method 列明确标注，不静默冒充精确值。
    const std::uint64_t n = acc.frames;
    const double fps = seconds > 0 ? static_cast<double>(n) / seconds : 0;
    const double rate =
        acc.device_ns_sum ? acc.bytes_sum / (acc.device_ns_sum / 1.0e9) / 1.0e6 : 0;
    const double preprocess_wall_total = acc.preprocess_sum;
    const double preprocess_cpu_percent =
        preprocess_wall_total > 0.0
            ? acc.preprocess_cpu_sum / preprocess_wall_total * 100.0
            : 0.0;
    const double p95_frame =
        acc.sample_overflow
            ? RunAccumulator::p95_of(acc.buckets, acc.frames, acc.frame_ms_max)
            : RunAccumulator::exact_p95(acc.frame_samples);
    const double p95_preprocess =
        acc.sample_overflow
            ? RunAccumulator::p95_of(acc.preprocess_buckets, acc.frames,
                                     acc.preprocess_max)
            : RunAccumulator::exact_p95(acc.preprocess_samples);
    ensure_parent(o.summary_csv);
    static const char* kSummaryHeader =
        "preprocess,ocr_mode,ai_mode,width,height,frame_bytes,frames,warmup,fps,"
        "mean_capture_ms,mean_copy_ms,mean_convert_ms,mean_preprocess_ms,"
        "p95_preprocess_ms,mean_preprocess_cpu_ms,preprocess_cpu_pct,"
        "mean_frame_ms,p95_frame_ms,cpu_process_pct,"
        "cpu_machine_pct,dma_data_rate_MBps,ocr_submitted,ocr_completed,"
        "ocr_dropped,mean_ocr_ms,p95_method";
    const bool header = csv_header_pending(o.summary_csv, kSummaryHeader);
    std::ofstream file(o.summary_csv, std::ios::app);
    if (!file) throw std::runtime_error("cannot open summary CSV: " + o.summary_csv);
    if (header) file << kSummaryHeader << '\n';
    file << std::fixed << std::setprecision(6) << preprocess_name(o.preprocess) << ','
         << ocr_name(o.ocr_mode) << ',' << o.ai_mode << ',' << o.width << ',' << o.height
         << ',' << (static_cast<std::uint64_t>(o.width) * o.height * 2) << ',' << n
         << ',' << o.warmup << ',' << fps << ',' << acc.mean(acc.capture_sum) << ','
         << acc.mean(acc.copy_sum) << ','
         << acc.mean(acc.convert_sum) << ',' << acc.mean(acc.preprocess_sum) << ','
         << p95_preprocess
         << ',' << acc.mean(acc.preprocess_cpu_sum) << ',' << preprocess_cpu_percent << ','
         << acc.mean(acc.frame_ms_sum)
         << ',' << p95_frame << ',' << cpu.process_percent
         << ',' << cpu.machine_percent << ',' << rate << ',' << ocr.submitted << ','
         << ocr.completed << ',' << ocr.dropped << ',' << ocr.mean_ms << ','
         << acc.p95_method() << '\n';
    std::cout << std::fixed << std::setprecision(3) << "\n[RESULT] mode="
              << preprocess_name(o.preprocess) << " OCR=" << ocr_name(o.ocr_mode)
              << " FPS=" << fps << " preprocess=" << acc.mean(acc.preprocess_sum)
              << " ms CPU="
              << cpu.process_percent << "% (process), preprocess CPU="
              << preprocess_cpu_percent << "% DMA=" << rate << " MB/s"
              << " p95_method=" << acc.p95_method() << "\n";
}

} // namespace

int run_application(int argc, char** argv, const Options& o)
{
    try {
        const bool requested_display_worker = g_disp_thread;
        const auto display_policy = p3_display::choose_backend(
            o.display, requested_display_worker, cv::getBuildInformation());
        g_disp_thread = display_policy.use_worker;
        std::printf("[disp-policy] display=%d backend=%s requested_worker=%d selected=%s reason=%s\n",
                    o.display ? 1 : 0, display_policy.backend.c_str(),
                    requested_display_worker ? 1 : 0, display_policy.selected,
                    display_policy.reason);
        // [T00] 统一配置：尽早加载并显式打印全部生效值（非法值 fail-fast）。
        p3_config_print(p3_config());
        // [T01] 本 run 的标识与 run_config.json（配置 + 命令行，供审计复现）。
        const std::string run_id = make_run_id();
        {
            ensure_parent(o.run_config_json);
            std::ofstream cfg_file(o.run_config_json, std::ios::trunc);
            if (cfg_file) {
                std::string argv_json = "[";
                for (int i = 0; i < argc; ++i) {
                    argv_json += (i ? ", \"" : "\"");
                    argv_json += argv[i];
                    argv_json += "\"";
                }
                argv_json += "]";
                cfg_file << p3_config_json(p3_config(), run_id, argv_json,
                                           o.ai_mode, o.fpga_decision,
                                           o.prefetch ? 1 : 0);
                std::cout << "[T01] run_config.json written: " << o.run_config_json
                          << " (run_id=" << run_id << ")\n";
            } else {
                std::cerr << "[T01] WARN cannot write " << o.run_config_json << "\n";
            }
        }
        if (o.ocr_replay) return run_ocr_replay(o);
        if (!o.input_dir.empty()) return run_accuracy_dataset(o);
        const bool prep_enabled = p3_prep::enabled(std::getenv("P3_PREP_PIPE"));
        if (prep_enabled && (!o.prefetch || o.ai_mode != 3 || o.width != 1280 ||
                             o.height != 360 || o.timeout_ms == 0 || o.timeout_ms > 1000))
            throw std::invalid_argument("prep candidate requires mode3/SBS1280x360/prefetch/finite capture <=1000ms");
        PcieFrameSource source(o.device, o.width, o.height);
        std::cout << "[PCIe] true coherent mmap, two buffers: " << source.mmap_bytes()
                  << " bytes; one frame: " << source.frame_bytes() << " bytes\n";
        std::vector<std::uint8_t> copy_buffer;
        if (o.preprocess == PreprocessMode::CopyCpu) copy_buffer.resize(source.frame_bytes());
        cv::Mat color(o.height, o.width, CV_8UC3);
        std::unique_ptr<RgaPreprocessor> rga;
        if (o.preprocess == PreprocessMode::MmapRga)
            rga = std::make_unique<RgaPreprocessor>(source, color);

        std::optional<P3DecisionBridge> decision_bridge;
        if (o.fpga_decision) {
            if (o.ai_mode != 3)
                throw std::invalid_argument("--fpga-decision requires --ai-mode 3");
            decision_bridge.emplace();
            std::string error;
            if (!decision_bridge->open(o.pci_bdf, &error))
                throw std::runtime_error("open FPGA decision bridge: " + error);
            std::cout << "[decision] BAR keys/LED bridge on " << o.pci_bdf
                      << "; lane_source=twinlite (no RTL lane access)\n";
        }

        std::unique_ptr<UfldLaneModel> ufld_lane_model;
        std::unique_ptr<TwinliteLaneModel> twinlite_lane_model;
        const bool enable_crosswalk = env_flag("P3_ENABLE_CROSSWALK", false);
        p3_crosswalk::Config crosswalk_config;
        crosswalk_config.prediction_enabled=env_flag("P3_ENABLE_CROSSWALK_PREDICTION", true);
        p3_crosswalk::PredictiveDetector crosswalk_detector(crosswalk_config);
        std::ofstream crosswalk_csv;
        if (enable_crosswalk) {
            if (const char* path = std::getenv("P3_CROSSWALK_CSV")) {
                if (std::ifstream(path).good()) throw std::runtime_error("crosswalk CSV already exists");
                crosswalk_csv.open(path);
                if (!crosswalk_csv) throw std::runtime_error("cannot open crosswalk CSV");
                crosswalk_csv << "delivery_id,observation_id,source_ms,now_ms,state,stripes,persons,inside,boundary,unknown,geometry_ms,association_draw_ms,twin_support,projection_ms,prediction_age_ms,prediction_sigma_px,tracker_view_ms\n";
            }
            std::printf("[crosswalk] enabled=1 visual-only left-eye no-extra-NPU decision-unchanged\n");
        }

        int ufld_lane_interval = 20;
        const bool enable_twinlite = env_flag("P3_ENABLE_TWINLITE", false);
        const bool enable_ufld = !enable_twinlite &&
                                 p3_config().enable_ufld;  // [T00] 统一配置
        if (o.ai_mode == 3 && enable_ufld) {
            ufld_lane_model = std::make_unique<UfldLaneModel>();
            if (!ufld_lane_model->init(
                    "/root/board_deploy/ufldv2_320x1600.rknn")) {
                std::cerr << "[ufld] disabled: model init failed\n";
                ufld_lane_model.reset();
            }
            ufld_lane_interval = p3_config().lane_interval;  // [T00] 统一配置
            std::cout << "[ufld] lane interval=" << ufld_lane_interval << "\n";
        } else if (o.ai_mode == 3 && enable_twinlite) {
            twinlite_lane_model = std::make_unique<TwinliteLaneModel>();
            if (!twinlite_lane_model->init(
                    "/root/board_deploy/twinlite_small_lane_288x512_int8_calib128.rknn")) {
                std::cerr << "[twinlite] disabled: model init failed\n";
                twinlite_lane_model.reset();
            }
            ufld_lane_interval = p3_config().lane_interval;
            std::cout << "[twinlite] lane source interval="
                      << ufld_lane_interval << "\n";
        } else if (o.ai_mode == 3) {
            std::cout << "[lane] TwinLite unavailable: no lane authorization\n";
        }

        if (o.ai_mode == 1 || o.ai_mode == 2) yolo_init();
        P_HLPR_Context lpr = nullptr;
        std::unique_ptr<OcrPipeline> ocr;
        if (o.ai_mode == 1) {
            lpr = create_lpr_context();
            ocr = std::make_unique<OcrPipeline>(lpr, o.ocr_mode);
        }

        cv::Ptr<cv::freetype::FreeType2> font;
        if (o.display) {
            font = cv::freetype::createFreeType2();
            font->loadFontData("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc", 0);
            if (g_display_on_gui) g_gui_font = font;
            // [disp-thread2] in thread mode the worker owns the window: highgui binds a
            // window to its creating thread, which is why v1 stayed black.
            if (g_disp_thread) {
                // [fs-decouple2] fullscreen must not be tied to --direct-display:
                // the canvas layout puts the text in the top band and still wants
                // the whole screen.
                const char* __nofs2 = std::getenv("P3_NOFULLSCREEN");
                const bool __fs2 = !(__nofs2 && __nofs2[0] != 0 && __nofs2[0] != '0' &&
                                      __nofs2[0] != 'f');
                g_disp_publisher.start("FPGA Heterogeneous Vision", __fs2);
                // [d2] a worker that cannot open a window must not deadlock us and
                // must not silently black-screen the demo: fall back to drawing on
                // the main thread, which is the old, known-good path.
                if (!g_disp_publisher.wait_ready()) {
                    std::fprintf(stderr, "[disp] worker not ready: %s\n",
                                 g_disp_publisher.error().c_str());
                    g_disp_publisher.stop();
                    // [d2d] failed() only says the worker reported an error; it
                    // may still be tearing its window down.  exited() is the only
                    // proof that the old GUI owner is gone.
                    g_disp_worker_exited = g_disp_publisher.exited();
                    g_disp_thread = false;
                    if (!g_disp_worker_exited) {
                        g_disp_disabled = true;
                        std::fprintf(stderr,
                                     "[disp] worker exit unconfirmed; display disabled,"
                                     " supervisor must restart the candidate\n");
                    }
                } else {
                    g_disp_worker_exited = false;   // still running, owns the window
                }
            } else if (g_qt_channel) {
                if (!g_qt_channel->start("FPGA Heterogeneous Vision",
                                        !env_flag("P3_NOFULLSCREEN", false)))
                    throw std::runtime_error("main GUI startup failed: " + g_qt_channel->status().error);
            } else {
                cv::namedWindow("FPGA Heterogeneous Vision", cv::WINDOW_NORMAL);
            // [nofs] experiment: keep the window out of fullscreen when the env
            // is set, so imshow's scaling cost can be measured on its own.
            const char* __nofs = std::getenv("P3_NOFULLSCREEN");
            const bool __skip_fs = __nofs && __nofs[0] != 0 && __nofs[0] != '0' && __nofs[0] != 'f';
            // [fs-decouple] fullscreen is a display property, not a property of
            // which image we show: the canvas layout keeps its text in the top band
            // and should be allowed to fill the screen as well.
            if (!__skip_fs)
                cv::setWindowProperty("FPGA Heterogeneous Vision",
                                      cv::WND_PROP_FULLSCREEN,
                                      cv::WINDOW_FULLSCREEN);
            }
        }
        if (o.save_every) std::filesystem::create_directories(o.save_dir);

        std::cout << "[Warmup] " << o.warmup << " frames\n";
        for (std::uint32_t i = 0; i < o.warmup; ++i) {
            const std::uint32_t buffer_index = i % source.buffer_count();
            source.capture(buffer_index, o.timeout_ms);
            double copy_ms = 0.0, convert_ms = 0.0;
            preprocess_frame(o.preprocess, source.buffer(buffer_index), copy_buffer,
                             o.width, o.height, color, rga.get(), buffer_index,
                             copy_ms, convert_ms);
            if (o.ai_mode == 3) {
                // Warmup follows the same ownership contract as live frames.
                // Never dirty the virtual-address-imported RGA destination.
                cv::Mat warmup_render = p3_render::owned_frame(color);
                p3_engine_run(warmup_render);
                if (decision_bridge) {
                    // [N1-5] 传入灯观察快照（去重投票 + 证据 TTL 判定）。
                    const P3LampObservation lamp_obs =
                        p3_engine_last_light_observation();
                    decision_bridge->update(p3_engine_last_detections(), i + 1,
                                            &lamp_obs);
                }
            }
        }

        // [T01] 四模型预热（含复核器）→ 状态清零，正式统计不受预热污染。
        if (o.ai_mode == 3) {
            p3_engine_warmup();
            p3_engine_reset_state();
            if (decision_bridge) decision_bridge->reset_state();
        }

        // One existing capture/preparation worker per RUN. Destruction order:
        // prefetcher stops before prepared source, its RGA handles and buffers.
        std::unique_ptr<PreparedSource> prepared_source;
        std::unique_ptr<PreparedPrefetcher> prepared_prefetcher;
        std::unique_ptr<FILE, decltype(&std::fclose)> prep_trace(nullptr, &std::fclose);
        if (prep_enabled) {
            const char* trace_path = std::getenv("P3_PREP_PIPE_CSV");
            if (!trace_path || !*trace_path)
                throw std::invalid_argument("P3_PREP_PIPE_CSV required for candidate evidence");
            prep_trace.reset(std::fopen(trace_path, "wx"));
            if (!prep_trace) throw std::runtime_error("prep trace must be a NEW private file");
            std::fprintf(prep_trace.get(), "frame,delivery_id,source_seq,bytes,device_ns,capture_ready_ms,prepared_ready_ms,consume_ms,capture_wall_ms,prep_ms,queue_ms\n");
            prepared_source = std::make_unique<PreparedSource>(source,
                [&](std::uint32_t) { return std::make_unique<PreparedPixels>(source, o.preprocess); },
                [&](std::uint32_t slot, PreparedPixels& pixels) {
                    p3_prep::Timing result;
                    const double cpu_start = current_thread_cpu_ms();
                    result.preprocess_ms = preprocess_frame(o.preprocess, source.buffer(slot),
                        pixels.copy, o.width, o.height, pixels.color, pixels.rga.get(), slot,
                        result.copy_ms, result.convert_ms);
                    result.preprocess_cpu_ms = current_thread_cpu_ms() - cpu_start;
                    return result;
                });
            rga.reset(); // warmup-only destination must not outlive/rebind its Mat
            prepared_prefetcher = std::make_unique<PreparedPrefetcher>(*prepared_source, o.timeout_ms);
            std::printf("[prep-pipe] enabled=1 lifetime=run slots=%u timestamp=capture-return-before-prep\n", source.buffer_count());
        }
        std::unique_ptr<FramePrefetcher> prefetcher;
        if (o.prefetch && !prep_enabled)
            prefetcher = std::make_unique<FramePrefetcher>(source, o.timeout_ms);

        std::vector<FrameTiming> timing;
        RunAccumulator accumulator;  // [T01] 有界在线统计
        if (o.frames)
            timing.reserve(std::min<std::size_t>(o.frames, 257));
        ProcessCpuMeter cpu_meter;
        cpu_meter.start();
        const auto run_start = std::chrono::steady_clock::now();
        std::uint32_t index = 0;
        // [N1-3] lane_overlay 耗时改为有界在线统计（原实现逐帧 push_back，
        // 无限运行会持续增长）。
        RoadSpeedEstimator speed_estimator;
        const bool enable_speed = p3_config().enable_speed;  // [T00] 统一配置
        bool stop = false;
        while (!stop && (!o.frames || index < o.frames)) {
            if (g_qt_channel) {
                const auto status = g_qt_channel->status();
                const auto health = g_disp_health.observe(
                    status.published_seq, status.shown_seq, status.failed,
                    status.error, std::chrono::steady_clock::now());
                if (health.kind != p3_disp::Health::None) {
                    g_disp_fault_reported = true;
                    g_qt_channel->request_stop();
                    throw std::runtime_error("main GUI failed/stalled: " + health.detail);
                }
                if (status.stop_requested) break;
            }
            const auto frame_start = std::chrono::steady_clock::now();
            FrameTiming t;
            // [C01] 本帧统一源时间（steady ms，采集返回、预处理前打点）。
            double frame_source_ms = 0.0;
            p3_prep::BusyLease<PreparedPrefetcher> prepared_lease;
            if (prepared_prefetcher) {
                const auto captured = prepared_prefetcher->wait();
                prepared_lease.acquire(*prepared_prefetcher, captured.buffer_index);
                const auto& result = captured.capture;
                color = prepared_source->prepared(captured.buffer_index).color;
                frame_source_ms = result.capture_ready_ms;
                p3_engine_note_capture(frame_source_ms); // producer capture-return; never consumption now
                t.capture_ms = result.capture_wall_ms;
                t.device_ns = result.device_elapsed_ns;
                t.bytes = result.bytes;
                t.preprocess_ms = result.preparation.preprocess_ms;
                t.preprocess_cpu_ms = result.preparation.preprocess_cpu_ms;
                t.copy_ms = result.preparation.copy_ms;
                t.convert_ms = result.preparation.convert_ms;
                const double consume_ms = p3_prep::mono_ms();
                const int recorded = std::fprintf(prep_trace.get(), "%u,%llu,%u,%u,%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    index + 1, static_cast<unsigned long long>(captured.delivery_id),
                    result.sequence, result.bytes, static_cast<unsigned long long>(result.device_elapsed_ns),
                    result.capture_ready_ms, result.prepared_ready_ms, consume_ms,
                    result.capture_wall_ms, result.preparation.preprocess_ms,
                    consume_ms - result.prepared_ready_ms);
                if (recorded < 0) throw std::runtime_error("prep evidence write failed");
            } else {
            FramePrefetcher::Lease captured;
            std::uint32_t buffer_index = 0;
            if (prefetcher) {
                captured = prefetcher->wait();
                buffer_index = captured.buffer_index;
                // [C01] OFF 与 ON 同时域：producer 在采集返回、预处理前打点。
                frame_source_ms = captured.capture_ready_ms;
            } else {
                buffer_index = index % source.buffer_count();
                const auto capture_start = std::chrono::steady_clock::now();
                captured.capture = source.capture(buffer_index, o.timeout_ms);
                const auto capture_done = std::chrono::steady_clock::now();
                captured.capture_ms = std::chrono::duration<double, std::milli>(
                                          capture_done - capture_start)
                                          .count();
                frame_source_ms = std::chrono::duration<double, std::milli>(
                                      capture_done.time_since_epoch())
                                      .count();
            }
            t.capture_ms = captured.capture_ms;
            // [C01] 统一源时间：采集返回、预处理前；不再在消费点取 now。
            p3_engine_note_capture(frame_source_ms);
            t.device_ns = captured.capture.device_elapsed_ns;
            t.bytes = captured.capture.bytes;
            const double preprocess_cpu_start = current_thread_cpu_ms();
            t.preprocess_ms = preprocess_frame(o.preprocess, source.buffer(buffer_index),
                                               copy_buffer, o.width, o.height, color,
                                               rga.get(), buffer_index,
                                               t.copy_ms, t.convert_ms);
            t.preprocess_cpu_ms = current_thread_cpu_ms() - preprocess_cpu_start;
            if (prefetcher)
                prefetcher->release(buffer_index);

            }

            // [EXP gate4] 画布复用：仅"需要显示 或 需要存帧"时才构建 720p 画布；
            // headless 且本帧不存帧时整帧跳过（省 2.76MB zeros + 1.38MB copyTo）
            static cv::Mat g_canvas;
            const bool direct_ai_display =
                o.display && o.direct_display && o.ai_mode == 3;
            // [PERF-P3a] GUI-thread composition only for the canvas channel.
            const bool compose_gui =
                g_display_on_gui && !direct_ai_display && g_qt_channel != nullptr;
            const bool need_save = o.save_every && (index + 1) % o.save_every == 0;
            const bool need_canvas = (!direct_ai_display && o.display) || need_save;
            int y_offset = 0;
            // [残影-fix] 合成逻辑提取到 display_compose.h（逐句等价），
            // 供桌面回归直连真实路径（A/B 帧无残留断言）。
            cv::Mat display = p3_display::compose_display_base(
                g_canvas, color, o.width, o.height, need_canvas, o.ai_mode,
                y_offset);

            const auto npu_start = std::chrono::steady_clock::now();
            cv::Mat ai_render;
            P3DecisionOutput decision;
            RoadSpeedEstimator::Estimate speed;
            // [PERF-SPLIT 2026-10-22] Worker-side stage split (opt-in log).
            double __split_eng_ms = -1.0;
            double __split_join_ms = -1.0;
            double __split_dec_ms = -1.0;
            if (o.ai_mode == 2) yolo_run_main(display);
            else if (o.ai_mode == 3) {
                // Keep the captured frame immutable. Detection overlays are
                // rendered onto a fresh output frame every iteration.
                // [PERF-P1a 2026-10-22] Overlap the Twin update with the
                // engine's light/object pipeline. The two use separate RKNN
                // contexts; every consumer (draw, road input, decision) joins
                // the future first, so the frame contract is unchanged.
                // [PERF-P2a 2026-10-22] The phase-stagger experiment was
                // measured as an 11% regression (it removed the CPU overlap
                // the single NPU was not constrained by); do not re-enable.
                std::future<bool> twin_update_future;
                if (twinlite_lane_model &&
                    (index % static_cast<std::uint32_t>(ufld_lane_interval) == 0)) {
                    try {
                        twin_update_future = std::async(
                            std::launch::async, [&]() {
                                return twinlite_lane_model->update(
                                    color, static_cast<std::uint64_t>(index) + 1);
                            });
                    } catch (const std::exception&) {
                        // [AUDIT-E5] Thread creation failed: keep the frame
                        // contract by running the update inline.
                        twinlite_lane_model->update(
                            color, static_cast<std::uint64_t>(index) + 1);
                    }
                }
                ai_render = p3_render::owned_frame(color);
                const auto __eng0 = std::chrono::steady_clock::now();
                p3_engine_run(ai_render);
                const auto __eng1 = std::chrono::steady_clock::now();
                if (twin_update_future.valid())
                    twin_update_future.get();
                const auto __join = std::chrono::steady_clock::now();
                __split_eng_ms = std::chrono::duration<double, std::milli>(
                                     __eng1 - __eng0).count();
                __split_join_ms = std::chrono::duration<double, std::milli>(
                                      __join - __eng1).count();
                if (enable_speed)
                    speed = speed_estimator.update(color);
                if (ufld_lane_model &&
                    (index % static_cast<std::uint32_t>(ufld_lane_interval) == 0)) {
                    ufld_lane_model->update(color);
                }
                if (ufld_lane_model)
                    ufld_lane_model->draw(ai_render);
                if (twinlite_lane_model)
                    twinlite_lane_model->draw(
                        ai_render, static_cast<std::uint64_t>(index) + 1,
                        static_cast<std::uint64_t>(ufld_lane_interval + 2));
                P3RoadSafetyAssessment road_safety;
                const std::uint64_t delivery_id =
                    static_cast<std::uint64_t>(index) + 1;
                RoadInputFrame frame_input;
                frame_input.frame_source_ms = frame_source_ms;
                // [C-R1] 决策当前时间在真正求值时打点；桥会用其唯一决策
                // 时钟覆盖此值。不是帧采集时间。
                frame_input.decision_now_ms =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count();
                frame_input.delivery_id = delivery_id;
                const EgoLanePostprocessResult* lane_input =
                    (twinlite_lane_model &&
                     twinlite_lane_model->fresh_at(delivery_id, 0))
                        ? &twinlite_lane_model->lane_result() : nullptr;
                // [C-R1/C-R2] 与集成测试共用同一条打包路径。
                P3RoadSafetyInput road_input = build_road_input(
                    lane_input, p3_engine_last_detections(),
                    p3_engine_last_object_observation(), frame_input);
                if (decision_bridge) {
                    // [N1-5] 灯观察快照随帧传入（与 detections 同源）。
                    const P3LampObservation lamp_obs =
                        p3_engine_last_light_observation();
                    // [L01] Vehicle advice is decoupled from TwinLite
                    // readiness: the observation is always built and the
                    // input always passed. Without evidence the advisor
                    // returns Unknown instead of a null pointer silently
                    // cancelling every vehicle veto.
                    const P3RoadSafetyInput* safety_input = &road_input;
                    decision = decision_bridge->update(
                        p3_engine_last_detections(), index + 1, &lamp_obs,
                        safety_input);
                    if (safety_input) road_safety = decision.road_safety;
                    if ((index % 30) == 0) {
                        std::printf(
                            "[road-advice] frame=%u front=%s objects=%d/%d/%d "
                            "left_change=%s right_change=%s reason=%s\n",
                            index + 1,
                            p3_road_action_name(road_safety.front_action),
                            road_safety.front_object_count,
                            road_safety.left_object_count,
                            road_safety.right_object_count,
                            p3_lane_change_advice_name(road_safety.left_change),
                            p3_lane_change_advice_name(road_safety.right_change),
                            road_safety.reason.c_str());
                    }
                } else {
                    road_safety = assess_p3_road_safety(road_input);
                }
                // The existing bridge has already computed its action. This
                // observer only annotates the owned output frame and does not
                // alter road_input, lamp priority, TTL or FPGA output fields.
                if (enable_crosswalk) {
                    const double cw_now = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    const cv::Mat* mask = twinlite_lane_model
                        ? twinlite_lane_model->mask_for_success(delivery_id) : nullptr;
                    const cv::Mat cw_left = color(cv::Rect(0, 0, color.cols / 2, color.rows));
                    const bool cw_ran = (index % 2 == 0);
                    if (cw_ran) crosswalk_detector.update({delivery_id, frame_source_ms, cw_now,
                                                          cw_left, mask ? *mask : cv::Mat{}});
                    const auto cw_view_start=std::chrono::steady_clock::now();
                    const auto cw_result = crosswalk_detector.view(delivery_id, cw_now, frame_source_ms);
                    const double cw_view_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-cw_view_start).count();
                    const auto cw_obs = p3_engine_last_object_observation();
                    std::vector<p3_crosswalk::Person> cw_persons;
                    for (const auto& d : p3_engine_last_detections()) {
                        if (d.model_id != 2 || d.cls != 0) continue;
                        cw_persons.push_back({{d.x1, d.y1, d.x2-d.x1, d.y2-d.y1},
                            d.score, d.obs_run_id, d.obs_capture_ms, d.obs_age_frames,
                            d.obs_origin == 0 && cw_obs.status == 1 &&
                                d.obs_run_id == cw_obs.run_id});
                    }
                    const auto cw_draw_start = std::chrono::steady_clock::now();
                    const auto cw_people = p3_crosswalk::associate(cw_result, cw_persons, cw_now);
                    cv::Mat cw_render = ai_render(cv::Rect(0, 0, ai_render.cols / 2, ai_render.rows));
                    p3_crosswalk::draw(cw_render, cw_result, cw_people);
                    const double cw_draw_ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now()-cw_draw_start).count();
                    int cw_inside=0, cw_boundary=0, cw_unknown=0;
                    for (const auto& p : cw_people) {
                        cw_inside += p.relation == p3_crosswalk::Relation::Inside;
                        cw_boundary += p.relation == p3_crosswalk::Relation::Boundary;
                        cw_unknown += p.relation == p3_crosswalk::Relation::Unknown;
                    }
                    if (crosswalk_csv && index < 30000) {
                        crosswalk_csv << delivery_id << ',' << cw_result.delivery_id << ','
                            << std::fixed << std::setprecision(3)
                            << cw_result.source_ms << ',' << cw_now << ','
                            << p3_crosswalk::state_name(cw_result.state) << ','
                            << cw_result.stripes.size() << ',' << cw_people.size() << ','
                            << cw_inside << ',' << cw_boundary << ',' << cw_unknown << ','
                            << (cw_ran ? cw_result.elapsed_ms : 0.0) << ',' << cw_draw_ms << ','
                            << cw_result.twin_support << ',' << cw_result.projection_ms << ','
                            << cw_result.prediction_age_ms << ',' << cw_result.prediction_sigma_px << ',' << cw_view_ms << '\n';
                    }
                    if (index % 30 == 0) std::printf(
                        "[crosswalk] frame=%llu state=%s stripes=%zu person=%zu inside=%d geom=%.3f draw=%.3f\n",
                        static_cast<unsigned long long>(delivery_id),
                        p3_crosswalk::state_name(cw_result.state), cw_result.stripes.size(),
                        cw_people.size(), cw_inside, cw_ran ? cw_result.elapsed_ms : 0.0, cw_draw_ms);
                }
                if (decision_osd::enabled() && direct_ai_display) {
                    decision_osd::Context ctx;
                    ctx.bridge_connected = static_cast<bool>(decision_bridge);
                    ctx.output_valid = ctx.bridge_connected;
                    decision_osd::FreeTypeTextSink text(font);
                    decision_osd::Style style;
                    style.safe_area = decision_osd::direct_safe_area(ai_render);
                    style.compact = true;
                    {
                        const int bw = std::min(ai_render.cols / 2, 460);
                        style.grant_badge_area = cv::Rect(
                            ai_render.cols - bw - 8, 6, bw,
                            std::max(36, ai_render.rows / 10));
                    }
                    decision_osd::draw(ai_render, decision, ctx, style, text);
                }
                p3_display::place_ai_frame(display, ai_render, y_offset, o.ai_mode);
                if (y_offset && !compose_gui)
                    draw_lane_status(display, decision, font, speed,
                                     ufld_lane_model.get());
                if (direct_ai_display)
                    draw_lane_status_compact(ai_render, decision, font, speed);
                if (decision_osd::enabled() && !direct_ai_display && y_offset > 0 &&
                    !compose_gui) {
                    decision_osd::Context ctx;
                    ctx.bridge_connected = static_cast<bool>(decision_bridge);
                    ctx.output_valid = ctx.bridge_connected;
                    decision_osd::FreeTypeTextSink text(font);
                    decision_osd::Style style;
                    style.safe_area = decision_osd::canvas_band_safe_area(display, y_offset);
                    {
                        const int bw = std::min(display.cols / 2, 460);
                        style.grant_badge_area = cv::Rect(
                            display.cols - bw - 8, 6, bw,
                            std::max(36, display.rows / 14));
                    }
                    decision_osd::draw(display, decision, ctx, style, text);
                }
                __split_dec_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - __join).count();
            }
            else if (o.ai_mode == 1 && (index + 1) % o.ocr_interval == 0) {
                std::vector<cv::Rect> detections;
                yolo_run_plate(color, detections);
                if (detections.empty()) ocr->clear_overlays();
                else {
                    std::vector<cv::Mat> crops;
                    std::vector<cv::Rect> rects;
                    make_plate_crops(color, detections, crops, rects);
                    ocr->submit(std::move(crops), std::move(rects), y_offset);
                }
            }
            t.npu_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - npu_start).count();
            // [N1-1] 逐帧模型状态与检测数（perf_frames_v2）。
            if (o.ai_mode == 3) {
                const P3ModelFrameState ls = p3_engine_last_light_state();
                const P3ModelFrameState ss = p3_engine_last_sign_state();
                const P3ModelFrameState os = p3_engine_last_object_state();
                t.light_scheduled = ls.scheduled ? 1 : 0;
                t.light_attempted = ls.attempted ? 1 : 0;
                t.light_succeeded = ls.succeeded ? 1 : 0;
                t.light_failed = ls.failed ? 1 : 0;
                t.sign_scheduled = ss.scheduled ? 1 : 0;
                t.sign_attempted = ss.attempted ? 1 : 0;
                t.sign_succeeded = ss.succeeded ? 1 : 0;
                t.sign_failed = ss.failed ? 1 : 0;
                t.object_scheduled = os.scheduled ? 1 : 0;
                t.object_attempted = os.attempted ? 1 : 0;
                t.object_succeeded = os.succeeded ? 1 : 0;
                t.object_failed = os.failed ? 1 : 0;
                t.det_count =
                    static_cast<int>(p3_engine_last_detections().size());
            }
            if (ocr) {
                for (const auto& plate : ocr->overlays()) {
                    cv::rectangle(display, plate.rect, cv::Scalar(0, 255, 0), 3);
                    if (font) font->putText(display, plate.text + " (" + plate.type + ")",
                                            cv::Point(plate.rect.x, std::max(25, plate.rect.y - 10)),
                                            25, cv::Scalar(0, 255, 0), -1, 8, true);
                }
            }
            if (o.save_every && (index + 1) % o.save_every == 0) {
                const auto path = std::filesystem::path(o.save_dir) /
                                  ("frame_" + std::to_string(index + 1) + ".jpg");
                cv::imwrite(path.string(), display);
            }
            // [d2e] Watch for a live worker that stopped presenting frames.
            if (g_disp_thread && !g_disp_disabled) {
                const p3_disp::Health hh = g_disp_health.observe(
                    g_disp_publisher.published_seq(), g_disp_publisher.shown_seq(),
                    g_disp_publisher.failed(), g_disp_publisher.error(),
                    std::chrono::steady_clock::now());
                if (hh.kind == p3_disp::Health::Failed) {
                    std::fprintf(stderr,
                                 "[disp-health] FAILED pub_seq=%lu shown_seq=%lu: %s\n",
                                 hh.published, hh.shown, hh.detail.c_str());
                    g_disp_fault_reported = true;
                    g_disp_disabled = true;
                    g_disp_publisher.stop();
                    g_disp_worker_exited = g_disp_publisher.exited();
                } else if (hh.kind == p3_disp::Health::Stalled) {
                    std::fprintf(stderr,
                                 "[disp-health] STALLED age=%lldms pub_seq=%lu"
                                 " shown_seq=%lu: %s\n",
                                 hh.age_ms, hh.published, hh.shown, hh.detail.c_str());
                    g_disp_fault_reported = true;
                    g_disp_disabled = true;
                    g_disp_publisher.stop();
                    g_disp_worker_exited = g_disp_publisher.exited();
                }
            }
            if (o.display && !g_disp_disabled) {
                const auto __disp0 = std::chrono::steady_clock::now();
                // Restore the original ASCII HUD; keep the current Qt actor and OSD.
                std::string hud = std::string(preprocess_name(o.preprocess)) + " / AI " +
                                  std::to_string(o.ai_mode) + " " + ai_mode_name(o.ai_mode);
                if (o.ai_mode == 1)
                    hud += " / OCR " + std::string(ocr_name(o.ocr_mode));
                if (o.ai_mode == 3)
                    hud += " / det=" + std::to_string(p3_engine_last_detections().size());
                hud += " / #" + std::to_string(index + 1);
                if (compose_gui) {
                    // [PERF-P3a] Publish the pre-compose canvas; the GUI thread
                    // draws OSD/lane status, prescales and adds the HUD.
                    const auto __ims0 = std::chrono::steady_clock::now();
                    const bool accepted = g_qt_channel->publish(
                        p3_main_gui::owned_snapshot(
                            display, decision, index + 1, true, y_offset,
                            static_cast<bool>(decision_bridge), hud,
                            speed.kmh, speed.valid),
                        index + 1, index + 1);
                    const auto status = g_qt_channel->status();
                    if (status.failed)
                        throw std::runtime_error("main GUI publication failed: " + status.error);
                    stop = !accepted || status.stop_requested;
                    g_disp_imshow_ms += std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - __ims0).count();
                } else {
                cv::Mat shown = direct_ai_display ? ai_render : display;
                // [prescale] compose at 1920x1080 so imshow blits 1:1 instead of
                // upscaling every frame (round 2 measured 17.7 ms going into that).
                static const bool __prescale = [] {
                    const char* v = std::getenv("P3_PRESCALE");
                    return v && v[0] != 0 && v[0] != '0' && v[0] != 'f';
                }();
                if (__prescale) {
                    static cv::Mat __canvas;
                    static int __last_w = -1, __last_h = -1;
                    if (__canvas.rows != 1080 || __canvas.cols != 1920) {
                        __canvas = cv::Mat::zeros(1080, 1920, CV_8UC3);
                        __last_w = __last_h = -1;
                    }
                    const double __s = std::min(1920.0 / shown.cols, 1080.0 / shown.rows);
                    const int __w = static_cast<int>(std::lround(shown.cols * __s));
                    const int __h = static_cast<int>(std::lround(shown.rows * __s));
                    // [prescale3] reuse the destination: allocating a fresh 3.1 MB
                    // Mat每 frame causes repeated page faults (draw was 13.8 ms).
                    static cv::Mat __tmp;
                    cv::resize(shown, __tmp, cv::Size(__w, __h), 0, 0, cv::INTER_LINEAR);
                    // [prescale2] clear only when the band geometry changes; the
                    // letterbox areas are static, so per-frame clearing was 8 MB of wasted
                    // memory traffic (measured 19 ms in round 3).
                    if (__w != __last_w || __h != __last_h) {
                        __canvas.setTo(0);
                        __last_w = __w;
                        __last_h = __h;
                    }
                    __tmp.copyTo(__canvas(cv::Rect((1920 - __w) / 2, (1080 - __h) / 2, __w, __h)));
                    shown = __canvas;
                }
                cv::putText(shown, hud, cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, .9, cv::Scalar(0, 255, 255), 2);
                const auto __ims0 = std::chrono::steady_clock::now();
                if (g_disp_thread) {
                    g_disp_publisher.publish(shown);
                } else if (g_qt_channel) {
                    const bool accepted = g_qt_channel->publish(
                        p3_main_gui::owned_snapshot(shown, decision, index + 1),
                        index + 1, index + 1);
                    const auto status = g_qt_channel->status();
                    if (status.failed)
                        throw std::runtime_error("main GUI publication failed: " + status.error);
                    stop = !accepted || status.stop_requested;
                } else {
                    cv::imshow("FPGA Heterogeneous Vision", shown);
                    const int key = cv::pollKey();
                    ++g_disp_sync_completed;
                    stop = key == 'q' || key == 'Q' || key == 27;
                }
                g_disp_imshow_ms += std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - __ims0).count();
                }
                g_disp_total_ms += std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - __disp0).count();
                if (++g_disp_frames % 30 == 0) {
                    std::printf(g_qt_channel ? "[disp-producer] frames=%d total=%.1f ms clone_publish=%.1f ms draw=%.1f ms\n" :
                                "[disp] frames=%d total=%.1f ms imshow=%.1f ms draw=%.1f ms\n",
                                g_disp_frames, g_disp_total_ms / 30.0,
                                g_disp_imshow_ms / 30.0,
                                (g_disp_total_ms - g_disp_imshow_ms) / 30.0);
                    g_disp_total_ms = 0.0;
                    g_disp_imshow_ms = 0.0;
                    // [disp-drop] how many published frames the worker never showed
                    if (g_disp_thread) {
                        const unsigned long pub = g_disp_publisher.published();
                        const unsigned long shw = g_disp_publisher.shown();
                        std::printf("[disp] published=%lu shown=%lu dropped=%lu\n",
                                    pub, shw, pub > shw ? pub - shw : 0UL);
                    } else if (g_qt_channel) {
                        const auto status = g_qt_channel->status();
                        std::printf("[disp-main-actor] published=%lu completed=%lu published_seq=%lu shown_seq=%lu coalesced=%lu\n",
                                    status.published, status.shown, status.published_seq,
                                    status.shown_seq, status.coalesced);
                    } else {
                        std::printf("[disp-sync] completed=%lu\n", g_disp_sync_completed);
                    }
                }
            }
            t.frame_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - frame_start).count();
            if (const char* __split_path = std::getenv("P3_FRAME_SPLIT")) {
                static FILE* __split_file = std::fopen(__split_path, "a");
                if (__split_file)
                    std::fprintf(__split_file,
                                 "%u,%.2f,%.2f,%.2f,%.2f,%.2f\n", index + 1,
                                 t.preprocess_ms, __split_eng_ms,
                                 __split_join_ms, __split_dec_ms, t.frame_ms);
            }
            timing.push_back(t);
            // [T01] 有界批量写盘：旧实现整场累积在内存（无限增长）。
            // 每 256 帧落盘一次并清空；CSV 语义与旧实现一致（append）。
            constexpr std::size_t kFlushBatch = 256;
            if (timing.size() >= kFlushBatch) {
                const std::size_t start = index + 1 - timing.size();
                append_raw_csv(o, timing, start);
                append_perf_frames_v2(o, timing, start, run_id);
                for (const auto& tv : timing)
                    accumulator.add(tv);
                timing.clear();
            }
            ++index;
            if (g_qt_channel) g_compute_progress_ms.store(steady_ms());
            if (index % 30 == 0)
                std::cout << "\r[Run] frame " << index << " latest FPS " << std::fixed
                          << std::setprecision(1) << (1000.0 / t.frame_ms) << std::flush;
            if (o.ai_mode == 3 && enable_speed && index % 30 == 0) {
                std::printf(
                    "\n[speed] frame=%u kmh=%.1f flow=%.2f inliers=%d "
                    "valid=%d moving=%d\n",
                    index, speed.kmh, speed.flow_px, speed.inliers,
                    speed.valid ? 1 : 0, speed.moving ? 1 : 0);
            }
        }
        if (prepared_prefetcher) {
            prepared_prefetcher->stop();
            prepared_source->rethrow_error(); // failure after the final consumed frame is NOT success
            if (std::fflush(prep_trace.get()) != 0)
                throw std::runtime_error("prep evidence flush failed");
        }
        if (ocr) ocr->wait_idle();
        const double seconds = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - run_start).count();
        const CpuUsage cpu = cpu_meter.stop();
        const OcrStatistics ocr_stats = ocr ? ocr->statistics() : OcrStatistics{};
        {
            const std::size_t start = index >= timing.size() ? index - timing.size() : 0;
            append_raw_csv(o, timing, start);
            append_perf_frames_v2(o, timing, start, run_id);
            for (const auto& tv : timing)
                accumulator.add(tv);
            write_perf_summary_v2(o, accumulator, seconds, run_id);
        }
        append_summary_csv(o, accumulator, cpu, seconds, ocr_stats);
        ocr.reset();
        if (lpr) HLPR_ReleaseContext(lpr);
        // [d2c] Only tear highgui down when nobody else can still be inside it.
        // A worker that had to be detached may still be blocked in imshow;
        // calling destroyAllWindows() from here would be a second GUI owner.
        // [d2d] An independent liveness state: g_disp_thread says which drawing
        // path we chose, not whether a worker still exists.
        if (g_disp_thread) {
            g_disp_publisher.stop();
            g_disp_worker_exited = g_disp_publisher.exited();
        }
        if (o.display && g_disp_worker_exited && !g_qt_channel) cv::destroyAllWindows();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        return 1;
    }
}

int main(int argc, char** argv)
{
    try {
        const Options options = parse_options(argc, argv);
        g_display_on_gui = env_flag("P3_DISPLAY_ON_GUI", true);
        // [AUDIT-L1] Force the lazy config load on the main thread before the
        // compute worker (and its async helper) can race the init flag.
        (void)p3_config();
        const auto backend = p3_display::gui_backend(cv::getBuildInformation());
        const bool use_actor = options.display && options.ai_mode == 3 &&
            !options.ocr_replay && options.input_dir.empty() &&
            backend.compare(0, 2, "QT") == 0 && env_flag("P3_QT_MAIN_ACTOR", true);
        if (!use_actor) return run_application(argc, argv, options);
        std::printf("[qt-main-actor] enabled: GUI=main capture/NPU/Twin/decision/render=single-compute-worker\n");
        g_qt_channel = std::make_shared<p3_main_gui::DisplayChannel>();
        p3_main_gui::HighGui gui;
        // [PERF-P3a 2026-10-22] All final-canvas composition on the GUI thread.
        gui.set_composer([](const cv::Mat& base,
                            const p3_main_gui::DisplayFrame& frame) {
            cv::Mat canvas = base.clone();
            if (frame.y_offset > 0 && g_gui_font) {
                // [AUDIT-H1] Use the worker's speed readout; the UFLD lane
                // line stays worker-only (UFLD is off in the shipped config).
                RoadSpeedEstimator::Estimate speed;
                speed.kmh = frame.speed_kmh;
                speed.valid = frame.speed_valid;
                draw_lane_status(canvas, frame.decision, g_gui_font, speed,
                                 nullptr);
                if (decision_osd::enabled()) {
                    decision_osd::Context ctx;
                    ctx.bridge_connected = frame.bridge_connected;
                    ctx.output_valid = frame.bridge_connected;
                    decision_osd::FreeTypeTextSink text(g_gui_font);
                    decision_osd::Style style;
                    style.safe_area =
                        decision_osd::canvas_band_safe_area(canvas, frame.y_offset);
                    {
                        const int bw = std::min(canvas.cols / 2, 460);
                        style.grant_badge_area = cv::Rect(
                            canvas.cols - bw - 8, 6, bw,
                            std::max(36, canvas.rows / 14));
                    }
                    decision_osd::draw(canvas, frame.decision, ctx, style, text);
                }
            }
            static cv::Mat prescale_canvas;
            static cv::Mat resized;
            static int last_w = -1, last_h = -1;
            static const bool prescale = [] {
                const char* v = std::getenv("P3_PRESCALE");
                return v && v[0] != 0 && v[0] != '0' && v[0] != 'f';
            }();
            cv::Mat out = canvas;
            if (prescale) {
                if (prescale_canvas.rows != 1080 || prescale_canvas.cols != 1920) {
                    prescale_canvas = cv::Mat::zeros(1080, 1920, CV_8UC3);
                    last_w = last_h = -1;
                }
                const double s =
                    std::min(1920.0 / canvas.cols, 1080.0 / canvas.rows);
                const int w = static_cast<int>(std::lround(canvas.cols * s));
                const int h = static_cast<int>(std::lround(canvas.rows * s));
                cv::resize(canvas, resized, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);
                if (w != last_w || h != last_h) {
                    prescale_canvas.setTo(0);
                    last_w = w;
                    last_h = h;
                }
                resized.copyTo(prescale_canvas(
                    cv::Rect((1920 - w) / 2, (1080 - h) / 2, w, h)));
                out = prescale_canvas;
            }
            if (!frame.hud.empty()) {
                cv::putText(out, frame.hud, cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX, .9,
                            cv::Scalar(0, 255, 255), 2);
            }
            return out;
        });
        std::promise<int> completed;
        auto returned = completed.get_future();
        g_compute_progress_ms.store(steady_ms());
        std::thread compute([&] {
            int result = 1;
            try { result = run_application(argc, argv, options); }
            catch (...) { std::fprintf(stderr, "[qt-main-actor] uncaught compute failure\n"); }
            g_qt_channel->worker_done();
            // Ready only at actual thread exit (including TLS destruction).
            completed.set_value_at_thread_exit(result);
        });
        bool exit_unconfirmed_reported = false;
        long long stop_at = 0;
        while (true) {
            g_qt_channel->pump(gui); // all real HighGUI calls stay on main
            const auto status = g_qt_channel->status();
            const bool exited = returned.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
            if (exited && (status.gui_exited || (status.failed && status.close_attempted))) break;
            const auto now = steady_ms();
            if (now - g_compute_progress_ms.load() > 5000 && !status.stop_requested) {
                std::fprintf(stderr, "[compute-health] no compute progress for 5000ms; stop requested\n");
                g_qt_channel->request_stop();
                g_disp_fault_reported = true;
            }
            if (status.stop_requested && !exited) {
                if (!stop_at) stop_at = now;
                if (now - stop_at > 3000 && !exit_unconfirmed_reported) {
                    std::fprintf(stderr, "[compute-health] exit UNCONFIRMED; no join/fallback; external supervisor must terminate\n");
                    exit_unconfirmed_reported = true;
                }
            }
            // Never unconditionally join a possibly blocked compute worker.
            // Keep owned state alive; bounded external supervisor handles a hang.
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        compute.join(); // the at-thread-exit future above confirmed actual return
        const int result = returned.get();
        const auto status = g_qt_channel->status();
        std::printf("[qt-main-actor] compute_exit=%d gui_exit_confirmed=%d published=%lu completed=%lu coalesced=%lu\n",
                    result, status.gui_exited, status.published, status.shown, status.coalesced);
        return (result || status.failed || !status.gui_exited ||
                g_disp_fault_reported) ? 1 : 0;
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        return 1;
    }
}
