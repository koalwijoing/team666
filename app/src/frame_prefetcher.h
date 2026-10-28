// ============================================================
// frame_prefetcher.h —— 后台采集预取（自 main.cpp 提取，T02-C 修复）
// ============================================================
// 模板化以便用假采集源直接测试真实实现（不复制逻辑）。
// 修复点（T02-C）：
//   1) wait() 选中最新 Ready 槽后，把比它旧且未被占用的 Ready 槽释放回 Free
//      并通知生产者（原实现遗留旧 Ready 槽可被后续消费）。
//   2) 停止等待的谓词在 stopping_/worker_error_ 时即可返回，不依赖只有
//      join 之后才变化的条件（原实现存在通知丢失后永久悬挂的窗口）。
//   3) 记录 last_delivered_capture_seq / 丢弃计数 / 序号倒退计数，供审计。
//   4) 绝不回收 Filling/Busy 槽。
#ifndef P3_FRAME_PREFETCHER_H
#define P3_FRAME_PREFETCHER_H

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

template <typename SourceT>
class FramePrefetcherT {
public:
    using CaptureT = decltype(std::declval<SourceT&>().capture(
        std::declval<std::uint32_t>(), std::declval<std::uint32_t>()));

    struct Lease {
        std::uint32_t buffer_index = 0;
        CaptureT capture;
        double capture_ms = 0.0;
        // [C01 2026-10-21] 采集返回瞬间的主机单调时间（steady ms），在
        // producer 线程、任何预处理之前打点；与 ON 模式的
        // capture_ready_ms 同一时域。不是传感器曝光时间。
        double capture_ready_ms = 0.0;
        // [N1-4] 主机单调交付序号（严格递增，与驱动原始序号解耦）。
        std::uint64_t delivery_id = 0;
    };

    FramePrefetcherT(SourceT& source, std::uint32_t timeout_ms)
        : source_(source), timeout_ms_(timeout_ms),
          states_(source.buffer_count(), SlotState::Free),
          results_(source.buffer_count()),
          capture_ms_(source.buffer_count()),
          capture_ready_ms_(source.buffer_count()),
          ready_generation_(source.buffer_count(), 0)
    {
        if (source.buffer_count() < 2)
            throw std::runtime_error("frame prefetch requires at least two DMA buffers");
        worker_ = std::thread([this] { worker_loop(); });
    }

    ~FramePrefetcherT() { stop(); }

    FramePrefetcherT(const FramePrefetcherT&) = delete;
    FramePrefetcherT& operator=(const FramePrefetcherT&) = delete;

    Lease wait()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_cv_.wait(lock, [this] {
            if (worker_error_)
                return true;
            if (stopping_)
                return true;  // [T02-C] 停止后立即退出等待，不依赖 join 状态
            for (const auto state : states_)
                if (state == SlotState::Ready)
                    return true;
            return false;
        });
        if (worker_error_)
            std::rethrow_exception(worker_error_);

        // [N1-4] 选"生产顺序最新"的 Ready 槽：用 Ready generation（worker 标记
        // Ready 的先后，单线程严格递增）作为排序键——原始驱动序号可能回绕或
        // 源重置，不能用作排序键，否则会把重置前的高序号误选为最新。
        std::uint32_t best = source_.buffer_count();
        std::uint64_t best_generation = 0;
        bool found = false;
        for (std::uint32_t i = 0; i < states_.size(); ++i) {
            if (states_[i] != SlotState::Ready)
                continue;
            if (!found || ready_generation_[i] > best_generation) {
                best = i;
                best_generation = ready_generation_[i];
                found = true;
            }
        }
        if (!found)
            throw std::runtime_error("frame prefetch stopped without a ready frame");

        // 最新帧策略：比 best 旧（generation 更小）、仍处 Ready 的槽释放回 Free。
        bool freed_any = false;
        for (std::uint32_t i = 0; i < states_.size(); ++i) {
            if (i == best || states_[i] != SlotState::Ready)
                continue;
            states_[i] = SlotState::Free;
            ++dropped_ready_;
            freed_any = true;
        }
        if (freed_any)
            free_cv_.notify_one();

        // [N1-4] 主机单调交付 ID：严格递增，与驱动序号解耦（正式交付不回退）。
        ++delivery_id_;
        // 原始序号只作诊断：识别回绕（32 位）与源重置/倒退。
        const std::uint64_t raw = results_[best].sequence;
        if (has_delivered_) {
            const bool wrapped = (last_delivered_raw_sequence_ > 0x80000000ull) &&
                                 (raw < 0x80000000ull);
            if (!wrapped && raw < last_delivered_raw_sequence_)
                ++source_reset_count_;  // 源序号重置/倒退（重建诊断基线）
        }
        last_delivered_raw_sequence_ = raw;
        has_delivered_ = true;

        states_[best] = SlotState::Busy;
        Lease lease;
        lease.buffer_index = best;
        lease.capture = results_[best];
        lease.capture_ms = capture_ms_[best];
        lease.capture_ready_ms = capture_ready_ms_[best];
        lease.delivery_id = delivery_id_;
        return lease;
    }

    void release(std::uint32_t buffer_index)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (buffer_index >= states_.size() || states_[buffer_index] != SlotState::Busy)
                throw std::out_of_range("invalid prefetched buffer release");
            states_[buffer_index] = SlotState::Free;
        }
        free_cv_.notify_one();
    }

    void stop() noexcept
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        free_cv_.notify_all();
        ready_cv_.notify_all();
        if (worker_.joinable())
            worker_.join();
    }

    // ---- 审计计数（T02-C/N1-4 使用） ----
    std::uint64_t dropped_ready_count() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return dropped_ready_;
    }
    // 源序号重置/倒退次数（诊断；正式交付 ID 与之解耦）。
    std::uint64_t source_reset_count() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return source_reset_count_;
    }
    // 最后一次交付的原始驱动序号（仅诊断）。
    std::uint64_t last_delivered_raw_sequence() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return has_delivered_ ? last_delivered_raw_sequence_ : 0;
    }
    // 主机单调交付 ID（最后一次交付）。
    std::uint64_t delivery_id() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return delivery_id_;
    }

private:
    enum class SlotState { Free, Filling, Ready, Busy };

    void worker_loop() noexcept
    {
        try {
            while (true) {
                std::uint32_t buffer_index = 0;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    free_cv_.wait(lock, [this] {
                        if (stopping_)
                            return true;
                        for (const auto state : states_)
                            if (state == SlotState::Free)
                                return true;
                        return false;
                    });
                    if (stopping_)
                        return;

                    for (std::uint32_t i = 0; i < states_.size(); ++i) {
                        if (states_[i] == SlotState::Free) {
                            buffer_index = i;
                            states_[i] = SlotState::Filling;
                            break;
                        }
                    }
                }

                const auto capture_start = std::chrono::steady_clock::now();
                // capture 抛异常时把槽恢复为 Free，避免 Filling 残留。
                CaptureT result = [&]() {
                    try {
                        return source_.capture(buffer_index, timeout_ms_);
                    } catch (...) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        states_[buffer_index] = SlotState::Free;
                        free_cv_.notify_all();
                        throw;
                    }
                }();
                const auto capture_done = std::chrono::steady_clock::now();
                const double capture_ms = std::chrono::duration<double, std::milli>(
                                              capture_done - capture_start)
                                              .count();
                // [C01] 采集返回瞬间（预处理/入队之前）的源时间打点。
                const double capture_ready_ms =
                    std::chrono::duration<double, std::milli>(
                        capture_done.time_since_epoch())
                        .count();

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (stopping_) {
                        states_[buffer_index] = SlotState::Free;
                        return;
                    }
                    results_[buffer_index] = result;
                    capture_ms_[buffer_index] = capture_ms;
                    capture_ready_ms_[buffer_index] = capture_ready_ms;
                    // [N1-4] 记录生产顺序（选"最新帧"的排序键）。
                    ready_generation_[buffer_index] = ++generation_counter_;
                    states_[buffer_index] = SlotState::Ready;
                }
                ready_cv_.notify_one();
            }
        } catch (...) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                worker_error_ = std::current_exception();
                stopping_ = true;
            }
            ready_cv_.notify_all();
        }
    }

    SourceT& source_;
    std::uint32_t timeout_ms_;
    std::vector<SlotState> states_;
    std::vector<CaptureT> results_;
    std::vector<double> capture_ms_;
    // [C01] 每槽最近一次采集返回的源时间（steady ms，producer 打点）。
    std::vector<double> capture_ready_ms_;
    std::thread worker_;
    mutable std::mutex mutex_;
    std::condition_variable free_cv_;
    std::condition_variable ready_cv_;
    std::exception_ptr worker_error_;
    bool stopping_ = false;
    // [N1-4] 交付/审计字段：delivery_id_ 主机单调；ready_generation_ 为每槽
    // 最近一次 Ready 的生产序号（选最新帧的排序键）；原始序号仅诊断。
    std::uint64_t delivery_id_ = 0;
    std::uint64_t generation_counter_ = 0;
    std::vector<std::uint64_t> ready_generation_;
    std::uint64_t last_delivered_raw_sequence_ = 0;
    bool has_delivered_ = false;
    std::uint64_t dropped_ready_ = 0;
    std::uint64_t source_reset_count_ = 0;
};

#endif  // P3_FRAME_PREFETCHER_H
