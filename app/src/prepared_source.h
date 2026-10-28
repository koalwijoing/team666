#pragma once
// Existing FramePrefetcherT owns FREE/FILLING/READY/BUSY. Preparation occurs in
// its one capture producer; a BUSY slot protects BOTH raw and prepared pixels.
// No new per-frame worker, shared_ptr ring or independent NPU/decision owner.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace p3_prep {
inline double mono_ms() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline bool enabled(const char* value) {
    if (!value || std::string(value) == "0") return false;
    if (std::string(value) == "1") return true;
    throw std::invalid_argument("P3_PREP_PIPE must be exactly 0 or 1");
}
struct Timing {
    double preprocess_ms = 0, preprocess_cpu_ms = 0, copy_ms = 0, convert_ms = 0;
};
inline void validate_timing(const Timing& t) {
    for (double n : {t.preprocess_ms, t.preprocess_cpu_ms, t.copy_ms, t.convert_ms})
        if (!std::isfinite(n) || n < 0) throw std::runtime_error("invalid preparation timing");
}

template<class Source, class Buffer>
class SourceAdapter {
public:
    using Raw = decltype(std::declval<Source&>().capture(std::uint32_t{}, std::uint32_t{}));
    struct Capture : Raw {
        // Host capture RETURN before preparation: not sensor exposure time.
        double capture_ready_ms = 0, prepared_ready_ms = 0, capture_wall_ms = 0;
        Timing preparation;
    };
    using Factory = std::function<std::unique_ptr<Buffer>(std::uint32_t)>;
    using Prepare = std::function<Timing(std::uint32_t, Buffer&)>;
    SourceAdapter(Source& source, Factory factory, Prepare prepare)
        : source_(source), prepare_(std::move(prepare)) {
        if (source_.buffer_count() < 2) throw std::invalid_argument("two capture slots required");
        for (std::uint32_t i = 0; i < source_.buffer_count(); ++i) {
            auto buffer = factory(i);
            if (!buffer) throw std::invalid_argument("null prepared buffer");
            buffers_.push_back(std::move(buffer)); // stable pointee: RGA may retain its Mat reference
        }
    }
    std::uint32_t buffer_count() const { return source_.buffer_count(); }
    Capture capture(std::uint32_t index, std::uint32_t timeout) try {
        if (index >= buffers_.size()) throw std::out_of_range("prepared slot");
        Capture result;
        const double begin = mono_ms();
        static_cast<Raw&>(result) = source_.capture(index, timeout);
        result.capture_ready_ms = mono_ms();
        result.capture_wall_ms = result.capture_ready_ms - begin;
        result.preparation = prepare_(index, *buffers_[index]);
        validate_timing(result.preparation);
        result.prepared_ready_ms = mono_ms();
        return result; // exceptions propagate into the existing prefetcher's failure path
    } catch (...) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        error_ = std::current_exception();
        throw;
    }
    void rethrow_error() const {
        std::lock_guard<std::mutex> lock(error_mutex_);
        if (error_) std::rethrow_exception(error_);
    }
    const Buffer& prepared(std::uint32_t index) const { return *buffers_.at(index); }
private:
    Source& source_;
    Prepare prepare_;
    std::vector<std::unique_ptr<Buffer>> buffers_;
    mutable std::mutex error_mutex_;
    std::exception_ptr error_;
};

// Scope covers the entire consumer iteration, including rendering/deep GUI
// snapshot publication. On exceptions, return exactly the acquired BUSY lease.
template<class Prefetcher>
class BusyLease {
public:
    BusyLease() = default;
    BusyLease(const BusyLease&) = delete;
    BusyLease& operator=(const BusyLease&) = delete;
    void acquire(Prefetcher& owner, std::uint32_t index) {
        if (owner_) throw std::logic_error("prepared lease already held");
        owner_ = &owner;
        index_ = index;
    }
    ~BusyLease() noexcept {
        if (owner_) owner_->release(index_); // invariant failure terminates, never silently passes
    }
private:
    Prefetcher* owner_ = nullptr;
    std::uint32_t index_ = 0;
};
}
