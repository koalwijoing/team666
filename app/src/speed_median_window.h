// [SPEED-MEDIAN 2026-10-22] Causal median window for the road speed estimate.
//
// Rules (user request 2026-10-22):
//   * at most the last 3 VALID samples (median, not mean, so one optical-flow
//     spike cannot move the reading);
//   * bounded time span: samples older than kMaxSpanS are dropped;
//   * the window is only valid while its newest sample is fresh (< kMaxAgeS);
//     invalid frames are not backfilled with zeros and stale values are not
//     reported as current.
// Header-only and dependency-free so the window logic is unit-testable
// without OpenCV/RKNN.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

class SpeedMedianWindow {
public:
    static constexpr int kCapacity = 3;
    static constexpr double kMaxSpanS = 1.5;
    static constexpr double kMaxAgeS = 0.6;

    void reset() { count_ = 0; }

    void push(double kmh, double t_s)
    {
        if (!std::isfinite(kmh) || !std::isfinite(t_s))
            return;
        // [AUDIT-M5 2026-10-22] Ignore out-of-order samples entirely: a
        // sample older than the newest entry used to be appended as current
        // and corrupt the freshness check in median().
        if (count_ > 0 && t_s < buf_t_[count_ - 1])
            return;
        // Bound the span first, relative to the incoming sample time.
        int kept = 0;
        std::array<double, kCapacity> values{};
        std::array<double, kCapacity> times{};
        for (int i = 0; i < count_; ++i) {
            if (t_s >= buf_t_[i] && t_s - buf_t_[i] <= kMaxSpanS) {
                values[kept] = buf_v_[i];
                times[kept] = buf_t_[i];
                ++kept;
            }
        }
        // Keep the newest kCapacity-1 and append the new sample.
        const int start = kept > kCapacity - 1 ? kept - (kCapacity - 1) : 0;
        int n = 0;
        for (int i = start; i < kept; ++i) {
            buf_v_[n] = values[i];
            buf_t_[n] = times[i];
            ++n;
        }
        buf_v_[n] = kmh;
        buf_t_[n] = t_s;
        count_ = n + 1;
    }

    // True when a median exists and the newest sample is not stale.
    bool median(double now_s, double* out_kmh) const
    {
        if (count_ <= 0 || !std::isfinite(now_s) || !out_kmh)
            return false;
        if (now_s < buf_t_[count_ - 1] ||
            now_s - buf_t_[count_ - 1] > kMaxAgeS)
            return false;
        std::array<double, kCapacity> copy{};
        for (int i = 0; i < count_; ++i)
            copy[i] = buf_v_[i];
        const int middle = count_ / 2;
        std::nth_element(copy.begin(), copy.begin() + middle,
                         copy.begin() + count_);
        const double upper = copy[middle];
        if (count_ % 2)
            *out_kmh = upper;
        else
            *out_kmh = (copy[middle - 1] + upper) * 0.5;
        return true;
    }

private:
    std::array<double, kCapacity> buf_v_{};
    std::array<double, kCapacity> buf_t_{};
    int count_ = 0;
};
