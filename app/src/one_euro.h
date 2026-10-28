// [LANE-OEF 2026-10-22] One Euro filter for the ego-lane geometry.
//
// Scope: ONLY the quadratic geometry coefficients (c0,c1,c2) of the selected
// pair. Marking classes (solid/dashed/unknown), pair validity, scores and
// source times are never touched, so the evidence contract is unchanged.
//
// Why One Euro here: the lane model updates sparsely (every 2nd frame,
// ~3.7-7 Hz). A fixed low-pass either lags during a curve or jitters at
// rest. One Euro adapts the cutoff to the observed coefficient velocity:
// strong smoothing when the geometry is steady, lower lag when it moves.
//
// [CALIBRATION 2026-10-22, 240-frame real stream] Real per-update deltas:
//   |dc0| p90 ~33 (p50 8.8), |dc1| p90 ~32-37 (p50 8.2),
//   |dc2| p90 ~11-15 (p50 3.8-4.3).
// A normalized jump gate at 0.35 fired on nearly every frame (false lane
// switches). The jump gate is therefore ABSOLUTE at ~3x the noise p90:
//   c0 > 100, c1 > 110, c2 > 45 px.
// Measured on the same stream with these settings: continuous stretches keep
// blending (198-211 of 233 transitions) and the blended jitter drops
// 18-32% at beta=0.2 (39% at beta=0).
//
// Safety rules (all reset, never extrapolate):
//   * no measurement -> the caller does not read the filter;
//   * a gap longer than kResetGapS (lost/expired lane) resets to the next
//     observation instead of blending across it;
//   * |dc| above kJumpAbs (a real lane/geometry switch) resets immediately
//     instead of dragging the line across the scene.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

class LaneOneEuro {
public:
    using State = std::array<double, 3>;

    // Typical coefficient magnitudes (p90 of the calibration stream). Used
    // only to normalize the beta term; the jump gate is absolute.
    static constexpr State kScale{{30.0, 130.0, 300.0}};

    static constexpr double kMinCutoff = 0.7;   // Hz, steady-state smoothing
    static constexpr double kBeta = 0.2;        // mild speed adaptation
    static constexpr double kDerivCutoff = 1.0; // Hz, derivative low-pass
    static constexpr double kResetGapS = 0.6;   // lost/expired lane
    static constexpr State kJumpAbs{{100.0, 110.0, 45.0}};

    void reset()
    {
        initialized_ = false;
        x_.fill(0.0);
        dx_.fill(0.0);
    }

    bool initialized() const { return initialized_; }
    const State& state() const { return x_; }

    // Accepts one observed coefficient triple with its real dt (seconds).
    // Returns false for a non-finite observation (state untouched); the
    // caller must only write state() back when this returns true.
    bool update(const State& z, double dt_s)
    {
        for (int i = 0; i < 3; ++i)
            if (!std::isfinite(z[i]))
                return false;
        bool reset = !initialized_ || !(dt_s > 0.0) || dt_s > kResetGapS;
        if (!reset) {
            for (int i = 0; i < 3; ++i) {
                if (std::fabs(z[i] - x_[i]) > kJumpAbs[i]) {
                    reset = true;
                    break;
                }
            }
        }
        if (reset) {
            x_ = z;
            dx_.fill(0.0);
            initialized_ = true;
            return true;
        }

        const double dt = std::clamp(dt_s, 0.02, 0.5);
        for (int i = 0; i < 3; ++i) {
            const double scale = kScale[i];
            const double z_n = z[i] / scale;
            const double x_n = x_[i] / scale;
            const double d_prev = dx_[i] / scale;

            const double d_raw = (z_n - x_n) / dt;
            const double a_d = alpha(kDerivCutoff, dt);
            const double d_n = a_d * d_raw + (1.0 - a_d) * d_prev;

            const double cutoff = kMinCutoff + kBeta * std::fabs(d_n);
            const double a = alpha(cutoff, dt);
            const double x_new = a * z_n + (1.0 - a) * x_n;

            x_[i] = x_new * scale;
            dx_[i] = d_n * scale;
        }
        return true;
    }

private:
    static double alpha(double cutoff, double dt)
    {
        constexpr double kPi = 3.14159265358979323846;
        const double tau = 1.0 / (2.0 * kPi * cutoff);
        return 1.0 / (1.0 + tau / dt);
    }

    bool initialized_ = false;
    State x_{};
    State dx_{};
};
