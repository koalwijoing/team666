// [LANE-KF 2026-10-22] Display-only smoothing of the selected ego-lane pair.
//
// ⚠ STATUS: REJECTED (2026-10-22 A/B). On a real 58-observation lane stream
// the chi-square gate rejected 57/58 updates (the Q/R scales did not match the
// real coefficient magnitudes), so the line FROZE at its first observation
// (right side ~37.7 px off raw). Do NOT enable P3_LANE_SMOOTH=kf without
// recalibrating Q/R. The shipped smoother is One Euro (one_euro.h).
//
// The decision pipeline (TwinliteLaneModel::lane_result_ and the decision
// bridge) is NOT modified: the filter state is consumed only by draw(). The
// state is the quadratic coefficients (c0,c1,c2) of one lane candidate in the
// mask coordinate convention x = c0*z^2 + c1*z + c2, z=(y-208)/68.
//
// Design constraints:
//   * identity transition + small process noise (random walk), so the filter
//     never fabricates motion;
//   * a hard innovation gate (chi-square, 3 dof, 99%) rejects lane swaps,
//     occlusion jumps and mis-fits instead of dragging the line;
//   * reset() on init/release and on a long observation gap, so a stale state
//     cannot survive a real scene change;
//   * no time/TTL logic here: freshness stays in the existing source-time
//     contract of the model/bridge.
#pragma once

#include <array>
#include <cmath>
#include <cstdio>

class LaneKalman {
public:
    using State = std::array<double, 3>;

    void reset()
    {
        initialized_ = false;
        x_ = State{};
        P_ = {};
    }

    bool initialized() const { return initialized_; }
    const State& state() const { return x_; }

    // Returns true when the measurement is accepted, false when gated out.
    bool update(const State& z)
    {
        if (!std::isfinite(z[0]) || !std::isfinite(z[1]) || !std::isfinite(z[2]))
            return false;
        if (!initialized_) {
            x_ = z;
            P_ = {};
            for (int i = 0; i < 3; ++i)
                P_[i][i] = R_[i];
            initialized_ = true;
            return true;
        }

        // Predict: identity transition, additive process noise.
        for (int i = 0; i < 3; ++i)
            P_[i][i] += Q_[i];

        // Innovation y = z - x.
        State y{};
        for (int i = 0; i < 3; ++i)
            y[i] = z[i] - x_[i];

        // S = P + R (both diagonal here, but keep the full matrix form).
        std::array<std::array<double, 3>, 3> S = P_;
        for (int i = 0; i < 3; ++i)
            S[i][i] += R_[i];

        // Gate: Mahalanobis distance with the full 3x3 inverse.
        const double d2 = mahalanobis2(y, S);
        if (!std::isfinite(d2) || d2 > kGate)
            return false;

        // K = P * S^-1; x += K*y; P = (I-K)*P.
        const auto Sinv = invert3(S);
        if (!Sinv.valid)
            return false;
        std::array<std::array<double, 3>, 3> K{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                double sum = 0.0;
                for (int k = 0; k < 3; ++k)
                    sum += P_[i][k] * Sinv.m[k][j];
                K[i][j] = sum;
            }
        for (int i = 0; i < 3; ++i) {
            double dx = 0.0;
            for (int j = 0; j < 3; ++j)
                dx += K[i][j] * y[j];
            x_[i] += dx;
        }
        std::array<std::array<double, 3>, 3> Pn{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                double sum = 0.0;
                for (int k = 0; k < 3; ++k) {
                    const double ik = (i == k) ? 1.0 : 0.0;
                    sum += (ik - K[i][k]) * P_[k][j];
                }
                Pn[i][j] = sum;
            }
        P_ = Pn;
        return true;
    }

private:
    struct Mat3 {
        std::array<std::array<double, 3>, 3> m{};
        bool valid = false;
    };

    static double mahalanobis2(const State& y,
                               const std::array<std::array<double, 3>, 3>& S)
    {
        const auto inv = invert3(S);
        if (!inv.valid)
            return std::nan("");
        double d2 = 0.0;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                d2 += y[i] * inv.m[i][j] * y[j];
        return d2;
    }

    static Mat3 invert3(std::array<std::array<double, 3>, 3> a)
    {
        Mat3 out;
        double det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
                     a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                     a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
        if (!std::isfinite(det) || std::abs(det) < 1e-12)
            return out;
        const double id = 1.0 / det;
        out.m[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * id;
        out.m[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * id;
        out.m[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * id;
        out.m[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * id;
        out.m[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * id;
        out.m[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * id;
        out.m[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * id;
        out.m[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * id;
        out.m[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * id;
        out.valid = true;
        return out;
    }

    bool initialized_ = false;
    State x_{};
    std::array<std::array<double, 3>, 3> P_{};
    // Coefficients are (c0=curvature, c1=slope, c2=x at z=0). The c2 term is
    // pixel-scale, the c0 term is tiny; the noise must follow that order.
    // Per-update process noise (mask px units), deliberately small.
    static constexpr State Q_{{0.0001, 0.0025, 0.25}};
    // Measurement noise of one candidate fit (mask px units).
    static constexpr State R_{{0.0004, 0.01, 4.0}};
    // Chi-square 3 dof, 99%.
    static constexpr double kGate = 11.34;
};
