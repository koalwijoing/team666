// [PAIR-HYSTERESIS 2026-10-22] Temporal identity hold for the ego-lane pair.
//
// The frozen analyzer re-selects the best geometric pair every frame, and on
// real footage the selected indices change in ~42% of updates. Smoothing
// cannot (and must not) bridge those identity switches, so the remaining
// jitter is dominated by them. This layer keeps the previous pair while it
// still matches the new frame's candidates by geometry; a switch is accepted
// only after kPairMissFrames consecutive failed matches.
//
// It never overrides the analyzer's decision that a valid ego pair exists:
// rematch only runs when the analyzer itself found a pair. The sanity checks
// (ego enclosure, width range) still apply to the matched pair.
#pragma once

#include "twinlite_ego_lane_postprocess.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace lane_pair_hysteresis {
constexpr int kRows = 5;
constexpr int kMinSpan = 32;      // same as the analyzer's kMinTrackSpan
constexpr double kEgoX = 256.0;   // mask center (kMaskWidth / 2)
constexpr double kMatchPx = 12.0; // median |x - anchor| accepted as same lane
}  // namespace lane_pair_hysteresis

struct LanePairAnchor {
    std::array<int, lane_pair_hysteresis::kRows> rows{};
    std::array<double, lane_pair_hysteresis::kRows> left{};
    std::array<double, lane_pair_hysteresis::kRows> right{};
    bool valid = false;
};

inline LanePairAnchor make_pair_anchor(const EgoLanePostprocessResult& result)
{
    using namespace lane_pair_hysteresis;
    LanePairAnchor anchor;
    const int li = result.left_index;
    const int ri = result.right_index;
    const int count = static_cast<int>(result.candidates.size());
    if (li < 0 || ri < 0 || li >= count || ri >= count || li == ri)
        return anchor;
    const auto& left = result.candidates[li].candidate;
    const auto& right = result.candidates[ri].candidate;
    const int lo = std::max(left.y_min, right.y_min);
    const int hi = std::min(left.y_max, right.y_max);
    if (hi - lo < kMinSpan)
        return anchor;
    for (int n = 0; n < kRows; ++n) {
        const int y = lo + (hi - lo) * n / (kRows - 1);
        anchor.rows[n] = y;
        anchor.left[n] = left.x_at(y);
        anchor.right[n] = right.x_at(y);
    }
    anchor.valid = true;
    return anchor;
}

inline bool rematch_pair(const std::vector<EgoLaneCandidateResult>& candidates,
                         const LanePairAnchor& previous,
                         int* left_index, int* right_index)
{
    using namespace lane_pair_hysteresis;
    if (!previous.valid || !left_index || !right_index)
        return false;
    const auto match = [&](const std::array<double, kRows>& anchor,
                           int exclude) {
        int best = -1;
        double best_error = 1e9;
        for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
            if (i == exclude)
                continue;
            const auto& candidate = candidates[i].candidate;
            bool covers = true;
            for (int n = 0; n < kRows; ++n) {
                if (candidate.y_min > previous.rows[n] ||
                    candidate.y_max < previous.rows[n]) {
                    covers = false;
                    break;
                }
            }
            if (!covers)
                continue;
            std::array<double, kRows> errors{};
            for (int n = 0; n < kRows; ++n)
                errors[n] =
                    std::fabs(candidate.x_at(previous.rows[n]) - anchor[n]);
            std::sort(errors.begin(), errors.end());
            const double median = errors[kRows / 2];
            if (median < best_error) {
                best_error = median;
                best = i;
            }
        }
        return (best >= 0 && best_error <= kMatchPx) ? best : -1;
    };
    const int li = match(previous.left, -1);
    if (li < 0)
        return false;
    const int ri = match(previous.right, li);
    if (ri < 0)
        return false;
    const auto& left = candidates[li].candidate;
    const auto& right = candidates[ri].candidate;
    const int lo = std::max(left.y_min, right.y_min);
    const int hi = std::min(left.y_max, right.y_max);
    if (hi - lo < kMinSpan)
        return false;
    const double xl = left.x_at(hi);
    const double xr = right.x_at(hi);
    if (!(xl < kEgoX && kEgoX < xr))
        return false;
    const double width = xr - xl;
    if (!(55.0 <= width && width <= 300.0))
        return false;
    *left_index = li;
    *right_index = ri;
    return true;
}
