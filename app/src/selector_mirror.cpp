// selector_mirror.cpp - exact mirror of the frozen Python oracle
// (selector.py @ 6048b15b...). Line-by-line port; any behavioral difference
// is a defect of this file, not an allowed variation.
#include "selector_mirror.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace twinlite_identity_mirror {
namespace {

// Python round(x, 3) is round-half-even on the decimal representation.
// std::nearbyint uses the current rounding mode (to-nearest-even by default),
// so nearbyint(x * 1000) / 1000 matches for every value whose x*1000 is not
// exactly on a .5 boundary in binary. The comparator declares a 1e-3
// tolerance for this field and also checks the unrounded value at 5e-4.
double py_round3(double v)
{
    return std::nearbyint(v * 1000.0) / 1000.0;
}

int span_of(const MirrorCandidate& c)
{
    return static_cast<int>(c.y_max.i - c.y_min.i);
}

double coef3(const MirrorCandidate& c, int k) { return coef_at(c, k); }

bool valid_candidate(const MirrorCandidate& c)
{
    if (!c.is_object) return false;
    if (!c.coef_is_array || c.coef_len != 3) return false;
    for (int k = 0; k < 3; ++k) {
        const JNum& v = c.coef[k];
        if (v.kind == NumKind::Int) continue;  // int is Real and finite
        if (v.kind == NumKind::Real) {
            if (!std::isfinite(v.d)) return false;
            continue;
        }
        return false;  // bool / missing / non-number
    }
    const auto integral = [](const JNum& v) { return v.kind == NumKind::Int; };
    if (!integral(c.y_min) || !integral(c.y_max) ||
        !integral(c.domain_first_y) || !integral(c.domain_last_y) ||
        !integral(c.paintable_pool)) {
        return false;
    }
    return 0 <= c.y_min.i && c.y_min.i <= c.y_max.i && c.y_max.i < 288 &&
           0 <= c.domain_first_y.i && c.domain_first_y.i <= c.domain_last_y.i &&
           c.domain_last_y.i < 360 && c.paintable_pool.i >= 0;
}

struct MergePlan {
    std::vector<std::pair<double, int>> kept;
    std::vector<MergeEntry> merged;
};

MergePlan merge_plan(const std::vector<std::pair<double, int>>& side,
                     const std::vector<MirrorCandidate>& candidates)
{
    // R1 representatives: not directly shadowed by any strictly longer
    // same-line candidate on the same side (no transitive chains).
    std::vector<int> reps;
    for (const auto& item : side) {
        const int i = item.second;
        bool shadowed = false;
        for (const auto& other : side) {
            const int j = other.second;
            if (j == i) continue;
            if (span_of(candidates[j]) <= span_of(candidates[i])) continue;
            if (same_line_direct(i, j, candidates, nullptr)) {
                shadowed = true;
                break;
            }
        }
        if (!shadowed) reps.push_back(i);
    }
    MergePlan plan;
    for (const auto& item : side) {
        const std::pair<double, int> xy = item;
        const int i = item.second;
        struct Target {
            int j;
            double d;
            int n;
            int lo;
            int hi;
        };
        std::vector<Target> targets;
        for (const int j : reps) {
            if (j == i) continue;
            if (span_of(candidates[j]) <= span_of(candidates[i])) continue;
            SameLineOutcome out;
            if (same_line_direct(i, j, candidates, &out)) {
                targets.push_back(Target{j, out.max_dx, out.shared_rows,
                                         out.common_lo, out.common_hi});
            }
        }
        if (targets.empty()) {
            plan.kept.push_back(xy);
            continue;
        }
        bool ambiguous = false;
        for (std::size_t m = 0; m < targets.size() && !ambiguous; ++m) {
            for (std::size_t k = m + 1; k < targets.size(); ++k) {
                if (!same_line_direct(targets[m].j, targets[k].j, candidates,
                                      nullptr)) {
                    ambiguous = true;
                    break;
                }
            }
        }
        if (ambiguous) {
            plan.kept.push_back(xy);
            continue;
        }
        // Python max on ties keeps the first maximal element.
        const Target* best = &targets[0];
        for (std::size_t t = 1; t < targets.size(); ++t) {
            if (span_of(candidates[targets[t].j]) >
                span_of(candidates[best->j])) {
                best = &targets[t];
            }
        }
        MergeEntry entry;
        entry.index = i;
        entry.same_line_of = best->j;
        entry.max_dx = py_round3(best->d);
        entry.max_dx_raw = best->d;
        entry.shared_rows = best->n;
        entry.common_lo = best->lo;
        entry.common_hi = best->hi;
        plan.merged.push_back(entry);
    }
    return plan;
}

bool tied_at(const std::vector<std::pair<double, int>>& side, std::size_t position)
{
    for (const int delta : {-1, 1}) {
        const long long p = static_cast<long long>(position) + delta;
        if (p >= 0 && p < static_cast<long long>(side.size())) {
            if (std::fabs(side[position].first - side[static_cast<std::size_t>(p)].first) <=
                NUMERIC_EPS) {
                return true;
            }
        }
    }
    return false;
}

} // namespace

double coef_at(const MirrorCandidate& c, int k)
{
    const JNum& v = c.coef[k];
    return v.kind == NumKind::Int ? static_cast<double>(v.i) : v.d;
}

double x_mask(const MirrorCandidate& c, double mask_y)
{
    const double z = (mask_y - 208.0) / 68.0;
    const double a = coef3(c, 0);
    const double b = coef3(c, 1);
    const double cc = coef3(c, 2);
    return a * z * z + b * z + cc;
}

bool pair_geometry_ok(const MirrorCandidate& left, const MirrorCandidate& right)
{
    // Exactly the frozen candidate geometry gates; no new threshold.
    const int lo = std::max(static_cast<int>(left.y_min.i),
                            static_cast<int>(right.y_min.i));
    const int hi = std::min(static_cast<int>(left.y_max.i),
                            static_cast<int>(right.y_max.i));
    if (hi - lo < 32) return false;
    const double xl = x_mask(left, static_cast<double>(hi));
    const double xr = x_mask(right, static_cast<double>(hi));
    const double wb = xr - xl;
    if (!(55.0 <= wb && wb <= 300.0)) return false;
    if (!(xl < EGO_X_MASK && EGO_X_MASK < xr)) return false;
    if (std::fabs((xl + xr) / 2.0 - EGO_X_MASK) / std::max(wb, 1.0) > 0.40) {
        return false;
    }
    int contains = 0;
    bool has_previous = false;
    double previous = 0.0;
    for (int n = 0; n < 9; ++n) {
        const double y = static_cast<double>(lo) +
                         (static_cast<double>(hi - lo) * n) / 8.0;
        const double l = x_mask(left, y);
        const double r = x_mask(right, y);
        const double width = r - l;
        if (!(18.0 <= width && width <= 340.0)) return false;
        if (has_previous && width - previous < -std::max(8.0, wb * 0.12)) {
            return false;
        }
        previous = width;
        has_previous = true;
        if (l < EGO_X_MASK && EGO_X_MASK < r) ++contains;
    }
    return contains >= 7;
}

bool common_measured_rows(const MirrorCandidate& a, const MirrorCandidate& b,
                          int* lo, int* hi)
{
    const int a_lo = std::max(static_cast<int>(a.domain_first_y.i),
                              static_cast<int>(std::ceil(a.y_min.i * MASK_SCALE)));
    const int a_hi = std::min(static_cast<int>(a.domain_last_y.i),
                              static_cast<int>(std::floor(a.y_max.i * MASK_SCALE)));
    const int b_lo = std::max(static_cast<int>(b.domain_first_y.i),
                              static_cast<int>(std::ceil(b.y_min.i * MASK_SCALE)));
    const int b_hi = std::min(static_cast<int>(b.domain_last_y.i),
                              static_cast<int>(std::floor(b.y_max.i * MASK_SCALE)));
    const int out_lo = std::max(a_lo, b_lo);
    const int out_hi = std::min(a_hi, b_hi);
    if (out_lo > out_hi) return false;
    *lo = out_lo;
    *hi = out_hi;
    return true;
}

SameLineOutcome same_line_dx(const MirrorCandidate& a, const MirrorCandidate& b)
{
    SameLineOutcome out;
    int lo = 0;
    int hi = 0;
    if (!common_measured_rows(a, b, &lo, &hi)) {
        return out;  // (None, 0, None)
    }
    const int n = hi - lo + 1;
    out.has_common = true;
    out.common_lo = lo;
    out.common_hi = hi;
    out.shared_rows = n;
    if (n < SAME_LINE_MIN_SHARED) {
        return out;  // (None, n, rng)
    }
    double worst = 0.0;
    double max_pos = 0.0;
    double max_neg = 0.0;
    for (int y = lo; y <= hi; ++y) {
        const double ym = static_cast<double>(y) / MASK_SCALE;
        const double d = x_mask(a, ym) - x_mask(b, ym);
        if (std::fabs(d) > worst) worst = std::fabs(d);
        if (d > max_pos) max_pos = d;
        if (-d > max_neg) max_neg = -d;
    }
    // Two-sided crossing margin: both sides must exceed CROSS_MARGIN_MASK to
    // reject a same-line hypothesis (real duplicate fits wobble one-sided).
    if (max_pos >= CROSS_MARGIN_MASK && max_neg >= CROSS_MARGIN_MASK) {
        return out;  // (None, n, rng)
    }
    out.dx_ok = true;
    out.max_dx = worst;
    return out;
}

bool same_line_direct(int i, int j,
                      const std::vector<MirrorCandidate>& candidates,
                      SameLineOutcome* outcome)
{
    SameLineOutcome out = same_line_dx(candidates[i], candidates[j]);
    if (outcome) *outcome = out;
    return out.dx_ok && out.max_dx <= SAME_LINE_TOL_MASK;
}

OrderStableOutcome order_stable(
    int selected, const std::vector<std::pair<double, int>>& peers,
    const std::vector<MirrorCandidate>& candidates, int ref)
{
    OrderStableOutcome result;
    const MirrorCandidate& a = candidates[static_cast<std::size_t>(selected)];
    for (const auto& peer : peers) {
        const int pi = peer.second;
        if (pi == selected) continue;
        const MirrorCandidate& b = candidates[static_cast<std::size_t>(pi)];
        const long long end = std::min(
            {static_cast<long long>(REFERENCE_CAP_WORK),
             static_cast<long long>(a.domain_last_y.i),
             static_cast<long long>(b.domain_last_y.i),
             static_cast<long long>(std::floor(
                 static_cast<double>(std::min(a.y_max.i, b.y_max.i)) * MASK_SCALE))});
        const double z0 = (static_cast<double>(ref) / MASK_SCALE - 208) / 68.0;
        const double z1 = (static_cast<double>(end) / MASK_SCALE - 208) / 68.0;
        const double d0 = coef3(a, 0) - coef3(b, 0);
        const double d1 = coef3(a, 1) - coef3(b, 1);
        const double d2 = coef3(a, 2) - coef3(b, 2);
        const auto value = [&](double z) { return d0 * z * z + d1 * z + d2; };
        double values[3];
        int nv = 2;
        values[0] = value(z0);
        values[1] = value(z1);
        if (d0 != 0.0) {
            const double vertex = -d1 / (2.0 * d0);
            if (z0 < vertex && vertex < z1) {
                values[nv++] = value(vertex);
            }
        }
        bool all_finite = true;
        for (int k = 0; k < nv; ++k) {
            if (!std::isfinite(values[k])) {
                all_finite = false;
                break;
            }
        }
        if (!all_finite) {
            result.stable = false;
            result.peer = pi;
            return result;
        }
        double mn = values[0];
        double mx = values[0];
        for (int k = 1; k < nv; ++k) {
            mn = std::min(mn, values[k]);
            mx = std::max(mx, values[k]);
        }
        if ((values[0] > NUMERIC_EPS && mn <= NUMERIC_EPS) ||
            (values[0] < -NUMERIC_EPS && mx >= -NUMERIC_EPS) ||
            std::fabs(values[0]) <= NUMERIC_EPS) {
            result.stable = false;
            result.peer = pi;
            return result;
        }
    }
    return result;
}

SelectResult select_pair(const std::vector<MirrorCandidate>& candidates)
{
    // Compare all eligible candidates on ONE row inside ALL measured domains.
    SelectResult result;
    std::vector<int> pool;
    for (std::size_t idx = 0; idx < candidates.size(); ++idx) {
        const MirrorCandidate& c = candidates[idx];
        // Reject invalid declared eligible input instead of dropping it.
        if (!c.is_object || c.pool_eligible.kind != NumKind::Bool) {
            result.reason = "invalid_pool_input";
            return result;
        }
        if (c.pool_eligible.b) {
            if (!valid_candidate(c) || c.paintable_pool.i < MIN_POOL_ROWS) {
                result.reason = "invalid_eligible_candidate";
                return result;
            }
            pool.push_back(static_cast<int>(idx));
        }
    }
    if (pool.empty()) {
        return result;  // "no_pool"
    }
    long long lo = std::numeric_limits<long long>::min();
    long long hi = std::numeric_limits<long long>::max();
    for (const int i : pool) {
        const MirrorCandidate& c = candidates[static_cast<std::size_t>(i)];
        const long long cand_lo = std::max(
            static_cast<long long>(c.domain_first_y.i),
            static_cast<long long>(std::ceil(c.y_min.i * MASK_SCALE)));
        const long long cand_hi = std::min(
            static_cast<long long>(c.domain_last_y.i),
            static_cast<long long>(std::floor(c.y_max.i * MASK_SCALE)));
        lo = std::max(lo, cand_lo);
        hi = std::min(hi, cand_hi);
    }
    result.has_common_domain = true;
    result.common_lo = static_cast<int>(lo);
    result.common_hi = static_cast<int>(hi);
    const long long ref = std::min<long long>(REFERENCE_CAP_WORK, hi);
    if (lo > hi || ref < lo) {
        result.reason = "no_shared_measured_reference";
        return result;
    }
    result.has_reference = true;
    result.reference_y_work = static_cast<int>(ref);
    std::vector<std::pair<double, int>> left;
    std::vector<std::pair<double, int>> right;
    for (const int i : pool) {
        const double value = x_mask(candidates[static_cast<std::size_t>(i)],
                                    static_cast<double>(ref) / MASK_SCALE);
        if (!std::isfinite(value) || !(0.0 <= value && value < 512.0)) {
            result.reason = "reference_outside_image";
            return result;
        }
        result.candidates_at_reference.push_back(
            ReferenceEntry{i, value, value * MASK_SCALE});
        if (std::fabs(value - EGO_X_MASK) <= NUMERIC_EPS) {
            result.reason = "candidate_on_ego_reference";
            return result;
        }
        if (value < EGO_X_MASK) {
            left.emplace_back(value, i);
        } else {
            right.emplace_back(value, i);
        }
    }
    std::stable_sort(left.begin(), left.end(),
                     [](const std::pair<double, int>& a,
                        const std::pair<double, int>& b) {
                         return a.first > b.first;
                     });
    std::stable_sort(right.begin(), right.end(),
                     [](const std::pair<double, int>& a,
                        const std::pair<double, int>& b) {
                         return a.first < b.first;
                     });
    MergePlan left_plan = merge_plan(left, candidates);
    MergePlan right_plan = merge_plan(right, candidates);
    result.same_line_merged = left_plan.merged;
    result.same_line_merged.insert(result.same_line_merged.end(),
                                   right_plan.merged.begin(),
                                   right_plan.merged.end());
    if (left_plan.kept.empty() || right_plan.kept.empty()) {
        result.reason = "no_pair_passes_frozen_geometry";
        return result;
    }
    if (tied_at(left_plan.kept, 0) || tied_at(right_plan.kept, 0)) {
        result.reason = "ambiguous_reference_tie";
        return result;
    }
    const int li = left_plan.kept[0].second;
    const int ri = right_plan.kept[0].second;
    if (!pair_geometry_ok(candidates[static_cast<std::size_t>(li)],
                          candidates[static_cast<std::size_t>(ri)])) {
        result.reason = "nearest_pair_fails_frozen_geometry";
        return result;
    }
    // Opposite-pool peers participate in the ordering check.
    std::vector<std::pair<double, int>> all_peers = left_plan.kept;
    all_peers.insert(all_peers.end(), right_plan.kept.begin(),
                     right_plan.kept.end());
    for (const int selected : {li, ri}) {
        const OrderStableOutcome stable =
            order_stable(selected, all_peers, candidates,
                         static_cast<int>(ref));
        if (!stable.stable) {
            result.reason = "ambiguous_order_in_shared_measurement";
            result.has_ambiguous_order_pair = true;
            result.ambiguous_a = selected;
            result.ambiguous_b = stable.peer;
            return result;
        }
    }
    result.left_index = li;
    result.right_index = ri;
    result.reason = "common_reference_pair_hypothesis";
    result.pair_valid = true;
    result.geometry_gate_passed = true;
    return result;
}

} // namespace twinlite_identity_mirror
