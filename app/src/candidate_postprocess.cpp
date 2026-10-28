// [CAND] derived from official twinlite_ego_lane_postprocess.cpp (SHA b929e148...)
// by make_candidate.py; extraction/tracking/fitting identical; selection+type replaced.
// NOT DEPLOYED.
#include "candidate_postprocess.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <utility>
#include "lane_shared_frontend.h"

namespace cand_observe {

namespace {
constexpr int kMaskWidth = 512;
constexpr int kMaskHeight = 288;
constexpr int kRoiTop = 140;
constexpr int kRoiBottom = 276;
constexpr int kRoiX0 = 24;
constexpr int kRoiX1 = 488;
constexpr int kRowStep = 4;
constexpr int kMaxSegmentWidth = 40;
constexpr int kMinTransverseSpan = 64;
constexpr int kMinTransversePixels = 48;
constexpr double kMinTransverseFill = 0.55;
constexpr int kMinTrackPoints = 9;
constexpr int kMinTrackSpan = 32;
constexpr double kMaxFitRmse = 5.5;
constexpr int kMaxMissedRows = 3;
constexpr int kMaxTypeExtensionRows = 16;
constexpr int kMaxShortCandidateSpan = 64;
constexpr double kEgoX = kMaskWidth / 2.0;

struct Segment {
    int left = 0;
    int right = 0;
    double center = 0.0;
    int width = 0;
};

struct PaintDecision {
    LanePaintType type = LanePaintType::Unknown;
    double coverage = 0.0;
    int max_gap = 0;
    int on_runs = 0;
};
// [CAND] three-state evidence + nearest-pair pooling constants
constexpr int kWhiteMaxSaturation = 60;
constexpr int kMinLocalContrast = 16;
constexpr int kMaximumRawWidth = 12;
constexpr int kLocalSearchRadius = 6;
constexpr int kMinPaintablePool = 18;
constexpr int kMinEvidenceRows = 24;   // type-layer span gate (mirrors python min_paintable)
constexpr int kLongGapRows = 5;

double median(std::vector<double> values)
{
    if (values.empty()) return 0.0;
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const double high = values[middle];
    if (values.size() % 2 != 0) return high;
    std::nth_element(values.begin(), values.begin() + middle - 1,
                     values.begin() + middle);
    return (values[middle - 1] + high) * 0.5;
}

bool solve3(double a[3][4], std::array<double, 3>& answer)
{
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 3; ++row) {
            if (std::abs(a[row][col]) > std::abs(a[pivot][col])) pivot = row;
        }
        if (std::abs(a[pivot][col]) < 1e-12) return false;
        if (pivot != col) {
            for (int k = col; k < 4; ++k) std::swap(a[pivot][k], a[col][k]);
        }
        const double divisor = a[col][col];
        for (int k = col; k < 4; ++k) a[col][k] /= divisor;
        for (int row = 0; row < 3; ++row) {
            if (row == col) continue;
            const double factor = a[row][col];
            for (int k = col; k < 4; ++k) a[row][k] -= factor * a[col][k];
        }
    }
    answer = {a[0][3], a[1][3], a[2][3]};
    return true;
}

bool fit_polynomial(const std::vector<EgoLanePoint>& points,
                    const std::vector<bool>& keep,
                    std::array<double, 3>& coef)
{
    double normal[3][4]{};
    int count = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!keep[i]) continue;
        const double z = (points[i].y - 208.0) / 68.0;
        const double basis[3] = {z * z, z, 1.0};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col)
                normal[row][col] += basis[row] * basis[col];
            normal[row][3] += basis[row] * points[i].x;
        }
        ++count;
    }
    return count >= 3 && solve3(normal, coef);
}

std::vector<Segment> row_segments(const cv::Mat& mask, int y)
{
    const std::uint8_t* row = mask.ptr<std::uint8_t>(y);
    std::vector<int> xs;
    for (int x = kRoiX0; x < kRoiX1; ++x) {
        if (row[x] != 0) xs.push_back(x);
    }
    if (xs.empty()) return {};

    const int span = xs.back() - xs.front() + 1;
    const double fill = static_cast<double>(xs.size()) / std::max(1, span);
    if (span >= kMinTransverseSpan &&
        static_cast<int>(xs.size()) >= kMinTransversePixels &&
        fill >= kMinTransverseFill)
        return {};

    std::vector<Segment> out;
    int first = xs.front();
    int previous = first;
    for (std::size_t i = 1; i <= xs.size(); ++i) {
        if (i < xs.size() && xs[i] - previous <= 2) {
            previous = xs[i];
            continue;
        }
        const int width = previous - first + 1;
        if (width >= 1 && width <= kMaxSegmentWidth)
            out.push_back({first, previous, (first + previous) * 0.5, width});
        if (i < xs.size()) first = previous = xs[i];
    }
    return out;
}

bool same_curve(const EgoLaneCandidate& a, const EgoLaneCandidate& b)
{
    const int lo = std::max(a.y_min, b.y_min);
    const int hi = std::min(a.y_max, b.y_max);
    if (hi - lo < kMinTrackSpan) return false;
    std::vector<double> errors;
    for (int i = 0; i < 9; ++i) {
        const double y = lo + (hi - lo) * (i / 8.0);
        errors.push_back(std::abs(a.x_at(y) - b.x_at(y)));
    }
    return median(errors) <= 6.0;
}

bool fit_track(const std::vector<EgoLanePoint>& points,
               EgoLaneCandidate& candidate)
{
    if (static_cast<int>(points.size()) < kMinTrackPoints) return false;
    std::vector<bool> keep(points.size(), true);
    std::array<double, 3> coef{};
    for (int iteration = 0; iteration < 2; ++iteration) {
        if (std::count(keep.begin(), keep.end(), true) < kMinTrackPoints ||
            !fit_polynomial(points, keep, coef))
            return false;
        std::vector<double> errors;
        errors.reserve(points.size());
        for (const auto& point : points)
            errors.push_back(std::abs(point.x - (coef[0] * std::pow((point.y - 208.0) / 68.0, 2) +
                                                  coef[1] * ((point.y - 208.0) / 68.0) + coef[2])));
        std::vector<double> kept_errors;
        for (std::size_t i = 0; i < errors.size(); ++i)
            if (keep[i]) kept_errors.push_back(errors[i]);
        const double med = median(kept_errors);
        std::vector<double> deviations;
        deviations.reserve(kept_errors.size());
        for (double error : kept_errors) deviations.push_back(std::abs(error - med));
        const double mad = median(deviations);
        const double cutoff = std::max(3.0, med + 3.0 * 1.4826 * mad);
        for (std::size_t i = 0; i < keep.size(); ++i) keep[i] = errors[i] <= cutoff;
    }
    if (std::count(keep.begin(), keep.end(), true) < kMinTrackPoints ||
        !fit_polynomial(points, keep, coef))
        return false;

    std::vector<EgoLanePoint> used;
    double squared = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!keep[i]) continue;
        const double error = points[i].x -
            (coef[0] * std::pow((points[i].y - 208.0) / 68.0, 2) +
             coef[1] * ((points[i].y - 208.0) / 68.0) + coef[2]);
        squared += error * error;
        used.push_back(points[i]);
    }
    const double rmse = std::sqrt(squared / used.size());
    if (rmse > kMaxFitRmse) return false;
    const int y_min = std::min_element(used.begin(), used.end(),
        [](const auto& a, const auto& b) { return a.y < b.y; })->y;
    const int y_max = std::max_element(used.begin(), used.end(),
        [](const auto& a, const auto& b) { return a.y < b.y; })->y;
    const double expected = std::max(1.0, (y_max - y_min) / 4.0 + 1.0);
    const double continuity = std::min(1.0, used.size() / expected);
    candidate.coef = coef;
    candidate.points = std::move(used);
    candidate.y_min = y_min;
    candidate.y_max = y_max;
    candidate.rmse = rmse;
    candidate.score = candidate.points.size() + 4.0 * continuity - 1.5 * rmse;
    return true;
}

std::vector<EgoLaneCandidate> extract_candidates(const cv::Mat& mask)
{
    std::vector<int> rows;
    for (int y = kRoiBottom - 1; y >= kRoiTop; y -= kRowStep) rows.push_back(y);
    std::vector<std::vector<Segment>> segments;
    segments.reserve(rows.size());
    for (int y : rows) segments.push_back(row_segments(mask, y));

    std::vector<EgoLaneCandidate> found;
    for (std::size_t seed_index = 0; seed_index < rows.size(); ++seed_index) {
        for (const Segment& seed : segments[seed_index]) {
            std::vector<EgoLanePoint> points{{rows[seed_index], seed.center, seed.width}};
            std::size_t last_index = seed_index;
            int misses = 0;
            for (std::size_t index = seed_index + 1; index < rows.size(); ++index) {
                const int y = rows[index];
                const int dy = y - points.back().y;
                double slope = 0.0;
                if (points.size() >= 2) {
                    const std::size_t begin = points.size() > 4 ? points.size() - 4 : 0;
                    double mean_y = 0.0, mean_x = 0.0;
                    for (std::size_t i = begin; i < points.size(); ++i) {
                        mean_y += points[i].y;
                        mean_x += points[i].x;
                    }
                    const double count = points.size() - begin;
                    mean_y /= count;
                    mean_x /= count;
                    double numerator = 0.0, denominator = 0.0;
                    for (std::size_t i = begin; i < points.size(); ++i) {
                        numerator += (points[i].y - mean_y) * (points[i].x - mean_x);
                        denominator += (points[i].y - mean_y) * (points[i].y - mean_y);
                    }
                    if (denominator > 0.0) slope = numerator / denominator;
                    slope = std::clamp(slope, -2.5, 2.5);
                }
                const double predicted = points.back().x + slope * dy;
                const std::size_t row_gap = index - last_index;
                const double max_delta = std::min(42.0, 7.0 * row_gap + 5.0);
                const Segment* best = nullptr;
                double best_cost = 0.0;
                for (const Segment& choice : segments[index]) {
                    const double cost = std::abs(choice.center - predicted) +
                        0.20 * std::abs(choice.width - points.back().width);
                    if (!best || cost < best_cost) {
                        best = &choice;
                        best_cost = cost;
                    }
                }
                if (best && std::abs(best->center - predicted) <= max_delta) {
                    points.push_back({y, best->center, best->width});
                    last_index = index;
                    misses = 0;
                    continue;
                }
                if (++misses > kMaxMissedRows) break;
            }
            EgoLaneCandidate candidate;
            if (fit_track(points, candidate) &&
                candidate.y_max - candidate.y_min >= kMinTrackSpan)
                found.push_back(std::move(candidate));
        }
    }

    std::sort(found.begin(), found.end(), [](const auto& a, const auto& b) {
        if (a.score != b.score) return a.score > b.score;
        if (a.rmse != b.rmse) return a.rmse < b.rmse;
        return a.y_max > b.y_max;
    });
    std::vector<EgoLaneCandidate> unique;
    for (auto& candidate : found) {
        if (std::none_of(unique.begin(), unique.end(), [&](const auto& prior) {
                return same_curve(candidate, prior);
            }))
            unique.push_back(std::move(candidate));
    }
    return unique;
}

struct PairChoice {
    int left = -1;
    int right = -1;
    double score = 0.0;
};

// ---------------- [CAND-SEL] nearest-to-ego pairing ----------------
bool candidate_geometry_ok(const EgoLaneCandidate& left, const EgoLaneCandidate& right)
{
    const int lo = std::max(left.y_min, right.y_min);
    const int hi = std::min(left.y_max, right.y_max);
    if (hi - lo < kMinTrackSpan) return false;
    const double xl = left.x_at(hi), xr = right.x_at(hi);
    if (!(xl < xr)) return false;
    const double wb = xr - xl;
    if (!(55.0 <= wb && wb <= 300.0)) return false;
    const double cr = std::abs((xl + xr) * 0.5 - kEgoX) / std::max(wb, 1.0);
    if (cr > 0.40 || !(xl < kEgoX && kEgoX < xr)) return false;
    int contains = 0;
    double prev = 0.0;
    for (int n = 0; n < 9; ++n) {
        const double y = lo + (hi - lo) * (n / 8.0);
        const double w = right.x_at(y) - left.x_at(y);
        if (w < 18.0 || w > 340.0) return false;
        if (n > 0 && w - prev < -std::max(8.0, wb * 0.12)) return false;
        prev = w;
        if (left.x_at(y) < kEgoX && kEgoX < right.x_at(y)) ++contains;
    }
    return contains >= 7;
}

PairChoice select_pair(const std::vector<EgoLaneCandidateResult>& items)
{
    PairChoice choice;
    const double sy = 360.0 / kMaskHeight;
    auto bottom_x = [&](const EgoLaneCandidate& c) {
        const double y_sel = std::min(330.0, 1.25 * (c.y_max - 1));
        return c.x_at(y_sel / sy) * sy / (640.0 / kMaskWidth);   // back to mask x
    };
    std::vector<int> left_pool, right_pool;
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (!items[i].pool_eligible) continue;
        const double bx = bottom_x(items[i].candidate);
        if (bx < kEgoX) left_pool.push_back(static_cast<int>(i));
        else if (bx > kEgoX) right_pool.push_back(static_cast<int>(i));
    }
    std::sort(left_pool.begin(), left_pool.end(),
        [&](int a, int b) { return bottom_x(items[a].candidate) > bottom_x(items[b].candidate); });
    std::sort(right_pool.begin(), right_pool.end(),
        [&](int a, int b) { return bottom_x(items[a].candidate) < bottom_x(items[b].candidate); });
    for (int li : left_pool) {
        for (int ri : right_pool) {
            if (candidate_geometry_ok(items[li].candidate, items[ri].candidate)) {
                choice.left = li;
                choice.right = ri;
                choice.score = 0.0;
                return choice;
            }
        }
    }
    return choice;
}

// ---------------- [CAND-TYPE] three-state evidence (port of research _raw_rows) ----------------

struct EvidenceInfo {
    using State = EgoLaneEvidenceState;
    std::vector<State> rows;    // per work-row (y ascending from first_y)
    int first_y = 0;
    int paintable = 0;
};

EvidenceInfo three_state_scan(const cv::Mat& hsv, const cv::Mat& gray,
                             const EgoLaneCandidate& candidate)
{
    const double sx = 640.0 / kMaskWidth;
    const double sy = 360.0 / kMaskHeight;
    EvidenceInfo ev;
    const int lo = std::max(0, static_cast<int>(std::ceil(candidate.y_min * sy)));
    const int hi = std::min(359, static_cast<int>(std::floor(candidate.y_max * sy)));
    ev.first_y = lo;
    for (int y = lo; y <= hi; ++y) {
        const double pred = candidate.x_at(y / sy) * sx;
        const int x = static_cast<int>(std::nearbyint(pred));
        if (!(x >= 25 && x < 615)) {
            ev.rows.push_back(EvidenceInfo::State::Unobservable);
            continue;
        }
        std::vector<double> bg;
        for (int px = x - 19; px < x - 10; ++px) bg.push_back(gray.at<std::uint8_t>(y, px));
        for (int px = x + 11; px < x + 20; ++px) bg.push_back(gray.at<std::uint8_t>(y, px));
        const double background = median(std::move(bg));
        const int left = x - 6, right = x + 7;
        std::vector<int> indices;
        for (int px = left; px < right; ++px) {
            const cv::Vec3b p = hsv.at<cv::Vec3b>(y, px);
            const int h = p[0], s = p[1], v = p[2];
            const bool color = (s <= kWhiteMaxSaturation && v >= 110) ||
                               (h >= 12 && h <= 42 && s >= 70 && v >= 100);
            const bool bright = gray.at<std::uint8_t>(y, px) >= background + kMinLocalContrast;
            if (color && bright) indices.push_back(px - left);
        }
        std::vector<std::pair<int, int>> runs;
        for (int idx : indices) {
            if (!runs.empty() && idx == runs.back().second + 1) runs.back().second = idx;
            else runs.push_back({idx, idx});
        }
        runs.erase(std::remove_if(runs.begin(), runs.end(), [](const std::pair<int, int>& r) {
            return r.second - r.first + 1 > kMaximumRawWidth;
        }), runs.end());
        bool painted_row = false;
        if (!runs.empty()) {
            const auto best = std::min_element(runs.begin(), runs.end(),
                [&](const std::pair<int, int>& r1, const std::pair<int, int>& r2) {
                    const double c1 = std::abs(left + (r1.first + r1.second) / 2.0 - pred);
                    const double c2 = std::abs(left + (r2.first + r2.second) / 2.0 - pred);
                    if (c1 != c2) return c1 < c2;
                    return r1.first < r2.first;
                });
            double center = left + (best->first + best->second) / 2.0;
            const int ext_left = x - 24, ext_right = x + 25;
            std::vector<char> extended(ext_right - ext_left, 0);
            for (int px = ext_left; px < ext_right; ++px) {
                const cv::Vec3b p = hsv.at<cv::Vec3b>(y, px);
                const int h = p[0], s = p[1], v = p[2];
                const bool color = (s <= kWhiteMaxSaturation && v >= 110) ||
                                   (h >= 12 && h <= 42 && s >= 70 && v >= 100);
                extended[px - ext_left] =
                    (color && (gray.at<std::uint8_t>(y, px) >= background + kMinLocalContrast)) ? 1 : 0;
            }
            int pos = static_cast<int>(std::nearbyint(center)) - ext_left;
            int ea = pos, eb = pos;
            while (ea > 0 && extended[ea - 1]) --ea;
            while (eb + 1 < static_cast<int>(extended.size()) && extended[eb + 1]) ++eb;
            const int real_width = eb - ea + 1;
            if (real_width <= kMaximumRawWidth && ea > 0 &&
                eb < static_cast<int>(extended.size()) - 1) {
                center = ext_left + (ea + eb) / 2.0;
                if (std::abs(center - pred) > kLocalSearchRadius) {
                    ev.rows.push_back(EvidenceInfo::State::Unobservable);
                    continue;
                }
                painted_row = true;
            }
        }
        if (painted_row) {
            ev.rows.push_back(EvidenceInfo::State::Paint);
            ++ev.paintable;
            continue;
        }
        std::vector<double> ss, gg;
        for (int px = left; px < right; ++px) {
            const cv::Vec3b p = hsv.at<cv::Vec3b>(y, px);
            ss.push_back(p[1]);
            gg.push_back(gray.at<std::uint8_t>(y, px));
        }
        const double med_gray = median(std::move(gg));
        const bool clear = med_gray >= 35 && med_gray <= 190 && median(std::move(ss)) < 100 &&
                           indices.empty();
        if (clear) {
            ev.rows.push_back(EvidenceInfo::State::Clear);
            ++ev.paintable;
        } else {
            ev.rows.push_back(EvidenceInfo::State::Unobservable);
        }
    }
    return ev;
}

PaintDecision classify_evidence(const EvidenceInfo& ev, EgoLaneCandidateResult& out)
{
    PaintDecision decision;
    out.domain_rows = static_cast<int>(ev.rows.size());
    out.domain_first_y = ev.rows.empty() ? -1 : ev.first_y;
    out.domain_last_y = ev.rows.empty() ? -1 : ev.first_y + out.domain_rows - 1;
    out.evidence_rows.clear();
    out.evidence_runs.clear();
    for (std::size_t i = 0; i < ev.rows.size(); ++i) {
        const int y = ev.first_y + static_cast<int>(i);
        const auto state = ev.rows[i];
        out.evidence_rows.push_back({y, state});
        if (!out.evidence_runs.empty() && out.evidence_runs.back().state == state)
            out.evidence_runs.back().y_max = y;
        else out.evidence_runs.push_back({state, y, y});
    }
    out.paint_rows = out.paintable_rows = out.unobservable_rows = 0;
    out.long_gaps = out.dash_segments = out.observable_blocks = 0;
    out.qualified_paint_runs = out.repeated_paint_chain = 0;
    int chain = 0;
    const auto length = [](const EgoLaneEvidenceRun& r) { return r.y_max - r.y_min + 1; };
    for (std::size_t i = 0; i < out.evidence_runs.size(); ++i) {
        const auto& run = out.evidence_runs[i];
        const int n = length(run);
        if (run.state != EvidenceInfo::State::Unobservable &&
            (i == 0 || out.evidence_runs[i-1].state == EvidenceInfo::State::Unobservable))
            ++out.observable_blocks;
        if (run.state == EvidenceInfo::State::Paint) {
            out.paint_rows += n;
            out.paintable_rows += n;
            ++out.dash_segments;
            if (n >= 3) {
                ++out.qualified_paint_runs;
                const bool connected = i >= 2 &&
                    out.evidence_runs[i-1].state == EvidenceInfo::State::Clear &&
                    length(out.evidence_runs[i-1]) >= kLongGapRows &&
                    out.evidence_runs[i-2].state == EvidenceInfo::State::Paint &&
                    length(out.evidence_runs[i-2]) >= 3;
                chain = connected ? chain + 1 : 1;
                out.repeated_paint_chain = std::max(out.repeated_paint_chain, chain);
            }
        } else if (run.state == EvidenceInfo::State::Clear) {
            out.paintable_rows += n;
            decision.max_gap = std::max(decision.max_gap, n);
            if (n >= kLongGapRows) ++out.long_gaps;
        } else {
            out.unobservable_rows += n;
        }
    }
    out.paintable_pool = out.paintable_rows;
    out.pool_eligible = out.paintable_pool >= kMinPaintablePool;
    out.type_evidence_sufficient = out.paintable_rows >= kMinEvidenceRows &&
        out.paint_rows >= 6 && out.unobservable_rows == 0;
    decision.on_runs = out.dash_segments;
    decision.coverage = out.paintable_rows ?
        static_cast<double>(out.paint_rows) / out.paintable_rows : 0.0;
    out.type_reason = "insufficient_type_evidence";
    if (out.unobservable_rows) out.type_reason = "unobservable_in_full_domain";
    else if (out.type_evidence_sufficient) {
        out.type_reason = "ambiguous_current_pattern";
        if (decision.coverage >= 0.86 && decision.max_gap <= 2) {
            decision.type = LanePaintType::Solid;
            out.type_reason = "continuous_current_paint";
        } else if (decision.coverage >= 0.30 && out.repeated_paint_chain >= 3) {
            decision.type = LanePaintType::Dashed;
            out.type_reason = "repeated_current_paint_clear_chain";
        }
    }
    return decision;
}
cv::Scalar type_color(LanePaintType type)
{
    switch (type) {
        case LanePaintType::Dashed: return cv::Scalar(240, 32, 160);  // RGB purple
        case LanePaintType::Solid: return cv::Scalar(0, 215, 255);   // RGB yellow
        default: return cv::Scalar(220, 220, 220);
    }
}
}  // namespace

double EgoLaneCandidate::x_at(double y) const
{
    const double z = (y - 208.0) / 68.0;
    return coef[0] * z * z + coef[1] * z + coef[2];
}

const char* lane_paint_type_name(LanePaintType type)
{
    switch (type) {
        case LanePaintType::Dashed: return "DASHED";
        case LanePaintType::Solid: return "SOLID";
        default: return "UNKNOWN";
    }
}

const char* lane_evidence_state_name(EgoLaneEvidenceState state)
{
    switch (state) {
        case EgoLaneEvidenceState::Paint: return "paint";
        case EgoLaneEvidenceState::Clear: return "clear_gap";
        default: return "unobservable";
    }
}

EgoLaneCandidateResult classify_twinlite_evidence(
    const std::vector<EgoLaneEvidenceRow>& rows, int first_y, int last_y)
{
    EvidenceInfo ev;
    ev.first_y = first_y;
    if (!(rows.empty() && first_y == -1 && last_y == -1)) {
        if (first_y > last_y) throw std::invalid_argument("invalid physical evidence domain");
        ev.rows.assign(last_y - first_y + 1, EgoLaneEvidenceState::Unobservable);
        std::vector<bool> seen(ev.rows.size(), false);
        for (const auto& row : rows) {
            if (row.y < first_y || row.y > last_y || seen[row.y-first_y])
                throw std::invalid_argument("duplicate or out-of-domain evidence y");
            if (row.state != EgoLaneEvidenceState::Paint && row.state != EgoLaneEvidenceState::Clear &&
                row.state != EgoLaneEvidenceState::Unobservable)
                throw std::invalid_argument("invalid evidence state");
            seen[row.y-first_y] = true;
            ev.rows[row.y-first_y] = row.state;
        }
    }
    EgoLaneCandidateResult out;
    const PaintDecision decision = classify_evidence(ev, out);
    out.paint_type = decision.type;
    out.coverage = decision.coverage;
    out.max_gap = decision.max_gap;
    out.on_runs = decision.on_runs;
    return out;
}

EgoLaneCandidateResult analyze_twinlite_candidate_evidence(
    const cv::Mat& left_bgr, const EgoLaneCandidate& candidate)
{
    if (left_bgr.empty() || left_bgr.type() != CV_8UC3 || left_bgr.rows != 360 || left_bgr.cols != 640)
        throw std::invalid_argument("expected 640x360 uint8 BGR work frame");
    cv::Mat hsv, gray;
    cv::cvtColor(left_bgr, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(left_bgr, hsv, cv::COLOR_BGR2HSV);
    const EvidenceInfo ev = three_state_scan(hsv, gray, candidate);
    EgoLaneCandidateResult out;
    out.candidate = candidate;
    const PaintDecision decision = classify_evidence(ev, out);
    out.paint_type = decision.type;
    out.coverage = decision.coverage;
    out.max_gap = decision.max_gap;
    out.on_runs = decision.on_runs;
    return out;
}

EgoLanePostprocessResult analyze_twinlite_ego_lane(const cv::Mat& lane_mask,
                                                    const cv::Mat& left_bgr)
{
    EgoLanePostprocessResult result;
    if (lane_mask.empty() || lane_mask.type() != CV_8UC1 ||
        lane_mask.rows != kMaskHeight || lane_mask.cols != kMaskWidth ||
        left_bgr.empty() || left_bgr.type() != CV_8UC3 ||
        left_bgr.rows != 360 || left_bgr.cols != 640)
        return result;

    // Refresh from this call's pixels once. No static state or pointer-as-frame identity.
    cv::Mat hsv, gray;
    cv::cvtColor(left_bgr, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(left_bgr, hsv, cv::COLOR_BGR2HSV);
    std::vector<EgoLaneCandidate> candidates = extract_candidates(lane_mask);
    result.candidates.reserve(candidates.size());
    for (auto& candidate : candidates) {
        EgoLaneCandidateResult item;
        const EvidenceInfo ev = three_state_scan(hsv, gray, candidate);
        item.candidate = candidate;
        PaintDecision decision = classify_evidence(ev, item);
        item.paint_type = decision.type;
        item.coverage = decision.coverage;
        item.max_gap = decision.max_gap;
        item.on_runs = decision.on_runs;
        result.candidates.push_back(std::move(item));
    }
    const PairChoice pair = select_pair(result.candidates);
    if (pair.left >= 0 && pair.right >= 0) {
        result.left_index = pair.left;
        result.right_index = pair.right;
        result.pair_score = pair.score;
        result.reason = "ego pair selected";
    }
    return result;
}

void draw_twinlite_ego_lane(cv::Mat& left_bgr,
                            const EgoLanePostprocessResult& result)
{
    if (left_bgr.empty() || left_bgr.type() != CV_8UC3) return;
    const double sx = static_cast<double>(left_bgr.cols) / kMaskWidth;
    const double sy = static_cast<double>(left_bgr.rows) / kMaskHeight;
    for (std::size_t i = 0; i < result.candidates.size(); ++i) {
        const auto& item = result.candidates[i];
        const bool selected = static_cast<int>(i) == result.left_index ||
                              static_cast<int>(i) == result.right_index;
        const cv::Scalar color = type_color(item.paint_type);
        const int radius = selected
            ? std::max(3, static_cast<int>(std::lround(2.4 * sx)))
            : std::max(1, static_cast<int>(std::lround(1.4 * sx)));
        for (int y = item.candidate.y_min; y <= item.candidate.y_max; y += 2) {
            const int x = static_cast<int>(std::lround(item.candidate.x_at(y) * sx));
            const int yy = static_cast<int>(std::lround(y * sy));
            if (x >= 0 && x < left_bgr.cols && yy >= 0 && yy < left_bgr.rows)
                cv::circle(left_bgr, cv::Point(x, yy), radius, color, -1, cv::LINE_AA);
        }
    }

}


// R2 relay shared-context entry. The pool is converted explicitly
// field-by-field from the lane_shared global types; no casts, no layout
// assumptions; the shared pool itself is not modified.
namespace {

EgoLaneCandidate convert_shared_candidate(const ::EgoLaneCandidate& src)
{
    EgoLaneCandidate candidate;
    candidate.coef = src.coef;
    candidate.points.reserve(src.points.size());
    for (const auto& p : src.points)
        candidate.points.push_back(EgoLanePoint{p.y, p.x, p.width});
    candidate.y_min = src.y_min;
    candidate.y_max = src.y_max;
    candidate.rmse = src.rmse;
    candidate.score = src.score;
    return candidate;
}

}  // namespace

EgoLanePostprocessResult analyze_twinlite_ego_lane_shared(
    const lane_shared::SharedInputs& shared)
{
    EgoLanePostprocessResult result;
    if (!shared.raw_valid || !shared.gray_valid ||
        shared.left.empty() || shared.left.type() != CV_8UC3 ||
        shared.left.rows != 360 || shared.left.cols != 640)
        return result;
    cv::Mat hsv;
    cv::cvtColor(shared.left, hsv, cv::COLOR_BGR2HSV);
    result.candidates.reserve(shared.raw.size());
    for (const ::EgoLaneCandidate& src : shared.raw) {
        EgoLaneCandidate candidate = convert_shared_candidate(src);
        EgoLaneCandidateResult item;
        const EvidenceInfo ev = three_state_scan(hsv, shared.gray, candidate);
        item.candidate = std::move(candidate);
        PaintDecision decision = classify_evidence(ev, item);
        item.paint_type = decision.type;
        item.coverage = decision.coverage;
        item.max_gap = decision.max_gap;
        item.on_runs = decision.on_runs;
        result.candidates.push_back(std::move(item));
    }
    const PairChoice pair = select_pair(result.candidates);
    if (pair.left >= 0 && pair.right >= 0) {
        result.left_index = pair.left;
        result.right_index = pair.right;
        result.pair_score = pair.score;
        result.reason = "ego pair selected";
    }
    return result;
}

}  // namespace cand_observe
