// lane_shared_frontend.cpp - see lane_shared_frontend.h.
//
// The extraction helpers below are copied VERBATIM from the frozen production
// source twinlite_ego_lane_postprocess.cpp (b929e148...) — the same bodies the
// repair source candidate_postprocess.cpp (0c2c02df...) carries. Do not "tune"
// anything here: the desktop dual-path gate requires byte-identical pool output.
#include "lane_shared_frontend.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <utility>

namespace lane_shared {

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

struct Segment {
    int left = 0;
    int right = 0;
    double center = 0.0;
    int width = 0;
};

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

}  // namespace

SharedInputs build(const cv::Mat& lane_mask, const cv::Mat& left_bgr)
{
    SharedInputs out;
    // Guard parity with the frozen production entry (b929e148...): extraction
    // only when the mask passed ITS guard; gray only for the production-
    // permissive left-BGR form. The repair branch keeps its own 640x360 check.
    if (!(lane_mask.empty() || lane_mask.type() != CV_8UC1 ||
          lane_mask.rows != kMaskHeight || lane_mask.cols != kMaskWidth)) {
        out.raw = extract_candidates(lane_mask);
        out.raw_valid = true;
    }
    if (!left_bgr.empty() && left_bgr.type() == CV_8UC3) {
        cv::cvtColor(left_bgr, out.gray, cv::COLOR_BGR2GRAY);
        out.left = left_bgr;
        out.gray_valid = true;
    }
    return out;
}

}  // namespace lane_shared
