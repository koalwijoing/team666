#include "twinlite_ego_lane_postprocess.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <utility>
#include "lane_shared_frontend.h"

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
// [DASH-CHAIN 2026-10-21] 虚线断段容忍：原来连续漏 3 个采样行（3*4=12 mask
// px）就断轨，虚线每隔一段就有 10-30px 的空白，导致候选形成率低；实线连续
// 不受影响。放宽到 8 个采样行（32px），由斜率预测 + max_delta 横向门约束，
// 只影响“能不能把同一虚线的多段串起来”，不改变拟合/配对阈值。
constexpr int kMaxMissedRows = 8;
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

PairChoice select_pair(const std::vector<EgoLaneCandidate>& candidates)
{
    PairChoice best;
    bool found = false;
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        for (std::size_t j = i + 1; j < candidates.size(); ++j) {
            const auto& first = candidates[i];
            const auto& second = candidates[j];
            const int lo = std::max(first.y_min, second.y_min);
            const int hi = std::min(first.y_max, second.y_max);
            if (hi - lo < kMinTrackSpan) continue;
            const double x1 = first.x_at(hi), x2 = second.x_at(hi);
            const int left_index = x1 < x2 ? static_cast<int>(i) : static_cast<int>(j);
            const int right_index = x1 < x2 ? static_cast<int>(j) : static_cast<int>(i);
            const double xl = std::min(x1, x2), xr = std::max(x1, x2);
            const double width_bottom = xr - xl;
            if (!(55.0 <= width_bottom && width_bottom <= 300.0)) continue;
            const double center_ratio = std::abs((xl + xr) * 0.5 - kEgoX) /
                                        std::max(width_bottom, 1.0);
            if (center_ratio > 0.40 || !(xl < kEgoX && kEgoX < xr)) continue;

            std::array<double, 9> widths{};
            bool valid = true;
            int contains_center = 0;
            for (int n = 0; n < 9; ++n) {
                const double y = lo + (hi - lo) * (n / 8.0);
                const double left_x = candidates[left_index].x_at(y);
                const double right_x = candidates[right_index].x_at(y);
                widths[n] = right_x - left_x;
                if (widths[n] < 18.0 || widths[n] > 340.0) valid = false;
                if (left_x < kEgoX && kEgoX < right_x) ++contains_center;
            }
            if (!valid || contains_center < 7) continue;
            for (int n = 1; n < 9; ++n)
                if (widths[n] - widths[n - 1] < -std::max(8.0, width_bottom * 0.12))
                    valid = false;
            if (!valid) continue;

            const double top_width = widths.front();
            const double perspective_penalty =
                std::max(0.0, top_width - width_bottom * 1.12);
            const double width_prior_penalty = std::abs(width_bottom - 150.0) / 150.0;
            const double score = 6.0 * (1.0 - center_ratio) +
                0.015 * (candidates[left_index].score + candidates[right_index].score) -
                0.25 * (candidates[left_index].rmse + candidates[right_index].rmse) -
                0.75 * width_prior_penalty - 0.01 * perspective_penalty;
            if (!found || score > best.score) {
                best = {left_index, right_index, score};
                found = true;
            }
        }
    }
    return best;
}

std::vector<bool> sample_paint(const cv::Mat& gray,
                              const EgoLaneCandidate& candidate,
                              int extension_rows)
{
    const double sx = static_cast<double>(gray.cols) / kMaskWidth;
    const double sy = static_cast<double>(gray.rows) / kMaskHeight;
    const int rx = std::max(5, static_cast<int>(std::lround(10 * sx)));
    const int ry = std::max(2, static_cast<int>(std::lround(1.5 * sy)));
    const int y0 = std::max(kRoiTop, candidate.y_min - extension_rows);
    const int y1 = std::min(kRoiBottom - 1, candidate.y_max + extension_rows);
    std::vector<bool> support;
    for (int y = y0; y <= y1; y += 2) {
        const int x = static_cast<int>(std::lround(candidate.x_at(y) * sx));
        const int Y = static_cast<int>(std::lround(y * sy));
        if (x < rx + 8 || x >= gray.cols - rx - 8 || Y < ry || Y >= gray.rows - ry) {
            support.push_back(false);
            continue;
        }
        std::vector<double> patch;
        std::vector<double> background;
        for (int py = Y - ry; py <= Y + ry; ++py) {
            const std::uint8_t* row = gray.ptr<std::uint8_t>(py);
            for (int px = x - rx; px <= x + rx; ++px) patch.push_back(row[px]);
            for (int px = x - rx - 9; px < x - rx - 3; ++px) background.push_back(row[px]);
            for (int px = x + rx + 3; px < x + rx + 9; ++px) background.push_back(row[px]);
        }
        std::sort(patch.begin(), patch.end());
        const double p = 0.9 * (patch.size() - 1);
        const std::size_t lo = static_cast<std::size_t>(std::floor(p));
        const std::size_t hi = static_cast<std::size_t>(std::ceil(p));
        const double p90 = patch[lo] + (patch[hi] - patch[lo]) * (p - lo);
        const double threshold = std::max(128.0, median(std::move(background)) + 24.0);
        support.push_back(p90 >= threshold);
    }
    return support;
}

PaintDecision classify_support(const std::vector<bool>& support)
{
    std::vector<int> indices;
    for (std::size_t i = 0; i < support.size(); ++i)
        if (support[i]) indices.push_back(static_cast<int>(i));
    if (indices.size() < 5) return {};
    const int first = indices.front(), last = indices.back();
    std::vector<bool> visible(support.begin() + first, support.begin() + last + 1);
    if (static_cast<int>(visible.size()) <
        std::max(10, static_cast<int>(support.size() * 0.30)))
        return {};
    int on_runs = 0, max_gap = 0, gap_runs = 0, on_count = 0;
    bool state = visible.front();
    int count = 0;
    for (bool value : visible) {
        if (value == state) {
            ++count;
        } else {
            if (state) {
                ++on_runs;
                on_count += count;
            } else {
                max_gap = std::max(max_gap, count);
                if (count >= 2) ++gap_runs;
            }
            state = value;
            count = 1;
        }
    }
    if (state) {
        ++on_runs;
        on_count += count;
    } else {
        max_gap = std::max(max_gap, count);
        if (count >= 2) ++gap_runs;
    }
    const double coverage = static_cast<double>(on_count) / visible.size();
    PaintDecision result;
    result.coverage = coverage;
    result.max_gap = max_gap;
    result.on_runs = on_runs;
    if (on_runs >= 3 && gap_runs >= 2 && coverage <= 0.82)
        result.type = LanePaintType::Dashed;
    else if (coverage >= 0.86 && max_gap <= 2)
        result.type = LanePaintType::Solid;
    return result;
}

PaintDecision classify_candidate(const cv::Mat& gray,
                                 const EgoLaneCandidate& candidate)
{
    PaintDecision direct = classify_support(sample_paint(gray, candidate, 0));
    if (direct.type != LanePaintType::Unknown ||
        candidate.y_max - candidate.y_min > kMaxShortCandidateSpan)
        return direct;
    PaintDecision extended = classify_support(
        sample_paint(gray, candidate, kMaxTypeExtensionRows));
    return extended.type == LanePaintType::Unknown ? direct : extended;
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

EgoLanePostprocessResult analyze_twinlite_ego_lane(const cv::Mat& lane_mask,
                                                    const cv::Mat& left_bgr)
{
    EgoLanePostprocessResult result;
    if (lane_mask.empty() || lane_mask.type() != CV_8UC1 ||
        lane_mask.rows != kMaskHeight || lane_mask.cols != kMaskWidth ||
        left_bgr.empty() || left_bgr.type() != CV_8UC3)
        return result;

    std::vector<EgoLaneCandidate> candidates = extract_candidates(lane_mask);
    const PairChoice pair = select_pair(candidates);
    if (pair.left >= 0 && pair.right >= 0) {
        result.left_index = pair.left;
        result.right_index = pair.right;
        result.pair_score = pair.score;
        result.reason = "ego pair selected";
    }
    cv::Mat gray;
    cv::cvtColor(left_bgr, gray, cv::COLOR_BGR2GRAY);
    result.candidates.reserve(candidates.size());
    for (auto& candidate : candidates) {
        PaintDecision paint = classify_candidate(gray, candidate);
        result.candidates.push_back({std::move(candidate), paint.type,
                                     paint.coverage, paint.max_gap,
                                     paint.on_runs});
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


EgoLanePostprocessResult analyze_twinlite_ego_lane_shared(
    const lane_shared::SharedInputs& shared)
{
    // R2 relay: identical body to analyze_twinlite_ego_lane above, with the
    // once-per-update shared pool/gray substituted for the inline extraction
    // and BGR2GRAY. The original entry is kept unchanged as the parity oracle.
    EgoLanePostprocessResult result;
    if (!shared.raw_valid || !shared.gray_valid)
        return result;
    std::vector<EgoLaneCandidate> candidates = shared.raw;   // own copy; shared pool stays intact
    const PairChoice pair = select_pair(candidates);
    if (pair.left >= 0 && pair.right >= 0) {
        result.left_index = pair.left;
        result.right_index = pair.right;
        result.pair_score = pair.score;
        result.reason = "ego pair selected";
    }
    result.candidates.reserve(candidates.size());
    for (auto& candidate : candidates) {
        PaintDecision paint = classify_candidate(shared.gray, candidate);
        result.candidates.push_back({std::move(candidate), paint.type,
                                     paint.coverage, paint.max_gap,
                                     paint.on_runs});
    }
    return result;
}
