#include "ufld_lane_model.h"

#include "rknn_api.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {
constexpr int kInputWidth = 1600;
constexpr int kInputHeight = 320;
constexpr int kCutHeight = 144;

double ms_since(const std::chrono::steady_clock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - start)
        .count();
}

int argmax(const float* values, int count)
{
    int best = 0;
    for (int index = 1; index < count; ++index) {
        if (values[index] > values[best]) best = index;
    }
    return best;
}
}  // namespace

bool UfldLaneModel::init(const std::string& model_path)
{
    FILE* file = std::fopen(model_path.c_str(), "rb");
    if (!file) {
        std::printf("[ufld] open failed: %s\n", model_path.c_str());
        return false;
    }
    std::fseek(file, 0, SEEK_END);
    const long size = std::ftell(file);
    std::fseek(file, 0, SEEK_SET);
    std::vector<unsigned char> blob(static_cast<std::size_t>(size));
    if (std::fread(blob.data(), 1, blob.size(), file) != blob.size()) {
        std::fclose(file);
        return false;
    }
    std::fclose(file);

    rknn_context context = 0;
    if (rknn_init(&context, blob.data(), blob.size(), 0, nullptr) != RKNN_SUCC) {
        std::printf("[ufld] rknn_init failed\n");
        return false;
    }
    context_ = reinterpret_cast<void*>(context);

    rknn_input_output_num io_num{};
    rknn_query(context, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    output_count_ = std::min<int>(io_num.n_output, outputs_.size());
    for (int index = 0; index < output_count_; ++index) {
        rknn_tensor_attr attr{};
        attr.index = index;
        rknn_query(context, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr));
        outputs_[index].channels = attr.dims[1];
        outputs_[index].height = attr.dims[2];
        outputs_[index].width = attr.dims[3];
    }
    left_history_.assign(9, UfldLaneState::None);
    right_history_.assign(9, UfldLaneState::None);
    ready_ = true;
    std::printf("[ufld] engine init OK outputs=%d\n", output_count_);
    return true;
}

std::vector<cv::Point> UfldLaneModel::decode_side(
    const float* loc_row, const float* exist_row, int lane_index,
    int grid_count, int row_count, int frame_width, int frame_height,
    int cut_height)
{
    std::vector<cv::Point> points;
    points.reserve(row_count);
    const float* exist_side = exist_row + lane_index;
    for (int row = 0; row < row_count; ++row) {
        const float absent = exist_side[(0 * row_count + row) * 4];
        const float present = exist_side[(1 * row_count + row) * 4];
        if (present <= absent) continue;

        const float* row_scores =
            loc_row + static_cast<std::size_t>(row) * 4 + lane_index;
        int best_grid = 0;
        float best_score = row_scores[0];
        for (int grid = 1; grid < grid_count; ++grid) {
            const float score =
                row_scores[static_cast<std::size_t>(grid) * row_count * 4];
            if (score > best_score) {
                best_score = score;
                best_grid = grid;
            }
        }

        const int lo = std::max(0, best_grid - 1);
        const int hi = std::min(grid_count - 1, best_grid + 1);
        float weights[3]{};
        float max_score = -1.0e9f;
        for (int grid = lo; grid <= hi; ++grid) {
            weights[grid - lo] =
                row_scores[static_cast<std::size_t>(grid) * row_count * 4];
            max_score = std::max(max_score, weights[grid - lo]);
        }
        float sum = 0.0f;
        float weighted = 0.0f;
        for (int grid = lo; grid <= hi; ++grid) {
            const float value = std::exp(weights[grid - lo] - max_score);
            sum += value;
            weighted += value * grid;
        }
        const float normalized =
            (sum > 0.0f ? weighted / sum : static_cast<float>(best_grid) + 0.5f) /
            static_cast<float>(grid_count - 1);
        const float row_anchor =
            0.42f + 0.58f * static_cast<float>(row) /
                        static_cast<float>(row_count - 1);
        const int x = static_cast<int>(std::lround(
            normalized * static_cast<float>(frame_width - 1)));
        const int y = static_cast<int>(std::lround(
            cut_height + row_anchor * static_cast<float>(frame_height - cut_height)));
        points.emplace_back(std::clamp(x, 0, frame_width - 1),
                            std::clamp(y, 0, frame_height - 1));
    }
    return points;
}

void UfldLaneModel::classify(const cv::Mat& gray,
                             const std::vector<cv::Point>& points,
                             UfldLaneState& state, float& coverage)
{
    if (points.size() < 6) {
        state = UfldLaneState::None;
        coverage = 0.0f;
        return;
    }

    int support = 0;
    int total = 0;
    int max_gap = 0;
    int gap = 0;
    for (std::size_t index = 0; index < points.size(); index += 2) {
        const int x = std::clamp(points[index].x, 2, gray.cols - 3);
        const int y = std::clamp(points[index].y, 2, gray.rows - 3);
        double best_brightness = 0.0;
        for (int delta_x = -6; delta_x <= 6; delta_x += 2) {
            const int sample_x =
                std::clamp(x + delta_x, 2, gray.cols - 3);
            const cv::Rect patch_rect(sample_x - 2, y - 2, 5, 5);
            best_brightness =
                std::max(best_brightness, cv::mean(gray(patch_rect))[0]);
        }
        const bool bright = best_brightness >= 85.0;
        ++total;
        if (bright) {
            ++support;
            gap = 0;
        } else {
            ++gap;
            max_gap = std::max(max_gap, gap);
        }
    }

    coverage = total ? static_cast<float>(support) / total : 0.0f;
    int bright_segments = 0;
    bool previous_bright = false;
    for (std::size_t index = 0; index < points.size(); index += 2) {
        const int x = std::clamp(points[index].x, 2, gray.cols - 3);
        const int y = std::clamp(points[index].y, 2, gray.rows - 3);
        double best_brightness = 0.0;
        for (int delta_x = -6; delta_x <= 6; delta_x += 2) {
            const int sample_x =
                std::clamp(x + delta_x, 2, gray.cols - 3);
            best_brightness =
                std::max(best_brightness,
                         cv::mean(gray(cv::Rect(sample_x - 2, y - 2, 5, 5)))[0]);
        }
        const bool bright = best_brightness >= 85.0;
        if (bright && !previous_bright)
            ++bright_segments;
        previous_bright = bright;
    }

    if (coverage < 0.08f) {
        state = UfldLaneState::None;
    } else if (coverage >= 0.55f && max_gap <= 2) {
        state = UfldLaneState::Solid;
    } else if (bright_segments >= 2 && max_gap >= 3) {
        state = UfldLaneState::Dashed;
    } else if (coverage >= 0.35f) {
        state = UfldLaneState::Solid;
    } else if (bright_segments >= 2) {
        state = UfldLaneState::Dashed;
    } else {
        state = UfldLaneState::None;
    }
}

UfldLaneState UfldLaneModel::vote(std::deque<UfldLaneState>& history,
                                  UfldLaneState raw, UfldLaneState previous)
{
    history.push_back(raw);
    while (history.size() > 9) history.pop_front();
    int counts[3]{};
    for (const auto value : history)
        ++counts[static_cast<int>(value)];
    if (counts[static_cast<int>(UfldLaneState::None)] >= 8)
        return UfldLaneState::None;
    if (counts[static_cast<int>(UfldLaneState::Solid)] >= 4 &&
        counts[static_cast<int>(UfldLaneState::Solid)] >
            counts[static_cast<int>(UfldLaneState::Dashed)])
        return UfldLaneState::Solid;
    if (counts[static_cast<int>(UfldLaneState::Dashed)] >= 4 &&
        counts[static_cast<int>(UfldLaneState::Dashed)] >=
            counts[static_cast<int>(UfldLaneState::Solid)])
        return UfldLaneState::Dashed;
    if (previous != UfldLaneState::None)
        return previous;
    return counts[static_cast<int>(UfldLaneState::Solid)] >
                   counts[static_cast<int>(UfldLaneState::Dashed)]
               ? UfldLaneState::Solid
               : UfldLaneState::Dashed;
}

bool UfldLaneModel::update(const cv::Mat& sbs_bgr)
{
    if (!ready_ || sbs_bgr.empty() || sbs_bgr.cols < 2 || sbs_bgr.rows < kCutHeight + 8)
        return false;

    const auto start = std::chrono::steady_clock::now();
    cv::Mat left = sbs_bgr(cv::Rect(0, 0, sbs_bgr.cols / 2, sbs_bgr.rows));
    cv::Mat rgb;
    cv::cvtColor(left, rgb, cv::COLOR_BGR2RGB);
    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(kInputWidth, 533), 0, 0,
               cv::INTER_LINEAR);
    cv::Mat input =
        resized(cv::Rect(0, resized.rows - kInputHeight, kInputWidth,
                         kInputHeight))
            .clone();

    rknn_input rknn_in{};
    rknn_in.index = 0;
    rknn_in.type = RKNN_TENSOR_UINT8;
    rknn_in.size = static_cast<std::uint32_t>(input.total() * input.elemSize());
    rknn_in.fmt = RKNN_TENSOR_NHWC;
    rknn_in.buf = input.data;
    if (rknn_inputs_set(reinterpret_cast<rknn_context>(context_), 1, &rknn_in) !=
        RKNN_SUCC)
        return false;
    if (rknn_run(reinterpret_cast<rknn_context>(context_), nullptr) != RKNN_SUCC)
        return false;

    std::vector<rknn_output> outputs(output_count_);
    std::memset(outputs.data(), 0, sizeof(rknn_output) * outputs.size());
    for (int index = 0; index < output_count_; ++index) {
        outputs[index].index = index;
        outputs[index].want_float = 1;
    }
    if (rknn_outputs_get(reinterpret_cast<rknn_context>(context_), output_count_,
                         outputs.data(), nullptr) != RKNN_SUCC)
        return false;

    const float* loc_row = nullptr;
    const float* exist_row = nullptr;
    for (int index = 0; index < output_count_; ++index) {
        const OutputTensor& shape = outputs_[index];
        if (shape.channels == 200) loc_row = static_cast<const float*>(outputs[index].buf);
        if (shape.channels == 2 && shape.height == 72)
            exist_row = static_cast<const float*>(outputs[index].buf);
    }

    if (loc_row && exist_row) {
        cv::Mat gray;
        cv::cvtColor(left, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point> left_points =
            decode_side(loc_row, exist_row, 1, 200, 72, left.cols, left.rows,
                        kCutHeight);
        std::vector<cv::Point> right_points =
            decode_side(loc_row, exist_row, 2, 200, 72, left.cols, left.rows,
                        kCutHeight);

        UfldLaneState left_raw = UfldLaneState::None;
        UfldLaneState right_raw = UfldLaneState::None;
        classify(gray, left_points, left_raw, left_.coverage);
        classify(gray, right_points, right_raw, right_.coverage);
        if (left_points.size() >= 6) {
            left_.points = std::move(left_points);
            left_missing_ = 0;
        } else if (++left_missing_ > 10) {
            left_.points.clear();
        }
        if (right_points.size() >= 6) {
            right_.points = std::move(right_points);
            right_missing_ = 0;
        } else if (++right_missing_ > 10) {
            right_.points.clear();
        }
        left_.state = vote(left_history_, left_raw, left_.state);
        right_.state = vote(right_history_, right_raw, right_.state);
    }

    rknn_outputs_release(reinterpret_cast<rknn_context>(context_), output_count_,
                         outputs.data());
    ++frame_index_;
    last_ms_ = ms_since(start);
    return true;
}

void UfldLaneModel::draw(cv::Mat& frame) const
{
    const auto draw_side = [&](const UfldLaneResult& result) {
        if (result.points.size() < 2 ||
            result.state == UfldLaneState::None)
            return;
        const cv::Scalar color =
            result.state == UfldLaneState::Solid
                ? cv::Scalar(255, 80, 255)
                : cv::Scalar(0, 215, 255);
        if (result.state == UfldLaneState::Solid) {
            cv::polylines(frame, std::vector<std::vector<cv::Point>>{result.points},
                          false, color, 2, cv::LINE_AA);
        } else {
            for (std::size_t index = 0; index + 1 < result.points.size(); ++index) {
                if ((index / 2) % 2 == 0)
                    cv::line(frame, result.points[index],
                             result.points[index + 1], color, 2, cv::LINE_AA);
            }
        }
    };
    draw_side(left_);
    draw_side(right_);
}

void UfldLaneModel::release()
{
    if (context_) {
        rknn_destroy(reinterpret_cast<rknn_context>(context_));
        context_ = nullptr;
    }
    ready_ = false;
}
