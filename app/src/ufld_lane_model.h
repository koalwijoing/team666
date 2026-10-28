#ifndef UFLD_LANE_MODEL_H
#define UFLD_LANE_MODEL_H

#include <opencv2/opencv.hpp>

#include <array>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

enum class UfldLaneState : int {
    None = 0,
    Dashed = 1,
    Solid = 2,
};

struct UfldLaneResult {
    UfldLaneState state = UfldLaneState::None;
    float coverage = 0.0f;
    std::vector<cv::Point> points;
};

class UfldLaneModel {
public:
    bool init(const std::string& model_path);
    bool update(const cv::Mat& sbs_bgr);
    void draw(cv::Mat& frame) const;
    void release();

    bool ready() const { return ready_; }
    double last_ms() const { return last_ms_; }
    const UfldLaneResult& left() const { return left_; }
    const UfldLaneResult& right() const { return right_; }

private:
    struct OutputTensor {
        int channels = 0;
        int height = 0;
        int width = 0;
    };

    static UfldLaneState vote(std::deque<UfldLaneState>& history,
                              UfldLaneState raw, UfldLaneState previous);
    static void classify(const cv::Mat& gray,
                         const std::vector<cv::Point>& points,
                         UfldLaneState& state, float& coverage);
    static std::vector<cv::Point> decode_side(
        const float* loc_row, const float* exist_row, int lane_index,
        int grid_count, int row_count, int frame_width, int frame_height,
        int cut_height);

    void* context_ = nullptr;
    int output_count_ = 0;
    std::array<OutputTensor, 4> outputs_{};
    bool ready_ = false;
    double last_ms_ = 0.0;
    int frame_index_ = 0;
    UfldLaneResult left_;
    UfldLaneResult right_;
    std::deque<UfldLaneState> left_history_;
    std::deque<UfldLaneState> right_history_;
    int left_missing_ = 0;
    int right_missing_ = 0;
};

#endif  // UFLD_LANE_MODEL_H
