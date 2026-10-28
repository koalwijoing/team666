#ifndef TWINLITE_EGO_LANE_POSTPROCESS_H
#define TWINLITE_EGO_LANE_POSTPROCESS_H

#include <opencv2/opencv.hpp>

#include <array>
#include <string>
#include <vector>

enum class LanePaintType { Dashed, Solid, Unknown };

struct EgoLanePoint {
    int y = 0;
    double x = 0.0;
    int width = 0;
};

struct EgoLaneCandidate {
    // x = coef[0] * z^2 + coef[1] * z + coef[2], z=(y-208)/68.
    std::array<double, 3> coef{};
    std::vector<EgoLanePoint> points;
    int y_min = 0;
    int y_max = 0;
    double rmse = 0.0;
    double score = 0.0;

    double x_at(double y) const;
};

struct EgoLaneCandidateResult {
    EgoLaneCandidate candidate;
    LanePaintType paint_type = LanePaintType::Unknown;
    double coverage = 0.0;
    int max_gap = 0;
    int on_runs = 0;
};

struct EgoLanePostprocessResult {
    std::vector<EgoLaneCandidateResult> candidates;
    int left_index = -1;
    int right_index = -1;
    double pair_score = 0.0;
    std::string reason = "no plausible pair enclosing ego path";
};

EgoLanePostprocessResult analyze_twinlite_ego_lane(const cv::Mat& lane_mask,
                                                    const cv::Mat& left_bgr);
void draw_twinlite_ego_lane(cv::Mat& left_bgr,
                            const EgoLanePostprocessResult& result);
const char* lane_paint_type_name(LanePaintType type);


// R2 relay (single-pass): definition appended in the .cpp next to the frozen
// oracle entry; shared context declared in lane_shared_frontend.h.
namespace lane_shared { struct SharedInputs; }
EgoLanePostprocessResult analyze_twinlite_ego_lane_shared(
    const lane_shared::SharedInputs& shared);

#endif  // TWINLITE_EGO_LANE_POSTPROCESS_H
