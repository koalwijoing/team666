#ifndef CANDIDATE_LANE_POSTPROCESS_H
#define CANDIDATE_LANE_POSTPROCESS_H

#include <opencv2/opencv.hpp>

#include <array>
#include <string>
#include <vector>

namespace lane_shared { struct SharedInputs; }

namespace cand_observe {

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

enum class EgoLaneEvidenceState { Paint, Clear, Unobservable };

struct EgoLaneEvidenceRow {
    int y = 0;
    EgoLaneEvidenceState state = EgoLaneEvidenceState::Unobservable;
};

struct EgoLaneEvidenceRun {
    EgoLaneEvidenceState state = EgoLaneEvidenceState::Unobservable;
    int y_min = 0;
    int y_max = 0;
};

struct EgoLaneCandidateResult {
    EgoLaneCandidate candidate;
    LanePaintType paint_type = LanePaintType::Unknown;
    double coverage = 0.0;
    int max_gap = 0;
    int on_runs = 0;
    // [CAND] three-state evidence tallies
    int paint_rows = 0;
    int paintable_rows = 0;
    int long_gaps = 0;
    int dash_segments = 0;
    // Frozen repair contract v1: full candidate scan domain, no endpoint trim.
    // paintable_rows is retained as an alias of paintable_pool (all P+C).
    int paintable_pool = 0;
    bool pool_eligible = false;
    bool type_evidence_sufficient = false;
    int unobservable_rows = 0;
    int domain_first_y = -1;
    int domain_last_y = -1;
    int domain_rows = 0;
    int observable_blocks = 0;
    int qualified_paint_runs = 0;
    int repeated_paint_chain = 0;
    std::string type_reason = "insufficient_type_evidence";
    std::vector<EgoLaneEvidenceRow> evidence_rows;
    std::vector<EgoLaneEvidenceRun> evidence_runs;
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

// Additive diagnostic APIs; the model's analyze/draw entry points stay intact.
// Missing integer y inside [first_y,last_y] is U; duplicates/out-of-domain throw.
EgoLaneCandidateResult classify_twinlite_evidence(
    const std::vector<EgoLaneEvidenceRow>& rows, int first_y, int last_y);
EgoLaneCandidateResult analyze_twinlite_candidate_evidence(
    const cv::Mat& left_bgr, const EgoLaneCandidate& candidate);
const char* lane_evidence_state_name(EgoLaneEvidenceState state);



// R2 relay (single-pass): definition appended in the .cpp inside this
// namespace; shared context declared in lane_shared_frontend.h.
EgoLanePostprocessResult analyze_twinlite_ego_lane_shared(
    const lane_shared::SharedInputs& shared);
}  // namespace cand_observe
#endif  // CANDIDATE_LANE_POSTPROCESS_H
