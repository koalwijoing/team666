// [C-R1/C-R2] Single production packing path for P3RoadSafetyInput.
// Used by main.cpp AND the freshness integration test so the test exercises
// the real wiring (observation source time, decision now, Twin run/eye,
// object observation identity), not a hand-built ideal input.
#ifndef P3_ROAD_INPUT_BUILD_H
#define P3_ROAD_INPUT_BUILD_H

#include "p3_engine.h"
#include "p3_road_safety_advisor.h"
#include "twinlite_ego_lane_postprocess.h"

#include <cstdint>
#include <vector>

inline P3RoadMarking road_marking_from_twinlite(LanePaintType type)
{
    if (type == LanePaintType::Dashed) return P3RoadMarking::Dashed;
    if (type == LanePaintType::Solid) return P3RoadMarking::Solid;
    return P3RoadMarking::Unknown;
}

inline P3RoadLaneBoundary road_boundary_from_twinlite(
    const EgoLaneCandidateResult& source)
{
    P3RoadLaneBoundary result;
    result.coef = source.candidate.coef;
    result.y_min = source.candidate.y_min;
    result.y_max = source.candidate.y_max;
    result.marking = road_marking_from_twinlite(source.paint_type);
    return result;
}

struct RoadInputFrame {
    double frame_source_ms = 0.0;  // capture-return source time (steady ms)
    double decision_now_ms = 0.0;  // decision clock (steady ms)
    std::uint64_t delivery_id = 0;
};

inline P3RoadSafetyInput build_road_input(
    const EgoLanePostprocessResult* lane,
    const std::vector<P3Det>& detections,
    const P3ObjectObservation& obj_obs,
    const RoadInputFrame& frame)
{
    P3RoadSafetyInput in;
    in.decision_now_ms = frame.decision_now_ms;
    if (lane) {
        const bool indexes_valid =
            lane->left_index >= 0 && lane->right_index >= 0 &&
            lane->left_index < static_cast<int>(lane->candidates.size()) &&
            lane->right_index < static_cast<int>(lane->candidates.size());
        if (indexes_valid) {
            in.twinlite_lane.fresh = true;
            in.twinlite_lane.valid = true;
            in.twinlite_lane.source_ms = frame.frame_source_ms;
            in.twinlite_lane.source_run_id = frame.delivery_id;
            in.twinlite_lane.source_eye = 0;  // left eye only
            in.twinlite_lane.score = lane->pair_score;
            in.twinlite_lane.left = road_boundary_from_twinlite(
                lane->candidates[static_cast<std::size_t>(lane->left_index)]);
            in.twinlite_lane.right = road_boundary_from_twinlite(
                lane->candidates[static_cast<std::size_t>(lane->right_index)]);
        }
    }
    // [C-R2] Observation-level identity covers successful 0-box observations.
    in.object_observation_fresh = obj_obs.status == 1;
    in.object_observation_source_ms = obj_obs.capture_done_ms;
    in.object_observation_run_id = obj_obs.run_id;
    for (const P3Det& det : detections) {
        if (det.model_id != 2) continue;
        in.objects.push_back({det.x1, det.y1, det.x2, det.y2, det.score,
                              det.cls, det.obs_age_frames, det.obs_capture_ms});
    }
    return in;
}

#endif  // P3_ROAD_INPUT_BUILD_H
