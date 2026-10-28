#pragma once
#include "p3_road_safety_advisor.h"

// Synthetic metadata for new observations in the existing device-free tests.
// Production source/expected decisions stay unchanged. A skipped Twin update
// is deliberately NOT stamped; the real bridge cache must age on its old source.
inline P3RoadSafetyInput fresh_fixture(const P3RoadSafetyInput& supplied,
                                     double now, std::uint64_t id) {
    auto in = supplied;
    in.decision_now_ms = now;
    if (in.twinlite_lane.fresh) {
        in.twinlite_lane.source_ms = now;
        in.twinlite_lane.source_run_id = id;
        in.twinlite_lane.source_eye = 0;
    }
    if (in.object_observation_fresh) {
        in.object_observation_source_ms = now;
        in.object_observation_run_id = id;
    }
    for (auto& object : in.objects)
        object.source_ms = now - object.age_frames * 130.0;
    return in;
}

inline void fixture_geometry(P3RoadSafetyInput& in) {
    in.twinlite_lane.left.coef = {0.0, 0.0, 100.0};
    in.twinlite_lane.right.coef = {0.0, 0.0, 300.0};
    in.twinlite_lane.left.y_min = in.twinlite_lane.right.y_min = 0;
    in.twinlite_lane.left.y_max = in.twinlite_lane.right.y_max = 287;
}
