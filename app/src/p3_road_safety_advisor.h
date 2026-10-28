#ifndef P3_ROAD_SAFETY_ADVISOR_H
#define P3_ROAD_SAFETY_ADVISOR_H

#include "p3_lane_state.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

// Conservative advice only: this module never commands a motor and does not
// claim metric distance or TTC. Image-size thresholds need labeled-board
// calibration before they can be used for physical actuation.
enum class P3RoadMarking { Unknown, Dashed, Solid, Conflict };
enum class P3RoadZone { Unknown, Front, Left, Right };
enum class P3RoadAction { Unknown, Continue, SlowDown, Stop };
enum class P3LaneChangeAdvice {
    Unknown,
    NoKnownBlocker,
    BlockedBySolidLine,
    BlockedByNearbyObject,
    BlockedByConflictOrMissingData,
    // [L01-R1] A target exists whose position cannot be proven (unlocalized
    // or over-aged): this is not "no blocker observed now" and must not
    // grant a new lane-change permit.
    BlockedByUnresolvedObjectRisk,
};

struct P3RoadLaneBoundary {
    // TwinLite mask-space polynomial: x=a*z^2+b*z+c, z=(y-208)/68.
    std::array<double, 3> coef{};
    int y_min = 0;
    int y_max = 0;
    P3RoadMarking marking = P3RoadMarking::Unknown;
};

struct P3RoadLanePair {
    bool fresh = false;
    bool valid = false;
    double score = 0.0;
    // [C01/C-R4] TwinLite 观察的源时间/运行身份/目别（steady ms，采集返回）。
    // 0/非有限/未来/非左目/旧 run 由桥显式判不可用；不得用决策 now 代替。
    double source_ms = 0.0;
    std::uint64_t source_run_id = 0;
    int source_eye = -1;  // 0=left；其它值（含 -1 未知）拒绝
    P3RoadLaneBoundary left;
    P3RoadLaneBoundary right;
};

struct P3RoadObject {
    float x1 = 0.0f;
    float y1 = 0.0f;
    float x2 = 0.0f;
    float y2 = 0.0f;
    float confidence = 0.0f;
    int class_id = -1;       // COCO ids: person=0, bicycle=1, car=2, etc.
    int age_frames = 0;      // 0=inferred now; >0=tracked/cached observation
    // [C01] 产生该观察的源时间（steady ms，采集返回）；跟踪复用不刷新。
    // 0/非有限/未来值由顾问显式判不可用。
    double source_ms = 0.0;
};

struct P3RoadSafetyInput {
    int image_width = 640;
    int image_height = 360;
    // TwinLite is the only source of lane type and geometry. An absent or
    // unusable pair cannot grant lateral clearance; there is no other lane
    // engine or authority-mode fallback.
    P3RoadLanePair twinlite_lane;
    bool object_observation_fresh = false;
    // [C-R2] 整次对象观察的源时间/run 身份（成功 0 框也必须有）。
    double object_observation_source_ms = 0.0;
    std::uint64_t object_observation_run_id = 0;
    std::vector<P3RoadObject> objects;
    // [C-R1] 决策当前时间（steady ms），由桥/纯顾问在真正求值时打点；
    // 不是当前帧采集时间。<=0/非有限时对象/观察源年龄无法判定。
    double decision_now_ms = 0.0;
    // [C01] 对象源时间门（与 max_object_age_frames 同时生效，不替换）。
    // [USER-A 2026-10-22] 对象源窗口 250 -> 400 ms，与 Twin 源窗口对齐：
    // 对象观察与 Twin 车道来自同一帧采集，250 ms 比车道自身的 400 ms 门更严，
    // 在 7 Hz 流水线尾部会周期性把对象判成过期，导致 A/D 的 right_change
    // 变成 Unknown。2 帧身份门保持不变。
    double object_source_ttl_ms = 400.0;

    // Initial image-space heuristics only; they are not speed/distance units.
    float min_object_confidence = 0.35f;
    float front_slow_height_fraction = 0.15f;
    float front_stop_height_fraction = 0.25f;
    float side_change_block_height_fraction = 0.12f;
    int max_object_age_frames = 2;
};

struct P3RoadSafetyAssessment {
    int left_lane_state = LANE_NONE;
    int right_lane_state = LANE_NONE;
    P3RoadMarking left_marking = P3RoadMarking::Unknown;
    P3RoadMarking right_marking = P3RoadMarking::Unknown;
    P3LaneChangeAdvice left_change = P3LaneChangeAdvice::Unknown;
    P3LaneChangeAdvice right_change = P3LaneChangeAdvice::Unknown;
    P3RoadAction front_action = P3RoadAction::Unknown;
    P3RoadZone nearest_object_zone = P3RoadZone::Unknown;
    int front_object_count = 0;
    int left_object_count = 0;
    int right_object_count = 0;
    int unlocalized_object_count = 0;
    float largest_front_height_fraction = 0.0f;
    std::string reason = "waiting for fresh observations";
};

P3RoadSafetyAssessment assess_p3_road_safety(const P3RoadSafetyInput& input);
const char* p3_road_marking_name(P3RoadMarking value);
const char* p3_road_action_name(P3RoadAction value);
const char* p3_lane_change_advice_name(P3LaneChangeAdvice value);
const char* p3_road_zone_name(P3RoadZone value);

#endif  // P3_ROAD_SAFETY_ADVISOR_H
