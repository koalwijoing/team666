#include "p3_road_safety_advisor.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kMaskWidth = 512.0;
constexpr double kMaskHeight = 288.0;

bool road_user(int cls)
{
    return cls == 0 || cls == 1 || cls == 2 || cls == 3 || cls == 5 || cls == 7;
}

int state_from(P3RoadMarking marking)
{
    if (marking == P3RoadMarking::Dashed) return LANE_DASHED;
    if (marking == P3RoadMarking::Solid) return LANE_SOLID;
    if (marking == P3RoadMarking::Conflict) return LANE_INVALID;
    return LANE_NONE;
}

double curve_x(const P3RoadLaneBoundary& b, double y)
{
    const double z = (y - 208.0) / 68.0;
    return b.coef[0] * z * z + b.coef[1] * z + b.coef[2];
}

bool lane_bounds(const P3RoadSafetyInput& in, double y_image,
                 double& left, double& right)
{
    if (!in.twinlite_lane.fresh || !in.twinlite_lane.valid ||
        in.image_width <= 0 || in.image_height <= 0)
        return false;
    const auto& l = in.twinlite_lane.left;
    const auto& r = in.twinlite_lane.right;
    const int lo = std::max(l.y_min, r.y_min);
    const int hi = std::min(l.y_max, r.y_max);
    const double y = y_image * kMaskHeight / in.image_height;
    if (hi <= lo || y < lo - 12 || y > hi + 12) return false;
    left = curve_x(l, y) * in.image_width / kMaskWidth;
    right = curve_x(r, y) * in.image_width / kMaskWidth;
    return std::isfinite(left) && std::isfinite(right) && left < right;
}

bool twin_geometry_available(const P3RoadSafetyInput& in)
{
    if (!in.twinlite_lane.fresh || !in.twinlite_lane.valid ||
        in.image_width <= 0 || in.image_height <= 0)
        return false;
    const auto& l = in.twinlite_lane.left;
    const auto& r = in.twinlite_lane.right;
    const int lo = std::max(l.y_min, r.y_min);
    const int hi = std::min(l.y_max, r.y_max);
    if (l.y_min < 0 || r.y_min < 0 ||
        l.y_max >= kMaskHeight || r.y_max >= kMaskHeight || hi <= lo)
        return false;
    // A fresh type without finite, ordered image-space geometry does not
    // prove side clearance, even when the object detector reports no boxes.
    // Inspect the common endpoints and midpoint, without inventing geometry
    // from another engine or the forward-alert fallback band.
    for (int i = 0; i < 3; ++i) {
        const double y_mask = lo + (hi - lo) * (i / 2.0);
        double left = 0.0, right = 0.0;
        if (!lane_bounds(in, y_mask * in.image_height / kMaskHeight,
                         left, right) ||
            left < 0.0 || right > in.image_width)
            return false;
    }
    return true;
}

P3RoadZone localize(const P3RoadSafetyInput& in, const P3RoadObject& obj,
                    bool* used_fallback, bool* boundary_risk)
{
    // Box-bottom is an image-space ground-contact PROXY, not a measured
    // wheel position/distance. Do not use upper body/roof overlap to classify
    // an adjacent vehicle as occupying our path on a perspective road.
    const double bottom = std::min<double>(obj.y2, in.image_height - 1);
    const double upper = std::max<double>(obj.y1, bottom - std::min(8.0, (obj.y2 - obj.y1) * .10));
    const double center = (obj.x1 + obj.x2) * .5;
    const double width = obj.x2 - obj.x1;
    P3RoadZone zones[2] = {P3RoadZone::Unknown, P3RoadZone::Unknown};
    bool geometry_seen = true;
    for (int i = 0; i < 2; ++i) {
        const double y = i ? upper : bottom;
        double left = 0.0, right = 0.0;
        const double mask_y = y * kMaskHeight / in.image_height;
        const int lo = std::max(in.twinlite_lane.left.y_min, in.twinlite_lane.right.y_min);
        const int hi = std::min(in.twinlite_lane.left.y_max, in.twinlite_lane.right.y_max);
        // Integer mask rows cover half a pixel either side of their center;
        // e.g. source row359 maps to287.2 in a 288-high mask. This is pixel
        // support, not the previous +/-12-row extrapolation permission.
        if (!lane_bounds(in, y, left, right) || mask_y < lo - .5 || mask_y > hi + .5 ||
            left < 0.0 || right > in.image_width) {
            geometry_seen = false;
            break;  // no extrapolated geometry may qualify a direct STOP
        }
        const double margin = .05 * (right - left);
        if (center >= left + margin && center <= right - margin)
            zones[i] = P3RoadZone::Front;
        else if (center < left && std::max(0.0, obj.x2 - left) / width <= .20)
            zones[i] = P3RoadZone::Left;
        else if (center > right && std::max(0.0, right - obj.x1) / width <= .20)
            zones[i] = P3RoadZone::Right;
    }
    if (!geometry_seen) {
        // [L01-A2] No sample fell inside reliable geometry (no pair, empty
        // pair range, or the target outside the valid height/coverage): a
        // centre-band fallback may drive CAUTION, never direct STOP or
        // lateral clearance. A large side box touching this band is not
        // sufficient evidence of ego-lane occupancy.
        if (used_fallback) *used_fallback = true;
        if (in.image_width <= 0) return P3RoadZone::Unknown;
        if (obj.x2 >= 0.30 * in.image_width && obj.x1 <= 0.70 * in.image_width)
            return P3RoadZone::Front;
        return P3RoadZone::Unknown;
    }
    if (used_fallback) *used_fallback = false;
    if (zones[0] == zones[1] && zones[0] != P3RoadZone::Unknown)
        return zones[0];
    if (boundary_risk) *boundary_risk = true; // straddling/disagreeing: caution + both-side veto
    return P3RoadZone::Unknown;
}

P3LaneChangeAdvice lane_advice(int state, P3RoadMarking marking,
                               bool lane_geometry_fresh, bool objects_fresh,
                               int nearby, bool unresolved_risk)
{
    if (state == LANE_INVALID || marking == P3RoadMarking::Conflict)
        return P3LaneChangeAdvice::BlockedByConflictOrMissingData;
    if (state == LANE_SOLID || marking == P3RoadMarking::Solid)
        return P3LaneChangeAdvice::BlockedBySolidLine;
    if (!lane_geometry_fresh || state != LANE_DASHED ||
        marking != P3RoadMarking::Dashed || !objects_fresh)
        return P3LaneChangeAdvice::Unknown;
    // [L01-R1] A target whose position is unproven (unlocalized/over-aged/
    // fallback-only) blocks BOTH sides: "position unknown" is not "clear".
    if (unresolved_risk)
        return P3LaneChangeAdvice::BlockedByUnresolvedObjectRisk;
    return nearby ? P3LaneChangeAdvice::BlockedByNearbyObject
                  : P3LaneChangeAdvice::NoKnownBlocker;
}
}  // namespace

P3RoadSafetyAssessment assess_p3_road_safety(const P3RoadSafetyInput& in)
{
    P3RoadSafetyAssessment out;
    // [C01] 源时间合同：缺失/非有限/未来/时钟不匹配一律不可用，不得用决策
    // now 补缺或把负年龄截为 0。
    constexpr double kSourceFutureEpsMs = 1.0;
    const bool now_source_valid = std::isfinite(in.decision_now_ms) &&
                                  in.decision_now_ms > 0.0;
    const auto source_age_valid = [&](double source_ms) {
        return now_source_valid && std::isfinite(source_ms) &&
               source_ms > 0.0 &&
               source_ms <= in.decision_now_ms + kSourceFutureEpsMs;
    };
    const bool pair_fresh = in.twinlite_lane.fresh && in.twinlite_lane.valid &&
                            source_age_valid(in.twinlite_lane.source_ms);
    // [C-R2] 整次对象观察（含 0 框成功）必须有可用的观察级源时间；过期/
    // 缺失/未来的“空检测”不能证明净空。
    const bool object_obs_time_valid =
        source_age_valid(in.object_observation_source_ms) &&
        (in.decision_now_ms - in.object_observation_source_ms) <=
            in.object_source_ttl_ms;
    const bool object_fresh = in.object_observation_fresh &&
                              object_obs_time_valid;
    if (pair_fresh) {
        out.left_marking = in.twinlite_lane.left.marking;
        out.right_marking = in.twinlite_lane.right.marking;
        out.left_lane_state = state_from(out.left_marking);
        out.right_lane_state = state_from(out.right_marking);
    }
    // No TwinLite pair means unknown types and no lateral permit. Geometric
    // fallback below is exclusively a conservative forward-alert heuristic.

    int left_near = 0, right_near = 0;
    bool unlocalized = false, saw_front = false, stop = false, slow = false;
    bool uncertain_caution = false;
    bool stale_present = false;     // [L01-R1] over-aged: position unproven
    bool fallback_present = false;  // [L01-A2] located via centre fallback
    if (object_fresh) {
        for (const auto& obj : in.objects) {
            // [AUDIT-M6 2026-10-22] A NaN box must not enter localize():
            // every comparison against NaN is false, so conservatism currently
            // depends on luck. Reject non-finite evidence explicitly.
            if (!std::isfinite(obj.x1) || !std::isfinite(obj.y1) ||
                !std::isfinite(obj.x2) || !std::isfinite(obj.y2) ||
                !std::isfinite(obj.confidence) ||
                !road_user(obj.class_id) || obj.confidence < in.min_object_confidence ||
                obj.age_frames < 0 ||
                obj.x2 <= obj.x1 || obj.y2 <= obj.y1 || obj.x2 < 0 || obj.y2 < 0 ||
                obj.x1 >= in.image_width || obj.y1 >= in.image_height)
                continue;
            if (obj.age_frames > in.max_object_age_frames) {
                // [L01-R1] Over-aged targets do not drive the current
                // STOP/SLOW decision, but they cannot prove the lane is
                // clear either: they enter the lane-change veto.
                stale_present = true;
                continue;
            }
            if (!source_age_valid(obj.source_ms) ||
                (in.decision_now_ms - obj.source_ms) >
                    in.object_source_ttl_ms) {
                // [C01] 源时间缺失/非有限/未来/超源时间门：位置不可证明，
                // 与超龄同样进入变道否决；不把负年龄截为 0。
                stale_present = true;
                continue;
            }
            bool used_fallback = false;
            bool boundary_risk = false;
            const float height = std::clamp(
                (obj.y2 - obj.y1) / static_cast<float>(in.image_height), 0.0f, 1.0f);
            const P3RoadZone zone = localize(in, obj, &used_fallback, &boundary_risk);
            if (zone == P3RoadZone::Unknown) {
                ++out.unlocalized_object_count;
                unlocalized = true;
                uncertain_caution = uncertain_caution ||
                    (boundary_risk && height >= in.front_slow_height_fraction);
                continue;
            }
            if (used_fallback) {
                // [L01-A2] The position came from the fallback band: usable
                // for forward alerts, not proof that the side is clear.
                fallback_present = true;
            }
            if (zone == P3RoadZone::Front) {
                ++out.front_object_count;
                saw_front = true;
                out.largest_front_height_fraction = std::max(
                    out.largest_front_height_fraction, height);
                const float person_scale = obj.class_id == 0 ? 0.8f : 1.0f;
                stop = stop || (!used_fallback && height >= in.front_stop_height_fraction * person_scale);
                slow = slow || height >= in.front_slow_height_fraction * person_scale;
                uncertain_caution = uncertain_caution ||
                    (used_fallback && height >= in.front_slow_height_fraction * person_scale);
            } else if (zone == P3RoadZone::Left) {
                ++out.left_object_count;
                left_near += height >= in.side_change_block_height_fraction;
            } else if (zone == P3RoadZone::Right) {
                ++out.right_object_count;
                right_near += height >= in.side_change_block_height_fraction;
            }
        }
    }

    const bool lane_fresh = pair_fresh && twin_geometry_available(in);
    // [L01-A/R1] Position evidence accumulates per target (OR): unlocalized,
    // over-aged and fallback-only targets all make "no blocker observed now"
    // untrue; a reliable Front does not clear another target's fallback risk.
    const bool unresolved_object_risk =
        unlocalized || stale_present || fallback_present;
    out.left_change = lane_advice(out.left_lane_state, out.left_marking,
                                  lane_fresh, object_fresh,
                                  left_near, unresolved_object_risk);
    out.right_change = lane_advice(out.right_lane_state, out.right_marking,
                                   lane_fresh, object_fresh,
                                   right_near, unresolved_object_risk);
    if (!object_fresh) {
        out.front_action = P3RoadAction::Unknown;
        out.reason = in.object_observation_fresh
            ? "vehicle detector observation stale or time invalid"
            : "vehicle detector has no fresh successful observation";
    } else if (stop) {
        out.front_action = P3RoadAction::Stop;
        out.nearest_object_zone = P3RoadZone::Front;
        out.reason = "large front road user (uncalibrated image-size heuristic)";
    } else if (uncertain_caution) {
        out.front_action = P3RoadAction::SlowDown;
        out.reason = "unproven ego-path overlap; caution only (no direct stop)";
    } else if (slow) {
        out.front_action = P3RoadAction::SlowDown;
        out.nearest_object_zone = P3RoadZone::Front;
        out.reason = "near front road user (uncalibrated image-size heuristic)";
    } else if (unlocalized) {
        out.front_action = P3RoadAction::Unknown;
        out.reason = "road user detected but ego-lane geometry is unavailable";
    } else {
        out.front_action = P3RoadAction::Continue;
        out.reason = saw_front ? "front road user below advisory threshold"
                               : "no front threat above advisory threshold";
        if (saw_front) out.nearest_object_zone = P3RoadZone::Front;
    }
    if (left_near > 0) out.nearest_object_zone = P3RoadZone::Left;
    if (right_near > 0 && left_near == 0 && !saw_front)
        out.nearest_object_zone = P3RoadZone::Right;
    return out;
}

const char* p3_road_marking_name(P3RoadMarking v)
{
    switch (v) {
    case P3RoadMarking::Dashed: return "虚线";
    case P3RoadMarking::Solid: return "实线";
    case P3RoadMarking::Conflict: return "冲突";
    default: return "未知";
    }
}

const char* p3_road_action_name(P3RoadAction v)
{
    switch (v) {
    case P3RoadAction::Continue: return "可继续";
    case P3RoadAction::SlowDown: return "建议减速";
    case P3RoadAction::Stop: return "建议停车";
    default: return "未知";
    }
}

const char* p3_lane_change_advice_name(P3LaneChangeAdvice v)
{
    switch (v) {
    case P3LaneChangeAdvice::NoKnownBlocker: return "当前观测未发现阻碍";
    case P3LaneChangeAdvice::BlockedBySolidLine: return "实线禁止";
    case P3LaneChangeAdvice::BlockedByNearbyObject: return "侧车阻止";
    case P3LaneChangeAdvice::BlockedByConflictOrMissingData: return "冲突/数据不足";
    case P3LaneChangeAdvice::BlockedByUnresolvedObjectRisk:
        return "目标位置未定，拒绝变道";
    default: return "未知";
    }
}

const char* p3_road_zone_name(P3RoadZone v)
{
    switch (v) {
    case P3RoadZone::Front: return "前方";
    case P3RoadZone::Left: return "左侧";
    case P3RoadZone::Right: return "右侧";
    default: return "位置未知";
    }
}
