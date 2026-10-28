#include "p3_decision_bridge.h"

#include "perf_config.h"

#include <chrono>
#include <cmath>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <stdexcept>

namespace {
constexpr std::uint16_t REG_CTRL = 0x200;
constexpr std::uint16_t REG_STATUS = 0x204;
constexpr std::uint16_t REG_EVENT = 0x208;
constexpr std::uint16_t REG_INJECT = 0x20C;
constexpr std::uint32_t EVENT_MASK = 0x1F;
constexpr std::uint32_t CTRL_RESERVED_MASK = 0x0000FFF0;
constexpr std::uint32_t STATUS_RESERVED_MASK = 0x0000E0C0;
constexpr std::uint32_t CTRL_STATUS_MATCH_MASK = 0xFFFF000F;
constexpr int DEBOUNCE_FRAMES = 5;
constexpr int RED_MEMORY_FRAMES = 30;
constexpr int ALARM_HOLD_FRAMES = 20;
// 低于该置信度的灯检测不参与方向合成（引擎侧已按 light_conf 过滤，
// 这里再设一个语义下限，避免极低分检测干扰方向判定）。
constexpr float MIN_LIGHT_SIGNAL_CONF = 0.30f;

// Opt-in simulation interaction profile. No perception TTL or risk threshold
// changes. Unset variables retain the delivered release behaviour.
bool retry_uncertain_request()
{
    const char* value = std::getenv("P3_AD_RETRY_UNCERTAIN");
    return value && std::strcmp(value, "1") == 0;
}

bool strict_simulated_lane_edge()
{
    const char* value = std::getenv("P3_SIM_LANE_EDGE_STRICT");
    return !value || std::strcmp(value, "0") != 0;
}

double lane_request_wait_ms()
{
    const char* value = std::getenv("P3_AD_REQUEST_WAIT_MS");
    if (!value) return 1000.0;
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (end == value || *end || !std::isfinite(parsed) ||
        parsed < 1000.0 || parsed > 2000.0)
        return 1000.0;
    return parsed;
}

// [C-N2] 同一 run + 同一 source_ms 的重复 Twin 包必须 payload 完全一致；
// 任何字段变化都视为冲突，不得覆盖已接纳的观察。
bool same_twin_payload(const P3RoadLanePair& a, const P3RoadLanePair& b)
{
    const auto same_boundary = [](const P3RoadLaneBoundary& x,
                                  const P3RoadLaneBoundary& y) {
        return x.marking == y.marking && x.y_min == y.y_min &&
               x.y_max == y.y_max && x.coef[0] == y.coef[0] &&
               x.coef[1] == y.coef[1] && x.coef[2] == y.coef[2];
    };
    return same_boundary(a.left, b.left) && same_boundary(a.right, b.right);
}

// 颜色 → 灯状态；NONE 表示非红绿黄。
P3LightState color_to_state(p3_light::Color c)
{
    switch (c) {
    case p3_light::COLOR_RED: return P3LightState::RED;
    case p3_light::COLOR_GREEN: return P3LightState::GREEN;
    case p3_light::COLOR_YELLOW: return P3LightState::YELLOW;
    default: return P3LightState::NONE;
    }
}

// 最保守优先合成：RED > YELLOW > GREEN。
P3LightState worst_of(P3LightState a, P3LightState b)
{
    auto rank = [](P3LightState s) {
        switch (s) {
        case P3LightState::RED: return 3;
        case P3LightState::YELLOW: return 2;
        case P3LightState::GREEN: return 1;
        default: return 0;
        }
    };
    return rank(a) >= rank(b) ? a : b;
}
}

bool P3DecisionBridge::open(const std::string& bdf, std::string* error)
{
    if (!bar0_.open(bdf, 8192, error))
        return false;
    return true;
}

void P3DecisionBridge::reset_state()
{
    light_history_.clear();
    stable_light_ = P3LightState::NONE;
    red_memory_frames_ = 0;
    alarm_hold_frames_ = 0;
    alarm_ = false;
    alarm_reason_.clear();
    key_feedback_.clear();
    key_feedback_until_ms_ = 0.0;
    pending_lane_key_ = 0;
    pending_lane_started_ms_ = pending_lane_deadline_ms_ = 0.0;
    twin_auth_left_ = LANE_NONE;
    twin_auth_right_ = LANE_NONE;
    twin_auth_last_ms_ = 0.0;
    twin_lane_cache_ = P3RoadLanePair{};
    left_channel_ = DirectionLightChannel{};
    straight_channel_ = DirectionLightChannel{};
    right_channel_ = DirectionLightChannel{};
    lamp_state_ = LampEvidenceState{};
    light_evidence_.clear();
    light_direction_ = p3_light::DIR_UNKNOWN;
    light_confidence_ = 0.0f;
    light_fresh_ = false;
    // lane_position_ / running_ 保留（用户交互状态，跨预热不清）。
}

P3LightState P3DecisionBridge::classify_light(
    const std::vector<P3Det>& detections, P3LanePosition lane,
    std::string* evidence, int* dominant_dir, float* confidence,
    bool* light_fresh)
{
    using p3_light::Color;
    using p3_light::Direction;

    const Direction lane_dir = (lane == P3LanePosition::LEFT) ? p3_light::DIR_LEFT
                             : (lane == P3LanePosition::RIGHT) ? p3_light::DIR_RIGHT
                                                               : p3_light::DIR_FORWARD;

    bool has_unknown_dir = false;  // 存在方向不可判定的灯
    bool has_off_relevant = false;  // Preserve the current v9 dark-head guard.
    // [T02-A] 与本车道相关、但未经复核（预算外/开关关闭）或复核出错的
    // 候选：不能作为放行依据，也不允许被已接受绿/黄覆盖为放行。
    bool has_unverified_relevant = false;
    bool fresh = false;
    Direction dominant = p3_light::DIR_UNKNOWN;
    float best_conf = 0.0f;
    P3LightState relevant = P3LightState::NONE;  // 与当前车道相关的合成结果
    std::string ev;

    for (const P3Det& det : detections) {
        if (det.model_id != 0)
            continue;
        fresh = true;
        // [AUDIT-H1 2026-10-22] A NaN score must not pass the gate: NaN
        // comparisons are false, so the old check let a non-number through
        // and a NaN-score green could release GO.
        if (!std::isfinite(det.score))
            continue;
        // [VERIFY-TRUST 2026-10-24] A candidate accepted by the 224x224
        // light/not-light verifier carries evidence even when its raw
        // full-frame score is below the generic signal gate: the verifier
        // already scored the crop at >= light_verify_conf. The raw-score gate
        // now applies only to candidates the verifier did NOT accept.
        // Symptom this fixes: with P3_LIGHT_CONF=0.20 the detector drew and
        // labelled a green "go", but its raw score (0.2x) failed the 0.30
        // gate here, so the vote never saw it and the light diagnostic
        // reported 无灯 (NONE) on a visibly detected green.
        if (det.verify_state != kLightVerifyAccepted &&
            det.score < MIN_LIGHT_SIGNAL_CONF)
            continue;

        // [T02-A] 复核状态门控：拒绝候选不参与（引擎已剔除，防御处理）。
        if (det.verify_state == kLightVerifyRejected) {
            ev += "rej ";
            continue;
        }
        const bool unverified = (det.verify_state == kLightVerifyNotEvaluated ||
                                 det.verify_state == kLightVerifyError);

        if (det.label == "unknown") {
            has_unknown_dir = true;
            ev += "unknown";
            if (unverified)
                ev += "(uv)";
            ev += "@" + std::to_string(det.score).substr(0, 4) + " ";
            continue;
        }
        if (det.label == "off") {
            has_off_relevant = true;
            ev += "off ";
            continue;
        }
        Color color = p3_light::COLOR_NONE;
        Direction dir = p3_light::DIR_UNKNOWN;
        if (!p3_light::parse_label(det.label, &color, &dir))
            continue;  // 非本类别表的标签：不参与语义合成
        const bool matches_lane = (dir == p3_light::DIR_ROUND) ||
                                  (dir == lane_dir);
        ev += det.label + "@" + std::to_string(det.score).substr(0, 4) +
              (unverified ? "(uv)" : "") + (matches_lane ? "* " : " ");
        if (!matches_lane)
            continue;  // 其它方向的箭头：与本车道无关，不因它放行/禁行

        if (unverified) {
            // 未复核/出错的相关候选：仅作保守证据。
            has_unverified_relevant = true;
            continue;
        }

        const P3LightState s = color_to_state(color);
        if (s == P3LightState::NONE)
            continue;
        const P3LightState merged = worst_of(relevant, s);
        if (merged != relevant && det.score >= best_conf) {
            dominant = dir;
            best_conf = det.score;
        }
        relevant = merged;
    }

    P3LightState verdict = P3LightState::NONE;
    if (relevant == P3LightState::RED) {
        verdict = relevant;  // 已接受红灯优先，未复核候选不影响其确定性
    } else if (has_unverified_relevant || has_unknown_dir) {
        // [T02-A] 存在未确认的相关候选：不得被已接受绿/黄覆盖为放行。
        verdict = P3LightState::UNKNOWN;
    } else if (relevant != P3LightState::NONE) {
        verdict = relevant;
    }

    if (verdict == P3LightState::NONE && has_off_relevant)
        verdict = P3LightState::UNKNOWN;
    if (evidence)
        *evidence = ev.empty() ? "no-light" : ev;
    if (dominant_dir)
        *dominant_dir = static_cast<int>(dominant);
    if (confidence)
        *confidence = best_conf;
    if (light_fresh)
        *light_fresh = fresh;
    return verdict;
}

P3LightState P3DecisionBridge::debounce_light(P3LightState raw)
{
    light_history_.push_back(raw);
    while (light_history_.size() > DEBOUNCE_FRAMES)
        light_history_.pop_front();
    if (light_history_.size() < DEBOUNCE_FRAMES)
        return P3LightState::NONE;

    for (P3LightState state :
         {P3LightState::RED, P3LightState::YELLOW, P3LightState::UNKNOWN,
          P3LightState::GREEN}) {
        const int count =
            static_cast<int>(std::count(light_history_.begin(),
                                        light_history_.end(), state));
        if (count >= DEBOUNCE_FRAMES - 1)
            return state;
    }
    return P3LightState::NONE;
}

P3Action P3DecisionBridge::apply_light_to_action(P3Action action,
                                                 P3LightState light,
                                                 bool running,
                                                 std::string* reason)
{
    if (action != P3Action::GO)
        return action;
    if (!running)
        return P3Action::STOP;
    switch (light) {
    case P3LightState::RED:
        return P3Action::STOP;
    case P3LightState::UNKNOWN:
        // 有灯目标但方向/颜色语义不可判定且无圆灯依据：保守停车。
        if (reason && reason->empty())
            *reason = "light direction unknown, conservative stop";
        return P3Action::STOP;
    case P3LightState::YELLOW:
        return P3Action::SLOW;
    default:
        return action;
    }
}

bool P3DecisionBridge::decision_abi_matches(std::uint32_t ctrl,
                                            std::uint32_t status,
                                            std::uint32_t events,
                                            std::uint32_t inject,
                                            std::string* error)
{
    const char* why = nullptr;
    if (ctrl & CTRL_RESERVED_MASK)
        why = "decision ABI mismatch: CTRL reserved bits are nonzero";
    else if (status & STATUS_RESERVED_MASK)
        why = "decision ABI mismatch: STATUS reserved bits are nonzero";
    else if ((ctrl ^ status) & CTRL_STATUS_MATCH_MASK)
        why = "decision ABI mismatch: CTRL/STATUS readback differs";
    else if (events & ~EVENT_MASK)
        why = "decision ABI mismatch: EVENT has unsupported bits";
    else if (inject != 0)
        why = "decision ABI mismatch: INJECT is not read-as-zero";

    if (error)
        *error = why ? why : "";
    return why == nullptr;
}

std::uint32_t P3DecisionBridge::filter_sticky_events(
    std::uint32_t events, std::uint32_t& rearmed)
{
    events &= EVENT_MASK;
    const std::uint32_t fresh = events & rearmed;
    rearmed &= ~events;
    rearmed |= EVENT_MASK & ~events;
    return fresh;
}

void P3DecisionBridge::trigger_alarm(const std::string& reason)
{
    alarm_ = true;
    alarm_reason_ = reason;
    alarm_hold_frames_ = ALARM_HOLD_FRAMES;
}

P3DecisionBridge::P3DecisionBridge()
{
    light_ttl_ms_ = static_cast<double>(p3_config().light_ttl_ms);
    // A retired launch switch must never silently restore RTL authority.
    // [AUDIT-M3 2026-10-22] Fail fast on ANY unrecognized value: the old check
    // only inspected the first character, so "no"/"off"/"2" slipped through.
    const char* auth_env = std::getenv("P3_LANE_AUTH_TWIN");
    if (auth_env && *auth_env) {
        const bool twin_truthy =
            std::strcmp(auth_env, "1") == 0 || std::strcmp(auth_env, "t") == 0 ||
            std::strcmp(auth_env, "T") == 0 || std::strcmp(auth_env, "true") == 0 ||
            std::strcmp(auth_env, "TRUE") == 0 || std::strcmp(auth_env, "on") == 0 ||
            std::strcmp(auth_env, "ON") == 0;
        if (!twin_truthy)
            throw std::invalid_argument(
                "RTL lane authority retired: P3_LANE_AUTH_TWIN value '" +
                std::string(auth_env) + "' is not a recognized Twin enable");
    }
    if (const char* v = std::getenv("P3_LANE_POS_RESET_S")) {
        const double parsed = std::atof(v);
        if (std::isfinite(parsed)) lane_pos_reset_s_ = parsed;
    }

}

double P3DecisionBridge::now_ms() const
{
    if (now_ms_fn_)
        return now_ms_fn_();
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

P3LightState P3DecisionBridge::apply_lamp_evidence(
    P3LightState composed, const P3LampObservation* obs, double now_ms_value,
    double ttl_ms, LampEvidenceState& state, bool* should_vote,
    bool* stale_out, bool* invalid_meta_out)
{
    if (should_vote)
        *should_vote = false;
    // [C-R3] 非法灯时间（NaN/Inf/0/未来/时钟不匹配）不能刷新成功时刻、
    // 不能投票，也不能把旧绿复活。
    constexpr double kFutureEpsMs = 1.0;
    const bool obs_time_valid =
        obs && obs->status == 1 && std::isfinite(obs->capture_done_ms) &&
        obs->capture_done_ms > 0.0 && std::isfinite(now_ms_value) &&
        obs->capture_done_ms <= now_ms_value + kFutureEpsMs;
    if (obs && obs->status == 1 && obs_time_valid) {
        state.has_success = true;
        state.last_success_capture_ms = obs->capture_done_ms;
    }
    const bool invalid_time = obs && obs->status == 1 && !obs_time_valid;
    if (invalid_meta_out)
        *invalid_meta_out = invalid_time;
    const bool stale = state.has_success && ttl_ms > 0 &&
                       (now_ms_value - state.last_success_capture_ms) > ttl_ms;
    if (stale_out)
        *stale_out = stale || invalid_time;
    // 观察去重：只有"本帧成功推理且 run_id 变化"才允许投票一次
    // （同一次观察的多个框已在 classify_light 内先合成为一个状态）。
    if (should_vote && obs && obs->status == 1 && obs_time_valid &&
        obs->run_id != state.last_voted_run) {
        *should_vote = true;
        state.last_voted_run = obs->run_id;
    }
    if (invalid_time) {
        // 非法观察不携带任何放行证据；红灯保持最保守，绿/黄降为 UNKNOWN。
        return composed == P3LightState::RED ? P3LightState::RED
                                             : P3LightState::UNKNOWN;
    }
    P3LightState verdict = composed;
    if (stale && (verdict == P3LightState::GREEN ||
                  verdict == P3LightState::YELLOW)) {
        // 灯证据过期：不得沿用绿灯/黄灯放行；红灯维持最保守状态。
        verdict = P3LightState::UNKNOWN;
    }
    return verdict;
}

int P3DecisionBridge::twin_auth_state(P3RoadMarking marking)
{
    if (marking == P3RoadMarking::Dashed) return LANE_DASHED;
    if (marking == P3RoadMarking::Solid) return LANE_SOLID;
    if (marking == P3RoadMarking::Conflict) return LANE_INVALID;
    return LANE_NONE;
}

bool P3DecisionBridge::twin_auth_window_fresh(double last_ms, double now_ms)
{
    return last_ms > 0.0 && now_ms - last_ms <= kTwinAuthHoldMs;
}

P3LightState P3DecisionBridge::DirectionLightChannel::apply(
    P3LightState raw, const P3LampObservation* obs, double now, double ttl)
{
    // [TWIN-AUTH/P2] Same evidence contract as the main light pipeline:
    // successful-run dedup and source time for voting, immediate red/unknown,
    // 4-of-5 debounce with no implicit recovery, TTL downgrade, red memory.
    // [C-R3] 方向通道同一时间合同：非法时刻不刷新、不投票、不复活旧绿。
    constexpr double kFutureEpsMs = 1.0;
    const bool obs_time_valid =
        obs && obs->status == 1 && std::isfinite(obs->capture_done_ms) &&
        obs->capture_done_ms > 0.0 && std::isfinite(now) &&
        obs->capture_done_ms <= now + kFutureEpsMs;
    if (obs && obs->status == 1 && obs_time_valid) {
        has_success = true;
        last_success_capture_ms = obs->capture_done_ms;
    }
    const bool invalid_time = obs && obs->status == 1 && !obs_time_valid;
    const bool stale = has_success && ttl > 0 &&
                       (now - last_success_capture_ms) > ttl;
    bool vote = false;
    if (obs && obs->status == 1 && obs_time_valid &&
        obs->run_id != last_voted_run) {
        vote = true;
        last_voted_run = obs->run_id;
    }
    if (vote) {
        if (raw == P3LightState::RED ||
            (raw == P3LightState::UNKNOWN && stable != P3LightState::RED)) {
            history.clear();
            history.push_back(raw);
            stable = raw;
        } else {
            const P3LightState previous = stable;
            history.push_back(raw);
            while (history.size() > static_cast<std::size_t>(DEBOUNCE_FRAMES))
                history.pop_front();
            P3LightState voted = P3LightState::NONE;
            if (history.size() >= static_cast<std::size_t>(DEBOUNCE_FRAMES)) {
                for (P3LightState state :
                     {P3LightState::RED, P3LightState::YELLOW,
                      P3LightState::UNKNOWN, P3LightState::GREEN}) {
                    if (std::count(history.begin(), history.end(), state) >=
                        DEBOUNCE_FRAMES - 1) {
                        voted = state;
                        break;
                    }
                }
            }
            const bool clear_consensus =
                history.size() == 5 &&
                std::count(history.begin(), history.end(),
                           P3LightState::NONE) >= 4;
            stable = voted == P3LightState::NONE && !clear_consensus &&
                             (previous == P3LightState::UNKNOWN ||
                              previous == P3LightState::RED)
                         ? previous
                         : voted;
        }
    }
    if ((stale || invalid_time) && (stable == P3LightState::GREEN ||
                                    stable == P3LightState::YELLOW)) {
        stable = P3LightState::UNKNOWN;
        // [C-N1] 过期/非法后清掉旧绿票；恢复需重新集满 4/5。
        history.clear();
    }
    if (invalid_time)
        history.clear();  // [C-N1] 非法观察后不得复用任何旧票
    // Match the already-qualified main-light rule: a HELD red is not a new
    // red observation. Cached/failed frames must never renew the counter.
    // Expiry alone does not turn red into NONE: clearing still needs fresh
    // successful no-light consensus (or verified green) above.
    if (vote && raw == P3LightState::RED) {
        red_memory = RED_MEMORY_FRAMES;
    } else if (stable == P3LightState::GREEN) {
        red_memory = 0;
    } else if (red_memory > 0) {
        --red_memory;
        stable = P3LightState::RED;
    }
    return stable;
}

void P3DecisionBridge::update_direction_channels(
    const std::vector<P3Det>& detections, const P3LampObservation* lamp,
    double now_ms_value)
{
    const P3LightState raw_left =
        classify_light(detections, P3LanePosition::LEFT);
    const P3LightState raw_straight =
        classify_light(detections, P3LanePosition::STRAIGHT);
    const P3LightState raw_right =
        classify_light(detections, P3LanePosition::RIGHT);
    left_channel_.apply(raw_left, lamp, now_ms_value, light_ttl_ms_);
    straight_channel_.apply(raw_straight, lamp, now_ms_value, light_ttl_ms_);
    right_channel_.apply(raw_right, lamp, now_ms_value, light_ttl_ms_);
}

void P3DecisionBridge::rearm_acknowledged_events(
    std::uint32_t cleared, std::uint32_t after_clear, std::uint32_t& rearmed)
{
    // EVENT is a sticky W1C latch, not the physical key-down level. Readback
    // completes the posted PCI write and proves which bits actually cleared.
    rearmed |= cleared & ~after_clear & EVENT_MASK;
}

std::uint32_t P3DecisionBridge::select_lane_request(
    std::uint32_t events, double now, P3LightState left_gate,
    P3LightState right_gate, const P3DecisionOutput& snapshot)
{
    const auto feedback = [&](std::uint32_t key, const char* text) {
        key_feedback_ = key == 4 ? "last A: " : "last D: ";
        key_feedback_ += text;
        key_feedback_until_ms_ = std::isfinite(now) ? now + 3000.0 : 0.0;
    };
    const std::uint32_t lateral = events & 0x0C;
    // S/W/F invalidate previous lateral intent, including mixed key chords.
    if (events & 0x13) {
        if (pending_lane_key_ || lateral)
            feedback(pending_lane_key_ ? pending_lane_key_ : lateral,
                     "cancelled: control key changed");
        pending_lane_key_ = 0;
        return events & ~0x0CU;
    }
    if (lateral == 0x0C) {
        pending_lane_key_ = 0;
        return events;  // Existing simultaneous-key conflict/alarm path.
    }
    if (!running_) {
        pending_lane_key_ = 0;
        return events;
    }
    if (lateral && lateral != pending_lane_key_) {
        // Opposite press replaces intent. Identical repeats never extend TTL.
        pending_lane_key_ = lateral;
        pending_lane_started_ms_ = now;
        pending_lane_deadline_ms_ = now + lane_request_wait_ms();
    }
    if (!pending_lane_key_)
        return events;
    const std::uint32_t key = pending_lane_key_;
    if (!std::isfinite(now) || !std::isfinite(pending_lane_started_ms_) ||
        now < pending_lane_started_ms_ || now >= pending_lane_deadline_ms_) {
        feedback(key, "expired: evidence not ready; press again");
        pending_lane_key_ = 0;
        return events & ~0x0CU;
    }
    const bool left = key == 4;
    const auto gate = left ? left_gate : right_gate;
    const int state = left ? snapshot.left_lane_state : snapshot.right_lane_state;
    const auto advice = left ? snapshot.road_safety.left_change : snapshot.road_safety.right_change;
    const auto marking = left ? snapshot.road_safety.left_marking : snapshot.road_safety.right_marking;
    const bool edge = strict_simulated_lane_edge() &&
        (left ? lane_position_ == P3LanePosition::LEFT
              : lane_position_ == P3LanePosition::RIGHT);
    const bool uncertain_wait = retry_uncertain_request() &&
        advice == P3LaneChangeAdvice::BlockedByUnresolvedObjectRisk;
    // An unlocalized straddling target can ask for caution while its position
    // is unresolved. Keep that SLOW output, but retain operator intent. A real
    // front object or any other concrete veto still consumes the request.
    const bool uncertain_slow = uncertain_wait &&
        snapshot.road_safety.front_action == P3RoadAction::SlowDown &&
        snapshot.road_safety.front_object_count == 0;
    // [LANE-POS 2026-10-23] "already leftmost/rightmost" is a statement about
    // the CURRENT scene, so it only counts while lane evidence is fresh; a
    // stale position from an old route must not veto the new scene.
    const bool hard_veto = (edge && snapshot.lane_fresh) || gate == P3LightState::RED ||
        gate == P3LightState::YELLOW || gate == P3LightState::UNKNOWN ||
        state == LANE_SOLID || state == LANE_INVALID ||
        marking == P3RoadMarking::Conflict ||
        advice == P3LaneChangeAdvice::BlockedBySolidLine ||
        advice == P3LaneChangeAdvice::BlockedByNearbyObject ||
        (advice == P3LaneChangeAdvice::BlockedByUnresolvedObjectRisk && !uncertain_wait) ||
        snapshot.road_safety.front_action == P3RoadAction::Stop ||
        (snapshot.road_safety.front_action == P3RoadAction::SlowDown && !uncertain_slow);
    const bool incomplete = !snapshot.lane_fresh || state == LANE_NONE ||
        advice == P3LaneChangeAdvice::Unknown ||
        advice == P3LaneChangeAdvice::BlockedByConflictOrMissingData ||
        uncertain_wait;
    if (!hard_veto && incomplete) {
        feedback(key, !snapshot.lane_fresh || state == LANE_NONE
            ? "waiting: fresh dashed lane evidence"
            : "waiting: fresh vehicle clearance evidence");
        return events & ~0x0CU;
    }
    // Grant OR a concrete refusal consumes intent. No automatic resurrection
    // when a previously observed solid line/neighbor/red light disappears.
    pending_lane_key_ = 0;
    return (events & ~0x0CU) | key;
}

void P3DecisionBridge::process_key_events(
    std::uint32_t events, P3LightState light,
    P3LightState left_gate_light, P3LightState right_gate_light,
    int left_lane_state, int right_lane_state, bool lane_fresh,
    bool* left_request,
    bool* right_request, P3DecisionOutput& output,
    const P3RoadSafetyAssessment* road_safety)
{
    if (events & 0x01) {
        lane_position_ = static_cast<P3LanePosition>(
            (static_cast<int>(lane_position_) + 1) % 3);
    }
    if (events & 0x02)
        running_ = !running_;

    const bool left_press = (events & 0x04) && running_;
    const bool right_press = (events & 0x08) && running_;

    // [L01-R2] Simultaneous left+right requests are ambiguous input: refuse
    // BOTH explicitly, do not process left then right.
    if (left_press && right_press) {
        trigger_alarm("conflicting left/right lane requests");
        output.reason = "conflicting left/right lane requests refused";
    } else if (left_press) {
        // [USER-A] The key gate carries the MAIN-light verdict: NONE (no
        // light or round green) proceeds; RED or UNKNOWN blocks.
        if (left_gate_light == P3LightState::RED ||
            left_gate_light == P3LightState::UNKNOWN) {
            trigger_alarm("lane change blocked by red/unknown light");
            output.reason = "lane change blocked by red/unknown light";
        } else if (!lane_fresh) {
            // [LANE-POS 2026-10-23] Stale evidence is not "already at the
            // edge": report the real reason first.
            const char* reason = "lane change evidence stale or unavailable";
            trigger_alarm(reason);
            output.reason = reason;
        } else if (strict_simulated_lane_edge() && lane_position_ == P3LanePosition::LEFT) {
            trigger_alarm("left edge lane");
            output.reason = "already leftmost lane (simulated position; S changes assumed lane)";
        } else if (left_lane_state == LANE_INVALID) {
            trigger_alarm("left lane data invalid");
            output.reason = "left lane data invalid";
        } else if (left_lane_state == LANE_SOLID) {
            trigger_alarm("solid line lane change");
            output.reason = "left solid line blocks lane change";
        } else if (left_lane_state == LANE_DASHED) {
            if (!road_safety) {
                // [L01-R3] No vehicle input is not "checked and clear".
                trigger_alarm("left lane change: vehicle input missing");
                output.reason =
                    "left lane change refused: vehicle input missing";
            } else if (road_safety->left_change !=
                       P3LaneChangeAdvice::NoKnownBlocker) {
                trigger_alarm("left lane change safety evidence incomplete");
                output.reason = std::string("left lane change refused: ") +
                    p3_lane_change_advice_name(road_safety->left_change);
            } else {
                // [L01-R2] Register the request and candidate action only;
                // update() commits the lane position after ALL vetoes merge.
                if (left_request)
                    *left_request = true;
                output.action = P3Action::TURN_LEFT;
            }
        } else {
            trigger_alarm("left lane state not dashed");
            output.reason = "left lane not dashed, change refused";
        }
    } else if (right_press) {
        if (right_gate_light == P3LightState::RED ||
            right_gate_light == P3LightState::UNKNOWN) {
            trigger_alarm("lane change blocked by red/unknown light");
            output.reason = "lane change blocked by red/unknown light";
        } else if (!lane_fresh) {
            const char* reason = "lane change evidence stale or unavailable";
            trigger_alarm(reason);
            output.reason = reason;
        } else if (strict_simulated_lane_edge() && lane_position_ == P3LanePosition::RIGHT) {
            trigger_alarm("right edge lane");
            output.reason = "already rightmost lane (simulated position; S changes assumed lane)";
        } else if (right_lane_state == LANE_INVALID) {
            trigger_alarm("right lane data invalid");
            output.reason = "right lane data invalid";
        } else if (right_lane_state == LANE_SOLID) {
            trigger_alarm("solid line lane change");
            output.reason = "right solid line blocks lane change";
        } else if (right_lane_state == LANE_DASHED) {
            if (!road_safety) {
                trigger_alarm("right lane change: vehicle input missing");
                output.reason =
                    "right lane change refused: vehicle input missing";
            } else if (road_safety->right_change !=
                       P3LaneChangeAdvice::NoKnownBlocker) {
                trigger_alarm("right lane change safety evidence incomplete");
                output.reason = std::string("right lane change refused: ") +
                    p3_lane_change_advice_name(road_safety->right_change);
            } else {
                if (right_request)
                    *right_request = true;
                output.action = P3Action::TURN_RIGHT;
            }
        } else {
            trigger_alarm("right lane state not dashed");
            output.reason = "right lane not dashed, change refused";
        }
    }
    if ((events & 0x10) && light == P3LightState::RED) {
        trigger_alarm("forced drive on red");
        output.reason = "forced drive while red";
    }
}

P3DecisionOutput P3DecisionBridge::update(
    const std::vector<P3Det>& detections, std::uint32_t frame_id,
    const P3LampObservation* lamp, const P3RoadSafetyInput* road_input)
{
    const double now_ms_value = now_ms();
    // [L01-R2] Lane-change requests registered this frame (committed after
    // all vetoes merge).
    bool left_request = false;
    bool right_request = false;
    std::uint32_t processed_keys = 0;
    std::uint32_t decision_keys = 0;
    const std::uint32_t pending_before = pending_lane_key_;

    const P3LightState raw_light =
        classify_light(detections, lane_position_, &light_evidence_,
                       &light_direction_, &light_confidence_, &light_fresh_);
    // [N1-5] 灯证据门禁：按不同成功观察去重投票；证据过期时绿/黄降为
    // UNKNOWN（红灯保持）。无新观察的帧不重复计票（stable_light_ 保持）。
    bool lamp_should_vote = false;
    bool lamp_stale = false;
    bool lamp_invalid_meta = false;
    const P3LightState lamp_verdict = apply_lamp_evidence(
        raw_light, lamp, now_ms_value, light_ttl_ms_, lamp_state_,
        &lamp_should_vote, &lamp_stale, &lamp_invalid_meta);
    if (lamp_should_vote) {
        // Safety evidence must not wait behind four historical green votes.
        // Erase those votes too: recovery requires four NEW successful greens,
        // not one new green plus votes from before the uncertain/red signal.
        if (lamp_verdict == P3LightState::RED ||
            (lamp_verdict == P3LightState::UNKNOWN &&
             stable_light_ != P3LightState::RED)) {
            light_history_.clear();
            light_history_.push_back(lamp_verdict);
            stable_light_ = lamp_verdict;
        } else {
            const P3LightState previous = stable_light_;
            const P3LightState voted = debounce_light(lamp_verdict);
            const bool clear_consensus = light_history_.size() == DEBOUNCE_FRAMES &&
                std::count(light_history_.begin(), light_history_.end(),
                           P3LightState::NONE) >= DEBOUNCE_FRAMES - 1;
            // NONE also means "not enough matching votes", not just no lamp.
            // Do not convert that incomplete recovery into an implicit GO.
            stable_light_ = voted == P3LightState::NONE && !clear_consensus &&
                (previous == P3LightState::UNKNOWN || previous == P3LightState::RED)
                    ? previous : voted;
        }
    }
    // [N1-fix] 证据过期时不得保留旧绿灯/黄灯——过期场景恰恰是"无新观察"
    // （cached/失败/未运行），此路径不投票，必须把降级强制写回稳定状态。
    if (lamp_stale && (stable_light_ == P3LightState::GREEN ||
                       stable_light_ == P3LightState::YELLOW)) {
        stable_light_ = P3LightState::UNKNOWN;
        // [C-N1] 过期/非法后不得保留旧绿票：恢复必须重新集满 4/5。
        light_history_.clear();
    }
    // [C-R3] 非法元数据观察连“保持 NONE”都不允许（NONE 会放行）：只要本帧
    // 有灯观察但时刻非法，稳定态强制 UNKNOWN；红灯保持保守。
    if (lamp_invalid_meta && stable_light_ != P3LightState::RED) {
        stable_light_ = P3LightState::UNKNOWN;
        light_history_.clear();  // [C-N1] 清掉旧绿票，禁止 1 新绿复用
    }

    // [RED-MEMORY-FIX 2026-10-21] Re-arm ONLY on a fresh RED vote. The old
    // branch re-armed whenever the stable state was RED and the decay branch
    // forced RED again, so one red observation latched RED indefinitely in a
    // no-light scene (no-light/unknown/yellow could never clear it). Inside
    // the window the decay branch still holds RED; when the counter reaches
    // zero the next successful observation can clear the state. A real red
    // light re-arms on every new observation.
    const bool red_vote =
        lamp_should_vote && lamp_verdict == P3LightState::RED;
    if (red_vote) {
        red_memory_frames_ = RED_MEMORY_FRAMES;
    } else if (stable_light_ == P3LightState::GREEN) {
        red_memory_frames_ = 0;
    } else if (red_memory_frames_ > 0) {
        --red_memory_frames_;
        stable_light_ = P3LightState::RED;
    }

    P3DecisionOutput output;
    output.light = stable_light_;
    output.light_direction = light_direction_;
    output.light_confidence = light_confidence_;
    output.light_fresh = light_fresh_;
    output.light_evidence = light_evidence_;
    output.light_stale = lamp_stale;  // [N1-5]
    // [L01-R2] output.lane is assigned at the end of the function, after the
    // lane position is atomically committed (same-frame consistency).
    // One TwinLite snapshot drives both the HUD/key gate and the advisor.
    // Cached geometry never refreshes object evidence or the lane timestamp.
    const double auth_now = now_ms_value;
    P3RoadSafetyInput current = road_input ? *road_input : P3RoadSafetyInput{};
    // [C01] Twin 授权窗按**观察源时间**起算，不用消费/决策 now 刷新。
    // 缺失/非有限/未来/时钟不匹配的源时间一律不可用：不刷新缓存，旧缓存
    // 仍按它自己的源时间自然过期。
    constexpr double kSourceFutureEpsMs = 1.0;
    const auto& incoming = current.twinlite_lane;
    const bool twin_source_valid =
        std::isfinite(incoming.source_ms) && incoming.source_ms > 0.0 &&
        std::isfinite(auth_now) &&
        incoming.source_ms <= auth_now + kSourceFutureEpsMs;
    const bool twin_eye_valid = incoming.source_eye == 0;
    // [C-R4] 源身份/顺序：旧 run、旧源时间不得覆盖更新缓存；同一观察的
    // 重复消费幂等（同 run + 同时间）。未实现 eye/run 的旧包一律拒绝。
    const bool cache_empty = twin_lane_cache_.source_ms <= 0.0;
    // 新 run 必须同时不倒退源时间；同 run 只能是同一观察（幂等）；
    // 旧 run、同 run 换新时间、倒退时间一律拒绝。
    const bool twin_order_ok =
        cache_empty ||
        (incoming.source_run_id > twin_lane_cache_.source_run_id &&
         incoming.source_ms >= twin_lane_cache_.source_ms) ||
        (incoming.source_run_id == twin_lane_cache_.source_run_id &&
         incoming.source_ms == twin_lane_cache_.source_ms &&
         same_twin_payload(incoming, twin_lane_cache_));
    if (road_input && incoming.fresh && incoming.valid && twin_source_valid &&
        twin_eye_valid && twin_order_ok) {
        twin_lane_cache_ = incoming;
        twin_auth_last_ms_ = incoming.source_ms;
        twin_auth_left_ = twin_auth_state(twin_lane_cache_.left.marking);
        twin_auth_right_ = twin_auth_state(twin_lane_cache_.right.marking);
    }
    const bool auth_fresh = twin_auth_window_fresh(twin_auth_last_ms_, auth_now);
    // [LANE-POS 2026-10-23] A route/scene switch drops the lane pair for a
    // while; holding the old simulated position then refused legal changes
    // ("already leftmost/rightmost"). Reset to STRAIGHT after a gap.
    if (auth_fresh) last_lane_evidence_ms_ = auth_now;
    if (lane_pos_reset_s_ > 0.0 &&
        lane_position_ != P3LanePosition::STRAIGHT &&
        last_lane_evidence_ms_ > 0.0 &&
        auth_now - last_lane_evidence_ms_ > lane_pos_reset_s_ * 1000.0) {
        lane_position_ = P3LanePosition::STRAIGHT;
        last_lane_evidence_ms_ = auth_now;
                std::printf("[lane-pos] auto-reset to STRAIGHT after %.1f s without lane evidence\n",
                    lane_pos_reset_s_);
    }
    current.twinlite_lane = auth_fresh ? twin_lane_cache_ : P3RoadLanePair{};
    // [C-R1] 顾问的“当前时间”必须是决策 now，不是当前帧采集时间。
    current.decision_now_ms = auth_now;
    if (road_input)
        output.road_safety = assess_p3_road_safety(current);
    const int auth_left_state = auth_fresh ? twin_auth_left_ : LANE_NONE;
    const int auth_right_state = auth_fresh ? twin_auth_right_ : LANE_NONE;
    output.left_lane_state = auth_left_state;
    output.right_lane_state = auth_right_state;
    output.lane_fresh = auth_fresh;
    output.twin_pair_valid = auth_fresh && twin_lane_cache_.valid;
    output.lane_age_ms = twin_auth_last_ms_ > 0.0
        ? auth_now - twin_auth_last_ms_ : -1.0;

    // [TWIN-AUTH/P2] Verified per-direction light states for the lane-select
    // main-light re-derivation. The channels run the full evidence contract;
    // the re-derivation never reads a raw frame verdict.
    update_direction_channels(detections, lamp, auth_now);
    const bool any_light_dets = light_fresh_;
    const auto gate_of = [any_light_dets](P3LightState s) {
        if (s == P3LightState::NONE && any_light_dets)
            return P3LightState::UNKNOWN;
        return s;
    };
    const P3LightState raw_gate_left = gate_of(left_channel_.stable);
    const P3LightState raw_gate_right = gate_of(right_channel_.stable);
    // [USER-A 2026-10-22] A/D target gate: a verified RED arrow for the
    // target still blocks; otherwise the MAIN light color decides (user
    // policy: "ignore arrows"): main NONE/round GREEN releases, main RED
    // blocks, YELLOW/UNKNOWN stays fail-closed. This removes the old
    // "channel NONE while any light exists -> UNKNOWN" veto that blocked
    // lane changes in light-less scenes with an irrelevant/false detection.
    const auto key_gate_of = [main = stable_light_](P3LightState channel) {
        if (channel == P3LightState::RED)
            return P3LightState::RED;
        // [USER-A] A target-direction yellow arrow still demotes the turn to
        // SLOW in the final veto; only NONE/UNKNOWN directions are ignored.
        if (channel == P3LightState::YELLOW)
            return P3LightState::YELLOW;
        if (main == P3LightState::NONE || main == P3LightState::GREEN)
            return P3LightState::NONE;
        if (main == P3LightState::RED)
            return P3LightState::RED;
        return P3LightState::UNKNOWN;
    };
    const P3LightState key_gate_left = key_gate_of(raw_gate_left);
    const P3LightState key_gate_right = key_gate_of(raw_gate_right);
    const P3LanePosition position_before_keys = lane_position_;

    // [N1-fix] 无 BAR 映射（未 open / 桌面集成测试）时跳过事件与 ABI 校验，
    // 不进行空指针寄存器访问；灯/车道/红记忆等纯逻辑路径保持可用。
    // 注意：这是"ABI 不适用"，不是"ABI 校验失败"（后者仍走保守分支）。
    std::string abi_error;
    if (bar0_.data() != nullptr) {
        const std::uint32_t ctrl =
            *reinterpret_cast<volatile std::uint32_t*>(bar0_.data() + REG_CTRL);
        const std::uint32_t status =
            *reinterpret_cast<volatile std::uint32_t*>(bar0_.data() + REG_STATUS);
        const std::uint32_t events =
            *reinterpret_cast<volatile std::uint32_t*>(bar0_.data() + REG_EVENT);
        const std::uint32_t inject =
            *reinterpret_cast<volatile std::uint32_t*>(bar0_.data() + REG_INJECT);
        output.decision_abi_valid =
            decision_abi_matches(ctrl, status, events, inject, &abi_error);
        if (output.decision_abi_valid) {
            // W1C only after confirming that these offsets implement the
            // expected register map. Repeated/stuck high levels are handled
            // once, and only after a low read re-arms that event bit.
            const std::uint32_t event_bits = events & EVENT_MASK;
            const std::uint32_t filtered_events =
                filter_sticky_events(event_bits, rearmed_key_events_);
            // Preserve a simultaneous raw A+D conflict even when one bit
            // was disarmed by the preceding poll. Do not turn a chord into
            // the opposite single-key request through sticky filtering.
            const std::uint32_t fresh_events =
                (event_bits & 0x0C) == 0x0C && (filtered_events & 0x0C)
                    ? filtered_events | 0x0C : filtered_events;
            processed_keys = fresh_events;
            if (event_bits)
                *reinterpret_cast<volatile std::uint32_t*>(bar0_.data() +
                                                           REG_EVENT) = event_bits;
            if (event_bits) {
                const std::uint32_t after_clear =
                    *reinterpret_cast<volatile std::uint32_t*>(bar0_.data() + REG_EVENT);
                rearm_acknowledged_events(event_bits, after_clear, rearmed_key_events_);
            }
            decision_keys = select_lane_request(fresh_events, auth_now,
                key_gate_left, key_gate_right, output);
            // [L01-R3] Always pass road_safety (default Unknown assessment
            // without vehicle input): missing vehicle input must take the
            // explicit refusal branch, never be skipped as a null pointer.
            process_key_events(
                decision_keys, stable_light_, key_gate_left, key_gate_right,
                auth_left_state, auth_right_state, auth_fresh,
                &left_request, &right_request, output, &output.road_safety);
        }
    } else {
        output.decision_abi_valid = false;
    }
    if (!output.decision_abi_valid && pending_lane_key_) {
        key_feedback_ = pending_lane_key_ == 4 ? "last A: " : "last D: ";
        key_feedback_ += "cancelled: decision ABI unavailable";
        key_feedback_until_ms_ = now_ms_value + 3000.0;
        pending_lane_key_ = 0;
    }

    // [TWIN-AUTH/P2] Same-frame light re-derivation after a position change
    // (S lane select or a committed turn): adopt the TARGET direction's
    // verified channel verdict, isolate the old direction's vote history and
    // mirror the red memory.
    if (lane_position_ != position_before_keys) {
        // [USER-A 2026-10-22] The post-select main light follows the same
        // policy: the verified target RED arrow is kept; otherwise the prior
        // main light (none/round green) wins over an irrelevant/false
        // direction, so a granted turn is not re-blocked by an unused arrow.
        const P3LightState target =
            lane_position_ == P3LanePosition::LEFT
                ? key_gate_left
                : lane_position_ == P3LanePosition::RIGHT
                      ? key_gate_right
                      : key_gate_of(gate_of(straight_channel_.stable));
        light_history_.clear();
        if (target != P3LightState::NONE)
            light_history_.push_back(target);
        stable_light_ = target;
        red_memory_frames_ =
            target == P3LightState::RED ? RED_MEMORY_FRAMES : 0;
        output.light = stable_light_;
    }

    if (alarm_hold_frames_ > 0) {
        --alarm_hold_frames_;
        alarm_ = true;
        output.alarm = true;
        output.reason = alarm_reason_;
    } else {
        alarm_ = false;
        alarm_reason_.clear();
    }

    if (output.action == P3Action::GO) {
        output.action = apply_light_to_action(output.action, stable_light_,
                                              running_, &output.reason);
    } else if (running_ &&
               (output.action == P3Action::TURN_LEFT ||
                output.action == P3Action::TURN_RIGHT)) {
        // [L01-R2/P9] Yellow SLOW semantics take priority over a lane change
        // request (the turn is not allowed to override it). The current
        // direction's stable light alone is not enough: a goForward green
        // with a warningLeft arrow leaves the current state GREEN while the
        // TARGET direction's verified channel is YELLOW, so the target
        // verdict is merged here as well, before the atomic commit.
        const P3LightState target_gate =
            output.action == P3Action::TURN_LEFT ? key_gate_left
                                                 : key_gate_right;
        if (stable_light_ == P3LightState::YELLOW ||
            target_gate == P3LightState::YELLOW) {
            output.action = P3Action::SLOW;
            if (output.reason.empty())
                output.reason = "yellow light priority over lane change";
        }
    }

    // [L01-R2] Decide first, commit last: a front STOP/SLOW overrides a lane
    // change request; the lane position commits only when the merged result
    // is still the turn, so refusals never move the simulated position.
    if (road_input && output.road_safety.front_action == P3RoadAction::Stop &&
        output.action != P3Action::STOP) {
        output.action = P3Action::STOP;
        output.reason = output.road_safety.reason;
    } else if (road_input &&
               output.road_safety.front_action == P3RoadAction::SlowDown &&
               (output.action == P3Action::GO ||
                output.action == P3Action::TURN_LEFT ||
                output.action == P3Action::TURN_RIGHT)) {
        output.action = P3Action::SLOW;
        output.reason = output.road_safety.reason;
    }

    // [L01-R2/TWIN-AUTH] Final light-constraint backstop: a committed turn
    // must never survive a red/unknown light, even if a caller raced the key
    // path. [USER-A 2026-10-22] Uses the same target-direction key gate as
    // the request, so an irrelevant/false arrow cannot re-block a granted
    // turn while a verified target red still does.
    if (running_ &&
        (output.action == P3Action::TURN_LEFT ||
         output.action == P3Action::TURN_RIGHT)) {
        const P3LightState gate_light =
            output.action == P3Action::TURN_LEFT ? key_gate_left
                                                 : key_gate_right;
        if (gate_light == P3LightState::RED ||
            gate_light == P3LightState::UNKNOWN) {
            output.action = P3Action::STOP;
            if (output.reason.empty())
                output.reason = "turn blocked by red/unknown light";
        }
    }

    // [L01-R2] Atomic commit of the simulated lane position after ALL vetoes.
    if (left_request && output.action == P3Action::TURN_LEFT) {
        lane_position_ = static_cast<P3LanePosition>(std::max(0, static_cast<int>(lane_position_) - 1));
    } else if (right_request && output.action == P3Action::TURN_RIGHT) {
        lane_position_ = static_cast<P3LanePosition>(std::min(2, static_cast<int>(lane_position_) + 1));
    }
    output.lane = lane_position_;

    output.led_left = output.action == P3Action::TURN_LEFT;
    output.led_right = output.action == P3Action::TURN_RIGHT;
    output.led_middle =
        output.action == P3Action::GO || output.action == P3Action::STOP ||
        output.action == P3Action::SLOW;
    output.alarm = output.alarm || alarm_;
    if (bar0_.data() != nullptr && !output.decision_abi_valid) {
        // ABI 校验失败（BAR 已映射但寄存器不符合预期）：软件保守 STOP，
        // 该 STOP 仅诊断用，绝不写未经验证的硬件映射。
        output.action = P3Action::STOP;
        output.alarm = true;
        output.led_left = false;
        output.led_middle = true;
        output.led_right = false;
        output.reason = abi_error;
    }
    if (decision_keys & 0x0C) {
        const bool both = (decision_keys & 0x0C) == 0x0C;
        const bool left = (decision_keys & 0x04) != 0;
        const bool granted = !both && ((left_request && output.action == P3Action::TURN_LEFT) ||
                                      (right_request && output.action == P3Action::TURN_RIGHT));
        key_feedback_ = both ? "last A+D: " : left ? "last A: " : "last D: ";
        if (granted)
            key_feedback_ += left ? "granted left change" : "granted right change";
        else
            key_feedback_ += std::string("refused: ") + (!running_ ? "simulation paused" :
                output.reason.empty() ? "final safety veto" : output.reason);
        key_feedback_until_ms_ = now_ms_value + 3000.0;
        std::printf("[key-feedback] frame=%u events=0x%02x action=%d lane=%d left_gate=%d right_gate=%d "
                    "twin_fresh=%d types=(%d,%d) result=%s\n", frame_id, decision_keys,
                    int(output.action), int(output.lane), int(raw_gate_left), int(raw_gate_right),
                    int(output.lane_fresh), output.left_lane_state, output.right_lane_state,
                    key_feedback_.c_str());
    }
    if (!key_feedback_.empty() && now_ms_value <= key_feedback_until_ms_) {
        output.reason = key_feedback_ + (output.reason.empty() ? "" : " | now: " + output.reason);
    }
    if (pending_before != pending_lane_key_ || (processed_keys & 0x1F) || (decision_keys & 0x0C)) {
        std::printf("[key-request] frame=%u input=0x%02x evaluated=0x%02x pending=0x%02x deadline_ms=%.1f result=%s\n",
                    frame_id, processed_keys, decision_keys, pending_lane_key_,
                    pending_lane_deadline_ms_, key_feedback_.c_str());
    }
    write_output(output, frame_id);
    // Preserve the v9 diagnostic edge for a light-only transition.
    static int log_last_light = -1;
    const bool light_changed = static_cast<int>(output.light) != log_last_light;
    const bool state_changed =
        output.action != logged_action_ || output.lane != logged_lane_ ||
        output.alarm != logged_alarm_;
    if (state_changed || light_changed || (processed_keys & 0x0C) ||
        (decision_keys & 0x0C) || pending_before != pending_lane_key_ || (frame_id % 30) == 0) {
        std::printf(
            "[decision] frame=%u action=%d light=%d stale=%d dir=%s conf=%.2f "
            "lf=%d lane=%d lane_source=twinlite types=(%d,%d) "
            "twin_fresh=%d twin_age_ms=%.1f abi=%d alarm=%d led=(%d,%d,%d) "
            "light_ev=[%s] reason=%s\n",
            frame_id, static_cast<int>(output.action),
            static_cast<int>(output.light), output.light_stale ? 1 : 0,
            p3_light::direction_name(
                static_cast<p3_light::Direction>(output.light_direction)),
            output.light_confidence, output.light_fresh ? 1 : 0,
            static_cast<int>(output.lane),
            output.left_lane_state, output.right_lane_state,
            output.lane_fresh ? 1 : 0, output.lane_age_ms,
            output.decision_abi_valid ? 1 : 0,
            output.alarm ? 1 : 0, output.led_left ? 1 : 0,
            output.led_middle ? 1 : 0, output.led_right ? 1 : 0,
            output.light_evidence.c_str(),
            output.reason.empty() ? "-" : output.reason.c_str());
        logged_action_ = output.action;
        logged_lane_ = output.lane;
        logged_alarm_ = output.alarm;
        log_last_light = static_cast<int>(output.light);
    }
    return output;
}

void P3DecisionBridge::write_output(const P3DecisionOutput& output,
                                    std::uint32_t frame_id)
{
    if (!output.decision_abi_valid || !bar0_.is_open())
        return;
    const std::uint32_t ctrl =
        ((frame_id & 0xFFFF) << 16) |
        ((output.alarm ? 1U : 0U) << 3) |
        ((output.led_middle ? 1U : 0U) << 2) |
        ((output.led_right ? 1U : 0U) << 1) |
        (output.led_left ? 1U : 0U);
    *reinterpret_cast<volatile std::uint32_t*>(bar0_.data() + REG_CTRL) = ctrl;
}
