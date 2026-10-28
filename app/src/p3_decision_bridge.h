#ifndef P3_DECISION_BRIDGE_H
#define P3_DECISION_BRIDGE_H

#include "p3_lane_state.h"
#include "light_classes.h"
#include "p3_engine.h"
#include "p3_road_safety_advisor.h"
#include "pci_bar0.h"

#include <cstdint>
#include <deque>
#include <string>

enum class P3LightState : int {
    NONE = 0,
    RED = 1,
    GREEN = 2,
    YELLOW = 3,
    // 观测到灯目标但无法判定其对当前车道方向的放行语义（方向未知 /
    // 颜色复核产生了训练表外的组合）。按 fail-closed 保守停车。
    UNKNOWN = 4,
};

enum class P3LanePosition : int {
    LEFT = 0,
    STRAIGHT = 1,
    RIGHT = 2,
};

enum class P3Action : int {
    GO = 0,
    STOP = 1,
    SLOW = 2,
    TURN_LEFT = 3,
    TURN_RIGHT = 4,
};

struct P3DecisionOutput {
    P3Action action = P3Action::GO;
    P3LightState light = P3LightState::NONE;
    // 灯信号诊断：主导方向（p3_light::Direction）、相关信号的最大置信度、
    // 本帧是否有灯检测（freshness）以及合成证据字符串。
    int light_direction = p3_light::DIR_UNKNOWN;
    float light_confidence = 0.0f;
    bool light_fresh = false;
    std::string light_evidence;
    P3LanePosition lane = P3LanePosition::STRAIGHT;
    int left_lane_state = LANE_NONE;
    int right_lane_state = LANE_NONE;
    // TwinLite-only authorization snapshot, never RTL diagnostics.
    bool lane_fresh = false;
    bool twin_pair_valid = false;
    double lane_age_ms = -1.0;  // -1: no successful Twin pair yet
    bool decision_abi_valid = false;
    P3RoadSafetyAssessment road_safety;
    // [N1-5] 灯证据 TTL 判定：true = 当前无足够新的成功灯观察，
    // 绿灯/黄灯证据已过期（不得据此放行）。
    bool light_stale = false;
    bool alarm = false;
    bool led_left = false;
    bool led_middle = true;
    bool led_right = false;
    std::string reason;
};

class P3DecisionBridge {
public:
    P3DecisionBridge();

    bool open(const std::string& bdf, std::string* error = nullptr);
    // Clear observation histories after warmup; preserve user lane/running state.
    void reset_state();
    // [N1-5] 可选传入灯观察快照（引擎每帧提供）：用于观察去重投票与证据 TTL。
    P3DecisionOutput update(const std::vector<P3Det>& detections,
                            std::uint32_t frame_id,
                            const P3LampObservation* lamp = nullptr,
                            const P3RoadSafetyInput* road_input = nullptr);
    // [N1-5] 桌面测试注入假时钟（毫秒，单调）。nullptr 恢复默认 steady 时钟。
    void set_now_fn_for_test(double (*fn)()) { now_ms_fn_ = fn; }

    // Test-only access to the private decision path.
    friend class P3DecisionBridgeAccess;

private:
    // [N1-5] 灯证据状态机（纯函数，供假时钟/去重测试直连）：
    //   - 只有 status==1 且 run_id 变化时才允许投票（同一次观察不重复计票）；
    //   - 最近成功观察的采集时刻超过 ttl 后，GREEN/YELLOW 降为 UNKNOWN
    //     （红灯不降——维持最保守状态）；无任何成功观察时不触发（正常无灯道路）。
    struct LampEvidenceState {
        std::uint64_t last_voted_run = 0;
        double last_success_capture_ms = 0;
        bool has_success = false;
    };
    static P3LightState apply_lamp_evidence(
        P3LightState composed, const P3LampObservation* obs, double now_ms,
        double ttl_ms, LampEvidenceState& state, bool* should_vote,
        bool* stale_out, bool* invalid_meta_out = nullptr);

    // [N1-5] 当前时刻（毫秒）；默认 steady_clock，测试可注入假时钟。
    double now_ms() const;

    static P3LightState classify_light(const std::vector<P3Det>& detections,
                                       P3LanePosition lane,
                                       std::string* evidence = nullptr,
                                       int* dominant_dir = nullptr,
                                       float* confidence = nullptr,
                                       bool* light_fresh = nullptr);
    // 灯语义 → 动作映射（纯函数，供桌面回归直接验证）。
    static P3Action apply_light_to_action(P3Action action, P3LightState light,
                                          bool running, std::string* reason);
    static bool decision_abi_matches(std::uint32_t ctrl,
                                     std::uint32_t status,
                                     std::uint32_t events,
                                     std::uint32_t inject,
                                     std::string* error);
    static std::uint32_t filter_sticky_events(std::uint32_t events,
                                              std::uint32_t& rearmed);
    static void rearm_acknowledged_events(std::uint32_t cleared,
                                          std::uint32_t after_clear,
                                          std::uint32_t& rearmed);
    // Operator intent only, not a held permit. Retry the ordinary final gates.
    std::uint32_t select_lane_request(std::uint32_t events, double now,
                                      P3LightState left_gate,
                                      P3LightState right_gate,
                                      const P3DecisionOutput& snapshot);
    // [TWIN-AUTH] Authorization-view helpers (test-visible via the friend).
    static int twin_auth_state(P3RoadMarking marking);
    static bool twin_auth_window_fresh(double last_ms, double now_ms);
    // [TWIN-AUTH/P2] Per-direction light evidence channels. Each channel
    // runs the same contract as the main state: one vote per successful
    // observation run, immediate red/unknown, 4-of-5 green debounce, TTL
    // downgrade, red memory. The key gate reads only the target channel.
    struct DirectionLightChannel {
        std::deque<P3LightState> history;
        P3LightState stable = P3LightState::NONE;
        int red_memory = 0;
        std::uint64_t last_voted_run = 0;
        double last_success_capture_ms = 0.0;
        bool has_success = false;
        P3LightState apply(P3LightState raw, const P3LampObservation* obs,
                           double now, double ttl);
    };
    void update_direction_channels(const std::vector<P3Det>& detections,
                                   const P3LampObservation* lamp,
                                   double now_ms_value);
    DirectionLightChannel left_channel_;
    DirectionLightChannel straight_channel_;
    DirectionLightChannel right_channel_;
    // 400 ms: covers the 2-frame model cadence at the measured
    // 7.4-7.7 fps (~260 ms) plus one frame of jitter.
    static constexpr double kTwinAuthHoldMs = 400.0;
    P3LightState debounce_light(P3LightState raw);
    // [TWIN-AUTH] left/right_lane_state and lane_fresh carry the TwinLite
    // authorization view (fresh DASHED permits, SOLID refuses, else closed).
    // left/right_gate_light are classified for the TARGET lane direction so a
    // turn can never ride the old direction's green. Requests are registered
    // here; position commits in update() only after all vetoes merge.
    void process_key_events(std::uint32_t events, P3LightState light,
                            P3LightState left_gate_light,
                            P3LightState right_gate_light,
                            int left_lane_state, int right_lane_state,
                            bool lane_fresh, bool* left_request,
                            bool* right_request, P3DecisionOutput& output,
                            const P3RoadSafetyAssessment* road_safety = nullptr);
    void trigger_alarm(const std::string& reason);
    void write_output(const P3DecisionOutput& output, std::uint32_t frame_id);

    PciBar0Mapper bar0_;
    std::deque<P3LightState> light_history_;
    P3LightState stable_light_ = P3LightState::NONE;
    P3LanePosition lane_position_ = P3LanePosition::STRAIGHT;
    int red_memory_frames_ = 0;
    int alarm_hold_frames_ = 0;
    bool alarm_ = false;
    bool running_ = true;
    std::string alarm_reason_;
    // Display/log feedback only. It never changes action, lane or permits.
    std::string key_feedback_;
    double key_feedback_until_ms_ = 0.0;
    // [LANE-POS 2026-10-23] scene-cut auto reset of the simulated position.
    double lane_pos_reset_s_ = 3.0;   // P3_LANE_POS_RESET_S, <=0 disables
    double last_lane_evidence_ms_ = 0.0;
    std::uint32_t pending_lane_key_ = 0;
    double pending_lane_started_ms_ = 0.0;
    double pending_lane_deadline_ms_ = 0.0;
    P3Action logged_action_ = P3Action::GO;
    P3LanePosition logged_lane_ = P3LanePosition::STRAIGHT;
    bool logged_alarm_ = false;
    // 灯信号诊断缓存（每次 update 刷新，供输出与日志）。
    std::string light_evidence_;
    int light_direction_ = p3_light::DIR_UNKNOWN;
    float light_confidence_ = 0.0f;
    bool light_fresh_ = false;
    // [N1-5] 灯证据状态与 TTL（毫秒；台架工程值，测试可注入假时钟）。
    LampEvidenceState lamp_state_;
    // [LIGHT-TTL 2026-10-22] 300 -> 500 ms. Source-time age envelope at the
    // decision point: 2-frame light refresh (~290 ms at 7 FPS) + capture ->
    // decision latency (~270 ms). 300 ms marked every alternate frame stale
    // and forced a GREEN->UNKNOWN flicker. Keep in sync with
    // perf_config.h::light_ttl_ms (this member is the value actually used).
    double light_ttl_ms_ = 500.0;
    double (*now_ms_fn_)() = nullptr;  // nullptr = 默认 steady 时钟
    // A sticky event bit is handled once, then re-armed only after observing
    // it low. Bits that are already high at startup are discarded as stale.
    std::uint32_t rearmed_key_events_ = 0;
    // TwinLite is the only lane source; legacy RTL authorization is retired.
    int twin_auth_left_ = LANE_NONE;
    int twin_auth_right_ = LANE_NONE;
    double twin_auth_last_ms_ = 0.0;
    // [TWIN-AUTH] Last fresh TwinLite pair, shared by the key gate and the
    // advisor so both act on the same snapshot at the same age.
    P3RoadLanePair twin_lane_cache_;
};

#endif  // P3_DECISION_BRIDGE_H
