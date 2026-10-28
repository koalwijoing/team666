#ifndef LANE_MONITOR_H
#define LANE_MONITOR_H

#include <cstdint>

enum LaneState : int {
    LANE_NONE = 0,
    LANE_DASHED = 1,
    LANE_SOLID = 2,
    LANE_INVALID = 3
};

struct LaneSample {
    std::uint32_t seq = 0;
    std::uint32_t frame_id = 0;
    int left_state = LANE_NONE;
    int right_state = LANE_NONE;
    float left_a = 0.0f;
    float left_b = 0.0f;
    float right_a = 0.0f;
    float right_b = 0.0f;
    bool left_fit_valid = false;
    bool right_fit_valid = false;
    bool busy = false;
    bool protocol_error = false;
};

class LaneMonitor {
public:
    void attach(volatile std::uint8_t* bar_base) { bar_ = bar_base; }
    bool attached() const { return bar_ != nullptr; }

    bool read_sample(LaneSample& sample);
    // Calibrated 2026-10-11: threshold 3 because the daytime room light
    // drops the camera exposure and the road video's lane-mark gradients
    // fall below the original 30 (3 verified to lock the left divider).
    void enable(std::uint16_t threshold = 3, std::uint16_t expected_left = 342,
                std::uint16_t expected_right = 895, std::uint16_t window = 48);
    void soft_reset();

private:
    volatile std::uint8_t* bar_ = nullptr;
    std::uint32_t rd32(std::uint16_t offset);
    void wr32(std::uint16_t offset, std::uint32_t value);
};

#endif  // LANE_MONITOR_H
