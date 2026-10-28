#ifndef P3_LANE_STATE_H
#define P3_LANE_STATE_H

// Source-neutral semantic states. The production lane source is TwinLite;
// these constants do not read or imply the retired RTL lane register ABI.
enum LaneState : int {
    LANE_NONE = 0,
    LANE_DASHED = 1,
    LANE_SOLID = 2,
    LANE_INVALID = 3
};

#endif  // P3_LANE_STATE_H
