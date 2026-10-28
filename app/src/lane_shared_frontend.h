// lane_shared_frontend.h - shared read-only per-update lane frontend (R2 relay,
// 2026-10-16, follows CODEX duplication_review.md).
//
// Extracts ONCE per update the two duplicated CPU products:
//   * the mask -> raw-geometry candidate pool. The extraction function bodies
//     are identical in the production source (twinlite_ego_lane_postprocess.cpp
//     b929e148...) and in the wrapped repair source (candidate_postprocess.cpp
//     0c2c02df..., packaged copies in this window package);
//   * the BGR2GRAY of THIS call's left-eye pixels.
// Branch-specific work (HSV, three-state scan, frozen classifiers, pair
// selection, identity, local segments, drawing) stays in the branches.
//
// Contract (her review, enforced by the desktop dual-path gate):
//   * lifetime: exactly one update() call; NO static state, no caching, no
//     reuse across deliveries — every build reads the current pixels;
//   * the pool is READ-ONLY to consumers: each branch makes its own copies
//     with explicit per-field conversion (never reinterpret_cast / layout
//     assumptions);
//   * guard parity: raw_valid mirrors the frozen production mask guard;
//     gray_valid mirrors the production-permissive left-BGR guard; the repair
//     branch keeps its own 640x360 requirement on its side.
#pragma once

#include <opencv2/opencv.hpp>

#include <vector>

#include "twinlite_ego_lane_postprocess.h"   // global EgoLaneCandidate/EgoLanePoint

namespace lane_shared {

struct SharedInputs {
    bool raw_valid = false;    // mask passed the frozen production guard
    bool gray_valid = false;   // left frames passed the production-permissive guard
    cv::Mat gray;              // CV_8UC1 of THIS call's left_bgr
    cv::Mat left;              // shallow ref to THIS call's left_bgr (branch-local HSV input)
    std::vector<EgoLaneCandidate> raw;   // full pool, frozen extraction order
};

// Build once per update from the CURRENT pixels; empty/invalid inputs leave the
// corresponding *_valid false (consumers keep their own original guards).
SharedInputs build(const cv::Mat& lane_mask, const cv::Mat& left_bgr);

}  // namespace lane_shared
