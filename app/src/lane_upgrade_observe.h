// lane_upgrade_observe.h - OBSERVE-ONLY display-upgrade probe for the route-A
// board window (2026-10-16).
//
// Architecture (same discipline as veto_observe): inserted with TWO call lines
// into TwinliteLaneModel (update/draw) of an ISOLATED tree. The production
// analysis result is passed in READ-ONLY and is never modified; the probe
// computes, side-by-side on the same mask/frame:
//     repaired candidate postprocess (cand_observe, frozen 0c2c02df)
//       -> per-candidate local segments (local_segments_mirror, frozen contract)
//       -> identity pair (n05b adapter -> N05-A selector mirror)
// draws NEW overlay items on the left eye (amber = untyped paint evidence,
// yellow/purple = local SOLID/DASHED evidence). Only observed paint rows are
// drawn, at x(y); clear and unobservable rows are never joined or filled.
//
// Guarantees:
//   * INERT unless P3_LANE_UPGRADE=1;
//   * READ-ONLY on the pipeline (const refs; nothing upstream changes; the
//     only writes are the CSV and the new overlay pixels);
//   * FAIL-SAFE: any internal error disables the probe with one stderr note;
//     it never propagates into the pipeline;
//   * NO authority fields, NO decision use: the identity result is displayed
//     as diagnostics only (authority fields are constantly false by contract).
#ifndef LANE_UPGRADE_OBSERVE_H
#define LANE_UPGRADE_OBSERVE_H

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "candidate_postprocess.h"     // namespace-wrapped repaired source (cand_observe)
#include "local_segments_mirror.h"     // route-A component 1 mirror
#include "mirror_adapter.h"            // n05b adapter (types qualified to cand_observe)
#include "selector_mirror.h"           // N05-A identity mirror

namespace lane_upgrade_observe {

struct DrawSeg {
    int y0 = 0, y1 = 0;    // work rows (0..359)
    int x = 0;             // work cols in the 640-wide left eye
    bool typed = false;
    std::string label;     // SOLID / DASHED when typed
    bool single_stroke = false;
    std::vector<cv::Point> paint_points;
};

struct State {
    bool inited = false;
    bool enabled = false;
    bool draw_on = true;
    std::string csv_path;
    std::ofstream csv;
    int logged = 0;
    int max_rows = 4000;
    // last observation (for display)
    bool fresh = false;
    unsigned long long last_id = 0, drawn_id = 0;
    double repaired_ms = 0.0, segments_ms = 0.0, total_ms = 0.0;
    int n_candidates = 0, n_paint_segs = 0, n_typed = 0;
    int id_left = -1, id_right = -1;
    bool abstain = true;
    std::string id_reason;
    std::string typed_labels;
    std::vector<DrawSeg> segs;
};

inline State& st() {
    static State s;
    return s;
}

inline void ensure_init() {
    State& s = st();
    if (s.inited) return;
    s.inited = true;
    const char* env = std::getenv("P3_LANE_UPGRADE");
    s.enabled = env && std::atoi(env) == 1;
    if (!s.enabled) return;
    const char* csv = std::getenv("P3_LANE_UPGRADE_CSV");
    s.csv_path = (csv && *csv) ? csv : "/dev/null";
    if (const char* n = std::getenv("P3_LANE_UPGRADE_MAX_ROWS"))
        s.max_rows = std::max(1, std::atoi(n));
    if (const char* d = std::getenv("P3_LANE_UPGRADE_DRAW"))
        s.draw_on = std::atoi(d) != 0;
    s.csv.open(s.csv_path.c_str(), std::ios::out);
    if (!s.csv) throw std::runtime_error("lane-upgrade: cannot open CSV");
    s.csv << "delivery_id,repaired_ms,segments_ms,total_ms,candidates,"
             "paint_segs,typed,id_left,id_right,abstain,id_reason,typed_labels\n";
    s.csv.flush();
}

// x in the 640-wide left-eye work frame for a work row, via the frozen
// z-convention (mask 512x288 -> work 640x360, sx = sy = 1.25).
inline int work_x_at(const cand_observe::EgoLaneCandidate& c, int y_work) {
    const double sx = 640.0 / 512.0, sy = 360.0 / 288.0;
    return static_cast<int>(std::nearbyint(c.x_at(y_work / sy) * sx));
}

inline DrawSeg make_draw_segment(const cand_observe::EgoLaneCandidateResult& e,
                                int y0, int y1, const std::string& label,
                                bool single_stroke) {
    DrawSeg d;
    d.y0 = y0;
    d.y1 = y1;
    d.typed = !label.empty();
    d.label = label;
    d.single_stroke = single_stroke;
    for (const auto& row : e.evidence_rows) {
        if (row.y < y0 || row.y > y1 ||
            row.state != cand_observe::EgoLaneEvidenceState::Paint) continue;
        const int x = work_x_at(e.candidate, row.y);
        if (row.y >= 0 && row.y < 360 && x >= 0 && x < 640)
            d.paint_points.emplace_back(x, row.y);
    }
    if (!d.paint_points.empty())
        d.x = d.paint_points[d.paint_points.size() / 2].x;
    return d;
}

// A horizontal stroke for ONE evidenced row, never a vertical rectangle or a
// polyline across missing/clear rows. Labels are separate diagnostic text.
inline void draw_segment_pixels(cv::Mat& frame, const DrawSeg& d, bool labels) {
    if (frame.empty() || frame.type() != CV_8UC3 || d.paint_points.empty()) return;
    const cv::Scalar color = !d.typed ? cv::Scalar(0, 166, 255)
        : d.label == "SOLID" ? cv::Scalar(0, 255, 255)
                             : cv::Scalar(180, 0, 255);
    const int half_width = d.typed ? 2 : 1;
    for (const auto& p : d.paint_points) {
        if (p.y < 0 || p.y >= frame.rows || p.x < 0 || p.x >= frame.cols) continue;
        cv::line(frame, cv::Point(std::max(0, p.x - half_width), p.y),
                 cv::Point(std::min(frame.cols - 1, p.x + half_width), p.y),
                 color, 1, cv::LINE_8);
    }
    if (labels && d.typed) {
        const auto& p = d.paint_points.front();
        if (p.y > 12) {
            const std::string text = "LOCAL " + d.label;
            cv::putText(frame, text, cv::Point(std::max(0, p.x - 14), p.y - 3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.30, color, 1);
        }
    }
}

// The production result parameter is a template so the probe works whether the
// caller's type is the production EgoLanePostprocessResult or the
// namespace-wrapped cand_observe one; it is read-only either way.
// R2 relay: the body after the repair analysis moved into set_from_result_impl
// so the pre-computed shared-path repair result can reuse it unchanged.
inline void set_from_result_impl(const cand_observe::EgoLanePostprocessResult& rep,
                                 unsigned long long delivery_id,
                                 const std::chrono::steady_clock::time_point& t0,
                                 double rep_ms) {
    State& s = st();
    if (!s.enabled || s.logged >= s.max_rows) return;
    const auto t2 = std::chrono::steady_clock::now();

    s.segs.clear();
    s.n_candidates = static_cast<int>(rep.candidates.size());
    s.n_paint_segs = 0;
    s.n_typed = 0;
    s.typed_labels.clear();
    for (size_t ci = 0; ci < rep.candidates.size(); ++ci) {
        const auto& e = rep.candidates[ci];
        std::vector<local_segments_mirror::EvidenceRow> rows;
        rows.reserve(e.evidence_rows.size());
        for (const auto& r : e.evidence_rows) {
            local_segments_mirror::EvidenceRow row;
            row.y = r.y;
            row.status = (r.state == cand_observe::EgoLaneEvidenceState::Paint) ? "paint"
                         : (r.state == cand_observe::EgoLaneEvidenceState::Clear) ? "clear_gap"
                                                                                 : "unobservable";
            row.reason = local_segments_mirror::ReasonField::absent();
            rows.push_back(row);
        }
        const local_segments_mirror::Result res =
            local_segments_mirror::local_segments(rows);

        std::vector<std::string> typed_by_index(res.observed.size());
        for (const auto& t : res.typed) {
            if (t.index >= 0 && t.index < static_cast<int>(typed_by_index.size()))
                typed_by_index[static_cast<size_t>(t.index)] = t.label;
        }
        for (const auto& seg : res.observed) {
            if (seg.m.paints == 0) continue;   // nothing paint-like to show
            DrawSeg d = make_draw_segment(
                e, seg.y0, seg.y1,
                typed_by_index[static_cast<size_t>(seg.index)], seg.single_stroke);
            if (d.paint_points.empty()) continue;
            s.segs.push_back(d);
            ++s.n_paint_segs;
            if (d.typed) {
                ++s.n_typed;
                if (!s.typed_labels.empty()) s.typed_labels += ";";
                s.typed_labels += "c" + std::to_string(ci) + ":" + d.label +
                                  std::to_string(d.y1 - d.y0 + 1);
            }
        }
    }
    const auto t3 = std::chrono::steady_clock::now();

    const twinlite_identity_mirror::SelectResult sel =
        twinlite_identity_mirror::select_pair(n05b::adapt_result(rep));
    s.id_left = sel.left_index;
    s.id_right = sel.right_index;
    s.id_reason = sel.reason;
    s.abstain = (sel.left_index < 0 || sel.right_index < 0);

    const auto t4 = std::chrono::steady_clock::now();
    const double seg_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
    const double tot_ms = std::chrono::duration<double, std::milli>(t4 - t0).count();
    s.repaired_ms = rep_ms;
    s.segments_ms = seg_ms;
    s.total_ms = tot_ms;
    s.csv << delivery_id << ',' << rep_ms << ',' << seg_ms << ',' << tot_ms << ','
          << s.n_candidates << ',' << s.n_paint_segs << ',' << s.n_typed << ','
          << s.id_left << ',' << s.id_right << ',' << (s.abstain ? 1 : 0) << ','
          << (s.id_reason.empty() ? "NA" : s.id_reason) << ','
          << (s.typed_labels.empty() ? "NA" : s.typed_labels) << '\n';
    s.csv.flush();
    ++s.logged;
    s.fresh = true;
    s.last_id = delivery_id;
    s.drawn_id = 0;
}

template <class ProductionResult>
inline void set_from_update(const cv::Mat& mask, const cv::Mat& left_bgr,
                            const ProductionResult& production_result,
                            unsigned long long delivery_id) {
    try {
        (void)production_result;   // read-only by contract; referenced for clarity only
        ensure_init();
        State& s = st();
        if (!s.enabled || s.logged >= s.max_rows) return;
        const auto t0 = std::chrono::steady_clock::now();

        const auto t1 = std::chrono::steady_clock::now();
        const cand_observe::EgoLanePostprocessResult rep =
            cand_observe::analyze_twinlite_ego_lane(mask, left_bgr);
        const double rep_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - t1).count();
        set_from_result_impl(rep, delivery_id, t0, rep_ms);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[lane-upgrade] disabled after error: %s\n", e.what());
        st().enabled = false;
    } catch (...) {
        std::fprintf(stderr, "[lane-upgrade] disabled after unknown error\n");
        st().enabled = false;
    }
}

// R2 relay front-end for the pre-computed shared-path repair result.
// analyze_ms spans the shared build + both branch analyses (documented scope).
inline void set_from_update_result(const cand_observe::EgoLanePostprocessResult& rep,
                                   unsigned long long delivery_id,
                                   double analyze_ms) {
    try {
        ensure_init();
        State& s = st();
        if (!s.enabled || s.logged >= s.max_rows) return;
        const auto t0 = std::chrono::steady_clock::now();
        set_from_result_impl(rep, delivery_id, t0, analyze_ms);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[lane-upgrade] disabled after error: %s\n", e.what());
        st().enabled = false;
    } catch (...) {
        std::fprintf(stderr, "[lane-upgrade] disabled after unknown error\n");
        st().enabled = false;
    }
}

// Draw the NEW overlay for this display frame (left eye), gated by the same
// freshness window the production draw uses. Never redraws one id twice.
inline void draw(cv::Mat& left_frame, unsigned long long display_delivery_id,
                 unsigned long long max_age_frames) {
    try {
        State& s = st();
        if (!s.enabled || !s.draw_on || !s.fresh) return;
        if (display_delivery_id < s.last_id) return;
        if (display_delivery_id - s.last_id > max_age_frames) return;
        if (s.drawn_id == display_delivery_id) return;
        s.drawn_id = display_delivery_id;
        if (left_frame.empty()) return;

        for (const auto& d : s.segs) {
            draw_segment_pixels(left_frame, d, true);
        }
        char line[200];
        std::snprintf(line, sizeof(line),
                      "UPG pair=%d/%d %s | obs=%d typ=%d | %.1fms",
                      s.id_left, s.id_right, s.abstain ? "(abstain)" : "",
                      s.n_paint_segs, s.n_typed, s.total_ms);
        // [UI-2026-10-23] Debug legend hidden for judging (cv::putText cannot
        // render Chinese; the lane status HUD above carries the信息 already).
        (void)line;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[lane-upgrade] disabled after error: %s\n", e.what());
        st().enabled = false;
    } catch (...) {
        std::fprintf(stderr, "[lane-upgrade] disabled after unknown error\n");
        st().enabled = false;
    }
}

}  // namespace lane_upgrade_observe

#endif  // LANE_UPGRADE_OBSERVE_H
