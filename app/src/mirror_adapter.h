// mirror_adapter.h - N05-B strongly-typed adapter: repair postprocess result
// (cand_observe::EgoLaneCandidateResult, frozen source candidate_postprocess.cpp
// 0c2c02df...) -> N05-A mirror input (MirrorCandidate).
//
// Wiring contract (per CODEX N05-B B01):
//   * maps ACTUAL in-memory fields only; no JSON round-trip anywhere on the
//     runtime path;
//   * candidate original order and indices are preserved 1:1;
//   * coef stays double; row units are the frozen ones: y_min/y_max are mask
//     rows (0..287), domain_first_y/last_y are work rows (0..359); the
//     postprocess z-convention (z=(y-208)/68) is identical to the mirror's;
//   * P/C/U evidence, type and every other field stay READ-ONLY: the adapter
//     copies the numeric fields the selector consumes and the type string for
//     the ledger; nothing is upgraded, refit or dropped.
#pragma once

#include "candidate_postprocess.h"     // frozen 0c2c02df... (header 794a953d...)
#include "selector_mirror.h"           // N05-A mirror cc137c38.../4667a18e...

namespace n05b {

inline twinlite_identity_mirror::MirrorCandidate adapt_candidate(
    const cand_observe::EgoLaneCandidateResult& e)
{
    using namespace twinlite_identity_mirror;
    MirrorCandidate c;
    c.is_object = true;
    c.coef_is_array = true;
    c.coef_len = 3;
    for (int k = 0; k < 3; ++k) {
        c.coef[k] = JNum::Real(e.candidate.coef[static_cast<std::size_t>(k)]);
    }
    c.y_min = JNum::Int(e.candidate.y_min);
    c.y_max = JNum::Int(e.candidate.y_max);
    c.domain_first_y = JNum::Int(e.domain_first_y);
    c.domain_last_y = JNum::Int(e.domain_last_y);
    c.paintable_pool = JNum::Int(e.paintable_pool);
    c.pool_eligible = JNum::Bool(e.pool_eligible);
    c.type = lane_paint_type_name(e.paint_type);  // ledger only, never selected on
    return c;
}

inline std::vector<twinlite_identity_mirror::MirrorCandidate> adapt_result(
    const cand_observe::EgoLanePostprocessResult& result)
{
    std::vector<twinlite_identity_mirror::MirrorCandidate> out;
    out.reserve(result.candidates.size());
    for (const cand_observe::EgoLaneCandidateResult& e : result.candidates) {
        out.push_back(adapt_candidate(e));  // order preserved 1:1
    }
    return out;
}

} // namespace n05b
