// selector_mirror.h - C++ isolation mirror of the frozen Python identity
// selector (oracle SHA256 6048b15bd8dd664d945f5c919584e67e7a0b86c37db4cfc52a90b41707e1df9d,
// twinlite_identity_n02_candidate_20261015/selector.py).
//
// Scope (N05-A agreement): "frozen candidate input -> selection / diagnostics"
// ONLY. This module does no file I/O, has no globals, and must never be
// hooked into main / decision bridge / local-D display / any default path.
//
// Boundaries:
//  - Candidate inputs are READ-ONLY. Original P/C/U evidence, type and all
//    other fields are never modified and never used to upgrade UNKNOWN.
//  - authority / decision_eligible / deployment_approved / material_verified
//    / identity_certified stay false in every result.
//
// Units (frozen; do not change thresholds or reinterpret):
//  - "work" rows: 640x360 delivery rows (0..359).
//  - "mask" rows: candidate y_min/y_max domain (0..287); MASK_SCALE = 1.25.
//  - SAME_LINE_TOL_MASK (6.0) and CROSS_MARGIN_MASK (2.0) are MASK px.
//  - SAME_LINE_MIN_SHARED (12) counts WORK rows (fit ∩ evidence intersection).
#pragma once

#include <string>
#include <utility>
#include <vector>

namespace twinlite_identity_mirror {

constexpr double MASK_SCALE = 1.25;
constexpr double EGO_X_MASK = 256.0;
constexpr int REFERENCE_CAP_WORK = 330;
constexpr int MIN_POOL_ROWS = 18;
constexpr double NUMERIC_EPS = 1e-9;
constexpr double SAME_LINE_TOL_MASK = 6.0;
constexpr int SAME_LINE_MIN_SHARED = 12;
constexpr double CROSS_MARGIN_MASK = 2.0;

// Typed scalar carrying the JSON type tag, so the Python oracle's
// isinstance() semantics (bool is not a number; 300.5 is not Integral)
// can be mirrored exactly.
enum class NumKind { Missing, Bool, Int, Real };

struct JNum {
    NumKind kind = NumKind::Missing;
    bool b = false;
    long long i = 0;
    double d = 0.0;

    static JNum Bool(bool v) {
        JNum n;
        n.kind = NumKind::Bool;
        n.b = v;
        return n;
    }
    static JNum Int(long long v) {
        JNum n;
        n.kind = NumKind::Int;
        n.i = v;
        return n;
    }
    static JNum Real(double v) {
        JNum n;
        n.kind = NumKind::Real;
        n.d = v;
        return n;
    }
};

// Raw candidate as received from the frozen record (only fields the selector
// reads; everything else in the record is ignored). `type` is carried for
// the ledger/tests only and never influences selection.
struct MirrorCandidate {
    bool is_object = true;
    bool coef_is_array = false;
    int coef_len = 0;
    JNum coef[3];
    JNum y_min;
    JNum y_max;
    JNum domain_first_y;
    JNum domain_last_y;
    JNum paintable_pool;
    JNum pool_eligible;
    std::string type;
};

struct MergeEntry {
    int index = -1;
    int same_line_of = -1;
    double max_dx = 0.0;      // Python round(d, 3)  (mask px)
    double max_dx_raw = 0.0;  // unrounded d (diagnostic only)
    int shared_rows = 0;
    int common_lo = 0;
    int common_hi = 0;
};

struct ReferenceEntry {
    int index = -1;
    double x_mask = 0.0;
    double x_work = 0.0;
};

struct SelectResult {
    int left_index = -1;
    int right_index = -1;
    bool has_reference = false;
    int reference_y_work = 0;      // valid iff has_reference
    bool has_common_domain = false;
    int common_lo = 0;             // valid iff has_common_domain
    int common_hi = 0;
    std::vector<ReferenceEntry> candidates_at_reference;
    std::string reason = "no_pool";
    std::vector<MergeEntry> same_line_merged;
    bool has_ambiguous_order_pair = false;
    int ambiguous_a = 0;
    int ambiguous_b = 0;
    // Constant false / true fields, mirrored from the oracle defaults.
    bool authority = false;
    bool decision_eligible = false;
    bool deployment_approved = false;
    bool material_verified = false;
    bool identity_certified = false;
    bool reference_is_current_measured_domain = true;
    // Extra diagnostic (not in the oracle dict): frozen geometry gate on the
    // returned pair; null/false never means a different selection.
    bool pair_valid = false;
    bool geometry_gate_passed = false;
};

struct SameLineOutcome {
    bool dx_ok = false;      // false <=> oracle returned dx = None
    double max_dx = 0.0;     // valid iff dx_ok
    int shared_rows = 0;
    bool has_common = false; // false <=> oracle returned rng = None
    int common_lo = 0;
    int common_hi = 0;
};

struct OrderStableOutcome {
    bool stable = true;
    int peer = -1;
};

// --- frozen primitives (exact mirrors) ---
double coef_at(const MirrorCandidate& c, int k);  // int->double, real as-is
double x_mask(const MirrorCandidate& c, double mask_y);
bool pair_geometry_ok(const MirrorCandidate& left, const MirrorCandidate& right);
bool common_measured_rows(const MirrorCandidate& a, const MirrorCandidate& b,
                          int* lo, int* hi);
SameLineOutcome same_line_dx(const MirrorCandidate& a, const MirrorCandidate& b);
bool same_line_direct(int i, int j,
                      const std::vector<MirrorCandidate>& candidates,
                      SameLineOutcome* outcome);
OrderStableOutcome order_stable(
    int selected, const std::vector<std::pair<double, int>>& peers,
    const std::vector<MirrorCandidate>& candidates, int ref);

// --- top level ---
SelectResult select_pair(const std::vector<MirrorCandidate>& candidates);

} // namespace twinlite_identity_mirror
