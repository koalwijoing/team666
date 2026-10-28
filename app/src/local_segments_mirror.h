// local_segments_mirror.h - C++ isolation mirror of the frozen Python local
// measurement-domain module (candidate_local_segments.py SHA256
// 3a55e75b9a633b48e02bbd63051c990905350086238d00a49a9ca71dff428fff,
// local & crossing copies byte-identical; contract frozen after the 2026-10-15
// CODEX remediation).
//
// Scope: DIAGNOSTIC display evidence only. Blocks/runs/labels are block-scoped
// pixel evidence for review/display; they grant NO lane-change permission and
// carry NO authority fields. The whole-boundary marking is never derived from
// local segments (see the frozen docstring). Pure functions, no I/O.
//
// Validation mirrors the Python ValueError contract: every illegal input form
// that Python rejects must be rejected here too (std::runtime_error; message
// text kept close to the Python text, minus Python-typing-only distinctions
// that cannot occur in a typed C++ input - the JSON reader maps such forms to
// out-of-range integers so they still reject).
#pragma once

#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace local_segments_mirror {

constexpr const char* kProtocol = "local_segments_v1_diagnostic_only_no_authority";

// Python: sample["reason"] can be an absent key, an explicit None, or a string
// (".get('reason', '?')" only substitutes "?" for ABSENT; explicit None becomes
// the None key, which json.dumps renders as "null"). Modeled exactly.
struct ReasonField {
    enum Kind { kAbsent, kNull, kString } kind = kAbsent;
    std::string value;  // kString only
    static ReasonField absent() { return ReasonField{}; }
    static ReasonField null() { ReasonField r; r.kind = kNull; return r; }
    static ReasonField str(std::string v) { ReasonField r; r.kind = kString; r.value = std::move(v); return r; }
};

struct EvidenceRow {
    int y = -1;            // must satisfy 0 <= y < 360
    std::string status;    // paint / clear_gap / unobservable
    ReasonField reason;
};

struct StatusRun {
    std::string status;
    int y0 = 0, y1 = 0, n = 0;
    bool operator==(const StatusRun& o) const {
        return status == o.status && y0 == o.y0 && y1 == o.y1 && n == o.n;
    }
};

// Python _block_metrics dict (cov already rounded to 4 decimals there).
struct BlockMetrics {
    int rows = 0, paints = 0, clears = 0, paintable = 0;
    double cov = 0.0;
    int max_gap = 0;
    std::vector<int> paint_runs, clear_runs;
    int chain = 0;
};

struct ObservedSegment {
    int index = 0, y0 = 0, y1 = 0;
    BlockMetrics m;
    std::string bounded_by[2];  // left, right: "domain_start"/"u", "domain_end"/"u"
    bool single_stroke = false;
};

struct TypedSegment {
    int index = 0, y0 = 0, y1 = 0;
    std::string label;          // SOLID / DASHED (BLOCK-scoped evidence)
    BlockMetrics basis;
    std::string bounded_by[2];
    bool single_stroke = false;
};

struct Result {
    std::vector<ObservedSegment> observed;
    std::vector<TypedSegment> typed;
    std::optional<int> bottom_segment_index;               // nullopt = Python None
    std::vector<std::pair<std::string, int>> u_reason_counts;  // first-seen order
    int n_observed = 0, n_typed = 0;
    std::string protocol = kProtocol;
};

// Frozen public entries (mirrors of normalize_evidence_rows / _runs /
// local_segments). Invalid input throws std::runtime_error (Python ValueError).
std::vector<EvidenceRow> normalize_evidence_rows(
    const std::vector<EvidenceRow>& samples,
    std::optional<std::pair<int, int>> domain = std::nullopt);

std::vector<StatusRun> status_runs(const std::vector<EvidenceRow>& rows);

Result local_segments(const std::vector<EvidenceRow>& samples,
                      int min_paintable = 24, int gap_rows = 5,
                      std::optional<std::pair<int, int>> domain = std::nullopt);

// Python round(value, 4) for the cov field (round-half-even on the scaled
// binary value, matching CPython's correctly-rounded short-repr algorithm for
// the magnitudes encountered here). Kept public: the parity tool re-checks it.
double py_round4(double value);

}  // namespace local_segments_mirror
