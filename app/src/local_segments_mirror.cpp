// local_segments_mirror.cpp - faithful port of the frozen Python module
// (candidate_local_segments.py, SHA 3a55e75b...). Line-by-line parity is
// re-checked by tools/parity_local_segments.py on 1609 real candidates.
#include "local_segments_mirror.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace local_segments_mirror {

namespace {

int physical_y(int value) {
    if (value < 0 || value >= 360)
        throw std::runtime_error("expected an integer physical y in 0..359");
    return value;
}

void check_status(const std::string& status) {
    if (status != "paint" && status != "clear_gap" && status != "unobservable")
        throw std::runtime_error("expected a three-state evidence status");
}

std::pair<int, int> checked_domain(const std::optional<std::pair<int, int>>& domain) {
    if (!domain)
        throw std::runtime_error("expected an inclusive two-integer physical domain");
    const int lo = physical_y(domain->first);
    const int hi = physical_y(domain->second);
    if (lo > hi)
        throw std::runtime_error("physical domain start must not exceed its end");
    return {lo, hi};
}

std::string reason_key(const ReasonField& reason) {
    switch (reason.kind) {
        case ReasonField::kAbsent: return "?";   // Python .get("reason", "?")
        case ReasonField::kNull:   return "null";  // json.dumps({None: 1}) -> "null"
        case ReasonField::kString: return reason.value;
    }
    return "?";
}

BlockMetrics block_metrics(const std::vector<EvidenceRow>& rows, int gap_rows) {
    BlockMetrics m;
    m.rows = static_cast<int>(rows.size());
    for (const auto& s : rows) {
        if (s.status == "paint") ++m.paints;
        else if (s.status == "clear_gap") ++m.clears;
    }
    m.paintable = m.paints + m.clears;
    const std::vector<StatusRun> rr = status_runs(rows);
    for (const auto& r : rr) {
        if (r.status == "paint") m.paint_runs.push_back(r.n);
        else if (r.status == "clear_gap") m.clear_runs.push_back(r.n);
    }
    const double cov = m.paintable ? static_cast<double>(m.paints) / m.paintable : 0.0;
    m.cov = py_round4(cov);
    m.max_gap = 0;
    for (int n : m.clear_runs) m.max_gap = std::max(m.max_gap, n);
    int chain = 0, longest = 0;
    for (size_t i = 0; i < rr.size(); ++i) {
        const auto& r = rr[i];
        if (r.status != "paint" || r.n < 3) continue;
        const bool conn = i >= 2 && rr[i - 1].status == "clear_gap" && rr[i - 1].n >= gap_rows
                          && rr[i - 2].status == "paint" && rr[i - 2].n >= 3;
        chain = conn ? chain + 1 : 1;
        longest = std::max(longest, chain);
    }
    m.chain = longest;
    return m;
}

}  // namespace

double py_round4(double value) {
    return std::nearbyint(value * 10000.0) / 10000.0;
}

std::vector<EvidenceRow> normalize_evidence_rows(
    const std::vector<EvidenceRow>& samples,
    std::optional<std::pair<int, int>> domain) {
    std::optional<std::pair<int, int>> bounds;
    if (domain.has_value()) bounds = checked_domain(domain);

    std::map<int, EvidenceRow> by_y;
    for (const auto& sample : samples) {
        const int y = physical_y(sample.y);
        check_status(sample.status);
        if (by_y.count(y)) throw std::runtime_error("duplicate physical evidence y");
        if (bounds && !(bounds->first <= y && y <= bounds->second))
            throw std::runtime_error("evidence y lies outside the declared domain");
        EvidenceRow copy = sample;
        copy.y = y;
        by_y[y] = copy;
    }

    if (!bounds) {
        if (by_y.empty()) return {};
        bounds = {by_y.begin()->first, by_y.rbegin()->first};
    }
    std::vector<EvidenceRow> out;
    out.reserve(static_cast<size_t>(bounds->second - bounds->first + 1));
    for (int y = bounds->first; y <= bounds->second; ++y) {
        auto it = by_y.find(y);
        if (it != by_y.end()) {
            out.push_back(it->second);
        } else {
            EvidenceRow row;
            row.y = y;
            row.status = "unobservable";
            row.reason = ReasonField::str("missing_evidence_row");
            out.push_back(row);
        }
    }
    return out;
}

std::vector<StatusRun> status_runs(const std::vector<EvidenceRow>& rows) {
    std::vector<StatusRun> out;
    for (const auto& s : normalize_evidence_rows(rows)) {
        if (!out.empty() && out.back().status == s.status) {
            out.back().n += 1;
            out.back().y1 = s.y;
        } else {
            out.push_back(StatusRun{s.status, s.y, s.y, 1});
        }
    }
    return out;
}

Result local_segments(const std::vector<EvidenceRow>& samples_in,
                      int min_paintable, int gap_rows,
                      std::optional<std::pair<int, int>> domain) {
    const std::vector<EvidenceRow> samples = normalize_evidence_rows(samples_in, domain);

    std::vector<std::vector<EvidenceRow>> blocks;
    std::vector<EvidenceRow> cur;
    for (const auto& s : samples) {
        if (s.status == "unobservable") {
            if (!cur.empty()) { blocks.push_back(cur); cur.clear(); }
        } else {
            cur.push_back(s);
        }
    }
    if (!cur.empty()) blocks.push_back(cur);

    Result result;
    {
        std::vector<std::pair<std::string, int>> order;
        std::map<std::string, size_t> pos;
        for (const auto& s : samples) {
            if (s.status != "unobservable") continue;
            const std::string key = reason_key(s.reason);
            auto it = pos.find(key);
            if (it == pos.end()) {
                pos[key] = order.size();
                order.push_back({key, 1});
            } else {
                order[it->second].second += 1;
            }
        }
        result.u_reason_counts = order;
    }
    if (blocks.empty()) {
        result.n_observed = 0;
        result.n_typed = 0;
        return result;
    }

    for (size_t bi = 0; bi < blocks.size(); ++bi) {
        const auto& blk = blocks[bi];
        const int first = blk.front().y, last = blk.back().y;
        const std::string left_ctx = (first == samples.front().y) ? "domain_start" : "u";
        const std::string right_ctx = (last == samples.back().y) ? "domain_end" : "u";
        const BlockMetrics m = block_metrics(blk, gap_rows);
        ObservedSegment seg;
        seg.index = static_cast<int>(bi);
        seg.y0 = first;
        seg.y1 = last;
        seg.m = m;
        seg.bounded_by[0] = left_ctx;
        seg.bounded_by[1] = right_ctx;
        seg.single_stroke = (m.paint_runs.size() == 1);
        result.observed.push_back(seg);

        std::string label;
        if (m.paintable >= min_paintable && m.paints >= 6) {
            if (m.cov >= 0.86 && m.max_gap <= 2) label = "SOLID";
            else if (m.cov >= 0.30 && m.chain >= 3) label = "DASHED";
        }
        if (!label.empty()) {
            TypedSegment t;
            t.index = static_cast<int>(bi);
            t.y0 = first;
            t.y1 = last;
            t.label = label;
            t.basis = m;
            t.bounded_by[0] = left_ctx;
            t.bounded_by[1] = right_ctx;
            t.single_stroke = (m.paint_runs.size() == 1);
            result.typed.push_back(t);
        }
    }

    for (auto it = result.observed.rbegin(); it != result.observed.rend(); ++it) {
        if (it->m.paints > 0) {
            result.bottom_segment_index = it->index;
            break;
        }
    }
    result.n_observed = static_cast<int>(result.observed.size());
    result.n_typed = static_cast<int>(result.typed.size());
    return result;
}

}  // namespace local_segments_mirror
