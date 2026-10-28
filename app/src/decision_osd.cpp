// Decision-layer OSD implementation (display only). See decision_osd.h.
//
// Layout rules (D07): every row has an independent height; text is wrapped or
// ellipsized using the actual TextSink measurement; all drawing happens on a
// clipped ROI of the frame, so border/AA/text/icons cannot leave the safe
// area. If even the compact level cannot fit, draw() refuses (returns false).
#include "decision_osd.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace decision_osd {
namespace {

const cv::Scalar kCardBg(28, 28, 28);
const cv::Scalar kBorder(120, 120, 120);
const cv::Scalar kWhite(255, 255, 255);
const cv::Scalar kGray(160, 160, 160);
const cv::Scalar kRed(40, 40, 220);
const cv::Scalar kGreen(70, 190, 90);
const cv::Scalar kOrange(0, 165, 255);
const cv::Scalar kCyan(220, 220, 80);
const cv::Scalar kDim(90, 90, 90);

enum class Card { Degraded, Stop, Go, Slow, TurnLeft, TurnRight };

Card card_of(const P3DecisionOutput& out, const Context& ctx) {
    if (!ctx.bridge_connected || !ctx.output_valid) return Card::Degraded;
    switch (out.action) {
        case P3Action::STOP: return Card::Stop;
        case P3Action::GO: return Card::Go;
        case P3Action::SLOW: return Card::Slow;
        case P3Action::TURN_LEFT: return Card::TurnLeft;
        case P3Action::TURN_RIGHT: return Card::TurnRight;
    }
    return Card::Degraded;
}

// ---- UTF-8 helpers -------------------------------------------------------
// Split into code points (each one a byte string, no copy of the whole text).
std::vector<std::string> utf8_codepoints(const std::string& s) {
    std::vector<std::string> out;
    for (std::size_t i = 0; i < s.size();) {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        std::size_t len = 1;
        if ((c & 0xE0) == 0xC0) len = 2;
        else if ((c & 0xF0) == 0xE0) len = 3;
        else if ((c & 0xF8) == 0xF0) len = 4;
        if (i + len > s.size()) len = 1;
        out.push_back(s.substr(i, len));
        i += len;
    }
    return out;
}

std::string ellipsis(const TextSink& sink) {
    return sink.supports_unicode() ? "\xE2\x80\xA6" : "...";
}

// Truncate so that text+ellipsis fits max_w; keeps UTF-8 boundaries.
std::string truncate_fit(const TextSink& sink, const std::string& s, int px,
                         int max_w) {
    if (max_w <= 0) return {};
    if (sink.measure(s, px) <= max_w) return s;
    const std::string dots = ellipsis(sink);
    const auto cps = utf8_codepoints(s);
    std::string acc;
    for (const std::string& cp : cps) {
        const std::string next = acc + cp;
        if (sink.measure(next + dots, px) > max_w) break;
        acc = next;
    }
    if (acc.empty()) return sink.measure(dots, px) <= max_w ? dots : std::string();
    return acc + dots;
}

// Greedy wrap to at most max_lines lines; ellipsis on the last line when the
// remaining text does not fit.
std::vector<std::string> wrap_fit(const TextSink& sink, const std::string& s,
                                  int px, int max_w, std::size_t max_lines) {
    std::vector<std::string> lines;
    if (s.empty() || max_w <= 0 || max_lines == 0) return lines;
    const auto cps = utf8_codepoints(s);
    std::size_t i = 0;
    while (i < cps.size() && lines.size() < max_lines) {
        std::string line;
        while (i < cps.size()) {
            const std::string next = line + cps[i];
            if (sink.measure(next, px) > max_w) break;
            line = next;
            ++i;
        }
        if (line.empty()) {
            // Single code point wider than the line: truncate it.
            line = truncate_fit(sink, cps[i], px, max_w);
            ++i;
        }
        if (i >= cps.size()) {
            lines.push_back(line);
            break;
        }
        if (lines.size() + 1 == max_lines) {
            // Last allowed line: fit the remainder plus an ellipsis.
            std::string rest = line;
            for (std::size_t k = i; k < cps.size(); ++k) rest += cps[k];
            lines.push_back(truncate_fit(sink, rest, px, max_w));
            break;
        }
        lines.push_back(line);
    }
    return lines;
}

std::string reason_zh(const std::string& reason) {
    if (reason.empty()) return {};
    const auto current = reason.find(" | now: ");
    if (current != std::string::npos)
        return reason_zh(reason.substr(0, current)) + "；当前：" +
               reason_zh(reason.substr(current + 8));
    for (const auto& key : {std::string("A+D"), std::string("A"), std::string("D")}) {
        const std::string prefix = "last " + key + ": ";
        if (reason.rfind(prefix, 0) == 0)
            return "上次" + key + "：" + reason_zh(reason.substr(prefix.size()));
    }
    if (reason.rfind("refused: ", 0) == 0)
        return "拒绝—" + reason_zh(reason.substr(9));
    if (reason.rfind("waiting: ", 0) == 0)
        return "等待：" + reason_zh(reason.substr(9));
    if (reason.rfind("expired: ", 0) == 0)
        return "超时：" + reason_zh(reason.substr(9));
    struct Pair { const char* needle; const char* zh; };
    static const Pair kPairs[] = {
        {"granted left change", "已获准左变道"},
        {"granted right change", "已获准右变道"},
        {"simulation paused", "当前停驶，未执行变道"},
        {"final safety veto", "最终安全条件不满足"},
        {"unproven ego-path overlap", "目标占道未确认：谨慎减速"},
        {"left solid line blocks lane change", "实线禁止变道"},
        {"right solid line blocks lane change", "实线禁止变道"},
        {"solid line lane change", "实线禁止变道"},
        {"lane change blocked by red/unknown light", "红灯/未知灯：禁止变道"},
        {"turn blocked by red/unknown light", "转向被红/未知灯阻止"},
        {"yellow light priority over lane change", "黄灯优先：减速"},
        {"conflicting left/right lane requests refused", "左右键同时按下：冲突"},
        {"conflicting left/right lane requests", "左右键同时按下：冲突"},
        {"lane change evidence stale or unavailable", "车道证据过期/不可用"},
        {"lane change evidence not current", "车道证据非当前帧"},
        {"left lane change safety evidence incomplete", "左变道：安全证据不完整"},
        {"right lane change safety evidence incomplete", "右变道：安全证据不完整"},
        {"left lane state not dashed", "左车道非虚线：禁止变道"},
        {"right lane state not dashed", "右车道非虚线：禁止变道"},
        {"left lane not dashed, change refused", "非虚线：禁止变道"},
        {"right lane not dashed, change refused", "非虚线：禁止变道"},
        {"lane state not dashed", "非虚线：禁止变道"},
        {"left edge lane", "已是最左车道"},
        {"right edge lane", "已是最右车道"},
        {"already leftmost lane", "已是最左车道"},
        {"already rightmost lane", "已是最右车道"},
        {"left lane data invalid", "左车道数据无效"},
        {"right lane data invalid", "右车道数据无效"},
        {"left lane change refused", "左变道被拒"},
        {"right lane change refused", "右变道被拒"},
        {"vehicle input missing", "缺少车辆输入"},
        {"safety evidence incomplete", "安全证据不完整"},
        {"not enough matching votes", "匹配投票不足"},
        {"fresh dashed lane evidence", "需要新的虚线车道证据"},
        {"evidence not ready", "证据未就绪"},
        {"press again", "请再按一次"},
        {"checked and clear", "已确认净空"},
        {"light direction unknown, conservative stop", "灯方向未知：保守停车"},
        {"front road user below advisory threshold", "前方目标未达阈值"},
        {"near front road user", "前方近距目标"},
        {"large front road user", "前方大型目标"},
        {"front road user", "前方道路目标"},
        {"forced drive on red", "红灯强行驶"},
        {"forced drive while red", "红灯强行驶"},
        {"decision ABI", "决策接口异常"},
        {"abi", "决策接口异常"},
    };
    for (const Pair& p : kPairs) {
        if (reason.find(p.needle) != std::string::npos) return p.zh;
    }
    return reason;
}

const char* light_zh(P3LightState s) {
    switch (s) {
        case P3LightState::RED: return "红";
        case P3LightState::GREEN: return "绿";
        case P3LightState::YELLOW: return "黄";
        case P3LightState::UNKNOWN: return "未知";
        case P3LightState::NONE: default: return "无灯";
    }
}

const char* light_en(P3LightState s) {
    switch (s) {
        case P3LightState::RED: return "RED";
        case P3LightState::GREEN: return "GREEN";
        case P3LightState::YELLOW: return "YELLOW";
        case P3LightState::UNKNOWN: return "UNK";
        case P3LightState::NONE: default: return "none";
    }
}

void draw_alarm_triangle(cv::Mat& f, cv::Point c, int r, const cv::Scalar& col) {
    std::vector<cv::Point> pts{{c.x, c.y - r}, {c.x - r, c.y + r},
                               {c.x + r, c.y + r}};
    cv::fillConvexPoly(f, pts, col, cv::LINE_AA);
    cv::polylines(f, pts, true, kWhite, 2, cv::LINE_AA);
    cv::putText(f, "!", cv::Point(c.x - r / 4, c.y + r / 2),
                cv::FONT_HERSHEY_SIMPLEX, r / 20.0, kWhite, 2, cv::LINE_AA);
}

void draw_icon(cv::Mat& f, Card card, cv::Rect box, TextSink& text) {
    const cv::Point c(box.x + box.width / 2, box.y + box.height / 2);
    const int r = std::min(box.width, box.height) / 2;
    switch (card) {
        case Card::Stop: {
            std::vector<cv::Point> pts;
            for (int i = 0; i < 8; ++i) {
                const double a = CV_PI / 8.0 + i * CV_PI / 4.0;
                pts.emplace_back(c.x + static_cast<int>(r * std::cos(a)),
                                 c.y + static_cast<int>(r * std::sin(a)));
            }
            cv::fillConvexPoly(f, pts, kRed, cv::LINE_AA);
            cv::polylines(f, pts, true, kWhite, 3, cv::LINE_AA);
            const std::string t = text.supports_unicode() ? "停" : "STOP";
            text.put(f, t, cv::Point(c.x - r / 2, c.y + r / 3),
                     std::max(10, r), kWhite);
            break;
        }
        case Card::Go: {
            cv::circle(f, c, r, kGreen, -1, cv::LINE_AA);
            cv::circle(f, c, r, kWhite, 3, cv::LINE_AA);
            std::vector<cv::Point> check{{c.x - r / 2, c.y},
                                         {c.x - r / 6, c.y + r / 2},
                                         {c.x + r * 3 / 5, c.y - r / 2}};
            cv::polylines(f, check, false, kWhite, std::max(2, r / 6),
                          cv::LINE_AA);
            break;
        }
        case Card::Slow: {
            draw_alarm_triangle(f, c, r, kOrange);
            break;
        }
        case Card::TurnLeft:
        case Card::TurnRight: {
            const bool left = card == Card::TurnLeft;
            const int dir = left ? -1 : 1;
            cv::circle(f, c, r, kGreen, -1, cv::LINE_AA);
            cv::circle(f, c, r, kWhite, 3, cv::LINE_AA);
            std::vector<cv::Point> arrow{
                {c.x + dir * r / 2, c.y},
                {c.x - dir * r / 5, c.y - r / 2},
                {c.x - dir * r / 5, c.y - r / 6},
                {c.x - dir * r / 2, c.y - r / 6},
                {c.x - dir * r / 2, c.y + r / 6},
                {c.x - dir * r / 5, c.y + r / 6},
                {c.x - dir * r / 5, c.y + r / 2}};
            cv::fillConvexPoly(f, arrow, kWhite, cv::LINE_AA);
            break;
        }
        case Card::Degraded:
        default: {
            cv::circle(f, c, r, kDim, -1, cv::LINE_AA);
            cv::circle(f, c, r, kWhite, 2, cv::LINE_AA);
            text.put(f, "?", cv::Point(c.x - r / 3, c.y + r / 3),
                     std::max(10, r), kWhite);
            break;
        }
    }
}

// Small lane-selector arrow; dir -1 left, 0 straight (up), +1 right.
void draw_selector_arrow(cv::Mat& f, cv::Point center, int h, int dir,
                         const cv::Scalar& col, bool filled) {
    if (dir != 0) {
        std::vector<cv::Point> tri{{center.x + dir * h / 2, center.y},
                                   {center.x - dir * h / 4, center.y - h / 2},
                                   {center.x - dir * h / 4, center.y + h / 2}};
        if (filled) cv::fillConvexPoly(f, tri, col, cv::LINE_AA);
        cv::polylines(f, tri, true, col, 2, cv::LINE_AA);
    } else {
        std::vector<cv::Point> up{{center.x, center.y - h / 2},
                                  {center.x - h / 3, center.y},
                                  {center.x + h / 3, center.y}};
        if (filled) cv::fillConvexPoly(f, up, col, cv::LINE_AA);
        cv::polylines(f, up, true, col, 2, cv::LINE_AA);
        cv::line(f, cv::Point(center.x, center.y - h / 6),
                 cv::Point(center.x, center.y + h / 2), col, 2, cv::LINE_AA);
    }
}

struct Metrics {
    int base = 16;
    int pad = 6;
    int icon = 40;
    int reason_px = 12;
    int lane_px = 10;
    int diag_px = 9;
    int arrow = 16;
    int arrow_gap = 9;
    int gap = 3;
    int gap_s = 2;
};

Metrics metrics_for(double scale, bool compact) {
    const double s = scale * (compact ? 0.85 : 1.0);
    Metrics m;
    m.base = std::max(10, static_cast<int>(std::lround(16 * s)));
    m.pad = std::max(4, static_cast<int>(std::lround(6 * s)));
    m.icon = std::max(m.base + 4, static_cast<int>(std::lround(2.2 * m.base)));
    m.reason_px = std::max(9, static_cast<int>(std::lround(0.74 * m.base)));
    m.lane_px = std::max(9, static_cast<int>(std::lround(0.62 * m.base)));
    m.diag_px = std::max(8, static_cast<int>(std::lround(0.55 * m.base)));
    m.arrow = std::max(9, static_cast<int>(std::lround(1.0 * m.base)));
    m.arrow_gap = std::max(4, static_cast<int>(std::lround(0.55 * m.base)));
    m.gap = std::max(2, m.pad / 2);
    m.gap_s = std::max(1, m.pad / 3);
    return m;
}

}  // namespace

bool enabled() {
    const char* v = std::getenv("P3_DECISION_OSD");
    return v && std::strcmp(v, "1") == 0;
}

cv::Rect direct_safe_area(const cv::Mat& ai_render) {
    if (ai_render.empty()) return {};
    const int w = ai_render.cols;
    const int h = ai_render.rows;
    // Title band: main draws a yellow line at (20,40) with scale .9 and
    // thickness 2; reserve the top 56 px in every direct layout.
    const int top = std::min(56, std::max(8, h - 60));
    if (w > 700) {
        // 1280x360 SBS: right-eye top area below the title band; the compact
        // HUD sits top-left and the mode-J badge sits left near y~46.
        return cv::Rect(static_cast<int>(w * 0.70), top,
                        static_cast<int>(w * 0.29),
                        std::max(60, h - top - 124));
    }
    // Single-eye frame: top-right corner below the title band.
    return cv::Rect(static_cast<int>(w * 0.50), top,
                    static_cast<int>(w * 0.48),
                    std::max(60, static_cast<int>(h * 0.42) - 50));
}

cv::Rect canvas_band_safe_area(const cv::Mat& display, int y_offset) {
    if (display.empty() || y_offset <= 40) return {};
    // The top band is shared by the lane HUD and the yellow title, so use
    // the BOTTOM black band below the placed video strip instead.
    const int bottom_start = display.rows - y_offset;
    const int band_h = display.rows - bottom_start;
    if (band_h < 60) return {};
    return cv::Rect(display.cols / 2, bottom_start + 8,
                    display.cols / 2 - 16, band_h - 16);
}

// ---- key-result badge + monitoring/key split (2026-10-23) ---------------
// Category A (bottom card): what the vehicle monitoring observed (action,
// light, lane, automatic alarm reasons).
// Category B (top-right badge): the outcome of the last A/D key request
// (granted = green, refused = red + reason), latched for two seconds so it
// does not flash for a single frame.
int g_key_kind = 0;  // 1 grant, -1 refuse, 0 none
std::string g_key_raw;
std::string g_key_label;
std::chrono::steady_clock::time_point g_key_t0;
bool g_key_on = false;
P3Action g_prev_action = P3Action::GO;

// Splits the bridge reason "last X: <feedback> | now: <monitoring>" into the
// key segment and the monitoring segment. Returns true when a key segment
// exists (even if the monitoring tail is empty).
bool split_key_feedback(const std::string& reason, std::string* key,
                        std::string* current)
{
    key->clear();
    current->clear();
    static const char* kPrefixes[] = {"last A+D: ", "last A: ", "last D: "};
    for (const char* prefix : kPrefixes) {
        const std::string p(prefix);
        if (reason.rfind(p, 0) == 0) {
            const std::string rest = reason.substr(p.size());
            const auto now_pos = rest.find(" | now: ");
            if (now_pos == std::string::npos) {
                *key = rest;
            } else {
                *key = rest.substr(0, now_pos);
                *current = rest.substr(now_pos + 8);
            }
            return true;
        }
    }
    return false;
}

void draw_key_badge(cv::Mat& frame, const std::string& label, int kind,
                    TextSink& text, const cv::Rect& area)
{
    const int px = std::clamp(frame.rows / 32, 16, 42);
    const int pad = std::max(6, px / 2);
    const int text_w = text.measure(label, px);
    const int w = text_w + 2 * pad;
    const int h = px + 2 * pad;
    if (w > area.width || h > area.height) return;
    const cv::Rect box(area.x + area.width - w, area.y, w, h);
    cv::Mat r = frame(box);
    {
        cv::Mat overlay(r.size(), CV_8UC3, kCardBg);
        cv::addWeighted(r, 0.35, overlay, 0.65, 0.0, r);
    }
    const cv::Scalar col = kind == 1 ? kGreen : (kind == 2 ? kOrange : kRed);
    cv::rectangle(r, cv::Rect(1, 1, r.cols - 2, r.rows - 2), col,
                  std::max(1, px / 10), cv::LINE_AA);
    text.put(r, label, cv::Point(pad, pad + px), px, col);
}

void update_key_badge(const P3DecisionOutput& out, const Context& ctx,
                      const std::string& key_raw)
{
    const bool valid = ctx.bridge_connected && ctx.output_valid;
    std::string raw = key_raw;
    if (raw.empty() && valid &&
        (out.action == P3Action::TURN_LEFT || out.action == P3Action::TURN_RIGHT))
        raw = out.action == P3Action::TURN_LEFT ? "granted left change"
                                                : "granted right change";
    if (!raw.empty() && raw != g_key_raw) {
        g_key_raw = raw;
        g_key_label = reason_zh(raw);
        if (raw.find("granted") != std::string::npos)
            g_key_kind = 1;
        else if (raw.find("waiting") != std::string::npos ||
                 raw.find("expired") != std::string::npos)
            g_key_kind = 2;
        else
            g_key_kind = -1;
        g_key_t0 = std::chrono::steady_clock::now();
        g_key_on = true;
    }
    g_prev_action = valid ? out.action : P3Action::GO;
    if (g_key_on &&
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - g_key_t0).count() > 2000)
        g_key_on = false;
}

bool draw(cv::Mat& frame, const P3DecisionOutput& out, const Context& ctx,
          const Style& style, TextSink& text) {
    if (frame.empty()) return false;
    const bool zh = text.supports_unicode();
    std::string key_raw, current_raw;
    const bool had_key = split_key_feedback(out.reason, &key_raw, &current_raw);
    update_key_badge(out, ctx, key_raw);
    if (g_key_on && !style.grant_badge_area.empty()) {
        const std::string label =
            zh ? g_key_label : (g_key_kind > 0 ? "GRANTED" : "REFUSED");
        draw_key_badge(frame, label, g_key_kind, text,
                       style.grant_badge_area);
    }
    cv::Rect safe = style.safe_area & cv::Rect(0, 0, frame.cols, frame.rows);
    if (safe.width < 120 || safe.height < 50) return false;

    const Card card = card_of(out, ctx);
    std::string main = zh ? main_label_zh(out, ctx) : main_label_en(out, ctx);
    if (main.empty()) main = zh ? "决策未接入/不可用" : "decision unavailable";

    // [UI-SPLIT 2026-10-23] Category A only: the key-press outcome moved to
    // the top-right badge; the card keeps the monitoring reason (the tail
    // after "| now:") and a generic line when the refusal was the only text.
    std::string reason;
    if (out.alarm && ctx.bridge_connected && ctx.output_valid) {
        // Two-category layout only when the caller provided a badge area; an
        // empty area keeps the legacy one-card rendering (tests, old callers).
        const bool split = !style.grant_badge_area.empty();
        const std::string base = (split && had_key) ? current_raw : out.reason;
        if (!base.empty())
            reason = "报警：" + reason_zh(base);
        else if (had_key && split)
            reason = "报警：按键请求被拒绝";
        else
            reason = "报警：原因未提供";
    }

    std::string diag;
    {
        const bool turn = card == Card::TurnLeft || card == Card::TurnRight;
        char buf[160];
        if (zh) {
            std::snprintf(buf, sizeof(buf), "%s：%s%s%s",
                          turn ? "灯诊断（快照）" : "灯诊断", light_zh(out.light),
                          out.light_stale ? " 过期" : "",
                          (!out.decision_abi_valid && ctx.bridge_connected)
                              ? " 接口异常" : "");
        } else {
            std::snprintf(buf, sizeof(buf), "%s: %s%s%s",
                          turn ? "light diag (snapshot)" : "light diag",
                          light_en(out.light), out.light_stale ? " stale" : "",
                          (!out.decision_abi_valid && ctx.bridge_connected)
                              ? " ABI-fault" : "");
        }
        diag = buf;
        if (card == Card::Degraded)
            diag = zh ? "无决策桥 / 本帧无输出" : "no bridge / no output";
    }
    const std::string lane_label = zh ? "按键选择" : "key-select";

    // Try normal -> compact -> minimal; refuse if nothing fits.
    struct Fit {
        bool ok = false;
        Metrics m;
        int card_w = 0;
        int card_h = 0;
        int main_px = 0;
        std::string main_text;
        std::vector<std::string> reason_lines;
        std::vector<std::string> diag_lines;
        bool show_lane_label = false;
        std::string diag_text;
    } fit;

    for (double level : {1.0, 0.82, 0.66}) {
        const double scale =
            std::clamp(std::min(safe.width / 320.0, safe.height / 100.0), 0.5,
                       3.0) *
            level;
        Metrics m = metrics_for(scale, style.compact);
        const int outer = std::max(2, m.pad / 2);
        const int max_card_w = safe.width - 2 * outer;
        const int max_card_h = safe.height - 2 * outer;
        const int text_x = m.pad + m.icon + m.pad;
        if (max_card_w <= text_x + 30) continue;  // no room for text
        const int content_w = max_card_w - text_x - m.pad;

        // Main label: shrink the font first, then ellipsize.
        int main_px = m.base;
        while (main_px > std::max(10, (m.base * 7) / 10) &&
               text.measure(main, main_px) > content_w)
            --main_px;
        std::string main_text = truncate_fit(text, main, main_px, content_w);

        // Reason: at most two lines, wrapping on measured width.
        std::vector<std::string> reason_lines =
            wrap_fit(text, reason, m.reason_px, content_w, 2);

        // Lane row: drop the label when the arrows leave no room.
        const int arrows_w = 3 * m.arrow + 2 * m.arrow_gap;
        const bool show_label =
            arrows_w + m.pad + text.measure(lane_label, m.lane_px) <= content_w;

        std::string diag_text = truncate_fit(text, diag, m.diag_px, content_w);

        const int row_main = std::max(m.icon, main_px);
        int reason_h = 0;
        if (!reason_lines.empty()) {
            reason_h = static_cast<int>(reason_lines.size()) * m.reason_px +
                       (static_cast<int>(reason_lines.size()) - 1) * m.gap_s;
        }
        const int lane_row_h = std::max(m.arrow, m.lane_px);
        const int content_h = m.pad + row_main +
                              (reason_lines.empty() ? 0 : m.gap + reason_h) +
                              m.gap + lane_row_h + m.gap_s + m.diag_px + m.pad;
        if (content_h > max_card_h) continue;

        fit.ok = true;
        fit.m = m;
        fit.card_w = max_card_w;
        fit.card_h = content_h;
        fit.main_px = main_px;
        fit.main_text = main_text;
        fit.reason_lines = reason_lines;
        fit.diag_text = diag_text;
        fit.show_lane_label = show_label;
        break;
    }
    if (!fit.ok) return false;

    const Metrics& m = fit.m;
    const int outer = std::max(2, m.pad / 2);
    const cv::Rect roi(safe.x + outer, safe.y + outer, fit.card_w, fit.card_h);
    cv::Mat r = frame(roi);  // all drawing below is clipped to this ROI

    {
        cv::Mat overlay(r.size(), CV_8UC3, kCardBg);
        cv::addWeighted(r, 0.45, overlay, 0.55, 0.0, r);
    }
    const int t = std::max(2, m.pad / 2);
    const cv::Scalar border = card == Card::Stop || out.alarm ? kRed : kBorder;
    cv::rectangle(r, cv::Rect(t / 2, t / 2, r.cols - t, r.rows - t), border, t,
                  cv::LINE_AA);

    const int row_main = std::max(m.icon, fit.main_px);
    const cv::Rect icon_box(m.pad, m.pad + (row_main - m.icon) / 2, m.icon,
                            m.icon);
    draw_icon(r, card, icon_box, text);
    const int text_x = m.pad + m.icon + m.pad;

    const cv::Scalar main_col =
        card == Card::Stop ? kRed
        : card == Card::Go || card == Card::TurnLeft ||
                  card == Card::TurnRight
              ? kGreen
        : card == Card::Slow ? kOrange
                             : kGray;
    int y = m.pad + (row_main + fit.main_px) / 2 - 1;
    text.put(r, fit.main_text, cv::Point(text_x, y), fit.main_px, main_col);

    y = m.pad + row_main;
    if (!fit.reason_lines.empty()) {
        y += m.gap;
        for (const std::string& line : fit.reason_lines) {
            text.put(r, line, cv::Point(m.pad, y + m.reason_px), m.reason_px,
                     kRed);
            y += m.reason_px + m.gap_s;
        }
    }

    {
        y += m.gap;
        const int cy = y + std::max(m.arrow, m.lane_px) / 2;
        int x = m.pad;
        const int lane = static_cast<int>(out.lane);
        for (int i = 0; i < 3; ++i) {
            const int dir = i == 0 ? -1 : (i == 2 ? 1 : 0);
            const bool on = i == lane;
            draw_selector_arrow(r, cv::Point(x + m.arrow / 2, cy), m.arrow,
                                dir, on ? kCyan : kDim, on);
            x += m.arrow + m.arrow_gap;
        }
        if (fit.show_lane_label) {
            text.put(r, lane_label, cv::Point(x + m.pad / 2, cy + m.lane_px / 2),
                     m.lane_px, kGray);
        }
        y += std::max(m.arrow, m.lane_px) + m.gap_s;
    }

    text.put(r, fit.diag_text, cv::Point(m.pad, y + m.diag_px), m.diag_px,
             (!out.decision_abi_valid && ctx.bridge_connected) ? kRed : kGray);
    return true;
}

bool draw_if_enabled(cv::Mat& frame, const P3DecisionOutput& out,
                     const Context& ctx, const Style& style, TextSink& text) {
    if (!enabled()) return false;
    return draw(frame, out, ctx, style, text);
}

std::string main_label_zh(const P3DecisionOutput& out, const Context& ctx) {
    if (!ctx.bridge_connected || !ctx.output_valid) return "决策未接入/不可用";
    switch (out.action) {
        case P3Action::STOP: return "停止（模拟）";
        case P3Action::GO: return "允许行驶 / 保持本车道（模拟）";
        case P3Action::SLOW: return "减速（模拟）";
        case P3Action::TURN_LEFT: return "获准左变道（模拟）";
        case P3Action::TURN_RIGHT: return "获准右变道（模拟）";
    }
    return "决策未接入/不可用";
}

std::string main_label_en(const P3DecisionOutput& out, const Context& ctx) {
    if (!ctx.bridge_connected || !ctx.output_valid) return "decision unavailable";
    switch (out.action) {
        case P3Action::STOP: return "STOP (sim)";
        case P3Action::GO: return "GO / keep lane (sim)";
        case P3Action::SLOW: return "SLOW (sim)";
        case P3Action::TURN_LEFT: return "LEFT change granted (sim)";
        case P3Action::TURN_RIGHT: return "RIGHT change granted (sim)";
    }
    return "decision unavailable";
}

void FreeTypeTextSink::put(cv::Mat& frame, const std::string& utf8,
                           const cv::Point& org, int pixel_height,
                           const cv::Scalar& color) {
    if (!font_.empty()) {
        font_->putText(frame, utf8, org, pixel_height, color, -1, cv::LINE_8,
                       true);
    } else {
        cv::putText(frame, utf8, org, cv::FONT_HERSHEY_SIMPLEX,
                    pixel_height / 30.0, color, 1, cv::LINE_AA);
    }
}

int FreeTypeTextSink::measure(const std::string& utf8, int pixel_height) const {
    if (!font_.empty()) {
        int baseline = 0;
        return font_->getTextSize(utf8, pixel_height, -1, &baseline).width;
    }
    int baseline = 0;
    return cv::getTextSize(utf8, cv::FONT_HERSHEY_SIMPLEX,
                           pixel_height / 30.0, 1, &baseline)
        .width;
}

}  // namespace decision_osd
