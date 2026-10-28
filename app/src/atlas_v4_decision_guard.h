#ifndef ATLAS_V4_DECISION_GUARD_H
#define ATLAS_V4_DECISION_GUARD_H
#include <cstdlib>
#include "light_classes.h"
// Simulation-only agreement gate, NOT a certified standalone v4 classifier.
// Keep any legacy red, including the original label before HSV recoloring.
//
// [ARROW-CORRECTION 2026-10-19] Narrow asymmetric exception to the "no
// mapping into legacy7 IDs" rule: a ROUND legacy class with a fresh,
// same-colour, high-confidence ARROW attribute direction adopts the arrow
// subclass of that colour (round->arrow only). This makes lateral requests
// strictly more restricted, never less. It exists because the seven-class
// head mis-reads small green arrows as round "go" while the v4 attribute
// reads the same arrow correctly (board evidence: legacy=go vs
// color=green/direction=forward d=0.999 -> previously "unknown").
// P3_ARROW_CORRECTION=0 restores the strict pre-patch rule.
inline bool atlas_arrow_correction_enabled() {
    const char* v = std::getenv("P3_ARROW_CORRECTION");
    return !(v && v[0] == '0' && v[1] == '\0');
}

inline std::string atlas_guard_label(const std::string& original,
                                    const std::string& refined,
                                    const std::string& color,
                                    const std::string& direction, bool fresh,
                                    const std::string& direction_loose = "") {
    using namespace p3_light;
    Color oc=COLOR_NONE, rc=COLOR_NONE;
    Direction od=DIR_UNKNOWN, rd=DIR_UNKNOWN;
    const bool op=parse_label(original,&oc,&od);
    const bool rp=parse_label(refined,&rc,&rd);
    // [COLOR-ONLY 2026-10-21] 用户规则：只要颜色是绿的一律放行，红的就不
    // 通过，箭头方向不再参与决策。板端证据：238 条 go+green 属性行中，远小
    // 灯（w≈18px）v4 方向恒为 unknown、颜色置信≈0.98；旧规则"方向必须一致"
    // 把它们全部打成 UNKNOWN/STOP。现改为颜色优先：
    //   1) legacy 红优先（保持 stop）；
    //   2) v4 判红 -> stop（哪怕 legacy 是绿）；
    //   3) v4 判绿 -> go；判黄 -> warning（方向忽略）；
    //   4) v4 判不出颜色时保留 legacy 的颜色（绿/黄）；
    //   5) 其余保持严格 fail-closed。
    if (op && oc==COLOR_RED) return original;
    if (fresh && color=="red") return "stop";
    if (rp && rc==COLOR_RED) return refined;
    if (fresh && color=="off") return "off";
    if (fresh && color=="green") return "go";
    if (fresh && color=="yellow") return "warning";
    if (fresh && op && (oc==COLOR_GREEN || oc==COLOR_YELLOW))
        return original;
    if (!fresh || !op || !rp || oc!=rc || od!=rd) return "unknown";
    if (color!=color_name(rc) || direction!=direction_name(rd)) return "unknown";
    if (rc!=COLOR_GREEN && rc!=COLOR_YELLOW) return "unknown";
    return refined;
}
#endif
