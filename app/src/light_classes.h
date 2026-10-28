// ============================================================
// light_classes.h —— 灯类别映射的唯一来源（single source of truth）
// ============================================================
// 顺序必须与训练导出完全一致，训练侧见:
//   ai/datasets/LISA/data/lisa_v4_mix.yaml
//   ai/scripts/33_训练灯v4.py
// 实测核实（2026-10-08, lisa_v4_headless.onnx 对 LISA val 标注框）:
//   模型输出 argmax == 训练标注 cls（36/55 全部高置信样本一致），
//   旧应用表（0=stop,1=go,...）仅 5/55，确认错位。
// 修改本表前必须同步测试: tests/test_light_class_map.py（离线断言）。
#ifndef P3_LIGHT_CLASSES_H
#define P3_LIGHT_CLASSES_H

#include <cstring>
#include <string>

namespace p3_light {

enum Color {
    COLOR_NONE = 0,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_YELLOW,
};

enum Direction {
    DIR_ROUND = 0,   // 圆形灯: 对任意车道方向都有效
    DIR_FORWARD,
    DIR_LEFT,
    DIR_RIGHT,       // 类别表暂无该类，保留用于未知/未来扩展
    DIR_UNKNOWN,
};

struct LightClass {
    const char* label;
    Color color;
    Direction dir;
};

// 训练类别顺序（不得重排）。
inline constexpr LightClass kLightClasses[7] = {
    {"go",          COLOR_GREEN,  DIR_ROUND},
    {"goForward",   COLOR_GREEN,  DIR_FORWARD},
    {"goLeft",      COLOR_GREEN,  DIR_LEFT},
    {"stop",        COLOR_RED,    DIR_ROUND},
    {"stopLeft",    COLOR_RED,    DIR_LEFT},
    {"warning",     COLOR_YELLOW, DIR_ROUND},
    {"warningLeft", COLOR_YELLOW, DIR_LEFT},
};
inline constexpr int kLightClassCount = 7;

// 供检测解码使用的名字表（替代引擎里的手写数组）。
inline const char* const kLightClassNames[kLightClassCount] = {
    "go", "goForward", "goLeft", "stop", "stopLeft", "warning",
    "warningLeft"};

// 颜色 × 方向 → 类别名；不存在该组合时返回 nullptr。
inline const char* label_for(Color color, Direction dir)
{
    if (color == COLOR_NONE)
        return nullptr;
    for (int i = 0; i < kLightClassCount; ++i) {
        if (kLightClasses[i].color == color && kLightClasses[i].dir == dir)
            return kLightClasses[i].label;
    }
    return nullptr;
}

// 类别名 → 颜色 + 方向。返回 -1 表示未知标签（如颜色复核产生的
// "unknown"），由调用方按"方向未知"保守处理。
inline int class_index_of(const std::string& label)
{
    for (int i = 0; i < kLightClassCount; ++i) {
        if (label == kLightClasses[i].label)
            return i;
    }
    return -1;
}

inline bool parse_label(const std::string& label, Color* color,
                        Direction* dir)
{
    const int idx = class_index_of(label);
    if (idx < 0)
        return false;
    *color = kLightClasses[idx].color;
    *dir = kLightClasses[idx].dir;
    return true;
}

inline const char* color_name(Color c)
{
    switch (c) {
    case COLOR_RED: return "red";
    case COLOR_GREEN: return "green";
    case COLOR_YELLOW: return "yellow";
    default: return "none";
    }
}

inline const char* direction_name(Direction d)
{
    switch (d) {
    case DIR_ROUND: return "round";
    case DIR_FORWARD: return "forward";
    case DIR_LEFT: return "left";
    case DIR_RIGHT: return "right";
    default: return "unknown";
    }
}

// 颜色复核结果应用：保留模型给出的方向，只更新颜色。
// - new_color == COLOR_NONE（复核无法判定）或与当前颜色一致 → 不改写，返回 false
// - 组合存在于类别表（round/left 全色、forward 仅绿）→ 写入对应 label/cls，返回 true
// - 组合不存在（如 forward+红/黄）→ 写入 "unknown"/cls=-1（方向未知），返回 true
// 绝不把箭头重写成圆灯类别。
inline bool resolve_recolor(Color cur_color, Direction dir, Color new_color,
                            std::string* out_label, int* out_cls)
{
    if (new_color == COLOR_NONE || new_color == cur_color)
        return false;
    const char* combined = label_for(new_color, dir);
    if (combined) {
        *out_label = combined;
        *out_cls = class_index_of(combined);
    } else {
        *out_label = "unknown";
        *out_cls = -1;
    }
    return true;
}

}  // namespace p3_light

#endif  // P3_LIGHT_CLASSES_H
