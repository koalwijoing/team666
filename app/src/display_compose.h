// ============================================================
// display_compose.h —— 显示合成提取（残影专项：供回归测试直连真实路径）
// ============================================================
// 从 main.cpp 主循环提取，语义与原实现逐句对应：
//   * 720p 画布复用：每帧整幅清零，再把"当帧"内容放回中心条带；
//   * ai_mode==3：底图不带 color（中心由 place_ai_frame 放入当帧 ai_render）；
//   * 直显/无画布路径：display 即当帧 color，不做任何跨帧复用。
// 测试点：A 帧（含框）→ B 帧（无框）后，B 输出不得残留 A 的任何像素/框。
#ifndef P3_DISPLAY_COMPOSE_H
#define P3_DISPLAY_COMPOSE_H

#include <cstdint>
#include <opencv2/core.hpp>

namespace p3_display {

// 构建本帧显示底图。canvas 为跨帧复用的持久缓冲（由调用方持有）。
// 返回值为本帧实际显示的 Mat：画布路径返回 canvas（与 canvas 共享缓冲），
// 否则返回 color 本身。
inline cv::Mat compose_display_base(cv::Mat& canvas, const cv::Mat& color,
                                    std::uint32_t width, std::uint32_t height,
                                    bool need_canvas, int ai_mode, int& y_offset)
{
    y_offset = 0;
    if (width == 1280 && height == 360 && need_canvas) {
        if (canvas.empty())
            canvas = cv::Mat::zeros(720, 1280, CV_8UC3);
        canvas.setTo(cv::Scalar::all(0));
        y_offset = 180;
        if (ai_mode != 3)
            color.copyTo(canvas(cv::Rect(0, y_offset, 1280, 360)));
        return canvas;
    }
    return color;
}

// ai-mode-3：把当帧 AI 渲染帧放回画布中心条带（直显路径 y_offset==0，无操作）。
inline void place_ai_frame(cv::Mat& display, const cv::Mat& ai_render,
                           int y_offset, int ai_mode)
{
    if (ai_mode == 3 && y_offset)
        ai_render.copyTo(display(cv::Rect(0, y_offset, ai_render.cols,
                                          ai_render.rows)));
}

}  // namespace p3_display

#endif  // P3_DISPLAY_COMPOSE_H
