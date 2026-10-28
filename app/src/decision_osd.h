// Decision-layer OSD (display only). Reads the SAME frame's final
// P3DecisionOutput and renders icons/text; never re-derives or mutates the
// decision, never touches detections/keys/LEDs/permission bits.
//
// Switch: P3_DECISION_OSD=1 enables (default off). With the switch off the
// caller does not invoke draw_if_enabled() and the frame is byte-identical.
//
// Placement: every drawing is clipped to the caller-provided safe ROI; all
// rows (main/reason/selector/diagnostic) get independent heights and the text
// is wrapped/truncated by the real TextSink's measured pixel width.
#ifndef DECISION_OSD_H
#define DECISION_OSD_H

#include <opencv2/opencv.hpp>
#include <opencv2/freetype.hpp>

#include <string>

#include "p3_decision_bridge.h"

namespace decision_osd {

// Display context supplied by the caller. Defaults describe "no decision":
// a default-constructed P3DecisionOutput is GO, which must NOT be painted as
// a real permission.
struct Context {
    bool bridge_connected = false;  // decision bridge exists in this run
    bool output_valid = false;      // this frame actually produced a decision
};

// Text abstraction: production wraps a font loaded once by the caller; tests
// record strings without loading fonts.
class TextSink {
public:
    virtual ~TextSink() = default;
    virtual bool supports_unicode() const = 0;
    virtual void put(cv::Mat& frame, const std::string& utf8,
                     const cv::Point& org, int pixel_height,
                     const cv::Scalar& color) = 0;
    // Advance width in pixels (used for wrapping and card sizing).
    virtual int measure(const std::string& utf8, int pixel_height) const = 0;
};

// Production sink over an already-loaded freetype font. When the font is
// null it falls back to cv::putText with English text.
class FreeTypeTextSink : public TextSink {
public:
    explicit FreeTypeTextSink(const cv::Ptr<cv::freetype::FreeType2>& font)
        : font_(font) {}
    bool supports_unicode() const override { return !font_.empty(); }
    void put(cv::Mat& frame, const std::string& utf8, const cv::Point& org,
             int pixel_height, const cv::Scalar& color) override;
    int measure(const std::string& utf8, int pixel_height) const override;

private:
    cv::Ptr<cv::freetype::FreeType2> font_;
};

struct Style {
    cv::Rect safe_area;   // explicit frame coordinates; drawing is clipped here
    bool compact = false; // direct-display corner layout
    // Explicit top-right badge area for the 2-second lane-change grant badge
    // (2026-10-23). Empty = no badge (the default keeps the old contract that
    // nothing is drawn outside safe_area).
    cv::Rect grant_badge_area;
};

// Explicit geometry helpers (the caller supplies the frame, not a guess from
// its height):
//  * direct display: the AI-owned 1280x360 SBS frame; right-eye top area,
//    starting BELOW the yellow title line drawn by main at (20,40) scale .9
//    (title band reserved: y < 56 untouched), clear of the compact HUD
//    (top-left) and the mode-J badge (left, y~46).
//  * canvas: the BOTTOM black band below the placed video strip (the top
//    band is shared by draw_lane_status and the yellow title); the OSD never
//    touches the top band, the video strip or the capture color frame.
cv::Rect direct_safe_area(const cv::Mat& ai_render);
cv::Rect canvas_band_safe_area(const cv::Mat& display, int y_offset);

// Runtime switch; default off.
bool enabled();

// Draw the decision OSD. `out` is read-only and never modified. Returns
// false when the frame is empty or the safe area cannot fit even the compact
// layout (explicit refusal instead of overflow).
bool draw(cv::Mat& frame, const P3DecisionOutput& out, const Context& ctx,
          const Style& style, TextSink& text);

// Convenience for the main integration: draws only when enabled().
bool draw_if_enabled(cv::Mat& frame, const P3DecisionOutput& out,
                     const Context& ctx, const Style& style, TextSink& text);

// Exposed for tests/diagnostics: the short Chinese/English labels the OSD
// would use for this output and context.
std::string main_label_zh(const P3DecisionOutput& out, const Context& ctx);
std::string main_label_en(const P3DecisionOutput& out, const Context& ctx);

}  // namespace decision_osd

#endif  // DECISION_OSD_H
