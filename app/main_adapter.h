#ifndef P3_QT_MAIN_ADAPTER_H
#define P3_QT_MAIN_ADAPTER_H
#include "main_gui_channel.h"
#include "p3_decision_bridge.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

namespace p3_main_gui {
struct DisplayFrame {
    cv::Mat pixels;
    P3DecisionOutput decision;
    std::uint64_t frame_id = 0;
    // [PERF-P3a 2026-10-22] When true, the GUI main thread composes the final
    // canvas (decision OSD, lane status, prescale, HUD) instead of the compute
    // worker. The published pixels are then the pre-compose canvas.
    bool compose_on_gui = false;
    int y_offset = 0;
    bool bridge_connected = false;
    std::string hud;
    // [AUDIT-H1 2026-10-22] Carry the speed readout to the GUI composer; the
    // worker computes it and the composer used to always render "--".
    double speed_kmh = 0.0;
    bool speed_valid = false;
};
using DisplayChannel = Channel<DisplayFrame>;

inline std::unique_ptr<DisplayFrame> owned_snapshot(
        const cv::Mat& pixels, const P3DecisionOutput& decision, std::uint64_t id,
        bool compose_on_gui = false, int y_offset = 0,
        bool bridge_connected = false, std::string hud = {},
        double speed_kmh = 0.0, bool speed_valid = false) {
    if (pixels.empty()) throw std::invalid_argument("cannot publish an empty GUI frame");
    auto out = std::make_unique<DisplayFrame>();
    out->pixels = pixels.clone(); // never share the RGA/render/static-canvas buffer
    out->decision = decision;
    out->frame_id = id;
    out->compose_on_gui = compose_on_gui;
    out->y_offset = y_offset;
    out->bridge_connected = bridge_connected;
    out->hud = std::move(hud);
    out->speed_kmh = speed_kmh;
    out->speed_valid = speed_valid;
    return out;
}

// Called ONLY by the main-owned Channel::pump. No engine/PCI/decision access.
class HighGui : public DisplayChannel::Gui {
public:
    void open(const std::string& window, bool fullscreen) override {
        window_ = window;
        cv::namedWindow(window_, cv::WINDOW_NORMAL);
        if (fullscreen)
            cv::setWindowProperty(window_, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        std::printf("[gui-main] opened; GUI owner=main; call completion is NOT scan-out\n");
    }
    void show(const DisplayChannel::Snapshot& snapshot) override {
        if (snapshot.frame_id != snapshot.decision_frame_id ||
            snapshot.frame_id != snapshot.image->frame_id)
            throw std::logic_error("GUI received mismatched frame/decision snapshot");
        started_ = std::chrono::steady_clock::now();
        if (snapshot.image->compose_on_gui && composer_) {
            cv::Mat final = composer_(snapshot.image->pixels, *snapshot.image);
            cv::imshow(window_, final);
        } else {
            cv::imshow(window_, snapshot.image->pixels);
        }
        pending_ = true;
        last_sequence_ = snapshot.sequence;
        last_frame_ = snapshot.frame_id;
        last_decision_frame_ = snapshot.decision_frame_id;
    }
    // Installed by main before the pump loop; runs ONLY on this GUI thread.
    void set_composer(
        std::function<cv::Mat(const cv::Mat&, const DisplayFrame&)> composer) {
        composer_ = std::move(composer);
    }
    int poll() override {
        const int key = cv::pollKey();
        if (pending_) {
            pending_ = false;
            ++completed_;
            call_ms_ += std::chrono::duration<double,std::milli>(
                std::chrono::steady_clock::now() - started_).count();
            if (completed_ % 30 == 0) {
                std::printf("[gui-main] completed=%lu sequence=%lu frame=%lu decision_frame=%lu mean_call_ms=%.2f\n",
                            completed_, last_sequence_, last_frame_, last_decision_frame_, call_ms_ / 30);
                call_ms_ = 0;
            }
        }
        return key;
    }
    void close() override {
        std::printf("[gui-main] closing; completed=%lu last_frame=%lu\n", completed_, last_frame_);
        cv::destroyWindow(window_);
        std::printf("[gui-main] close returned on main\n");
    }
private:
    std::string window_;
    std::function<cv::Mat(const cv::Mat&, const DisplayFrame&)> composer_;
    bool pending_ = false;
    std::uint64_t completed_ = 0, last_sequence_ = 0, last_frame_ = 0;
    std::uint64_t last_decision_frame_ = 0;
    double call_ms_ = 0;
    std::chrono::steady_clock::time_point started_;
};
} // namespace p3_main_gui
#endif
