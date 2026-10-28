// Prototype only: not yet connected to main.cpp or the default application.
// All GUI callbacks execute on the constructing/main thread. A separate sole
// producer transfers owned immutable frame+decision snapshots into one slot.
#ifndef P3_MAIN_GUI_CHANNEL_H
#define P3_MAIN_GUI_CHANNEL_H
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace p3_main_gui {
template<class Frame> class Channel {
public:
    struct Snapshot {
        std::uint64_t sequence, frame_id, decision_frame_id;
        std::shared_ptr<const Frame> image;
    };
    struct Status {
        bool ready, failed, stop_requested, worker_done, gui_exited, close_attempted;
        std::uint64_t published, shown, published_seq, shown_seq, coalesced;
        std::string error;
    };
    struct Gui {
        virtual ~Gui() = default;
        virtual void open(const std::string&, bool) = 0;
        virtual void show(const Snapshot&) = 0;
        virtual int poll() = 0;
        virtual void close() = 0;
    };

    Channel() : owner_(std::this_thread::get_id()) {}
    Channel(const Channel&) = delete;
    Channel& operator=(const Channel&) = delete;

    bool start(std::string window, bool fullscreen, int timeout_ms = 3000) {
        std::unique_lock<std::mutex> lk(mu_);
        bind_producer();
        if (start_requested_ || timeout_ms <= 0)
            throw std::logic_error("invalid/repeated GUI startup");
        window_ = std::move(window);
        fullscreen_ = fullscreen;
        start_requested_ = true;
        if (!cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                          [&] { return ready_ || failed_ || stop_; })) {
            fail_locked("GUI main thread startup timed out");
        }
        return ready_ && !failed_ && !stop_;
    }

    // Frame MUST deeply own its pixel storage (e.g. clone cv::Mat before here).
    // unique_ptr transfer prevents reusing this packet after publication. The
    // caller's actual clone contract still needs an OpenCV integration test.
    bool publish(std::unique_ptr<Frame> image, std::uint64_t frame,
                 std::uint64_t decision_frame) {
        std::lock_guard<std::mutex> lk(mu_);
        bind_producer();
        if (!image || frame != decision_frame || frame <= last_frame_)
            throw std::logic_error("invalid/nonmonotonic frame-decision snapshot");
        if (!ready_ || failed_ || stop_ || done_) return false;
        if (latest_) ++coalesced_;
        // [AUDIT-M3 2026-10-22] Track the published sequence explicitly; the
        // stall detector compares published_seq against shown_seq.
        ++published_;
        ++published_seq_;
        latest_ = std::make_shared<const Snapshot>(Snapshot{
            published_seq_, frame, decision_frame,
            std::shared_ptr<const Frame>(std::move(image))});
        last_frame_ = frame;
        return true;
    }

    void request_stop() {
        std::lock_guard<std::mutex> lk(mu_);
        stop_ = true;
        cv_.notify_all();
    }

    void worker_done() {
        std::lock_guard<std::mutex> lk(mu_);
        bind_producer();
        done_ = true; // lifecycle flag, NOT proof that an external thread joined
        cv_.notify_all();
    }

    Status status() const {
        std::lock_guard<std::mutex> lk(mu_);
        return {ready_, failed_, stop_, done_, !gui_maybe_open_, close_attempted_,
                published_, shown_, published_seq_, shown_seq_, coalesced_, error_};
    }

    // Never calls GUI on a producer. A blocking callback does not hold mu_, so
    // status/stop/publication remain usable and GUI exit is never pre-claimed.
    void pump(Gui& gui) {
        if (std::this_thread::get_id() != owner_)
            throw std::logic_error("GUI pump must run on its main owner");
        bool open = false;
        {
            std::lock_guard<std::mutex> lk(mu_);
            if (start_requested_ && !open_attempted_ && !failed_ && !stop_) {
                open_attempted_ = true;
                gui_maybe_open_ = true;
                open = true;
            }
        }
        if (open) {
            try {
                gui.open(window_, fullscreen_);
                std::lock_guard<std::mutex> lk(mu_);
                ready_ = !failed_ && !stop_;
                cv_.notify_all();
            } catch (const std::exception& e) { fail(e.what()); }
              catch (...) { fail("unknown GUI open failure"); }
        }
        std::shared_ptr<const Snapshot> taken;
        bool poll = false;
        {
            std::lock_guard<std::mutex> lk(mu_);
            if (ready_ && !failed_) {
                taken = std::move(latest_);
                poll = true;
            }
        }
        if (poll) {
            try {
                if (taken) gui.show(*taken);
                const int key = gui.poll();
                std::lock_guard<std::mutex> lk(mu_);
                if (taken) { ++shown_; shown_seq_ = taken->sequence; }
                if (key == 'q' || key == 'Q' || key == 27) stop_ = true;
            } catch (const std::exception& e) { fail(e.what()); }
              catch (...) { fail("unknown GUI show/poll failure"); }
        }
        bool close = false;
        {
            std::lock_guard<std::mutex> lk(mu_);
            if (done_ && !latest_ && gui_maybe_open_ && !close_attempted_) {
                close_attempted_ = true;
                close = true;
            }
            if (failed_) latest_.reset();
        }
        if (close) {
            try {
                gui.close();
                std::lock_guard<std::mutex> lk(mu_);
                gui_maybe_open_ = false; // only AFTER the close call returns
                ready_ = false;
            } catch (const std::exception& e) { fail(e.what()); }
              catch (...) { fail("unknown GUI close failure"); }
        }
    }

private:
    void bind_producer() { // mu_ held
        const auto id = std::this_thread::get_id();
        if (id == owner_) throw std::logic_error("GUI owner is not the producer");
        if (producer_ == std::thread::id{}) producer_ = id;
        if (producer_ != id) throw std::logic_error("multiple compute producers");
    }
    void fail_locked(const std::string& why) {
        failed_ = true;
        stop_ = true;
        if (error_.empty()) error_ = why;
        cv_.notify_all();
    }
    void fail(const std::string& why) {
        std::lock_guard<std::mutex> lk(mu_);
        fail_locked(why);
    }
    const std::thread::id owner_;
    std::thread::id producer_;
    mutable std::mutex mu_;
    std::condition_variable cv_;
    bool start_requested_ = false, open_attempted_ = false, ready_ = false;
    bool failed_ = false, stop_ = false, done_ = false;
    bool gui_maybe_open_ = false, close_attempted_ = false, fullscreen_ = true;
    std::string window_, error_;
    std::uint64_t published_ = 0, shown_ = 0, shown_seq_ = 0;
    std::uint64_t published_seq_ = 0;
    std::uint64_t coalesced_ = 0, last_frame_ = 0;
    std::shared_ptr<const Snapshot> latest_;
};
} // namespace p3_main_gui
#endif
