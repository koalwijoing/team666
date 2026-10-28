// disp_publish.h -- display worker (v5: no destructor race, no dual GUI owner).
//
// History, because each step was a real failure:
//   v1 window created in main, imshow in worker  -> highgui binds window to its
//      creating thread, so the window stayed black;
//   v2 GTK paints only inside its event loop      -> worker must call pollKey();
//   v3 thread owns the window, exceptions caught  -> ok, but the destructor could
//      free the mutex/Mat while a blocked worker still touched them;
//   v4 stop() detached after a grace period       -> ASan: heap-use-after-free
//      (worker writes freed state after Publisher is destroyed);
//   v5 (this) removes both faults by construction:
//        * all mutable state lives in a shared_ptr the worker also holds, so the
//          object cannot be freed while the worker can still reach it;
//        * stop() never frees anything and never pretends the worker exited: it
//          joins only when the worker confirmed exit, otherwise it returns and
//          lets the process end (the OS reclaims the thread);
//        * no fallback window is created while an unconfirmed worker might still
//          own one - a caller that cannot display simply stops displaying.
#ifndef P3_DISP_PUBLISH_H
#define P3_DISP_PUBLISH_H

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace p3_disp {

// Test-only injection points; the shipped build never defines P3_DISP_TEST.
// Under P3_DISP_TEST the HighGUI calls go through this interface so a test can
// drive the failure modes deterministically (a CPU stub instead of GTK).
struct Gui {
    virtual ~Gui() = default;
    virtual void open(const std::string& window, bool fullscreen) = 0;
    virtual void show(const std::string& window, const cv::Mat& frame) = 0;
    virtual void close(const std::string& window) = 0;
};

class Publisher {
public:
    Publisher() : st_(std::make_shared<State>()) {}

    bool start(const std::string& window, bool fullscreen)
    {
        const auto st = st_;                  // keep the state alive for the thread
        std::lock_guard<std::mutex> lk(st->mu);
        if (thread_.joinable()) return true;
        st->window = window;
        st->fullscreen = fullscreen;
        st->stop = false;
        st->ready = false;
        st->failed = false;
        thread_ = std::thread([st] { run(st); });
        return true;
    }

    // Bounded: returns false if the worker neither became ready nor failed in time.
    bool wait_ready(int timeout_ms = 5000)
    {
        const auto st = st_;
        std::unique_lock<std::mutex> lk(st->mu);
        const bool ok = st->cv_ready.wait_for(
            lk, std::chrono::milliseconds(timeout_ms),
            [&] { return st->ready || st->failed; });
        if (!ok) {
            st->error = "display worker did not become ready in time";
            return false;
        }
        return !st->failed;
    }

    void publish(const cv::Mat& frame)
    {
        const auto st = st_;
        if (st->failed.load()) return;        // no consumer left
        cv::Mat copy = frame.clone();         // refcounted: reader keeps its own buffer
        {
            std::lock_guard<std::mutex> lk(st->mu);
            if (st->stop) return;
            st->latest = copy;
            st->latest_seq = ++st->published_seq;
            st->has_new = true;
            ++st->published;
        }
        st->cv.notify_one();
    }

    // Never frees shared state and never claims a stuck worker exited.
    void stop(int grace_ms = 1500)
    {
        // Idempotent: main calls stop() explicitly and ~Publisher calls it again.
        // Without this guard the second call would sit through the whole grace
        // period again, delaying shutdown for no reason.
        if (!thread_.joinable()) return;
        const auto st = st_;
        {
            std::lock_guard<std::mutex> lk(st->mu);
            st->stop = true;
        }
        st->cv.notify_all();
        st->cv_ready.notify_all();
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(grace_ms);
        while (!st->exited.load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if (st->exited.load()) {
            if (thread_.joinable()) thread_.join();
        } else {
            // The worker is inside a blocking highgui call.  Detach it WITHOUT
            // freeing anything: it still holds its own reference to State, so it
            // cannot touch freed memory; the process exits and the OS reclaims it.
            if (thread_.joinable()) thread_.detach();
        }
    }

    ~Publisher() { stop(); }

#ifdef P3_DISP_TEST
    void set_gui(std::shared_ptr<Gui> gui) { st_->gui = std::move(gui); }
#endif

    bool failed() const { return st_->failed.load(); }
    // True only when the worker has genuinely returned; a detached, still-busy
    // worker reports false and the caller must not touch GUI state.
    bool exited() const { return st_->exited.load(); }
    std::string error() const
    {
        const auto st = st_;
        std::lock_guard<std::mutex> lk(st->mu);
        return st->error;
    }
    unsigned long published() const { return st_->published.load(); }
    unsigned long shown() const { return st_->shown.load(); }
    unsigned long published_seq() const { return st_->published_seq.load(); }
    unsigned long shown_seq() const { return st_->shown_seq.load(); }

private:
    struct State {
        std::mutex mu;
        std::condition_variable cv;
        std::condition_variable cv_ready;
        cv::Mat latest;
        bool has_new = false;
        bool ready = false;
        bool stop = false;
        bool fullscreen = true;
        std::string window;
        std::string error;
        std::atomic<bool> failed{false};
        std::atomic<bool> exited{false};
        std::atomic<unsigned long> published{0};
        std::atomic<unsigned long> shown{0};
        // Pending is a fact the worker publishes, not a difference of counters.
        std::atomic<unsigned long> published_seq{0};
        std::atomic<unsigned long> shown_seq{0};
        unsigned long latest_seq = 0;      // seq of the frame currently in the slot
#ifdef P3_DISP_TEST
        std::shared_ptr<Gui> gui;
#endif
    };

    static void gui_open(State& s, const std::string& w, bool fs)
    {
#ifdef P3_DISP_TEST
        if (s.gui) { s.gui->open(w, fs); return; }
#endif
        cv::namedWindow(w, cv::WINDOW_NORMAL);
        if (fs)
            cv::setWindowProperty(w, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    }

    static void gui_show(State& s, const cv::Mat& frame)
    {
#ifdef P3_DISP_TEST
        if (s.gui) { s.gui->show(s.window, frame); return; }
#endif
        cv::imshow(s.window, frame);
        cv::pollKey();                    // GTK paints only inside its event loop
    }

    static void gui_close(State& s)
    {
#ifdef P3_DISP_TEST
        if (s.gui) { s.gui->close(s.window); return; }
#endif
        cv::destroyWindow(s.window);
    }

    static void run(std::shared_ptr<State> st)
    {
        bool window_open = false;
        try {
            gui_open(*st, st->window, st->fullscreen);
            window_open = true;
            {
                std::lock_guard<std::mutex> lk(st->mu);
                st->ready = true;
            }
            st->cv_ready.notify_all();

            while (true) {
                cv::Mat frame;
                unsigned long taken = 0;
                {
                    std::unique_lock<std::mutex> lk(st->mu);
                    st->cv.wait(lk, [&] { return st->has_new || st->stop; });
                    if (!st->has_new && st->stop) break;
                    frame = st->latest;
                    taken = st->latest_seq;
                    st->has_new = false;
                }
                if (frame.empty()) continue;
                gui_show(*st, frame);
                st->shown_seq = taken;     // completing the call != scan-out, but it is a fact
                ++st->shown;
            }
        } catch (const std::exception& e) {
            std::lock_guard<std::mutex> lk(st->mu);
            st->error = e.what();
            st->failed = true;
        } catch (...) {
            std::lock_guard<std::mutex> lk(st->mu);
            st->error = "unknown exception in display worker";
            st->failed = true;
        }
        if (window_open) {
            try {
                gui_close(*st);              // the worker owns the window
            } catch (...) {
            }
        }
        {
            std::lock_guard<std::mutex> lk(st->mu);
            // [AUDIT-M2 2026-10-22] ready means "window opened"; it must NOT
            // be re-asserted after the worker closes the window.
            st->ready = false;
        }
        st->exited = true;
        st->cv_ready.notify_all();
    }

    std::shared_ptr<State> st_;
    std::thread thread_;
};


// ---------------------------------------------------------------------------
// Runtime health: a worker can be alive and still present nothing.  "Frames are
// being published but nothing has been presented for a while" is a fault the
// application must surface - processing FPS must never look healthy while the
// screen is frozen.
//
// Two mistakes were found in the first version and are fixed here:
//   1) the age was measured from the steady_clock epoch, so the very first
//      pending frame was reported as hours of stall;
//   2) pending was inferred from published - shown, but the single slot
//      coalesces frames, so that difference never returns to zero after a burst
//      and a healthy idle display was reported as stalled.
// Pending is now a fact published by the worker (a sequence number it has taken
// and one it has finished presenting), and the age is measured from the moment
// the pending state actually began.
//
// The clock is injected so tests advance it deterministically.  The threshold is
// an engineering fault threshold, NOT a proof about HDMI scan-out.
struct Health {
    enum Kind { None, Failed, Stalled };
    Kind kind = None;
    unsigned long published = 0;
    unsigned long shown = 0;
    long long age_ms = 0;
    std::string detail;
};

class HealthMonitor {
public:
    explicit HealthMonitor(int stall_ms = 2000) : stall_ms_(stall_ms) {}

    // published_seq / shown_seq are worker-owned sequence numbers.
    //
    // Progress is "the presented sequence advanced", NOT "the queue was seen
    // empty".  A busy but healthy display always has the next frame pending, so
    // tying the timer to pending=false disabled working displays; and resetting
    // on published_seq would hide a genuinely frozen presentation forever.
    // The timer therefore resets only when shown advances, or while nothing is
    // pending; published advancing never resets it.
    Health observe(unsigned long published_seq, unsigned long shown_seq, bool failed,
                   const std::string& error,
                   std::chrono::steady_clock::time_point now)
    {
        Health h;
        if (failed) {
            if (!reported_failed_) {
                reported_failed_ = true;
                h.kind = Health::Failed;
                h.published = published_seq;
                h.shown = shown_seq;
                h.detail = error;
            }
            return h;
        }
        if (shown_seq != last_shown_) {     // a presentation really completed
            last_shown_ = shown_seq;
            last_progress_ = now;
            stuck_ = false;
        }
        const bool pending = published_seq > shown_seq;
        if (!pending) {                     // idle: keep the baseline fresh
            last_progress_ = now;
            stuck_ = false;
            return h;
        }
        if (!seen_) {                       // first pending sample: start the clock
            seen_ = true;
            last_progress_ = now;
            return h;
        }
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now - last_progress_).count();
        if (age >= stall_ms_ && !stuck_) {
            stuck_ = true;
            h.kind = Health::Stalled;
            h.published = published_seq;
            h.shown = shown_seq;
            h.age_ms = age;
            h.detail = "no completed presentation within the threshold";
        }
        return h;
    }

private:
    int stall_ms_;
    unsigned long last_shown_ = 0;
    bool seen_ = false;
    bool stuck_ = false;
    bool reported_failed_ = false;
    std::chrono::steady_clock::time_point last_progress_{};
};

}  // namespace p3_disp

#endif  // P3_DISP_PUBLISH_H