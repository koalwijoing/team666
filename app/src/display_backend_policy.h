#ifndef P3_DISPLAY_BACKEND_POLICY_H
#define P3_DISPLAY_BACKEND_POLICY_H

#include <cctype>
#include <string>

namespace p3_display {

// Pure selection policy. P3_DISPLAY_THREAD is a request, not permission to
// create Qt GUI objects outside main. No GUI calls or environment mutation.
struct BackendPolicy {
    std::string backend;
    bool use_worker = false;
    const char* selected = "none";
    const char* reason = "display-off";
};

inline std::string gui_backend(const std::string& build_information) {
    std::size_t begin = 0;
    while (begin < build_information.size()) {
        const auto end = build_information.find('\n', begin);
        const auto length = (end == std::string::npos ? build_information.size() : end) - begin;
        auto line = build_information.substr(begin, length);
        auto first = line.find_first_not_of(" \t\r");
        if (first != std::string::npos && line.compare(first, 4, "GUI:") == 0) {
            first = line.find_first_not_of(" \t\r", first + 4);
            if (first == std::string::npos) return "UNKNOWN";
            const auto last = line.find_first_of(" \t\r", first);
            auto value = line.substr(first, last == std::string::npos ? last : last - first);
            for (auto& c : value) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            return value;
        }
        if (end == std::string::npos) break;
        begin = end + 1;
    }
    return "UNKNOWN";
}

inline BackendPolicy choose_backend(bool display, bool requested_worker,
                                    const std::string& build_information) {
    BackendPolicy policy;
    policy.backend = gui_backend(build_information);
    if (!display) return policy;
    policy.selected = "main";
    policy.reason = "worker-not-requested";
    if (!requested_worker) return policy;
    if (policy.backend.compare(0, 2, "QT") == 0) {
        policy.reason = "qt-main-thread-only";
        return policy;
    }
    // Only retain the already-tested legacy worker on its known backend.
    // An unrecognized future backend must not silently acquire a worker GUI.
    if (policy.backend.compare(0, 3, "GTK") == 0) {
        policy.use_worker = true;
        policy.selected = "worker";
        policy.reason = "legacy-gtk-worker";
    } else {
        policy.reason = "unverified-backend-main-only";
    }
    return policy;
}

}  // namespace p3_display
#endif
