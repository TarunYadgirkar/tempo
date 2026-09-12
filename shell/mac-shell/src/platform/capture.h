// capture.h — ScreenCaptureKit window capture: real mac app windows as
// spatial panels.
//
// launch_app() looks up an on-screen window by bundle id or title substring,
// starts an SCStream for it, and spawns a captured panel in the scene. When
// Screen Recording permission is absent it degrades gracefully: requests
// access once via CGRequestScreenCaptureAccess (interactive runs only — never
// under SPATULA_MAC_HEADLESS=1, so CI never blocks on the TCC prompt), logs
// guidance, and spawns an internal test-card panel instead.
//
// Input injection into captured windows uses CGEvent posts, gated three
// ways: the panel must be focused (scene routes input to the focused panel
// only), injection must not be disabled (`SPATULA_MAC_INJECT=0`; it is ON by
// default for interactive runs and always off headless), AND the process must
// be Accessibility-trusted (AXIsProcessTrusted; interactive runs prompt once
// via AXIsProcessTrustedWithOptions).
//
// Focusing a captured panel also fires an `inject_event::kind::focus`, which
// makes the source window main+focused over the Accessibility API and raises
// it — a CGEventPostToPid keystroke is routed by the target app to whatever
// it considers its key window, so without this the keys land in whichever
// window that app happened to leave focused.

#pragma once

#include <cstdint>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace mac_shell {

class scene;

// TCC status probes for the `permissions` control verb. Read-only: neither
// ever triggers an OS prompt, so they are safe headless/CI.
bool screen_capture_permitted();
bool accessibility_permitted();

// Debounce for the source-window geometry poll: a window can be absent from
// one SCShareableContent reply while it is mid-resize/space-switch, so a
// stream only counts as ended after consecutive misses. Header-only so the
// headless test can pin it without ScreenCaptureKit.
constexpr int GEOMETRY_MISSES_BEFORE_ENDED = 2;

struct geometry_miss_tracker {
    int misses = 0;

    // Returns true when the stream should be treated as ended.
    bool note(bool found) {
        misses = found ? 0 : misses + 1;
        return misses >= GEOMETRY_MISSES_BEFORE_ENDED;
    }
};

// Capture-surface px per window point (SCStreamConfiguration width/height and
// the click-injection mapping must agree on this).
constexpr double CAPTURE_SCALE = 2.0;

struct screen_point {
    double x = 0, y = 0;
};

// Surface-local capture px → global screen point.
//
// A pure translate, no Y flip: SCWindow.frame and kCGWindowBounds are the same
// rect for the same windowID (verified at runtime — top-left-origin global
// display space, y = 33 for a window sitting under the menu bar of a 982 pt
// main display), and that is also the space CGEventCreateMouseEvent takes.
// Displays placed above/left of the main one give the origin negative
// components in that same space, which the translate carries through.
// Header-only so the headless test can pin it without ScreenCaptureKit.
inline screen_point window_point(double frame_x, double frame_y,
                                 double surface_x, double surface_y) {
    return {frame_x + surface_x / CAPTURE_SCALE,
            frame_y + surface_y / CAPTURE_SCALE};
}

// Key-name → macOS virtual keycode for the handful of names the control
// plane's `key` verb sends to captured windows. Unknown names resolve only
// when they are a plain non-negative integer literal in range; anything else
// is rejected so a typo cannot post an arbitrary keycode (an empty string used
// to parse as 0, i.e. `a`). Header-only so the headless test can pin it.
constexpr unsigned long MAX_VIRTUAL_KEYCODE = 0x7f;

inline bool keycode_for_name(const std::string &name, uint16_t &out) {
    static const std::map<std::string, uint16_t> table = {
        {"Return", 36},    {"Tab", 48},    {"space", 49}, {"Escape", 53},
        {"BackSpace", 51}, {"Delete", 51}, {"Left", 123}, {"Right", 124},
        {"Down", 125},     {"Up", 126},
    };
    auto it = table.find(name);
    if (it != table.end()) {
        out = it->second;
        return true;
    }
    if (name.empty())
        return false;
    for (char c : name)
        if (c < '0' || c > '9')
            return false;
    char *end = nullptr;
    unsigned long v = std::strtoul(name.c_str(), &end, 10);
    if (!end || *end != '\0' || v > MAX_VIRTUAL_KEYCODE)
        return false;
    out = (uint16_t)v;
    return true;
}

class capture_manager {
   public:
    explicit capture_manager(scene &scene_ref);
    ~capture_manager();

    capture_manager(const capture_manager &) = delete;
    capture_manager &operator=(const capture_manager &) = delete;

    // Async: resolves the target and spawns the panel from a background
    // completion. Returns false when capture is impossible up-front (no
    // Screen Recording permission) — an internal fallback panel is spawned
    // either way, so callers still reply ok.
    bool launch_app(const std::string &target);

    // Latest captured frame for a panel, as a +1 retained CVPixelBufferRef
    // (call CVPixelBufferRelease). nullptr when no frame yet / not captured.
    void *copy_latest_pixel_buffer(uint64_t handle);

    // `screenshot <handle>`: one panel's own pixels — the captured window's
    // latest frame, or an internal panel's CPU surface — never the composited
    // scene, so passthrough camera frames cannot leak into a per-window shot
    // (same rule as wxrd's window_capture.c). Pull-based: no renderer needed.
    // False with a one-word `err` (no_such_window, no_frame, …).
    bool capture_panel_png(uint64_t handle, const std::string &path,
                           std::string &err);

    // Stop streams whose panel is gone.
    void gc(const std::vector<uint64_t> &live_handles);

    // Wire the CGEvent injector into the scene (reads SPATULA_MAC_INJECT).
    void install_input_injector();

   private:
    struct impl;
    std::shared_ptr<impl> impl_;
    static void attempt_capture(std::weak_ptr<impl> weak, std::string want,
                                int attempts_left);
};

}  // namespace mac_shell
