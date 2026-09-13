// control_server.h — unix-socket control plane for mac-shell.
//
// Mirrors the semantics of vendor/wxrd/src/control.c (verb set, reply shapes,
// subscribe event stream, same-uid peer check) but runs on its own thread
// with poll() instead of the compositor's wl_event_loop. The wire layer —
// line parsing (control_protocol.c) and per-connection byte buffering
// (control_conn.c) — is compiled directly from the wxrd sources.
//
// Socket path: $SPATIAL_OS_SOCK, else $TMPDIR/spatial-os.sock (macOS has no
// $XDG_RUNTIME_DIR; $TMPDIR is the per-user 0700
// confstr(_CS_DARWIN_USER_TEMP_DIR) directory). With neither set the server
// refuses to start rather than bind a world-traversable /tmp path.

#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "control_conn.h"
#include "control_protocol.h"

#include "core/layout_store.h"

namespace mac_shell {

class scene;

class control_server {
   public:
    explicit control_server(scene &scene_ref);
    ~control_server();

    control_server(const control_server &) = delete;
    control_server &operator=(const control_server &) = delete;

    // Bind + listen + start the service thread. Returns false on failure
    // (reason logged to stderr and left in last_error()).
    bool start();
    void stop();

    const std::string &sock_path() const { return sock_path_; }
    // Human-readable reason the last start() failed — rendered on the shell's
    // startup-error card so a Dock launch explains itself without a terminal.
    const std::string &last_error() const { return error_; }

    // Stage 2 verb `launch-app <bundle-id-or-window-title>`: capture a real
    // mac window as a panel. The handler returns false when capture cannot
    // start (no permission / no renderer); the caller decides the fallback
    // before returning, so the verb still replies ok. Unset → err
    // unimplemented.
    using launch_app_fn = std::function<bool(const std::string &target)>;
    void set_launch_app_handler(launch_app_fn fn) {
        launch_app_ = std::move(fn);
    }

    // `permissions` / `stats` verbs: handlers return the "k=v k=v" tail of an
    // ok reply. permissions unset → "ok screen_capture=0 accessibility=0"
    // (truthful on builds without the capture layer); stats unset → err
    // unimplemented.
    using status_fn = std::function<std::string()>;
    void set_permissions_handler(status_fn fn) {
        permissions_ = std::move(fn);
    }
    void set_stats_handler(status_fn fn) { stats_ = std::move(fn); }

    // `screenshot [<path>]`: grab the next composited frame to a PNG. The
    // handler blocks until the renderer has written it (or gives up), so the
    // reply only ever names a file that exists. Unset (headless / no
    // renderer) → err unimplemented.
    using screenshot_fn =
        std::function<bool(const std::string &path, std::string &err)>;
    void set_screenshot_handler(screenshot_fn fn) {
        screenshot_ = std::move(fn);
    }

    // `screenshot <handle>`: one panel's own pixels, pull-based (no
    // renderer). Unset → err unsupported.
    using window_shot_fn = std::function<bool(
        uint64_t handle, const std::string &path, std::string &err)>;
    void set_window_shot_handler(window_shot_fn fn) {
        window_shot_ = std::move(fn);
    }

    // `frame-export on <dir> | off | status`: publish the live camera frame,
    // its depth map and the pose it was taken under into a directory, for a
    // vision process outside the shell (platform/frame_export.h). The handler
    // takes the raw argument tail, fills `reply` with the "k=v" tail of an ok
    // line, and on refusal returns false with a one-word `err` code. Unset →
    // err unimplemented.
    using frame_export_fn = std::function<bool(
        const std::string &arg, std::string &reply, std::string &err)>;
    void set_frame_export_handler(frame_export_fn fn) {
        frame_export_ = std::move(fn);
    }

   private:
    struct conn;

    void run();
    void accept_conns();
    void handle_conn(conn &c, short revents);
    void flush_conn(conn &c);
    void close_conn(size_t idx);
    void broadcast_events();
    void dispatch(conn &c, const ctl_request_t &req);
    static void on_line_trampoline(ctl_conn_t *cc, const char *line,
                                   void *user);
    void on_line(conn &c, const char *line);

    // Handles `screenshot`, which the vendored parser only knows in its
    // `screenshot <handle>` form. Returns false when the line is not one.
    bool handle_screenshot(conn &c, const char *line);

    // `layout load` for a saved captured_window panel: re-runs `launch-app`
    // and returns the handle of the panel that appeared, or 0 when nothing
    // did (no renderer, no permission, the app is gone).
    uint64_t relaunch_captured(const layout_panel &saved);

    scene &scene_;
    launch_app_fn launch_app_;
    status_fn permissions_;
    status_fn stats_;
    screenshot_fn screenshot_;
    window_shot_fn window_shot_;
    frame_export_fn frame_export_;
    std::string sock_path_;
    std::string error_;
    int listen_fd_ = -1;
    int wake_pipe_[2] = {-1, -1};
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::vector<conn *> conns_;
};

// Empty when no per-user location is available (see the header comment).
std::string resolve_control_sock_path();

}  // namespace mac_shell
