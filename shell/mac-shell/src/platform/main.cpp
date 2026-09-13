// main.cpp — mac-shell entry point.
//
// Flags:
//   --replay <session.bin>   replay a recorded session (loops) instead of UDP
//   --port <n>               live UDP port (default 9898)
//   --once                   replay the file once instead of looping
//
// SPATULA_MAC_HEADLESS=1 → no UI event loop: tick thread + control socket
// only (CI mode, mirrors WXRD_HEADLESS=1). Without it the Metal renderer
// window opens (Stage 2).
//
// A busy UDP port starts the shell deaf rather than killing it; the tick
// loop retries the bind (platform/port_retry.h). A control socket another
// Spatula still owns is fatal headless and an error card otherwise.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include <csignal>

#include "spatial_bridge.h"

#include "platform/control_server.h"
#include "platform/frame_grab.h"
#include "platform/port_retry.h"
#include "core/scene.h"

#if defined(__APPLE__) && defined(MAC_SHELL_HAVE_RENDERER)
#include "platform/capture.h"
#include "platform/frame_export.h"
#include "platform/renderer.h"
#endif

namespace {

std::atomic<bool> g_running{true};

void handle_signal(int) { g_running = false; }

void usage(const char *argv0) {
    std::fprintf(stderr,
                 "usage: %s [--replay session.bin] [--port N] [--once]\n"
                 "env:   SPATULA_MAC_HEADLESS=1  no UI, tick thread + control "
                 "socket only\n"
                 "       SPATIAL_OS_SOCK         control socket path override\n",
                 argv0);
}

}  // namespace

int main(int argc, char **argv) {
    const char *replay_path = nullptr;
    int port = 9898;
    bool loop = true;

    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--replay") == 0 && i + 1 < argc) {
            replay_path = argv[++i];
        } else if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--once") == 0) {
            loop = false;
        } else if (std::strcmp(argv[i], "--help") == 0 ||
                   std::strcmp(argv[i], "-h") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            std::fprintf(stderr, "mac-shell: unknown argument '%s'\n",
                         argv[i]);
            usage(argv[0]);
            return 2;
        }
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    std::signal(SIGPIPE, SIG_IGN);

    const char *headless_env = getenv("SPATULA_MAC_HEADLESS");
    bool headless = headless_env && std::strcmp(headless_env, "1") == 0;

    // A busy UDP port is not fatal in either mode: the shell comes up deaf,
    // says so (error card when there's a window), and keeps retrying the
    // bind so it recovers the moment the other process quits.
    std::string startup_error_title;
    std::string startup_error;
    std::atomic<sb_receiver_t *> receiver{nullptr};
    bool port_busy = false;
    if (replay_path) {
        sb_receiver_t *r = sb_receiver_from_file(replay_path, loop);
        if (!r) {
            std::fprintf(stderr, "mac-shell: cannot open replay file %s\n",
                         replay_path);
            return 1;
        }
        receiver.store(r);
        std::fprintf(stderr, "mac-shell: replaying %s%s\n", replay_path,
                     loop ? " (loop)" : "");
    } else {
        sb_receiver_t *r = sb_receiver_create((uint16_t)port);
        if (!r) {
            std::fprintf(stderr, "mac-shell: cannot bind UDP port %d\n", port);
            port_busy = true;
            char msg[192];
            std::snprintf(msg, sizeof(msg),
                          "UDP port %d is already in use - another app (or a "
                          "second Spatula) grabbed it first. Quit that one "
                          "and Spatula connects itself.",
                          port);
            startup_error_title = "Port didn't open";
            startup_error = msg;
        } else {
            receiver.store(r);
            std::fprintf(stderr, "mac-shell: listening on UDP %d\n", port);
        }
    }
    if (sb_receiver_t *r = receiver.load())
        sb_start(r);

    mac_shell::scene world;
    world.set_receiver(receiver.load());

    mac_shell::port_retry retry(world, receiver, port);
    if (port_busy)
        retry.arm(0.0);

    // SPATULA_MAC_DEPTH_OCCLUSION=0|1|soft — live-comparable LiDAR occlusion
    // mode (also switchable at runtime via `depth-occlusion on|off|soft`).
    if (const char *docc = getenv("SPATULA_MAC_DEPTH_OCCLUSION")) {
        if (std::strcmp(docc, "0") == 0 || std::strcmp(docc, "off") == 0)
            world.set_depth_occlusion_mode(0);
        else if (std::strcmp(docc, "1") == 0 || std::strcmp(docc, "on") == 0)
            world.set_depth_occlusion_mode(1);
        else if (std::strcmp(docc, "soft") == 0)
            world.set_depth_occlusion_mode(2);
    }

    // SPATULA_MAC_HAND_DEBUG=0 — start without the 21-joint skeleton overlay
    // (also toggled at runtime via `hands overlay on|off`).
    if (const char *hd = getenv("SPATULA_MAC_HAND_DEBUG"))
        world.set_hand_overlay(std::strcmp(hd, "1") == 0);

    // Same policy as the UDP bind: fatal headless, explained on a card when
    // there is a window. Exiting silently is how a Dock launch used to look
    // like nothing happened at all.
    mac_shell::control_server control(world);
    if (!control.start()) {
        std::fprintf(stderr, "mac-shell: control plane failed to start\n");
        if (headless) {
            if (sb_receiver_t *r = receiver.load()) {
                sb_stop(r);
                sb_receiver_destroy(r);
            }
            return 1;
        }
        startup_error_title = "Spatula can't start";
        startup_error = control.last_error();
    }

    // By reference: the retry loop may swap a receiver in later.
    control.set_stats_handler([&world, &receiver]() {
        sb_receiver_t *r = receiver.load();
        sb_intrinsics_t intr;
        char buf[256];
        std::snprintf(
            buf, sizeof(buf),
            "packet_rate=%.1f panels=%zu frame_age_ms=%lld have_intrinsics=%d "
            "view_lag_ms=%.1f",
            r ? (double)sb_get_packet_rate(r) : 0.0, world.panel_count(),
            (long long)(r ? sb_get_frame_age_ms(r) : -1),
            r && sb_get_latest_intrinsics(r, &intr) ? 1 : 0,
            (double)world.view_lag_ms());
        return std::string(buf);
    });

    mac_shell::frame_grabber grabber;

#if defined(__APPLE__) && defined(MAC_SHELL_HAVE_RENDERER)
    // `frame-export on <dir>|off|status` — the camera/depth/pose tap the
    // mac-side hand tracker reads (platform/frame_export.h). Wired in both
    // modes: the renderer feeds it the frame it already decoded, and the
    // headless tick loop below pulls one itself.
    control.set_frame_export_handler([](const std::string &arg,
                                        std::string &reply, std::string &err) {
        return mac_shell::frame_export_verb(
            mac_shell::frame_export_instance(), arg, reply, err);
    });
    // Capture layer runs in both modes so the verb set stays uniform;
    // headless launch-app degrades to a test card (permission preflight
    // fails without a stable app identity / window server).
    mac_shell::capture_manager capture(world);
    capture.install_input_injector();
    control.set_launch_app_handler(
        [&capture](const std::string &target) {
            return capture.launch_app(target);
        });
    // Radial-launcher app targets go through the same capture path.
    world.set_app_launcher(
        [&capture](const std::string &target) { capture.launch_app(target); });
    control.set_permissions_handler([]() {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "screen_capture=%d accessibility=%d",
                      mac_shell::screen_capture_permitted() ? 1 : 0,
                      mac_shell::accessibility_permitted() ? 1 : 0);
        return std::string(buf);
    });
    // Per-window `screenshot <handle>` is pull-based (the panel's own latest
    // pixels), so it works headless too; the whole-scene form needs the
    // renderer and is wired below.
    control.set_window_shot_handler([&capture](uint64_t handle,
                                               const std::string &path,
                                               std::string &err) {
        return capture.capture_panel_png(handle, path, err);
    });
    if (!headless) {
        control.set_screenshot_handler(
            [&grabber](const std::string &path, std::string &err) {
                return grabber.grab(
                    path,
                    std::chrono::milliseconds(mac_shell::SHOT_TIMEOUT_MS),
                    err);
            });
        mac_shell::renderer_boot_info boot;
        boot.udp_port = port;
        boot.grabber = &grabber;
        boot.replay = replay_path != nullptr;
        boot.startup_error_title = startup_error_title;
        boot.startup_error = startup_error;
        boot.retry = &retry;
        int rc = mac_shell::run_renderer_app(world, receiver.load(), &capture,
                                             g_running, boot);
        control.stop();
        if (sb_receiver_t *r = receiver.load()) {
            sb_stop(r);
            sb_receiver_destroy(r);
        }
        return rc;
    }
#else
    if (!headless)
        std::fprintf(stderr,
                     "mac-shell: renderer not built — running headless\n");
#endif

    // Headless: fixed-cadence tick loop.
    using clock = std::chrono::steady_clock;
    auto boot_at = clock::now();
    auto last = boot_at;
    while (g_running) {
        auto now = clock::now();
        float dt = std::chrono::duration<float>(now - last).count();
        last = now;
        retry.poll(std::chrono::duration<double>(now - boot_at).count());
        world.tick(dt);
#if defined(__APPLE__) && defined(MAC_SHELL_HAVE_RENDERER)
        // Headless has no renderer to consume camera frames, so the exporter
        // does it here. Only while export is on: sb_get_latest_frame hands
        // ownership over, and draining frames nobody looks at is pure waste.
        if (mac_shell::frame_export_instance().enabled()) {
            if (sb_receiver_t *r = receiver.load()) {
                sb_frame_t frame;
                if (sb_get_latest_frame(r, &frame)) {
                    mac_shell::frame_export_instance().offer_frame(frame, world,
                                                                   r);
                    sb_free_frame(&frame);
                }
            }
        }
#endif
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    control.stop();
    if (sb_receiver_t *r = receiver.load()) {
        sb_stop(r);
        sb_receiver_destroy(r);
    }
    return 0;
}
