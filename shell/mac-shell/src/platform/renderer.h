// renderer.h — Stage 2 Metal renderer (AppKit + MTKView).
//
// run_renderer_app owns the UI event loop: it ticks the scene from the
// MTKView draw callback at 60 fps, renders passthrough + panels + hands +
// keyboard placeholder, and returns when the window closes or `running`
// drops. Head pose drives the view matrix when the bridge has pose data;
// otherwise a fly-cam (WASD + mouse drag) substitutes.
//
// env:
//   SPATULA_MAC_SCREENSHOT=<path.png>  write a framebuffer grab ~2 s in
//                                      (the `screenshot` control verb takes
//                                      the same grab, on demand)
//   SPATULA_MAC_EXIT_AFTER=<seconds>   quit automatically (verification runs)
//   SPATULA_MAC_METALLIB=<path>        override the compiled shader library
//   SPATULA_MAC_NO_BONJOUR=1           don't advertise _spatialbridge._udp
//                                      (test / screenshot runs)

#pragma once

#include <atomic>
#include <string>

#include "spatial_bridge.h"

namespace mac_shell {

class scene;
class capture_manager;
class port_retry;
class frame_grabber;

// Launch facts the HUD needs: the UDP port (welcome card shows it, and the
// Bonjour advertisement carries it), replay mode (no Bonjour chip — nothing
// advertises in replay), and an explainable startup error (e.g. port already
// in use) rendered as an in-window error card instead of a silent log line.
struct renderer_boot_info {
    int udp_port = 9898;
    bool replay = false;
    std::string startup_error_title;  // card headline
    std::string startup_error;        // full sentence, wrapped under it
    // Polled from the render tick; clears the card and starts advertising
    // when a busy UDP port finally opens.
    port_retry *retry = nullptr;
    // Drained every frame: serves the `screenshot` control verb and the
    // SPATULA_MAC_SCREENSHOT env grab from the same drawable.
    frame_grabber *grabber = nullptr;
};

int run_renderer_app(scene &scene_ref, sb_receiver_t *receiver,
                     capture_manager *capture, std::atomic<bool> &running,
                     const renderer_boot_info &boot);

}  // namespace mac_shell
