// frame_export.h — the `frame-export` control verb: publish the live camera
// frame, its depth map and the pose/intrinsics it was taken under, so a
// process outside the shell can do vision work on it.
//
// This is the Mac-side hand tracking's input tap. MediaPipe runs in a Python
// process (hands/); it needs the colour image to find landmarks, the LiDAR
// depth map to turn each landmark into metres, and the intrinsics plus the
// SCENE-frame head pose AT CAPTURE TIME to put those metres in the same frame
// the panels live in. Sending all four over the control socket would flood it,
// so they go to a directory instead and the socket only carries the switch.
//
// Files, all rewritten in place at up to FRAME_EXPORT_MAX_HZ:
//
//   <dir>/latest.jpg     the camera image, re-encoded from the decoded RGBA
//   <dir>/latest.depth   raw float32 metres, row-major, 0 = no reading
//   <dir>/latest.json    the sidecar (below)
//
// Each is written to a temp file in the same directory and rename()d into
// place, so a reader never sees a half-written file. The JSON is renamed
// LAST and carries the frame timestamp, which makes it the commit record:
// poll latest.json, and the .jpg and .depth it describes are already there.
//
// Sidecar shape:
//   {"t_ns":<frame timestamp, ns since the Unix epoch>,
//    "seq":<monotonic export counter>,
//    "image":{"width":w,"height":h,"file":"latest.jpg"},
//    "head":{"frame":"scene","pos":[x,y,z],"quat":[x,y,z,w]} | null,
//    "intrinsics":{"fx":..,"fy":..,"cx":..,"cy":..,
//                  "image_width":..,"image_height":..} | null,
//    "depth":{"width":w,"height":h,"file":"latest.depth",
//             "format":"float32","t_ns":..} | null}
//
// `head` is null before a world origin is captured, `intrinsics` null until
// the first 0x0A packet, `depth` null until the first 0x0B — a consumer must
// handle all three, because a run can legitimately have colour and nothing
// else. The intrinsics are in ARKit capture pixels, which are NOT the JPEG's
// pixels; scale by image_width/width (core/hand_inject.h unproject_pixel does
// exactly this).

#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include "spatial_bridge.h"

namespace mac_shell {

class scene;

// The phone streams ~30 Hz; MediaPipe on the Mac keeps up with about half
// that, and re-encoding a JPEG per frame is not free. 15 Hz is the cap.
constexpr int FRAME_EXPORT_MAX_HZ = 15;

class frame_export {
   public:
    // Turn export on, writing into `dir` (created if missing). The directory
    // must resolve inside $TMPDIR or $HOME — the control socket is reachable
    // by anything running as the user, and this verb is a file-write
    // primitive. On refusal returns false with a one-word `err`.
    bool enable(const std::string &dir, std::string &err);
    void disable();
    // "on dir=<path> frames=<n> hz=<cap>" / "off frames=<n>".
    std::string status() const;
    // Cheap enough to call per rendered frame before decoding anything.
    bool enabled() const { return enabled_.load(std::memory_order_relaxed); }

    // Publish one decoded camera frame. Pulls the matching depth map and the
    // pose at the frame's timestamp from `world`, and the intrinsics from
    // `rx` (which is latest-wins, so reading it here does not starve the
    // renderer). Silently returns when export is off or the 15 Hz gate has
    // not opened; failures are logged once per transition, never fatal.
    void offer_frame(const sb_frame_t &frame, const scene &world,
                     sb_receiver_t *rx);

   private:
    std::atomic<bool> enabled_{false};
    mutable std::mutex mutex_;
    std::string dir_;
    uint64_t frames_ = 0;
    uint64_t last_write_ns_ = 0;
    bool warned_ = false;
};

// The one exporter for the process. A singleton because the two places that
// hold a decoded camera frame — the renderer's passthrough upload and the
// headless tick loop — are on opposite sides of the renderer build flag, and
// threading a pointer between them buys nothing over naming the same object.
frame_export &frame_export_instance();

// Handler body for the `frame-export on <dir> | off | status` verb. Split out
// so main.cpp's wiring is one line and the verb's grammar is tested here.
bool frame_export_verb(frame_export &fx, const std::string &arg,
                       std::string &reply, std::string &err);

}  // namespace mac_shell
