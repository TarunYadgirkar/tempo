// hand_inject.h — Mac-side hand tracking fed back into the phone's hand path.
//
// The iPhone's Vision hand tracking is the weak link in the gesture stack: it
// loses the hand on rotation and its fingertips wander by centimetres, so
// pinch/point/fist fire late or not at all. The fix runs the tracking on the
// Mac instead (MediaPipe Hand Landmarker over the streamed camera frames,
// fingertip depth from the LiDAR map) and pushes the result back in through
// `hands-inject`, which is a drop-in replacement for the 0x05 packet's joints.
//
// Everything here is pure math and parsing so tests can pin it without a
// receiver, a socket or a camera.
//
// ---------------------------------------------------------------------------
// Joint order
// ---------------------------------------------------------------------------
// MediaPipe's 21 hand landmarks and SB_JOINT_* (spatial_bridge.h, the order
// the 0x05 packet packs and the order gesture_engine.h's GE_JOINT_* mirrors)
// enumerate the same skeleton in the same sequence:
//
//   0 wrist                          0 SB_JOINT_WRIST
//   1..4  thumb  CMC MCP IP TIP      1..4  THUMB_CMC MCP IP TIP
//   5..8  index  MCP PIP DIP TIP     5..8  INDEX_MCP PIP DIP TIP
//   9..12 middle MCP PIP DIP TIP     9..12 MIDDLE_MCP PIP DIP TIP
//   13..16 ring  MCP PIP DIP TIP     13..16 RING_MCP PIP DIP TIP
//   17..20 pinky MCP PIP DIP TIP     17..20 PINKY_MCP PIP DIP TIP
//
// So the mapping is the identity. It is spelled out as a table anyway, and
// pinned by a test, because "they happen to agree today" is exactly the kind
// of fact that silently stops being true when either side renumbers.

#pragma once

#include <cstdint>
#include <string>

#include "spatial_bridge.h"

namespace mac_shell {

// Injection older than this stops overriding the phone hands.
constexpr uint64_t HAND_INJECT_FRESH_MS = 150;

// MediaPipe landmark index -> SB_JOINT_* index; -1 when out of range.
int mediapipe_to_sb_joint(int mp_index);
// Inverse, for the eval path that reports mac joints in MediaPipe terms.
int sb_to_mediapipe_joint(int sb_index);

// Unproject one pixel of a depth/colour image into ARKit CAMERA space
// (+X right, +Y up, -Z the view direction).
//
// `intr` is in pixels of intr.image_width x intr.image_height — ARKit's
// capture resolution, which is normally larger than the streamed JPEG and the
// depth map. `u`, `v` are pixel coordinates in an image of `img_w` x `img_h`
// (v growing downward, image convention); the two resolutions are related by
// a pure scale because both maps are colour-aligned. `depth_m` is metres
// measured ALONG THE OPTICAL AXIS, matching sb_depth_t.
//
// False for a non-positive depth (sb_depth_t's "no reading" sentinel) or
// degenerate intrinsics.
bool unproject_pixel(const sb_intrinsics_t &intr, float img_w, float img_h,
                     float u, float v, float depth_m, float out_cam[3]);

// Camera-space point -> scene frame, given the SCENE-frame camera pose that
// was current when the frame was exposed (scene::head_pose_at).
void camera_to_scene(const float cam_pos[3], const float cam_quat[4],
                     const float p_cam[3], float out[3]);

// One `hands-inject` payload after parsing. Joints are SCENE-frame metres in
// SB_JOINT_* order, ready to hand straight to the scene.
struct injected_hands {
    uint64_t t_ms = 0;  // sender's wall clock, ms since the Unix epoch
    int count = 0;      // 0..2
    sb_hand_t hands[2] = {};
    bool is_left[2] = {false, false};
};

// Parses {"t":ms,"hands":[{"chirality":"left|right","confidence":0..1,
// "joints":[[x,y,z] x21 in MediaPipe landmark order]}]}.
// Joints are reordered into SB_JOINT_* order on the way in. False + a short
// `err` for anything malformed or non-finite.
bool parse_hands_inject(const std::string &json, injected_hands &out,
                        std::string &err);

// Milliseconds since the Unix epoch — the clock `t` is measured against.
uint64_t hand_inject_now_ms();

// Latest injection plus its freshness. Holds no lock of its own; the scene
// owns it under the scene mutex.
class hand_inject_store {
   public:
    void set(const injected_hands &h);
    bool ever_set() const { return ever_; }
    // Age of the held injection in ms; 0 when nothing was ever injected.
    uint64_t age_ms(uint64_t now_ms) const;
    // True while the held injection is younger than HAND_INJECT_FRESH_MS and
    // so should replace the phone's hands.
    bool fresh(uint64_t now_ms) const;
    const injected_hands &hands() const { return hands_; }

   private:
    injected_hands hands_;
    uint64_t received_ms_ = 0;
    bool ever_ = false;
};

}  // namespace mac_shell
