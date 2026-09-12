// features.h — Hand feature extraction from raw joint data.
// Internal header; not part of the public API.

#pragma once

#include "gesture_engine.h"
#include <cmath>
#include <string>
#include <unordered_map>

namespace ge {

// 3D vector math (avoid pulling in a full math library for 5 operations)
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    float dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3 &o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    float length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const {
        float l = length();
        return l > 1e-8f ? Vec3{x / l, y / l, z / l} : Vec3{};
    }
};

inline float distance(const Vec3 &a, const Vec3 &b) { return (a - b).length(); }

// Get joint position as Vec3 from ge_hand_t
inline Vec3 joint_pos(const ge_hand_t &hand, int joint_idx) {
    return {hand.joints[joint_idx][0], hand.joints[joint_idx][1], hand.joints[joint_idx][2]};
}

inline float joint_conf(const ge_hand_t &hand, int joint_idx) {
    return hand.joints[joint_idx][3];
}

// ---------------------------------------------------------------------------
// Feature values computed per hand per frame
// ---------------------------------------------------------------------------

struct HandFeatures {
    // Finger-tip to thumb-tip distances (meters)
    float thumb_index_distance;
    float thumb_middle_distance;
    float thumb_ring_distance;
    float thumb_pinky_distance;

    // Per-hand metric scale (m): mean of the index/middle/ring proximal
    // phalanx (rigid bones). The hand is normalized to a canonical scale
    // about the wrist before shape-feature extraction, so every distance
    // feature here is scale-invariant (independent of hand size / the capture
    // pipeline's metric scale). Exposed via the public ge_hand_scale().
    float hand_scale_m;

    // Finger curl angles (0 = fully extended, 1 = fully curled)
    float index_curl;
    float middle_curl;
    float ring_curl;
    float pinky_curl;
    float thumb_curl;
    float all_fingers_curl;  // average of index/middle/ring/pinky curls

    // Palm openness (0 = fist, 1 = fully open)
    float palm_openness;

    // Thumb position projected onto index finger bone chain (0..1)
    float thumb_on_index_projection;

    // Thumb position projected onto middle finger bone chain (0..1)
    // Used for projection-based middle-finger pinch detection.
    float thumb_on_middle_projection;

    // Perpendicular distance from thumb tip to the index finger axis
    // (line from INDEX_MCP to INDEX_TIP).  Small when the thumb is near
    // the side of the index finger (scroll gesture trigger).
    float thumb_to_index_line_distance;

    // Signed displacement of the thumb from the index TIP, projected
    // onto the finger's pointing direction (the DIP→TIP unit vector).
    // Positive = thumb is AHEAD of the tip in the direction the finger
    // points (the natural pinch-closing direction).  Negative = thumb
    // is BEHIND the tip (further down the finger toward MCP).  Near
    // zero = thumb is BESIDE the tip in the perpendicular plane.
    //
    // This is the key signal that distinguishes a loose-pinch (thumb
    // hovering near where the index tip points, ~25 mm ahead) from a
    // scroll-pose (thumb sliding along the side, projection near
    // zero or negative).  Distance-from-tip alone can't tell them
    // apart when the user's pinch doesn't fully close.
    float thumb_along_index_tip_direction;

    // Cosine of the angle between the middle finger's last segment
    // (DIP→TIP) and the vector from MIDDLE_TIP to THUMB_TIP.
    //
    // ≈ +1: the middle fingertip is pointing TOWARD the thumb tip —
    //       the natural pose of a 3-finger right-click where the
    //       middle is "aimed at" the thumb-index pinch.
    // ≈  0: middle fingertip pointing perpendicular to the thumb
    //       (extended away in a different direction).
    // ≈ -1: middle pointing AWAY from the thumb.
    //
    // For a "loose" 3-finger right-click where the middle fingertip
    // doesn't fully reach the thumb (clip 8b3d04, thumb-middle stays
    // at 70-80 mm) this cosine still reads +0.85+ because the user
    // has aimed the middle finger at the pinch even without contact —
    // a signal pose-distance alone misses.
    float middle_tip_direction_to_thumb_cos;

    // Fingertip gather radius: distance from centroid of all 5 fingertips to
    // the farthest tip.  Small when all fingertips touch together (gather
    // gesture).  Distinguished from a fist by requiring low curl (fingers
    // extended, not curled).
    float fingertip_gather_radius;

    // Palm normal: cross product of (INDEX_MCP-WRIST) x (PINKY_MCP-WRIST),
    // normalized.  Points in the palm-facing direction (verified from recording).
    Vec3 palm_normal;

    // Key positions (world space)
    Vec3 palm_center;
    Vec3 pinch_midpoint;    // midpoint between thumb tip and index tip
    Vec3 wrist_pos;

    // Minimum confidence across all joints
    float min_confidence;

    // Per-fingertip tracker confidence (0..1).  iPhone Vision drops
    // the middle/ring/pinky confidence to 0.10-0.30 when the finger
    // is occluded by an active thumb-index pinch — engine code can
    // gate trigger conditions on these so it doesn't reject a pose
    // (e.g. 3-finger right-click) just because the tracker can't
    // confirm an occluded finger's position.
    float thumb_tip_confidence;
    float index_tip_confidence;
    float middle_tip_confidence;
    float ring_tip_confidence;
    float pinky_tip_confidence;

    // Time (ms) since the palm normal last flipped sign — a fist_
    // launcher rotation flips the palm Y component from -0.85ish to
    // +0.85ish (or vice versa) over ~33 ms.  Real scroll / pinch
    // poses keep the palm orientation steady.  Engine code populates
    // this from a per-hand temporal tracker; compute_features
    // initialises it to a large constant so a hand seen for the
    // first time isn't treated as having just flipped.  Gestures
    // gate "must not be transitioning into a fist" by requiring this
    // value > 500 ms or so.
    float palm_y_flip_recency_ms;

    // Time (ms) since thumb_index_distance last exceeded 30 mm.
    // A real pinch_select / right_click is a CONVERGENCE: thumb
    // starts apart from the index and approaches.  Clip 553e7e
    // holds ti=70 mm for 800 ms then drops to 9 mm.  Clip 39c1bf
    // has ti=12 mm from the very first frame — no convergence,
    // just a held pose.  Gestures gate this < 500 ms so a fresh
    // converge fires but a long-held "resting thumb at tip" pose
    // doesn't.
    float thumb_index_recency_above_30mm_ms;

    // Time (ms) since thumb_middle_distance last exceeded 60 mm.
    // Same idea as ti convergence but on the 3-finger-pinch
    // dimension.  Clip db9395 starts with tm=32 mm already
    // (thumb already between index and middle from frame 0) so a
    // pure-pose right_click trigger fires before scroll can
    // claim — the tm gate suppresses that.  Real right_click clips
    // (af0461, 5ca5ef, 778bb2, 8b3d04, 0b7036) all show tm > 60 mm
    // within the 67-370 ms preceding the trigger.
    float thumb_middle_recency_above_60mm_ms;

    // Time (ms) since this hand last received a pinch_select END
    // (or CANCEL).  Used by a second pinch_select variant with a
    // shorter min_hold so a quick double-tap's brief 2nd beat
    // (clip ad376d 2nd beat = 67 ms, aab512 2nd beat = 100 ms)
    // can fire even though the same hold would let a fist
    // transit fire spuriously.  The recent-pinch context tells
    // us the user is intentionally tapping.  Initialised to 1e6
    // so a fresh hand without any pinch history uses the long
    // tight-variant hold.
    float pinch_select_end_recency_ms;

    // Time (ms) since palm_normal.y last dipped below -0.5
    // (palm facing AWAY from camera).  Real scroll / pinch clips
    // keep the palm facing the camera (py ~+0.8) the whole time.
    // Fist_launcher clips start with palm DOWN (py ~-0.8) then
    // flip during the rotation.  After the flip py is positive
    // and other scroll triggers can match — wide-arc-scroll uses
    // this gate to reject "still recovering from a fist" clips
    // (4ad403, 9330e9, 14759f, b5bec9).
    float palm_y_below_neg_half_recency_ms;

    // Shape-of-convergence features. Capture the MOTION SIGNATURE of a
    // pinch (peak → valley → release) independently of the user's resting
    // distance. The existing recency features (above) assume an open-hand
    // rest with thumb-index > 30 mm; the user's natural rest is 12–18 mm,
    // so those gates can never fire on a tight-rest pinch. The shape
    // features are dimensionally different: ratio is unitless, velocity
    // is m/s; a held resting hand stays at ratio ≈ 0 and velocity ≈ 0
    // regardless of how tight the rest is. Computed by a ring-buffer
    // tracker in engine.cpp; compute_features initialises them to safe
    // "no shape evidence" defaults (ratio 0, velocity 0, min = current).
    float thumb_index_distance_min_over_50ms;    // m  — 50ms window (noise-robust release)
    float thumb_index_distance_min_over_200ms;   // m  — trailing-window min
    float thumb_index_convergence_velocity_mps;  // m/s — max closing speed in last 200 ms
    float thumb_index_convergence_ratio;         // (peak-valley)/peak over last 500 ms
    float thumb_middle_distance_min_over_200ms;  // m
    float thumb_middle_convergence_velocity_mps; // m/s
    float thumb_middle_convergence_ratio;        // dimensionless

    // Trailing-window MAX of thumb_to_index_line_distance (perpendicular
    // distance from thumb tip to the index polyline). For a real pinch
    // the thumb stays near the index axis throughout — perp_max stays
    // low. For a fist transit the thumb sweeps PAST the index axis on
    // its way into the closed-fist position; perp briefly dips low
    // (≈ 0.004 m) as it crosses but the surrounding frames have perp
    // ≥ 0.02 m. The windowed-max distinguishes "was on-axis throughout"
    // from "transited the axis briefly".
    float thumb_to_index_line_distance_max_over_200ms;  // m

    // Trailing-window MAX of middle_curl and ring_curl. The iPhone Vision
    // tracker drops middle/ring fingertip confidence when those fingers
    // are CURLED into the palm (clip cadaef, fist_launcher): the curl
    // reading flickers between 0.03 and 0.85 on the same held fist as
    // the tracker falls back to a low-confidence prior on the occluded
    // fingertip. Per-frame middle_curl > 0.35 gates this off, killing
    // the 350-ms hold every time the flicker hits the low value. The
    // windowed-max stays committed at the high curl value while the
    // fingertip is briefly mis-tracked.
    float index_curl_max_over_200ms;
    float middle_curl_max_over_200ms;
    float ring_curl_max_over_200ms;

    // Trailing-window MIN of middle_tip_confidence. The occluded-middle
    // pinch_right_click variant uses confidence < 0.55 as a proxy for
    // "middle finger hidden behind the thumb-index pinch". The proxy
    // is correct for sustained dips (real occlusion drops confidence
    // to 0.10-0.30 and HOLDS it there — clip 4cd4b2: confidence
    // 0.158-0.192 throughout the held pose) but wrong for borderline
    // tracker noise (clip b21dab: confidence wobbles 0.51-0.59 during
    // a real pinch_select where the middle is clearly visible at
    // tm = 65-75 mm). The windowed-min discriminates: a real occlusion
    // has min ≤ 0.30 over the prior 200 ms; noise sits at min ≈ 0.51.
    float middle_tip_confidence_min_over_200ms;

    // Absolute rate of change of thumb_on_index_projection over a
    // trailing window (~200ms).  High when the thumb is sliding along
    // the index finger (scroll gesture); near zero when the thumb is
    // stationary at the tip (pinch gesture).  Units: 1/s (projection
    // is dimensionless 0..1, so velocity is per-second).  Computed by
    // the engine's temporal tracker from a per-hand ring buffer.
    float thumb_on_index_projection_velocity;
    float thumb_on_index_projection_range_500ms;

    // Wrist speed (m/s), EMA over ~100 ms — populated by the engine's
    // temporal tracker (0 in bare compute_features output).  A pinch pose
    // that appears while the whole hand is sweeping fast is usually
    // tracking noise; pinch/scroll triggers gate on this staying low.
    float wrist_speed_mps;

    // Time (ms) since fist_launcher last emitted END or CANCEL.
    // Right after a fist opens, the user's hand is mid-extension
    // — fingers haven't fully unfurled and the iPhone Vision
    // tracker briefly reports low middle_tip_confidence, which
    // matches the occluded-middle right_click variant.  In-
    // person report 2026-05-23: "right after fist_launcher I
    // tried to select an icon (pinch click) but it registered
    // a right_click."  Right_click variants gate on this > 300 ms
    // so the hand has time to fully re-extend before a 3-finger
    // pinch is recognised.
    float fist_launcher_end_recency_ms;
};

// Compute all features from raw joint data.  Fingertip joints with
// confidence below min_tip_confidence are treated like zero-frame
// artifacts: features derived from them come back indeterminate (NaN).
HandFeatures compute_features(const ge_hand_t &hand,
                              float min_tip_confidence = 0.0f);

// Robust per-hand metric scale (m): mean of the index/middle/ring proximal
// phalanx. Used to normalize the hand before shape-feature extraction.
float compute_hand_scale(const ge_hand_t &hand);

// Signed distance of the thumb from the palm plane, in units of the hand's
// own proximal-phalanx scale.  Backs ge_hand_chirality().
//
// The palm plane is spanned by (INDEX_MCP - WRIST) and (PINKY_MCP - WRIST);
// in a right-handed world frame their cross product points PALMAR for a
// right hand and DORSAL for a left one (that is what makes it a chirality
// test, and it is why palm_normal reads palm-facing on right-hand clips).
// The thumb opposes the fingers, so anatomically it is always on the palmar
// side.  Hence: positive = right hand, negative = left hand, and the
// magnitude is how far off-plane the thumb is (near zero = undecidable).
float palm_thumb_signed_offset(const ge_hand_t &hand);

// Compute curl for a single finger given its 4 joint indices (MCP, PIP, DIP, TIP).
// Returns 0..1 where 0 = straight, 1 = fully curled.
float compute_finger_curl(const ge_hand_t &hand, int mcp, int pip, int dip, int tip);

// Project thumb tip onto the polyline MCP->PIP->DIP->TIP of the index finger.
// Returns 0..1 where 0 = at MCP (knuckle), 1 = at TIP (fingertip).
float project_thumb_on_index(const ge_hand_t &hand);

// Project thumb tip onto the polyline MCP->PIP->DIP->TIP of the middle finger.
// Returns 0..1 where 0 = at MCP (knuckle), 1 = at TIP (fingertip).
float project_thumb_on_middle(const ge_hand_t &hand);

// Feature lookup by name (for config-driven gesture definitions)
float get_feature_by_name(const HandFeatures &f, const std::string &name);

}  // namespace ge
