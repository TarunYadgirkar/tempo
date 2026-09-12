// placement_math.h — pure math for panel spawn placement and the
// gather-panels recall arc. Header-only and Metal/AppKit-free so
// tests/test_placement.cpp can pin the numbers against synthetic head poses
// and planes.
//
// Panels spawn in front of the CURRENT head pose: a fixed distance along the
// head's forward direction projected to horizontal, slightly below eye
// height, facing the head, clamped above the floor plane, and pulled toward
// the head if the spot would land inside/behind a detected plane ("goes
// under a chair" fix).

#pragma once

#include <cmath>

#include "core/anchor_math.h"
#include "spatial_bridge.h"
#include "core/vec_math.h"

namespace mac_shell {

struct spawn_tuning {
    float distance_m = 0.9f;       // along the horizontal head forward
    float below_eyes_m = 0.10f;    // spawn height = head height − this
    float floor_clearance_m = 0.20f;  // min height above the lowest floor
    float plane_clearance_m = 0.15f;  // min distance in front of any plane
    float plane_margin_m = 0.30f;     // extent margin for the pull-in test
};

// Horizontal projection of the head's forward (-Z of the head quat, scene
// frame). Falls back to scene -Z when the user looks straight up/down.
inline void head_forward_horizontal(const float head_quat[4], float out[3]) {
    const float fwd_local[3] = {0.0f, 0.0f, -1.0f};
    quat_rotate_vec(head_quat, fwd_local, out);
    out[1] = 0.0f;
    if (v3normalize(out) < 0.2f) {
        out[0] = 0.0f;
        out[1] = 0.0f;
        out[2] = -1.0f;
    }
}

// Yaw for a panel at `pos` to face `head_pos` (front row = (sin, 0, cos)).
// Degenerate (panel at the head) → 0 (front toward scene +Z, the identity
// orientation legacy panels used).
inline float yaw_facing(const float pos[3], const float head_pos[3]) {
    float dx = head_pos[0] - pos[0];
    float dz = head_pos[2] - pos[2];
    if (dx * dx + dz * dz < 1e-8f)
        return 0.0f;
    return std::atan2(dx, dz);
}

// Scene-frame y of the lowest detected horizontal plane (the floor). Returns
// false when no horizontal plane is known.
inline bool lowest_horizontal_plane_y(const anchor_env &env,
                                      const sb_plane_t *planes, int n_planes,
                                      float *out_y) {
    bool found = false;
    float lowest = 0.0f;
    for (int i = 0; i < n_planes; i++) {
        if (planes[i].is_removed || planes[i].alignment != 0)
            continue;
        float c[3], nrm[3];
        plane_to_scene_frame(env, planes[i], c, nrm);
        if (!found || c[1] < lowest) {
            lowest = c[1];
            found = true;
        }
    }
    if (found && out_y)
        *out_y = lowest;
    return found;
}

// If `pos` sits behind (or within clearance of) a detected plane whose
// extent it overlaps, pull it back along the head→pos segment until it is
// `plane_clearance_m` clear. Applied per plane; the head itself is assumed
// to be in free space.
inline void pull_clear_of_planes(const anchor_env &env, const float head_pos[3],
                                 const sb_plane_t *planes, int n_planes,
                                 const spawn_tuning &t, float pos[3]) {
    for (int i = 0; i < n_planes; i++) {
        if (planes[i].is_removed)
            continue;
        float c[3], nrm[3], right[3], fwd[3];
        plane_to_scene_basis(env, planes[i], c, nrm, right, fwd);
        v3normalize(nrm);
        // Orient the normal toward the head.
        float to_head[3];
        v3sub(head_pos, c, to_head);
        float h_side = v3dot(to_head, nrm);
        if (h_side < 0.0f) {
            nrm[0] = -nrm[0];
            nrm[1] = -nrm[1];
            nrm[2] = -nrm[2];
            h_side = -h_side;
        }
        if (h_side <= t.plane_clearance_m)
            continue;  // head itself inside the clearance band — skip
        float d[3];
        v3sub(pos, c, d);
        float p_side = v3dot(d, nrm);
        if (p_side >= t.plane_clearance_m)
            continue;  // already clear
        // Overlap test against the plane extent (with margin).
        float in_plane[3] = {d[0] - p_side * nrm[0], d[1] - p_side * nrm[1],
                             d[2] - p_side * nrm[2]};
        float lx = v3dot(in_plane, right);
        float lz = v3dot(in_plane, fwd);
        float hx = planes[i].extent[0] * 0.5f + t.plane_margin_m;
        float hz = planes[i].extent[1] * 0.5f + t.plane_margin_m;
        if (std::fabs(lx) > hx || std::fabs(lz) > hz)
            continue;  // beside the plane, not behind it
        // Solve pos' = head + s*(pos − head) with side(pos') == clearance.
        float denom = p_side - h_side;
        if (denom >= -1e-6f)
            continue;
        float s = (t.plane_clearance_m - h_side) / denom;
        if (s <= 0.0f || s >= 1.0f)
            continue;
        pos[0] = head_pos[0] + s * (pos[0] - head_pos[0]);
        pos[1] = head_pos[1] + s * (pos[1] - head_pos[1]);
        pos[2] = head_pos[2] + s * (pos[2] - head_pos[2]);
    }
}

// Place a point `distance_m` along `fwd_h` (horizontal unit vector) from the
// head, at head height minus below_eyes_m, floor-clamped and pulled clear of
// planes. Shared by spawn and the gather arc.
inline void place_in_front(const anchor_env &env, const float head_pos[3],
                           const float fwd_h[3], const sb_plane_t *planes,
                           int n_planes, const spawn_tuning &t,
                           float out_pos[3], float *out_yaw) {
    out_pos[0] = head_pos[0] + fwd_h[0] * t.distance_m;
    out_pos[1] = head_pos[1] - t.below_eyes_m;
    out_pos[2] = head_pos[2] + fwd_h[2] * t.distance_m;
    float floor_y;
    if (lowest_horizontal_plane_y(env, planes, n_planes, &floor_y) &&
        out_pos[1] < floor_y + t.floor_clearance_m)
        out_pos[1] = floor_y + t.floor_clearance_m;
    pull_clear_of_planes(env, head_pos, planes, n_planes, t, out_pos);
    if (out_yaw)
        *out_yaw = yaw_facing(out_pos, head_pos);
}

// Spawn placement for a new panel given the current head pose (scene frame).
inline void spawn_pose_in_front(const anchor_env &env, const float head_pos[3],
                                const float head_quat[4],
                                const sb_plane_t *planes, int n_planes,
                                const spawn_tuning &t, float out_pos[3],
                                float *out_yaw) {
    float fwd[3];
    head_forward_horizontal(head_quat, fwd);
    place_in_front(env, head_pos, fwd, planes, n_planes, t, out_pos, out_yaw);
}

// Gather-panels recall arc: index/count → a slot on an arc centred on the
// head's forward, ~28° apart, all facing the head.
constexpr float GATHER_ARC_STEP_RAD = 0.49f;  // ~28° between panels

inline void gather_arc_position(const anchor_env &env, const float head_pos[3],
                                const float head_quat[4], int index, int count,
                                const sb_plane_t *planes, int n_planes,
                                const spawn_tuning &t, float out_pos[3],
                                float *out_yaw) {
    float fwd[3];
    head_forward_horizontal(head_quat, fwd);
    float off = ((float)index - (float)(count - 1) * 0.5f) *
                GATHER_ARC_STEP_RAD;
    float c = std::cos(off), s = std::sin(off);
    // Rotate fwd about +Y by `off` (right-handed: +off swings toward +X when
    // fwd is -Z).
    float dir[3] = {fwd[0] * c + fwd[2] * s, 0.0f,
                    -fwd[0] * s + fwd[2] * c};
    place_in_front(env, head_pos, dir, planes, n_planes, t, out_pos, out_yaw);
}

}  // namespace mac_shell
