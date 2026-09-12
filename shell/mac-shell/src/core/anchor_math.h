// anchor_math.h — port of the anchor pose math in vendor/wxrd/src/anchors.c
// to pure C++ (no graphene, no wlroots, no theme runtime).
//
// The wxrd source of truth is anchors.c; every function here mirrors its
// counterpart operation-for-operation so the two implementations produce the
// same numbers. Theme constants are the compiled defaults from
// vendor/wxrd/src/theme_runtime.c:24-37.

#pragma once

#include <cstdint>
#include <string>

#include "spatial_bridge.h"

namespace mac_shell {

// Compiled defaults mirroring theme_runtime.c.
struct anchor_config {
    float hover_distance_m = 0.02f;
    float snap_magnetism_m = 0.08f;
    float snap_tilt_near_m = 0.40f;
    float snap_tilt_far_m = 1.00f;
    float snap_tilt_near_deg = 80.0f;
    float snap_tilt_mid_deg = 38.0f;
    float snap_tilt_far_deg = 0.0f;
};

// World-frame context the anchor math needs: the ARKit world origin captured
// on the first valid pose (anchors.c g_world_origin_*) and the head position
// in the origin-subtracted scene frame (anchors.c g_head_pos).
struct anchor_env {
    anchor_config config;

    float world_origin_pos[3] = {0.0f, 0.0f, 0.0f};
    float world_origin_rot_inv[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    // anchors.c guards on BOTH pos and rot being captured; scene captures them
    // together so one flag suffices.
    bool have_world_origin = false;

    float head_pos[3] = {0.0f, 0.0f, 0.0f};
};

// Bring a raw-frame plane centre/normal into the scene frame (anchors.c
// plane_to_scene_frame).
void plane_to_scene_frame(const anchor_env &env, const sb_plane_t &plane,
                          float out_center[3], float out_normal[3]);

// Scene-frame centre + normal + right/forward in-plane basis (anchors.c
// plane_to_scene_basis).
void plane_to_scene_basis(const anchor_env &env, const sb_plane_t &plane,
                          float out_center[3], float out_normal[3],
                          float out_right[3], float out_fwd[3]);

// Ergonomic tilt for horizontal-surface snaps (anchors.c
// horizontal_tilt_deg): degrees from flat as a function of head height above
// the surface, three regimes with 0.06 m smooth shoulders.
float horizontal_tilt_deg(const anchor_env &env, float head_above_m);

// Panel placement matrix for a plane (anchors.c plane_to_matrix). out_m is
// row-major with row-vector convention: row 0 = right, row 1 = up, row 2 =
// front (faces the head), row 3 = translation — identical storage to the
// float[16] handed to graphene_matrix_init_from_float in wxrd.
void plane_to_matrix(const anchor_env &env, const sb_plane_t &plane,
                     const float offset[3], float out_m[16]);

// Plane search shared by try-snap and snap-kind (anchors.c
// anchors_snap_core). vp is the panel centre in the scene frame;
// alignment_filter < 0 means any plane. On success returns the index of the
// snapped plane in planes[] and writes the preserved in-plane offset;
// returns -1 when no plane is within snap_magnetism_m (or in extent).
int anchors_snap_core(const anchor_env &env, const float vp[3],
                      const sb_plane_t *planes, int n_planes,
                      int alignment_filter, float out_offset[3]);

// Parse a 32-hex-digit UUID (no dashes) — anchors.h wxrd_parse_uuid_hex.
bool parse_uuid_hex(const char *hex, uint8_t out[16]);

// The inverse: 32 lowercase hex digits, the spelling every reply and layout
// file uses.
std::string uuid_to_hex(const uint8_t uuid[16]);

}  // namespace mac_shell
