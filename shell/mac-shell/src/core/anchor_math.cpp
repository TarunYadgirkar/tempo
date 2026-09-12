// anchor_math.cpp — see anchor_math.h. Port of vendor/wxrd/src/anchors.c.

#include "core/anchor_math.h"

#include <cmath>
#include <cstring>

#include "core/vec_math.h"

namespace mac_shell {

namespace {

constexpr float DEG2RAD = 0.01745329252f;
// anchors.c SNAP_TILT_SHOULDER_M.
constexpr float SNAP_TILT_SHOULDER_M = 0.06f;

}  // namespace

void plane_to_scene_frame(const anchor_env &env, const sb_plane_t &plane,
                          float out_center[3], float out_normal[3]) {
    v3copy(plane.center, out_center);
    v3copy(plane.normal, out_normal);

    if (env.have_world_origin) {
        out_center[0] -= env.world_origin_pos[0];
        out_center[1] -= env.world_origin_pos[1];
        out_center[2] -= env.world_origin_pos[2];
        quat_rotate_vec(env.world_origin_rot_inv, out_center, out_center);
        quat_rotate_vec(env.world_origin_rot_inv, out_normal, out_normal);
    }
}

void plane_to_scene_basis(const anchor_env &env, const sb_plane_t &plane,
                          float out_center[3], float out_normal[3],
                          float out_right[3], float out_fwd[3]) {
    plane_to_scene_frame(env, plane, out_center, out_normal);

    const float world_up[3] = {0.0f, 1.0f, 0.0f};
    const float world_fwd[3] = {0.0f, 0.0f, -1.0f};

    // right = cross(n, world_up); fall back to world_fwd if nearly parallel.
    v3cross(out_normal, world_up, out_right);
    if (v3length(out_right) < 1e-4f)
        v3cross(out_normal, world_fwd, out_right);
    v3normalize(out_right);

    // fwd = cross(right, n) — completes the right-handed basis.
    v3cross(out_right, out_normal, out_fwd);
    v3normalize(out_fwd);
}

float horizontal_tilt_deg(const anchor_env &env, float head_above_m) {
    const anchor_config &t = env.config;
    float h = head_above_m < 0.0f ? 0.0f : head_above_m;
    float near_m = t.snap_tilt_near_m;
    float far_m = t.snap_tilt_far_m;
    float nd = t.snap_tilt_near_deg;
    float md = t.snap_tilt_mid_deg;
    float fd = t.snap_tilt_far_deg;
    float e = SNAP_TILT_SHOULDER_M;
    if (far_m < near_m + 2.0f * e)  // degenerate config → keep a valid band
        far_m = near_m + 2.0f * e;

    if (h <= near_m) return nd;
    if (h < near_m + e) return nd + (md - nd) * ((h - near_m) / e);
    if (h <= far_m - e) return md;
    if (h < far_m) return md + (fd - md) * ((h - (far_m - e)) / e);
    return fd;
}

void plane_to_matrix(const anchor_env &env, const sb_plane_t &plane,
                     const float offset[3], float out_m[16]) {
    float c[3], n[3];
    plane_to_scene_frame(env, plane, c, n);
    v3normalize(n);

    // Shift the placement centre by the panel's in-plane offset.
    c[0] += offset[0];
    c[1] += offset[1];
    c[2] += offset[2];

    // Flip the normal to point toward the head so the window faces the user.
    float to_head[3] = {env.head_pos[0] - c[0], env.head_pos[1] - c[1],
                        env.head_pos[2] - c[2]};
    float n_face[3] = {n[0], n[1], n[2]};
    if (v3dot(to_head, n_face) < 0.0f) {
        n_face[0] = -n_face[0];
        n_face[1] = -n_face[1];
        n_face[2] = -n_face[2];
    }

    const float WUP[3] = {0.0f, 1.0f, 0.0f};
    float right[3], up[3], front[3];

    if (plane.alignment == 1) {
        // VERTICAL (wall): flat on the wall, facing the room.
        v3copy(n_face, front);
        float d = v3dot(WUP, front);
        up[0] = WUP[0] - d * front[0];
        up[1] = WUP[1] - d * front[1];
        up[2] = WUP[2] - d * front[2];
        if (v3normalize(up) < 1e-4f) {
            up[0] = 0.0f;
            up[1] = 1.0f;
            up[2] = 0.0f;
        }
        v3cross(up, front, right);
        v3normalize(right);
    } else {
        // HORIZONTAL (desk/floor): tilt toward the head about the in-plane
        // side axis by θ.
        float fh[3] = {env.head_pos[0] - c[0], 0.0f, env.head_pos[2] - c[2]};
        if (v3normalize(fh) < 1e-4f) {
            fh[0] = 0.0f;
            fh[1] = 0.0f;
            fh[2] = -1.0f;
        }
        float side[3];
        v3cross(n_face, fh, side);
        if (v3normalize(side) < 1e-4f) {
            side[0] = 1.0f;
            side[1] = 0.0f;
            side[2] = 0.0f;
        }
        float th = horizontal_tilt_deg(env, env.head_pos[1] - c[1]) * DEG2RAD;
        float ct = std::cos(th), st = std::sin(th);
        for (int i = 0; i < 3; i++) {
            front[i] = n_face[i] * ct + fh[i] * st;
            up[i] = -fh[i] * ct + n_face[i] * st;
            right[i] = side[i];
        }
        v3normalize(front);
        v3normalize(up);
    }

    // Translation: lift off the surface toward the head by the hover offset.
    const float hover = env.config.hover_distance_m;
    float tx = c[0] + n_face[0] * hover;
    float ty = c[1] + n_face[1] * hover;
    float tz = c[2] + n_face[2] * hover;

    float m[16] = {
        right[0], right[1], right[2], 0.0f,  // row 0 — right
        up[0],    up[1],    up[2],    0.0f,  // row 1 — up
        front[0], front[1], front[2], 0.0f,  // row 2 — front (faces head)
        tx,       ty,       tz,       1.0f,  // row 3 — translation
    };
    std::memcpy(out_m, m, sizeof(m));
}

int anchors_snap_core(const anchor_env &env, const float vp[3],
                      const sb_plane_t *planes, int n_planes,
                      int alignment_filter, float out_offset[3]) {
    if (!planes || n_planes <= 0)
        return -1;

    const float snap_thresh = env.config.snap_magnetism_m;

    int best = -1;
    float best_abs_d = snap_thresh;
    for (int i = 0; i < n_planes; i++) {
        const sb_plane_t &pl = planes[i];
        if (pl.is_removed)
            continue;
        if (alignment_filter >= 0 && pl.alignment != alignment_filter)
            continue;

        float c[3], n[3], right[3], fwd[3];
        plane_to_scene_basis(env, pl, c, n, right, fwd);

        // Signed perpendicular distance from view centre to plane.
        float dx = vp[0] - c[0];
        float dy = vp[1] - c[1];
        float dz = vp[2] - c[2];
        float d = dx * n[0] + dy * n[1] + dz * n[2];
        float ad = std::fabs(d);
        if (ad >= best_abs_d)
            continue;

        // In-extent test: project (vp - c) onto the plane basis.
        float proj_x = dx - d * n[0];
        float proj_y = dy - d * n[1];
        float proj_z = dz - d * n[2];
        float lx = proj_x * right[0] + proj_y * right[1] + proj_z * right[2];
        float lz = proj_x * fwd[0] + proj_y * fwd[1] + proj_z * fwd[2];

        float hx = pl.extent[0] * 0.5f;
        float hz = pl.extent[1] * 0.5f;
        if (std::fabs(lx) > hx || std::fabs(lz) > hz)
            continue;

        best = i;
        best_abs_d = ad;
    }

    if (best < 0)
        return -1;

    // Preserve where the panel sits on the plane: in-plane component of
    // (view centre − plane centre).
    float bc[3], bn[3];
    plane_to_scene_frame(env, planes[best], bc, bn);
    v3normalize(bn);
    float ov[3] = {vp[0] - bc[0], vp[1] - bc[1], vp[2] - bc[2]};
    float od = v3dot(ov, bn);
    out_offset[0] = ov[0] - od * bn[0];
    out_offset[1] = ov[1] - od * bn[1];
    out_offset[2] = ov[2] - od * bn[2];
    return best;
}

bool parse_uuid_hex(const char *hex, uint8_t out[16]) {
    if (!hex || std::strlen(hex) != 32)
        return false;
    auto nibble = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    for (int i = 0; i < 16; i++) {
        int hi = nibble(hex[i * 2]);
        int lo = nibble(hex[i * 2 + 1]);
        if (hi < 0 || lo < 0)
            return false;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return true;
}

std::string uuid_to_hex(const uint8_t uuid[16]) {
    static const char hex[] = "0123456789abcdef";
    std::string s(32, '0');
    for (int i = 0; i < 16; i++) {
        s[i * 2] = hex[(uuid[i] >> 4) & 0xf];
        s[i * 2 + 1] = hex[uuid[i] & 0xf];
    }
    return s;
}

}  // namespace mac_shell
