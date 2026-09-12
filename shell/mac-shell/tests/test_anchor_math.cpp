// test_anchor_math.cpp — unit tests for the anchors.c math port.
//
// Expected values are hand-derived from the same formulas wxrd uses
// (vendor/wxrd/src/anchors.c with the theme_runtime.c defaults), so a change
// that diverges from the wxrd behaviour fails here.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "core/anchor_math.h"
#include "core/vec_math.h"

using namespace mac_shell;

static int g_failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, \
                         #cond);                                          \
            g_failures++;                                                 \
        }                                                                 \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                 \
    do {                                                                      \
        float va = (a), vb = (b);                                             \
        if (std::fabs(va - vb) > (eps)) {                                     \
            std::fprintf(stderr, "FAIL %s:%d: %s=%g != %s=%g\n", __FILE__,    \
                         __LINE__, #a, (double)va, #b, (double)vb);           \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

static sb_plane_t make_plane(uint8_t id, const float center[3],
                             const float normal[3], float w, float h,
                             uint8_t alignment) {
    sb_plane_t p{};
    std::memset(p.uuid, id, 16);
    std::memcpy(p.center, center, sizeof(p.center));
    std::memcpy(p.normal, normal, sizeof(p.normal));
    p.extent[0] = w;
    p.extent[1] = h;
    p.alignment = alignment;
    p.is_removed = 0;
    return p;
}

static void test_theme_defaults() {
    anchor_config c;
    CHECK_NEAR(c.hover_distance_m, 0.02f, 1e-9f);
    CHECK_NEAR(c.snap_magnetism_m, 0.08f, 1e-9f);
    CHECK_NEAR(c.snap_tilt_near_m, 0.40f, 1e-9f);
    CHECK_NEAR(c.snap_tilt_far_m, 1.00f, 1e-9f);
    CHECK_NEAR(c.snap_tilt_near_deg, 80.0f, 1e-9f);
    CHECK_NEAR(c.snap_tilt_mid_deg, 38.0f, 1e-9f);
    CHECK_NEAR(c.snap_tilt_far_deg, 0.0f, 1e-9f);
}

static void test_tilt_regimes() {
    anchor_env env;
    // Same regime boundaries as anchors.c horizontal_tilt_deg with the
    // 0.06 m shoulders: near ≤ 0.40 → 80°, [0.46, 0.94] → 38°, ≥ 1.0 → 0°.
    CHECK_NEAR(horizontal_tilt_deg(env, -0.5f), 80.0f, 1e-4f);
    CHECK_NEAR(horizontal_tilt_deg(env, 0.2f), 80.0f, 1e-4f);
    CHECK_NEAR(horizontal_tilt_deg(env, 0.40f), 80.0f, 1e-4f);
    CHECK_NEAR(horizontal_tilt_deg(env, 0.43f), 59.0f, 1e-3f);  // mid-shoulder
    CHECK_NEAR(horizontal_tilt_deg(env, 0.46f), 38.0f, 1e-3f);
    CHECK_NEAR(horizontal_tilt_deg(env, 0.70f), 38.0f, 1e-4f);
    CHECK_NEAR(horizontal_tilt_deg(env, 0.94f), 38.0f, 1e-3f);
    CHECK_NEAR(horizontal_tilt_deg(env, 0.97f), 19.0f, 1e-3f);  // far shoulder
    CHECK_NEAR(horizontal_tilt_deg(env, 1.0f), 0.0f, 1e-4f);
    CHECK_NEAR(horizontal_tilt_deg(env, 1.5f), 0.0f, 1e-4f);
}

static void test_vertical_wall_matrix() {
    anchor_env env;  // head at origin, no world origin captured
    const float c[3] = {0.0f, 1.0f, -2.0f};
    const float n[3] = {0.0f, 0.0f, 1.0f};
    sb_plane_t wall = make_plane(0xAA, c, n, 2.0f, 2.0f, 1);

    float offset[3] = {0, 0, 0};
    float m[16];
    plane_to_matrix(env, wall, offset, m);

    // right
    CHECK_NEAR(m[0], 1.0f, 1e-5f);
    CHECK_NEAR(m[1], 0.0f, 1e-5f);
    CHECK_NEAR(m[2], 0.0f, 1e-5f);
    // up
    CHECK_NEAR(m[4], 0.0f, 1e-5f);
    CHECK_NEAR(m[5], 1.0f, 1e-5f);
    CHECK_NEAR(m[6], 0.0f, 1e-5f);
    // front faces the head (+Z)
    CHECK_NEAR(m[8], 0.0f, 1e-5f);
    CHECK_NEAR(m[9], 0.0f, 1e-5f);
    CHECK_NEAR(m[10], 1.0f, 1e-5f);
    // translation lifted 0.02 m off the wall toward the head
    CHECK_NEAR(m[12], 0.0f, 1e-5f);
    CHECK_NEAR(m[13], 1.0f, 1e-5f);
    CHECK_NEAR(m[14], -1.98f, 1e-5f);
    CHECK_NEAR(m[15], 1.0f, 1e-6f);
}

static void test_horizontal_desk_matrix() {
    anchor_env env;  // head at origin
    const float c[3] = {0.0f, -0.8f, -1.0f};
    const float n[3] = {0.0f, 1.0f, 0.0f};
    sb_plane_t desk = make_plane(0xBB, c, n, 2.0f, 1.2f, 0);

    float offset[3] = {0, 0, 0};
    float m[16];
    plane_to_matrix(env, desk, offset, m);

    // head_above = 0.8 → mid regime → 38° tilt about the side axis.
    const float th = 38.0f * 0.01745329252f;
    const float ct = std::cos(th), st = std::sin(th);
    // fh = normalize(head - c, horizontal) = (0, 0, 1)
    // side = cross(n, fh) = (1, 0, 0)
    CHECK_NEAR(m[0], 1.0f, 1e-5f);   // right = side
    CHECK_NEAR(m[1], 0.0f, 1e-5f);
    CHECK_NEAR(m[2], 0.0f, 1e-5f);
    CHECK_NEAR(m[4], 0.0f, 1e-5f);   // up = -fh*ct + n*st
    CHECK_NEAR(m[5], st, 1e-5f);
    CHECK_NEAR(m[6], -ct, 1e-5f);
    CHECK_NEAR(m[8], 0.0f, 1e-5f);   // front = n*ct + fh*st
    CHECK_NEAR(m[9], ct, 1e-5f);
    CHECK_NEAR(m[10], st, 1e-5f);
    // translation: hover along the (unflipped) normal
    CHECK_NEAR(m[12], 0.0f, 1e-5f);
    CHECK_NEAR(m[13], -0.78f, 1e-5f);
    CHECK_NEAR(m[14], -1.0f, 1e-5f);
}

static void test_normal_flip_toward_head() {
    anchor_env env;  // head at origin
    // Desk ABOVE the head: normal +Y points away from the head, so
    // plane_to_matrix must flip it and hover downward.
    const float c[3] = {0.0f, 0.7f, -1.0f};
    const float n[3] = {0.0f, 1.0f, 0.0f};
    sb_plane_t shelf = make_plane(0xCC, c, n, 2.0f, 2.0f, 0);

    float offset[3] = {0, 0, 0};
    float m[16];
    plane_to_matrix(env, shelf, offset, m);
    CHECK_NEAR(m[13], 0.68f, 1e-5f);  // 0.7 − 0.02 (flipped hover)
}

static void test_in_plane_offset_applied() {
    anchor_env env;
    const float c[3] = {0.0f, -0.8f, -1.0f};
    const float n[3] = {0.0f, 1.0f, 0.0f};
    sb_plane_t desk = make_plane(0xBB, c, n, 2.0f, 1.2f, 0);

    float offset[3] = {0.3f, 0.0f, -0.1f};
    float m[16];
    plane_to_matrix(env, desk, offset, m);
    CHECK_NEAR(m[12], 0.3f, 1e-5f);
    CHECK_NEAR(m[13], -0.78f, 1e-5f);
    CHECK_NEAR(m[14], -1.1f, 1e-5f);
}

static void test_world_origin_subtraction() {
    anchor_env env;
    env.have_world_origin = true;
    env.world_origin_pos[0] = 0.0f;
    env.world_origin_pos[1] = 0.0f;
    env.world_origin_pos[2] = 0.0f;
    // Origin rotation: 90° about +Y. inv = conjugate.
    const float s = std::sin(0.25f * (float)M_PI);
    const float w = std::cos(0.25f * (float)M_PI);
    env.world_origin_rot_inv[0] = 0.0f;
    env.world_origin_rot_inv[1] = -s;
    env.world_origin_rot_inv[2] = 0.0f;
    env.world_origin_rot_inv[3] = w;

    const float c[3] = {1.0f, 0.0f, 0.0f};
    const float n[3] = {1.0f, 0.0f, 0.0f};
    sb_plane_t pl = make_plane(0xDD, c, n, 1.0f, 1.0f, 1);

    float out_c[3], out_n[3];
    plane_to_scene_frame(env, pl, out_c, out_n);
    // Rotating (1,0,0) by −90° about Y gives (0,0,1).
    CHECK_NEAR(out_c[0], 0.0f, 1e-5f);
    CHECK_NEAR(out_c[1], 0.0f, 1e-5f);
    CHECK_NEAR(out_c[2], 1.0f, 1e-5f);
    CHECK_NEAR(out_n[2], 1.0f, 1e-5f);

    // Translation-only origin.
    anchor_env env2;
    env2.have_world_origin = true;
    env2.world_origin_pos[0] = 1.0f;
    env2.world_origin_pos[1] = 2.0f;
    env2.world_origin_pos[2] = 3.0f;
    plane_to_scene_frame(env2, pl, out_c, out_n);
    CHECK_NEAR(out_c[0], 0.0f, 1e-5f);
    CHECK_NEAR(out_c[1], -2.0f, 1e-5f);
    CHECK_NEAR(out_c[2], -3.0f, 1e-5f);
    CHECK_NEAR(out_n[0], 1.0f, 1e-5f);  // normal untouched by translation
}

// The SCENE frame stays gravity-aligned: scene::update_head captures only the
// YAW of the session-start pose, so a phone that was pitched when the session
// began does not tilt the whole world (docs/coordinate-systems.md section 9).
static void test_gravity_aligned_origin_rotation() {
    // Session start: yawed 90 deg about +Y and pitched 30 deg about local +X.
    const float yaw = 0.5f * (float)M_PI;
    const float pitch = 30.0f * (float)M_PI / 180.0f;
    const float qy[4] = {0.0f, std::sin(yaw * 0.5f), 0.0f, std::cos(yaw * 0.5f)};
    const float qx[4] = {std::sin(pitch * 0.5f), 0.0f, 0.0f,
                         std::cos(pitch * 0.5f)};
    float start[4];
    quat_mul(qy, qx, start);

    float yaw_only[4];
    quat_yaw_only(start, yaw_only);
    CHECK_NEAR(yaw_only[0], 0.0f, 1e-5f);  // pitch stripped
    CHECK_NEAR(yaw_only[2], 0.0f, 1e-5f);
    CHECK_NEAR(yaw_only[1], qy[1], 1e-4f);
    CHECK_NEAR(yaw_only[3], qy[3], 1e-4f);

    // The env scene builds from that capture: conjugate of the yaw-only quat.
    anchor_env env;
    env.have_world_origin = true;
    env.world_origin_rot_inv[0] = -yaw_only[0];
    env.world_origin_rot_inv[1] = -yaw_only[1];
    env.world_origin_rot_inv[2] = -yaw_only[2];
    env.world_origin_rot_inv[3] = yaw_only[3];

    const float c[3] = {1.0f, -0.8f, 0.0f};
    const float n[3] = {0.0f, 1.0f, 0.0f};  // gravity-up desk
    sb_plane_t desk = make_plane(0xEE, c, n, 1.0f, 1.0f, 0);

    float oc[3], on[3];
    plane_to_scene_frame(env, desk, oc, on);
    CHECK_NEAR(on[0], 0.0f, 1e-5f);
    CHECK_NEAR(on[1], 1.0f, 1e-5f);  // still horizontal in SCENE
    CHECK_NEAR(on[2], 0.0f, 1e-5f);
    CHECK_NEAR(oc[1], -0.8f, 1e-5f);  // height survives a yaw-only rotation

    // The old full-quaternion capture tilted it by the session-start pitch.
    anchor_env tilted = env;
    tilted.world_origin_rot_inv[0] = -start[0];
    tilted.world_origin_rot_inv[1] = -start[1];
    tilted.world_origin_rot_inv[2] = -start[2];
    tilted.world_origin_rot_inv[3] = start[3];
    plane_to_scene_frame(tilted, desk, oc, on);
    CHECK(std::fabs(on[1] - 1.0f) > 0.1f);

    std::printf("PASS test_gravity_aligned_origin_rotation\n");
}

static void test_snap_core() {
    anchor_env env;  // head at origin
    const float desk_c[3] = {0.0f, -0.8f, -1.0f};
    const float up[3] = {0.0f, 1.0f, 0.0f};
    const float wall_c[3] = {0.0f, -0.75f, -2.0f};
    const float fwd[3] = {0.0f, 0.0f, 1.0f};
    sb_plane_t planes[2] = {
        make_plane(0x11, wall_c, fwd, 2.0f, 2.0f, 1),
        make_plane(0x22, desk_c, up, 2.0f, 1.2f, 0),
    };

    float offset[3];

    // Within 0.08 m of the desk: snaps, offset keeps the in-plane drop point.
    const float vp1[3] = {0.3f, -0.75f, -1.1f};
    int idx = anchors_snap_core(env, vp1, planes, 2, /*any*/ -1, offset);
    CHECK(idx == 1);
    CHECK_NEAR(offset[0], 0.3f, 1e-5f);
    CHECK_NEAR(offset[1], 0.0f, 1e-5f);
    CHECK_NEAR(offset[2], -0.1f, 1e-5f);

    // Too far from every plane → no snap.
    const float vp2[3] = {0.0f, -0.5f, -1.0f};
    CHECK(anchors_snap_core(env, vp2, planes, 2, -1, offset) == -1);

    // In range perpendicular but outside the plane extent → no snap.
    const float vp3[3] = {1.5f, -0.78f, -1.0f};
    CHECK(anchors_snap_core(env, vp3, planes, 2, -1, offset) == -1);

    // Near the wall (0.04 m perpendicular, inside its extent). The desk is
    // also within 0.06 m perpendicular here but 0.96 m past its half-extent
    // of 0.6 m, so the horizontal filter finds nothing — same in-extent rule
    // as anchors.c.
    const float vp4[3] = {0.0f, -0.74f, -1.96f};
    CHECK(anchors_snap_core(env, vp4, planes, 2, -1, offset) == 0);  // wall
    CHECK(anchors_snap_core(env, vp4, planes, 2, 1, offset) == 0);   // wall
    CHECK(anchors_snap_core(env, vp4, planes, 2, 0, offset) == -1);  // horiz

    // Near only the desk: horizontal filter snaps, wall filter does not.
    const float vp5[3] = {0.2f, -0.74f, -1.2f};
    CHECK(anchors_snap_core(env, vp5, planes, 2, 0, offset) == 1);
    CHECK(anchors_snap_core(env, vp5, planes, 2, 1, offset) == -1);

    // Removed planes are never candidates.
    planes[1].is_removed = 1;
    CHECK(anchors_snap_core(env, vp1, planes, 2, -1, offset) == -1);
    planes[1].is_removed = 0;
}

static void test_uuid_parse() {
    uint8_t out[16];
    CHECK(parse_uuid_hex("00112233445566778899aabbccddeeff", out));
    CHECK(out[0] == 0x00 && out[1] == 0x11 && out[15] == 0xff);
    CHECK(!parse_uuid_hex("0011", out));
    CHECK(!parse_uuid_hex("zz112233445566778899aabbccddeeff", out));
    CHECK(!parse_uuid_hex(nullptr, out));
}

int main() {
    test_theme_defaults();
    test_tilt_regimes();
    test_vertical_wall_matrix();
    test_horizontal_desk_matrix();
    test_normal_flip_toward_head();
    test_in_plane_offset_applied();
    test_world_origin_subtraction();
    test_gravity_aligned_origin_rotation();
    test_snap_core();
    test_uuid_parse();

    if (g_failures) {
        std::fprintf(stderr, "test_anchor_math: %d FAILURES\n", g_failures);
        return 1;
    }
    std::printf("test_anchor_math: all tests passed\n");
    return 0;
}
