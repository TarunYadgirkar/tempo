// test_placement.cpp — panel spawn placement against synthetic head poses
// and planes (src/core/placement_math.h), plus the scene-level spawn-in-front and
// gather-panels behaviours.

#include <cmath>
#include <cstdio>
#include <cstring>

#include "core/placement_math.h"
#include "core/scene.h"

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

#define CHECK_NEAR(a, b, eps)                                              \
    do {                                                                   \
        float va = (a), vb = (b);                                          \
        if (std::fabs(va - vb) > (eps)) {                                  \
            std::fprintf(stderr, "FAIL %s:%d: %s=%g != %s=%g\n", __FILE__, \
                         __LINE__, #a, (double)va, #b, (double)vb);        \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

namespace {

const float IDENTITY_QUAT[4] = {0, 0, 0, 1};

sb_plane_t make_plane(uint8_t id, float cx, float cy, float cz, float nx,
                      float ny, float nz, float w, float h,
                      uint8_t alignment) {
    sb_plane_t p{};
    std::memset(p.uuid, id, 16);
    p.center[0] = cx;
    p.center[1] = cy;
    p.center[2] = cz;
    p.normal[0] = nx;
    p.normal[1] = ny;
    p.normal[2] = nz;
    p.extent[0] = w;
    p.extent[1] = h;
    p.alignment = alignment;
    return p;
}

// Quaternion for a rotation of `rad` about +Y (xyzw).
void yaw_quat(float rad, float out[4]) {
    out[0] = 0;
    out[1] = std::sin(rad * 0.5f);
    out[2] = 0;
    out[3] = std::cos(rad * 0.5f);
}

void test_spawn_in_front_identity() {
    anchor_env env{};
    const float head[3] = {0, 0, 0};
    spawn_tuning t;
    float pos[3], yaw;
    spawn_pose_in_front(env, head, IDENTITY_QUAT, nullptr, 0, t, pos, &yaw);
    CHECK_NEAR(pos[0], 0.0f, 1e-5f);
    CHECK_NEAR(pos[1], -0.10f, 1e-5f);  // head height minus 10 cm
    CHECK_NEAR(pos[2], -0.9f, 1e-5f);   // 0.9 m along -Z
    CHECK_NEAR(yaw, 0.0f, 1e-5f);       // front (0,0,1) → toward the head
    std::printf("PASS test_spawn_in_front_identity\n");
}

void test_spawn_follows_head_yaw() {
    anchor_env env{};
    const float head[3] = {1.0f, 1.6f, -2.0f};
    float q[4];
    yaw_quat(-1.5707963f, q);  // looking along +X... verify via forward
    float fwd[3];
    head_forward_horizontal(q, fwd);
    spawn_tuning t;
    float pos[3], yaw;
    spawn_pose_in_front(env, head, q, nullptr, 0, t, pos, &yaw);
    // 0.9 m along the horizontal forward, whatever that is.
    CHECK_NEAR(pos[0], head[0] + 0.9f * fwd[0], 1e-4f);
    CHECK_NEAR(pos[2], head[2] + 0.9f * fwd[2], 1e-4f);
    CHECK_NEAR(pos[1], head[1] - 0.10f, 1e-5f);
    // Faces the head: front = (sin yaw, 0, cos yaw) points back at it.
    float front[3] = {std::sin(yaw), 0, std::cos(yaw)};
    float to_head[3] = {head[0] - pos[0], 0, head[2] - pos[2]};
    v3normalize(to_head);
    CHECK_NEAR(v3dot(front, to_head), 1.0f, 1e-4f);
    std::printf("PASS test_spawn_follows_head_yaw\n");
}

void test_spawn_pitched_head_stays_horizontal() {
    // Looking steeply down must NOT put the panel under the desk: forward is
    // projected to horizontal before placement.
    anchor_env env{};
    const float head[3] = {0, 1.5f, 0};
    // Pitch -70° about +X: forward (0,0,-1) → mostly -Y, some -Z.
    float a = -70.0f * 0.017453293f;
    float q[4] = {std::sin(a * 0.5f), 0, 0, std::cos(a * 0.5f)};
    spawn_tuning t;
    float pos[3], yaw;
    spawn_pose_in_front(env, head, q, nullptr, 0, t, pos, &yaw);
    CHECK_NEAR(pos[1], head[1] - 0.10f, 1e-4f);  // height unaffected by pitch
    float horiz = std::sqrt(pos[0] * pos[0] + pos[2] * pos[2]);
    CHECK_NEAR(horiz, 0.9f, 1e-3f);
    std::printf("PASS test_spawn_pitched_head_stays_horizontal\n");
}

void test_floor_clamp() {
    anchor_env env{};
    const float head[3] = {0, 0.2f, 0};  // crouching: spawn would be at 0.1
    sb_plane_t floor = make_plane(0x01, 0, 0.0f, -1.0f, 0, 1, 0, 4, 4, 0);
    spawn_tuning t;
    float pos[3], yaw;
    spawn_pose_in_front(env, head, IDENTITY_QUAT, &floor, 1, t, pos, &yaw);
    CHECK_NEAR(pos[1], 0.20f, 1e-5f);  // clamped to floor + 20 cm

    // The LOWEST horizontal plane is the floor: a desk above doesn't clamp.
    sb_plane_t planes[2] = {
        make_plane(0x02, 0, 0.7f, -1.0f, 0, 1, 0, 1, 1, 0),   // desk
        make_plane(0x03, 0, -1.4f, -1.0f, 0, 1, 0, 6, 6, 0),  // floor
    };
    const float head2[3] = {0, 0.0f, 0};
    spawn_pose_in_front(env, head2, IDENTITY_QUAT, planes, 2, t, pos, &yaw);
    CHECK_NEAR(pos[1], -0.10f, 1e-5f);  // -0.1 > (-1.4 + 0.2): no clamp
    std::printf("PASS test_floor_clamp\n");
}

void test_wall_pull_in() {
    anchor_env env{};
    const float head[3] = {0, 0, 0};
    // Wall 0.5 m ahead, facing the head: the 0.9 m spawn would land behind
    // it; it must be pulled toward the head until clear.
    sb_plane_t wall =
        make_plane(0x04, 0, 0, -0.5f, 0, 0, 1, 3.0f, 2.5f, 1);
    spawn_tuning t;
    float pos[3], yaw;
    spawn_pose_in_front(env, head, IDENTITY_QUAT, &wall, 1, t, pos, &yaw);
    CHECK(pos[2] > -0.5f + t.plane_clearance_m - 1e-4f);
    CHECK(pos[2] < -0.2f);  // still meaningfully in front of the user
    // A wall far to the side must not disturb the spawn.
    sb_plane_t side_wall =
        make_plane(0x05, 5.0f, 0, -0.5f, 0, 0, 1, 1.0f, 1.0f, 1);
    spawn_pose_in_front(env, head, IDENTITY_QUAT, &side_wall, 1, t, pos,
                        &yaw);
    CHECK_NEAR(pos[2], -0.9f, 1e-5f);
    std::printf("PASS test_wall_pull_in\n");
}

void test_gather_arc() {
    anchor_env env{};
    const float head[3] = {0, 1.0f, 0};
    spawn_tuning t;
    const int N = 3;
    float pos[N][3], yaw[N];
    for (int i = 0; i < N; i++)
        gather_arc_position(env, head, IDENTITY_QUAT, i, N, nullptr, 0, t,
                            pos[i], &yaw[i]);
    // Middle slot straight ahead; outer slots symmetric about it.
    CHECK_NEAR(pos[1][0], 0.0f, 1e-4f);
    CHECK_NEAR(pos[1][2], -0.9f, 1e-4f);
    CHECK_NEAR(pos[0][0], -pos[2][0], 1e-4f);
    CHECK_NEAR(pos[0][2], pos[2][2], 1e-4f);
    CHECK(std::fabs(pos[0][0]) > 0.2f);  // actually spread out
    for (int i = 0; i < N; i++) {
        // All at 0.9 m horizontal distance, below eye height, facing head.
        float dx = pos[i][0] - head[0], dz = pos[i][2] - head[2];
        CHECK_NEAR(std::sqrt(dx * dx + dz * dz), 0.9f, 1e-3f);
        CHECK_NEAR(pos[i][1], 0.9f, 1e-4f);
        float front[3] = {std::sin(yaw[i]), 0, std::cos(yaw[i])};
        float to_head[3] = {head[0] - pos[i][0], 0, head[2] - pos[i][2]};
        v3normalize(to_head);
        CHECK_NEAR(v3dot(front, to_head), 1.0f, 1e-4f);
    }
    std::printf("PASS test_gather_arc\n");
}

// ---------------------------------------------------------------------------
// scene level: spawn happens in front of the CURRENT head, and
// gather-panels recalls strays.
// ---------------------------------------------------------------------------

sb_pose_t pose_at(float x, float y, float z) {
    sb_pose_t p{};
    p.timestamp_ns = 1;
    p.pos[0] = x;
    p.pos[1] = y;
    p.pos[2] = z;
    p.rot[3] = 1.0f;
    p.tracking_quality = 1.0f;
    return p;
}

void test_scene_spawn_in_front_of_current_head() {
    scene s;
    s.inject_pose(pose_at(0, 0, 0));  // origin capture
    // The user walks 2 m to the left; a panel launched NOW must appear in
    // front of them there — not at the world-origin spawn shelf.
    s.inject_pose(pose_at(-2.0f, 0, 0));
    s.tick(1.0f / 60.0f);
    uint64_t h = s.spawn_panel("test-card", "here");
    float pos[3];
    CHECK(s.panel_pose(h, pos));
    CHECK_NEAR(pos[0], -2.0f, 1e-3f);
    CHECK_NEAR(pos[1], -0.10f, 1e-3f);
    CHECK_NEAR(pos[2], -0.9f, 1e-3f);
    std::printf("PASS test_scene_spawn_in_front_of_current_head\n");
}

void test_scene_gather_panels() {
    scene s;
    s.inject_pose(pose_at(0, 0, 0));
    s.tick(1.0f / 60.0f);
    uint64_t a = s.spawn_panel("test-card", "a");
    uint64_t b = s.spawn_panel("test-card", "b");
    // Scatter them: one far behind, one far off to the side.
    float lost1[3] = {0.0f, -0.5f, 4.0f};
    float lost2[3] = {-5.0f, 0.0f, 0.5f};
    CHECK(s.move_panel(a, lost1, false));
    CHECK(s.move_panel(b, lost2, false));

    CHECK(s.gather_panels() == 2);
    for (int i = 0; i < 120; i++)  // 2 s of spring settling
        s.tick(1.0f / 60.0f);

    float pa[3], pb[3];
    CHECK(s.panel_pose(a, pa));
    CHECK(s.panel_pose(b, pb));
    for (const float *p : {pa, pb}) {
        CHECK(p[2] < -0.4f);  // in front of the head again
        float d = std::sqrt(p[0] * p[0] + p[2] * p[2]);
        CHECK(d > 0.5f && d < 1.3f);
        CHECK_NEAR(p[1], -0.10f, 1e-2f);
    }
    // Anchored panels stay put.
    uint64_t c = s.spawn_panel("test-card", "c");
    sb_plane_t desk = make_plane(0x11, 0, -0.8f, -1.0f, 0, 1, 0, 2, 1.2f, 0);
    s.inject_planes(&desk, 1);
    float drop[3] = {0.3f, -0.75f, -1.1f};
    CHECK(s.move_panel(c, drop, false));
    uint8_t uuid[16];
    CHECK(s.anchor_panel(c, 2, nullptr, uuid) == 1);
    s.tick(1.0f / 60.0f);
    float before[3];
    CHECK(s.panel_pose(c, before));
    CHECK(s.gather_panels() == 2);  // a + b only; c is anchored
    for (int i = 0; i < 30; i++)
        s.tick(1.0f / 60.0f);
    float after[3];
    CHECK(s.panel_pose(c, after));
    CHECK_NEAR(after[0], before[0], 1e-4f);
    CHECK_NEAR(after[1], before[1], 1e-4f);
    CHECK_NEAR(after[2], before[2], 1e-4f);
    std::printf("PASS test_scene_gather_panels\n");
}

void test_double_pinch_empty_space_gathers() {
    scene s;
    s.inject_pose(pose_at(0, 0, 0));
    s.tick(1.0f / 60.0f);
    uint64_t a = s.spawn_panel("test-card", "stray");
    float lost[3] = {0.0f, -0.5f, 5.0f};
    CHECK(s.move_panel(a, lost, false));

    ge_event_t pinch{};
    pinch.type = GE_EVENT_BEGIN;
    pinch.action = GE_ACTION_POINTER_CLICK;
    pinch.gesture_name = "pinch_select";
    pinch.position[0] = 0.0f;
    pinch.position[1] = 0.0f;
    pinch.position[2] = -0.3f;  // empty space: no panel near the pinch/ray

    // A single empty-space pinch does nothing.
    s.inject_gesture(pinch);
    for (int i = 0; i < 6; i++)
        s.tick(1.0f / 60.0f);
    float pos[3];
    CHECK(s.panel_pose(a, pos));
    CHECK_NEAR(pos[2], 5.0f, 1e-3f);

    // A second one within the double-pinch window recalls the stray.
    s.inject_gesture(pinch);
    for (int i = 0; i < 120; i++)
        s.tick(1.0f / 60.0f);
    CHECK(s.panel_pose(a, pos));
    CHECK(pos[2] < -0.4f);

    // Two pinches far apart in time do NOT gather.
    CHECK(s.move_panel(a, lost, false));
    s.inject_gesture(pinch);
    for (int i = 0; i < 60; i++)  // 1 s > the 0.6 s window
        s.tick(1.0f / 60.0f);
    s.inject_gesture(pinch);
    for (int i = 0; i < 30; i++)
        s.tick(1.0f / 60.0f);
    CHECK(s.panel_pose(a, pos));
    CHECK_NEAR(pos[2], 5.0f, 1e-3f);
    std::printf("PASS test_double_pinch_empty_space_gathers\n");
}

}  // namespace

int main() {
    test_spawn_in_front_identity();
    test_spawn_follows_head_yaw();
    test_spawn_pitched_head_stays_horizontal();
    test_floor_clamp();
    test_wall_pull_in();
    test_gather_arc();
    test_scene_spawn_in_front_of_current_head();
    test_scene_gather_panels();
    test_double_pinch_empty_space_gathers();

    if (g_failures) {
        std::fprintf(stderr, "test_placement: %d FAILURES\n", g_failures);
        return 1;
    }
    std::printf("test_placement: all tests passed\n");
    return 0;
}
