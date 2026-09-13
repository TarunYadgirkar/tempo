// test_hand_inject.cpp — the two pieces of mac-side hand tracking that are
// pure math, and the two places a silent mistake in them would be invisible
// until a gesture stopped working:
//
//   * the MediaPipe <-> SB_JOINT_* landmark mapping, and
//   * unprojecting a depth pixel into camera space and on into the scene.
//
// Plus the parse + freshness contract of `hands-inject`, which decides
// whether the phone's hands or the Mac's are in charge.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

#include "core/hand_inject.h"
#include "core/scene.h"
#include "core/vec_math.h"

static int g_failures = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, \
                         #cond);                                         \
            g_failures++;                                                \
        }                                                                \
    } while (0)

static bool near(float a, float b, float eps = 1e-4f) {
    return std::fabs(a - b) <= eps;
}

using namespace mac_shell;

// ---------------------------------------------------------------------------
// joint order
// ---------------------------------------------------------------------------

static void test_joint_order() {
    // Total, and a bijection: every landmark lands on exactly one SB joint.
    bool seen[SB_HAND_JOINT_COUNT] = {};
    for (int mp = 0; mp < SB_HAND_JOINT_COUNT; mp++) {
        int sb = mediapipe_to_sb_joint(mp);
        CHECK(sb >= 0 && sb < SB_HAND_JOINT_COUNT);
        CHECK(!seen[sb]);
        seen[sb] = true;
        CHECK(sb_to_mediapipe_joint(sb) == mp);
    }
    for (int sb = 0; sb < SB_HAND_JOINT_COUNT; sb++)
        CHECK(seen[sb]);

    CHECK(mediapipe_to_sb_joint(-1) == -1);
    CHECK(mediapipe_to_sb_joint(SB_HAND_JOINT_COUNT) == -1);
    CHECK(sb_to_mediapipe_joint(99) == -1);

    // The anchors the gesture engine actually reads. If MediaPipe ever
    // renumbers, these are the lines that must change with it.
    CHECK(mediapipe_to_sb_joint(0) == SB_JOINT_WRIST);
    CHECK(mediapipe_to_sb_joint(4) == SB_JOINT_THUMB_TIP);
    CHECK(mediapipe_to_sb_joint(5) == SB_JOINT_INDEX_MCP);
    CHECK(mediapipe_to_sb_joint(8) == SB_JOINT_INDEX_TIP);
    CHECK(mediapipe_to_sb_joint(12) == SB_JOINT_MIDDLE_TIP);
    CHECK(mediapipe_to_sb_joint(16) == SB_JOINT_RING_TIP);
    CHECK(mediapipe_to_sb_joint(17) == SB_JOINT_PINKY_MCP);
    CHECK(mediapipe_to_sb_joint(20) == SB_JOINT_PINKY_TIP);
}

// ---------------------------------------------------------------------------
// depth unprojection
// ---------------------------------------------------------------------------

static sb_intrinsics_t make_intr() {
    sb_intrinsics_t in{};
    in.fx = 1000.0f;
    in.fy = 1000.0f;
    in.cx = 960.0f;
    in.cy = 540.0f;
    in.image_width = 1920.0f;
    in.image_height = 1080.0f;
    return in;
}

static void test_unproject() {
    const sb_intrinsics_t intr = make_intr();
    float p[3];

    // Principal point at 2 m: straight down the optical axis, which is -Z.
    CHECK(unproject_pixel(intr, 1920, 1080, 960.0f, 540.0f, 2.0f, p));
    CHECK(near(p[0], 0.0f));
    CHECK(near(p[1], 0.0f));
    CHECK(near(p[2], -2.0f));

    // One focal length right of centre at 1 m is 1 m right.
    CHECK(unproject_pixel(intr, 1920, 1080, 1960.0f, 540.0f, 1.0f, p));
    CHECK(near(p[0], 1.0f));
    CHECK(near(p[1], 0.0f));

    // Image v grows downward, camera +Y is up: below centre is negative Y.
    CHECK(unproject_pixel(intr, 1920, 1080, 960.0f, 1540.0f, 1.0f, p));
    CHECK(near(p[1], -1.0f));

    // The streamed image is a scaled copy of the capture the intrinsics
    // describe, so the same physical point must unproject identically from a
    // half-size image at half the pixel coordinates. This is the bug that
    // would put every landmark at twice the angle off-axis.
    float q[3];
    CHECK(unproject_pixel(intr, 960, 540, 980.0f, 270.0f, 1.0f, q));
    CHECK(unproject_pixel(intr, 1920, 1080, 1960.0f, 540.0f, 1.0f, p));
    CHECK(near(q[0], p[0]));
    CHECK(near(q[1], p[1]));
    CHECK(near(q[2], p[2]));

    // sb_depth_t's "no reading" sentinel and degenerate intrinsics are
    // refusals, not silently-zero points.
    CHECK(!unproject_pixel(intr, 1920, 1080, 960.0f, 540.0f, 0.0f, p));
    CHECK(!unproject_pixel(intr, 1920, 1080, 960.0f, 540.0f, -1.0f, p));
    CHECK(!unproject_pixel(intr, 1920, 1080, 960.0f, 540.0f, NAN, p));
    sb_intrinsics_t bad{};
    CHECK(!unproject_pixel(bad, 1920, 1080, 10.0f, 10.0f, 1.0f, p));
    CHECK(!unproject_pixel(intr, 0, 0, 10.0f, 10.0f, 1.0f, p));
}

static void test_camera_to_scene() {
    float p[3];
    const float p_cam[3] = {0.1f, 0.2f, -1.0f};

    // Identity rotation: a pure translation.
    const float ident[4] = {0, 0, 0, 1};
    const float origin[3] = {1.0f, 2.0f, 3.0f};
    camera_to_scene(origin, ident, p_cam, p);
    CHECK(near(p[0], 1.1f));
    CHECK(near(p[1], 2.2f));
    CHECK(near(p[2], 2.0f));

    // Yaw +90° about +Y takes camera -Z (forward) onto scene -X.
    const float s = std::sin(1.5707963f * 0.5f), c = std::cos(1.5707963f * 0.5f);
    const float yaw90[4] = {0, s, 0, c};
    const float fwd[3] = {0, 0, -1};
    const float zero[3] = {0, 0, 0};
    camera_to_scene(zero, yaw90, fwd, p);
    CHECK(near(p[0], -1.0f, 1e-3f));
    CHECK(near(p[1], 0.0f, 1e-3f));
    CHECK(near(p[2], 0.0f, 1e-3f));

    // Round trip against the whole pipeline: a pixel on the optical axis two
    // metres out, from a camera one metre up looking down -Z, must land two
    // metres in front of the head at head height.
    const sb_intrinsics_t intr = make_intr();
    float cam[3];
    CHECK(unproject_pixel(intr, 1920, 1080, intr.cx, intr.cy, 2.0f, cam));
    const float head[3] = {0.0f, 1.0f, 0.0f};
    camera_to_scene(head, ident, cam, p);
    CHECK(near(p[0], 0.0f));
    CHECK(near(p[1], 1.0f));
    CHECK(near(p[2], -2.0f));
}

// ---------------------------------------------------------------------------
// hands-inject payload
// ---------------------------------------------------------------------------

// 21 landmarks whose x encodes the landmark index, so a reorder is visible.
static std::string joints_json(float base) {
    std::string s = "[";
    for (int i = 0; i < 21; i++) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s[%.3f,0.0,-0.5]", i ? "," : "",
                      (double)(base + (float)i));
        s += buf;
    }
    return s + "]";
}

static std::string payload(const char *chirality, float base) {
    return std::string("{\"t\":1000,\"hands\":[{\"chirality\":\"") + chirality +
           "\",\"confidence\":0.9,\"joints\":" + joints_json(base) + "}]}";
}

static void test_parse() {
    injected_hands out;
    std::string err;

    CHECK(parse_hands_inject(payload("left", 0.0f), out, err));
    CHECK(out.count == 1);
    CHECK(out.is_left[0]);
    CHECK(out.t_ms == 1000);
    CHECK(out.hands[0].joint_count == SB_HAND_JOINT_COUNT);
    // Landmark mp must have landed on SB joint mediapipe_to_sb_joint(mp).
    for (int mp = 0; mp < SB_HAND_JOINT_COUNT; mp++) {
        const int sb = mediapipe_to_sb_joint(mp);
        CHECK(near(out.hands[0].joints[sb][0], (float)mp));
        CHECK(near(out.hands[0].joints[sb][3], 0.9f));
    }

    // Two hands, and hand_index numbered in arrival order.
    std::string two = "{\"t\":5,\"hands\":[{\"chirality\":\"left\",\"joints\":" +
                      joints_json(0.0f) +
                      "},{\"chirality\":\"right\",\"joints\":" +
                      joints_json(100.0f) + "}]}";
    CHECK(parse_hands_inject(two, out, err));
    CHECK(out.count == 2);
    CHECK(out.is_left[0] && !out.is_left[1]);
    CHECK(out.hands[1].hand_index == 1);
    // Confidence defaults to 1 when the sender omits it.
    CHECK(near(out.hands[0].joints[SB_JOINT_WRIST][3], 1.0f));

    // No hands in view is a valid frame, not an error.
    CHECK(parse_hands_inject("{\"t\":5,\"hands\":[]}", out, err));
    CHECK(out.count == 0);

    // Refusals. Each of these reaching the gesture engine would be worse than
    // a dropped frame.
    CHECK(!parse_hands_inject("not json", out, err));
    CHECK(!parse_hands_inject("[1,2,3]", out, err));
    CHECK(!parse_hands_inject("{\"hands\":[]}", out, err));  // no t
    CHECK(!parse_hands_inject(
        "{\"t\":1,\"hands\":[{\"chirality\":\"middle\",\"joints\":" +
            joints_json(0.0f) + "}]}",
        out, err));
    CHECK(!parse_hands_inject("{\"t\":1,\"hands\":[{\"chirality\":\"left\","
                              "\"joints\":[[0,0,0]]}]}",
                              out, err));  // 1 joint, not 21
    CHECK(!parse_hands_inject(
        "{\"t\":1,\"hands\":[{\"chirality\":\"left\",\"joints\":" +
            joints_json(NAN) + "}]}",
        out, err));
    CHECK(!err.empty());
}

static void test_store_freshness() {
    hand_inject_store store;
    CHECK(!store.ever_set());
    CHECK(!store.fresh(hand_inject_now_ms()));
    CHECK(store.age_ms(hand_inject_now_ms()) == 0);

    injected_hands h;
    std::string err;
    CHECK(parse_hands_inject(payload("right", 0.0f), h, err));
    store.set(h);
    const uint64_t t0 = hand_inject_now_ms();
    CHECK(store.ever_set());
    CHECK(store.fresh(t0));
    CHECK(store.fresh(t0 + HAND_INJECT_FRESH_MS - 1));
    CHECK(!store.fresh(t0 + HAND_INJECT_FRESH_MS));
    CHECK(store.age_ms(t0 + 40) >= 40);
}

// ---------------------------------------------------------------------------
// scene integration: injection outranks the phone while it is fresh
// ---------------------------------------------------------------------------

static void test_scene_takeover() {
    scene world;
    std::string err;

    // Phone hand first: a wrist parked at x = -9, so the source is obvious.
    sb_hand_t phone{};
    phone.joint_count = SB_HAND_JOINT_COUNT;
    for (int j = 0; j < SB_HAND_JOINT_COUNT; j++) {
        phone.joints[j][0] = -9.0f;
        phone.joints[j][3] = 1.0f;
    }
    world.inject_hand(0, phone);
    world.tick(0.016f);
    CHECK(world.hands_source_status().rfind("source=phone", 0) == 0);

    sb_hand_t got;
    CHECK(world.hand_joints(0, got));
    CHECK(near(got.joints[SB_JOINT_WRIST][0], -9.0f));

    // Mac hands take over, in MediaPipe order, and land in SB order.
    CHECK(world.hands_inject(payload("left", 0.0f), err));
    world.tick(0.016f);
    CHECK(world.hands_source_status().rfind("source=mac", 0) == 0);
    CHECK(world.hand_joints(0, got));
    CHECK(near(got.joints[SB_JOINT_INDEX_TIP][0], 8.0f));

    // `hands dump` reports the source and reports joints back in MediaPipe
    // order, so the eval harness can compare the two sources landmark by
    // landmark without a second table.
    const std::string dump = world.hands_dump_json();
    CHECK(dump.find("\"source\":\"mac\"") != std::string::npos);
    CHECK(dump.find("\"frame\":\"scene\"") != std::string::npos);
    CHECK(dump.find("[8.00000,0.00000,-0.50000") != std::string::npos);

    // A malformed payload is refused and leaves the held injection alone.
    CHECK(!world.hands_inject("{\"t\":1}", err));
    CHECK(world.hands_source_status().rfind("source=mac", 0) == 0);
}

int main() {
    test_joint_order();
    test_unproject();
    test_camera_to_scene();
    test_parse();
    test_store_freshness();
    test_scene_takeover();

    if (g_failures) {
        std::fprintf(stderr, "hand_inject: %d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("hand_inject: all checks passed\n");
    return 0;
}
