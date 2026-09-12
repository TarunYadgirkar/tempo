// test_tap.cpp — Synthetic tests for ge_tap_listener_t.
//
// Drives a fingertip down through a configured plane and back up,
// verifying:
//   1. Exactly one tap fires at the correct contact coordinates.
//   2. Suppression: a second descent without lifting first is ignored.
//   3. After lifting > lift_threshold, the next descent fires again.
//   4. Lateral motion across the plane (no vertical descent) does not fire.
//   5. Per-fingertip independence: thumb tapping while index hasn't
//      doesn't suppress index.

#include "gesture_engine.h"
#include "gesture_tap.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

struct TapEvent {
    int hand;
    int finger;
    float xy[2];
};

static std::vector<TapEvent> g_events;

static void on_tap(int hand, int finger, const float xy[2], void *ud) {
    (void)ud;
    g_events.push_back({hand, finger, {xy[0], xy[1]}});
}

// Hand fixture: place every joint sanely, then move just the requested
// fingertip to the supplied world coordinates.  We don't care about the
// other features of the gesture engine here.
static ge_hand_t make_hand_with_tip(int tip_joint, float wx, float wy, float wz) {
    ge_hand_t h{};
    h.present = true;
    for (int j = 0; j < GE_JOINT_COUNT; ++j) {
        h.joints[j][0] = 0; h.joints[j][1] = 0; h.joints[j][2] = 0;
        h.joints[j][3] = 1.0f;
    }
    h.joints[tip_joint][0] = wx;
    h.joints[tip_joint][1] = wy;
    h.joints[tip_joint][2] = wz;
    h.joints[tip_joint][3] = 1.0f;
    return h;
}

// Plane: horizontal table at y=0, normal pointing up (+Y), x-axis = +X.
static const float kOrigin[3] = {0.0f, 0.0f, 0.0f};
static const float kNormal[3] = {0.0f, 1.0f, 0.0f};
static const float kAxisX[3]  = {1.0f, 0.0f, 0.0f};

static ge_tap_listener_t *make_listener() {
    return ge_tap_listener_create(kOrigin, kNormal, kAxisX,
                                  /*slab*/ 0.005f,
                                  /*lift*/ 0.01f,
                                  on_tap, nullptr);
}

// Send two frames of identical data to bootstrap "previous position".
static void send_seed(ge_tap_listener_t *l, const ge_hand_t &hand) {
    ge_hand_t hands[2] = {hand, {}};
    hands[1].present = false;
    ge_tap_listener_update(l, hands, 1.0f / 60.0f);
}

// ---------------------------------------------------------------------------
// Test 1: single descent fires exactly one tap
// ---------------------------------------------------------------------------

static void test_single_tap_fires() {
    g_events.clear();
    auto *l = make_listener();
    assert(l);

    // Index fingertip starts at y=+0.05, target contact at world (0.10, 0, -0.20).
    // Walk it down: 0.05 → 0.03 → 0.01 → 0.001 → -0.002.
    float ys[] = {0.05f, 0.03f, 0.01f, 0.001f, -0.002f};
    for (float y : ys) {
        ge_hand_t h = make_hand_with_tip(GE_JOINT_INDEX_TIP, 0.10f, y, -0.20f);
        ge_hand_t hands[2] = {h, {}};
        hands[1].present = false;
        ge_tap_listener_update(l, hands, 1.0f / 60.0f);
    }
    // Lift back up
    for (float y : {0.0f, 0.005f, 0.02f}) {
        ge_hand_t h = make_hand_with_tip(GE_JOINT_INDEX_TIP, 0.10f, y, -0.20f);
        ge_hand_t hands[2] = {h, {}};
        hands[1].present = false;
        ge_tap_listener_update(l, hands, 1.0f / 60.0f);
    }

    assert(g_events.size() == 1 && "single descent fires one tap");
    assert(g_events[0].hand == 0);
    assert(g_events[0].finger == 1);  // INDEX
    // Plane (y=0, x-axis = +X, y-axis = normal × axis_x = (0,1,0) × (1,0,0) = (0,0,-1))
    // World pos (0.10, ~0, -0.20): plane_x = 0.10, plane_y = -in.z * (-1) = 0.20
    assert(std::abs(g_events[0].xy[0] - 0.10f) < 0.01f);
    assert(std::abs(g_events[0].xy[1] - 0.20f) < 0.01f);

    ge_tap_listener_destroy(l);
    std::printf("PASS test_single_tap_fires\n");
}

// ---------------------------------------------------------------------------
// Test 2: suppression — second descent without lifting is ignored
// ---------------------------------------------------------------------------

static void test_suppression_no_double_tap() {
    g_events.clear();
    auto *l = make_listener();
    assert(l);

    auto descend_once = [&](float start_y) {
        for (float y : {start_y, start_y * 0.5f, 0.001f, -0.002f}) {
            ge_hand_t h = make_hand_with_tip(GE_JOINT_INDEX_TIP, 0.0f, y, 0.0f);
            ge_hand_t hands[2] = {h, {}};
            hands[1].present = false;
            ge_tap_listener_update(l, hands, 1.0f / 60.0f);
        }
    };
    descend_once(0.04f);
    // Without lifting above 0.01m, descend again.  Should NOT tap.
    for (float y : {0.001f, 0.002f, 0.001f, -0.002f}) {
        ge_hand_t h = make_hand_with_tip(GE_JOINT_INDEX_TIP, 0.0f, y, 0.0f);
        ge_hand_t hands[2] = {h, {}};
        hands[1].present = false;
        ge_tap_listener_update(l, hands, 1.0f / 60.0f);
    }
    assert(g_events.size() == 1 && "no double-tap without lift");

    ge_tap_listener_destroy(l);
    std::printf("PASS test_suppression_no_double_tap\n");
}

// ---------------------------------------------------------------------------
// Test 3: after lifting > lift_threshold, next descent fires again
// ---------------------------------------------------------------------------

static void test_relift_then_retap() {
    g_events.clear();
    auto *l = make_listener();
    assert(l);

    // First tap.
    for (float y : {0.05f, 0.02f, 0.001f, -0.002f}) {
        ge_hand_t h = make_hand_with_tip(GE_JOINT_INDEX_TIP, 0, y, 0);
        ge_hand_t hands[2] = {h, {}};
        hands[1].present = false;
        ge_tap_listener_update(l, hands, 1.0f / 60.0f);
    }
    // Lift well above threshold (>0.01).
    for (float y : {0.005f, 0.02f, 0.04f}) {
        ge_hand_t h = make_hand_with_tip(GE_JOINT_INDEX_TIP, 0, y, 0);
        ge_hand_t hands[2] = {h, {}};
        hands[1].present = false;
        ge_tap_listener_update(l, hands, 1.0f / 60.0f);
    }
    // Second descent.
    for (float y : {0.03f, 0.01f, 0.001f, -0.002f}) {
        ge_hand_t h = make_hand_with_tip(GE_JOINT_INDEX_TIP, 0, y, 0);
        ge_hand_t hands[2] = {h, {}};
        hands[1].present = false;
        ge_tap_listener_update(l, hands, 1.0f / 60.0f);
    }
    assert(g_events.size() == 2 && "two taps separated by a lift");

    ge_tap_listener_destroy(l);
    std::printf("PASS test_relift_then_retap\n");
}

// ---------------------------------------------------------------------------
// Test 4: pure lateral motion at constant height does not fire
// ---------------------------------------------------------------------------

static void test_lateral_motion_no_tap() {
    g_events.clear();
    auto *l = make_listener();
    assert(l);

    // Fingertip slides parallel to plane at y=+0.02 (above slab).
    for (float x : {-0.05f, -0.02f, 0.0f, 0.02f, 0.05f}) {
        ge_hand_t h = make_hand_with_tip(GE_JOINT_INDEX_TIP, x, 0.02f, 0.0f);
        ge_hand_t hands[2] = {h, {}};
        hands[1].present = false;
        ge_tap_listener_update(l, hands, 1.0f / 60.0f);
    }
    assert(g_events.empty() && "lateral motion above slab does not tap");

    ge_tap_listener_destroy(l);
    std::printf("PASS test_lateral_motion_no_tap\n");
}

// ---------------------------------------------------------------------------
// Test 5: thumb and index tap independently
// ---------------------------------------------------------------------------

static void test_per_fingertip_independence() {
    g_events.clear();
    auto *l = make_listener();
    assert(l);

    // Frame 0: both at +0.04 (seed)
    {
        ge_hand_t h{};
        h.present = true;
        for (int j = 0; j < GE_JOINT_COUNT; ++j) { h.joints[j][3] = 1.0f; }
        h.joints[GE_JOINT_THUMB_TIP][0]=0;     h.joints[GE_JOINT_THUMB_TIP][1]=0.04f;
        h.joints[GE_JOINT_INDEX_TIP][0]=0.05f; h.joints[GE_JOINT_INDEX_TIP][1]=0.04f;
        ge_hand_t hands[2] = {h, {}};
        hands[1].present = false;
        ge_tap_listener_update(l, hands, 1.0f / 60.0f);
    }
    // Frame 1: thumb taps (descends through plane), index stays high
    {
        ge_hand_t h{};
        h.present = true;
        for (int j = 0; j < GE_JOINT_COUNT; ++j) { h.joints[j][3] = 1.0f; }
        h.joints[GE_JOINT_THUMB_TIP][0]=0;     h.joints[GE_JOINT_THUMB_TIP][1]=-0.002f;
        h.joints[GE_JOINT_INDEX_TIP][0]=0.05f; h.joints[GE_JOINT_INDEX_TIP][1]=0.04f;
        ge_hand_t hands[2] = {h, {}};
        hands[1].present = false;
        ge_tap_listener_update(l, hands, 1.0f / 60.0f);
    }
    assert(g_events.size() == 1 && g_events[0].finger == 0 && "thumb tap fires");

    g_events.clear();
    // Index tap doesn't care that thumb is suppressed.
    {
        ge_hand_t h{};
        h.present = true;
        for (int j = 0; j < GE_JOINT_COUNT; ++j) { h.joints[j][3] = 1.0f; }
        h.joints[GE_JOINT_THUMB_TIP][0]=0;     h.joints[GE_JOINT_THUMB_TIP][1]=-0.002f;
        h.joints[GE_JOINT_INDEX_TIP][0]=0.05f; h.joints[GE_JOINT_INDEX_TIP][1]=-0.002f;
        ge_hand_t hands[2] = {h, {}};
        hands[1].present = false;
        ge_tap_listener_update(l, hands, 1.0f / 60.0f);
    }
    assert(g_events.size() == 1 && g_events[0].finger == 1 && "index tap fires while thumb suppressed");

    ge_tap_listener_destroy(l);
    std::printf("PASS test_per_fingertip_independence\n");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
    std::printf("=== gesture-engine tap tests ===\n");
    test_single_tap_fires();
    test_suppression_no_double_tap();
    test_relift_then_retap();
    test_lateral_motion_no_tap();
    test_per_fingertip_independence();
    std::printf("=== ALL TAP TESTS PASSED ===\n");
    return 0;
}
