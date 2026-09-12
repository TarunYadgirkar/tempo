// test_gestures.cpp — Unit tests for the gesture recognition engine.
//
// Tests use synthetic hand joint data to verify:
//   1. Feature extraction (distances, curls, projections)
//   2. Gesture state machine (begin, update, end, cancel)
//   3. Hysteresis (activate at one threshold, deactivate at another)
//   4. Priority suppression (higher-priority gesture blocks lower)
//   5. Delta tracking for dynamic gestures

#include "gesture_engine.h"
#include "ge_features.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

// ---------------------------------------------------------------------------
// Helpers: build synthetic hand data
// ---------------------------------------------------------------------------

// Create a hand with all joints at a default "open hand" position
static ge_hand_t make_open_hand() {
    ge_hand_t h{};
    h.present = true;

    // Place joints in a rough open-hand configuration
    // Wrist at origin, fingers extending along +Y
    float base_y = 0.0f;
    // Wrist
    h.joints[GE_JOINT_WRIST][0] = 0; h.joints[GE_JOINT_WRIST][1] = base_y; h.joints[GE_JOINT_WRIST][2] = 0;
    h.joints[GE_JOINT_WRIST][3] = 1.0f;

    // Thumb: extending to the side (realistic spread so fingertip_gather_radius
    // stays above 0.065m for the default open hand configuration)
    float tx = -0.07f;
    h.joints[GE_JOINT_THUMB_CMC][0] = tx; h.joints[GE_JOINT_THUMB_CMC][1] = 0.02f; h.joints[GE_JOINT_THUMB_CMC][2] = 0; h.joints[GE_JOINT_THUMB_CMC][3] = 1.0f;
    h.joints[GE_JOINT_THUMB_MCP][0] = tx; h.joints[GE_JOINT_THUMB_MCP][1] = 0.04f; h.joints[GE_JOINT_THUMB_MCP][2] = 0; h.joints[GE_JOINT_THUMB_MCP][3] = 1.0f;
    h.joints[GE_JOINT_THUMB_IP][0]  = tx; h.joints[GE_JOINT_THUMB_IP][1]  = 0.06f; h.joints[GE_JOINT_THUMB_IP][2]  = 0; h.joints[GE_JOINT_THUMB_IP][3]  = 1.0f;
    h.joints[GE_JOINT_THUMB_TIP][0] = tx; h.joints[GE_JOINT_THUMB_TIP][1] = 0.08f; h.joints[GE_JOINT_THUMB_TIP][2] = 0; h.joints[GE_JOINT_THUMB_TIP][3] = 1.0f;

    // Fingers: extending along +Y with realistic X spread (~24cm thumb-to-pinky
    // so a thumb-at-index-tip pose stays well outside pinch_right_click's
    // 55 mm thumb-middle trigger threshold — index at x=-0.04, middle at
    // x=+0.08 keeps thumb-middle ≈ 120 mm during a clean 2-finger pinch).
    struct { int mcp, pip, dip, tip; float x_offset; } fingers[] = {
        {GE_JOINT_INDEX_MCP,  GE_JOINT_INDEX_PIP,  GE_JOINT_INDEX_DIP,  GE_JOINT_INDEX_TIP,  -0.04f},
        {GE_JOINT_MIDDLE_MCP, GE_JOINT_MIDDLE_PIP, GE_JOINT_MIDDLE_DIP, GE_JOINT_MIDDLE_TIP,  0.08f},
        {GE_JOINT_RING_MCP,   GE_JOINT_RING_PIP,   GE_JOINT_RING_DIP,   GE_JOINT_RING_TIP,   0.14f},
        {GE_JOINT_PINKY_MCP,  GE_JOINT_PINKY_PIP,  GE_JOINT_PINKY_DIP,  GE_JOINT_PINKY_TIP,   0.20f},
    };

    for (auto &f : fingers) {
        // Straight finger extending along +Y
        h.joints[f.mcp][0] = f.x_offset; h.joints[f.mcp][1] = 0.04f; h.joints[f.mcp][2] = 0; h.joints[f.mcp][3] = 1.0f;
        h.joints[f.pip][0] = f.x_offset; h.joints[f.pip][1] = 0.07f; h.joints[f.pip][2] = 0; h.joints[f.pip][3] = 1.0f;
        h.joints[f.dip][0] = f.x_offset; h.joints[f.dip][1] = 0.10f; h.joints[f.dip][2] = 0; h.joints[f.dip][3] = 1.0f;
        h.joints[f.tip][0] = f.x_offset; h.joints[f.tip][1] = 0.13f; h.joints[f.tip][2] = 0; h.joints[f.tip][3] = 1.0f;
    }

    return h;
}

// Move thumb tip to a specific position (for pinch testing)
static void set_thumb_tip(ge_hand_t &h, float x, float y, float z) {
    h.joints[GE_JOINT_THUMB_TIP][0] = x;
    h.joints[GE_JOINT_THUMB_TIP][1] = y;
    h.joints[GE_JOINT_THUMB_TIP][2] = z;
}

// Curl a finger by bending its joints (MCP->PIP->DIP->TIP fold back).
// The tip must end up on the opposite side of MCP from PIP to produce
// a high curl value (~0.85) with the MCP→PIP vs MCP→TIP angle formula.
static void curl_finger(ge_hand_t &h, int mcp, int pip, int dip, int tip) {
    float mx = h.joints[mcp][0], my = h.joints[mcp][1], mz = h.joints[mcp][2];
    h.joints[pip][0] = mx; h.joints[pip][1] = my + 0.03f; h.joints[pip][2] = mz;
    h.joints[dip][0] = mx; h.joints[dip][1] = my + 0.02f; h.joints[dip][2] = mz - 0.02f;
    h.joints[tip][0] = mx; h.joints[tip][1] = my - 0.01f; h.joints[tip][2] = mz - 0.01f;
}

static void curl_all_fingers(ge_hand_t &h) {
    curl_finger(h, GE_JOINT_INDEX_MCP,  GE_JOINT_INDEX_PIP,  GE_JOINT_INDEX_DIP,  GE_JOINT_INDEX_TIP);
    curl_finger(h, GE_JOINT_MIDDLE_MCP, GE_JOINT_MIDDLE_PIP, GE_JOINT_MIDDLE_DIP, GE_JOINT_MIDDLE_TIP);
    curl_finger(h, GE_JOINT_RING_MCP,   GE_JOINT_RING_PIP,   GE_JOINT_RING_DIP,   GE_JOINT_RING_TIP);
    curl_finger(h, GE_JOINT_PINKY_MCP,  GE_JOINT_PINKY_PIP,  GE_JOINT_PINKY_DIP,  GE_JOINT_PINKY_TIP);
}

// ---------------------------------------------------------------------------
// Test 1: Feature extraction — open hand
// ---------------------------------------------------------------------------

static void test_features_open_hand() {
    ge_hand_t h = make_open_hand();
    auto f = ge::compute_features(h);

    // Open hand should have low curl values
    assert(f.index_curl < 0.15f && "open hand index curl should be low");
    assert(f.middle_curl < 0.15f && "open hand middle curl should be low");
    assert(f.all_fingers_curl < 0.15f && "open hand average curl should be low");
    assert(f.palm_openness > 0.85f && "open hand palm should be open");

    // Thumb and index tips are far apart (open hand)
    assert(f.thumb_index_distance > 0.04f && "open hand pinch distance should be large");

    std::printf("PASS test_features_open_hand\n");
}

// ---------------------------------------------------------------------------
// Test 2: Feature extraction — pinch (thumb tip touching index tip)
// ---------------------------------------------------------------------------

static void test_features_pinch() {
    ge_hand_t h = make_open_hand();
    // Move thumb tip to index tip position
    float ix = h.joints[GE_JOINT_INDEX_TIP][0];
    float iy = h.joints[GE_JOINT_INDEX_TIP][1];
    float iz = h.joints[GE_JOINT_INDEX_TIP][2];
    set_thumb_tip(h, ix, iy, iz);

    auto f = ge::compute_features(h);

    assert(f.thumb_index_distance < 0.001f && "pinch distance should be ~0");

    std::printf("PASS test_features_pinch\n");
}

// ---------------------------------------------------------------------------
// Test 3: Feature extraction — curled fingers (fist)
// ---------------------------------------------------------------------------

static void test_features_fist() {
    ge_hand_t h = make_open_hand();
    curl_all_fingers(h);
    // Also curl thumb
    curl_finger(h, GE_JOINT_THUMB_CMC, GE_JOINT_THUMB_MCP, GE_JOINT_THUMB_IP, GE_JOINT_THUMB_TIP);

    auto f = ge::compute_features(h);

    assert(f.all_fingers_curl > 0.7f && "fist should have high curl");
    assert(f.palm_openness < 0.3f && "fist should have low palm openness");

    std::printf("PASS test_features_fist\n");
}

// ---------------------------------------------------------------------------
// Test 4: Thumb-on-index projection
// ---------------------------------------------------------------------------

static void test_thumb_on_index_projection() {
    ge_hand_t h = make_open_hand();

    // Place thumb tip at index MCP (knuckle) — should give projection ~0
    float mx = h.joints[GE_JOINT_INDEX_MCP][0];
    float my = h.joints[GE_JOINT_INDEX_MCP][1];
    float mz = h.joints[GE_JOINT_INDEX_MCP][2];
    set_thumb_tip(h, mx, my, mz);
    float proj0 = ge::project_thumb_on_index(h);
    assert(proj0 < 0.15f && "thumb at index MCP should project near 0");

    // Place thumb tip at index TIP (fingertip) — should give projection ~1
    float tx = h.joints[GE_JOINT_INDEX_TIP][0];
    float ty = h.joints[GE_JOINT_INDEX_TIP][1];
    float tz = h.joints[GE_JOINT_INDEX_TIP][2];
    set_thumb_tip(h, tx, ty, tz);
    float proj1 = ge::project_thumb_on_index(h);
    assert(proj1 > 0.85f && "thumb at index TIP should project near 1");

    // Place thumb tip at index PIP (middle of finger) — should give projection ~0.33
    float px = h.joints[GE_JOINT_INDEX_PIP][0];
    float py = h.joints[GE_JOINT_INDEX_PIP][1];
    float pz = h.joints[GE_JOINT_INDEX_PIP][2];
    set_thumb_tip(h, px, py, pz);
    float projm = ge::project_thumb_on_index(h);
    assert(projm > 0.2f && projm < 0.5f && "thumb at index PIP should project ~0.33");

    std::printf("PASS test_thumb_on_index_projection\n");
}

// ---------------------------------------------------------------------------
// Test 5: Gesture state machine — pinch_select begin/end
// ---------------------------------------------------------------------------

struct EventLog {
    std::vector<ge_event_t> events;
};

static void log_callback(const ge_event_t *event, void *user_data) {
    auto *log = static_cast<EventLog *>(user_data);
    log->events.push_back(*event);
    // Copy gesture name since the pointer may not persist
}

static void test_pinch_gesture_lifecycle() {
    ge_engine_t *ge = ge_create();
    assert(ge);

    EventLog log;
    ge_set_callback(ge, log_callback, &log);

    ge_hand_t hands[2]{};
    hands[0] = make_open_hand();
    hands[1].present = false;

    // Frame 1: open hand, no gesture
    ge_update(ge, hands, 1.0f / 30.0f);
    assert(log.events.empty() && "no gesture on open hand");

    // Frame 2+: move thumb to index tip (pinch).  Place 2 mm past
    // the tip in the finger pointing direction so the new
    // thumb_along_index_tip_direction > 0 gate (added 2026-05-23
    // to reject right_click poses where the thumb sits behind the
    // tip-plane) is satisfied — a real-world pinch lands with the
    // thumb pad slightly past the tip.
    float ix = hands[0].joints[GE_JOINT_INDEX_TIP][0];
    float iy = hands[0].joints[GE_JOINT_INDEX_TIP][1];
    float iz = hands[0].joints[GE_JOINT_INDEX_TIP][2];
    set_thumb_tip(hands[0], ix, iy + 0.002f, iz);

    // Simulate several frames to pass min_hold_ms (120ms for pinch_select)
    // At 30fps, each frame is ~33ms. First frame enters Pending (hold=0),
    // subsequent frames accumulate hold time until >=120ms -> Active.
    ge_update(ge, hands, 1.0f / 30.0f);  // enter Pending, hold=0
    ge_update(ge, hands, 1.0f / 30.0f);  // hold=33ms
    ge_update(ge, hands, 1.0f / 30.0f);  // hold=66ms
    ge_update(ge, hands, 1.0f / 30.0f);  // hold=99ms
    ge_update(ge, hands, 1.0f / 30.0f);  // hold=132ms, transition to Active

    // Should have received a BEGIN event
    bool got_begin = false;
    for (auto &ev : log.events) {
        if (ev.type == GE_EVENT_BEGIN) {
            got_begin = true;
            assert(ev.hand_index == 0);
        }
    }
    assert(got_begin && "should get BEGIN after min_hold_ms");

    // Release pinch (move thumb away).  The One-Euro joint filter needs a
    // couple of frames to track an instantaneous fixture teleport — a real
    // release spans several frames anyway.
    set_thumb_tip(hands[0], -0.1f, 0.1f, 0.0f);  // far from index
    log.events.clear();
    for (int i = 0; i < 3; ++i)
        ge_update(ge, hands, 1.0f / 30.0f);

    bool got_end = false;
    for (auto &ev : log.events) {
        if (ev.type == GE_EVENT_END) got_end = true;
    }
    assert(got_end && "should get END when pinch released");

    ge_destroy(ge);
    std::printf("PASS test_pinch_gesture_lifecycle\n");
}

// ---------------------------------------------------------------------------
// Test 6: Hysteresis — doesn't oscillate near threshold
// ---------------------------------------------------------------------------

static void test_hysteresis() {
    ge_engine_t *ge = ge_create();
    assert(ge);

    EventLog log;
    ge_set_callback(ge, log_callback, &log);

    ge_hand_t hands[2]{};
    hands[0] = make_open_hand();
    hands[1].present = false;

    // Feed one frame of the open hand so the convergence-recency
    // tracker (added 2026-05-23) sees thumb_index_distance > 30 mm
    // before the pinch.  Without this, the gate "thumb was distant
    // recently" fails and pinch_select never fires on the
    // synthetic-from-zero pinch pose.
    ge_update(ge, hands, 1.0f / 30.0f);

    // Hover the thumb BEYOND the index tip (ti = 80 mm, along the pointing
    // direction) before closing.  The joint filter smears fixture teleports
    // over ~2 frames; approaching from beyond the tip keeps
    // thumb_on_index_projection clamped at 1.0 the whole way, so the
    // filtered transit can't read as a thumb-slide (scroll).  80 mm sits
    // outside the LOOSE variant's 75 mm band (a 45 mm hover with approach
    // velocity IS a loose pinch and would fire it), and holding > 500 ms
    // ages the open-hand projection sample out of the standard variant's
    // range window — so the pinch below fires the STANDARD variant, whose
    // 35 / 48 mm hysteresis this test measures.
    float ix = hands[0].joints[GE_JOINT_INDEX_TIP][0];
    float iy = hands[0].joints[GE_JOINT_INDEX_TIP][1];
    float iz = hands[0].joints[GE_JOINT_INDEX_TIP][2];
    set_thumb_tip(hands[0], ix, iy + 0.08f, iz);
    for (int i = 0; i < 16; ++i)
        ge_update(ge, hands, 1.0f / 30.0f);
    log.events.clear();

    // Activate pinch: 2 mm past the tip in the finger pointing direction
    // so the along > 0 trigger gate is satisfied (see
    // test_pinch_gesture_lifecycle for the matching note).
    set_thumb_tip(hands[0], ix + 0.01f, iy + 0.002f, iz);

    // Pass through min_hold
    for (int i = 0; i < 5; ++i)
        ge_update(ge, hands, 1.0f / 30.0f);

    int begin_count = 0;
    for (auto &ev : log.events) {
        if (ev.type == GE_EVENT_BEGIN) begin_count++;
    }
    assert(begin_count == 1 && "should activate once");

    // Move to 0.035m (between the standard variant's trigger=0.035 and
    // release=0.048)
    log.events.clear();
    set_thumb_tip(hands[0], ix + 0.035f, iy, iz);
    ge_update(ge, hands, 1.0f / 30.0f);

    bool got_end = false;
    for (auto &ev : log.events) {
        if (ev.type == GE_EVENT_END) got_end = true;
    }
    assert(!got_end && "should NOT deactivate in hysteresis zone");

    // Move to 0.07m (past all release thresholds) — NOW should deactivate.
    // A few frames so the joint filter converges on the fixture teleport.
    log.events.clear();
    set_thumb_tip(hands[0], ix + 0.07f, iy, iz);
    for (int i = 0; i < 3; ++i)
        ge_update(ge, hands, 1.0f / 30.0f);

    got_end = false;
    for (auto &ev : log.events) {
        if (ev.type == GE_EVENT_END) got_end = true;
    }
    assert(got_end && "should deactivate past release threshold");

    ge_destroy(ge);
    std::printf("PASS test_hysteresis\n");
}

// ---------------------------------------------------------------------------
// Test 7: Hand lost cancels active gesture
// ---------------------------------------------------------------------------

static void test_hand_lost_cancel() {
    ge_engine_t *ge = ge_create();
    assert(ge);

    EventLog log;
    ge_set_callback(ge, log_callback, &log);

    ge_hand_t hands[2]{};
    hands[0] = make_open_hand();
    hands[1].present = false;

    // Open-hand frame so the convergence-recency tracker has a
    // pre-pinch ti > 30 mm — see test_hysteresis note.
    ge_update(ge, hands, 1.0f / 30.0f);
    log.events.clear();

    // Activate pinch.  +2 mm in y to keep along > 0.
    float ix = hands[0].joints[GE_JOINT_INDEX_TIP][0];
    float iy = hands[0].joints[GE_JOINT_INDEX_TIP][1];
    float iz = hands[0].joints[GE_JOINT_INDEX_TIP][2];
    set_thumb_tip(hands[0], ix, iy + 0.002f, iz);

    for (int i = 0; i < 5; ++i)
        ge_update(ge, hands, 1.0f / 30.0f);

    // Now lose the hand
    log.events.clear();
    hands[0].present = false;
    ge_update(ge, hands, 1.0f / 30.0f);

    bool got_cancel = false;
    for (auto &ev : log.events) {
        if (ev.type == GE_EVENT_CANCEL) got_cancel = true;
    }
    assert(got_cancel && "should get CANCEL when hand is lost");

    ge_destroy(ge);
    std::printf("PASS test_hand_lost_cancel\n");
}

// ---------------------------------------------------------------------------
// Test 8: Delta tracking for scroll gesture
// ---------------------------------------------------------------------------

static void test_scroll_delta() {
    ge_engine_t *ge = ge_create();
    assert(ge);

    EventLog log;
    ge_set_callback(ge, log_callback, &log);

    ge_hand_t hands[2]{};
    hands[0] = make_open_hand();
    hands[1].present = false;

    // Scroll requires convergence evidence (thumb must have been far
    // from index recently) + scalar motion gate (projection must
    // change). First run a few frames with the default open hand
    // (thumb at x=-0.07, far from index) to build convergence history.
    for (int i = 0; i < 4; ++i)
        ge_update(ge, hands, 1.0f / 30.0f);

    float ix = hands[0].joints[GE_JOINT_INDEX_TIP][0];
    float iy = hands[0].joints[GE_JOINT_INDEX_TIP][1];
    float iz = hands[0].joints[GE_JOINT_INDEX_TIP][2];
    // Place thumb 15 mm beside the index tip in z.
    set_thumb_tip(hands[0], ix, iy, iz + 0.015f);

    // Pose-hold frames so min_hold accumulates.
    for (int i = 0; i < 4; ++i)
        ge_update(ge, hands, 1.0f / 30.0f);
    // Now slide toward MCP (projection drops from 1.00 to ~0.7).
    set_thumb_tip(hands[0], ix, iy - 0.03f, iz + 0.015f);
    for (int i = 0; i < 4; ++i)
        ge_update(ge, hands, 1.0f / 30.0f);

    bool got_scroll_begin = false;
    for (auto &ev : log.events) {
        if (ev.type == GE_EVENT_BEGIN && ev.action == GE_ACTION_SCROLL)
            got_scroll_begin = true;
    }
    assert(got_scroll_begin
           && "scroll should activate when thumb sits beside index tip");

    // Continue sliding the thumb further toward MCP (but still within
    // release threshold) — UPDATE should report a scalar delta.
    log.events.clear();
    set_thumb_tip(hands[0], ix, iy - 0.04f, iz + 0.015f);
    ge_update(ge, hands, 1.0f / 30.0f);

    bool got_update_with_delta = false;
    for (auto &ev : log.events) {
        if (ev.type == GE_EVENT_UPDATE && ev.action == GE_ACTION_SCROLL) {
            if (std::abs(ev.delta[0]) > 0.001f) got_update_with_delta = true;
        }
    }
    assert(got_update_with_delta && "scroll UPDATE should report scalar delta");

    ge_destroy(ge);
    std::printf("PASS test_scroll_delta\n");
}

// ---------------------------------------------------------------------------
// Test 9: ge_get_feature returns computed values
// ---------------------------------------------------------------------------

static void test_get_feature() {
    ge_engine_t *ge = ge_create();
    assert(ge);

    ge_hand_t hands[2]{};
    hands[0] = make_open_hand();
    hands[1].present = false;

    ge_update(ge, hands, 1.0f / 30.0f);

    float dist = ge_get_feature(ge, 0, "thumb_index_distance");
    assert(dist > 0.01f && "should return a positive distance for open hand");

    float curl = ge_get_feature(ge, 0, "all_fingers_curl");
    assert(curl < 0.2f && "should return low curl for open hand");

    // Non-present hand should return 0
    float absent = ge_get_feature(ge, 1, "thumb_index_distance");
    assert(absent == 0.0f && "absent hand feature should be 0");

    ge_destroy(ge);
    std::printf("PASS test_get_feature\n");
}

// ---------------------------------------------------------------------------
// Test 10: Thumb-on-middle projection
// ---------------------------------------------------------------------------

static void test_thumb_on_middle_projection() {
    ge_hand_t h = make_open_hand();

    // Place thumb tip at middle finger MCP — should give projection ~0
    float mx = h.joints[GE_JOINT_MIDDLE_MCP][0];
    float my = h.joints[GE_JOINT_MIDDLE_MCP][1];
    float mz = h.joints[GE_JOINT_MIDDLE_MCP][2];
    set_thumb_tip(h, mx, my, mz);
    float proj0 = ge::project_thumb_on_middle(h);
    assert(proj0 < 0.15f && "thumb at middle MCP should project near 0");

    // Place thumb tip at middle finger TIP — should give projection ~1
    float tx = h.joints[GE_JOINT_MIDDLE_TIP][0];
    float ty = h.joints[GE_JOINT_MIDDLE_TIP][1];
    float tz = h.joints[GE_JOINT_MIDDLE_TIP][2];
    set_thumb_tip(h, tx, ty, tz);
    float proj1 = ge::project_thumb_on_middle(h);
    assert(proj1 > 0.85f && "thumb at middle TIP should project near 1");

    std::printf("PASS test_thumb_on_middle_projection\n");
}

// ---------------------------------------------------------------------------
// Test 11: Fingertip gather radius
// ---------------------------------------------------------------------------

static void test_fingertip_gather_radius() {
    // Open hand: tips are spread apart, gather radius should be large
    ge_hand_t h = make_open_hand();
    auto f = ge::compute_features(h);
    assert(f.fingertip_gather_radius > 0.03f && "open hand fingertips should be spread");

    // All tips at the same point: gather radius should be ~0
    ge_hand_t h2 = make_open_hand();
    float gx = 0.0f, gy = 0.10f, gz = 0.0f;
    h2.joints[GE_JOINT_THUMB_TIP][0] = gx;  h2.joints[GE_JOINT_THUMB_TIP][1] = gy;  h2.joints[GE_JOINT_THUMB_TIP][2] = gz;
    h2.joints[GE_JOINT_INDEX_TIP][0] = gx;  h2.joints[GE_JOINT_INDEX_TIP][1] = gy;  h2.joints[GE_JOINT_INDEX_TIP][2] = gz;
    h2.joints[GE_JOINT_MIDDLE_TIP][0] = gx; h2.joints[GE_JOINT_MIDDLE_TIP][1] = gy; h2.joints[GE_JOINT_MIDDLE_TIP][2] = gz;
    h2.joints[GE_JOINT_RING_TIP][0] = gx;   h2.joints[GE_JOINT_RING_TIP][1] = gy;   h2.joints[GE_JOINT_RING_TIP][2] = gz;
    h2.joints[GE_JOINT_PINKY_TIP][0] = gx;  h2.joints[GE_JOINT_PINKY_TIP][1] = gy;  h2.joints[GE_JOINT_PINKY_TIP][2] = gz;
    auto f2 = ge::compute_features(h2);
    assert(f2.fingertip_gather_radius < 0.001f && "gathered fingertips should have radius ~0");

    std::printf("PASS test_fingertip_gather_radius\n");
}

// ---------------------------------------------------------------------------
// Test 12: Right-click gesture fires with projection-based detection
// ---------------------------------------------------------------------------

static void test_right_click_gesture() {
    ge_engine_t *ge = ge_create();
    assert(ge);

    EventLog log;
    ge_set_callback(ge, log_callback, &log);

    ge_hand_t hands[2]{};
    hands[0] = make_open_hand();
    hands[1].present = false;

    // Open-hand frame so the convergence-recency tracker sees a
    // distant thumb before the 3-finger pinch.
    ge_update(ge, hands, 1.0f / 30.0f);
    log.events.clear();

    // Right-click was redefined 2026-05-22 as a 3-finger pinch:
    // thumb + index + middle all touching, ring/pinky extended.
    // Place index and middle tips ~20 mm apart, thumb between them
    // — pairwise distances all under the right_click thresholds but
    // none at literal zero (the engine's > 0.008 m lower-bound rejects
    // tracker zero-frame artifacts and would also reject this if all
    // three joints coincided).
    hands[0].joints[GE_JOINT_INDEX_TIP][0]  = -0.02f;
    hands[0].joints[GE_JOINT_INDEX_TIP][1]  =  0.13f;
    hands[0].joints[GE_JOINT_INDEX_TIP][2]  =  0.0f;
    hands[0].joints[GE_JOINT_MIDDLE_TIP][0] =  0.00f;
    hands[0].joints[GE_JOINT_MIDDLE_TIP][1] =  0.13f;
    hands[0].joints[GE_JOINT_MIDDLE_TIP][2] =  0.0f;
    set_thumb_tip(hands[0], -0.01f, 0.13f, 0.0f);   // midway

    // Run enough frames for min_hold_ms = 67 ms (~3 frames at 30 Hz).
    for (int i = 0; i < 6; ++i)
        ge_update(ge, hands, 1.0f / 30.0f);

    bool got_right_click = false;
    bool got_pinch = false;
    for (auto &ev : log.events) {
        if (ev.type == GE_EVENT_BEGIN && ev.action == GE_ACTION_POINTER_RIGHT_CLICK)
            got_right_click = true;
        if (ev.type == GE_EVENT_BEGIN && ev.action == GE_ACTION_POINTER_CLICK)
            got_pinch = true;
    }
    // pinch_right_click outranks pinch_select (priority 3 vs 1): the
    // 3-finger pose must end up as a right-click.  A pinch_select BEGIN
    // may precede it (shorter hold) but then gets cancelled — the
    // consumer commits a click on END, never on BEGIN.
    assert(got_right_click && "3-finger pose must fire right-click");
    if (got_pinch) {
        bool pinch_cancelled = false;
        for (auto &ev : log.events) {
            if (ev.type == GE_EVENT_CANCEL && ev.action == GE_ACTION_POINTER_CLICK)
                pinch_cancelled = true;
        }
        assert(pinch_cancelled && "an earlier pinch_select must be cancelled by right-click");
    }

    ge_destroy(ge);
    std::printf("PASS test_right_click_gesture\n");
}

// ---------------------------------------------------------------------------
// Test 15: Palm normal computation
// ---------------------------------------------------------------------------

static void test_palm_normal() {
    ge_hand_t h = make_open_hand();
    auto f = ge::compute_features(h);

    // Open hand with wrist at origin, index MCP at x=-0.04 and pinky MCP at x=0.10.
    // In our layout: fingers extend along +Y, spread along X.
    //   to_index = INDEX_MCP - WRIST = (-0.04, 0.04, 0)
    //   to_pinky = PINKY_MCP - WRIST = (0.10, 0.04, 0)
    //   cross = (-0.04,0.04,0) x (0.10,0.04,0)
    //         = (0.04*0 - 0*0.04, 0*0.10 - (-0.04)*0, (-0.04)*0.04 - 0.04*0.10)
    //         = (0, 0, -0.0016 - 0.004)
    //         = (0, 0, -0.0056)
    //   normalized = (0, 0, -1)
    // So palm_normal should point in -Z direction for this hand layout.
    assert(f.palm_normal.z < -0.9f && "palm normal should point in -Z for test hand layout");
    assert(std::abs(f.palm_normal.x) < 0.1f && "palm normal X should be near 0");
    assert(std::abs(f.palm_normal.y) < 0.1f && "palm normal Y should be near 0");

    // Verify it's normalized
    float len = f.palm_normal.length();
    assert(std::abs(len - 1.0f) < 0.01f && "palm normal should be unit length");

    // Verify get_feature_by_name works
    float nx = ge::get_feature_by_name(f, "palm_normal_x");
    float ny = ge::get_feature_by_name(f, "palm_normal_y");
    float nz = ge::get_feature_by_name(f, "palm_normal_z");
    assert(std::abs(nx - f.palm_normal.x) < 1e-6f);
    assert(std::abs(ny - f.palm_normal.y) < 1e-6f);
    assert(std::abs(nz - f.palm_normal.z) < 1e-6f);

    std::printf("PASS test_palm_normal\n");
}

// ---------------------------------------------------------------------------
// Test: keyboard_anchor gesture fires after 1500ms hold of open palm
// face-down; ge_pending_progress reports intermediate progress.
// ---------------------------------------------------------------------------

// Build an open hand laid flat with palm facing down (palm_normal ≈
// (0, -1, 0)).  Fingers extend in the -Z direction (forward) and
// INDEX_MCP / PINKY_MCP are at the same Y as the wrist so the cross
// product (INDEX-WRIST) × (PINKY-WRIST) lies in -Y.
static ge_hand_t make_palm_down_hand() {
    ge_hand_t h{};
    h.present = true;
    auto set = [&](int j, float x, float y, float z) {
        h.joints[j][0] = x; h.joints[j][1] = y; h.joints[j][2] = z;
        h.joints[j][3] = 1.0f;
    };
    /* Wrist at origin. */
    set(GE_JOINT_WRIST, 0.0f, 0.0f, 0.0f);
    /* Thumb spread to the left, all in the plane Y=0 (palm flat). */
    set(GE_JOINT_THUMB_CMC, -0.07f, 0.0f, -0.02f);
    set(GE_JOINT_THUMB_MCP, -0.07f, 0.0f, -0.04f);
    set(GE_JOINT_THUMB_IP,  -0.07f, 0.0f, -0.06f);
    set(GE_JOINT_THUMB_TIP, -0.07f, 0.0f, -0.08f);
    /* Four fingers extending forward (-Z), all in Y=0. */
    struct { int mcp,pip,dip,tip; float x; } fs[] = {
        {GE_JOINT_INDEX_MCP,  GE_JOINT_INDEX_PIP,  GE_JOINT_INDEX_DIP,  GE_JOINT_INDEX_TIP,  -0.04f},
        {GE_JOINT_MIDDLE_MCP, GE_JOINT_MIDDLE_PIP, GE_JOINT_MIDDLE_DIP, GE_JOINT_MIDDLE_TIP,  0.00f},
        {GE_JOINT_RING_MCP,   GE_JOINT_RING_PIP,   GE_JOINT_RING_DIP,   GE_JOINT_RING_TIP,    0.04f},
        {GE_JOINT_PINKY_MCP,  GE_JOINT_PINKY_PIP,  GE_JOINT_PINKY_DIP,  GE_JOINT_PINKY_TIP,   0.08f},
    };
    for (auto &f : fs) {
        set(f.mcp, f.x, 0.0f, -0.04f);
        set(f.pip, f.x, 0.0f, -0.07f);
        set(f.dip, f.x, 0.0f, -0.10f);
        set(f.tip, f.x, 0.0f, -0.13f);
    }
    return h;
}

static void test_palm_down_features() {
    /* Sanity: the synthetic hand really does have palm_normal facing
     * down and high openness.  The gesture test below depends on this. */
    auto f = ge::compute_features(make_palm_down_hand());
    assert(f.palm_normal.y < -0.9f && "palm_normal.y should be very negative");
    assert(f.palm_openness > 0.8f);
    assert(f.all_fingers_curl < 0.15f);
    std::printf("PASS test_palm_down_features\n");
}

static void test_keyboard_anchor_pending_progress_and_fire() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    EventLog log;
    ge_set_callback(ge, log_callback, &log);

    ge_hand_t hands[2];
    hands[0] = make_palm_down_hand();
    hands[1].present = false;

    /* Tick at 60 Hz.  After 5 frames (~83 ms) we should be in
     * Pending state but not yet emitting anything. */
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 5; ++i) ge_update(ge, hands, dt);
    assert(log.events.empty() && "no BEGIN yet at ~83ms hold");
    float p_early = ge_pending_progress(ge, "keyboard_anchor", 0);
    assert(p_early > 0.0f && p_early < 0.10f &&
           "pending progress should be small at ~83ms / 1500ms");

    /* Tick to ~600 ms (well into the holding window). */
    for (int i = 0; i < 31; ++i) ge_update(ge, hands, dt);
    assert(log.events.empty() && "no BEGIN at ~600ms hold");
    float p_mid = ge_pending_progress(ge, "keyboard_anchor", 0);
    assert(p_mid > 0.35f && p_mid < 0.5f &&
           "pending progress should be ~0.4 at 600ms / 1500ms");

    /* Tick out to ~1600 ms.  BEGIN should have fired by then. */
    for (int i = 0; i < 60; ++i) ge_update(ge, hands, dt);
    bool saw_begin = false;
    for (const auto &e : log.events) {
        if (e.type == GE_EVENT_BEGIN
            && e.action == GE_ACTION_KEYBOARD_ANCHOR) {
            saw_begin = true;
            /* event.position carries palm_center at activation. */
            assert(std::isfinite(e.position[0]));
            assert(std::isfinite(e.position[1]));
            assert(std::isfinite(e.position[2]));
        }
    }
    assert(saw_begin && "keyboard_anchor BEGIN expected after 1500ms hold");
    /* After Active, ge_pending_progress should be 0. */
    float p_after = ge_pending_progress(ge, "keyboard_anchor", 0);
    assert(p_after == 0.0f);

    ge_destroy(ge);
    std::printf("PASS test_keyboard_anchor_pending_progress_and_fire\n");
}

static void test_keyboard_anchor_cancels_on_palm_flip() {
    ge_engine_t *ge = ge_create();
    EventLog log;
    ge_set_callback(ge, log_callback, &log);

    ge_hand_t hands[2];
    hands[0] = make_palm_down_hand();
    hands[1].present = false;

    /* Hold palm-down for 400 ms → Pending but not yet Active. */
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 24; ++i) ge_update(ge, hands, dt);
    assert(ge_pending_progress(ge, "keyboard_anchor", 0) > 0.20f);

    /* Flip palm up (mirror across Y=0): keep fingers in same direction
     * but invert palm normal by swapping INDEX_MCP and PINKY_MCP X
     * values.  This makes (INDEX-WRIST) × (PINKY-WRIST) reverse sign. */
    hands[0].joints[GE_JOINT_INDEX_MCP][0] =  0.08f;
    hands[0].joints[GE_JOINT_PINKY_MCP][0] = -0.04f;

    /* Hold the new orientation: keyboard_anchor should release. */
    for (int i = 0; i < 10; ++i) ge_update(ge, hands, dt);
    assert(ge_pending_progress(ge, "keyboard_anchor", 0) == 0.0f);
    /* No BEGIN ever fired. */
    for (const auto &e : log.events) {
        assert(!(e.type == GE_EVENT_BEGIN
                 && e.action == GE_ACTION_KEYBOARD_ANCHOR));
    }
    ge_destroy(ge);
    std::printf("PASS test_keyboard_anchor_cancels_on_palm_flip\n");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Test: engine-hand -> input-slot remap accessor (ge_hand_input_slot)
//
// Reproduces the 2026-07-02 launcher failure.  A single hand seen only in
// tracker input slot 1 is bound by the continuity remap to engine hand 0, so
// its gesture events carry hand_index == 0.  A caller that then reads
// hands[0] gets the EMPTY slot (wrist at origin) — which pinned the fist-roll
// tracker at init=0 / rot=0.  ge_hand_input_slot() must report the true
// binding (engine 0 -> input slot 1) so the caller can recover the joints.
// ---------------------------------------------------------------------------
static void test_hand_input_slot_remap() {
    ge_engine_t *ge = ge_create();
    assert(ge);

    // Bad args → -1, never a crash.
    assert(ge_hand_input_slot(nullptr, 0) == -1);
    assert(ge_hand_input_slot(ge, -1) == -1);
    assert(ge_hand_input_slot(ge, 2) == -1);
    // No hands fed yet → neither engine hand is present.
    assert(ge_hand_input_slot(ge, 0) == -1);
    assert(ge_hand_input_slot(ge, 1) == -1);

    // Present ONLY in input slot 1 (slot 0 empty) — the hardware case.
    ge_hand_t hands[2]{};
    hands[0].present = false;
    hands[1] = make_open_hand();
    ge_update(ge, hands, 1.0f / 30.0f);

    // The lone hand must bind to engine hand 0 and report input slot 1.
    assert(ge_hand_input_slot(ge, 0) == 1 &&
           "hand seen only in input slot 1 must map engine-hand 0 -> slot 1");
    assert(ge_hand_input_slot(ge, 1) == -1 &&
           "the second engine hand is absent");

    // Identity case: a hand present only in slot 0 maps engine 0 -> slot 0.
    ge_engine_t *ge2 = ge_create();
    assert(ge2);
    ge_hand_t h2[2]{};
    h2[0] = make_open_hand();
    h2[1].present = false;
    ge_update(ge2, h2, 1.0f / 30.0f);
    assert(ge_hand_input_slot(ge2, 0) == 0);
    assert(ge_hand_input_slot(ge2, 1) == -1);

    ge_destroy(ge2);
    ge_destroy(ge);
    std::printf("PASS test_hand_input_slot_remap\n");
}

// ---------------------------------------------------------------------------
// Zero-frame artifact filter: iPhone Vision emits joint (0,0,0) for a few
// ticks when it loses lock on an occluded finger.  Features derived from a
// zeroed joint must come back indeterminate (NaN), triggers reading them must
// not fire, and a Pending hold must survive a 1-frame artifact unreset.
// ---------------------------------------------------------------------------

static void set_joint(ge_hand_t &h, int j, float x, float y, float z) {
    h.joints[j][0] = x; h.joints[j][1] = y; h.joints[j][2] = z;
}

static void translate_hand(ge_hand_t &h, float tx, float ty, float tz) {
    for (int j = 0; j < GE_JOINT_COUNT; ++j) {
        h.joints[j][0] += tx; h.joints[j][1] += ty; h.joints[j][2] += tz;
    }
}

static void test_zero_frame_artifact_features_indeterminate() {
    ge_hand_t h = make_open_hand();
    set_joint(h, GE_JOINT_MIDDLE_TIP, 0.0f, 0.0f, 0.0f);   // the artifact

    auto f = ge::compute_features(h);

    assert(std::isnan(f.thumb_middle_distance) &&
           "thumb-middle distance from a zeroed tip must be indeterminate");
    assert(std::isnan(f.thumb_on_middle_projection));
    assert(std::isnan(f.middle_tip_direction_to_thumb_cos));
    assert(std::isnan(f.middle_curl));
    assert(std::isnan(f.fingertip_gather_radius));
    // Features not involving the zeroed joint stay determinate.
    assert(!std::isnan(f.thumb_index_distance));
    assert(!std::isnan(f.index_curl));
    assert(!std::isnan(f.thumb_on_index_projection));

    std::printf("PASS test_zero_frame_artifact_features_indeterminate\n");
}

static void test_zero_frame_artifact_no_spurious_right_click() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    EventLog log;
    ge_set_callback(ge, log_callback, &log);

    ge_hand_t hands[2]{};
    hands[0] = make_open_hand();
    hands[1].present = false;

    // The failure regime from clip db9395: a TRUE 2-finger pinch held near
    // the body, i.e. near the WORLD origin.  Translate the hand so the
    // pinched thumb sits ~15 mm from (0,0,0): a zeroed middle tip then
    // yields thumb-"middle" ≈ 15 mm — past the static > 8 mm lower-bound
    // workaround, but well inside right_click's convergence thresholds.
    const float tx = 0.055f, ty = -0.132f;
    translate_hand(hands[0], tx, ty, 0.0f);

    // Open frame first (recency trackers want a distant thumb).
    ge_update(ge, hands, 1.0f / 30.0f);
    log.events.clear();

    // Pinch thumb onto the index tip (2 mm past it, per the along-gate).
    float ix = hands[0].joints[GE_JOINT_INDEX_TIP][0];
    float iy = hands[0].joints[GE_JOINT_INDEX_TIP][1];
    float iz = hands[0].joints[GE_JOINT_INDEX_TIP][2];
    set_thumb_tip(hands[0], ix, iy + 0.002f, iz);
    // Middle tip: the tracker loses lock behind the pinch and emits the
    // zero-frame artifact with collapsed confidence.
    set_joint(hands[0], GE_JOINT_MIDDLE_TIP, 0.0f, 0.0f, 0.0f);
    hands[0].joints[GE_JOINT_MIDDLE_TIP][3] = 0.2f;

    for (int i = 0; i < 10; ++i)
        ge_update(ge, hands, 1.0f / 30.0f);

    for (auto &ev : log.events) {
        assert(!(ev.type == GE_EVENT_BEGIN
                 && ev.action == GE_ACTION_POINTER_RIGHT_CLICK) &&
               "zero-frame middle-tip artifact must not fire right_click");
    }

    ge_destroy(ge);
    std::printf("PASS test_zero_frame_artifact_no_spurious_right_click\n");
}

static void test_zero_frame_artifact_preserves_pending_hold() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    EventLog log;
    ge_set_callback(ge, log_callback, &log);

    ge_hand_t hands[2]{};
    hands[0] = make_open_hand();
    hands[1].present = false;
    // Off-origin: the fixture wrist sits at (0,0,0), and the engine's
    // whole-hand tracker-garbage gate (wrist AND index tip at origin)
    // would otherwise read a zeroed index tip as "hand absent".
    translate_hand(hands[0], 0.30f, 0.20f, 0.10f);

    float ix = hands[0].joints[GE_JOINT_INDEX_TIP][0];
    float iy = hands[0].joints[GE_JOINT_INDEX_TIP][1];
    float iz = hands[0].joints[GE_JOINT_INDEX_TIP][2];

    // Open frame with the thumb hovering just BEYOND the index tip:
    // ti = 32 mm (so the convergence ratio sees a real approach) while
    // the thumb-on-index projection stays clamped at 1.0 throughout —
    // keeps the standard pinch_select variant's projection-stability
    // condition satisfied.
    set_thumb_tip(hands[0], ix, iy + 0.032f, iz);
    ge_update(ge, hands, 1.0f / 30.0f);
    log.events.clear();

    // Pinch frame: enters Pending (standard variant, min_hold 60 ms).
    set_thumb_tip(hands[0], ix, iy + 0.002f, iz);
    ge_update(ge, hands, 1.0f / 30.0f);

    // 1-frame index-tip artifact mid-hold.  The trigger reads an
    // indeterminate ti: "not met" but the hold timer must FREEZE, not
    // reset (no release-then-retrigger).
    ge_hand_t saved = hands[0];
    set_joint(hands[0], GE_JOINT_INDEX_TIP, 0.0f, 0.0f, 0.0f);
    ge_update(ge, hands, 1.0f / 30.0f);
    hands[0] = saved;

    // 2 more pinch frames: hold reaches 66 ms ≥ 60 ms → BEGIN.  If the
    // artifact had reset Pending, hold would only reach 33 ms here.
    for (int i = 0; i < 2; ++i)
        ge_update(ge, hands, 1.0f / 30.0f);

    bool got_begin = false;
    for (auto &ev : log.events) {
        if (ev.type == GE_EVENT_BEGIN && ev.action == GE_ACTION_POINTER_CLICK)
            got_begin = true;
    }
    assert(got_begin &&
           "a 1-frame artifact must not reset the Pending hold timer");

    ge_destroy(ge);
    std::printf("PASS test_zero_frame_artifact_preserves_pending_hold\n");
}

// ---------------------------------------------------------------------------
// Noisy-fixture robustness tests.  Deterministic gaussian tracking noise at
// the levels the iPhone Vision pipeline actually delivers (~±10 mm), checking
// the One-Euro filter + rearm debounce + velocity gates keep event streams
// clean.  σ_rigid shifts the whole hand per frame; σ_joint perturbs each
// joint independently.
// ---------------------------------------------------------------------------

#include <random>

static void add_tracking_noise(ge_hand_t &h, std::mt19937 &rng,
                               float sigma_rigid, float sigma_joint) {
    std::normal_distribution<float> nr(0.0f, sigma_rigid);
    std::normal_distribution<float> nj(0.0f, sigma_joint);
    float rx = nr(rng), ry = nr(rng), rz = nr(rng);
    for (int j = 0; j < GE_JOINT_COUNT; ++j) {
        h.joints[j][0] += rx + nj(rng);
        h.joints[j][1] += ry + nj(rng);
        h.joints[j][2] += rz + nj(rng);
    }
}

struct ActionCounts {
    int click_begin = 0, click_end = 0, click_cancel = 0;
    int any_begin = 0;
};

static ActionCounts count_actions(const std::vector<ge_event_t> &events) {
    ActionCounts c;
    for (const auto &ev : events) {
        if (ev.type == GE_EVENT_BEGIN) ++c.any_begin;
        if (ev.action != GE_ACTION_POINTER_CLICK) continue;
        if (ev.type == GE_EVENT_BEGIN)  ++c.click_begin;
        if (ev.type == GE_EVENT_END)    ++c.click_end;
        if (ev.type == GE_EVENT_CANCEL) ++c.click_cancel;
    }
    return c;
}

// (a) A pinch held for >1 s under ±10 mm noise must produce exactly one
// BEGIN/END pair — no flicker retriggering, no scroll stealing the hand.
static void test_noisy_pinch_hold_single_pair() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    EventLog log;
    ge_set_callback(ge, log_callback, &log);

    std::mt19937 rng(42);
    const float kSigmaRigid = 0.0035f, kSigmaJoint = 0.002f;
    const float dt = 1.0f / 30.0f;

    ge_hand_t base = make_open_hand();
    float ix = base.joints[GE_JOINT_INDEX_TIP][0];
    float iy = base.joints[GE_JOINT_INDEX_TIP][1];
    float iz = base.joints[GE_JOINT_INDEX_TIP][2];

    ge_hand_t hands[2]{};
    hands[1].present = false;

    auto tick_pose = [&](float thumb_dy, int frames) {
        for (int i = 0; i < frames; ++i) {
            hands[0] = base;
            set_thumb_tip(hands[0], ix, iy + thumb_dy, iz);
            add_tracking_noise(hands[0], rng, kSigmaRigid, kSigmaJoint);
            ge_update(ge, hands, dt);
        }
    };

    // Hover at 80 mm: outside every pinch_select band.  A 45 mm hover with
    // approach velocity is a LOOSE pinch by definition and used to be
    // masked only by that variant's inverted release.
    tick_pose(0.08f, 8);    // hover beyond the tip (convergence evidence)
    tick_pose(0.002f, 40);  // pinch held ~1.3 s under noise
    tick_pose(0.08f, 12);   // release back to the hover

    ActionCounts c = count_actions(log.events);
    if (c.click_begin != 1 || c.click_end != 1 || c.click_cancel != 0) {
        std::printf("noisy hold: begin=%d end=%d cancel=%d\n", c.click_begin,
                    c.click_end, c.click_cancel);
    }
    assert(c.click_begin == 1 && "jittered hold must BEGIN exactly once");
    assert(c.click_end == 1 && "jittered hold must END exactly once");
    assert(c.click_cancel == 0 && "no gesture may steal the jittered hold");

    ge_destroy(ge);
    std::printf("PASS test_noisy_pinch_hold_single_pair\n");
}

// (b) A hand sweeping fast with fingers apart must never trigger anything —
// distances transit trigger bands, but only from whole-hand motion + noise.
static void test_fast_wave_no_triggers() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    EventLog log;
    ge_set_callback(ge, log_callback, &log);

    std::mt19937 rng(7);
    const float dt = 1.0f / 30.0f;
    ge_hand_t hands[2]{};
    hands[1].present = false;

    // 3 s sweep: ±0.4 m at 1.2 Hz → peak wrist speed ≈ 3 m/s.
    for (int i = 0; i < 90; ++i) {
        float t = i * dt;
        float x = 0.4f * std::sin(2.0f * (float)M_PI * 1.2f * t);
        hands[0] = make_open_hand();
        translate_hand(hands[0], x, 0.0f, 0.0f);
        add_tracking_noise(hands[0], rng, 0.005f, 0.003f);
        ge_update(ge, hands, dt);
    }

    ActionCounts c = count_actions(log.events);
    if (c.any_begin != 0)
        std::printf("fast wave: %d spurious BEGINs\n", c.any_begin);
    assert(c.any_begin == 0 && "fast wave with open fingers must not trigger");

    ge_destroy(ge);
    std::printf("PASS test_fast_wave_no_triggers\n");
}

// (c) thumb-index distance oscillating ±3 mm around the trigger threshold
// must not machine-gun BEGIN/END pairs.
static void test_near_threshold_oscillation_no_machine_gun() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    EventLog log;
    ge_set_callback(ge, log_callback, &log);

    const float dt = 1.0f / 30.0f;
    ge_hand_t hands[2]{};
    hands[1].present = false;
    ge_hand_t base = make_open_hand();
    float ix = base.joints[GE_JOINT_INDEX_TIP][0];
    float iy = base.joints[GE_JOINT_INDEX_TIP][1];
    float iz = base.joints[GE_JOINT_INDEX_TIP][2];

    hands[0] = base;
    set_thumb_tip(hands[0], ix, iy + 0.045f, iz);
    for (int i = 0; i < 5; ++i) ge_update(ge, hands, dt);

    // 3 s of ti = 35 ± 3 mm at 6 Hz, straddling the 35 mm trigger.
    for (int i = 0; i < 90; ++i) {
        float t = i * dt;
        float ti = 0.035f + 0.003f * std::sin(2.0f * (float)M_PI * 6.0f * t);
        hands[0] = base;
        set_thumb_tip(hands[0], ix, iy + ti, iz);
        ge_update(ge, hands, dt);
    }

    // Exactly ONE click: the first dip under 35 mm carries convergence
    // evidence (ratio > 0.10 from the 45 mm hover), BEGIN fires, the 250 ms
    // click cap ENDs it, and the rearm debounce never clears because the
    // trigger is un-met for only ~83 ms per 167 ms cycle (< 100 ms).
    ActionCounts c = count_actions(log.events);
    if (c.click_begin != 1 || c.any_begin != 1)
        std::printf("oscillation: click_begin=%d any_begin=%d\n",
                    c.click_begin, c.any_begin);
    assert(c.click_begin == 1 &&
           "±3 mm oscillation must click exactly once");
    assert(c.any_begin == 1 &&
           "±3 mm oscillation must not fire any other gesture");

    ge_destroy(ge);
    std::printf("PASS test_near_threshold_oscillation_no_machine_gun\n");
}

// ---------------------------------------------------------------------------
// Hysteresis invariant over every compiled definition: for each trigger /
// release pair on the same feature the release threshold must sit strictly
// on the far side of the trigger (Less-trigger -> Greater-release above it,
// Greater-trigger -> Less-release below it).  An inverted pair releases on
// the frame after BEGIN — the loose pinch variants shipped that way.
// ---------------------------------------------------------------------------

static bool op_is_less(const char *op) {
    return std::strcmp(op, "less") == 0 || std::strcmp(op, "less_eq") == 0;
}
static bool op_is_greater(const char *op) {
    return std::strcmp(op, "greater") == 0 || std::strcmp(op, "greater_eq") == 0;
}

static void test_release_wider_than_trigger_for_every_variant() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    int pairs = 0;
    for (int g = 0; g < ge_gesture_count(ge); ++g) {
        int nt = ge_gesture_condition_count(ge, g, "trigger");
        int nr = ge_gesture_condition_count(ge, g, "release");
        for (int i = 0; i < nt; ++i) {
            const char *tf, *top; float tth;
            assert(ge_gesture_condition_at(ge, g, "trigger", i, &tf, &top, &tth));
            for (int j = 0; j < nr; ++j) {
                const char *rf, *rop; float rth;
                assert(ge_gesture_condition_at(ge, g, "release", j, &rf, &rop, &rth));
                if (std::strcmp(tf, rf) != 0) continue;
                bool checked = false, ok = true;
                if (op_is_less(top) && op_is_greater(rop)) {
                    checked = true; ok = rth > tth;
                } else if (op_is_greater(top) && op_is_less(rop)) {
                    checked = true; ok = rth < tth;
                }
                if (!checked) continue;
                ++pairs;
                if (!ok) {
                    std::printf("inverted hysteresis: %s.%s %s trigger %s %g vs release %s %g\n",
                                ge_gesture_name_at(ge, g), ge_gesture_variant_at(ge, g),
                                tf, top, tth, rop, rth);
                }
                assert(ok && "release threshold must be strictly wider than trigger");
            }
        }
    }
    assert(pairs >= 8 && "expected the compiled defs to carry hysteresis pairs");
    ge_destroy(ge);
    std::printf("PASS test_release_wider_than_trigger_for_every_variant (%d pairs)\n", pairs);
}

// ---------------------------------------------------------------------------
// indeterminate_release_ms: a NaN feature must not park a gesture forever.
// ---------------------------------------------------------------------------

#include <fstream>
#include <string>
#include <unistd.h>

static std::string write_temp_toml(const std::string &content) {
    char tmpl[] = "/tmp/spatos-test-gestures-XXXXXX";
    int fd = mkstemp(tmpl);
    assert(fd >= 0);
    close(fd);
    std::ofstream out(tmpl);
    out << content;
    return std::string(tmpl);
}

static int count_type(const std::vector<ge_event_t> &events, ge_event_type_t type,
                      ge_action_t action) {
    int n = 0;
    for (const auto &ev : events)
        if (ev.type == type && ev.action == action) ++n;
    return n;
}

// Bring a translated open hand to a settled 80 mm hover, then pinch; the
// STANDARD pinch_select variant fires on the 3rd pinch frame.
static void pinch_to_active(ge_engine_t *ge, ge_hand_t hands[2], EventLog &log) {
    float ix = hands[0].joints[GE_JOINT_INDEX_TIP][0];
    float iy = hands[0].joints[GE_JOINT_INDEX_TIP][1];
    float iz = hands[0].joints[GE_JOINT_INDEX_TIP][2];
    set_thumb_tip(hands[0], ix, iy + 0.08f, iz);
    for (int i = 0; i < 16; ++i) ge_update(ge, hands, 1.0f / 30.0f);
    log.events.clear();
    set_thumb_tip(hands[0], ix, iy + 0.002f, iz);
    for (int i = 0; i < 4; ++i) ge_update(ge, hands, 1.0f / 30.0f);
    assert(count_type(log.events, GE_EVENT_BEGIN, GE_ACTION_POINTER_CLICK) == 1);
}

// (a) Active + release unreadable for longer than the budget -> CANCEL.
static void test_indeterminate_release_cancels_active() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    // Lift the click cap so only the indeterminate budget can end it.
    std::string cfg = write_temp_toml("[pinch_select]\nmax_active_ms = 0\n");
    assert(ge_load_config(ge, cfg.c_str()));
    std::remove(cfg.c_str());
    assert(ge_gesture_indeterminate_release_ms(ge, "pinch_select") == 300);

    EventLog log;
    ge_set_callback(ge, log_callback, &log);
    ge_hand_t hands[2]{};
    hands[0] = make_open_hand();
    hands[1].present = false;
    translate_hand(hands[0], 0.30f, 0.20f, 0.10f);
    pinch_to_active(ge, hands, log);
    log.events.clear();

    // Index tip drops out: thumb_index_distance reads NaN, so the release
    // is indeterminate.  Inside the 300 ms budget nothing happens ...
    set_joint(hands[0], GE_JOINT_INDEX_TIP, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 6; ++i) ge_update(ge, hands, 1.0f / 30.0f);   // 200 ms
    assert(count_type(log.events, GE_EVENT_CANCEL, GE_ACTION_POINTER_CLICK) == 0);
    assert(count_type(log.events, GE_EVENT_END, GE_ACTION_POINTER_CLICK) == 0);
    // ... past it the gesture gives up with CANCEL, never END.
    for (int i = 0; i < 4; ++i) ge_update(ge, hands, 1.0f / 30.0f);   // 333 ms
    assert(count_type(log.events, GE_EVENT_CANCEL, GE_ACTION_POINTER_CLICK) == 1 &&
           "indeterminate release past the budget must CANCEL");
    assert(count_type(log.events, GE_EVENT_END, GE_ACTION_POINTER_CLICK) == 0);

    ge_destroy(ge);
    std::printf("PASS test_indeterminate_release_cancels_active\n");
}

// (b) The Pending freeze is capped by the same budget: a hold that has been
// unreadable for > 300 ms resets instead of resuming where it left off.
static void test_indeterminate_pending_freeze_capped() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    EventLog log;
    ge_set_callback(ge, log_callback, &log);

    ge_hand_t hands[2]{};
    hands[0] = make_open_hand();
    hands[1].present = false;
    translate_hand(hands[0], 0.30f, 0.20f, 0.10f);
    float ix = hands[0].joints[GE_JOINT_INDEX_TIP][0];
    float iy = hands[0].joints[GE_JOINT_INDEX_TIP][1];
    float iz = hands[0].joints[GE_JOINT_INDEX_TIP][2];

    // Same shape as test_zero_frame_artifact_preserves_pending_hold, but
    // the artifact lasts 10 frames (333 ms > 300 ms budget).
    set_thumb_tip(hands[0], ix, iy + 0.032f, iz);
    ge_update(ge, hands, 1.0f / 30.0f);
    set_thumb_tip(hands[0], ix, iy + 0.002f, iz);
    ge_update(ge, hands, 1.0f / 30.0f);           // Pending, hold 0
    ge_hand_t saved = hands[0];
    set_joint(hands[0], GE_JOINT_INDEX_TIP, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 10; ++i) ge_update(ge, hands, 1.0f / 30.0f);
    hands[0] = saved;

    // A frozen hold would need only 2 more frames (hold 66 >= 60 ms).  A
    // reset one re-enters Pending at 0 and needs 3.
    for (int i = 0; i < 2; ++i) ge_update(ge, hands, 1.0f / 30.0f);
    assert(count_type(log.events, GE_EVENT_BEGIN, GE_ACTION_POINTER_CLICK) == 0 &&
           "a hold unreadable past the budget must reset, not resume");
    ge_update(ge, hands, 1.0f / 30.0f);
    assert(count_type(log.events, GE_EVENT_BEGIN, GE_ACTION_POINTER_CLICK) == 1 &&
           "the pinch still fires after a fresh hold");

    ge_destroy(ge);
    std::printf("PASS test_indeterminate_pending_freeze_capped\n");
}

// (c) Rearm: after END, an unreadable trigger counts as released once the
// budget runs out, so a follow-up pinch is not swallowed by the disarm.
static int rearm_after_indeterminate(const char *toml) {
    ge_engine_t *ge = ge_create();
    assert(ge);
    if (toml) {
        std::string cfg = write_temp_toml(toml);
        assert(ge_load_config(ge, cfg.c_str()));
        std::remove(cfg.c_str());
    }
    EventLog log;
    ge_set_callback(ge, log_callback, &log);
    ge_hand_t hands[2]{};
    hands[0] = make_open_hand();
    hands[1].present = false;
    translate_hand(hands[0], 0.30f, 0.20f, 0.10f);
    pinch_to_active(ge, hands, log);
    // Hold until the 250 ms click cap ENDs it.
    for (int i = 0; i < 9; ++i) ge_update(ge, hands, 1.0f / 30.0f);
    assert(count_type(log.events, GE_EVENT_END, GE_ACTION_POINTER_CLICK) == 1);
    log.events.clear();

    // Index tip unreadable for 14 frames (467 ms): budget (300) + rearm
    // debounce (100) both elapse.
    ge_hand_t saved = hands[0];
    set_joint(hands[0], GE_JOINT_INDEX_TIP, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 14; ++i) ge_update(ge, hands, 1.0f / 30.0f);
    hands[0] = saved;

    // Second beat: the DOUBLE-TAP variant (50 ms hold, no convergence gate)
    // is the one a quick follow-up pinch fires.
    for (int i = 0; i < 5; ++i) ge_update(ge, hands, 1.0f / 30.0f);
    int begins = count_type(log.events, GE_EVENT_BEGIN, GE_ACTION_POINTER_CLICK);
    ge_destroy(ge);
    return begins;
}

static void test_indeterminate_rearm_capped() {
    assert(rearm_after_indeterminate(nullptr) == 1 &&
           "unreadable trigger past the budget must rearm the name");
    // Control: with a huge budget the artifact still counts as "held" and
    // the second beat stays disarmed.
    assert(rearm_after_indeterminate("[pinch_select]\nindeterminate_release_ms = 100000\n") == 0 &&
           "with the budget lifted the disarm must persist");
    std::printf("PASS test_indeterminate_rearm_capped\n");
}

// ---------------------------------------------------------------------------
// max_active_ms on pinch_right_click and fist_launcher: neither may stay
// Active forever when the release never comes.
// ---------------------------------------------------------------------------

static void test_right_click_max_active_ends() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    assert(ge_gesture_max_active_ms(ge, "pinch_right_click") == 250);
    EventLog log;
    ge_set_callback(ge, log_callback, &log);
    ge_hand_t hands[2]{};
    hands[0] = make_open_hand();
    hands[1].present = false;
    ge_update(ge, hands, 1.0f / 30.0f);
    hands[0].joints[GE_JOINT_INDEX_TIP][0]  = -0.02f;
    hands[0].joints[GE_JOINT_MIDDLE_TIP][0] =  0.00f;
    set_thumb_tip(hands[0], -0.01f, 0.13f, 0.0f);
    for (int i = 0; i < 20; ++i) ge_update(ge, hands, 1.0f / 30.0f);   // 667 ms held
    assert(count_type(log.events, GE_EVENT_BEGIN, GE_ACTION_POINTER_RIGHT_CLICK) == 1);
    assert(count_type(log.events, GE_EVENT_END, GE_ACTION_POINTER_RIGHT_CLICK) == 1 &&
           "held right-click pose must END at max_active_ms");
    ge_destroy(ge);
    std::printf("PASS test_right_click_max_active_ends\n");
}

static void test_fist_launcher_max_active_ends() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    assert(ge_gesture_max_active_ms(ge, "fist_launcher.per_frame") == 10000);
    assert(ge_gesture_max_active_ms(ge, "fist_launcher.windowed") == 10000);
    EventLog log;
    ge_set_callback(ge, log_callback, &log);
    ge_hand_t hands[2]{};
    hands[0] = make_open_hand();
    hands[1].present = false;
    curl_all_fingers(hands[0]);
    curl_finger(hands[0], GE_JOINT_THUMB_CMC, GE_JOINT_THUMB_MCP, GE_JOINT_THUMB_IP,
                GE_JOINT_THUMB_TIP);
    for (int i = 0; i < 300; ++i) ge_update(ge, hands, 1.0f / 30.0f);   // 10 s
    assert(count_type(log.events, GE_EVENT_BEGIN, GE_ACTION_TOGGLE_LAUNCHER) == 1);
    assert(count_type(log.events, GE_EVENT_END, GE_ACTION_TOGGLE_LAUNCHER) == 0 &&
           "fist held under the cap stays Active");
    for (int i = 0; i < 20; ++i) ge_update(ge, hands, 1.0f / 30.0f);
    assert(count_type(log.events, GE_EVENT_END, GE_ACTION_TOGGLE_LAUNCHER) == 1 &&
           "fist held past max_active_ms must END");
    ge_destroy(ge);
    std::printf("PASS test_fist_launcher_max_active_ends\n");
}

// ---------------------------------------------------------------------------
// Chirality: handedness from joint geometry, never from the wire slot.
// ---------------------------------------------------------------------------

// make_open_hand() is planar (every joint at z = 0), and a planar hand is its
// own mirror image — it carries no chirality at all.  Lift the thumb onto the
// palmar side to give it one.  The palm-plane normal
// (INDEX_MCP-WRIST) x (PINKY_MCP-WRIST) comes out along -Z for this fixture,
// so palmar is -Z; mirroring X then swaps the hand.
static ge_hand_t make_chiral_hand(bool left) {
    ge_hand_t h = make_open_hand();
    const int thumb[] = {GE_JOINT_THUMB_CMC, GE_JOINT_THUMB_MCP,
                         GE_JOINT_THUMB_IP,  GE_JOINT_THUMB_TIP};
    for (int j : thumb) h.joints[j][2] = -0.02f;
    if (left)
        for (int j = 0; j < GE_JOINT_COUNT; ++j) h.joints[j][0] = -h.joints[j][0];
    return h;
}

static void test_hand_chirality_from_geometry() {
    ge_hand_t right = make_chiral_hand(false);
    ge_hand_t leftie = make_chiral_hand(true);

    assert(ge_hand_chirality(&right) == GE_CHIRALITY_RIGHT);
    assert(ge_hand_chirality(&leftie) == GE_CHIRALITY_LEFT);
    assert(!ge_hand_is_left(&right));
    assert(ge_hand_is_left(&leftie));

    // Slot invariance — the whole point.  A right hand the tracker happens to
    // put in slot 0 (which ge_update's doc comment calls "left") must still
    // read RIGHT, and vice versa.
    for (int slot = 0; slot < 2; ++slot) {
        ge_hand_t hands[2]{};
        hands[slot] = right;
        hands[1 - slot] = leftie;
        assert(ge_hand_chirality(&hands[slot]) == GE_CHIRALITY_RIGHT);
        assert(ge_hand_chirality(&hands[1 - slot]) == GE_CHIRALITY_LEFT);
    }

    // A perfectly flat hand is mirror-symmetric, so the honest answer is
    // "cannot tell" — not a coin flip that would invert a scrub direction.
    ge_hand_t flat = make_open_hand();
    assert(ge_hand_chirality(&flat) == GE_CHIRALITY_UNKNOWN);
    assert(!ge_hand_is_left(&flat));

    ge_hand_t nulled{};
    assert(ge_hand_chirality(&nulled) == GE_CHIRALITY_UNKNOWN);
    assert(ge_hand_chirality(nullptr) == GE_CHIRALITY_UNKNOWN);

    std::printf("PASS test_hand_chirality_from_geometry\n");
}

// Guard against a well-meaning re-enable of the #if 0 grab_window block.
// Its trigger band (ti < 0.055, tm < 0.070, ring/pinky > 0.055) is satisfied
// CONTINUOUSLY by an idle open hand on every clip in the recording mirror
// (measured 2026-09-04: ti 0.051 / tm 0.062 / ring 0.067 / pinky 0.070), so
// compiling it in fires WINDOW_MOVE on a resting hand and starves
// pinch_select.  See the comment on the block in engine.cpp; the fix is
// `grab-threshold-tune`, a Track B in-person task.
static void test_grab_window_stays_uncompiled() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    assert(ge_gesture_min_hold_ms(ge, "grab_window") == -1);
    for (int i = 0; i < ge_gesture_count(ge); ++i)
        assert(std::strcmp(ge_gesture_name_at(ge, i), "grab_window") != 0);
    ge_destroy(ge);
    std::printf("PASS test_grab_window_stays_uncompiled\n");
}

int main() {
    std::printf("=== gesture-engine tests ===\n");
    test_hand_chirality_from_geometry();
    test_grab_window_stays_uncompiled();
    test_features_open_hand();
    test_features_pinch();
    test_features_fist();
    test_thumb_on_index_projection();
    test_thumb_on_middle_projection();
    test_fingertip_gather_radius();
    test_pinch_gesture_lifecycle();
    test_hysteresis();
    test_hand_lost_cancel();
    test_scroll_delta();
    test_get_feature();
    test_right_click_gesture();
    test_palm_normal();
    test_palm_down_features();
    test_keyboard_anchor_pending_progress_and_fire();
    test_keyboard_anchor_cancels_on_palm_flip();
    test_hand_input_slot_remap();
    test_zero_frame_artifact_features_indeterminate();
    test_zero_frame_artifact_no_spurious_right_click();
    test_zero_frame_artifact_preserves_pending_hold();
    test_noisy_pinch_hold_single_pair();
    test_fast_wave_no_triggers();
    test_near_threshold_oscillation_no_machine_gun();
    test_release_wider_than_trigger_for_every_variant();
    test_indeterminate_release_cancels_active();
    test_indeterminate_pending_freeze_capped();
    test_indeterminate_rearm_capped();
    test_right_click_max_active_ends();
    test_fist_launcher_max_active_ends();
    std::printf("=== ALL TESTS PASSED ===\n");
    return 0;
}
