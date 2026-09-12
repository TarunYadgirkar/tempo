// test_keyboard.cpp — virtual-keyboard integration at the scene level.
//
// Drives scene::tick with synthetic fingertip trajectories following the
// canonical plunge tap (kbd_tap_traj.h, the same conventions
// gen_synthetic_session.py --typing emits) and asserts the decoded keys land
// in the focused panel's input log. Also pins the keyboard plane placement:
// anchored to the nearest horizontal plane when one exists, the floating
// head-relative default otherwise.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "core/scene.h"

extern "C" {
#include "kbd_tap_traj.h"
#include "keyboard_geom.h"
}

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

constexpr float DT = 1.0f / 60.0f;

sb_pose_t identity_pose() {
    sb_pose_t p{};
    p.timestamp_ns = 1;
    p.rot[3] = 1.0f;
    p.tracking_quality = 1.0f;
    return p;
}

sb_plane_t horizontal_plane(uint8_t id, float cx, float cy, float cz,
                            float extent) {
    sb_plane_t p{};
    std::memset(p.uuid, id, 16);
    p.center[0] = cx;
    p.center[1] = cy;
    p.center[2] = cz;
    p.normal[1] = 1.0f;
    p.extent[0] = extent;
    p.extent[1] = extent;
    p.alignment = 0;
    return p;
}

struct plane_basis {
    float origin[3], right[3], down[3], normal[3];
};

bool parse_vec3(const std::string &json, const char *key, float out[3]) {
    std::string pat = std::string("\"") + key + "\":[";
    size_t at = json.find(pat);
    if (at == std::string::npos)
        return false;
    return std::sscanf(json.c_str() + at + pat.size(), "%f,%f,%f", &out[0],
                       &out[1], &out[2]) == 3;
}

bool parse_basis(const std::string &json, plane_basis &b) {
    return parse_vec3(json, "origin", b.origin) &&
           parse_vec3(json, "right", b.right) &&
           parse_vec3(json, "down", b.down) &&
           parse_vec3(json, "normal", b.normal);
}

void plane_to_world(const plane_basis &b, float x, float y, float z,
                    float out[3]) {
    for (int i = 0; i < 3; i++)
        out[i] = b.origin[i] + b.right[i] * x + b.down[i] * y +
                 b.normal[i] * z;
}

// Hand whose five fingertips sit at explicit plane-local positions; every
// other joint is placed far above the plane, spread out enough that no
// default gesture triggers (thumb far from index/middle, straight fingers).
sb_hand_t make_typing_hand(const plane_basis &b,
                           const float tips_plane[5][3]) {
    sb_hand_t h{};
    h.timestamp_ns = 1;
    h.hand_index = 0;
    h.joint_count = SB_HAND_JOINT_COUNT;
    const int tip_joints[5] = {SB_JOINT_THUMB_TIP, SB_JOINT_INDEX_TIP,
                               SB_JOINT_MIDDLE_TIP, SB_JOINT_RING_TIP,
                               SB_JOINT_PINKY_TIP};
    for (int t = 0; t < 5; t++) {
        float w[3];
        plane_to_world(b, tips_plane[t][0], tips_plane[t][1],
                       tips_plane[t][2], w);
        for (int i = 0; i < 3; i++)
            h.joints[tip_joints[t]][i] = w[i];
        h.joints[tip_joints[t]][3] = 1.0f;
    }
    // Chain joints behind each tip, stepped up and away with a wide spread so
    // curls read straight and pair distances stay large.
    struct {
        int mcp, pip, dip, tip;
        float spread;
    } chains[] = {
        {SB_JOINT_THUMB_CMC, SB_JOINT_THUMB_MCP, SB_JOINT_THUMB_IP,
         SB_JOINT_THUMB_TIP, -0.10f},
        {SB_JOINT_INDEX_MCP, SB_JOINT_INDEX_PIP, SB_JOINT_INDEX_DIP,
         SB_JOINT_INDEX_TIP, -0.03f},
        {SB_JOINT_MIDDLE_MCP, SB_JOINT_MIDDLE_PIP, SB_JOINT_MIDDLE_DIP,
         SB_JOINT_MIDDLE_TIP, 0.03f},
        {SB_JOINT_RING_MCP, SB_JOINT_RING_PIP, SB_JOINT_RING_DIP,
         SB_JOINT_RING_TIP, 0.09f},
        {SB_JOINT_PINKY_MCP, SB_JOINT_PINKY_PIP, SB_JOINT_PINKY_DIP,
         SB_JOINT_PINKY_TIP, 0.15f},
    };
    for (auto &c : chains) {
        const float *tip = h.joints[c.tip];
        const int backs[3] = {c.dip, c.pip, c.mcp};
        for (int k = 0; k < 3; k++) {
            float step = 0.03f * (float)(k + 1);
            h.joints[backs[k]][0] = tip[0] + c.spread * 0.2f * (float)(k + 1);
            h.joints[backs[k]][1] = tip[1] + step;
            h.joints[backs[k]][2] = tip[2] + step;
            h.joints[backs[k]][3] = 1.0f;
        }
    }
    for (int i = 0; i < 3; i++)
        h.joints[SB_JOINT_WRIST][i] =
            (h.joints[SB_JOINT_INDEX_MCP][i] +
             h.joints[SB_JOINT_PINKY_MCP][i]) *
                0.5f +
            (i == 1 ? 0.06f : 0.0f);
    h.joints[SB_JOINT_WRIST][3] = 1.0f;
    return h;
}

void park_tips(float tips[5][3]) {
    for (int t = 0; t < 5; t++) {
        tips[t][0] = KBD_TAP_PARK_X + 0.05f * (float)t;
        tips[t][1] = KBD_TAP_PARK_Y;
        tips[t][2] = KBD_TAP_PARK_Z;
    }
}

// One canonical plunge tap of the index finger over key `keysym`, plus a
// short parked tail so the decoder sees the lift complete.
void run_tap(scene &s, const plane_basis &b, uint32_t keysym) {
    int idx = kbd_geom_find_keysym(keysym);
    assert(idx >= 0);
    const kbd_key_geom_t *g = kbd_geom_get(idx);
    float tips[5][3];
    for (int f = 0; f < KBD_TAP_TOTAL_FRAMES; f++) {
        park_tips(tips);
        tips[1][0] = g->cx_m;
        tips[1][1] = g->cy_m;
        tips[1][2] = kbd_tap_traj_z(f);
        s.inject_hand(0, make_typing_hand(b, tips));
        s.tick(DT);
    }
    park_tips(tips);
    for (int f = 0; f < 14; f++) {
        s.inject_hand(0, make_typing_hand(b, tips));
        s.tick(DT);
    }
}

bool log_contains(const std::vector<std::string> &log,
                  const std::string &needle) {
    for (const auto &l : log)
        if (l == needle)
            return true;
    return false;
}

int log_count(const std::vector<std::string> &log, const std::string &needle) {
    int n = 0;
    for (const auto &l : log)
        if (l == needle)
            n++;
    return n;
}

// ---------------------------------------------------------------------------

void test_plane_anchoring() {
    scene s;
    s.inject_pose(identity_pose());
    sb_plane_t desk = horizontal_plane(0x11, 0.0f, -0.4f, -0.6f, 2.0f);
    s.inject_planes(&desk, 1);
    s.tick(DT);

    CHECK(s.keyboard_show());  // anchored
    std::string st = s.keyboard_status_json();
    CHECK(st.find("\"visible\":true") != std::string::npos);
    CHECK(st.find("\"anchored\":true") != std::string::npos);
    plane_basis b{};
    CHECK(parse_basis(st, b));
    // Head at the origin looking down at the desk: down points back at the
    // user (+Z), right is +X, and the keyboard centre sits 0.35 m out from
    // the head's plane projection.
    CHECK_NEAR(b.normal[1], 1.0f, 1e-3f);
    CHECK_NEAR(b.down[2], 1.0f, 1e-3f);
    CHECK_NEAR(b.right[0], 1.0f, 1e-3f);
    CHECK_NEAR(b.origin[0], -0.15f, 1e-3f);
    CHECK_NEAR(b.origin[1], -0.4f, 1e-3f);
    CHECK_NEAR(b.origin[2], -0.41f, 1e-3f);

    CHECK(s.keyboard_hide());
    st = s.keyboard_status_json();
    CHECK(st.find("\"visible\":false") != std::string::npos);
    std::printf("PASS test_plane_anchoring\n");
}

void test_floating_default() {
    scene s;
    s.inject_pose(identity_pose());
    s.tick(DT);
    CHECK(!s.keyboard_show());  // no planes → floating
    std::string st = s.keyboard_status_json();
    CHECK(st.find("\"anchored\":false") != std::string::npos);
    plane_basis b{};
    CHECK(parse_basis(st, b));
    CHECK_NEAR(b.origin[0], -0.15f, 1e-4f);
    CHECK_NEAR(b.origin[1], -0.20f, 1e-4f);
    CHECK_NEAR(b.origin[2], -0.50f, 1e-4f);
    std::printf("PASS test_floating_default\n");
}

// The floating default is laid out in the head's frame: yaw the head 90 deg and
// the whole plane swings with it instead of staying pinned to scene -Z.
void test_floating_default_follows_head_yaw() {
    scene s;
    s.inject_pose(identity_pose());  // origin captured looking down -Z

    sb_pose_t yawed = identity_pose();
    yawed.timestamp_ns = 2;
    const float half = 0.25f * (float)M_PI;  // 90 deg about +Y
    yawed.rot[1] = std::sin(half);
    yawed.rot[3] = std::cos(half);
    yawed.pos[0] = 0.5f;
    s.inject_pose(yawed);
    s.tick(DT);

    CHECK(!s.keyboard_show());  // still no planes -> floating
    plane_basis b{};
    CHECK(parse_basis(s.keyboard_status_json(), b));

    // Yawed 90 deg: the head looks along scene -X, so the space row ("down")
    // points back at the user along +X and the plane sits 0.5 m out that way.
    const float fwd_h[3] = {-1.0f, 0.0f, 0.0f};
    CHECK_NEAR(b.down[0], -fwd_h[0], 1e-3f);
    CHECK_NEAR(b.down[1], -fwd_h[1], 1e-3f);
    CHECK_NEAR(b.down[2], -fwd_h[2], 1e-3f);
    CHECK_NEAR(b.normal[1], 1.0f, 1e-3f);

    float head[3], q[4];
    CHECK(s.head_pose(head, q));
    float to_origin[3] = {b.origin[0] - head[0], b.origin[1] - head[1],
                          b.origin[2] - head[2]};
    CHECK(to_origin[0] * fwd_h[0] + to_origin[2] * fwd_h[2] > 0.0f);
    // head (0.5,0,0) + fwd_h*0.5 - up*0.2 - right*(width/2), right = (0,0,-1).
    CHECK_NEAR(b.origin[0], 0.0f, 1e-3f);
    CHECK_NEAR(b.origin[1], -0.20f, 1e-3f);
    CHECK_NEAR(b.origin[2], 0.15f, 1e-3f);
    std::printf("PASS test_floating_default_follows_head_yaw\n");
}

// A plane behind the user is never an anchor candidate, however close it is.
void test_anchor_prefers_plane_in_front() {
    scene s;
    s.inject_pose(identity_pose());  // head at the origin, looking down -Z
    sb_plane_t planes[2] = {
        horizontal_plane(0x21, 0.0f, -0.30f, 0.40f, 1.0f),   // behind, closer
        horizontal_plane(0x22, 0.0f, -0.70f, -1.20f, 1.0f),  // in front
    };
    s.inject_planes(planes, 2);
    s.tick(DT);

    CHECK(s.keyboard_show());
    plane_basis b{};
    CHECK(parse_basis(s.keyboard_status_json(), b));
    CHECK_NEAR(b.origin[1], -0.70f, 1e-3f);  // the plane in front won
    std::printf("PASS test_anchor_prefers_plane_in_front\n");
}

void test_typing_routes_to_focused_panel() {
    scene s;
    s.inject_pose(identity_pose());
    uint64_t h = s.spawn_panel("test-card", "typing-target");
    s.focus_panel(h);
    s.keyboard_show();
    plane_basis b{};
    CHECK(parse_basis(s.keyboard_status_json(), b));

    run_tap(s, b, 'h');
    run_tap(s, b, 'i');
    run_tap(s, b, KBD_KEYSYM_SPACE);
    run_tap(s, b, KBD_KEYSYM_RETURN);

    auto log = s.panel_input_log(h);
    CHECK(log_contains(log, "type:h"));
    CHECK(log_contains(log, "type:i"));
    CHECK(log_contains(log, "type: "));
    CHECK(log_contains(log, "key:Return"));
    CHECK(s.keyboard_status_json().find("\"last_keysym\":\"0xff0d\"") !=
          std::string::npos);
    std::printf("PASS test_typing_routes_to_focused_panel\n");
}

// Shift is a sticky one-shot: it capitalises exactly the next character and
// then clears itself, and never reaches the panel as a keystroke of its own.
void test_shift_types_uppercase() {
    scene s;
    s.inject_pose(identity_pose());
    uint64_t h = s.spawn_panel("test-card", "shift-target");
    s.focus_panel(h);
    s.keyboard_show();
    plane_basis b{};
    CHECK(parse_basis(s.keyboard_status_json(), b));

    run_tap(s, b, XK_Shift_L);
    CHECK(s.keyboard_status_json().find("\"shift\":\"once\"") !=
          std::string::npos);
    run_tap(s, b, 'h');
    run_tap(s, b, 'i');

    auto log = s.panel_input_log(h);
    CHECK(log_contains(log, "type:H"));
    CHECK(log_contains(log, "type:i"));  // one-shot cleared
    CHECK(!log_contains(log, "type:h"));
    CHECK(s.keyboard_status_json().find("\"shift\":\"off\"") !=
          std::string::npos);
    std::printf("PASS test_shift_types_uppercase\n");
}

// A second shift tap locks caps; a third releases it.
void test_shift_double_tap_caps_lock() {
    scene s;
    s.inject_pose(identity_pose());
    uint64_t h = s.spawn_panel("test-card", "caps-target");
    s.focus_panel(h);
    s.keyboard_show();
    plane_basis b{};
    CHECK(parse_basis(s.keyboard_status_json(), b));

    run_tap(s, b, XK_Shift_L);
    run_tap(s, b, XK_Shift_L);
    CHECK(s.keyboard_status_json().find("\"shift\":\"caps\"") !=
          std::string::npos);

    run_tap(s, b, 'h');
    run_tap(s, b, 'i');
    auto log = s.panel_input_log(h);
    CHECK(log_contains(log, "type:H"));
    CHECK(log_contains(log, "type:I"));

    run_tap(s, b, XK_Shift_L);
    CHECK(s.keyboard_status_json().find("\"shift\":\"off\"") !=
          std::string::npos);
    run_tap(s, b, 'a');
    CHECK(log_contains(s.panel_input_log(h), "type:a"));
    std::printf("PASS test_shift_double_tap_caps_lock\n");
}

// One tap on the layer key swaps the letter rows for digits and punctuation on
// the same rectangles; tapping it again restores the letters.
void test_layer_toggle_swaps_rows() {
    scene s;
    s.inject_pose(identity_pose());
    uint64_t h = s.spawn_panel("test-card", "layer-target");
    s.focus_panel(h);
    s.keyboard_show();
    plane_basis b{};
    CHECK(parse_basis(s.keyboard_status_json(), b));

    run_tap(s, b, XK_Mode_switch);
    CHECK(s.keyboard_status_json().find("\"layer\":\"symbols\"") !=
          std::string::npos);
    run_tap(s, b, 'q');  // the "q" rectangle now carries "1"
    run_tap(s, b, 'k');  // ...and the "k" rectangle carries "@"
    auto log = s.panel_input_log(h);
    CHECK(log_contains(log, "type:1"));
    CHECK(log_contains(log, "type:@"));
    CHECK(!log_contains(log, "type:q"));

    run_tap(s, b, XK_Mode_switch);
    CHECK(s.keyboard_status_json().find("\"layer\":\"letters\"") !=
          std::string::npos);
    run_tap(s, b, 'q');
    CHECK(log_contains(s.panel_input_log(h), "type:q"));
    std::printf("PASS test_layer_toggle_swaps_rows\n");
}

// The keycap texture must change when a modifier does — the labels are the
// only cue the user has for which alphabet the next tap produces.
void test_surface_redraws_on_layer_change() {
    scene s;
    s.inject_pose(identity_pose());
    s.keyboard_show();
    s.tick(DT);
    uint64_t before = s.snapshot_keyboard().version;

    plane_basis b{};
    CHECK(parse_basis(s.keyboard_status_json(), b));
    run_tap(s, b, XK_Mode_switch);
    s.tick(DT);
    CHECK(s.snapshot_keyboard().version != before);
    std::printf("PASS test_surface_redraws_on_layer_change\n");
}

void test_hold_auto_repeat() {
    scene s;
    s.inject_pose(identity_pose());
    uint64_t h = s.spawn_panel("test-card", "repeat-target");
    s.focus_panel(h);
    s.keyboard_show();
    plane_basis b{};
    CHECK(parse_basis(s.keyboard_status_json(), b));

    int idx = kbd_geom_find_keysym(KBD_KEYSYM_BACKSPACE);
    assert(idx >= 0);
    const kbd_key_geom_t *g = kbd_geom_get(idx);
    float tips[5][3];
    // Descend onto backspace, dwell at the plunge bottom for ~1.4 s
    // (hold_dwell 0.4 s + repeat_delay 0.3 s + several repeat periods), lift.
    for (int f = 0; f < KBD_TAP_DOWN_FRAMES; f++) {
        park_tips(tips);
        tips[1][0] = g->cx_m;
        tips[1][1] = g->cy_m;
        tips[1][2] = kbd_tap_traj_z(f);
        s.inject_hand(0, make_typing_hand(b, tips));
        s.tick(DT);
    }
    for (int f = 0; f < 84; f++) {
        park_tips(tips);
        tips[1][0] = g->cx_m;
        tips[1][1] = g->cy_m;
        tips[1][2] = KBD_TAP_Z_LOW;
        s.inject_hand(0, make_typing_hand(b, tips));
        s.tick(DT);
    }
    for (int f = KBD_TAP_DOWN_FRAMES; f < KBD_TAP_TOTAL_FRAMES; f++) {
        park_tips(tips);
        tips[1][0] = g->cx_m;
        tips[1][1] = g->cy_m;
        tips[1][2] = kbd_tap_traj_z(f);
        s.inject_hand(0, make_typing_hand(b, tips));
        s.tick(DT);
    }

    auto log = s.panel_input_log(h);
    int n = log_count(log, "key:BackSpace");
    if (n < 2)
        std::fprintf(stderr, "  hold produced %d BackSpace emits\n", n);
    CHECK(n >= 2);  // initial emit + at least one auto-repeat
    std::printf("PASS test_hold_auto_repeat (%d emits)\n", n);
}

// Open-hand fixture (gesture-engine test conventions): fingers along +Y,
// palm facing ±Z.
sb_hand_t make_open_hand(const float origin[3]) {
    sb_hand_t h{};
    h.timestamp_ns = 1;
    h.hand_index = 0;
    h.joint_count = SB_HAND_JOINT_COUNT;
    auto set = [&](int j, float x, float y, float z) {
        h.joints[j][0] = origin[0] + x;
        h.joints[j][1] = origin[1] + y;
        h.joints[j][2] = origin[2] + z;
        h.joints[j][3] = 1.0f;
    };
    set(SB_JOINT_WRIST, 0, 0, 0);
    const float tx = -0.07f;
    set(SB_JOINT_THUMB_CMC, tx, 0.02f, 0);
    set(SB_JOINT_THUMB_MCP, tx, 0.04f, 0);
    set(SB_JOINT_THUMB_IP, tx, 0.06f, 0);
    set(SB_JOINT_THUMB_TIP, tx, 0.08f, 0);
    struct {
        int mcp, pip, dip, tip;
        float x;
    } fingers[] = {
        {SB_JOINT_INDEX_MCP, SB_JOINT_INDEX_PIP, SB_JOINT_INDEX_DIP,
         SB_JOINT_INDEX_TIP, -0.04f},
        {SB_JOINT_MIDDLE_MCP, SB_JOINT_MIDDLE_PIP, SB_JOINT_MIDDLE_DIP,
         SB_JOINT_MIDDLE_TIP, 0.08f},
        {SB_JOINT_RING_MCP, SB_JOINT_RING_PIP, SB_JOINT_RING_DIP,
         SB_JOINT_RING_TIP, 0.14f},
        {SB_JOINT_PINKY_MCP, SB_JOINT_PINKY_PIP, SB_JOINT_PINKY_DIP,
         SB_JOINT_PINKY_TIP, 0.20f},
    };
    for (auto &f : fingers) {
        set(f.mcp, f.x, 0.04f, 0);
        set(f.pip, f.x, 0.07f, 0);
        set(f.dip, f.x, 0.10f, 0);
        set(f.tip, f.x, 0.13f, 0);
    }
    return h;
}

// Rotate the open hand ±90° about X so the palm faces straight down (which
// sign gives palm_normal_y < 0 depends on the engine's handedness convention,
// so the test tries both).
sb_hand_t make_palm_down_hand(const float origin[3], bool flip) {
    float zero[3] = {0, 0, 0};
    sb_hand_t h = make_open_hand(zero);
    for (int j = 0; j < SB_HAND_JOINT_COUNT; j++) {
        float y = h.joints[j][1], z = h.joints[j][2];
        if (!flip) {
            h.joints[j][1] = z;
            h.joints[j][2] = -y;
        } else {
            h.joints[j][1] = -z;
            h.joints[j][2] = y;
        }
        h.joints[j][0] += origin[0];
        h.joints[j][1] += origin[1];
        h.joints[j][2] += origin[2];
    }
    return h;
}

void test_palm_down_hold_summons() {
    const float origin[3] = {0.0f, -0.2f, -0.5f};
    bool summoned = false;
    for (int flip = 0; flip < 2 && !summoned; flip++) {
        scene s;
        s.inject_pose(identity_pose());
        bool arc_seen = false;
        // keyboard_anchor: 1500 ms hold → BEGIN toggles the keyboard.
        for (int f = 0; f < 150; f++) {
            s.inject_hand(0, make_palm_down_hand(origin, flip != 0));
            s.tick(DT);
            scene::hud_arc arcs[2];
            s.hud_arcs(arcs);
            if (arcs[0].active && arcs[0].progress > 0.2f)
                arc_seen = true;
            if (s.keyboard_visible())
                break;
        }
        if (s.keyboard_visible()) {
            summoned = true;
            CHECK(arc_seen);  // progress arc showed during the hold
        }
    }
    CHECK(summoned);
    std::printf("PASS test_palm_down_hold_summons\n");
}

// Hands resting flat ON the key field satisfy the palm-down summon pose, so
// an unguarded toggle dismisses the keyboard mid-typing. Only a palm held off
// the board may dismiss it.
void test_palm_over_board_does_not_dismiss() {
    scene s;
    s.inject_pose(identity_pose());
    s.keyboard_show();
    s.tick(DT);
    CHECK(s.keyboard_visible());

    const keyboard_plane &kp = s.snapshot_keyboard().plane;
    ge_event_t ev{};
    ev.type = GE_EVENT_BEGIN;
    ev.action = GE_ACTION_KEYBOARD_ANCHOR;
    ev.gesture_name = "keyboard_anchor";

    // Centre of the key field, 5 cm above it: a hand resting to type.
    for (int i = 0; i < 3; i++)
        ev.position[i] = kp.origin[i] +
                         kp.right[i] * (KBD_GEOM_WIDTH_M * 0.5f) +
                         kp.down[i] * (KBD_GEOM_HEIGHT_M * 0.5f) +
                         kp.normal[i] * 0.05f;
    s.inject_gesture(ev);
    CHECK(s.keyboard_visible());

    // Clear of the board along its own +x: the dismiss still works.
    for (int i = 0; i < 3; i++)
        ev.position[i] = kp.origin[i] + kp.right[i] * (KBD_GEOM_WIDTH_M + 0.2f);
    s.inject_gesture(ev);
    CHECK(!s.keyboard_visible());

    // ...and a palm over where the board used to be re-summons it, because the
    // guard only applies while the keyboard is up.
    for (int i = 0; i < 3; i++)
        ev.position[i] = kp.origin[i] +
                         kp.right[i] * (KBD_GEOM_WIDTH_M * 0.5f) +
                         kp.down[i] * (KBD_GEOM_HEIGHT_M * 0.5f);
    s.inject_gesture(ev);
    CHECK(s.keyboard_visible());
    std::printf("PASS test_palm_over_board_does_not_dismiss\n");
}

void test_keyboard_render_snapshot() {
    scene s;
    s.inject_pose(identity_pose());
    s.keyboard_show();
    s.tick(DT);
    auto rs = s.snapshot_keyboard();
    CHECK(rs.visible);
    CHECK(rs.surface_w > 0 && rs.surface_h > 0);
    CHECK(rs.rgba.size() ==
          (size_t)rs.surface_w * (size_t)rs.surface_h * 4);
    // Some key pixels must differ from the background.
    bool varied = false;
    for (size_t i = 4; i < rs.rgba.size(); i += 4)
        if (rs.rgba[i] != rs.rgba[0]) {
            varied = true;
            break;
        }
    CHECK(varied);
    s.keyboard_hide();
    CHECK(!s.snapshot_keyboard().visible);
    std::printf("PASS test_keyboard_render_snapshot\n");
}

}  // namespace

int main() {
    // Pin decoder/gesture/launcher configs to compiled defaults.
    setenv("XDG_CONFIG_HOME", "/nonexistent-spatula-kbd-test", 1);
    setenv("HOME", "/nonexistent-spatula-kbd-test", 1);

    test_plane_anchoring();
    test_floating_default();
    test_floating_default_follows_head_yaw();
    test_anchor_prefers_plane_in_front();
    test_typing_routes_to_focused_panel();
    test_shift_types_uppercase();
    test_shift_double_tap_caps_lock();
    test_layer_toggle_swaps_rows();
    test_surface_redraws_on_layer_change();
    test_hold_auto_repeat();
    test_palm_down_hold_summons();
    test_palm_over_board_does_not_dismiss();
    test_keyboard_render_snapshot();

    if (g_failures) {
        std::fprintf(stderr, "test_keyboard: %d FAILURES\n", g_failures);
        return 1;
    }
    std::printf("test_keyboard: all tests passed\n");
    return 0;
}
