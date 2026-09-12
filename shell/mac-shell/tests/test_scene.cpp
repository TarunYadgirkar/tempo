// test_scene.cpp — scene-core tests: fixed-dt tick, synthetic hands driving a
// pinch-focus through the real gesture engine, anchored panels tracking
// planes, and control operations.
//
// The synthetic hand fixture mirrors gesture-engine/tests/test_gestures.cpp
// (make_open_hand + thumb-to-index pinch) so the default pinch_select FSM
// fires exactly as it does in the engine's own suite.

#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>
#include <string>
#include <vector>

// Replaced globally so test_head_pose_ring can assert that the render-path
// pose lookup touches the heap zero times per frame.
static std::atomic<size_t> g_alloc_count{0};

void *operator new(size_t n) {
    g_alloc_count.fetch_add(1, std::memory_order_relaxed);
    void *p = std::malloc(n ? n : 1);
    if (!p)
        throw std::bad_alloc();
    return p;
}
void *operator new[](size_t n) { return ::operator new(n); }
void operator delete(void *p) noexcept { std::free(p); }
void operator delete[](void *p) noexcept { std::free(p); }
void operator delete(void *p, size_t) noexcept { std::free(p); }
void operator delete[](void *p, size_t) noexcept { std::free(p); }

#include "core/scene.h"
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

// ---------------------------------------------------------------------------
// fixtures
// ---------------------------------------------------------------------------

static sb_pose_t identity_pose() {
    sb_pose_t p{};
    p.timestamp_ns = 1;
    p.rot[3] = 1.0f;
    p.tracking_quality = 1.0f;
    return p;
}

// Open hand centred near `origin`, fingers along +Y — the gesture-engine test
// fixture translated to a world position.
static sb_hand_t make_open_hand(const float origin[3]) {
    sb_hand_t h{};
    h.timestamp_ns = 1;
    h.hand_index = 0;
    h.joint_count = SB_HAND_JOINT_COUNT;

    auto set = [&](int j, float x, float y, float z) {
        h.joints[j][0] = origin[0] + x;
        h.joints[j][1] = origin[1] + y;
        h.joints[j][2] = origin[2] + z;
        h.joints[j][3] = 1.0f;
        h.joints[j][4] = 0.0f;
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

// Pinch: thumb tip touching the index tip.
static sb_hand_t make_pinch_hand(const float origin[3]) {
    sb_hand_t h = make_open_hand(origin);
    h.joints[SB_JOINT_THUMB_TIP][0] = origin[0] - 0.04f;
    h.joints[SB_JOINT_THUMB_TIP][1] = origin[1] + 0.125f;
    h.joints[SB_JOINT_THUMB_TIP][2] = origin[2];
    return h;
}

static sb_plane_t make_plane(uint8_t id, float cx, float cy, float cz,
                             float nx, float ny, float nz, float w, float hgt,
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
    p.extent[1] = hgt;
    p.alignment = alignment;
    return p;
}

static void tick_n(scene &s, int n, float dt = 1.0f / 60.0f) {
    for (int i = 0; i < n; i++)
        s.tick(dt);
}

// ---------------------------------------------------------------------------
// tests
// ---------------------------------------------------------------------------

static void test_spawn_and_move() {
    scene s;
    s.inject_pose(identity_pose());

    uint64_t a = s.spawn_panel("test-card", "alpha");
    uint64_t b = s.spawn_panel("test-card", "beta");
    CHECK(a == 1 && b == 2);
    CHECK(s.panel_count() == 2);
    CHECK(s.focused_handle() == a);  // first panel takes focus

    float target[3] = {0.5f, 0.25f, -1.0f};
    CHECK(s.move_panel(a, target, false));
    float pos[3];
    CHECK(s.panel_pose(a, pos));
    CHECK_NEAR(pos[0], 0.5f, 1e-6f);
    CHECK_NEAR(pos[1], 0.25f, 1e-6f);
    CHECK_NEAR(pos[2], -1.0f, 1e-6f);

    // move-rel accumulates deterministically on the target.
    float d1[3] = {0.1f, 0.0f, 0.0f};
    float d2[3] = {0.1f, -0.05f, 0.0f};
    CHECK(s.move_panel(a, d1, true));
    CHECK(s.move_panel(a, d2, true));
    CHECK(s.panel_pose(a, pos));
    CHECK_NEAR(pos[0], 0.7f, 1e-6f);
    CHECK_NEAR(pos[1], 0.2f, 1e-6f);

    float bogus[3] = {0, 0, 0};
    CHECK(!s.move_panel(999, bogus, false));

    CHECK(s.close_panel(b));
    CHECK(s.panel_count() == 1);
}

static void test_pinch_focus() {
    scene s;
    s.inject_pose(identity_pose());

    uint64_t far_panel = s.spawn_panel("test-card", "far");
    uint64_t near_panel = s.spawn_panel("test-card", "near");
    // far stays at the spawn shelf (~1.2 m away); near sits right where the
    // pinch will happen.
    float near_pos[3] = {0.0f, 0.1f, 0.0f};
    CHECK(s.move_panel(near_panel, near_pos, false));
    CHECK(s.focused_handle() == far_panel);
    (void)s.drain_events();

    const float hand_origin[3] = {0.0f, 0.0f, 0.0f};

    // A few open-hand frames to settle features…
    for (int i = 0; i < 6; i++) {
        s.inject_hand(0, make_open_hand(hand_origin));
        s.tick(1.0f / 60.0f);
    }
    CHECK(s.focused_handle() == far_panel);  // no gesture yet

    // …then hold the pinch past the 80 ms min-hold.
    for (int i = 0; i < 15; i++) {
        s.inject_hand(0, make_pinch_hand(hand_origin));
        s.tick(1.0f / 60.0f);
    }

    CHECK(s.focused_handle() == near_panel);

    auto log = s.panel_input_log(near_panel);
    bool saw_click = false;
    for (const auto &l : log)
        if (l == "pinch-click")
            saw_click = true;
    CHECK(saw_click);

    auto events = s.drain_events();
    bool saw_focus = false, saw_gesture = false;
    for (const auto &e : events) {
        if (e == "event focus handle=" + std::to_string(near_panel))
            saw_focus = true;
        if (e.rfind("event gesture name=", 0) == 0)
            saw_gesture = true;
    }
    CHECK(saw_focus);
    CHECK(saw_gesture);

    // Release: open hand again → gesture ends without re-firing focus.
    for (int i = 0; i < 10; i++) {
        s.inject_hand(0, make_open_hand(hand_origin));
        s.tick(1.0f / 60.0f);
    }
    CHECK(s.focused_handle() == near_panel);
}

static void test_anchor_follows_plane() {
    scene s;
    s.inject_pose(identity_pose());

    sb_plane_t desk =
        make_plane(0x11, 0.0f, -0.8f, -1.0f, 0, 1, 0, 2.0f, 1.2f, 0);
    s.inject_planes(&desk, 1);

    uint64_t h = s.spawn_panel("test-card", "anchored");
    float drop[3] = {0.3f, -0.75f, -1.1f};
    CHECK(s.move_panel(h, drop, false));

    uint8_t out_uuid[16];
    CHECK(s.anchor_panel(h, /*closest-horizontal*/ 2, nullptr, out_uuid) == 1);
    CHECK(out_uuid[0] == 0x11);

    tick_n(s, 2);
    float pos[3];
    CHECK(s.panel_pose(h, pos));
    // centre + in-plane offset, hovered 0.02 m above the desk.
    CHECK_NEAR(pos[0], 0.3f, 1e-4f);
    CHECK_NEAR(pos[1], -0.78f, 1e-4f);
    CHECK_NEAR(pos[2], -1.1f, 1e-4f);

    // ARKit refines the plane → the anchored panel follows it.
    desk.center[1] = -0.9f;
    s.inject_planes(&desk, 1);
    tick_n(s, 2);
    CHECK(s.panel_pose(h, pos));
    CHECK_NEAR(pos[1], -0.88f, 1e-4f);

    // query/clear round trip.
    CHECK(s.query_anchor(h, out_uuid) == 1);
    CHECK(s.clear_anchor(h));
    CHECK(s.query_anchor(h, out_uuid) == 0);

    // Anchor by explicit uuid: zero offset → panel sits at plane centre.
    CHECK(s.anchor_panel(h, 0, desk.uuid, out_uuid) == 1);
    tick_n(s, 2);
    CHECK(s.panel_pose(h, pos));
    CHECK_NEAR(pos[0], 0.0f, 1e-4f);
    CHECK_NEAR(pos[1], -0.88f, 1e-4f);
    CHECK_NEAR(pos[2], -1.0f, 1e-4f);

    // Mid-air panel: closest-horizontal finds nothing within 0.08 m.
    uint64_t h2 = s.spawn_panel("test-card", "midair");
    float mid[3] = {0.0f, 0.5f, -1.0f};
    CHECK(s.move_panel(h2, mid, false));
    CHECK(s.anchor_panel(h2, 2, nullptr, out_uuid) == 0);
}

static void test_nan_plane_keeps_finite_pose() {
    scene s;
    s.inject_pose(identity_pose());
    sb_plane_t desk =
        make_plane(0x33, 0.0f, -0.8f, -1.0f, 0, 1, 0, 2.0f, 1.2f, 0);
    s.inject_planes(&desk, 1);

    uint64_t h = s.spawn_panel("test-card", "anchored");
    float drop[3] = {0.3f, -0.75f, -1.1f};
    CHECK(s.move_panel(h, drop, false));
    uint8_t out_uuid[16];
    CHECK(s.anchor_panel(h, 2, nullptr, out_uuid) == 1);
    tick_n(s, 2);
    float before[3];
    CHECK(s.panel_pose(h, before));

    const float nan = std::numeric_limits<float>::quiet_NaN();
    // Corrupt refinements of the anchored plane: NaN normal, then NaN
    // centre, then infinite extent. Each is dropped at ingestion and the
    // panel keeps its last finite pose.
    sb_plane_t bad = desk;
    bad.normal[1] = nan;
    s.inject_planes(&bad, 1);
    tick_n(s, 2);
    bad = desk;
    bad.center[2] = nan;
    s.inject_planes(&bad, 1);
    tick_n(s, 2);
    bad = desk;
    bad.extent[0] = std::numeric_limits<float>::infinity();
    s.inject_planes(&bad, 1);
    tick_n(s, 2);

    float after[3];
    CHECK(s.panel_pose(h, after));
    for (int i = 0; i < 3; i++) {
        CHECK(std::isfinite(after[i]));
        CHECK_NEAR(after[i], before[i], 1e-5f);
    }
    // A NaN plane never appears in the plane list either.
    CHECK(s.planes_json().find("nan") == std::string::npos);

    // A hand with a NaN joint is ignored rather than adopted.
    float origin[3] = {0.0f, -0.2f, -0.6f};
    sb_hand_t hand = make_open_hand(origin);
    hand.joints[SB_JOINT_WRIST][0] = nan;
    s.inject_hand(0, hand);
    sb_hand_t got;
    CHECK(!s.hand_joints(0, got));

    // Normalising a NaN vector yields a finite unit vector.
    float v[3] = {nan, 0.0f, 0.0f};
    v3normalize(v);
    CHECK(std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]));
    CHECK_NEAR(v3length(v), 1.0f, 1e-6f);
}

static void test_panel_cap() {
    scene s;
    for (size_t i = 0; i < MAX_PANELS; i++)
        CHECK(s.spawn_panel("test-card", "p") != 0);
    CHECK(s.panel_count() == MAX_PANELS);
    CHECK(s.spawn_panel("test-card", "overflow") == 0);
    CHECK(s.spawn_captured_panel("app", "overflow", 1, 100, 100) == 0);
    CHECK(s.panel_count() == MAX_PANELS);
    CHECK(s.close_panel(1));
    CHECK(s.spawn_panel("test-card", "fits-again") != 0);
}

static void test_input_routing() {
    scene s;
    s.inject_pose(identity_pose());

    // No focus → type refused (mirrors control.c no_focus guard).
    CHECK(!s.type_text("nope"));

    uint64_t h = s.spawn_panel("test-card", "typing");
    CHECK(s.focus_panel(h));
    CHECK(s.type_text("hello world"));
    CHECK(s.key_press("Return"));
    CHECK(s.click_panel(h, 10.0f, 20.0f, 0));
    CHECK(s.scroll(0.0f, -3.0f));

    auto log = s.panel_input_log(h);
    CHECK(log.size() == 4);
    CHECK(log[0] == "type:hello world");
    CHECK(log[1] == "key:Return");
    CHECK(log[2] == "click:10,20 btn=0");
    CHECK(log[3] == "scroll:0,-3");

    // The test-card surface re-renders with the log tail on tick.
    tick_n(s, 1);
    auto panels = s.snapshot_render_panels();
    CHECK(panels.size() == 1);
    CHECK(panels[0].width_px == 512 && panels[0].height_px == 384);
    bool any_nonzero = false;
    for (size_t i = 0; i < panels[0].rgba.size(); i += 4)
        if (panels[0].rgba[i] != 0) {
            any_nonzero = true;
            break;
        }
    CHECK(any_nonzero);

    CHECK(s.resize_panel(h, 640, 480));
    tick_n(s, 1);
    panels = s.snapshot_render_panels();
    CHECK(panels[0].width_px == 640 && panels[0].height_px == 480);
}

// ---------------------------------------------------------------------------
// per-panel surface versions: one panel's repaint must not invalidate every
// other panel's cached GPU texture, and a matching version must skip the
// multi-megabyte RGBA copy entirely
// ---------------------------------------------------------------------------

static void test_per_panel_surface_version() {
    scene s;
    uint64_t a = s.spawn_panel("test-card", "a");
    uint64_t b = s.spawn_panel("test-card", "b");
    tick_n(s, 1);

    auto version_of = [](const std::vector<scene::render_panel> &v,
                         uint64_t h) -> uint64_t {
        for (const auto &rp : v)
            if (rp.handle == h)
                return rp.surface_version;
        return 0;
    };
    auto panels = s.snapshot_render_panels();
    CHECK(panels.size() == 2);
    const uint64_t va = version_of(panels, a);
    const uint64_t vb = version_of(panels, b);
    CHECK(va > 0 && vb > 0);

    // type_text dirties only the focused panel's surface.
    uint64_t hit = 0;
    for (const auto &rp : panels)
        if (rp.focused)
            hit = rp.handle;
    CHECK(hit == a || hit == b);
    const uint64_t other = hit == a ? b : a;
    const uint64_t v_other = hit == a ? vb : va;
    CHECK(s.type_text("hello"));
    tick_n(s, 1);
    panels = s.snapshot_render_panels();
    CHECK(version_of(panels, hit) > (hit == a ? va : vb));
    CHECK(version_of(panels, other) == v_other);

    // Caller already holding the current versions gets no pixels back.
    const auto cached = panels;
    panels = s.snapshot_render_panels([&](uint64_t h) -> uint64_t {
        return version_of(cached, h);
    });
    for (const auto &rp : panels)
        CHECK(rp.rgba.empty());

    // Stale (or absent) cache still gets the full copy.
    panels = s.snapshot_render_panels([](uint64_t) -> uint64_t { return 0; });
    for (const auto &rp : panels)
        CHECK(rp.rgba.size() ==
              (size_t)rp.width_px * (size_t)rp.height_px * 4);
    std::printf("PASS test_per_panel_surface_version\n");
}

static void test_world_origin_capture() {
    scene s;
    // First pose away from the origin: captured as world origin, so the
    // scene-frame head lands at (0,0,0).
    sb_pose_t p = identity_pose();
    p.pos[0] = 1.0f;
    p.pos[1] = 1.5f;
    p.pos[2] = 0.5f;
    s.inject_pose(p);

    float head[3], quat[4];
    CHECK(s.head_pose(head, quat));
    CHECK_NEAR(head[0], 0.0f, 1e-5f);
    CHECK_NEAR(head[1], 0.0f, 1e-5f);
    CHECK_NEAR(head[2], 0.0f, 1e-5f);

    // Later poses are relative to that origin.
    p.pos[0] = 1.2f;
    s.inject_pose(p);
    CHECK(s.head_pose(head, quat));
    CHECK_NEAR(head[0], 0.2f, 1e-5f);

    // Raw-frame planes land in the scene frame for anchoring: a desk 0.8 m
    // below the captured origin behaves like scene y = -0.8.
    sb_plane_t desk =
        make_plane(0x22, 1.0f, 0.7f, -0.5f, 0, 1, 0, 2.0f, 1.2f, 0);
    s.inject_planes(&desk, 1);
    uint64_t h = s.spawn_panel("test-card", "origin");
    float drop[3] = {0.0f, -0.75f, -1.0f};
    CHECK(s.move_panel(h, drop, false));
    uint8_t out_uuid[16];
    CHECK(s.anchor_panel(h, 2, nullptr, out_uuid) == 1);
    tick_n(s, 1);
    float pos[3];
    CHECK(s.panel_pose(h, pos));
    CHECK_NEAR(pos[1], -0.78f, 1e-4f);  // (0.7 - 1.5) + 0.02 hover
}

// An ARKit world re-init (session_epoch bump) must drop the captured origin,
// invalidate every anchor, and re-anchor in the NEW frame instead of drifting.
static void test_session_epoch_reanchor() {
    scene s;
    sb_pose_t p = identity_pose();
    p.session_epoch = 1;
    s.inject_pose(p);

    sb_plane_t desk =
        make_plane(0x33, 0.0f, -0.8f, -1.0f, 0, 1, 0, 2.0f, 1.2f, 0);
    s.inject_planes(&desk, 1);

    uint64_t h = s.spawn_panel("test-card", "anchored");
    float drop[3] = {0.0f, -0.75f, -1.0f};
    CHECK(s.move_panel(h, drop, false));
    uint8_t out_uuid[16];
    CHECK(s.anchor_panel(h, 2, nullptr, out_uuid) == 1);
    CHECK(s.query_anchor(h, out_uuid) == 1);
    s.drain_events();

    // ARKit resets: same head pose, new world frame, and the origin it hands
    // us is 1 m away from the old one.
    p.session_epoch = 2;
    p.pos[0] = 1.0f;
    s.inject_pose(p);

    // The anchor is invalidated immediately, before any re-snap can run.
    CHECK(s.query_anchor(h, out_uuid) == 0);

    bool announced = false;
    for (const auto &line : s.drain_events())
        if (line == "event tracking-reset epoch=2")
            announced = true;
    CHECK(announced);
    bool toasted = false;
    for (const auto &t : s.snapshot_toasts())
        if (t.text == "Tracking reset — re-anchoring panels")
            toasted = true;
    CHECK(toasted);

    // The new origin is re-captured from the resetting pose. Once ARKit
    // restates the desk, the panel re-anchors to it — keeping its
    // head-relative placement, but now bound in the NEW frame.
    s.inject_planes(&desk, 1);
    tick_n(s, 2);
    CHECK(s.query_anchor(h, out_uuid) == 1);
    CHECK(out_uuid[0] == 0x33);
    float pos[3];
    CHECK(s.panel_pose(h, pos));
    CHECK_NEAR(pos[1], -0.78f, 1e-4f);

    // The binding is live in the new frame: refining the plane moves the
    // panel with it. Before the fix the panel kept a transform derived from
    // the dead origin and would have drifted by the 1 m origin step instead.
    desk.center[1] = -0.9f;
    s.inject_planes(&desk, 1);
    tick_n(s, 2);
    CHECK(s.panel_pose(h, pos));
    CHECK_NEAR(pos[1], -0.88f, 1e-4f);

    // Epoch 0 is "unknown", never a reset: a pre-epoch sender must not keep
    // tearing anchors down.
    s.drain_events();
    p.session_epoch = 0;
    s.inject_pose(p);
    tick_n(s, 1);
    CHECK(s.query_anchor(h, out_uuid) == 1);
    for (const auto &line : s.drain_events())
        CHECK(line.rfind("event tracking-reset", 0) != 0);
}

// A hole in the pose stream means the phone restarted its ARKit session, so the
// next fresh pose re-captures the world origin instead of teleporting the world.
static void test_origin_recapture_after_pose_gap() {
    scene s;
    sb_pose_t p = identity_pose();
    p.pos[0] = 1.0f;
    s.inject_pose(p);
    uint64_t h = s.spawn_panel("test-card", "gap");
    tick_n(s, 6);

    p.timestamp_ns = 2;
    p.pos[0] = 1.1f;
    s.inject_pose(p);
    float head[3], quat[4];
    CHECK(s.head_pose(head, quat));
    CHECK_NEAR(head[0], 0.1f, 1e-5f);

    // Stream drops for 3 s, then returns from somewhere else entirely.
    tick_n(s, 1, 3.0f);
    p.timestamp_ns = 3;
    p.pos[0] = 5.0f;
    s.inject_pose(p);
    CHECK(s.head_pose(head, quat));
    CHECK_NEAR(head[0], 0.0f, 1e-5f);  // origin moved to the post-gap pose

    // The new origin sticks: later poses are relative to it, not the old one.
    p.timestamp_ns = 4;
    p.pos[0] = 5.25f;
    s.inject_pose(p);
    CHECK(s.head_pose(head, quat));
    CHECK_NEAR(head[0], 0.25f, 1e-5f);

    // Re-serving the same (stale) pose over a long stall must NOT re-capture.
    tick_n(s, 1, 3.0f);
    s.inject_pose(p);
    CHECK(s.head_pose(head, quat));
    CHECK_NEAR(head[0], 0.25f, 1e-5f);

    float pos[3];
    CHECK(s.panel_pose(h, pos));
}

// ---------------------------------------------------------------------------
// pose/frame time alignment: the renderer draws the view from the pose that
// was current when the displayed passthrough frame was captured, so the ring
// must resolve an arbitrary frame timestamp, interpolate between samples, and
// fall back to the newest pose when the timestamp predates the ring
// ---------------------------------------------------------------------------

static void test_head_pose_ring() {
    scene s;
    const uint64_t base = 1'000'000'000ull;  // 1 s, so "before the ring" exists
    const uint64_t step_ns = 10'000'000ull;  // 10 ms between poses

    // t = 0..100 ms: +1 cm along X and +0.02 rad of yaw per step.
    for (int i = 0; i <= 10; i++) {
        sb_pose_t p = identity_pose();
        p.timestamp_ns = base + (uint64_t)i * step_ns;
        p.pos[0] = 0.01f * (float)i;
        const float yaw = 0.02f * (float)i;
        p.rot[0] = 0.0f;
        p.rot[1] = std::sin(yaw * 0.5f);
        p.rot[2] = 0.0f;
        p.rot[3] = std::cos(yaw * 0.5f);
        s.inject_pose(p);
    }

    float pos[3], quat[4], lag = -1.0f;

    // Exactly on a sample: that sample, and a lag of (100 - 40) ms.
    CHECK(s.head_pose_at(base + 4 * step_ns, pos, quat, &lag));
    CHECK_NEAR(pos[0], 0.04f, 1e-6f);
    CHECK_NEAR(quat[1], std::sin(0.08f * 0.5f), 1e-6f);
    CHECK_NEAR(lag, 60.0f, 1e-3f);

    // Between two samples: lerp'd position, slerp'd rotation.
    CHECK(s.head_pose_at(base + 4 * step_ns + step_ns / 2, pos, quat, &lag));
    CHECK_NEAR(pos[0], 0.045f, 1e-6f);
    CHECK_NEAR(quat[1], std::sin(0.09f * 0.5f), 1e-6f);
    CHECK_NEAR(lag, 55.0f, 1e-3f);

    // Older than the ring: the newest pose, with no claimed lag.
    CHECK(s.head_pose_at(base - 5 * step_ns, pos, quat, &lag));
    CHECK_NEAR(pos[0], 0.10f, 1e-6f);
    CHECK_NEAR(lag, 0.0f, 1e-6f);

    // Ahead of the newest pose (a frame stamped after the last pose packet)
    // also resolves to the newest pose rather than extrapolating.
    CHECK(s.head_pose_at(base + 50 * step_ns, pos, quat, &lag));
    CHECK_NEAR(pos[0], 0.10f, 1e-6f);

    // The ring is fixed size: past HEAD_POSE_RING samples the oldest are gone,
    // so their timestamps fall back to the newest pose instead of growing the
    // ring. 10 + 25 further poses evicts everything up to t = 50 ms.
    for (int i = 11; i <= 35; i++) {
        sb_pose_t p = identity_pose();
        p.timestamp_ns = base + (uint64_t)i * step_ns;
        p.pos[0] = 0.01f * (float)i;
        s.inject_pose(p);
    }
    CHECK(s.head_pose_at(base, pos, quat, &lag));
    CHECK_NEAR(pos[0], 0.35f, 1e-6f);  // evicted → newest
    CHECK(s.head_pose_at(base + 30 * step_ns, pos, quat, &lag));
    CHECK_NEAR(pos[0], 0.30f, 1e-6f);  // still resident

    // The render path calls this once per frame and must not touch the heap.
    const size_t before = g_alloc_count.load();
    for (int i = 0; i < 240; i++)
        s.head_pose_at(base + 30 * step_ns + (uint64_t)i * 1000ull, pos, quat,
                       &lag);
    CHECK(g_alloc_count.load() == before);

    // No renderer in a headless run, so nothing has published an offset yet.
    CHECK(s.view_lag_ms() < 0.0f);
    s.set_view_lag_ms(12.5f);
    CHECK_NEAR(s.view_lag_ms(), 12.5f, 1e-6f);
}

// A tracking reset invalidates the SCENE frame every ring sample was expressed
// in, so the ring must empty rather than interpolate across the discontinuity.
static void test_head_pose_ring_dropped_on_reset() {
    scene s;
    sb_pose_t p = identity_pose();
    p.session_epoch = 1;
    p.timestamp_ns = 1'000'000'000ull;
    p.pos[0] = 1.0f;
    s.inject_pose(p);
    p.timestamp_ns += 10'000'000ull;
    p.pos[0] = 1.1f;
    s.inject_pose(p);

    float pos[3], quat[4];
    CHECK(s.head_pose_at(p.timestamp_ns, pos, quat));
    CHECK_NEAR(pos[0], 0.1f, 1e-5f);

    // ARKit re-initialised: the old samples are unusable, and the first pose
    // of the new frame re-seeds the ring.
    p.session_epoch = 2;
    p.timestamp_ns += 10'000'000ull;
    p.pos[0] = 7.0f;
    s.inject_pose(p);
    CHECK(s.head_pose_at(p.timestamp_ns, pos, quat));
    CHECK_NEAR(pos[0], 0.0f, 1e-5f);
    // The pre-reset timestamp is gone from the ring, so it lands on the newest.
    CHECK(s.head_pose_at(1'000'000'000ull, pos, quat));
    CHECK_NEAR(pos[0], 0.0f, 1e-5f);
}

// ---------------------------------------------------------------------------
// grab feel: deadband before motion, then the panel tracks the hand target
// directly and never overshoots it — smoothing is the renderer's single move
// spring, not a second scene-side stage (docs/mac-shell-design.md)
// ---------------------------------------------------------------------------

static ge_event_t grab_event(ge_event_type_t type, float dx = 0.0f,
                             float dy = 0.0f, float dz = 0.0f) {
    ge_event_t ev{};
    ev.type = type;
    ev.action = GE_ACTION_WINDOW_MOVE;
    ev.gesture_name = "grab_window";
    ev.delta[0] = dx;
    ev.delta[1] = dy;
    ev.delta[2] = dz;
    return ev;
}

static void test_grab_deadband_and_spring() {
    scene s;
    s.inject_pose(identity_pose());
    uint64_t h = s.spawn_panel("test-card", "grabme");
    float start[3] = {0.1f, 0.0f, -1.0f};
    CHECK(s.move_panel(h, start, false));

    // Micro-jitter under the 20 mm deadband: the panel must not move.
    s.inject_gesture(grab_event(GE_EVENT_BEGIN));
    for (int i = 0; i < 12; i++) {
        float d = (i % 2) ? 0.004f : -0.004f;  // ±4 mm hand wobble
        s.inject_gesture(grab_event(GE_EVENT_UPDATE, d, 0.0f, 0.0f));
        s.tick(1.0f / 60.0f);
    }
    float pos[3];
    CHECK(s.panel_pose(h, pos));
    CHECK_NEAR(pos[0], start[0], 1e-4f);
    CHECK_NEAR(pos[1], start[1], 1e-4f);

    // Real drag: 8 cm to the right in 10 mm steps.
    for (int i = 0; i < 8; i++) {
        s.inject_gesture(grab_event(GE_EVENT_UPDATE, 0.01f, 0.0f, 0.0f));
        s.tick(1.0f / 60.0f);
    }
    // Deadband consumed 20 mm: net target = start + 0.06.
    const float target_x = start[0] + 0.06f;
    CHECK(s.panel_pose(h, pos));
    CHECK(pos[0] > start[0] + 1e-4f);
    CHECK(pos[0] < target_x + 5e-3f);
    // The hand stopped: the panel sits on the target and stays there — no
    // second stage left running, so nothing to overshoot.
    float max_x = pos[0];
    for (int i = 0; i < 90; i++) {  // 1.5 s
        s.tick(1.0f / 60.0f);
        CHECK(s.panel_pose(h, pos));
        if (pos[0] > max_x)
            max_x = pos[0];
    }
    CHECK_NEAR(pos[0], target_x, 1e-3f);
    CHECK(max_x < target_x + 5e-3f);

    s.inject_gesture(grab_event(GE_EVENT_END));
    CHECK(s.panel_pose(h, pos));
    CHECK_NEAR(pos[0], target_x, 1e-3f);
}

// ---------------------------------------------------------------------------
// pinch-focus must not select a panel behind the user / off the pinch ray
// ---------------------------------------------------------------------------

static void test_pinch_focus_not_behind() {
    scene s;
    s.inject_pose(identity_pose());

    // One panel, moved BEHIND the user relative to the pinch direction and
    // outside the direct-touch radius.  Old nearest-within-2 m logic would
    // have focused it; ray gating must not.
    uint64_t back = s.spawn_panel("test-card", "behind");
    float back_pos[3] = {0.0f, -0.6f, 0.4f};
    CHECK(s.move_panel(back, back_pos, false));
    s.focus_panel(back);
    // Clear focus bookkeeping: spawn a second panel far along the ray is
    // not needed — we just assert no *re*-focus event fires.
    (void)s.drain_events();

    const float hand_origin[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 6; i++) {
        s.inject_hand(0, make_open_hand(hand_origin));
        s.tick(1.0f / 60.0f);
    }
    for (int i = 0; i < 15; i++) {
        s.inject_hand(0, make_pinch_hand(hand_origin));
        s.tick(1.0f / 60.0f);
    }

    // The pinch fired (gesture event logged) but no click reached the
    // behind-the-user panel.
    auto log = s.panel_input_log(back);
    for (const auto &l : log)
        CHECK(l != "pinch-click");
}

// ---------------------------------------------------------------------------
// pinch → click injection
// ---------------------------------------------------------------------------

// make_pinch_hand's pinch midpoint (thumb tip + index tip) / 2 relative to the
// hand origin — the point the gesture engine reports in POINTER_CLICK events.
static const float PINCH_MIDPOINT_OFFSET[3] = {-0.04f, 0.1275f, 0.0f};

static void hand_origin_for_pinch(const float pinch_at[3], float out[3]) {
    for (int i = 0; i < 3; i++)
        out[i] = pinch_at[i] - PINCH_MIDPOINT_OFFSET[i];
}

// Panel with a known quad: 800x600 px at the default 0.6 m width (0.45 m
// tall), yaw 0 (right = +X, up = +Y, normal = +Z) so plane-local x/y are
// scene x/y. handle 1 spawns with no sideways step, so its yaw stays 0.
static uint64_t spawn_flat_panel(scene &s, const float pos[3]) {
    uint64_t h = s.spawn_captured_panel("app:test", "flat", 4242, 800, 600);
    CHECK(h == 1);
    CHECK(s.move_panel(h, pos, false));
    s.tick(1.0f / 60.0f);
    float m[16];
    CHECK(s.panel_matrix(h, m));
    CHECK_NEAR(m[0], 1.0f, 1e-5f);   // right = +X
    CHECK_NEAR(m[10], 1.0f, 1e-5f);  // normal = +Z
    return h;
}

struct captured_click {
    int count = 0;
    uint64_t handle = 0;
    int32_t pid = 0;
    float x = 0, y = 0;
    int button = -1;
};

static void install_click_capture(scene &s, captured_click &out) {
    s.set_input_injector(
        [&out](uint64_t handle, int32_t pid, const inject_event &ev) {
            if (ev.type != inject_event::kind::click)
                return;
            out.count++;
            out.handle = handle;
            out.pid = pid;
            out.x = ev.x;
            out.y = ev.y;
            out.button = ev.button;
        });
}

static void pinch_at(scene &s, const float point[3]) {
    float origin[3];
    hand_origin_for_pinch(point, origin);
    for (int i = 0; i < 6; i++) {
        s.inject_hand(0, make_open_hand(origin));
        s.tick(1.0f / 60.0f);
    }
    for (int i = 0; i < 15; i++) {
        s.inject_hand(0, make_pinch_hand(origin));
        s.tick(1.0f / 60.0f);
    }
}

// Direct touch: the pinch point sits on the panel plane, 0.15 m right of
// centre → 3/4 across an 800 px surface, vertically centred (Y down).
static void test_pinch_click_direct_touch() {
    scene s;
    s.inject_pose(identity_pose());
    const float panel_pos[3] = {0.0f, 0.0f, -0.5f};
    uint64_t h = spawn_flat_panel(s, panel_pos);
    captured_click click;
    install_click_capture(s, click);

    const float point[3] = {0.15f, 0.0f, -0.5f};
    pinch_at(s, point);

    CHECK(s.focused_handle() == h);
    CHECK(click.count == 1);
    CHECK(click.handle == h);
    CHECK(click.pid == 4242);
    CHECK(click.button == 0);
    CHECK_NEAR(click.x, 600.0f, 1.0f);
    CHECK_NEAR(click.y, 300.0f, 1.0f);
}

// Ray pinch: the hand is nowhere near the panel, so the head->pinch ray is
// what picks the pixel. Head at the origin, pinch at (0.05, 0, -0.30), panel
// plane at z = -0.9 → the ray lands at plane-local x = +0.15.
static void test_pinch_click_ray() {
    scene s;
    s.inject_pose(identity_pose());
    const float panel_pos[3] = {0.0f, 0.0f, -0.9f};
    uint64_t h = spawn_flat_panel(s, panel_pos);
    captured_click click;
    install_click_capture(s, click);

    const float point[3] = {0.05f, 0.0f, -0.30f};
    pinch_at(s, point);

    CHECK(s.focused_handle() == h);
    CHECK(click.count == 1);
    CHECK(click.button == 0);
    CHECK_NEAR(click.x, 600.0f, 2.0f);
    CHECK_NEAR(click.y, 300.0f, 2.0f);
}

// Inside the direct-touch radius but off the quad (the panel is 0.45 m tall,
// so +0.30 m clears its top edge): focus only, no click injected.
static void test_pinch_outside_quad_focuses_only() {
    scene s;
    s.inject_pose(identity_pose());
    const float panel_pos[3] = {0.0f, 0.0f, -0.5f};
    uint64_t h = spawn_flat_panel(s, panel_pos);
    s.focus_panel(0);
    captured_click click;
    install_click_capture(s, click);

    const float point[3] = {0.0f, 0.30f, -0.5f};
    pinch_at(s, point);

    CHECK(s.focused_handle() == h);
    CHECK(click.count == 0);
    bool saw_pinch = false;
    for (const auto &l : s.panel_input_log(h))
        if (l == "pinch-click")
            saw_pinch = true;
    CHECK(saw_pinch);
}

// The engine's right-click pointer action routes through the same hit test
// with button 1 (it used to fall through the handler's default: arm).
static void test_pinch_right_click() {
    scene s;
    s.inject_pose(identity_pose());
    const float panel_pos[3] = {0.0f, 0.0f, -0.5f};
    uint64_t h = spawn_flat_panel(s, panel_pos);
    captured_click click;
    install_click_capture(s, click);

    ge_event_t ev{};
    ev.type = GE_EVENT_BEGIN;
    ev.action = GE_ACTION_POINTER_RIGHT_CLICK;
    ev.gesture_name = "two_finger_pinch";
    ev.position[0] = -0.15f;
    ev.position[1] = 0.1125f;
    ev.position[2] = -0.5f;
    s.inject_gesture(ev);

    CHECK(s.focused_handle() == h);
    CHECK(click.count == 1);
    CHECK(click.button == 1);
    CHECK_NEAR(click.x, 200.0f, 1.0f);
    CHECK_NEAR(click.y, 150.0f, 1.0f);
}

// Aim feedback: the candidate is recomputed every tick from the live hand, so
// the renderer can rim it before the user commits to a pinch.
static void test_aim_handle_tracks_candidate() {
    scene s;
    s.inject_pose(identity_pose());
    uint64_t left = s.spawn_panel("test-card", "left");
    uint64_t right = s.spawn_panel("test-card", "right");
    float lpos[3] = {-0.6f, 0.0f, -0.9f};
    float rpos[3] = {0.6f, 0.0f, -0.9f};
    CHECK(s.move_panel(left, lpos, false));
    CHECK(s.move_panel(right, rpos, false));
    s.tick(1.0f / 60.0f);
    CHECK(s.aim_handle() == 0);  // no hand yet

    // Open-hand pinch midpoint offset (thumb tip + index tip) / 2.
    const float open_offset[3] = {-0.055f, 0.105f, 0.0f};
    auto aim_from = [&](const float point[3]) {
        float origin[3];
        for (int i = 0; i < 3; i++)
            origin[i] = point[i] - open_offset[i];
        for (int i = 0; i < 4; i++) {
            s.inject_hand(0, make_open_hand(origin));
            s.tick(1.0f / 60.0f);
        }
        return s.aim_handle();
    };

    const float toward_left[3] = {-0.2f, 0.0f, -0.3f};
    CHECK(aim_from(toward_left) == left);
    const float toward_right[3] = {0.2f, 0.0f, -0.3f};
    CHECK(aim_from(toward_right) == right);

    // Hand gone → nothing aimed.
    tick_n(s, 20);
    CHECK(s.aim_handle() == 0);

    // The snapshot carries it for the renderer.
    CHECK(aim_from(toward_right) == right);
    auto snap = s.snapshot_render_panels();
    for (const auto &rp : snap)
        CHECK(rp.aimed == (rp.handle == right));

    // An overlay owns the pointer vocabulary: no aim while it is up.
    s.keyboard_show();
    s.inject_hand(0, make_open_hand(toward_right));
    s.tick(1.0f / 60.0f);
    CHECK(s.aim_handle() == 0);
}

static void test_pinch_hit_beats_nearby_missed_panel() {
    scene s;
    s.inject_pose(identity_pose());
    const float near_pos[3] = {0.0f, 0.0f, -0.5f};
    const float hit_pos[3] = {0.0f, 0.54f, -0.9f};
    spawn_flat_panel(s, near_pos);
    uint64_t hit = s.spawn_captured_panel("app:test", "ray-hit", 4242, 800, 600);
    CHECK(s.move_panel(hit, hit_pos, false));
    s.tick(1.0f / 60.0f);
    captured_click click;
    install_click_capture(s, click);
    const float point[3] = {0.0f, 0.30f, -0.5f};
    pinch_at(s, point);
    CHECK(s.focused_handle() == hit);
    CHECK(s.aim_handle() == hit);
    CHECK(click.count == 1);
    CHECK(click.handle == hit);
}

static void test_aim_ignores_unreliable_fingertips() {
    scene s;
    s.inject_pose(identity_pose());
    const float pos[3] = {0.0f, 0.0f, -0.9f};
    uint64_t target = spawn_flat_panel(s, pos);
    const float origin[3] = {0.055f, -0.105f, -0.3f};
    for (int joint : {SB_JOINT_THUMB_TIP, SB_JOINT_INDEX_TIP}) {
        auto hand = make_open_hand(origin);
        hand.joints[joint][3] = 0.0f;
        s.inject_hand(0, hand);
        s.tick(1.0f / 60.0f);
        CHECK(s.aim_handle() == 0);
    }
    s.inject_hand(1, make_open_hand(origin));
    s.tick(1.0f / 60.0f);
    const float other_origin[3] = {-0.4f, -0.105f, -0.3f};
    auto unreliable = make_open_hand(other_origin);
    unreliable.joints[SB_JOINT_INDEX_TIP][3] = 0.0f;
    s.inject_hand(0, unreliable);
    s.tick(1.0f / 60.0f);
    CHECK(s.aim_handle() == target);
}

static void test_ray_picks_nearest_rotated_surface() {
    scene s;
    s.inject_pose(identity_pose());
    sb_plane_t planes[] = {
        make_plane(0x31, 0.1f, 0.0f, -0.9f, 0.8660254f, 0, 0.5f, 2, 2, 1),
        make_plane(0x32, 0.2f, 0.0f, -0.8f, 0, 0, 1, 2, 2, 1),
    };
    s.inject_planes(planes, 2);
    auto front = s.spawn_captured_panel("app:test", "slanted-front", 4242, 800, 600);
    auto rear = s.spawn_captured_panel("app:test", "rear", 4242, 800, 600);
    uint8_t uuid[16];
    CHECK(s.anchor_panel(front, 0, planes[0].uuid, uuid) == 1);
    CHECK(s.anchor_panel(rear, 0, planes[1].uuid, uuid) == 1);
    tick_n(s, 2);
    captured_click click;
    install_click_capture(s, click);
    const float point[3] = {0, 0, -0.3f};
    pinch_at(s, point);
    CHECK(click.count == 1);
    CHECK(click.handle == front);
    CHECK(s.aim_handle() == front);
}

int main() {
    test_ray_picks_nearest_rotated_surface();
    test_aim_ignores_unreliable_fingertips();
    test_pinch_hit_beats_nearby_missed_panel();
    test_spawn_and_move();
    test_pinch_focus();
    test_anchor_follows_plane();
    test_nan_plane_keeps_finite_pose();
    test_panel_cap();
    test_input_routing();
    test_world_origin_capture();
    test_session_epoch_reanchor();
    test_origin_recapture_after_pose_gap();
    test_head_pose_ring();
    test_head_pose_ring_dropped_on_reset();
    test_grab_deadband_and_spring();
    test_per_panel_surface_version();
    test_pinch_focus_not_behind();
    test_pinch_click_direct_touch();
    test_pinch_click_ray();
    test_pinch_outside_quad_focuses_only();
    test_pinch_right_click();
    test_aim_handle_tracks_candidate();

    if (g_failures) {
        std::fprintf(stderr, "test_scene: %d FAILURES\n", g_failures);
        return 1;
    }
    std::printf("test_scene: all tests passed\n");
    return 0;
}
