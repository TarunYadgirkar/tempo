// test_launcher.cpp — radial launcher: config load, control-plane selection,
// and the synthetic fist-rotation → selection → release-commit state machine
// through the real gesture engine + fist tracker.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>

#include "ui/launcher_menu.h"
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

namespace {

constexpr float DT = 1.0f / 60.0f;

std::string g_config_dir;

sb_pose_t identity_pose() {
    sb_pose_t p{};
    p.timestamp_ns = 1;
    p.rot[3] = 1.0f;
    p.tracking_quality = 1.0f;
    return p;
}

// Open-hand fixture (gesture-engine test conventions).
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

void curl_finger(sb_hand_t &h, int mcp, int pip, int dip, int tip) {
    float mx = h.joints[mcp][0], my = h.joints[mcp][1], mz = h.joints[mcp][2];
    h.joints[pip][0] = mx;
    h.joints[pip][1] = my + 0.03f;
    h.joints[pip][2] = mz;
    h.joints[dip][0] = mx;
    h.joints[dip][1] = my + 0.02f;
    h.joints[dip][2] = mz - 0.02f;
    h.joints[tip][0] = mx;
    h.joints[tip][1] = my - 0.01f;
    h.joints[tip][2] = mz - 0.01f;
}

sb_hand_t make_fist(const float origin[3]) {
    sb_hand_t h = make_open_hand(origin);
    curl_finger(h, SB_JOINT_INDEX_MCP, SB_JOINT_INDEX_PIP, SB_JOINT_INDEX_DIP,
                SB_JOINT_INDEX_TIP);
    curl_finger(h, SB_JOINT_MIDDLE_MCP, SB_JOINT_MIDDLE_PIP,
                SB_JOINT_MIDDLE_DIP, SB_JOINT_MIDDLE_TIP);
    curl_finger(h, SB_JOINT_RING_MCP, SB_JOINT_RING_PIP, SB_JOINT_RING_DIP,
                SB_JOINT_RING_TIP);
    curl_finger(h, SB_JOINT_PINKY_MCP, SB_JOINT_PINKY_PIP, SB_JOINT_PINKY_DIP,
                SB_JOINT_PINKY_TIP);
    curl_finger(h, SB_JOINT_THUMB_CMC, SB_JOINT_THUMB_MCP, SB_JOINT_THUMB_IP,
                SB_JOINT_THUMB_TIP);
    return h;
}

// Roll the whole hand about the forearm axis (wrist → middle MCP) by angle.
sb_hand_t rolled_fist(const float origin[3], float angle_rad) {
    sb_hand_t h = make_fist(origin);
    float wrist[3] = {h.joints[SB_JOINT_WRIST][0],
                      h.joints[SB_JOINT_WRIST][1],
                      h.joints[SB_JOINT_WRIST][2]};
    float axis[3] = {h.joints[SB_JOINT_MIDDLE_MCP][0] - wrist[0],
                     h.joints[SB_JOINT_MIDDLE_MCP][1] - wrist[1],
                     h.joints[SB_JOINT_MIDDLE_MCP][2] - wrist[2]};
    float len = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] +
                          axis[2] * axis[2]);
    for (int i = 0; i < 3; i++)
        axis[i] /= len;
    float c = std::cos(angle_rad), s = std::sin(angle_rad);
    for (int j = 0; j < SB_HAND_JOINT_COUNT; j++) {
        float v[3] = {h.joints[j][0] - wrist[0], h.joints[j][1] - wrist[1],
                      h.joints[j][2] - wrist[2]};
        float d = axis[0] * v[0] + axis[1] * v[1] + axis[2] * v[2];
        float cx = axis[1] * v[2] - axis[2] * v[1];
        float cy = axis[2] * v[0] - axis[0] * v[2];
        float cz = axis[0] * v[1] - axis[1] * v[0];
        h.joints[j][0] = wrist[0] + v[0] * c + cx * s + axis[0] * d * (1 - c);
        h.joints[j][1] = wrist[1] + v[1] * c + cy * s + axis[1] * d * (1 - c);
        h.joints[j][2] = wrist[2] + v[2] * c + cz * s + axis[2] * d * (1 - c);
    }
    return h;
}

void write_launcher_config(const std::string &dir) {
    std::string cfg_dir = dir + "/spatial-os";
    std::string cmd = "mkdir -p '" + cfg_dir + "'";
    CHECK(system(cmd.c_str()) == 0);
    std::ofstream f(cfg_dir + "/launcher.toml");
    f << "# test launcher config\n"
      << "[[entry]]\nlabel = \"Alpha\"\ntarget = \"internal:alpha\"\n\n"
      << "[[entry]]\nlabel = \"Beta\"\ntarget = \"internal:beta\"\n\n"
      << "[[entry]]\nlabel = \"Gamma\"\ntarget = \"internal:gamma\"\n\n"
      << "[[entry]]\nlabel = \"AppTarget\"\ntarget = \"app:Safari\"\n";
}

// ---------------------------------------------------------------------------

void test_config_and_model() {
    launcher_menu m;
    CHECK(m.entries().size() == 4);
    CHECK(m.entries()[0].label == "Alpha");
    CHECK(m.entries()[3].target == "app:Safari");

    std::string launched;
    m.set_launch_callback(
        [&](const launcher_entry &e) { launched = e.target; });
    m.show();
    CHECK(m.visible());
    CHECK(m.select(2));
    CHECK(!m.select(9));
    CHECK(m.focused_index() == 2);
    CHECK(m.commit());
    CHECK(launched == "internal:gamma");
    CHECK(!m.visible());

    // Commit with nothing focused fails.
    m.show();
    CHECK(!m.commit());
    m.hide();
    std::printf("PASS test_config_and_model\n");
}

void test_rotation_state_machine() {
    launcher_menu m;
    m.show();
    m.begin_rotation();

    const float origin[3] = {0.1f, -0.2f, -0.5f};
    // Establish the neutral, then sweep the roll; the focused wedge must move
    // monotonically across the page and commit on release.
    std::vector<int> seen;
    for (int f = 0; f < 90; f++) {
        float angle = -0.9f + 1.8f * (float)f / 89.0f;  // ±~51 deg
        sb_hand_t h = rolled_fist(origin, angle);
        m.update_rotation(h.joints[SB_JOINT_WRIST],
                          h.joints[SB_JOINT_INDEX_MCP],
                          h.joints[SB_JOINT_MIDDLE_MCP],
                          h.joints[SB_JOINT_PINKY_MCP], true);
        if (seen.empty() || seen.back() != m.focused_index())
            seen.push_back(m.focused_index());
    }
    CHECK(seen.size() >= 2);  // selection moved
    for (size_t i = 1; i < seen.size(); i++)
        CHECK(std::abs(seen[i] - seen[i - 1]) >= 1);

    std::string launched;
    m.set_launch_callback(
        [&](const launcher_entry &e) { launched = e.label; });
    int focused = m.focused_index();
    CHECK(focused >= 0);
    CHECK(m.end_rotation_commit());
    CHECK(launched == m.entries()[(size_t)focused].label);
    CHECK(!m.visible());
    std::printf("PASS test_rotation_state_machine (%zu focus steps)\n",
                seen.size());
}

// Jitter on a wedge boundary must not flip the focus: every flip re-draws
// the whole 768x768 HUD surface on the CPU.
void test_rotation_hysteresis() {
    launcher_menu m;
    m.show();
    m.begin_rotation();

    const float origin[3] = {0.1f, -0.2f, -0.5f};
    const float deg = 0.017453293f;
    auto roll_to = [&](float angle_deg) {
        sb_hand_t h = rolled_fist(origin, angle_deg * deg);
        m.update_rotation(h.joints[SB_JOINT_WRIST],
                          h.joints[SB_JOINT_INDEX_MCP],
                          h.joints[SB_JOINT_MIDDLE_MCP],
                          h.joints[SB_JOINT_PINKY_MCP], true);
        return m.focused_index();
    };

    // Creep up until the focus flips; that input angle is a wedge boundary.
    roll_to(-50.0f);
    int start = m.focused_index();
    float boundary = 0.0f;
    bool found = false;
    for (int i = 1; i <= 400 && !found; i++) {
        float a = -50.0f + 0.25f * (float)i;
        if (roll_to(a) != start) {
            boundary = a;
            found = true;
        }
    }
    CHECK(found);
    int held = m.focused_index();

    // +/-2 deg of jitter across it: the focus must not move at all.
    for (int f = 0; f < 60; f++) {
        roll_to(boundary + ((f % 2) ? 2.0f : -2.0f));
        CHECK(m.focused_index() == held);
    }

    // A deliberate sweep still moves it, and soon: one wedge is 30 deg
    // (SCRUB_SPAN_RAD / 4 entries), so a real scrub has to clear a wedge
    // plus the hysteresis band, not much more.
    float flipped_at = 0.0f;
    for (int i = 1; i <= 100 && flipped_at == 0.0f; i++) {
        float a = boundary + 0.5f * (float)i;
        if (roll_to(a) != held)
            flipped_at = a - boundary;
    }
    CHECK(flipped_at > 30.0f);   // hysteresis really is holding the wedge
    CHECK(flipped_at < 45.0f);   // ...but it is a band, not a lockup
    m.hide();
    std::printf("PASS test_rotation_hysteresis (boundary %.2f deg, flip at "
                "+%.2f deg)\n",
                (double)boundary, (double)flipped_at);
}

void test_scene_fist_gesture_flow() {
    scene s;
    s.inject_pose(identity_pose());
    s.tick(DT);
    size_t panels_before = s.panel_count();

    const float origin[3] = {0.0f, 0.0f, -0.4f};
    // Fist held: fist_launcher (min_hold 300 ms — the pre-arm gate) engages
    // the launcher. It must NOT appear before the hold has run.
    for (int f = 0; f < 10; f++) {  // ~167 ms of fist: still pre-arming
        s.inject_hand(0, make_fist(origin));
        s.tick(DT);
    }
    CHECK(!s.launcher_visible());
    for (int f = 0; f < 20; f++) {
        s.inject_hand(0, make_fist(origin));
        s.tick(DT);
    }
    CHECK(s.launcher_visible());

    // Roll the fist to scrub the selection.
    for (int f = 0; f < 60; f++) {
        float angle = 0.9f * (float)f / 59.0f;
        s.inject_hand(0, rolled_fist(origin, angle));
        s.tick(DT);
    }
    std::string st = s.launcher_status_json();
    CHECK(st.find("\"visible\":true") != std::string::npos);
    CHECK(st.find("\"focused\":-1") == std::string::npos);

    // Open the hand: fist_launcher END commits the focused entry, and the
    // menu lingers with the chosen wedge highlighted instead of vanishing.
    bool committed_while_visible = false;
    for (int f = 0; f < 20; f++) {
        s.inject_hand(0, make_open_hand(origin));
        s.tick(DT);
        if (s.panel_count() == panels_before + 1 && s.launcher_visible())
            committed_while_visible = true;
    }
    CHECK(s.panel_count() == panels_before + 1);
    CHECK(committed_while_visible);  // commit-linger held it up

    // The linger (300 ms) elapses → the menu fades without re-committing.
    for (int f = 0; f < 30; f++) {
        s.inject_hand(0, make_open_hand(origin));
        s.tick(DT);
    }
    CHECK(!s.launcher_visible());
    CHECK(s.panel_count() == panels_before + 1);
    std::printf("PASS test_scene_fist_gesture_flow\n");
}

void test_scene_tracking_loss_grace() {
    scene s;
    s.inject_pose(identity_pose());
    s.tick(DT);
    size_t panels_before = s.panel_count();

    const float origin[3] = {0.0f, 0.0f, -0.4f};
    for (int f = 0; f < 30; f++) {
        s.inject_hand(0, make_fist(origin));
        s.tick(DT);
    }
    CHECK(s.launcher_visible());

    // Hand vanishes (tracking drop → gesture CANCEL): the menu must survive
    // the 600 ms grace window...
    for (int f = 0; f < 24; f++)  // 400 ms with no hand
        s.tick(DT);
    CHECK(s.launcher_visible());

    // ...then fade out WITHOUT committing once the grace expires.
    for (int f = 0; f < 30; f++)  // +500 ms
        s.tick(DT);
    CHECK(!s.launcher_visible());
    CHECK(s.panel_count() == panels_before);  // nothing launched
    std::printf("PASS test_scene_tracking_loss_grace\n");
}

void test_scene_tracking_loss_resume() {
    scene s;
    s.inject_pose(identity_pose());
    s.tick(DT);
    size_t panels_before = s.panel_count();

    const float origin[3] = {0.0f, 0.0f, -0.4f};
    for (int f = 0; f < 30; f++) {
        s.inject_hand(0, make_fist(origin));
        s.tick(DT);
    }
    CHECK(s.launcher_visible());

    // Lose the hand for a few frames, then the fist comes back: the menu
    // must never have dismissed, and no commit fired for the dropout.
    for (int f = 0; f < 12; f++) {  // 200 ms gap
        s.tick(DT);
        CHECK(s.launcher_visible());
    }
    for (int f = 0; f < 25; f++) {  // fist re-acquired (300 ms re-hold)
        s.inject_hand(0, make_fist(origin));
        s.tick(DT);
        CHECK(s.launcher_visible());
    }
    CHECK(s.panel_count() == panels_before);

    // A deliberate release still commits normally afterwards.
    for (int f = 0; f < 50; f++) {
        s.inject_hand(0, make_open_hand(origin));
        s.tick(DT);
    }
    CHECK(s.panel_count() == panels_before + 1);
    CHECK(!s.launcher_visible());
    std::printf("PASS test_scene_tracking_loss_resume\n");
}

void test_scene_control_verbs() {
    scene s;
    s.inject_pose(identity_pose());
    s.tick(DT);
    s.launcher_show();
    CHECK(s.launcher_visible());
    std::string st = s.launcher_status_json();
    CHECK(st.find("\"Alpha\"") != std::string::npos);
    CHECK(s.launcher_select(0));
    CHECK(s.launcher_commit());
    CHECK(!s.launcher_visible());
    // internal:alpha → an internal test-card panel titled "alpha".
    CHECK(s.windows_json(false).find("alpha") != std::string::npos);

    // app: target without a registered app launcher → headless fallback
    // panel named after the target.
    s.launcher_show();
    CHECK(s.launcher_select(3));
    CHECK(s.launcher_commit());
    CHECK(s.windows_json(false).find("Safari") != std::string::npos);
    std::printf("PASS test_scene_control_verbs\n");
}

// Opening the menu and letting go without ever scrubbing must cancel, not
// launch whatever entry happens to sit first on the page.
void test_release_without_scrub_cancels() {
    launcher_menu m;
    std::string launched;
    m.set_launch_callback(
        [&](const launcher_entry &e) { launched = e.label; });
    m.show();
    m.begin_rotation();
    CHECK(m.focused_index() < 0);
    CHECK(!m.end_rotation_commit());
    CHECK(!m.visible());
    CHECK(launched.empty());

    m.show();
    m.begin_rotation();
    CHECK(!m.end_rotation_commit_keep_visible());
    CHECK(!m.visible());
    CHECK(launched.empty());
    std::printf("PASS test_release_without_scrub_cancels\n");
}

// The same cancel through the scene's gesture handler, plus the escape hatch
// for a menu already up: a second fist puts it away without launching.
void test_scene_launcher_cancel() {
    scene s;
    s.inject_pose(identity_pose());
    s.tick(DT);
    size_t panels_before = s.panel_count();

    ge_event_t ev{};
    ev.action = GE_ACTION_TOGGLE_LAUNCHER;
    ev.gesture_name = "fist_launcher";

    ev.type = GE_EVENT_BEGIN;
    s.inject_gesture(ev);
    CHECK(s.launcher_visible());
    for (int f = 0; f < 5; f++)  // no hand streaming → nothing ever scrubs
        s.tick(DT);
    ev.type = GE_EVENT_END;
    s.inject_gesture(ev);
    CHECK(!s.launcher_visible());
    CHECK(s.panel_count() == panels_before);

    // Menu up and unattended (control verb): a fist dismisses it.
    s.launcher_show();
    CHECK(s.launcher_visible());
    ev.type = GE_EVENT_BEGIN;
    s.inject_gesture(ev);
    CHECK(!s.launcher_visible());
    CHECK(s.panel_count() == panels_before);
    std::printf("PASS test_scene_launcher_cancel\n");
}

void test_snapshot() {
    scene s;
    s.launcher_show();
    s.launcher_select(1);
    auto rs = s.snapshot_launcher();
    CHECK(rs.visible);
    CHECK(rs.labels.size() == 4);
    CHECK(rs.focused == 1);
    CHECK(rs.rgba.size() ==
          (size_t)rs.surface_w * (size_t)rs.surface_h * 4);
    s.launcher_hide();
    CHECK(!s.snapshot_launcher().visible);
    std::printf("PASS test_snapshot\n");
}

// The built-in defaults (used when no launcher.toml is found) list real
// openable apps plus a "Close All" self-target — not the hand-skeleton
// overlay the user saw on the stale rig binary. Verified with an empty
// XDG_CONFIG_HOME so the test is independent of the dev machine's ~/.config.
void test_default_entries() {
    char tmpl[] = "/tmp/spatula-launcher-defaults-XXXXXX";
    const char *dir = mkdtemp(tmpl);
    assert(dir);
    const char *prev_xdg = getenv("XDG_CONFIG_HOME");
    const char *prev_home = getenv("HOME");
    setenv("XDG_CONFIG_HOME", dir, 1);
    setenv("HOME", "/nonexistent-spatula-launcher-defaults", 1);

    launcher_menu m;
    CHECK(m.entries().size() == 6);
    // Real openable apps + the close-all self-target + the internal test card.
    std::string labels;
    for (const auto &e : m.entries())
        labels += e.label + "|";
    CHECK(labels.find("Safari|") != std::string::npos);
    CHECK(labels.find("Terminal|") != std::string::npos);
    CHECK(labels.find("Finder|") != std::string::npos);
    CHECK(labels.find("Notes|") != std::string::npos);
    CHECK(labels.find("Close All|") != std::string::npos);
    CHECK(labels.find("Test Card|") != std::string::npos);

    // The "Close All" entry targets the close-all self-action.
    bool found_close_all = false;
    for (const auto &e : m.entries()) {
        if (e.label == "Close All") {
            CHECK(e.target == "close-all");
            found_close_all = true;
        }
    }
    CHECK(found_close_all);

    // Selecting and committing the "Close All" wedge fires the launch
    // callback with the close-all target (the scene wires that to
    // close_all_panels).
    std::string launched_target;
    m.set_launch_callback(
        [&](const launcher_entry &e) { launched_target = e.target; });
    m.show();
    // "Close All" is the 6th entry (index 5) on page 0.
    CHECK(m.select(5));
    CHECK(m.commit());
    CHECK(launched_target == "close-all");

    if (prev_xdg)
        setenv("XDG_CONFIG_HOME", prev_xdg, 1);
    else
        unsetenv("XDG_CONFIG_HOME");
    if (prev_home)
        setenv("HOME", prev_home, 1);
    else
        unsetenv("HOME");
    rmdir(dir);
    std::printf("PASS test_default_entries\n");
}

// Scene-level: committing the launcher's "Close All" entry closes every open
// panel without dismissing the launcher mid-gesture (the launcher is not a
// panel). Uses an empty XDG_CONFIG_HOME so the built-in defaults apply.
void test_scene_close_all_entry() {
    char tmpl[] = "/tmp/spatula-launcher-closeall-XXXXXX";
    const char *dir = mkdtemp(tmpl);
    assert(dir);
    const char *prev_xdg = getenv("XDG_CONFIG_HOME");
    const char *prev_home = getenv("HOME");
    setenv("XDG_CONFIG_HOME", dir, 1);
    setenv("HOME", "/nonexistent-spatula-launcher-closeall", 1);

    scene s;
    s.inject_pose(identity_pose());
    s.tick(DT);
    s.spawn_panel("test-card", "alpha");
    s.spawn_panel("test-card", "beta");
    s.spawn_note_panel("Note", "body", false);
    CHECK(s.panel_count() == 3);

    s.launcher_show();
    CHECK(s.launcher_visible());
    std::string st = s.launcher_status_json();
    CHECK(st.find("\"Close All\"") != std::string::npos);
    // "Close All" is the 6th entry (index 5) on page 0.
    CHECK(s.launcher_select(5));
    CHECK(s.launcher_commit());
    // The launcher hid on commit; every panel closed.
    CHECK(!s.launcher_visible());
    CHECK(s.panel_count() == 0);

    if (prev_xdg)
        setenv("XDG_CONFIG_HOME", prev_xdg, 1);
    else
        unsetenv("XDG_CONFIG_HOME");
    if (prev_home)
        setenv("HOME", prev_home, 1);
    else
        unsetenv("HOME");
    rmdir(dir);
    std::printf("PASS test_scene_close_all_entry\n");
}

}  // namespace

int main() {
    // Isolated config dir with a known launcher.toml; keeps gesture/keyboard
    // configs at compiled defaults too.
    char tmpl[] = "/tmp/spatula-launcher-test-XXXXXX";
    const char *dir = mkdtemp(tmpl);
    assert(dir);
    g_config_dir = dir;
    write_launcher_config(g_config_dir);
    setenv("XDG_CONFIG_HOME", g_config_dir.c_str(), 1);
    setenv("HOME", "/nonexistent-spatula-launcher-test", 1);

    test_config_and_model();
    test_rotation_state_machine();
    test_rotation_hysteresis();
    test_scene_fist_gesture_flow();
    test_scene_tracking_loss_grace();
    test_scene_tracking_loss_resume();
    test_scene_control_verbs();
    test_release_without_scrub_cancels();
    test_scene_launcher_cancel();
    test_snapshot();
    test_default_entries();
    test_scene_close_all_entry();

    if (g_failures) {
        std::fprintf(stderr, "test_launcher: %d FAILURES\n", g_failures);
        return 1;
    }
    std::printf("test_launcher: all tests passed\n");
    return 0;
}
