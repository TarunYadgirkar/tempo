// test_note_layout.cpp — the note card, the layout store and the `aim` reply,
// exercised against the scene core directly (no socket). The e2e reply shapes
// for the same features live in tests/test_hack_verbs_e2e.cpp.

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include "core/json_lite.h"
#include "core/layout_store.h"
#include "core/scene.h"
#include "ui/note_card.h"

using namespace mac_shell;

static int g_failures = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, \
                         #cond);                                         \
            g_failures++;                                                \
        }                                                                \
    } while (0)

#define CHECK_MSG(cond, msg)                                                  \
    do {                                                                      \
        if (!(cond)) {                                                        \
            std::fprintf(stderr, "FAIL %s:%d: %s (%s)\n", __FILE__, __LINE__, \
                         #cond, std::string(msg).c_str());                    \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

static bool contains(const std::string &s, const std::string &sub) {
    return s.find(sub) != std::string::npos;
}

// ---------------------------------------------------------------------------

static void test_wrap_text() {
    auto lines = wrap_text("the quick brown fox jumps over the lazy dog", 12,
                           6);
    CHECK(lines.size() >= 3);
    for (const auto &l : lines)
        CHECK_MSG((int)l.size() <= 12, l);
    // Words stay whole and no line starts or ends on a space.
    for (const auto &l : lines) {
        CHECK(l.empty() || l.front() != ' ');
        CHECK(l.empty() || l.back() != ' ');
    }

    // Explicit newlines start a new line.
    auto para = wrap_text("one\ntwo", 20, 6);
    CHECK(para.size() == 2 && para[0] == "one" && para[1] == "two");

    // A word longer than the measure is split rather than dropped.
    auto split = wrap_text("abcdefghij", 4, 6);
    CHECK(split.size() == 3 && split[0] == "abcd");

    // Past the cap, the last line ends in an ellipsis.
    auto capped = wrap_text("a b c d e f g h i j k l m n o p", 3, 2);
    CHECK(capped.size() == 2);
    CHECK_MSG(!capped.empty() && capped.back().size() >= 3 &&
                  capped.back().compare(capped.back().size() - 3, 3, "...") == 0,
              capped.empty() ? "" : capped.back());

    CHECK(wrap_text("anything", 12, 0).empty());
    CHECK(wrap_text("", 12, 6).empty());
}

// The accent rule is the card's one mark: present at the left padding when
// accent is on, absent when it is off.
static void test_note_card_accent() {
    panel_surface surface(NOTE_DEFAULT_W_PX, NOTE_DEFAULT_H_PX);
    auto rule_px = [&](int *rgb) {
        const uint8_t *p = surface.pixels() + ((size_t)60 * surface.width() + 29) * 4;
        rgb[0] = p[0];
        rgb[1] = p[1];
        rgb[2] = p[2];
    };

    int rgb[3];
    render_note_card(surface, "Standup notes", "Ship the aim verb.", true);
    rule_px(rgb);
    char msg[64];
    std::snprintf(msg, sizeof(msg), "rgb %d,%d,%d", rgb[0], rgb[1], rgb[2]);
    CHECK_MSG(rgb[0] > 220 && rgb[1] > 100 && rgb[1] < 150 && rgb[2] < 50, msg);

    render_note_card(surface, "Standup notes", "Ship the aim verb.", false);
    rule_px(rgb);
    std::snprintf(msg, sizeof(msg), "rgb %d,%d,%d", rgb[0], rgb[1], rgb[2]);
    CHECK_MSG(rgb[0] < 80, msg);  // plain card surface, not the accent
}

static void test_json_lite() {
    json_value v;
    std::string err;
    CHECK(json_parse(R"({"title":"a \"b\"","accent":true,"n":[1,2,3]})", v,
                     err));
    CHECK(v.is_object());
    CHECK(v.find("title") && v.find("title")->str == "a \"b\"");
    CHECK(v.find("accent") && v.find("accent")->boolean);
    float f[3];
    CHECK(v.find("n") && v.find("n")->floats(f, 3) && f[2] == 3.0f);
    CHECK(v.find("missing") == nullptr);

    CHECK(!json_parse("{\"a\":}", v, err));
    CHECK(!json_parse("{\"a\":1} junk", v, err));
    CHECK(!json_parse("", v, err));

    std::string out;
    json_escape("tab\there\n", out);
    CHECK_MSG(out == "tab\\there\\n", out);
}

static void test_layout_names() {
    CHECK(layout_name_valid("desk"));
    CHECK(layout_name_valid("desk-2_A"));
    CHECK(!layout_name_valid(""));
    CHECK(!layout_name_valid("../etc"));
    CHECK(!layout_name_valid("with space"));
    CHECK(!layout_name_valid(std::string(41, 'a')));
    CHECK(layout_name_valid(std::string(40, 'a')));
}

static void test_layout_roundtrip(const std::string &tmp_cfg) {
    setenv("SPATIAL_OS_CONFIG_DIR", tmp_cfg.c_str(), 1);

    layout_panel note;
    note.kind = "note";
    note.app_id = "note";
    note.title = "Wall \"sticky\"";
    note.body = "Line one\nline two";
    note.accent = true;
    note.pos[0] = 0.25f;
    note.pos[1] = -0.1f;
    note.pos[2] = -1.4f;
    note.yaw = 0.5f;
    note.width_px = 512;
    note.height_px = 320;
    note.has_anchor = true;
    std::memset(note.anchor_uuid, 0xab, 16);
    note.anchor_offset[0] = 0.02f;

    layout_panel card;
    card.kind = "internal";
    card.app_id = "test-card";
    card.title = "scratch";

    std::string err;
    CHECK_MSG(layout_save("desk", {note, card}, err), err);

    std::vector<std::string> names = layout_list();
    CHECK(names.size() == 1 && names[0] == "desk");

    std::vector<layout_panel> back;
    CHECK_MSG(layout_load("desk", back, err), err);
    CHECK(back.size() == 2);
    if (back.size() == 2) {
        CHECK(back[0].kind == "note");
        CHECK_MSG(back[0].title == note.title, back[0].title);
        CHECK_MSG(back[0].body == note.body, back[0].body);
        CHECK(back[0].accent);
        CHECK(std::fabs(back[0].pos[2] - (-1.4f)) < 1e-4f);
        CHECK(std::fabs(back[0].yaw - 0.5f) < 1e-4f);
        CHECK(back[0].width_px == 512 && back[0].height_px == 320);
        CHECK(back[0].has_anchor && back[0].anchor_uuid[0] == 0xab);
        CHECK(std::fabs(back[0].anchor_offset[0] - 0.02f) < 1e-4f);
        CHECK(back[1].kind == "internal" && !back[1].has_anchor);
    }

    CHECK(!layout_load("nope", back, err));
    CHECK_MSG(err == "not_found", err);
}

static void test_scene_notes_and_aim() {
    scene s;
    uint64_t h = s.spawn_note_panel("Standup notes", "Ship the aim verb.",
                                    true);
    CHECK(h != 0);
    s.tick(0.016f);

    std::string lw = s.windows_json(false);
    CHECK_MSG(contains(lw, "\"app_id\":\"note\""), lw);
    CHECK_MSG(contains(lw, "\"title\":\"Standup notes\""), lw);
    CHECK_MSG(contains(lw, "\"size\":[512,320]"), lw);

    // The card is actually drawn: the snapshot carries its pixels.
    auto panels = s.snapshot_render_panels();
    CHECK(panels.size() == 1);
    CHECK(!panels.empty() && panels[0].kind == panel_kind::note);
    CHECK(!panels.empty() && panels[0].rgba.size() == 512u * 320u * 4u);
    uint64_t drawn_version = panels.empty() ? 0 : panels[0].surface_version;

    scene::note_patch patch;
    patch.set_title = true;
    patch.title = "Standup notes, revised";
    CHECK(s.update_note(h, patch));
    CHECK(!s.update_note(9999, patch));
    s.tick(0.016f);
    lw = s.windows_json(false);
    CHECK_MSG(contains(lw, "Standup notes, revised"), lw);
    panels = s.snapshot_render_panels();
    CHECK(!panels.empty() && panels[0].surface_version > drawn_version);

    // A test card is not a note, so note-update must refuse it.
    uint64_t card = s.spawn_panel("test-card", "scratch");
    CHECK(!s.update_note(card, patch));

    // No hands in a headless scene: every vector is null, nothing is aimed.
    std::string aim = s.aim_json();
    CHECK_MSG(contains(aim, "\"hands\":0"), aim);
    CHECK_MSG(contains(aim, "\"aim\":null"), aim);
    CHECK_MSG(contains(aim, "\"ray_origin\":null"), aim);
    CHECK_MSG(contains(aim, "\"ray_dir\":null"), aim);
    CHECK_MSG(contains(aim, "\"pinching\":false"), aim);
    CHECK_MSG(contains(aim, "\"aimed_handle\":null"), aim);
    CHECK_MSG(contains(aim, "\"hit\":null"), aim);

    // capture_layout keeps the note's text and the card's kind.
    std::vector<layout_panel> saved = s.capture_layout();
    CHECK(saved.size() == 2);
    if (saved.size() == 2) {
        CHECK(saved[0].kind == "note");
        CHECK(saved[0].body == "Ship the aim verb.");
        CHECK(saved[0].accent);
        CHECK(saved[1].kind == "internal");
    }

    // Restoring a pose onto a fresh panel puts it back where it was.
    scene s2;
    uint64_t h2 = s2.spawn_note_panel("Standup notes", "Ship the aim verb.",
                                      true);
    layout_panel moved = saved[0];
    moved.pos[0] = 1.5f;
    moved.pos[1] = 0.25f;
    moved.pos[2] = -2.0f;
    CHECK(s2.apply_layout_pose(h2, moved));
    CHECK(!s2.apply_layout_pose(9999, moved));
    float pos[3] = {0, 0, 0};
    CHECK(s2.panel_pose(h2, pos));
    CHECK(std::fabs(pos[0] - 1.5f) < 1e-4f);
    CHECK(std::fabs(pos[2] - (-2.0f)) < 1e-4f);
    CHECK(s2.panel_handles().size() == 1);
}

// `subscribe` gets a dedicated pinch line. The gesture line alongside it
// carries the winning VARIANT name, so it cannot be matched on by itself.
static void test_pinch_events() {
    scene s;
    s.drain_events();

    ge_event_t ev = {};
    ev.action = GE_ACTION_POINTER_CLICK;
    ev.gesture_name = "pinch_select.loose";
    ev.type = GE_EVENT_BEGIN;
    ev.position[2] = -1.0f;
    s.inject_gesture(ev);

    std::vector<std::string> events = s.drain_events();
    bool saw_begin = false, saw_variant = false;
    for (const auto &e : events) {
        saw_begin = saw_begin || e == "event pinch phase=begin";
        saw_variant = saw_variant ||
                      e == "event gesture name=pinch_select.loose phase=begin";
    }
    CHECK(saw_begin);
    CHECK(saw_variant);
    CHECK_MSG(contains(s.aim_json(), "\"pinching\":true"), s.aim_json());

    ev.type = GE_EVENT_END;
    s.inject_gesture(ev);
    events = s.drain_events();
    bool saw_end = false;
    for (const auto &e : events)
        saw_end = saw_end || e == "event pinch phase=end";
    CHECK(saw_end);
    CHECK_MSG(contains(s.aim_json(), "\"pinching\":false"), s.aim_json());

    // A cancelled pinch (tracking loss) must also release the held flag.
    ev.type = GE_EVENT_BEGIN;
    s.inject_gesture(ev);
    ev.type = GE_EVENT_CANCEL;
    s.inject_gesture(ev);
    CHECK_MSG(contains(s.aim_json(), "\"pinching\":false"), s.aim_json());
}

int main() {
    std::string dir = "/tmp";
    if (const char *t = getenv("TMPDIR"); t && t[0])
        dir = t;
    if (dir.back() == '/')
        dir.pop_back();
    std::string cfg =
        dir + "/mac-shell-layout-" + std::to_string((long)getpid());

    test_wrap_text();
    test_note_card_accent();
    test_json_lite();
    test_layout_names();
    test_layout_roundtrip(cfg);
    test_scene_notes_and_aim();
    test_pinch_events();

    unlink((cfg + "/layouts/desk.json").c_str());
    rmdir((cfg + "/layouts").c_str());
    rmdir(cfg.c_str());

    if (g_failures) {
        std::fprintf(stderr, "test_note_layout: %d FAILURES\n", g_failures);
        return 1;
    }
    std::printf("test_note_layout: all tests passed\n");
    return 0;
}
