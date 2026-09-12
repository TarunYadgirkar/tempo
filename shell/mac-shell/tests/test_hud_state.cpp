// test_hud_state.cpp — welcome/link state machine transitions, toast queue
// lifetime + dedupe, first-run decisions, and the scene toast plumbing.
// Everything here runs headless (no AppKit, no persistence).

#include <cstdio>
#include <string>

#include "core/hud_state.h"
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

// Advance the model with a constant packet rate for `seconds`.
static void run_for(welcome_model &m, float rate, float seconds) {
    const float dt = 1.0f / 60.0f;
    for (float t = 0; t < seconds; t += dt)
        m.tick(rate, dt);
}

static void test_welcome_waiting_first() {
    welcome_model m;
    // Fresh launch: card held back until the quiet window elapses.
    run_for(m, 0.0f, 1.0f);
    CHECK(m.phase == link_phase::waiting_first);
    CHECK(!m.card_visible(true));
    run_for(m, 0.0f, 1.5f);  // total 2.5 s of silence
    CHECK(m.card_visible(true));
    CHECK(!m.lost_wording());  // never connected — no "lost" wording
    // A non-empty scene suppresses the card while waiting.
    CHECK(!m.card_visible(false));
}

static void test_welcome_connect_and_drop() {
    welcome_model m;
    run_for(m, 0.0f, 3.0f);
    CHECK(m.card_visible(true));

    // Packets arrive: card hides, connect event fires once.
    m.tick(59.0f, 1.0f / 60.0f);
    CHECK(m.phase == link_phase::connected);
    CHECK(m.just_connected);
    m.consume_events();
    run_for(m, 59.0f, 1.0f);
    CHECK(!m.just_connected);
    CHECK(!m.card_visible(true));

    // Stream drops: quiet window, then the card returns with an event —
    // even over a scene with panels.
    run_for(m, 0.0f, 1.0f);
    CHECK(m.phase == link_phase::connected);  // grace not yet elapsed
    CHECK(!m.just_lost);
    run_for(m, 0.0f, 1.5f);  // 2.5 s of silence
    CHECK(m.phase == link_phase::lost);
    CHECK(m.just_lost);
    m.consume_events();
    CHECK(m.card_visible(true));
    CHECK(m.card_visible(false));
    CHECK(!m.lost_wording());  // wording flips only after 5 s

    run_for(m, 0.0f, 3.0f);  // 5.5 s of silence
    CHECK(m.lost_wording());

    // Reconnect: back to connected, card gone, one connect event.
    m.tick(60.0f, 1.0f / 60.0f);
    CHECK(m.phase == link_phase::connected);
    CHECK(m.just_connected);
    CHECK(!m.card_visible(true));
}

static void test_toast_queue() {
    toast_queue q;
    CHECK(q.empty());
    q.push("hello");
    q.push("hello");  // duplicate while visible → dropped
    CHECK(q.size() == 1);
    CHECK(q.front() && q.front()->text == "hello");
    CHECK(q.front()->total_s == TOAST_DURATION_S);

    // Action toasts live longer.
    q.push("grant it", toast_action::open_screen_recording_settings);
    CHECK(q.size() == 2);

    // Expiry: the plain toast dies at TOAST_DURATION_S, the action toast
    // survives until TOAST_ACTION_DURATION_S.
    q.tick(TOAST_DURATION_S + 0.1f);
    CHECK(q.size() == 1);
    CHECK(q.front()->action == toast_action::open_screen_recording_settings);
    q.tick(TOAST_ACTION_DURATION_S);
    CHECK(q.empty());

    // Once expired, the same text may toast again.
    q.push("hello");
    CHECK(q.size() == 1);

    // Overflow keeps the newest TOAST_QUEUE_MAX.
    for (int i = 0; i < 10; i++)
        q.push("t" + std::to_string(i));
    CHECK(q.size() == TOAST_QUEUE_MAX);

    // dismiss_front pops exactly one.
    size_t before = q.size();
    q.dismiss_front();
    CHECK(q.size() == before - 1);
}

static void test_first_run_decisions() {
    first_run_prefs fresh;  // first ever launch
    CHECK(should_show_onboarding(fresh));
    CHECK(should_autoshow_cheatsheet(fresh));

    first_run_prefs second;
    second.seen_onboarding = true;
    second.session_count = CHEATSHEET_AUTO_SESSIONS;
    CHECK(!should_show_onboarding(second));
    CHECK(should_autoshow_cheatsheet(second));

    first_run_prefs veteran;
    veteran.seen_onboarding = true;
    veteran.session_count = CHEATSHEET_AUTO_SESSIONS + 1;
    CHECK(!should_show_onboarding(veteran));
    CHECK(!should_autoshow_cheatsheet(veteran));
}

static void test_scene_toast_plumbing() {
    scene s;
    CHECK(s.snapshot_toasts().empty());
    s.post_toast("stream dropped");
    auto toasts = s.snapshot_toasts();
    CHECK(toasts.size() == 1);
    CHECK(toasts[0].text == "stream dropped");

    // tick() advances toast lifetime; it expires without renderer help.
    for (int i = 0; i < 60 * 7; i++)
        s.tick(1.0f / 60.0f);
    CHECK(s.snapshot_toasts().empty());

    // dismiss_toast drops the visible one immediately.
    s.post_toast("a", toast_action::open_screen_recording_settings);
    s.dismiss_toast();
    CHECK(s.snapshot_toasts().empty());
}

static void test_firewall_hint() {
    welcome_model w;
    // Not advertising: never blame the firewall, the phone has no way in yet.
    for (int i = 0; i < 60 * 30; i++)
        w.tick(0.0f, 1.0f / 60.0f);
    CHECK(!w.firewall_hint(false));
    // Advertising and long silent: the one cause the user cannot discover.
    CHECK(w.firewall_hint(true));

    // A live link never shows it, however long it has been up.
    welcome_model c;
    for (int i = 0; i < 60 * 30; i++)
        c.tick(60.0f, 1.0f / 60.0f);
    CHECK(!c.firewall_hint(true));

    // Silent, but not yet long enough to be conclusive.
    welcome_model e;
    for (int i = 0; i < 60 * 5; i++)
        e.tick(0.0f, 1.0f / 60.0f);
    CHECK(!e.firewall_hint(true));
}

static void test_passthrough_fade() {
    // Fresh frames render at full strength.
    CHECK(passthrough_fade(0.0f) == 1.0f);
    CHECK(passthrough_fade(FRAME_STALE_AFTER_S) == 1.0f);
    // Then a short ramp to the clear colour, fully gone by stale+fade.
    CHECK(passthrough_fade(FRAME_STALE_AFTER_S + FRAME_FADE_S * 0.5f) < 1.0f);
    CHECK(passthrough_fade(FRAME_STALE_AFTER_S + FRAME_FADE_S * 0.5f) > 0.0f);
    // Float-exact equality does not hold at the boundary itself (1.3f - 1.0f
    // is a hair under 0.3f), so check it has effectively arrived there.
    CHECK(passthrough_fade(FRAME_STALE_AFTER_S + FRAME_FADE_S) < 0.001f);
    CHECK(passthrough_fade(FRAME_STALE_AFTER_S + FRAME_FADE_S * 1.01f) ==
          0.0f);
    CHECK(passthrough_fade(30.0f) == 0.0f);
    // Interruptible: a frame arriving mid-fade snaps straight back, because
    // the curve is a pure function of age rather than latched state.
    CHECK(passthrough_fade(0.01f) == 1.0f);
}

int main() {
    test_welcome_waiting_first();
    test_welcome_connect_and_drop();
    test_toast_queue();
    test_first_run_decisions();
    test_scene_toast_plumbing();
    test_firewall_hint();
    test_passthrough_fade();
    if (g_failures) {
        std::fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("test_hud_state: all checks passed\n");
    return 0;
}
