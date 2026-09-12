// hud_state.h — pure-logic state behind the shell's "explain yourself" UX:
// the welcome/empty-state card (waiting for the iPhone / connection lost),
// the in-scene toast queue, and the first-run persistence decisions
// (onboarding card, gesture cheat sheet). No AppKit, no Metal — the
// renderer feeds inputs and draws outputs, so every transition is
// headless-testable.

#pragma once

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

namespace mac_shell {

// ------------------------------------------------------------------
// welcome / link state machine
// ------------------------------------------------------------------

// Packet rate below this counts as silence (the receiver decays its rate
// estimate, so a dead link reads as a small nonzero for a moment).
constexpr float LINK_SILENCE_RATE = 1.0f;
// Silence longer than this shows the welcome card (and, mid-session, fires
// the stream-dropped event).
constexpr float LINK_QUIET_AFTER_S = 2.0f;
// Silence longer than this switches the card to the "connection lost —
// reopen SpatialBridge" wording.
constexpr float LINK_LOST_WORDING_S = 5.0f;
// Bonjour is advertising but nothing has arrived for this long. By far the
// most common cause on a working network is the macOS application firewall
// silently dropping our inbound UDP — it gives no error anywhere, so the
// card has to name it.
constexpr float LINK_FIREWALL_HINT_S = 15.0f;

enum class link_phase {
    waiting_first,  // never saw a packet this run
    connected,      // packets flowing
    lost,           // packets stopped after having been connected
};

// Drive with tick(packet_rate, dt) once per frame; read phase/show flags,
// drain the one-shot events.
struct welcome_model {
    link_phase phase = link_phase::waiting_first;
    float silence_s = 0.0f;

    // One-shot events, cleared by consume_events().
    bool just_connected = false;
    bool just_lost = false;

    void tick(float packet_rate, float dt) {
        const bool active = packet_rate >= LINK_SILENCE_RATE;
        if (active) {
            silence_s = 0.0f;
            if (phase != link_phase::connected) {
                if (phase == link_phase::lost)
                    just_connected = true;  // reconnect notice
                else
                    just_connected = true;
                phase = link_phase::connected;
            }
            return;
        }
        silence_s += dt;
        if (phase == link_phase::connected &&
            silence_s > LINK_QUIET_AFTER_S) {
            phase = link_phase::lost;
            just_lost = true;
        }
    }

    // Card visibility target. While waiting for the very first connection
    // the card only appears over an empty scene (packets quiet > 2 s, no
    // panels, no overlay); after a mid-session drop it appears regardless
    // of panels — losing the stream deserves prominence.
    bool card_visible(bool scene_empty) const {
        switch (phase) {
            case link_phase::waiting_first:
                return scene_empty && silence_s > LINK_QUIET_AFTER_S;
            case link_phase::lost:
                return true;
            case link_phase::connected:
                return false;
        }
        return false;
    }

    // "connection lost — reopen SpatialBridge" wording.
    bool lost_wording() const {
        return phase == link_phase::lost && silence_s > LINK_LOST_WORDING_S;
    }

    // Extra "the firewall may be eating this" line under the steps. Only
    // worth showing once we know the phone *should* be able to find us:
    // we are advertising, yet nothing has arrived for a long while.
    bool firewall_hint(bool bonjour_advertising) const {
        return bonjour_advertising && phase != link_phase::connected &&
               silence_s > LINK_FIREWALL_HINT_S;
    }

    void consume_events() { just_connected = just_lost = false; }
};

// ------------------------------------------------------------------
// passthrough freshness
// ------------------------------------------------------------------

// Poses and camera frames travel independently: ARKit keeps posing after the
// camera stream dies, so without a guard the passthrough quad holds the last
// JPEG forever and world-locked panels swim over a frozen photo. Hands already
// expire (HAND_STALE_S in scene.cpp); frames now do too.
constexpr float FRAME_STALE_AFTER_S = 1.0f;
constexpr float FRAME_FADE_S = 0.3f;

// Passthrough opacity for a frame this old: 1 = show it, 0 = fully faded to
// the clear colour. A pure function of age, so a newly arrived frame (age ~0)
// snaps straight back to 1 — the fade is interruptible by construction.
inline float passthrough_fade(float frame_age_s) {
    if (frame_age_s <= FRAME_STALE_AFTER_S)
        return 1.0f;
    const float t = (frame_age_s - FRAME_STALE_AFTER_S) / FRAME_FADE_S;
    return t >= 1.0f ? 0.0f : 1.0f - t;
}

// ------------------------------------------------------------------
// toast queue
// ------------------------------------------------------------------

constexpr float TOAST_DURATION_S = 6.0f;
// Toasts carrying an action button stay up longer.
constexpr float TOAST_ACTION_DURATION_S = 10.0f;
constexpr size_t TOAST_QUEUE_MAX = 4;

enum class toast_action {
    none,
    open_screen_recording_settings,  // deep-link System Settings
};

struct toast {
    std::string text;
    toast_action action = toast_action::none;
    float remaining_s = TOAST_DURATION_S;
    float total_s = TOAST_DURATION_S;
};

// FIFO of short-lived notices. Oldest visible first; duplicate text while
// still visible is dropped (a failing launch retried five times is one
// toast, not five).
class toast_queue {
   public:
    void push(const std::string &text,
              toast_action action = toast_action::none) {
        for (const auto &t : toasts_)
            if (t.text == text)
                return;
        if (toasts_.size() >= TOAST_QUEUE_MAX)
            toasts_.pop_front();
        toast t;
        t.text = text;
        t.action = action;
        t.total_s = action == toast_action::none ? TOAST_DURATION_S
                                                 : TOAST_ACTION_DURATION_S;
        t.remaining_s = t.total_s;
        toasts_.push_back(t);
    }

    void tick(float dt) {
        for (auto &t : toasts_)
            t.remaining_s -= dt;
        while (!toasts_.empty() && toasts_.front().remaining_s <= 0.0f)
            toasts_.pop_front();
    }

    void dismiss_front() {
        if (!toasts_.empty())
            toasts_.pop_front();
    }

    bool empty() const { return toasts_.empty(); }
    size_t size() const { return toasts_.size(); }
    // Currently shown toast (the renderer displays one at a time).
    const toast *front() const {
        return toasts_.empty() ? nullptr : &toasts_.front();
    }

   private:
    std::deque<toast> toasts_;
};

// ------------------------------------------------------------------
// first-run persistence decisions
// ------------------------------------------------------------------

// The cheat sheet auto-shows for the first N windowed sessions, then only
// on demand (? in the dock, Cmd+/, Help menu).
constexpr int CHEATSHEET_AUTO_SESSIONS = 2;

// Values loaded from NSUserDefaults by the renderer (headless runs never
// touch persistence — these helpers stay pure).
struct first_run_prefs {
    bool seen_onboarding = false;
    int session_count = 0;  // incremented per windowed launch
};

inline bool should_show_onboarding(const first_run_prefs &p) {
    return !p.seen_onboarding;
}

inline bool should_autoshow_cheatsheet(const first_run_prefs &p) {
    return p.session_count <= CHEATSHEET_AUTO_SESSIONS;
}

}  // namespace mac_shell
