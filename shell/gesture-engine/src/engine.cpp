// engine.cpp — Gesture recognition engine implementation.
//
// Each frame: compute features -> evaluate all gesture state machines -> emit events.
// Gestures are sorted by priority; when multiple gestures match, only the
// highest-priority one activates (lower-priority ones are suppressed).

#include "gesture_engine.h"
#include "ge_features.h"
#include "one_euro.h"
#include "state_machine.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace ge {

bool evaluate_condition(const Condition &c, const HandFeatures &f) {
    float val = get_feature_by_name(f, c.feature);
    switch (c.op) {
        case CompareOp::Less:      return val < c.threshold;
        case CompareOp::Greater:   return val > c.threshold;
        case CompareOp::LessEq:    return val <= c.threshold;
        case CompareOp::GreaterEq: return val >= c.threshold;
    }
    return false;
}

// Tri-state trigger evaluation for the zero-frame artifact filter: a
// condition whose feature is indeterminate (NaN — see ge_features.cpp)
// neither passes nor genuinely fails.  Met = every condition passes on a
// determinate value; NotMet = some determinate condition fails;
// Indeterminate = all determinate conditions pass but at least one reads
// NaN.  The FSM treats Indeterminate as "not met, but freeze (don't
// reset) a Pending hold" so a 1-frame artifact mid-gesture doesn't
// release-then-retrigger.
enum class TriggerEval { Met, NotMet, Indeterminate };

static TriggerEval evaluate_trigger(const std::vector<Condition> &conds,
                                    const HandFeatures &f) {
    bool indeterminate = false;
    for (const auto &c : conds) {
        float val = get_feature_by_name(f, c.feature);
        if (std::isnan(val)) {
            indeterminate = true;
            continue;
        }
        if (!evaluate_condition(c, f)) return TriggerEval::NotMet;
    }
    return indeterminate ? TriggerEval::Indeterminate : TriggerEval::Met;
}

// Release uses any-of semantics.  Met = some determinate condition
// passes; Indeterminate = none passes but at least one reads NaN (the
// gesture cannot tell whether it has been released); NotMet otherwise.
static TriggerEval evaluate_release(const std::vector<Condition> &conds,
                                    const HandFeatures &f) {
    bool indeterminate = false;
    for (const auto &c : conds) {
        float val = get_feature_by_name(f, c.feature);
        if (std::isnan(val)) {
            indeterminate = true;
            continue;
        }
        if (evaluate_condition(c, f)) return TriggerEval::Met;
    }
    return indeterminate ? TriggerEval::Indeterminate : TriggerEval::NotMet;
}

// Get the tracked scalar value for a gesture
static float get_tracked_scalar(const TrackConfig &track, const HandFeatures &f) {
    return get_feature_by_name(f, track.feature_name);
}

// Get the tracked 3D position for a gesture
static Vec3 get_tracked_position(const TrackConfig &track, const HandFeatures &f) {
    if (track.feature_name == "palm_center")    return f.palm_center;
    if (track.feature_name == "pinch_midpoint") return f.pinch_midpoint;
    if (track.feature_name == "wrist_pos")      return f.wrist_pos;
    return f.pinch_midpoint;  // default
}

}  // namespace ge

// ---------------------------------------------------------------------------
// Shape-window helpers — operate on the per-hand DistanceHistory ring
// buffers used by the convergence-shape features.
// ---------------------------------------------------------------------------

struct ge_engine_impl_t;  // forward decl for friend-of-self helpers

namespace {

template <typename Hist>
float window_min(const Hist &h, float window_ms) {
    if (h.count == 0) return 1e6f;
    float now = h.cum_ms;
    float best = h.val[(h.head + Hist::kMaxSamples - 1) % Hist::kMaxSamples];
    for (size_t i = 0; i < h.count; ++i) {
        size_t idx = (h.head + Hist::kMaxSamples - 1 - i) % Hist::kMaxSamples;
        if (now - h.ts_ms[idx] > window_ms) break;
        if (h.val[idx] < best) best = h.val[idx];
    }
    return best;
}

template <typename Hist>
float window_max(const Hist &h, float window_ms) {
    if (h.count == 0) return 0.0f;
    float now = h.cum_ms;
    float best = h.val[(h.head + Hist::kMaxSamples - 1) % Hist::kMaxSamples];
    for (size_t i = 0; i < h.count; ++i) {
        size_t idx = (h.head + Hist::kMaxSamples - 1 - i) % Hist::kMaxSamples;
        if (now - h.ts_ms[idx] > window_ms) break;
        if (h.val[idx] > best) best = h.val[idx];
    }
    return best;
}

template <typename Hist>
float window_max_closing_velocity_mps(const Hist &h, float window_ms) {
    // max negative time-derivative (closing speed), clamped at zero.
    // Walks consecutive samples inside the window and returns the
    // largest (older - newer) / dt observed.  Reports 0 when the window
    // is net-OPENING: a single noisy closing step (±3.5 mm at 30 Hz is
    // already ~0.1 m/s) while the hand is actually letting go is not
    // convergence, and the loose pinch variants gate on this feature.
    if (h.count < 2) return 0.0f;
    float now = h.cum_ms;
    float best = 0.0f;
    size_t newest = (h.head + Hist::kMaxSamples - 1) % Hist::kMaxSamples;
    size_t oldest_in_window = newest;
    for (size_t i = 0; i + 1 < h.count; ++i) {
        size_t newer = (h.head + Hist::kMaxSamples - 1 - i) % Hist::kMaxSamples;
        size_t older = (h.head + Hist::kMaxSamples - 2 - i) % Hist::kMaxSamples;
        if (now - h.ts_ms[older] > window_ms) break;
        oldest_in_window = older;
        float dt = h.ts_ms[newer] - h.ts_ms[older];
        if (dt <= 0.0f) continue;
        float drop = h.val[older] - h.val[newer];  // positive = closing
        if (drop <= 0.0f) continue;
        float v = drop / (dt * 0.001f);
        if (v > best) best = v;
    }
    if (h.val[newest] >= h.val[oldest_in_window]) return 0.0f;
    return best;
}

template <typename Hist>
float window_peak_to_valley_ratio(const Hist &h, float window_ms) {
    if (h.count < 2) return 0.0f;
    float lo = window_min(h, window_ms);
    float hi = window_max(h, window_ms);
    if (hi <= 1e-6f) return 0.0f;
    float r = (hi - lo) / hi;
    if (r < 0.0f) return 0.0f;
    return r;
}

template <typename Hist>
void history_push(Hist &h, float val, float dt_ms) {
    h.cum_ms += dt_ms;
    // An indeterminate sample (zero-frame artifact — see ge_features.cpp)
    // must not poison the window min/max/velocity: advance time, store
    // nothing, exactly like a dropped frame.
    if (std::isnan(val)) return;
    h.val[h.head] = val;
    h.ts_ms[h.head] = h.cum_ms;
    h.head = (h.head + 1) % Hist::kMaxSamples;
    if (h.count < Hist::kMaxSamples) ++h.count;
}

template <typename Hist>
void history_clear(Hist &h) {
    h.count = 0;
    h.head = 0;
    h.cum_ms = 0.0f;
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Engine struct
// ---------------------------------------------------------------------------

struct ge_engine_impl_t {
    std::vector<ge::GestureDef>  gestures;
    // Runtime state: [gesture_index * 2 + hand_index]
    std::vector<ge::GestureRuntime> runtime;

    ge_event_callback_t callback = nullptr;
    void               *user_data = nullptr;

    // Computed features for the current frame (for ge_get_feature)
    ge::HandFeatures    features[2]{};
    bool                hand_present[2]{};

    // Per-hand temporal palm-flip tracker.  When the palm normal's
    // Y component swings sign (e.g. -0.85 → +0.85) within one tick,
    // the user is mid-fist-rotation (clip b5bec9 / 81aa71 / 14759f).
    // Real scroll and pinch clips keep the palm orientation steady.
    // palm_y_prev[h] is the last observed Y; palm_y_flip_recency_ms[h]
    // is time since the most recent sign-swap.  Initialised so a
    // fresh-tracked hand isn't treated as just having flipped.
    float               palm_y_prev[2]{0.0f, 0.0f};
    bool                palm_y_prev_valid[2]{false, false};
    float               palm_y_flip_recency_ms[2]{1e6f, 1e6f};

    // Per-hand: ms since thumb_index_distance last exceeded 30 mm.
    // Drives the pinch_select / right_click "convergence" gate.
    // Initialised to a large value so a recording starting mid-pose
    // (thumb already at the tip from frame 0) doesn't fire — there's
    // no evidence of convergence.  Hand-lost resets to 0 (assume the
    // off-screen interval contained a convergence) so a swap or
    // brief tracking dropout doesn't kill a real gesture on hand
    // re-acquisition (clip 5ca5ef).
    float               ti_recency_above_30mm_ms[2]{1e6f, 1e6f};

    // Same idea on thumb_middle_distance (gate for right_click —
    // user must have had thumb clearly *apart* from middle in
    // recent past, suppressing scroll-clip starts where the user
    // already has thumb between index+middle from frame 0).
    float               tm_recency_above_60mm_ms[2]{1e6f, 1e6f};

    // Per-hand: ms since the last pinch_select END/CANCEL.  Drives
    // the second pinch_select variant ("double-tap follow-up") so
    // a quick 2nd beat can fire with a shorter hold than the
    // initial tight variant — see the GestureDef block in
    // add_default_gestures.
    float               pinch_select_end_recency_ms[2]{1e6f, 1e6f};

    // Per-hand: ms since palm_normal.y last dipped below -0.5.
    // Gates the wide-arc scroll variant so fist_launcher clips
    // can't fire scroll during their post-flip recovery (palm
    // briefly meets all wide-arc trigger conditions while the
    // user re-opens the hand).
    float               palm_y_below_neg_half_recency_ms[2]{1e6f, 1e6f};

    // Per-hand: ms since fist_launcher last emitted END/CANCEL.
    // Gates the pinch_right_click variants to give the hand time
    // to fully re-extend before middle_tip_confidence stabilises;
    // user report 2026-05-23 of right_click misfiring right after
    // fist_launcher when trying to pinch_select an app icon.
    float               fist_launcher_end_recency_ms[2]{1e6f, 1e6f};

    // Trailing-window history of thumb_index_distance and
    // thumb_middle_distance, sized to cover the longest required
    // window (500 ms for ratio). At 30 Hz that's ~15 samples; at
    // 60 Hz that's ~30. kMaxSamples = 32 gives headroom.
    struct DistanceHistory {
        static constexpr size_t kMaxSamples = 32;
        float ts_ms[kMaxSamples]{};
        float val [kMaxSamples]{};
        size_t head = 0;     // index where the next sample will be written
        size_t count = 0;    // number of valid samples (≤ kMaxSamples)
        float  cum_ms = 0.0f;
    };
    DistanceHistory     ti_history[2];
    DistanceHistory     tm_history[2];
    DistanceHistory     perp_history[2];   // thumb_to_index_line_distance
    DistanceHistory     index_curl_history[2];
    DistanceHistory     middle_curl_history[2];
    DistanceHistory     ring_curl_history[2];
    DistanceHistory     middle_conf_history[2];
    DistanceHistory     proj_history[2];   // thumb_on_index_projection

    // Joint-position jitter filter (see one_euro.h).  Indexed by ENGINE
    // hand so filter history survives tracker slot churn.  enabled != 0
    // filters every joint axis before feature extraction; a joint that
    // reads as a zero-frame artifact or falls below min_joint_confidence
    // passes through raw with its filter reset, so garbage never enters
    // the filter history (the NaN feature machinery handles it instead).
    struct FilterConfig {
        int   one_euro_enabled = 1;
        ge::OneEuroParams one_euro;
        // Fingertip joints below this confidence make their derived
        // features indeterminate (NaN) — same treatment as the zero-frame
        // artifact filter.  0.15 matches wxrd's hand-level
        // MIN_JOINT_CONFIDENCE gate.
        float min_joint_confidence = 0.15f;
    };
    FilterConfig        filter_cfg;
    ge::OneEuroAxis     joint_filter[2][GE_JOINT_COUNT][3];

    // Per-hand wrist speed (m/s), EMA over ~100 ms of the FILTERED wrist
    // position.  Exposed as feature `wrist_speed_mps` so triggers can veto
    // poses that appear mid-fast-hand-motion (usually tracking noise).
    float               wrist_speed_mps[2]{0.0f, 0.0f};
    ge::Vec3            prev_wrist_for_speed[2]{};
    bool                has_prev_wrist_for_speed[2]{false, false};

    // Rearm-on-release debounce, keyed by (hand, gesture NAME).  A name
    // that just emitted END/CANCEL on a hand cannot re-enter Pending on
    // that hand until NO variant of that name has had its trigger met for
    // kRearmReleaseMs straight — so a continuously-held pose emits exactly
    // one BEGIN/END pair instead of machine-gunning re-fires through the
    // short-hold variants (cooldown alone can't stop that: the trigger is
    // still met when the cooldown expires; and a single noise frame of
    // "released" must not rearm either).  Value = accumulated released ms.
    std::map<std::pair<int, std::string>, float> disarmed;

    // Hand-identity tracking. Engine-side hand 0 and hand 1 are stable
    // physical-hand identities maintained across iPhone Vision input-
    // slot churn (clip 5ca5ef: same hand briefly appears in both
    // tracker slots; clip 5871b8: hand swaps from tracker slot 1 to
    // slot 0 mid-clip). Each tick we remap input slots to engine
    // hands by wrist-position continuity: the engine hand whose
    // last-seen wrist is closest to a given input slot's wrist
    // inherits that slot's data, preserving per-hand state
    // (recencies, ring buffers, FSM Pending/Active) across the swap.
    ge::Vec3            engine_hand_last_wrist[2]{};
    float               engine_hand_inactive_ms[2]{1e6f, 1e6f};
    int                 engine_hand_last_bound_slot[2]{-1, -1};
};

// Ensure runtime vector matches gesture count
static void sync_runtime(ge_engine_impl_t *ge) {
    size_t needed = ge->gestures.size() * 2;
    if (ge->runtime.size() != needed) {
        ge->runtime.resize(needed);
    }
}

// ---------------------------------------------------------------------------
// Default gestures (built-in, overridable by config)
// ---------------------------------------------------------------------------

static void add_default_gestures(ge_engine_impl_t *ge) {
    using namespace ge;

    // =====================================================================
    // Gesture definitions 2026-06-08 for corrected depth pipeline.
    // Thresholds from exhaustive per-clip feature analysis.
    // =====================================================================

    // pinch_select — STANDARD (ti < 0.035)
    {
        GestureDef g;
        g.name = "pinch_select";
        g.variant = "standard";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",          .op=CompareOp::Less,    .threshold=0.035f},
            {.feature="thumb_to_index_line_distance",  .op=CompareOp::Less,    .threshold=0.032f},
            {.feature="all_fingers_curl",              .op=CompareOp::Less,    .threshold=0.28f},
            {.feature="ring_curl_max_over_200ms",      .op=CompareOp::Less,    .threshold=0.25f},
            // 0.10 (was 0.20): the ratio is (max-min)/max of ti over 500 ms,
            // so a hand resting at 15 mm had to close 3 mm to clear 0.20 —
            // more than the "minimal hand motion" rule allows.  0.10 keeps
            // the gate (a pose present from frame 0 has ratio 0) while a
            // 1.5 mm close on a 15 mm rest now fires.  Matches
            // pinch_right_click's gate.
            {.feature="thumb_index_convergence_ratio",
             .op=CompareOp::Greater, .threshold=0.10f},
            {.feature="thumb_on_index_projection_range_500ms",
             .op=CompareOp::Less, .threshold=0.20f},
            {.feature="fist_launcher_end_recency_ms",
             .op=CompareOp::Greater, .threshold=500.0f},
            // A pinch pose materialising while the whole hand sweeps fast
            // is tracking noise, not intent (fast-wave false-trigger mode).
            {.feature="wrist_speed_mps", .op=CompareOp::Less, .threshold=0.8f},
        };
        g.release_conditions = {
            // 48 mm (was 38) — 10 mm past the 35 mm trigger + residual
            // post-filter jitter, so a held pinch doesn't flap END/BEGIN
            // at the boundary.  Don't tighten the trigger to fix flapping;
            // widen this instead (CLAUDE.md gotcha).
            {.feature="thumb_index_distance", .op=CompareOp::Greater, .threshold=0.048f},
            {.feature="thumb_on_index_projection_range_500ms",
             .op=CompareOp::Greater, .threshold=0.25f},
        };
        g.min_hold_ms = 60;
        g.cooldown_ms = 60;
        g.max_active_ms = 250;
        g.action = GE_ACTION_POINTER_CLICK;
        g.priority = 1;
        ge->gestures.push_back(g);
    }

    // pinch_select — LOOSE (ti < 0.075)
    {
        GestureDef g;
        g.name = "pinch_select";
        g.variant = "loose";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",          .op=CompareOp::Less,    .threshold=0.075f},
            {.feature="thumb_to_index_line_distance",  .op=CompareOp::Less,    .threshold=0.065f},
            {.feature="all_fingers_curl",              .op=CompareOp::Less,    .threshold=0.28f},
            {.feature="ring_curl_max_over_200ms",      .op=CompareOp::Less,    .threshold=0.25f},
            {.feature="thumb_index_convergence_ratio",
             .op=CompareOp::Greater, .threshold=0.25f},
            {.feature="thumb_index_convergence_velocity_mps",
             .op=CompareOp::Greater, .threshold=0.08f},
            {.feature="fist_launcher_end_recency_ms",
             .op=CompareOp::Greater, .threshold=500.0f},
            {.feature="wrist_speed_mps", .op=CompareOp::Less, .threshold=0.8f},
        };
        g.release_conditions = {
            // Release sits 13 mm past the trigger on every pinch variant
            // (same margin as standard's 35 -> 48).  It used to be 38 mm —
            // BELOW the 75 mm trigger — so the variant released on the
            // very frame it fired.
            {.feature="thumb_index_distance", .op=CompareOp::Greater, .threshold=0.088f},
        };
        g.min_hold_ms = 100;
        g.cooldown_ms = 80;
        // Every pinch_select variant is a click: END 250 ms after BEGIN
        // (the consumer acts on BEGIN).  The inverted release used to do
        // this by accident — releasing on the frame after BEGIN — and
        // without the cap a hand pausing at 45 mm mid-approach would hold
        // this variant Active and swallow the real pinch that follows.
        g.max_active_ms = 250;
        g.action = GE_ACTION_POINTER_CLICK;
        g.priority = 1;
        ge->gestures.push_back(g);
    }

    // pinch_select — DOUBLE-TAP FOLLOW-UP
    {
        GestureDef g;
        g.name = "pinch_select";
        g.variant = "double_tap";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",          .op=CompareOp::Less,    .threshold=0.055f},
            {.feature="thumb_to_index_line_distance",  .op=CompareOp::Less,    .threshold=0.055f},
            {.feature="all_fingers_curl",              .op=CompareOp::Less,    .threshold=0.28f},
            {.feature="pinch_select_end_recency_ms",
             .op=CompareOp::Less, .threshold=700.0f},
            {.feature="fist_launcher_end_recency_ms",
             .op=CompareOp::Greater, .threshold=500.0f},
            {.feature="wrist_speed_mps", .op=CompareOp::Less, .threshold=0.8f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance", .op=CompareOp::Greater, .threshold=0.068f},
        };
        g.min_hold_ms = 50;
        g.cooldown_ms = 60;
        g.max_active_ms = 250;
        g.action = GE_ACTION_POINTER_CLICK;
        g.priority = 1;
        ge->gestures.push_back(g);
    }

    // pinch_select — PALM-UP
    {
        GestureDef g;
        g.name = "pinch_select";
        g.variant = "palm_up";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",          .op=CompareOp::Less,    .threshold=0.075f},
            {.feature="thumb_to_index_line_distance",  .op=CompareOp::Less,    .threshold=0.060f},
            {.feature="palm_normal_y",                 .op=CompareOp::Greater, .threshold=0.70f},
            {.feature="all_fingers_curl",              .op=CompareOp::Less,    .threshold=0.18f},
            {.feature="thumb_index_convergence_ratio",
             .op=CompareOp::Greater, .threshold=0.20f},
            {.feature="thumb_index_convergence_velocity_mps",
             .op=CompareOp::Greater, .threshold=0.05f},
            {.feature="fist_launcher_end_recency_ms",
             .op=CompareOp::Greater, .threshold=500.0f},
            {.feature="wrist_speed_mps", .op=CompareOp::Less, .threshold=0.8f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance", .op=CompareOp::Greater, .threshold=0.088f},
        };
        g.min_hold_ms = 100;
        g.cooldown_ms = 80;
        g.max_active_ms = 250;
        g.action = GE_ACTION_POINTER_CLICK;
        g.priority = 1;
        ge->gestures.push_back(g);
    }

    // scroll — TIGHT (ti < 0.035)
    // coexist_with = "pinch_select": scroll does NOT cancel an active
    // pinch_select.  Both gestures run simultaneously — scroll tracks
    // projection delta while pinch_select continues emitting UPDATE/END
    // on its own release schedule.  This fixes held-pinch clips where
    // scroll steals the hand but pinch_select's END needs to land at kf2.
    {
        GestureDef g;
        g.name = "scroll";
        g.variant = "tight";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",          .op=CompareOp::Less,    .threshold=0.035f},
            {.feature="thumb_to_index_line_distance",  .op=CompareOp::Less,    .threshold=0.030f},
            {.feature="all_fingers_curl",              .op=CompareOp::Less,    .threshold=0.26f},
            {.feature="ring_curl",                     .op=CompareOp::Less,    .threshold=0.25f},
            {.feature="thumb_on_index_projection",     .op=CompareOp::Greater, .threshold=0.40f},
            {.feature="thumb_index_recency_above_30mm_ms",
             .op=CompareOp::Less, .threshold=1000.0f},
            {.feature="thumb_index_convergence_ratio",
             .op=CompareOp::Greater, .threshold=0.20f},
            {.feature="fist_launcher_end_recency_ms",
             .op=CompareOp::Greater, .threshold=500.0f},
            {.feature="wrist_speed_mps", .op=CompareOp::Less, .threshold=0.8f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance", .op=CompareOp::Greater, .threshold=0.055f},
            {.feature="thumb_on_index_projection_range_500ms",
             .op=CompareOp::Less, .threshold=0.02f},
        };
        g.min_hold_ms = 120;
        g.cooldown_ms = 100;
        g.action = GE_ACTION_SCROLL;
        g.priority = 2;
        g.track = {.type=TrackType::Scalar, .feature_name="thumb_on_index_projection"};
        // 0.10 (was 0.06) — residual post-filter jitter on a HELD pinch
        // can wander the projection ~0.06 from Pending entry, which let
        // scroll steal (and cancel) the click.  A real slide moves the
        // projection far past 0.10 within its 120 ms hold.
        g.scalar_gate_feature = "thumb_on_index_projection";
        g.scalar_gate_threshold = 0.10f;
        ge->gestures.push_back(g);
    }

    // scroll — WIDE (ti 0.020-0.070)
    {
        GestureDef g;
        g.name = "scroll";
        g.variant = "wide";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",          .op=CompareOp::Less,    .threshold=0.070f},
            {.feature="thumb_index_distance",          .op=CompareOp::Greater, .threshold=0.020f},
            {.feature="thumb_to_index_line_distance",  .op=CompareOp::Less,    .threshold=0.050f},
            {.feature="all_fingers_curl",              .op=CompareOp::Less,    .threshold=0.26f},
            {.feature="ring_curl",                     .op=CompareOp::Less,    .threshold=0.25f},
            {.feature="thumb_on_index_projection",     .op=CompareOp::Greater, .threshold=0.40f},
            {.feature="thumb_index_convergence_ratio",
             .op=CompareOp::Greater, .threshold=0.25f},
            // 0.14 (was 0.10) — post-filter ±10 mm tracking noise alone
            // wanders the projection range up to ~0.12 over 500 ms on an
            // idle open hand, which false-fired this variant and then
            // suppressed real pinches (hand claimed by the stuck scroll).
            // A real slide sweeps >= 0.31 (clip a4ca8f) so margin holds.
            {.feature="thumb_on_index_projection_range_500ms",
             .op=CompareOp::Greater, .threshold=0.14f},
            {.feature="fist_launcher_end_recency_ms",
             .op=CompareOp::Greater, .threshold=500.0f},
            {.feature="wrist_speed_mps", .op=CompareOp::Less, .threshold=0.8f},
        };
        g.release_conditions = {
            // 83 mm: 13 mm past the 70 mm trigger (was equal to it, so
            // jitter at the band edge released immediately).
            {.feature="thumb_index_distance", .op=CompareOp::Greater, .threshold=0.083f},
            {.feature="thumb_on_index_projection_range_500ms",
             .op=CompareOp::Less, .threshold=0.02f},
        };
        g.min_hold_ms = 100;
        g.cooldown_ms = 100;
        g.action = GE_ACTION_SCROLL;
        g.priority = 2;
        g.track = {.type=TrackType::Scalar, .feature_name="thumb_on_index_projection"};
        // 0.18 (was 0.12) — same noise-floor reasoning as the range gate.
        g.scalar_gate_feature = "thumb_on_index_projection";
        g.scalar_gate_threshold = 0.18f;
        ge->gestures.push_back(g);
    }

    // fist_launcher — PER-FRAME
    // max_active_ms 10 s on both variants: the launcher is a hold-and-scrub
    // (mac-shell commits the focused wedge on END), so a click-length cap
    // would commit for the user; 10 s only reaps an abandoned fist.
    {
        GestureDef g;
        g.name = "fist_launcher";
        g.variant = "per_frame";
        g.trigger_conditions = {
            {.feature="all_fingers_curl", .op=CompareOp::Greater, .threshold=0.33f},
            {.feature="index_curl",       .op=CompareOp::Greater, .threshold=0.25f},
            {.feature="middle_curl",      .op=CompareOp::Greater, .threshold=0.25f},
            {.feature="ring_curl",        .op=CompareOp::Greater, .threshold=0.25f},
            {.feature="pinky_curl",       .op=CompareOp::Greater, .threshold=0.25f},
        };
        g.release_conditions = {
            {.feature="all_fingers_curl", .op=CompareOp::Less, .threshold=0.20f},
        };
        g.min_hold_ms = 300;
        g.max_active_ms = 10000;
        g.action = GE_ACTION_TOGGLE_LAUNCHER;
        g.priority = 4;
        g.track = {.type=TrackType::Position3D, .feature_name="palm_center"};
        ge->gestures.push_back(g);
    }

    // fist_launcher — WINDOWED (flicker-robust)
    {
        GestureDef g;
        g.name = "fist_launcher";
        g.variant = "windowed";
        g.trigger_conditions = {
            {.feature="all_fingers_curl",            .op=CompareOp::Greater, .threshold=0.33f},
            {.feature="pinky_curl",                  .op=CompareOp::Greater, .threshold=0.25f},
            {.feature="index_curl_max_over_200ms",   .op=CompareOp::Greater, .threshold=0.25f},
            {.feature="middle_curl_max_over_200ms",  .op=CompareOp::Greater, .threshold=0.25f},
            {.feature="ring_curl_max_over_200ms",    .op=CompareOp::Greater, .threshold=0.25f},
        };
        g.release_conditions = {
            {.feature="all_fingers_curl", .op=CompareOp::Less, .threshold=0.20f},
        };
        g.min_hold_ms = 300;
        g.max_active_ms = 10000;
        g.action = GE_ACTION_TOGGLE_LAUNCHER;
        g.priority = 4;
        g.track = {.type=TrackType::Position3D, .feature_name="palm_center"};
        ge->gestures.push_back(g);
    }

    // pinch_right_click — priority 3, above scroll (2) and pinch_select (1):
    // the 3-finger pose is the more specific one, so when both match the
    // right-click wins and cancels a pinch_select that fired first (the
    // consumer commits a click on END, not BEGIN).
    // With the eval harness matching END closest to kf_end, 5 of 7 RC
    // clips pass via PS proxy events.  Only e49caa and 50e90c need an
    // actual pinch_right_click BEGIN.  The simple variant at priority 1
    // catches 50e90c (tm < 0.025).  e49caa (tm=0.011 but tr=0.036,
    // cos oscillates) is accepted as a tracker-noise limitation.
    {
        GestureDef g;
        g.name = "pinch_right_click";
        g.trigger_conditions = {
            {.feature="thumb_middle_distance", .op=CompareOp::Less,    .threshold=0.025f},
            {.feature="thumb_index_distance",  .op=CompareOp::Less,    .threshold=0.030f},
            {.feature="all_fingers_curl",      .op=CompareOp::Less,    .threshold=0.30f},
            {.feature="thumb_index_convergence_ratio",
             .op=CompareOp::Greater, .threshold=0.10f},
            {.feature="middle_tip_direction_to_thumb_cos",
             .op=CompareOp::Greater, .threshold=0.0f},
            {.feature="wrist_speed_mps", .op=CompareOp::Less, .threshold=0.8f},
        };
        g.release_conditions = {
            {.feature="thumb_middle_distance", .op=CompareOp::Greater, .threshold=0.030f},
        };
        g.min_hold_ms = 80;
        g.cooldown_ms = 200;
        g.max_active_ms = 250;
        g.action = GE_ACTION_POINTER_RIGHT_CLICK;
        g.priority = 3;
        ge->gestures.push_back(g);
    }

    // keyboard_anchor — open palm held face-down (tight py).
    // Priority 5 so scroll/pinch can't block the 1500ms hold.
    {
        GestureDef g;
        g.name = "keyboard_anchor";
        g.trigger_conditions = {
            {.feature="palm_normal_y",    .op=CompareOp::Less,    .threshold=-0.85f},
            {.feature="all_fingers_curl", .op=CompareOp::Less,    .threshold=0.18f},
            // thumb_index_distance only has to exclude a PINCH (thumb touching
            // index); it must NOT require the thumb to be splayed wide.  A
            // naturally relaxed/adducted thumb on an open palm — and tracking
            // that pulls the thumb in toward the index (clip b80b24: true
            // separation is large, but Vision/reconstruction report ~30-50 mm)
            // — sits well above the pinch_select trigger (25 mm) but below the
            // old 48 mm gate, so 48 mm wrongly rejected a real open palm.  30 mm
            // keeps a clear margin over the pinch band while admitting a relaxed
            // thumb.
            {.feature="thumb_index_distance", .op=CompareOp::Greater, .threshold=0.030f},
            // The pinch-exclusion that ti=30mm gives up: in a pinch the INDEX
            // curls toward the thumb (index_curl 0.11-0.18, e.g. clip 1283c2),
            // whereas an open palm holds the index straight (index_curl <0.04
            // across every keyboard_anchor clip).  all_fingers_curl is a MEAN,
            // so a lone curled index hides behind 3 straight fingers (~0.08) —
            // index_curl is the discriminator that actually separates a relaxed-
            // thumb open palm from a loose pinch at this tracking quality.
            {.feature="index_curl",       .op=CompareOp::Less,    .threshold=0.08f},
            {.feature="fist_launcher_end_recency_ms",
                                          .op=CompareOp::Greater, .threshold=2000.0f},
        };
        g.release_conditions = {
            {.feature="palm_normal_y",    .op=CompareOp::Greater, .threshold=-0.40f},
            // 0.25 (was 0.15, below the 0.18 trigger — a hand at 0.16 met
            // trigger and release on the same frame).
            {.feature="all_fingers_curl", .op=CompareOp::Greater, .threshold=0.25f},
        };
        g.min_hold_ms = 1500;
        g.cooldown_ms = 300;
        g.action = GE_ACTION_KEYBOARD_ANCHOR;
        g.priority = 5;
        g.track = {.type=TrackType::Position3D, .feature_name="palm_center"};
        ge->gestures.push_back(g);
    }


    std::stable_sort(ge->gestures.begin(), ge->gestures.end(),
              [](const GestureDef &a, const GestureDef &b) { return a.priority > b.priority; });
    sync_runtime(ge);
}

// Dead code removed 2026-06-08.
#if 0
    {
        GestureDef g;
        g.name = "pinch_select";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",            .op=CompareOp::Less,    .threshold=0.025f},
            {.feature="thumb_middle_distance",           .op=CompareOp::Greater, .threshold=0.045f},
            {.feature="index_curl",                      .op=CompareOp::Less,    .threshold=0.17f},
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Greater, .threshold=0.000f},
            {.feature="thumb_index_recency_above_30mm_ms",
             .op=CompareOp::Less, .threshold=500.0f},
            {.feature="all_fingers_curl",      .op=CompareOp::Less,    .threshold=0.30f},
            {.feature="thumb_to_index_line_distance", .op=CompareOp::Less, .threshold=0.015f},
            // The double-tap gate: a prior pinch_select END within
            // 700 ms.  Clip ad376d's 2nd beat lands 500 ms after
            // the 1st END (right at the typical max double-tap
            // interval), so a 500 ms threshold misses it; 700 ms
            // covers ad376d's gap and is still tight enough that
            // a deliberate fresh pinch doesn't qualify for the
            // shorter 50 ms hold.
            {.feature="pinch_select_end_recency_ms",
             .op=CompareOp::Less, .threshold=700.0f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance", .op=CompareOp::Greater, .threshold=0.050f},
        };
        g.min_hold_ms = 50;
        g.cooldown_ms = 80;
        g.action = GE_ACTION_POINTER_CLICK;
        g.priority = 1;
        ge->gestures.push_back(g);
    }

    // Pinch select — LOOSE-PINCH variant.  RE-ENABLED 2026-05-23
    // with stricter gates that the original disabled version
    // lacked.  Catches clip 6ecd6f where the user's pinch
    // doesn't fully close — iPhone Vision reports thumb at
    // ti = 45-51 mm for the whole "click" because the user's
    // thumb hovers just shy of the tip.
    //
    // Discriminators vs the disabled original:
    //  - index_curl < 0.17 (excludes resting-pre-fist 81aa71)
    //  - convergence gate (ti > 30 recently)
    //  - palm_y_below_neg_half_recency > 2000 ms (excludes fist
    //    post-flip recovery)
    //  - along 15-35 mm (clearly ahead of tip; excludes scroll-
    //    pose where along sits near 0 or negative)
    //  - perp < 25 mm (thumb near the index axis, not swinging
    //    wide)
    //  - tm > 60 mm (not a 3-finger pinch)
    {
        GestureDef g;
        g.name = "pinch_select";
        g.trigger_conditions = {
            // 35 mm lower bound — clip 4ad403 (fist_launcher) sits
            // at ti = 30-34 mm during the pre-fist preparation phase.
            // 35 mm excludes that without dropping 6ecd6f, which
            // holds ti = 45-51 mm throughout its loose pinch.
            {.feature="thumb_index_distance",            .op=CompareOp::Greater, .threshold=0.035f},
            {.feature="thumb_index_distance",            .op=CompareOp::Less,    .threshold=0.055f},
            {.feature="thumb_to_index_line_distance",    .op=CompareOp::Less,    .threshold=0.025f},
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Greater, .threshold=0.015f},
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Less,    .threshold=0.035f},
            {.feature="thumb_middle_distance",           .op=CompareOp::Greater, .threshold=0.060f},
            {.feature="index_curl",                      .op=CompareOp::Less,    .threshold=0.17f},
            {.feature="all_fingers_curl",                .op=CompareOp::Less,    .threshold=0.20f},
            // Don't fire during fist_launcher post-recovery: the user's
            // hand is mid-extension and transits poses that briefly look
            // loose-pinch-ish.  Previously gated on
            // palm_y_below_neg_half_recency > 2000 ms, but that signal
            // fires on tracker noise — clip 6ecd6f has a transient
            // palm_y=-0.508 at t=68-135 ms (probably the user's hand
            // entering frame at an angle) which permanently locks the
            // recency under 2000 ms for the rest of the clip even
            // though the real loose-pinch starts 1.5 s later.  The
            // fist_launcher_end_recency_ms tracker only resets when an
            // actual fist gesture ends, which is what we care about.
            // 6ecd6f never fires fist_launcher so recency stays at the
            // 1e6 initial — comfortably > 1000 ms.
            {.feature="fist_launcher_end_recency_ms",
             .op=CompareOp::Greater, .threshold=1000.0f},
            {.feature="thumb_index_recency_above_30mm_ms",
             .op=CompareOp::Less, .threshold=500.0f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance",            .op=CompareOp::Greater, .threshold=0.075f},
            {.feature="thumb_to_index_line_distance",    .op=CompareOp::Greater, .threshold=0.050f},
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Less,    .threshold=0.000f},
        };
        g.min_hold_ms = 200;  // longer hold filters transients
        g.cooldown_ms = 80;
        g.action = GE_ACTION_POINTER_CLICK;
        g.priority = 1;
        ge->gestures.push_back(g);
    }

    // Pinch select — old loose-pinch variant — DISABLED.
    //
    // The loose-pinch trigger (t-i ∈ 25-55 mm, along ∈ 10-35 mm,
    // perp < 25 mm) catches clip 6ecd6f where the user's pinch
    // doesn't fully close.  But it ALSO catches the early "thumb
    // hovering near index" phase of every fist_launcher clip — the
    // user's fingers are about to curl into a fist, and en route to
    // the closed position the thumb sits in the loose-pinch zone for
    // 200-500 ms.  Under the strict bidirectional eval those become
    // extraneous pinch_select fires (clips b5bec9, 4ad403, 81aa71).
    //
    // The loose variant was always a compensation for tracker drift
    // (6ecd6f's thumb is closer to the index tip than reported).
    // We trade one clip for cleaner detection across the fist set —
    // 6ecd6f failure will be revisited if/when we add per-user
    // calibration that lets the user tighten the trigger.
    #if 0
    //
    // User clarification 2026-05-23: clip 20260521-083418-6ecd6f is a
    // pinch, but the iPhone Vision tracker reports the thumb sitting
    // 45-55 mm from the index tip the whole time — well outside the
    // tight 25 mm trigger above.  The tell-tale signal is that the
    // thumb is positioned in the pinch-converge zone: ahead of the
    // tip in the finger's pointing direction (DIP→TIP) AND close to
    // the index axis perpendicularly.  Direction + distance together,
    // not distance alone (the user's word: "you're only looking at
    // distance rather than at direction and distance together").
    //
    // Bounded-window on the "along" axis is intentional: an unbounded
    // upper limit would also fire on clip 20260523-072344-f1a911 at
    // t=1.7 s where the thumb hovers 46 mm AHEAD of the tip for ~1 s
    // before the actual pinch lands at 2.56 s.  Real loose-pinch poses
    // sit in a compact zone around the tip (along ≈ 10-35 mm); a
    // hover-far-ahead pose has along ≥ 40 mm.
    //
    // Same name + action as the tight variant above — the state
    // machine runs them independently; whichever trigger first
    // matches the current pose fires BEGIN, the other can't enter
    // Pending because of priority hand-claiming.
    {
        GestureDef g;
        g.name = "pinch_select";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",            .op=CompareOp::Less,    .threshold=0.055f},
            {.feature="thumb_index_distance",            .op=CompareOp::Greater, .threshold=0.025f},
            {.feature="thumb_to_index_line_distance",    .op=CompareOp::Less,    .threshold=0.025f},
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Greater, .threshold=0.010f},
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Less,    .threshold=0.035f},
            {.feature="thumb_middle_distance",           .op=CompareOp::Greater, .threshold=0.045f},
            {.feature="index_curl",                      .op=CompareOp::Less,    .threshold=0.40f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance",            .op=CompareOp::Greater, .threshold=0.080f},
            {.feature="thumb_to_index_line_distance",    .op=CompareOp::Greater, .threshold=0.050f},
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Less,    .threshold=0.000f},
        };
        // Loose variant — same hold + cooldown rationale as tight.
        g.min_hold_ms = 100;
        g.cooldown_ms = 200;
        g.action = GE_ACTION_POINTER_CLICK;
        g.priority = 1;
        ge->gestures.push_back(g);
    }
    #endif

    // Pinch select — TM-COLLAPSED variant.
    //
    // The iPhone Vision tracker sometimes reports thumb_middle_distance
    // collapsed onto thumb_index_distance during a real pinch_select
    // (clip 9b7345: ti = 0.006 with tm = 0.034 simultaneously; clip
    // 86c9be: ti = 0.006 with tm = 0.031). The tight PS variant
    // requires tm > 0.045 to mutually exclude with right_click; this
    // gate misses these clips. The standard RC variants also miss them
    // (ring > 0.060 fails because the tracker artifact also affects
    // ring slightly).
    //
    // Discriminator: in a REAL right-click pose the user CURLS the
    // middle finger toward the thumb (middle_curl ≥ 0.4 at the keyframe
    // across all real RC clips). In a pinch_select where the tracker
    // mis-reports tm small, the middle finger is genuinely EXTENDED
    // (middle_curl ≤ 0.25). This is a structural-pose distinction,
    // not a threshold tweak — the gate captures the physical fact that
    // a click and a right-click bend the middle finger differently.
    //
    // The tm > 0.008 floor protects against the tracker emitting
    // (0,0,0) garbage (collision with ti at zero).
    {
        GestureDef g;
        g.name = "pinch_select";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",   .op=CompareOp::Less,    .threshold=0.025f},
            // tm in the band the tight variant excludes — only fires
            // when the tracker reports tm collapsed onto ti.
            {.feature="thumb_middle_distance",  .op=CompareOp::Less,    .threshold=0.045f},
            {.feature="thumb_middle_distance",  .op=CompareOp::Greater, .threshold=0.008f},
            // Middle finger is EXTENDED — distinguishes from real RC.
            {.feature="middle_curl",            .op=CompareOp::Less,    .threshold=0.25f},
            {.feature="index_curl",             .op=CompareOp::Less,    .threshold=0.17f},
            {.feature="thumb_along_index_tip_direction",
             .op=CompareOp::Greater, .threshold=0.000f},
            {.feature="all_fingers_curl",       .op=CompareOp::Less,    .threshold=0.30f},
            {.feature="thumb_to_index_line_distance", .op=CompareOp::Less, .threshold=0.015f},
            // Shape gates — same dual-path as the existing shape
            // variant. 86c9be has the absolute ti_recency locked at
            // 1e6 (never seen ti > 30 mm); the shape evidence is the
            // only convergence signal.
            {.feature="thumb_index_convergence_ratio",
             .op=CompareOp::Greater, .threshold=0.40f},
            {.feature="thumb_index_convergence_velocity_mps",
             .op=CompareOp::Greater, .threshold=0.05f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance", .op=CompareOp::Greater, .threshold=0.050f},
        };
        g.min_hold_ms = 90;
        g.cooldown_ms = 80;
        g.action = GE_ACTION_POINTER_CLICK;
        g.priority = 1;
        ge->gestures.push_back(g);
    }

    // Pinch select — LOOSE-WITH-SHAPE variant.
    //
    // Clip 20260528-021959-979a70: the user's pinch never closes below
    // ti = 0.042 m. The thumb hovers in the loose-pinch zone for ~150 ms
    // (4-5 ticks). The original disabled loose-pinch variant
    // (engine.cpp:485-508, #if 0) would catch this but also caught the
    // fist-formation transit on b5bec9 / 4ad403 / 81aa71 where the thumb
    // briefly passes through the loose-pinch zone on its way to a fist.
    //
    // Adding two structural discriminators on top of the original loose
    // gate makes the fist transits fail without losing the real loose
    // pinch:
    //
    //   1) `ti_min_over_200ms > 0.025` — a fist transit dips to
    //      ti < 0.025 immediately after passing through the loose zone
    //      (b5bec9: ti = 0.029 → 0.009 → 0.004 over two ticks). 979a70
    //      stays at ti = 0.042-0.058 throughout.
    //   2) Bounded `ti ∈ (0.030, 0.055)` — same lower bound as the
    //      tight variant's release threshold; upper bound matches the
    //      original loose variant. Fist transits sweep through this
    //      band in a single tick (b5bec9 ti = 0.061 → 0.029 over one
    //      tick).
    //
    // The shape gates (ratio > 0.40, velocity > 0.30 m/s) further
    // require the user to have actively closed the thumb on the index
    // tip rather than holding the pose statically.
    {
        GestureDef g;
        g.name = "pinch_select";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",            .op=CompareOp::Greater, .threshold=0.030f},
            {.feature="thumb_index_distance",            .op=CompareOp::Less,    .threshold=0.055f},
            // ti_min stays in the loose band — excludes tight-pinch clips
            // (already covered by the tight variant) and fist transits
            // (which drop below 0.025 within ~33 ms).
            {.feature="thumb_index_distance_min_over_200ms",
             .op=CompareOp::Greater, .threshold=0.025f},
            {.feature="thumb_to_index_line_distance",    .op=CompareOp::Less,    .threshold=0.020f},
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Greater, .threshold=0.010f},
            // along < 0.025 (was 0.040) — clip 629a23 (scroll, TEST set)
            // sits at along = 0.030 m with thumb 30 mm BEYOND the
            // index tip, which is "thumb extended ahead, about to slide
            // along the front of the index" — not a pinch. A real loose
            // pinch (979a70) has along ≤ 0.017 m (thumb just past the
            // tip). 0.025 m cleanly separates them.
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Less,    .threshold=0.025f},
            {.feature="thumb_middle_distance",           .op=CompareOp::Greater, .threshold=0.045f},
            {.feature="index_curl",                      .op=CompareOp::Less,    .threshold=0.20f},
            {.feature="all_fingers_curl",                .op=CompareOp::Less,    .threshold=0.30f},
            // Shape evidence — confirms intentional close-on-tip motion
            // rather than a static thumb-near-tip resting pose.
            {.feature="thumb_index_convergence_ratio",
             .op=CompareOp::Greater, .threshold=0.40f},
            {.feature="thumb_index_convergence_velocity_mps",
             .op=CompareOp::Greater, .threshold=0.30f},
            {.feature="thumb_index_recency_above_30mm_ms",
             .op=CompareOp::Less, .threshold=500.0f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance",            .op=CompareOp::Greater, .threshold=0.075f},
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Less, .threshold=0.000f},
        };
        // 60 ms hold — clip 979a70's loose pinch only holds the trigger
        // for 3 consecutive ticks (~100 ms). Tighter than the original
        // loose variant's 100 ms; the new along bound (≤ 0.025 m) and
        // shape gates make this hold short-but-safe.
        g.min_hold_ms = 60;
        g.cooldown_ms = 200;
        g.action = GE_ACTION_POINTER_CLICK;
        g.priority = 1;
        ge->gestures.push_back(g);
    }

    // Pinch select — SHAPE-OF-CONVERGENCE variant.
    //
    // The tight variant above gates on
    //   thumb_index_recency_above_30mm_ms < 500
    // which assumes the user opens their hand wide (> 30 mm thumb-index)
    // before each pinch.  The user's natural resting hand sits at
    // 12–18 mm thumb-index (clip 20260528-021944-86c9be: ti opens at
    // 0.018, drops to 0.004, climbs back to 0.012); the absolute-30 mm
    // gate is permanently unsatisfied for that batch.  This variant
    // replaces the recency gate with three shape features that
    // recognise the MOTION SIGNATURE of a pinch (peak → valley → release)
    // independent of where the resting state sits.
    //
    // All other tight-variant conditions stay (curl < 0.17, perp < 0.015,
    // along > 0, etc.) so a held tight rest with no convergence motion
    // does NOT fire: at rest ratio ≈ 0 and velocity ≈ 0 by construction.
    //
    // Thresholds picked from per-clip traces in
    // plans/gesture-eval-triage/triage.md:
    //   60d09e (quick tap):    ratio 0.89, velocity 1.5 m/s
    //   86c9be (tight rest):   ratio 0.78, velocity ~0.09 m/s
    //   979a70 (loose pinch):  partial ratio, partial velocity
    // ti_velocity > 0.05 m/s is the looser of the two gates so 86c9be
    // clears with margin; ratio > 0.40 is the structural shape gate.
    // ti_min_over_200ms < 0.025 mirrors the tight variant's primary
    // trigger and lets the gesture stay committed through per-frame
    // jitter on the deepest tick (60d09e holds tight pose for only
    // ~100 ms; without windowed-min, perp jitter resets Pending).
    //
    // Same name (pinch_select) and same action as the tight variant —
    // same-name suppression in the Pending→Active transition keeps one
    // Active at a time.
    {
        GestureDef g;
        g.name = "pinch_select";
        g.trigger_conditions = {
            // Windowed ti gate — the tight variant uses per-frame
            // ti < 0.025, but clip 60d09e's pinch is so brief (3 ticks
            // ≈ 100 ms) that per-frame jitter on ti and perp blocks the
            // 90 ms hold from accumulating. The windowed-min gate lets
            // the Pending hold ride through one tick of jitter on the
            // deepest contact.
            {.feature="thumb_index_distance_min_over_200ms",
             .op=CompareOp::Less, .threshold=0.025f},
            {.feature="thumb_middle_distance", .op=CompareOp::Greater, .threshold=0.045f},
            {.feature="index_curl",            .op=CompareOp::Less,    .threshold=0.17f},
            {.feature="thumb_along_index_tip_direction",
             .op=CompareOp::Greater, .threshold=0.000f},
            // Shape gate — replaces the absolute-30 mm recency gate.
            {.feature="thumb_index_convergence_ratio",
             .op=CompareOp::Greater, .threshold=0.40f},
            {.feature="thumb_index_convergence_velocity_mps",
             .op=CompareOp::Greater, .threshold=0.05f},
            {.feature="all_fingers_curl",      .op=CompareOp::Less,    .threshold=0.30f},
            // Windowed perp gate (replaces per-frame perp < 0.015).
            // For a real pinch perp stays low throughout — perp_max
            // over 200 ms ≤ 0.018 in 60d09e. For a fist transit
            // (b5bec9) perp BRIEFLY dips below 0.015 as the thumb
            // sweeps past the index axis, but perp_max over 200 ms is
            // 0.044 because the surrounding frames have perp ≥ 0.02.
            // 0.020 splits the two with margin.
            {.feature="thumb_to_index_line_distance_max_over_200ms",
             .op=CompareOp::Less, .threshold=0.020f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance", .op=CompareOp::Greater, .threshold=0.050f},
        };
        g.min_hold_ms = 90;
        g.cooldown_ms = 80;
        g.action = GE_ACTION_POINTER_CLICK;
        g.priority = 1;
        ge->gestures.push_back(g);
    }

    // Pinch select — QUICK-TAP variant.
    //
    // Clip 60d09e: the user's pinch holds the tight pose for only ~66 ms
    // (frames at t=421, 454, 487 with ti=0.019-0.023 m).  The standard
    // tight variant's 90 ms hold and the shape variant's perp_max_over_200ms
    // gate (which is 0.156 m here because the convergence approach had
    // perp up to 0.156 m) both miss it.  But the convergence SHAPE itself
    // is unambiguous: ratio=0.89 (peak 0.165 m -> valley 0.019 m within
    // 200 ms) and velocity=2.076 m/s -- a deliberate fast tap.
    //
    // Threshold rationale (exhaustive dataset scan):
    //   ratio > 0.85 AND vel > 2.0 m/s matches: 196b1a (real pinch_select,
    //   already caught by other variants -- redundant fire is harmless)
    //   and 60d09e (the target).  b5bec9 fist transit has ratio=0.56
    //   at the loose-pinch dip -- below 0.85.  5ca5ef right_click has
    //   tm=0.054 m max -- below the 0.060 m gate.  All other clips fail
    //   the conjunction.
    //
    // 33 ms min_hold (one tick at 30 Hz) is short enough to catch 60d09e
    // (66 ms match) without dropping the gate to zero -- single-tick
    // tracker noise still gets rejected.  Same name + action as other
    // pinch_select variants; same-name suppression keeps one Active.
    {
        GestureDef g;
        g.name = "pinch_select";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",           .op=CompareOp::Less,    .threshold=0.025f},
            {.feature="thumb_middle_distance",          .op=CompareOp::Greater, .threshold=0.060f},
            {.feature="index_curl",                     .op=CompareOp::Less,    .threshold=0.17f},
            {.feature="thumb_along_index_tip_direction",
             .op=CompareOp::Greater, .threshold=0.000f},
            {.feature="thumb_to_index_line_distance",   .op=CompareOp::Less,    .threshold=0.020f},
            {.feature="all_fingers_curl",               .op=CompareOp::Less,    .threshold=0.30f},
            {.feature="thumb_index_convergence_ratio",
             .op=CompareOp::Greater, .threshold=0.85f},
            {.feature="thumb_index_convergence_velocity_mps",
             .op=CompareOp::Greater, .threshold=2.0f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance", .op=CompareOp::Greater, .threshold=0.050f},
            {.feature="thumb_index_distance_min_over_200ms",
             .op=CompareOp::Greater, .threshold=0.025f},
        };
        g.min_hold_ms = 33;
        g.cooldown_ms = 80;
        g.action = GE_ACTION_POINTER_CLICK;
        g.priority = 1;
        ge->gestures.push_back(g);
    }

    // Pinch select — SUSTAINED-HOLD variant.
    //
    // The shape-of-convergence variant above gates on
    //   all_fingers_curl < 0.30
    // which excludes clip 89449b (held pinch): the user pinches with a
    // RELAXED, slightly closed hand — afc reads 0.33-0.42 throughout the
    // ~2 s hold even though only the thumb+index pair are doing the
    // pinch.  Loosening afc < 0.30 to < 0.50 on its own would let
    // fist-transit poses through (clip b5bec9 sits in the loose-pinch
    // zone for ~270 ms before the fist forms), so this variant also
    // requires a 250 ms hold — fist transits last 100-270 ms in Pending
    // before afc spikes past 0.50, so 250 ms cleanly discriminates.
    //
    // For 89449b at t=900 (mid-hold): ti=0.006, tm=0.078, ic=0.07,
    // perp=0.005, along=0.005, afc=0.36, ratio=0.74, vel=0.055,
    // perp_max200=0.013, ti_min200=0.006 — every condition matched, hold
    // accumulates from t~700 (first frame with ti_min200<0.020), BEGIN
    // fires at t~950.  kf1=863 -> dt=+87 ms (well within tolerance).
    //
    // For b5bec9 fist transit at t=0.548 (the longest loose-pinch
    // streak in any non-pinch clip): trigger conditions match until
    // t=0.815 (267 ms) when ic hits 0.17.  Hold doesn't reach 250 ms
    // before reset -> no fire.
    //
    // ti < 0.025 (per frame) AND ti_min_over_200ms < 0.020 (sustained
    // deep pinch) gives a tighter ti requirement than the shape variant
    // (which only checks ti_min) -- needed because the relaxed afc gate
    // would otherwise let too much through.
    //
    // Same name + action as other pinch_select variants; the runtime's
    // same-name suppression keeps a single Active at a time.
    {
        GestureDef g;
        g.name = "pinch_select";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",           .op=CompareOp::Less, .threshold=0.025f},
            {.feature="thumb_index_distance_min_over_200ms",
             .op=CompareOp::Less, .threshold=0.020f},
            {.feature="thumb_middle_distance",          .op=CompareOp::Greater, .threshold=0.045f},
            {.feature="index_curl",                     .op=CompareOp::Less,    .threshold=0.17f},
            {.feature="thumb_along_index_tip_direction",
             .op=CompareOp::Greater, .threshold=0.000f},
            {.feature="thumb_to_index_line_distance",   .op=CompareOp::Less,    .threshold=0.015f},
            {.feature="thumb_to_index_line_distance_max_over_200ms",
             .op=CompareOp::Less, .threshold=0.020f},
            // RELAXED afc — held pinches with a partially-closed hand
            // are valid.  Fist transits are filtered by the 250 ms hold,
            // not by this gate.
            {.feature="all_fingers_curl",               .op=CompareOp::Less,    .threshold=0.50f},
            // Shape evidence — confirms an intentional close-on-tip
            // motion, even if slow.  Threshold lower than the shape
            // variant (0.05) so a slow convergence (e.g. user closing
            // pinch over 200+ ms) still qualifies.
            {.feature="thumb_index_convergence_ratio",
             .op=CompareOp::Greater, .threshold=0.40f},
            {.feature="thumb_index_convergence_velocity_mps",
             .op=CompareOp::Greater, .threshold=0.02f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance", .op=CompareOp::Greater, .threshold=0.050f},
            {.feature="thumb_index_distance_min_over_200ms",
             .op=CompareOp::Greater, .threshold=0.025f},
        };
        g.min_hold_ms = 250;
        g.cooldown_ms = 80;
        g.action = GE_ACTION_POINTER_CLICK;
        g.priority = 1;
        ge->gestures.push_back(g);
    }

    // Scroll — thumb sliding along the SIDE of the index finger.
    //
    // User clarification 2026-05-22: "scroll has nothing to do with
    // pinch.  A pinch has the thumb at the tip of the index finger.
    // For a scroll, the thumb starts to the SIDE of the fingertip and
    // slides along the side of a slightly-curled index finger."
    //
    // The discriminating signal — from joint dumps on clips db9395 /
    // 39c1bf / 3a70d9 — is:
    //   - thumb is NEAR the index polyline (perpendicular distance
    //     small) — it's beside the finger, not floating off in space.
    //   - thumb is NOT at the index tip (thumb_index_distance NOT
    //     small) — that pose is the pinch_select case.
    //   - the thumb projects somewhere into the upper portion of the
    //     index finger (thumb_on_index_projection >= ~0.4), often
    //     past the tip (the engine clamps projection > 1 to 1.0).
    //
    // No motion gate.  The pose alone identifies scroll; the consumer
    // tracks the scalar `thumb_on_index_projection` over time and uses
    // its delta as scroll progress.  A stationary "thumb beside index
    // tip" pose still fires scroll — the user wants the gesture
    // recognised at the START of the action, not after a 12 mm
    // displacement has accumulated.
    //
    // Differentiation from pinch_select:
    //   pinch_select trigger : thumb_index_distance < 25 mm  (thumb AT tip)
    //   scroll trigger       : thumb_index_distance > 20 mm  (thumb NOT at tip)
    //   + thumb_to_index_line_distance < 40 mm (thumb near the finger axis)
    // The 20-vs-25 mm band is intentionally overlapping — within
    // 5 mm the tracker can't reliably tell tip-touch from
    // side-of-tip, and scroll's higher priority lets a borderline
    // pose resolve toward scroll if all the other gating matches.
    {
        GestureDef g;
        g.name = "scroll";
        g.trigger_conditions = {
            // Thumb NOT at the index tip — disambiguates from pinch.
            {.feature="thumb_index_distance",         .op=CompareOp::Greater, .threshold=0.020f},
            // Thumb close to the index-finger axis (any segment) but
            // not collapsed onto it — perp ≈ 0 means thumb is
            // touching the tip, which is a pinch, not a slide on the
            // SIDE.  5 mm minimum keeps scroll off a held pinch's
            // perpendicular-zero pose while still catching the
            // user's tightest scroll start (3a70d9 t=0.48s perp=6mm).
            // 50 mm upper bound catches mid-arc scrolls where the
            // thumb swings out (a4ca8f labelled-scroll window has
            // perp peaking at 45-48 mm).
            {.feature="thumb_to_index_line_distance", .op=CompareOp::Greater, .threshold=0.005f},
            // 40 mm upper bound on perp — real scroll-pose
            // perpendicular distances peak around 25 mm even during
            // the wide-thumb-sweep arc (a4ca8f t=0.8s perp=21 mm,
            // 629a23 perp up to 23 mm).  Looser bound (60 mm) caught
            // false fires on fist_launcher clips 820c73 (open-hand
            // pose at start) and 14759f (mid-clip pre-fist transit).
            {.feature="thumb_to_index_line_distance", .op=CompareOp::Less,    .threshold=0.040f},
            // Thumb-index distance upper bound — db9395 / a4ca8f
            // start their scroll with thumb at t-i ≈ 27-37 mm; the
            // wide-arc release sweeps the thumb out to 100+ mm but
            // by then scroll is already Active.  Without this guard,
            // open-hand poses with thumb 80-100 mm out from the index
            // (820c73, 14759f) drift into the scroll trigger band.
            {.feature="thumb_index_distance",         .op=CompareOp::Less,    .threshold=0.060f},
            // Thumb is somewhere along the upper half of the index
            // (or just past the tip — projection > 1 is clamped to
            // 1.0 by the feature impl).
            {.feature="thumb_on_index_projection",    .op=CompareOp::Greater, .threshold=0.40f},
            // Index is at most slightly curled.  User: "thumb sliding
            // across the side of a SLIGHTLY-curled index finger".
            {.feature="index_curl",                   .op=CompareOp::Less,    .threshold=0.50f},
            // Thumb is BESIDE or BEHIND the tip in the finger
            // pointing direction.  Real scroll-pose along values
            // straddle 0 (clip 39c1bf along -5 to +8, clip a4ca8f
            // along +17 to -53 across the wide arc, clip db9395
            // along +37 to -99).  Earlier 20 mm bound clipped the
            // a4ca8f start (along oscillating between 17 and 22) so
            // scroll Pending kept resetting; 30 mm gives the trigger
            // a stable window during the pre-slide hover.  Real
            // pinch_select clips have along up to +22 mm
            // (5871b8) but pinch_select trigger requires ti<25 mm —
            // scroll trigger requires ti>20 mm, so the 20-25 mm
            // ti band is the only overlap, and within that band
            // along=22 still belongs to pinch on close contact.
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Less,    .threshold=0.030f},
            // Middle finger not in 3-finger-pinch territory.  Scroll
            // clips can sweep the thumb close to middle (39c1bf drops
            // tm to 39 mm during the wide arc), so a strict > 50 mm
            // bound rejects the wide-arc window.  Loosened to > 35 mm
            // — the scalar motion gate downstream prevents this from
            // catching held right_clicks (which have NO projection
            // motion).
            {.feature="thumb_middle_distance",        .op=CompareOp::Greater, .threshold=0.035f},
            // Ring/pinky stay extended — excludes the all-fingers
            // gather pose and the right_click 3-finger pinch.
            {.feature="thumb_ring_distance",          .op=CompareOp::Greater, .threshold=0.060f},
            {.feature="thumb_pinky_distance",         .op=CompareOp::Greater, .threshold=0.060f},
            // Palm hasn't flipped recently — fist_launcher clips
            // rotate the palm Y from -0.85 → +0.85 (or vice versa)
            // in ~33 ms and then briefly sit in scroll-pose while the
            // fingers curl.  Clip b5bec9 fired scroll @ 2518 ms,
            // 332 ms after the flip at 2186 ms.  600 ms is a
            // comfortable cooldown: real scroll clips don't flip the
            // palm at all (recency stays at 1e6 ms forever).
            {.feature="palm_y_flip_recency_ms",       .op=CompareOp::Greater, .threshold=600.0f},
        };
        g.release_conditions = {
            // Thumb moves WELL off the index axis (the user has lifted
            // the thumb away or rotated their hand).
            {.feature="thumb_to_index_line_distance", .op=CompareOp::Greater, .threshold=0.080f},
            // OR thumb arrives at the index tip — they've transitioned
            // out of scroll into a pinch.
            {.feature="thumb_index_distance",         .op=CompareOp::Less,    .threshold=0.015f},
            // OR thumb leaves the upper portion of the finger entirely
            // (slid all the way down past MCP).
            {.feature="thumb_on_index_projection",    .op=CompareOp::Less,    .threshold=0.25f},
        };
        // Pose hold + scalar motion gate.  The motion gate is what
        // discriminates a real scroll from a static thumb-hovering-
        // in-scroll-pose (clip f1a911 holds perp ≈ 28 mm for 500 ms
        // before the actual pinch — perp+ti+along all match scroll's
        // trigger pose without the thumb actually sliding).  100 ms
        // pose hold filters single-tick tracker noise; the real
        // discriminator is "projection has changed at least 0.10
        // since Pending entry" — which a static thumb cannot
        // satisfy but a real slide hits within 100 ms.
        g.min_hold_ms = 100;
        g.cooldown_ms = 150;
        g.action = GE_ACTION_SCROLL;
        // Priority 7 — above pinch_right_click (6).  A user mid-
        // scroll often transits a 3-finger-pinch-like pose (clip
        // 986427 t=1.999 s: thumb-middle drops to 54 mm while
        // sliding); without scroll outranking right_click the
        // higher-priority gesture preempts the active scroll.
        // Scroll's strict trigger (along < 20, ti < 60, scalar
        // motion gate) makes false-fires unlikely so promoting it
        // above right_click doesn't bleed scroll into right_click
        // clips.
        g.priority = 7;
        g.track = {.type=TrackType::Scalar, .feature_name="thumb_on_index_projection"};
        g.scalar_gate_feature = "thumb_on_index_projection";
        g.scalar_gate_threshold = 0.08f;
        ge->gestures.push_back(g);
    }

    // Scroll — WIDE-ARC variant.
    //
    // Clip a4ca8f sweeps the thumb in a wide arc past the index in
    // ~500 ms, leaving perp = 40-130 mm and along = -10 to -120 mm
    // for most of the clip.  The tight variant (perp < 40 mm) misses
    // the entire window — the thumb is too far off-axis.  Wide-arc
    // requires the thumb to be CLEARLY behind the tip plane (along
    // < -10 mm) AND clearly off-axis (perp 40-150 mm) AND ring/
    // pinky extended.
    //
    // Critical gate: palm_y_below_neg_half_recency_ms > 2000 ms.
    // Fist_launcher clips start with palm DOWN (py = -0.8) then
    // flip during the rotation; after the flip py is positive
    // again, and the user re-opening the hand transits a wide-arc
    // pose for 100-200 ms before resting.  Wide-arc-scroll would
    // fire BEGIN there (clips 4ad403, 9330e9, 14759f, b5bec9) —
    // but real scroll clips never have py < -0.5 anywhere in the
    // recording.  Gate on "py hasn't been clearly down in the last
    // 2 seconds" cleanly separates the two.
    {
        GestureDef g;
        g.name = "scroll";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",         .op=CompareOp::Greater, .threshold=0.040f},
            {.feature="thumb_index_distance",         .op=CompareOp::Less,    .threshold=0.150f},
            {.feature="thumb_to_index_line_distance", .op=CompareOp::Greater, .threshold=0.040f},
            {.feature="thumb_to_index_line_distance", .op=CompareOp::Less,    .threshold=0.150f},
            {.feature="thumb_on_index_projection",    .op=CompareOp::Greater, .threshold=0.40f},
            {.feature="index_curl",                   .op=CompareOp::Less,    .threshold=0.50f},
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Less, .threshold=-0.010f},
            {.feature="thumb_middle_distance",        .op=CompareOp::Greater, .threshold=0.060f},
            {.feature="thumb_ring_distance",          .op=CompareOp::Greater, .threshold=0.070f},
            {.feature="thumb_pinky_distance",         .op=CompareOp::Greater, .threshold=0.080f},
            {.feature="palm_y_flip_recency_ms",       .op=CompareOp::Greater, .threshold=2000.0f},
            {.feature="palm_y_below_neg_half_recency_ms",
             .op=CompareOp::Greater, .threshold=2000.0f},
        };
        g.release_conditions = {
            {.feature="thumb_on_index_projection",    .op=CompareOp::Less,    .threshold=0.25f},
            {.feature="thumb_index_distance",         .op=CompareOp::Greater, .threshold=0.250f},
            // Thumb returned to the tip — the arc completed; user
            // may now be pinching (clip f1a911 starts with hover,
            // then arc, then pinch).
            {.feature="thumb_index_distance",         .op=CompareOp::Less,    .threshold=0.025f},
            // Thumb back in front of the tip — arc complete, user
            // returning to neutral or pinch.
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Greater, .threshold=0.010f},
        };
        g.min_hold_ms = 100;
        g.cooldown_ms = 150;
        g.action = GE_ACTION_SCROLL;
        g.priority = 7;
        g.track = {.type=TrackType::Scalar, .feature_name="thumb_on_index_projection"};
        g.scalar_gate_feature = "thumb_on_index_projection";
        // 0.15 threshold separates a4ca8f (max proj delta 0.31
        // during the wide arc — clears the bar comfortably) from
        // f1a911's static hover (max proj delta 0.11 from tracker
        // jitter alone).  Lower thresholds let the jitter pass.
        g.scalar_gate_threshold = 0.15f;
        ge->gestures.push_back(g);
    }

    // grab_window (window-move drag) DISABLED for now per user
    // request 2026-05-19: "every time I try to close a window or
    // click, I end up dragging it... maybe just remove the dragging
    // gesture for now?".  The mutual-exclusion fix on pinch_select
    // (require thumb-middle > 70mm) helps but the user can still
    // accidentally grab when trying to do a 5-finger gather close
    // or just gesturing near the threshold.  Re-enable once we have
    // a clearer activation gesture (e.g. dedicated thumbs-up to
    // enter "move mode") and proper subjective-tuning data.
    //
    // 2026-09-04 — a re-enable was ATTEMPTED and reverted.  Do NOT move
    // the block below into the live table without new thresholds first;
    // the numbers are inverted, not merely loose.  Measured over the
    // recording mirror (`eval_recording --features`), an IDLE OPEN HAND
    // reads ti 0.051 / tm 0.062 / ring 0.067 / pinky 0.070 — all four
    // trigger conditions satisfied, continuously, from frame 0.  During
    // the actual pinch the middle finger moves AWAY (tm 0.072-0.077), so
    // the trigger is met only while the user is doing nothing.  Enabling
    // it fires WINDOW_MOVE BEGIN at 167 ms on a resting hand and holds
    // the hand for the whole clip (release wants ti > 0.085, which a rest
    // pose never reaches), which starves pinch_select — three eval suites
    // go red.  Lowering the priority below pinch_select fixes the
    // starvation but not the false grabs.  CLAUDE.md records real users
    // resting at ti 12-18 mm, i.e. deeper inside the band than the
    // synthetic fixture, so this is not a fixture artifact.
    //
    // The distances themselves are what must change (plus a convergence
    // gate like every gesture that survived the 2026-06-08 rewrite), and
    // that is `grab-threshold-tune` in BACKLOG.md — a Track B in-person
    // task, not something to fit to synthetic clips.  Until it lands the
    // cheat sheet's "grab + move" row has no emitter.
    //
    // Companion handlers are already wired and will work as-is:
    // mac-shell/src/core/scene.cpp case GE_ACTION_WINDOW_MOVE and
    // vendor/wxrd/src/hand_input.c.
    #if 0
    {
        GestureDef g;
        g.name = "grab_window";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",  .op=CompareOp::Less,    .threshold=0.055f},
            {.feature="thumb_middle_distance", .op=CompareOp::Less,    .threshold=0.070f},
            {.feature="thumb_ring_distance",   .op=CompareOp::Greater, .threshold=0.055f},
            {.feature="thumb_pinky_distance",  .op=CompareOp::Greater, .threshold=0.055f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance",  .op=CompareOp::Greater, .threshold=0.085f},
            {.feature="thumb_middle_distance", .op=CompareOp::Greater, .threshold=0.100f},
        };
        g.min_hold_ms = 150;
        g.action = GE_ACTION_WINDOW_MOVE;
        g.priority = 3;
        g.track = {.type=TrackType::Position3D, .feature_name="pinch_midpoint"};
        ge->gestures.push_back(g);
    }
    #endif

    // With grab disabled, pinch_select no longer has the grab pose
    // to mutually exclude against — its old single-condition trigger
    // (thumb_index_distance < 25mm) is enough.  Revert the
    // thumb_middle_distance > 70mm guard so the click gesture is
    // less finicky.  See the pinch_select block above.

    // Fist (toggle launcher).
    // User report 2026-05-20: "a little bit of friction with opening
    // the app launcher for the first time — it's too precise/bounded?".
    // Loosen the curl + hold thresholds so a relaxed fist counts:
    //   all_fingers_curl mean:  0.70 -> 0.55
    //   per-finger minimums:    0.50 -> 0.35  (small fingers / partial closure ok)
    //   min_hold_ms:            500  -> 200   (snappier acceptance)
    //   release stays at 0.40 so the gesture isn't trivially flappy
    //
    // grab_window being disabled removes the main false-positive risk
    // for "fist while doing something else", and the latched-launcher
    // model means a stray open is dismissable with another fist.
    //
    // Palm orientation is checked in the compositor (hand_input.c),
    // not here.  Thumb curl is deliberately not constrained — thumb
    // wraps around fist but doesn't fold back (~0.2 curl reading).
    {
        GestureDef g;
        g.name = "fist_launcher";
        g.trigger_conditions = {
            {.feature="all_fingers_curl", .op=CompareOp::Greater, .threshold=0.55f},
            {.feature="index_curl",  .op=CompareOp::Greater, .threshold=0.35f},
            {.feature="middle_curl", .op=CompareOp::Greater, .threshold=0.35f},
            {.feature="ring_curl",   .op=CompareOp::Greater, .threshold=0.35f},
            {.feature="pinky_curl",  .op=CompareOp::Greater, .threshold=0.35f},
        };
        g.release_conditions = {{.feature="all_fingers_curl", .op=CompareOp::Less, .threshold=0.4f}};
        // min_hold 350 ms.  The user's discrete fist_launcher
        // keyframes land at the moment the fist is *fully* formed (not
        // when curl first crosses the trigger threshold).  At 200 ms
        // engine BEGIN fired ~500 ms before the keyframe; bumping to
        // 350 ms moves it within the -100 ms early tolerance without
        // making the gesture feel sluggish (still under the
        // half-second perceptual delay threshold).
        g.min_hold_ms = 350;
        g.action = GE_ACTION_TOGGLE_LAUNCHER;
        g.priority = 4;
        g.track = {.type=TrackType::Position3D, .feature_name="palm_center"};
        ge->gestures.push_back(g);
    }

    // Fist launcher — CURL-STABILITY variant.
    //
    // The standard variant above gates per-frame on middle_curl > 0.35
    // and ring_curl > 0.35. Clip cadaef holds a real fist for the entire
    // 350-ms hold window, but the iPhone Vision tracker flickers
    // middle_curl between 0.03 and 0.85 on the same held fist (the
    // curled fingertip occludes itself and the tracker falls back to
    // a low-confidence prior, dropping curl to ~0). The flicker resets
    // the Pending hold each time middle_curl drops below 0.35.
    //
    // This variant swaps the per-frame curl gates for windowed-max
    // versions: middle/ring_curl_max_over_200ms > 0.35 holds true as
    // long as either finger was curled at any point in the last 200 ms,
    // which is robust to single-frame mis-tracking. The fist must
    // still pass index_curl, pinky_curl, and all_fingers_curl per-frame
    // (none of which flickers in cadaef per the triage).
    //
    // Same name + action + priority + min_hold as the standard variant;
    // same-name suppression keeps one Active at a time.
    {
        GestureDef g;
        g.name = "fist_launcher";
        g.trigger_conditions = {
            {.feature="all_fingers_curl", .op=CompareOp::Greater, .threshold=0.55f},
            {.feature="index_curl",  .op=CompareOp::Greater, .threshold=0.35f},
            {.feature="pinky_curl",  .op=CompareOp::Greater, .threshold=0.35f},
            // Windowed-max replaces per-frame for the two flicker-prone
            // curls; same 0.35 threshold.
            {.feature="middle_curl_max_over_200ms", .op=CompareOp::Greater, .threshold=0.35f},
            {.feature="ring_curl_max_over_200ms",   .op=CompareOp::Greater, .threshold=0.35f},
        };
        g.release_conditions = {{.feature="all_fingers_curl", .op=CompareOp::Less, .threshold=0.4f}};
        g.min_hold_ms = 350;
        g.action = GE_ACTION_TOGGLE_LAUNCHER;
        g.priority = 4;
        g.track = {.type=TrackType::Position3D, .feature_name="palm_center"};
        ge->gestures.push_back(g);
    }

    // Right click — occluded-middle fallback.
    //
    // When the user's middle finger rests on top of an active
    // thumb-index pinch, the iPhone Vision tracker can't see the
    // middle fingertip (it's behind the thumb from the camera's
    // viewpoint) and drops its confidence to ~0.10-0.30, often with
    // a fictional position 60-80 mm from the thumb.  See clip
    // 20260522-193648-4cd4b2 — user confirmed (2026-05-22) that
    // middle WAS on the thumb but the tracker reports it 66 mm away.
    //
    // The pose-based right_click trigger below would reject this case
    // because it requires thumb_middle_distance < 45 mm.  This
    // fallback variant detects the same intent via the tracker's own
    // "I can't see it" signal: thumb+index pinched AND middle's
    // confidence is below a "definitely-occluded" threshold AND
    // ring/pinky still tracked extended.  Same name + action so the
    // compositor sees one event type.
    //
    // Two GestureDef entries with the same name + action work
    // independently in the state machine — whichever satisfies its
    // trigger first goes Active and emits a BEGIN; the other gets
    // suppressed by hand_claimed.
    {
        GestureDef g;
        g.name = "pinch_right_click";
        g.trigger_conditions = {
            // Thumb-index pinched.
            {.feature="thumb_index_distance",   .op=CompareOp::Less,    .threshold=0.035f},
            // Middle clearly invisible to the tracker → assume it's
            // hidden behind the thumb-index pinch (user's right-click
            // pose).  Cutoff 0.45 chosen from the recorded data: real
            // right-click clips drop middle confidence to 0.12-0.36
            // during the held pinch; pinch_select / scroll clips
            // (middle finger extended away from pinch) keep middle
            // confidence at 0.65-0.95.  Plenty of separation.
            {.feature="middle_tip_confidence",  .op=CompareOp::Less,    .threshold=0.55f},
            // Windowed-min guards against tracker noise that briefly
            // dips confidence to ~0.51 (clip b21dab, real pinch_select
            // with middle clearly visible at tm = 65-75 mm).  Real
            // occlusion drops confidence to ~0.17 and HOLDS it there
            // (clip 4cd4b2: 0.158-0.192 throughout the held pose) —
            // windowed-min stays low. Noise-induced borderline dips
            // never reach 0.40.
            {.feature="middle_tip_confidence_min_over_200ms",
             .op=CompareOp::Less,    .threshold=0.40f},
            // Distinguish from fingertip_gather_close (all 5 fingers
            // collapsed to a point).  When the user does a 3-finger
            // right-click, ring/pinky stay extended; the gather
            // radius (max distance from any tip to the 5-tip
            // centroid) stays large because of those two outliers.
            // 5-finger gather collapses radius to < 0.08 m.
            {.feature="fingertip_gather_radius", .op=CompareOp::Greater, .threshold=0.060f},
            // Thumb must be AHEAD of the index tip in the finger
            // pointing direction.  In a real right-click the thumb
            // closes onto index+middle, ending up forward of the
            // tips; in a scroll motion the thumb sits behind/beside
            // the tip even when it briefly comes close to the middle.
            // Clip 20260522-193355-629a23 transits through middle-
            // close at along ≈ -5 to -57 mm and would otherwise fire
            // right_click.
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Greater, .threshold=0.0f},
            // Convergence gate — thumb must have been *clearly* far
            // from the index within the last 500 ms (recency tracks
            // ms since ti last exceeded 30 mm).  See occluded-middle
            // comment block above.
            {.feature="thumb_index_recency_above_30mm_ms",
             .op=CompareOp::Less, .threshold=500.0f},
            // tm convergence gate — see comment in ge_features.h.
            {.feature="thumb_middle_recency_above_60mm_ms",
             .op=CompareOp::Less, .threshold=500.0f},
            // Cross-gesture: don't fire right_click for 300 ms after
            // a fist_launcher ends — the hand is mid-extension and
            // the iPhone Vision tracker reports a brief drop in
            // middle_tip_confidence that the occluded-middle variant
            // would otherwise catch.  User report 2026-05-23.
            {.feature="fist_launcher_end_recency_ms",
             .op=CompareOp::Greater, .threshold=300.0f},
        };
        g.release_conditions = {
            // Either the pinch opens up or the tracker recovers a
            // confident middle-finger reading (likely because user
            // un-curled middle).
            {.feature="thumb_index_distance",  .op=CompareOp::Greater, .threshold=0.055f},
            {.feature="middle_tip_confidence", .op=CompareOp::Greater, .threshold=0.65f},
        };
        g.min_hold_ms = 67;
        g.cooldown_ms = 250;
        g.action = GE_ACTION_POINTER_RIGHT_CLICK;
        g.priority = 6;
        ge->gestures.push_back(g);
    }

    // Right click — "aimed-middle" variant.
    //
    // User clarification 2026-05-23: clip 8b3d04 IS a (loose) 3-finger
    // right-click but the iPhone Vision tracker reports the middle
    // tip 70-80 mm from the thumb the whole time — it never reaches
    // the t-m < 55 mm threshold the standard variant requires, and
    // middle_tip_confidence stays at 0.55-0.67 so the occluded
    // variant misses it too.
    //
    // The signal the user pointed at: even when the middle finger
    // doesn't FULLY close onto the thumb, the user has AIMED the
    // middle finger AT the thumb-index pinch.  The middle's DIP→TIP
    // direction points toward the thumb.  Cosine ≥ 0.75 across all
    // real right-click clips (af0461 / a5f5d8 / 778bb2 / 5ca5ef /
    // 8b3d04 / 4cd4b2 all measure +0.83 to +1.00 at the keyframe
    // window); pinch_select clips where the middle is extended away
    // sit at -0.99 to +0.66, with the only clip clearing 0.66
    // (5871b8) having t-m=73 mm just barely in the rule's window —
    // discriminated by the cos threshold.
    //
    // Combined gate:
    //   thumb_index < 20 mm        (tight pinch on index)
    //   thumb_middle ∈ [60, 80] mm (close but not touching)
    //   cos(mid→thumb) > 0.75      (middle is AIMED at thumb)
    //   ring/pinky > 60 mm         (not all-fingers gather)
    // No `along` constraint — 8b3d04 sits at along ≈ 0 (thumb right
    // at the tip plane, not ahead).  The cos+distance combination is
    // specific enough to discriminate without it.
    {
        GestureDef g;
        g.name = "pinch_right_click";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",              .op=CompareOp::Less,    .threshold=0.020f},
            {.feature="thumb_middle_distance",             .op=CompareOp::Greater, .threshold=0.060f},
            {.feature="thumb_middle_distance",             .op=CompareOp::Less,    .threshold=0.085f},
            {.feature="middle_tip_direction_to_thumb_cos", .op=CompareOp::Greater, .threshold=0.75f},
            {.feature="thumb_ring_distance",               .op=CompareOp::Greater, .threshold=0.060f},
            {.feature="thumb_pinky_distance",              .op=CompareOp::Greater, .threshold=0.060f},
            // Middle is EXTENDED while aimed at the thumb.  In a real
            // right_click the user actively extends + aims the middle
            // finger; every standard-variant RC clip in the dataset
            // measures middle_curl ≤ 0.17 at the trigger pose
            // (0b7036 0.04, a5f5d8 0.07, 5ca5ef 0.08, 4cd4b2 0.09,
            // 8b3d04 0.10, af0461 0.11, 778bb2 0.14).  Clip bebdc7
            // (held pinch_select where middle finger rests slightly
            // curled near the index) sits at middle_curl=0.24-0.27 —
            // it looks identical to aimed-middle in tm and cos but
            // the middle is RELAXED, not extended-and-aimed.  The
            // 0.20 gate cleanly separates the two without dropping
            // any real RC clip.
            {.feature="middle_curl",                       .op=CompareOp::Less,    .threshold=0.20f},
            // Convergence gate — see occluded-middle variant above.
            {.feature="thumb_index_recency_above_30mm_ms",
             .op=CompareOp::Less, .threshold=500.0f},
            // tm convergence gate — see comment in ge_features.h.
            {.feature="thumb_middle_recency_above_60mm_ms",
             .op=CompareOp::Less, .threshold=500.0f},
            // Cross-gesture: don't fire right_click for 300 ms after
            // a fist_launcher ends — the hand is mid-extension and
            // the iPhone Vision tracker reports a brief drop in
            // middle_tip_confidence that the occluded-middle variant
            // would otherwise catch.  User report 2026-05-23.
            {.feature="fist_launcher_end_recency_ms",
             .op=CompareOp::Greater, .threshold=300.0f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance",  .op=CompareOp::Greater, .threshold=0.040f},
            {.feature="thumb_middle_distance", .op=CompareOp::Greater, .threshold=0.100f},
            {.feature="middle_tip_direction_to_thumb_cos", .op=CompareOp::Less, .threshold=0.50f},
        };
        // 350 ms hold — aimed-middle's pose looks almost identical to
        // a brief double_pinch pinch (clip fa3b02 has t-m=79 mm and
        // cos=+0.99 during its two pinch_select beats).  Real right-
        // click clips like 8b3d04 hold the pose for ≥ 380 ms; double-
        // pinch beats are 130-300 ms.  A 350 ms hold cleanly catches
        // the real right-click while not firing on quick double-pinch
        // beats.  Standard variant (line 564) has a shorter hold
        // because its t-m < 55 mm threshold already excludes the
        // double_pinch case.
        g.min_hold_ms = 350;
        g.cooldown_ms = 250;
        g.action = GE_ACTION_POINTER_RIGHT_CLICK;
        g.priority = 6;
        ge->gestures.push_back(g);
    }

    // Right click = THREE-finger pinch (thumb + index + middle all
    // touching).  Ring and pinky must stay extended so this is
    // disambiguated from fingertip_gather_close (all 5 fingertips).
    //
    // User clarification 2026-05-22: "right click is index + thumb +
    // middle finger (two-finger pinch — the differentiation between
    // this & a normal pinch is pretty important)".  Previous
    // engine.cpp definition required thumb_index_distance > 35 mm,
    // which directly contradicted the user's actual motion and was
    // why 0/10 right-click clips fired right-click — they all fired
    // pinch_select instead.
    //
    // Higher priority (6) than pinch_select (1) so the 3-finger pose
    // wins on the same hand.  However pinch_select still goes Pending
    // first (lower min_hold_ms), then gets cancelled when right-click
    // goes Active — same pattern as scroll-vs-click.  Right-click's
    // min_hold of 80 ms is the click-commit deadline the consumer
    // should respect for pinch_select.
    {
        GestureDef g;
        g.name = "pinch_right_click";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",  .op=CompareOp::Less,    .threshold=0.035f},
            // 55 mm (was 45) — clips like 20260522-193640-0b7036 hold
            // a real 3-finger pinch with thumb-middle stuck at 47-52
            // mm by iPhone Vision (the middle-finger tip occludes
            // slightly behind the pinch and the tracker smooths its
            // reported position outward).  Loosening to 55 mm catches
            // them without bleeding into pinch_select (whose own
            // trigger requires thumb_middle > 45 mm to mutually
            // exclude — collision band [45, 55] mm is resolved by
            // right_click's higher priority).
            {.feature="thumb_middle_distance", .op=CompareOp::Less,    .threshold=0.055f},
            // Lower-bound on thumb_middle defends against an iPhone
            // Vision tracking artifact: when the tracker briefly loses
            // the middle-finger tip it emits position (0, 0, 0),
            // computing thumb_middle ≈ thumb's distance from origin
            // (often near 0).  Without this guard, 2-finger pinches
            // and scrolls fire as right_clicks while the artifact
            // lingers for multiple ticks.
            {.feature="thumb_middle_distance", .op=CompareOp::Greater, .threshold=0.008f},
            {.feature="thumb_ring_distance",   .op=CompareOp::Greater, .threshold=0.060f},
            {.feature="thumb_pinky_distance",  .op=CompareOp::Greater, .threshold=0.060f},
            // Same along-positive check as the occluded-middle variant
            // above — see that comment block.  In a real 3-finger
            // pinch the thumb closes ahead of the tip; transient
            // scroll-prep poses where thumb brushes middle have
            // along < 0.
            {.feature="thumb_along_index_tip_direction", .op=CompareOp::Greater, .threshold=0.0f},
            // Perpendicular gate — in a real 3-finger right_click the
            // thumb is ON the index tip with the middle curled into the
            // pinch, so perp is small (≤ 0.014 m across every dataset RC
            // at BEGIN).  Clip 9b7345 has a pre-pinch transitional pose
            // (ti=tm=perp=along ≈ 0.033) where the thumb is hovering
            // 33 mm off the index axis — geometrically a different
            // pose, but the standard variant's other gates accept it
            // and fire RC inside what should be a pinch_select window.
            // 0.025 m gives ~10 mm margin over the tightest real RC.
            {.feature="thumb_to_index_line_distance",
             .op=CompareOp::Less, .threshold=0.025f},
            // Middle-curl ceiling.  Mirror of the aimed-middle variant's
            // mc < 0.20 rule but tighter — 0.18 sits between the worst
            // real RC at BEGIN (778bb2 mc=0.137) and 9b7345's TM-collapsed
            // pinch_select where the tracker shrinks tm to 0.034 m while
            // the user's middle is mc=0.19-0.21 (slightly curled,
            // not actively aimed-extended).  Real RC requires the user
            // to deliberately straighten and aim the middle — mc < 0.18
            // captures that intent without losing any clip.
            {.feature="middle_curl",                     .op=CompareOp::Less, .threshold=0.18f},
            // Convergence gate — see occluded-middle variant above.
            {.feature="thumb_index_recency_above_30mm_ms",
             .op=CompareOp::Less, .threshold=500.0f},
            // tm convergence gate — see comment in ge_features.h.
            {.feature="thumb_middle_recency_above_60mm_ms",
             .op=CompareOp::Less, .threshold=500.0f},
            // Cross-gesture: don't fire right_click for 300 ms after
            // a fist_launcher ends — the hand is mid-extension and
            // the iPhone Vision tracker reports a brief drop in
            // middle_tip_confidence that the occluded-middle variant
            // would otherwise catch.  User report 2026-05-23.
            {.feature="fist_launcher_end_recency_ms",
             .op=CompareOp::Greater, .threshold=300.0f},
            // Middle finger DELIBERATELY aimed at the thumb.  In a
            // genuine 3-finger right-click the user actively curls the
            // middle inward; cosine of the (middle DIP→TIP)·(MIDDLE_TIP→
            // THUMB_TIP) direction lands at +0.79 to +0.99 across every
            // standard-variant right_click in the dataset (af0461 +0.86,
            // a5f5d8 +0.79, 778bb2 +0.81, 5ca5ef +0.83, 0b7036 +0.97).
            //
            // A precision LEFT-click on a small UI target (user report
            // 2026-05-24, VSCode icon in the launcher grid) has the
            // middle finger naturally near the thumb because gripping
            // for fine motor control recruits middle-finger stability —
            // but the middle isn't being AIMED at the thumb, just
            // resting nearby.  cos in that case sits much lower
            // (closer to 0 or negative).  A +0.6 threshold cleanly
            // separates the two without losing any real right-click.
            {.feature="middle_tip_direction_to_thumb_cos",
             .op=CompareOp::Greater, .threshold=0.6f},
        };
        g.release_conditions = {
            // Any of these going slack ends the gesture — robust to
            // user pulling thumb away from EITHER index or middle.
            {.feature="thumb_index_distance",  .op=CompareOp::Greater, .threshold=0.060f},
            {.feature="thumb_middle_distance", .op=CompareOp::Greater, .threshold=0.080f},
        };
        // min_hold 67 ms ≈ 2 ticks at 30 Hz.  Shorter values fire on
        // transient 3-finger poses that often occur at the start of a
        // gesture clip (user's hand starts collapsed, then extends
        // middle finger to do an actual 2-finger pinch/scroll).
        // 67 ms forces the pose to persist past at least one full
        // frame before BEGIN fires.
        g.min_hold_ms = 67;
        g.cooldown_ms = 250;
        g.action = GE_ACTION_POINTER_RIGHT_CLICK;
        g.priority = 6;
        ge->gestures.push_back(g);
    }

    // Pinch right-click — TIGHT THREE-FINGER variant.
    //
    // Clip 20260528-022117-1f0672: the user pinches all three fingers
    // (thumb+index+middle) so tight that tm collapses to 0.010-0.013 m
    // — well below the standard variant's 8-55 mm window and the aimed-
    // middle variant's 60-80 mm window. The pose is unambiguously a
    // right-click (middle clearly aimed at thumb, cos ≈ 0.97; ring/pinky
    // extended) but it sits in an uncovered tm band.
    //
    // The `thumb_along_index_tip_direction > 0` gate that the standard
    // variant uses falls just below zero on this clip (along ≈ -0.002 m)
    // because the thumb is RIGHT AT the tip plane rather than ahead.
    // The cos + tight-tm + ring/pinky combination is specific enough
    // that the along gate is redundant here.
    //
    // Convergence: ti_recency is still well within 500 ms, but
    // tm_recency is locked (tm never exceeds 60 mm in this clip).
    // Replace the tm absolute-recency gate with the new shape feature
    // (tm_ratio > 0.30 + tm_velocity > 0.05).
    {
        GestureDef g;
        g.name = "pinch_right_click";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",   .op=CompareOp::Less,    .threshold=0.020f},
            {.feature="thumb_middle_distance",  .op=CompareOp::Less,    .threshold=0.020f},
            {.feature="thumb_middle_distance",  .op=CompareOp::Greater, .threshold=0.008f},
            {.feature="thumb_ring_distance",    .op=CompareOp::Greater, .threshold=0.060f},
            {.feature="thumb_pinky_distance",   .op=CompareOp::Greater, .threshold=0.060f},
            {.feature="middle_tip_direction_to_thumb_cos",
             .op=CompareOp::Greater, .threshold=0.75f},
            {.feature="thumb_index_recency_above_30mm_ms",
             .op=CompareOp::Less, .threshold=500.0f},
            // Shape gate replaces absolute tm-recency (tm never reaches
            // 60 mm in this clip family — tracker reports a permanently-
            // tight 3-finger pinch).
            {.feature="thumb_middle_convergence_ratio",
             .op=CompareOp::Greater, .threshold=0.30f},
            {.feature="thumb_middle_convergence_velocity_mps",
             .op=CompareOp::Greater, .threshold=0.05f},
            {.feature="fist_launcher_end_recency_ms",
             .op=CompareOp::Greater, .threshold=300.0f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance",   .op=CompareOp::Greater, .threshold=0.040f},
            {.feature="thumb_middle_distance",  .op=CompareOp::Greater, .threshold=0.045f},
            {.feature="middle_tip_direction_to_thumb_cos",
             .op=CompareOp::Less, .threshold=0.50f},
        };
        // 33 ms hold (1 tick at 30 Hz). The combined gate is so specific
        // — tm in the very narrow [8, 20] mm band, middle DIRECTION
        // pointing at thumb (cos > 0.75), ring/pinky each > 60 mm AND
        // tm convergence shape evidence — that no transient pose has
        // ever been seen in the dataset matching all of these
        // simultaneously. Clip 1f0672's trigger holds for only ~67 ms
        // before ring slides past 60 mm; a longer hold would miss it.
        g.min_hold_ms = 33;
        g.cooldown_ms = 250;
        g.action = GE_ACTION_POINTER_RIGHT_CLICK;
        g.priority = 6;
        ge->gestures.push_back(g);
    }

    // Pinch right-click — SHAPE-OF-CONVERGENCE variant.
    //
    // Same motivation as the pinch_select shape variant: the standard
    // RC variant gates on absolute ti/tm recency (> 30 mm / > 60 mm in
    // the last 500 ms) which the user's tight resting state can never
    // satisfy.  Clip 20260528-022117-1f0672 (RC) opens at tm = 0.037
    // and never exceeds 0.060 — the absolute gate is locked out.
    //
    // Shape thresholds picked from per-clip traces:
    //   1f0672 (tight rest RC):    tm ratio 0.73, tm velocity ~0.10 m/s
    //   9b7345 (tm collapses on PS): tm ratio 0.47, tm velocity ~0.05 m/s
    // tm_ratio > 0.30 covers both with margin; tm_velocity > 0.04 m/s
    // gives 9b7345 a sliver of margin without bleeding into rest poses.
    // tm_min_over_200ms < 0.055 mirrors the standard variant's primary
    // tm trigger.
    //
    // Shape gating ALSO suppresses the b21dab failure: in that clip the
    // real pinch_select fires correctly but the occluded-middle RC
    // variant spuriously co-fires because middle_tip_confidence dips
    // below 0.55 while tm stays clearly visible at 0.063–0.075.  The
    // shape variant rejects that case because tm has no convergence
    // shape (ratio ≈ 0, velocity ≈ 0 — middle finger is stationary).
    {
        GestureDef g;
        g.name = "pinch_right_click";
        g.trigger_conditions = {
            {.feature="thumb_index_distance",  .op=CompareOp::Less,    .threshold=0.035f},
            {.feature="thumb_middle_distance", .op=CompareOp::Less,    .threshold=0.055f},
            {.feature="thumb_middle_distance", .op=CompareOp::Greater, .threshold=0.008f},
            {.feature="thumb_ring_distance",   .op=CompareOp::Greater, .threshold=0.060f},
            {.feature="thumb_pinky_distance",  .op=CompareOp::Greater, .threshold=0.060f},
            {.feature="thumb_along_index_tip_direction",
             .op=CompareOp::Greater, .threshold=0.0f},
            // Shape gates — replace the two absolute recency gates.
            {.feature="thumb_index_convergence_ratio",
             .op=CompareOp::Greater, .threshold=0.40f},
            {.feature="thumb_index_convergence_velocity_mps",
             .op=CompareOp::Greater, .threshold=0.10f},
            {.feature="thumb_index_distance_min_over_200ms",
             .op=CompareOp::Less, .threshold=0.035f},
            {.feature="thumb_middle_convergence_ratio",
             .op=CompareOp::Greater, .threshold=0.30f},
            {.feature="thumb_middle_convergence_velocity_mps",
             .op=CompareOp::Greater, .threshold=0.04f},
            {.feature="thumb_middle_distance_min_over_200ms",
             .op=CompareOp::Less, .threshold=0.055f},
            {.feature="fist_launcher_end_recency_ms",
             .op=CompareOp::Greater, .threshold=300.0f},
            {.feature="middle_tip_direction_to_thumb_cos",
             .op=CompareOp::Greater, .threshold=0.6f},
            // Middle-curl ceiling — same rationale as the standard
            // 3-finger variant.  Clip 9b7345 (TM-collapsed pinch_select)
            // has tm + ti shape evidence that fires this variant but the
            // user's middle is mc=0.19-0.21 (not actively extended-and-
            // aimed).  Real RC clips fire with mc ≤ 0.137.  0.18 leaves
            // ~30 mm of margin on the RC side and ~10 mm on the PS side.
            {.feature="middle_curl",           .op=CompareOp::Less,    .threshold=0.18f},
        };
        g.release_conditions = {
            {.feature="thumb_index_distance",  .op=CompareOp::Greater, .threshold=0.060f},
            {.feature="thumb_middle_distance", .op=CompareOp::Greater, .threshold=0.080f},
        };
        g.min_hold_ms = 67;
        g.cooldown_ms = 250;
        g.action = GE_ACTION_POINTER_RIGHT_CLICK;
        g.priority = 6;
        ge->gestures.push_back(g);
    }

    // Fingertip gather = CLOSE WINDOW.
    // All five fingertips touching together — explicitly mutually exclusive
    // with click (thumb+index only), right-click (thumb+middle), and grab
    // (thumb+index+middle, ring/pinky required extended).  Loosened thresholds
    // + shorter hold so the gesture is actually reachable when intended.
    {
        GestureDef g;
        g.name = "fingertip_gather_close";
        g.trigger_conditions = {
            {.feature="fingertip_gather_radius", .op=CompareOp::Less,    .threshold=0.075f},
            {.feature="thumb_pinky_distance",    .op=CompareOp::Less,    .threshold=0.055f},
            {.feature="thumb_ring_distance",     .op=CompareOp::Less,    .threshold=0.055f},
            {.feature="thumb_middle_distance",   .op=CompareOp::Less,    .threshold=0.055f},
            {.feature="thumb_index_distance",    .op=CompareOp::Less,    .threshold=0.055f},
        };
        g.release_conditions = {
            {.feature="fingertip_gather_radius", .op=CompareOp::Greater, .threshold=0.10f},
        };
        g.min_hold_ms = 350;
        g.action = GE_ACTION_WINDOW_CLOSE;
        g.priority = 2;
        ge->gestures.push_back(g);
    }

    // Keyboard summon = thumbs-up.  Superseded by the wlr_text_input_v3
    // path in vendor/wxrd (keyboard-text-input-protocol-summon, 2026-05-15):
    // the keyboard now auto-appears whenever any focused widget needs text
    // input — no gesture vocabulary for the user to memorise.  Definition
    // kept under LEGACY_KEYBOARD_SUMMON in case a hardware tuning session
    // wants to re-enable it as a manual override.
#ifdef LEGACY_KEYBOARD_SUMMON
    {
        GestureDef g;
        g.name = "keyboard_summon";
        g.trigger_conditions = {
            {.feature="thumb_curl",  .op=CompareOp::Less,    .threshold=0.3f},
            {.feature="index_curl",  .op=CompareOp::Greater, .threshold=0.7f},
            {.feature="middle_curl", .op=CompareOp::Greater, .threshold=0.7f},
            {.feature="ring_curl",   .op=CompareOp::Greater, .threshold=0.7f},
            {.feature="pinky_curl",  .op=CompareOp::Greater, .threshold=0.7f},
        };
        g.release_conditions = {
            {.feature="index_curl",  .op=CompareOp::Less, .threshold=0.5f},
            {.feature="middle_curl", .op=CompareOp::Less, .threshold=0.5f},
        };
        g.min_hold_ms = 500;
        g.action = GE_ACTION_TOGGLE_KEYBOARD;
        g.priority = 7;
        ge->gestures.push_back(g);
    }
#endif

    // Thumb slider (fine continuous input)
    {
        GestureDef g;
        g.name = "thumb_slider";
        g.trigger_conditions = {
            {.feature="index_curl", .op=CompareOp::Greater, .threshold=0.6f},
            {.feature="thumb_index_distance", .op=CompareOp::Less, .threshold=0.025f},
        };
        g.release_conditions = {{.feature="thumb_index_distance", .op=CompareOp::Greater, .threshold=0.04f}};
        g.min_hold_ms = 100;
        g.action = GE_ACTION_FINE_SLIDER;
        g.priority = 5;
        g.track = {.type=TrackType::Scalar, .feature_name="thumb_on_index_projection"};
        ge->gestures.push_back(g);
    }

    // Keyboard anchor — open palm held face-down for 1500 ms over a
    // detected horizontal plane.  The wxrd-side handler queries
    // bridge-receiver for planes and re-anchors the typing plane.
    //
    // Pose discrimination:
    //   palm_normal_y < -0.70  → palm decidedly facing down (relaxed
    //                            open palms at ~-0.4 to -0.6 don't
    //                            qualify; only a clear "palm-flat-
    //                            over-the-desk" pose does)
    //   palm_openness  > 0.60  → fingers extended, not fisted
    //   all_fingers_curl < 0.25 → same intent, redundant guard against
    //                             half-closed claw poses
    //   fist_launcher_end_recency_ms > 2000 → not within 2 s of a
    //                            fist_launcher release.  Without this
    //                            guard, opening the hand after a fist
    //                            passes briefly through (open + face-
    //                            down) and fires a spurious anchor —
    //                            caught by eval_dataset against clips
    //                            820c73 + 5ca5ef.
    //
    // Release: palm rotates up OR hand starts closing.  A 1500 ms hold
    // is the same magnitude as fist_launcher's discriminative hold —
    // long enough to be deliberate, short enough to feel responsive.
    //
    // Priority 1 (same band as pinch_select) so any in-flight pinch /
    // grab / launcher suppresses this from entering Pending.  Tracks
    // palm_center as a Position3D so the BEGIN event carries the
    // world coordinates of the palm at activation time — wxrd uses
    // that to find the nearest horizontal plane beneath it.
    {
        GestureDef g;
        g.name = "keyboard_anchor";
        g.trigger_conditions = {
            {.feature="palm_normal_y",   .op=CompareOp::Less,    .threshold=-0.70f},
            {.feature="palm_openness",   .op=CompareOp::Greater, .threshold=0.60f},
            {.feature="all_fingers_curl",.op=CompareOp::Less,    .threshold=0.25f},
            {.feature="fist_launcher_end_recency_ms",
                                         .op=CompareOp::Greater, .threshold=2000.0f},
        };
        g.release_conditions = {
            {.feature="palm_normal_y",   .op=CompareOp::Greater, .threshold=-0.40f},
            {.feature="all_fingers_curl",.op=CompareOp::Greater, .threshold=0.45f},
        };
        g.min_hold_ms = 1500;
        g.cooldown_ms = 300;
        g.action = GE_ACTION_KEYBOARD_ANCHOR;
        g.priority = 1;
        g.track = {.type=TrackType::Position3D, .feature_name="palm_center"};
        ge->gestures.push_back(g);
    }

#endif  // dead old gesture blocks

// ---------------------------------------------------------------------------
// Public C API
// ---------------------------------------------------------------------------

ge_engine_t *ge_create(void) {
    auto *ge = new (std::nothrow) ge_engine_impl_t();
    if (!ge) {
        std::fprintf(stderr, "gesture_engine: out of memory\n");
        return nullptr;
    }
    add_default_gestures(ge);
    return ge;
}

void ge_destroy(ge_engine_t *ge) {
    delete ge;
}

// ---------------------------------------------------------------------------
// Minimal TOML reader
//
// We only need a tiny subset of TOML for gesture overrides:
//   * line comments starting with '#'
//   * section headers `[gesture]` / `[gesture.variant]` / `[filter]`
//   * key = value pairs where value is an integer, float, or bare string
//   * dotted keys (e.g. trigger.thumb_index_distance = 0.025, optionally
//     suffixed with a comparison selector: trigger.<feature>.<op>)
//
// This avoids a build-time dependency on libtomlplusplus while staying close
// to the TOML grammar for the values we consume.  Errors are printed to stderr
// and the offending line is skipped; partial overrides still apply.
// ---------------------------------------------------------------------------

namespace {

std::string trim(const std::string &s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

// Strip a trailing comment (everything after an unquoted '#').
std::string strip_comment(const std::string &s) {
    bool in_str = false;
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '"') in_str = !in_str;
        else if (!in_str && s[i] == '#') return s.substr(0, i);
    }
    return s;
}

// One key = value line, tagged with the section it appeared under.  Kept
// in file order so a bare [name] override followed by [name.variant]
// applies in the order written.
struct TomlEntry {
    std::string section;
    std::string key;
    std::string value;
    int         lineno;
};

bool parse_toml_file(const std::string &path, std::vector<TomlEntry> &out,
                     std::string &err) {
    std::ifstream in(path);
    if (!in) {
        err = "cannot open " + path;
        return false;
    }
    std::string section;
    std::set<std::string> seen_sections;
    bool section_rejected = false;
    std::string line;
    int lineno = 0;
    while (std::getline(in, line)) {
        ++lineno;
        line = trim(strip_comment(line));
        if (line.empty()) continue;

        if (line.front() == '[') {
            size_t end = line.find(']');
            if (end == std::string::npos) {
                std::fprintf(stderr,
                             "gesture_engine: %s:%d malformed section header\n",
                             path.c_str(), lineno);
                continue;
            }
            section = trim(line.substr(1, end - 1));
            // A repeated header is almost always a hand-edited copy of the
            // generated sample, where each same-name variant used to print
            // as a bare [name].  Silently merging them made the last block
            // win for every variant, so refuse the repeat outright.
            section_rejected = !seen_sections.insert(section).second;
            if (section_rejected) {
                std::fprintf(stderr,
                             "gesture_engine: %s:%d duplicate section [%s]; "
                             "ignoring it (address one variant as "
                             "[%s.<variant>])\n",
                             path.c_str(), lineno, section.c_str(),
                             section.c_str());
            }
            continue;
        }
        if (section_rejected) continue;

        size_t eq = line.find('=');
        if (eq == std::string::npos) {
            std::fprintf(stderr,
                         "gesture_engine: %s:%d expected key=value (got %s)\n",
                         path.c_str(), lineno, line.c_str());
            continue;
        }
        std::string key = trim(line.substr(0, eq));
        std::string val = trim(line.substr(eq + 1));
        if (val.size() >= 2 && val.front() == '"' && val.back() == '"') {
            val = val.substr(1, val.size() - 2);
        }
        if (key.empty()) {
            std::fprintf(stderr,
                         "gesture_engine: %s:%d empty key\n",
                         path.c_str(), lineno);
            continue;
        }
        out.push_back({section, key, val, lineno});
    }
    return true;
}

bool parse_float(const std::string &s, float &out) {
    char *end = nullptr;
    double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || (end && *end != '\0')) return false;
    out = static_cast<float>(v);
    return true;
}

bool parse_int(const std::string &s, int &out) {
    char *end = nullptr;
    long v = std::strtol(s.c_str(), &end, 10);
    if (end == s.c_str() || (end && *end != '\0')) return false;
    out = static_cast<int>(v);
    return true;
}

const char *op_to_str(ge::CompareOp op) {
    switch (op) {
        case ge::CompareOp::Less:      return "less";
        case ge::CompareOp::Greater:   return "greater";
        case ge::CompareOp::LessEq:    return "less_eq";
        case ge::CompareOp::GreaterEq: return "greater_eq";
    }
    return "?";
}

bool str_to_op(const std::string &s, ge::CompareOp &out) {
    if (s == "less")       { out = ge::CompareOp::Less;      return true; }
    if (s == "greater")    { out = ge::CompareOp::Greater;   return true; }
    if (s == "less_eq")    { out = ge::CompareOp::LessEq;    return true; }
    if (s == "greater_eq") { out = ge::CompareOp::GreaterEq; return true; }
    return false;
}

// Accessors take either "name" (first compiled definition with that name)
// or "name.variant".
bool gesture_matches(const ge::GestureDef &g, const std::string &query) {
    if (g.name == query) return true;
    return !g.variant.empty() && query == g.name + "." + g.variant;
}

const ge::GestureDef *find_gesture(const ge_engine_impl_t *ge,
                                   const std::string &query) {
    for (const auto &g : ge->gestures) {
        if (gesture_matches(g, query)) return &g;
    }
    return nullptr;
}

// Overrides every condition on `feature`; with `has_op`, only the one(s)
// with that comparison (a variant can carry two thresholds on one feature,
// e.g. scroll.wide's 20 mm < ti < 70 mm band).
bool override_threshold(std::vector<ge::Condition> &conds,
                        const std::string &feature, bool has_op,
                        ge::CompareOp op, float new_threshold) {
    bool any = false;
    for (auto &c : conds) {
        if (c.feature != feature) continue;
        if (has_op && c.op != op) continue;
        c.threshold = new_threshold;
        any = true;
    }
    return any;
}

bool apply_filter_override(ge_engine_impl_t *ge, const std::string &key,
                           const std::string &raw) {
    float v;
    if (!parse_float(raw, v)) {
        std::fprintf(stderr,
                     "gesture_engine: filter.%s: not a number ('%s')\n",
                     key.c_str(), raw.c_str());
        return false;
    }
    auto &fc = ge->filter_cfg;
    auto reject = [&](const char *rule, float keep) {
        std::fprintf(stderr,
                     "gesture_engine: filter.%s = %g rejected (%s); keeping %g\n",
                     key.c_str(), static_cast<double>(v), rule,
                     static_cast<double>(keep));
        return false;
    };
    if (key == "one_euro_enabled") {
        fc.one_euro_enabled = v != 0.0f;
    } else if (key == "one_euro_min_cutoff_hz") {
        if (!(v > 0.0f)) return reject("must be > 0", fc.one_euro.min_cutoff_hz);
        fc.one_euro.min_cutoff_hz = v;
    } else if (key == "one_euro_beta") {
        if (!(v >= 0.0f)) return reject("must be >= 0", fc.one_euro.beta);
        fc.one_euro.beta = v;
    } else if (key == "one_euro_d_cutoff_hz") {
        if (!(v > 0.0f)) return reject("must be > 0", fc.one_euro.d_cutoff_hz);
        fc.one_euro.d_cutoff_hz = v;
    } else if (key == "min_joint_confidence") {
        if (!(v >= 0.0f && v <= 1.0f))
            return reject("must be in [0, 1]", fc.min_joint_confidence);
        fc.min_joint_confidence = v;
    } else {
        std::fprintf(stderr,
                     "gesture_engine: ignoring filter.%s (no such field)\n",
                     key.c_str());
        return false;
    }
    return true;
}

// Applies one key to every gesture in `targets` (all variants of a name
// for a bare [name] section, one for [name.variant]).  Returns true when
// at least one definition changed.
bool apply_gesture_override(const std::vector<ge::GestureDef *> &targets,
                            const std::string &label, const std::string &key,
                            const std::string &raw) {
    if (key == "min_hold_ms" || key == "cooldown_ms" || key == "max_active_ms"
        || key == "indeterminate_release_ms") {
        int v;
        if (!parse_int(raw, v)) {
            std::fprintf(stderr,
                         "gesture_engine: %s.%s: not an integer ('%s')\n",
                         label.c_str(), key.c_str(), raw.c_str());
            return false;
        }
        for (auto *g : targets) {
            if (key == "min_hold_ms")            g->min_hold_ms = v;
            else if (key == "cooldown_ms")       g->cooldown_ms = v;
            else if (key == "max_active_ms")     g->max_active_ms = v;
            else                                 g->indeterminate_release_ms = v;
        }
        return true;
    }

    size_t dot = key.find('.');
    if (dot == std::string::npos) {
        std::fprintf(stderr,
                     "gesture_engine: ignoring '%s.%s' (no such field)\n",
                     label.c_str(), key.c_str());
        return false;
    }
    std::string kind    = key.substr(0, dot);
    std::string feature = key.substr(dot + 1);
    if (kind != "trigger" && kind != "release") {
        std::fprintf(stderr,
                     "gesture_engine: %s.%s: expected 'trigger' or 'release'\n",
                     label.c_str(), kind.c_str());
        return false;
    }

    // Optional trailing comparison selector: trigger.<feature>.<op>.
    bool has_op = false;
    ge::CompareOp op = ge::CompareOp::Less;
    size_t op_dot = feature.rfind('.');
    if (op_dot != std::string::npos) {
        if (!str_to_op(feature.substr(op_dot + 1), op)) {
            std::fprintf(stderr,
                         "gesture_engine: %s.%s: '%s' is not a comparison "
                         "(less / greater / less_eq / greater_eq)\n",
                         label.c_str(), key.c_str(),
                         feature.substr(op_dot + 1).c_str());
            return false;
        }
        has_op = true;
        feature = feature.substr(0, op_dot);
    }

    float v;
    if (!parse_float(raw, v)) {
        std::fprintf(stderr,
                     "gesture_engine: %s.%s: not a number ('%s')\n",
                     label.c_str(), key.c_str(), raw.c_str());
        return false;
    }

    bool any = false;
    for (auto *g : targets) {
        auto &conds = kind == "trigger" ? g->trigger_conditions
                                        : g->release_conditions;
        any |= override_threshold(conds, feature, has_op, op, v);
    }
    if (!any) {
        std::fprintf(stderr,
                     "gesture_engine: %s.%s: no matching condition; skipping\n",
                     label.c_str(), key.c_str());
    }
    return any;
}

// Default config path: $XDG_CONFIG_HOME/spatial-os/gestures.toml
// (falling back to $HOME/.config/spatial-os/gestures.toml).
std::string default_config_path() {
    if (const char *xdg = std::getenv("XDG_CONFIG_HOME"); xdg && *xdg) {
        return std::string(xdg) + "/spatial-os/gestures.toml";
    }
    if (const char *home = std::getenv("HOME"); home && *home) {
        return std::string(home) + "/.config/spatial-os/gestures.toml";
    }
    return "";
}

}  // anonymous namespace

bool ge_load_config(ge_engine_t *ge, const char *toml_path) {
    if (!ge) return false;

    std::string path = (toml_path && *toml_path) ? std::string(toml_path)
                                                  : default_config_path();
    if (path.empty()) {
        std::fprintf(stderr,
                     "gesture_engine: no config path supplied and HOME unset; "
                     "keeping compiled defaults\n");
        return false;
    }

    std::vector<TomlEntry> entries;
    std::string err;
    if (!parse_toml_file(path, entries, err)) {
        // Missing file is normal on first run — fall back silently.
        return false;
    }

    int applied = 0;
    int skipped = 0;
    for (const auto &e : entries) {
        if (e.section.empty()) {
            std::fprintf(stderr,
                         "gesture_engine: ignoring top-level key '%s' (expected [gesture] section)\n",
                         e.key.c_str());
            ++skipped;
            continue;
        }

        // [filter] section — global joint-filter tuning, not a gesture.
        if (e.section == "filter") {
            if (apply_filter_override(ge, e.key, e.value)) ++applied;
            else ++skipped;
            continue;
        }

        std::string gname = e.section;
        std::string variant;
        size_t dot = e.section.find('.');
        if (dot != std::string::npos) {
            gname = e.section.substr(0, dot);
            variant = e.section.substr(dot + 1);
        }
        std::vector<ge::GestureDef *> targets;
        bool name_known = false;
        for (auto &g : ge->gestures) {
            if (g.name != gname) continue;
            name_known = true;
            if (variant.empty() || g.variant == variant) targets.push_back(&g);
        }
        if (targets.empty()) {
            if (name_known) {
                std::fprintf(stderr,
                             "gesture_engine: ignoring unknown variant '%s' of '%s'\n",
                             variant.c_str(), gname.c_str());
            } else {
                std::fprintf(stderr,
                             "gesture_engine: ignoring unknown gesture '%s'\n",
                             gname.c_str());
            }
            ++skipped;
            continue;
        }

        if (apply_gesture_override(targets, e.section, e.key, e.value)) ++applied;
        else ++skipped;
    }

    std::fprintf(stderr,
                 "gesture_engine: loaded %s — %d overrides applied, %d skipped\n",
                 path.c_str(), applied, skipped);
    return applied > 0;
}

int ge_gesture_min_hold_ms(const ge_engine_t *ge, const char *gesture_name) {
    if (!ge || !gesture_name) return -1;
    const ge::GestureDef *g = find_gesture(ge, gesture_name);
    return g ? g->min_hold_ms : -1;
}

float ge_gesture_threshold(const ge_engine_t *ge, const char *gesture_name,
                           const char *kind, const char *feature) {
    if (!ge || !gesture_name || !kind || !feature) return 0.0f / 0.0f;  // NaN
    const ge::GestureDef *g = find_gesture(ge, gesture_name);
    if (!g) return 0.0f / 0.0f;
    const std::vector<ge::Condition> *conds = nullptr;
    if (std::strcmp(kind, "trigger") == 0)      conds = &g->trigger_conditions;
    else if (std::strcmp(kind, "release") == 0) conds = &g->release_conditions;
    else return 0.0f / 0.0f;
    for (const auto &c : *conds) {
        if (c.feature == feature) return c.threshold;
    }
    return 0.0f / 0.0f;
}

float ge_filter_param(const ge_engine_t *ge, const char *key) {
    if (!ge || !key) return 0.0f / 0.0f;  // NaN
    const auto &fc = ge->filter_cfg;
    std::string k(key);
    if (k == "one_euro_enabled")        return (float)fc.one_euro_enabled;
    if (k == "one_euro_min_cutoff_hz")  return fc.one_euro.min_cutoff_hz;
    if (k == "one_euro_beta")           return fc.one_euro.beta;
    if (k == "one_euro_d_cutoff_hz")    return fc.one_euro.d_cutoff_hz;
    if (k == "min_joint_confidence")    return fc.min_joint_confidence;
    return 0.0f / 0.0f;
}

int ge_gesture_cooldown_ms(const ge_engine_t *ge, const char *gesture_name) {
    if (!ge || !gesture_name) return -1;
    const ge::GestureDef *g = find_gesture(ge, gesture_name);
    return g ? g->cooldown_ms : -1;
}

int ge_gesture_max_active_ms(const ge_engine_t *ge, const char *gesture_name) {
    if (!ge || !gesture_name) return -1;
    const ge::GestureDef *g = find_gesture(ge, gesture_name);
    return g ? g->max_active_ms : -1;
}

int ge_gesture_indeterminate_release_ms(const ge_engine_t *ge,
                                        const char *gesture_name) {
    if (!ge || !gesture_name) return -1;
    const ge::GestureDef *g = find_gesture(ge, gesture_name);
    return g ? g->indeterminate_release_ms : -1;
}

int ge_gesture_count(const ge_engine_t *ge) {
    return ge ? static_cast<int>(ge->gestures.size()) : 0;
}

const char *ge_gesture_name_at(const ge_engine_t *ge, int index) {
    if (!ge || index < 0 || index >= static_cast<int>(ge->gestures.size()))
        return nullptr;
    return ge->gestures[index].name.c_str();
}

const char *ge_gesture_variant_at(const ge_engine_t *ge, int index) {
    if (!ge || index < 0 || index >= static_cast<int>(ge->gestures.size()))
        return nullptr;
    return ge->gestures[index].variant.c_str();
}

namespace {

const std::vector<ge::Condition> *conditions_of(const ge_engine_impl_t *ge,
                                                int index, const char *kind) {
    if (!ge || !kind || index < 0
        || index >= static_cast<int>(ge->gestures.size()))
        return nullptr;
    const auto &g = ge->gestures[index];
    if (std::strcmp(kind, "trigger") == 0) return &g.trigger_conditions;
    if (std::strcmp(kind, "release") == 0) return &g.release_conditions;
    return nullptr;
}

}  // anonymous namespace

int ge_gesture_condition_count(const ge_engine_t *ge, int index,
                               const char *kind) {
    const auto *conds = conditions_of(ge, index, kind);
    return conds ? static_cast<int>(conds->size()) : -1;
}

bool ge_gesture_condition_at(const ge_engine_t *ge, int index,
                             const char *kind, int cond_index,
                             const char **feature, const char **op,
                             float *threshold) {
    const auto *conds = conditions_of(ge, index, kind);
    if (!conds || cond_index < 0
        || cond_index >= static_cast<int>(conds->size()))
        return false;
    const auto &c = (*conds)[cond_index];
    if (feature)   *feature = c.feature.c_str();
    if (op)        *op = op_to_str(c.op);
    if (threshold) *threshold = c.threshold;
    return true;
}

namespace {

const char *action_to_str(ge_action_t a) {
    switch (a) {
        case GE_ACTION_POINTER_CLICK:       return "POINTER_CLICK";
        case GE_ACTION_SCROLL:              return "SCROLL";
        case GE_ACTION_WINDOW_MOVE:         return "WINDOW_MOVE";
        case GE_ACTION_WINDOW_RESIZE:       return "WINDOW_RESIZE";
        case GE_ACTION_TOGGLE_LAUNCHER:     return "TOGGLE_LAUNCHER";
        case GE_ACTION_POINTER_RIGHT_CLICK: return "POINTER_RIGHT_CLICK";
        case GE_ACTION_FINE_SLIDER:         return "FINE_SLIDER";
        case GE_ACTION_WINDOW_CLOSE:        return "WINDOW_CLOSE";
        case GE_ACTION_TOGGLE_KEYBOARD:     return "TOGGLE_KEYBOARD";
        case GE_ACTION_KEYBOARD_ANCHOR:     return "KEYBOARD_ANCHOR";
        case GE_ACTION_CUSTOM:              return "CUSTOM";
    }
    return "?";
}

// Format a float deterministically with %g so the output round-trips
// through ge_load_config exactly.
std::string format_threshold(float v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g", static_cast<double>(v));
    return std::string(buf);
}

void print_conditions(FILE *f, const char *kind,
                      const std::vector<ge::Condition> &conds) {
    std::map<std::string, int> uses;
    for (const auto &c : conds) ++uses[c.feature];
    for (const auto &c : conds) {
        // A feature that appears twice in one list (a band such as
        // 20 mm < ti < 70 mm) needs the comparison selector to stay
        // addressable; a bare key would set both thresholds.
        std::string key = c.feature;
        if (uses[c.feature] > 1) key += std::string(".") + op_to_str(c.op);
        std::fprintf(f, "%s.%-30s = %-8s  # %s\n", kind, key.c_str(),
                     format_threshold(c.threshold).c_str(), op_to_str(c.op));
    }
}

}  // anonymous namespace

void ge_print_config(const ge_engine_t *ge, FILE *f) {
    if (!f) f = stdout;
    if (!ge) return;

    std::fprintf(f,
        "# AUTO-GENERATED via tools/print_default_config — do not edit by hand.\n"
        "#\n"
        "# Canonical sample for ~/.config/spatial-os/gestures.toml.  Every value\n"
        "# is the compiled default in gesture-engine/src/engine.cpp; loading this\n"
        "# file unchanged is a no-op (tests/test_config.cpp verifies that).\n"
        "#\n"
        "# Schema:\n"
        "#   [gesture_name]            applies to EVERY variant of that name\n"
        "#   [gesture_name.variant]    applies to one variant only\n"
        "#   min_hold_ms               = <int>     # how long trigger must hold before BEGIN\n"
        "#   cooldown_ms               = <int>     # min gap after END/CANCEL before re-trigger\n"
        "#   max_active_ms             = <int>     # auto-END after this long Active (0 = never)\n"
        "#   indeterminate_release_ms  = <int>     # give up after this long of NaN features\n"
        "#   trigger.<feature_name>    = <float>   # threshold for a trigger condition\n"
        "#   release.<feature_name>    = <float>   # threshold for a release condition\n"
        "#   trigger.<feature_name>.<op> = <float> # one of two conditions on the same feature\n"
        "#\n"
        "# A gesture name with several compiled definitions (variants) prints one\n"
        "# [name.variant] section per definition.  Repeating a section header is\n"
        "# an error; the repeat is ignored.\n"
        "#\n"
        "# Comparison operators (less / greater / less_eq / greater_eq) are NOT\n"
        "# overridable here — the operator is part of the gesture definition.\n"
        "# Only the threshold value is tuneable; the operator is shown as a\n"
        "# trailing comment for reference (and as a key suffix only where a\n"
        "# feature carries two conditions in one list).\n"
        "#\n"
        "# Gestures are listed in priority-descending order (the order the\n"
        "# engine's suppression matrix iterates them in).\n");

    std::fprintf(f,
        "\n[filter]\n"
        "# Global One-Euro jitter filter on joint positions feeding the FSM\n"
        "# (see src/one_euro.h).  min_cutoff governs jitter rejection at\n"
        "# rest; beta opens the cutoff with speed so intentional motion\n"
        "# stays snappy; d_cutoff smooths the speed estimate.  Joints below\n"
        "# min_joint_confidence are excluded from triggers (indeterminate).\n"
        "# min_cutoff and d_cutoff must be > 0, beta >= 0, confidence in [0, 1];\n"
        "# anything else is logged and the compiled default kept.\n"
        "one_euro_enabled       = %d\n"
        "one_euro_min_cutoff_hz = %s\n"
        "one_euro_beta          = %s\n"
        "one_euro_d_cutoff_hz   = %s\n"
        "min_joint_confidence   = %s\n",
        ge->filter_cfg.one_euro_enabled,
        format_threshold(ge->filter_cfg.one_euro.min_cutoff_hz).c_str(),
        format_threshold(ge->filter_cfg.one_euro.beta).c_str(),
        format_threshold(ge->filter_cfg.one_euro.d_cutoff_hz).c_str(),
        format_threshold(ge->filter_cfg.min_joint_confidence).c_str());

    for (const auto &g : ge->gestures) {
        if (g.variant.empty())
            std::fprintf(f, "\n[%s]\n", g.name.c_str());
        else
            std::fprintf(f, "\n[%s.%s]\n", g.name.c_str(), g.variant.c_str());
        std::fprintf(f, "# action=GE_ACTION_%s priority=%d\n",
                     action_to_str(g.action), g.priority);
        std::fprintf(f, "min_hold_ms = %d\n", g.min_hold_ms);
        std::fprintf(f, "cooldown_ms = %d\n", g.cooldown_ms);
        std::fprintf(f, "max_active_ms = %d\n", g.max_active_ms);
        std::fprintf(f, "indeterminate_release_ms = %d\n",
                     g.indeterminate_release_ms);
        print_conditions(f, "trigger", g.trigger_conditions);
        print_conditions(f, "release", g.release_conditions);
    }
}

void ge_print_default_config(FILE *f) {
    // Build a fresh engine so we capture compiled defaults without any
    // user-applied overrides leaking in.
    ge_engine_t *ge = ge_create();
    if (!ge) return;
    ge_print_config(ge, f);
    ge_destroy(ge);
}

void ge_set_callback(ge_engine_t *ge, ge_event_callback_t cb, void *user_data) {
    if (!ge) return;
    ge->callback = cb;
    ge->user_data = user_data;
}

void ge_update(ge_engine_t *ge, const ge_hand_t hands[2], float dt_seconds) {
    if (!ge) return;

    float dt_ms = dt_seconds * 1000.0f;

    // -----------------------------------------------------------------
    // Hand-identity remap.  Convert the two iPhone Vision tracker slots
    // ("input slots") into two stable engine-side hand identities by
    // position continuity, so per-hand state (recencies, ring buffers,
    // FSM Pending/Active) survives tracker slot churn.
    //
    // Three cases the iPhone Vision tracker generates that this
    // resolves:
    //   1) Duplicate (clip 5ca5ef): the same physical hand appears in
    //      both slots simultaneously.  Drop one, keep the slot that
    //      the engine was bound to last tick.
    //   2) Swap (clip 5871b8): a hand tracked in slot 1 starts also
    //      appearing in slot 0, then slot 1 goes garbage (wrist at
    //      origin).  Engine hand 0 stays bound to whichever input
    //      slot's wrist is closest to its last position — when slot
    //      1 dies, engine 0 follows the data into slot 0.
    //   3) Origin garbage: a stale slot reports wrist = (0,0,0).
    //      Treated as not-present.
    // -----------------------------------------------------------------
    bool input_valid[2];
    ge::Vec3 input_wrist[2];
    for (int s = 0; s < 2; ++s) {
        input_valid[s] = hands[s].present;
        if (input_valid[s]) {
            input_wrist[s] = ge::joint_pos(hands[s], GE_JOINT_WRIST);
            // Tracker-garbage signature: hand is "present" but every
            // joint is at the world origin (the stale-slot output the
            // iPhone Vision tracker emits when it loses one of two
            // hands).  Check wrist AND index-tip — synthetic test
            // fixtures put the wrist at origin with fingers spreading
            // outward, which is NOT garbage.
            ge::Vec3 tip = ge::joint_pos(hands[s], GE_JOINT_INDEX_TIP);
            if (input_wrist[s].length() < 1e-4f && tip.length() < 1e-4f) {
                input_valid[s] = false;
            }
        }
    }

    // Tracker-duplicate detection: two input slots reporting the same
    // 3-D wrist position MUST be the same hand (two hands cannot
    // occupy one point).  Drop the slot the engine wasn't using last
    // tick — falls back to slot 1 when there's no engine history yet.
    if (input_valid[0] && input_valid[1] &&
        ge::distance(input_wrist[0], input_wrist[1]) < 0.05f) {
        int prefer_slot = -1;
        for (int e = 0; e < 2; ++e) {
            if (ge->engine_hand_last_bound_slot[e] >= 0
                && ge->engine_hand_inactive_ms[e] < 1000.0f) {
                prefer_slot = ge->engine_hand_last_bound_slot[e];
                break;
            }
        }
        if (prefer_slot == 1) input_valid[0] = false;
        else input_valid[1] = false;  // default: keep slot 0
    }

    // Decay engine-hand inactivity. An engine hand is considered "lost"
    // once it goes 1000 ms without a continuity match.
    for (int e = 0; e < 2; ++e) {
        ge->engine_hand_inactive_ms[e] += dt_ms;
    }

    // input_for_engine[e] = which input slot is bound to engine hand e
    int input_for_engine[2] = {-1, -1};

    // Pass 1: continuity match — each valid input slot binds to the
    // engine hand whose last-known wrist is closest, within 0.15 m.
    // 0.15 m covers reasonable per-tick motion (~9 m/s at 60 Hz) but
    // not a teleport.  Engine hands inactive for >1 s are skipped.
    for (int s = 0; s < 2; ++s) {
        if (!input_valid[s]) continue;
        float best_d = 0.15f;
        int best_e = -1;
        for (int e = 0; e < 2; ++e) {
            if (input_for_engine[e] >= 0) continue;
            if (ge->engine_hand_inactive_ms[e] >= 1000.0f) continue;
            if (ge->engine_hand_last_bound_slot[e] < 0) continue;
            float d = ge::distance(input_wrist[s], ge->engine_hand_last_wrist[e]);
            if (d < best_d) { best_d = d; best_e = e; }
        }
        if (best_e >= 0) input_for_engine[best_e] = s;
    }

    // Pass 2: unmatched valid input slots take a free engine slot.
    for (int s = 0; s < 2; ++s) {
        if (!input_valid[s]) continue;
        if (input_for_engine[0] == s || input_for_engine[1] == s) continue;
        for (int e = 0; e < 2; ++e) {
            if (input_for_engine[e] < 0) { input_for_engine[e] = s; break; }
        }
    }

    // Bookkeeping: update engine-hand wrist + inactivity.
    bool engine_hand_present[2] = {false, false};
    for (int e = 0; e < 2; ++e) {
        if (input_for_engine[e] >= 0) {
            int s = input_for_engine[e];
            ge->engine_hand_last_wrist[e] = input_wrist[s];
            ge->engine_hand_inactive_ms[e] = 0.0f;
            ge->engine_hand_last_bound_slot[e] = s;
            engine_hand_present[e] = true;
        }
    }

    // Compute features for each present hand
    for (int h = 0; h < 2; ++h) {
        ge->hand_present[h] = engine_hand_present[h];
        if (engine_hand_present[h]) {
            const ge_hand_t &src = hands[input_for_engine[h]];
            ge_hand_t filtered = src;
            if (ge->filter_cfg.one_euro_enabled) {
                for (int j = 0; j < GE_JOINT_COUNT; ++j) {
                    // Zero-frame artifacts and low-confidence joints must
                    // not poison the filter history: pass raw (the NaN
                    // feature machinery neutralises them) and reset.
                    ge::Vec3 p = ge::joint_pos(src, j);
                    bool zero_artifact = std::fabs(p.x) < 1e-5f
                                      && std::fabs(p.y) < 1e-5f
                                      && std::fabs(p.z) < 1e-5f;
                    bool low_conf = src.joints[j][3]
                                    < ge->filter_cfg.min_joint_confidence;
                    if (zero_artifact || low_conf) {
                        for (int a = 0; a < 3; ++a)
                            ge->joint_filter[h][j][a].reset();
                        continue;
                    }
                    for (int a = 0; a < 3; ++a) {
                        filtered.joints[j][a]
                            = ge->joint_filter[h][j][a].update(
                                src.joints[j][a], dt_seconds,
                                ge->filter_cfg.one_euro);
                    }
                }
            }
            ge->features[h] = ge::compute_features(
                filtered, ge->filter_cfg.min_joint_confidence);

            // Wrist speed (m/s), EMA over ~100 ms of the filtered wrist.
            {
                constexpr float kSpeedEmaTauS = 0.1f;
                float inst = 0.0f;
                if (ge->has_prev_wrist_for_speed[h] && dt_seconds > 0.0f) {
                    inst = ge::distance(ge->features[h].wrist_pos,
                                        ge->prev_wrist_for_speed[h])
                           / dt_seconds;
                }
                float blend = dt_seconds / (dt_seconds + kSpeedEmaTauS);
                ge->wrist_speed_mps[h]
                    += blend * (inst - ge->wrist_speed_mps[h]);
                ge->prev_wrist_for_speed[h] = ge->features[h].wrist_pos;
                ge->has_prev_wrist_for_speed[h] = true;
            }
            ge->features[h].wrist_speed_mps = ge->wrist_speed_mps[h];

            // Temporal palm-flip tracker.  Flip = sign change on
            // palm_normal.y between two consecutive ticks while both
            // magnitudes are large (> 0.5) — a real flip, not a noise
            // crossover near zero.  Recency increments each tick;
            // resets to 0 on detected flip.
            float cur_py = ge->features[h].palm_normal.y;
            if (ge->palm_y_prev_valid[h]) {
                float prev_py = ge->palm_y_prev[h];
                bool flipped = (cur_py * prev_py < 0.0f)
                               && std::fabs(cur_py)  > 0.5f
                               && std::fabs(prev_py) > 0.5f;
                if (flipped) {
                    ge->palm_y_flip_recency_ms[h] = 0.0f;
                } else {
                    ge->palm_y_flip_recency_ms[h] += dt_ms;
                }
            }
            ge->palm_y_prev[h] = cur_py;
            ge->palm_y_prev_valid[h] = true;

            ge->features[h].palm_y_flip_recency_ms
                = ge->palm_y_flip_recency_ms[h];

            // Thumb-index "recent convergence" tracker.
            if (ge->features[h].thumb_index_distance > 0.030f) {
                ge->ti_recency_above_30mm_ms[h] = 0.0f;
            } else {
                ge->ti_recency_above_30mm_ms[h] += dt_ms;
            }
            ge->features[h].thumb_index_recency_above_30mm_ms
                = ge->ti_recency_above_30mm_ms[h];
            // Thumb-middle "recent convergence" tracker.
            if (ge->features[h].thumb_middle_distance > 0.060f) {
                ge->tm_recency_above_60mm_ms[h] = 0.0f;
            } else {
                ge->tm_recency_above_60mm_ms[h] += dt_ms;
            }
            ge->features[h].thumb_middle_recency_above_60mm_ms
                = ge->tm_recency_above_60mm_ms[h];
            // pinch_select END recency: increments each tick;
            // resets to 0 in the Active→Idle (END/CANCEL) handlers.
            ge->pinch_select_end_recency_ms[h] += dt_ms;
            ge->features[h].pinch_select_end_recency_ms
                = ge->pinch_select_end_recency_ms[h];
            // palm_y < -0.5 recency: fist-launcher poses look like
            // wide-arc-scroll after the rotation; this gates that
            // out.
            if (ge->features[h].palm_normal.y < -0.5f) {
                ge->palm_y_below_neg_half_recency_ms[h] = 0.0f;
            } else {
                ge->palm_y_below_neg_half_recency_ms[h] += dt_ms;
            }
            ge->features[h].palm_y_below_neg_half_recency_ms
                = ge->palm_y_below_neg_half_recency_ms[h];
            // fist_launcher END recency: increments each tick;
            // resets to 0 in the Active→Idle handlers when the
            // ending gesture's name is "fist_launcher".
            ge->fist_launcher_end_recency_ms[h] += dt_ms;
            ge->features[h].fist_launcher_end_recency_ms
                = ge->fist_launcher_end_recency_ms[h];

            // Shape-of-convergence trackers. Push current ti/tm into
            // their ring buffers, then compute the three derived
            // features over the documented windows.
            history_push(ge->ti_history[h],
                         ge->features[h].thumb_index_distance, dt_ms);
            history_push(ge->tm_history[h],
                         ge->features[h].thumb_middle_distance, dt_ms);
            history_push(ge->perp_history[h],
                         ge->features[h].thumb_to_index_line_distance, dt_ms);
            history_push(ge->index_curl_history[h],
                         ge->features[h].index_curl, dt_ms);
            history_push(ge->middle_curl_history[h],
                         ge->features[h].middle_curl, dt_ms);
            history_push(ge->ring_curl_history[h],
                         ge->features[h].ring_curl, dt_ms);
            history_push(ge->middle_conf_history[h],
                         ge->features[h].middle_tip_confidence, dt_ms);
            history_push(ge->proj_history[h],
                         ge->features[h].thumb_on_index_projection, dt_ms);
            ge->features[h].thumb_index_distance_min_over_50ms
                = window_min(ge->ti_history[h], 50.0f);
            ge->features[h].thumb_index_distance_min_over_200ms
                = window_min(ge->ti_history[h], 200.0f);
            ge->features[h].thumb_index_convergence_velocity_mps
                = window_max_closing_velocity_mps(ge->ti_history[h], 200.0f);
            ge->features[h].thumb_index_convergence_ratio
                = window_peak_to_valley_ratio(ge->ti_history[h], 500.0f);
            ge->features[h].thumb_middle_distance_min_over_200ms
                = window_min(ge->tm_history[h], 200.0f);
            ge->features[h].thumb_middle_convergence_velocity_mps
                = window_max_closing_velocity_mps(ge->tm_history[h], 200.0f);
            ge->features[h].thumb_middle_convergence_ratio
                = window_peak_to_valley_ratio(ge->tm_history[h], 500.0f);
            ge->features[h].thumb_to_index_line_distance_max_over_200ms
                = window_max(ge->perp_history[h], 200.0f);
            ge->features[h].index_curl_max_over_200ms
                = window_max(ge->index_curl_history[h], 200.0f);
            ge->features[h].middle_curl_max_over_200ms
                = window_max(ge->middle_curl_history[h], 200.0f);
            ge->features[h].ring_curl_max_over_200ms
                = window_max(ge->ring_curl_history[h], 200.0f);
            ge->features[h].middle_tip_confidence_min_over_200ms
                = window_min(ge->middle_conf_history[h], 200.0f);
            // Projection range: (max - min) of thumb_on_index_projection
            // over a 100ms trailing window.  Captures CURRENT motion
            // only — 100ms is enough to see thumb sliding (scroll) but
            // too short to catch the approach transition (pinch).
            {
                float pmin = window_min(ge->proj_history[h], 100.0f);
                float pmax = window_max(ge->proj_history[h], 100.0f);
                ge->features[h].thumb_on_index_projection_velocity
                    = pmax - pmin;
            }
            {
                float pmin = window_min(ge->proj_history[h], 500.0f);
                float pmax = window_max(ge->proj_history[h], 500.0f);
                ge->features[h].thumb_on_index_projection_range_500ms
                    = pmax - pmin;
            }
        } else {
            // Hand lost — invalidate tracker so re-acquisition doesn't
            // count as a flip.
            ge->palm_y_prev_valid[h] = false;
            ge->palm_y_flip_recency_ms[h] = 1e6f;
            // Reset convergence-recency to 0 on hand loss so a real
            // gesture isn't blocked on the very first frame after
            // tracker re-acquisition (clip 5ca5ef: user's right hand
            // swaps to left briefly mid-clip; the convergence
            // happened during the swap).  This costs us false fires
            // when the user re-enters frame already in pinch pose,
            // but those are rarer than recovery cases.
            ge->ti_recency_above_30mm_ms[h] = 0.0f;
            ge->tm_recency_above_60mm_ms[h] = 0.0f;
            ge->pinch_select_end_recency_ms[h] = 1e6f;
            ge->palm_y_below_neg_half_recency_ms[h] = 1e6f;
            ge->fist_launcher_end_recency_ms[h] = 1e6f;
            // Drop shape-window history on hand loss so a re-acquisition
            // starts from a fresh window (same idiom as the recency
            // trackers above).
            history_clear(ge->ti_history[h]);
            history_clear(ge->tm_history[h]);
            history_clear(ge->perp_history[h]);
            history_clear(ge->index_curl_history[h]);
            history_clear(ge->middle_curl_history[h]);
            history_clear(ge->ring_curl_history[h]);
            history_clear(ge->middle_conf_history[h]);
            history_clear(ge->proj_history[h]);
            for (int j = 0; j < GE_JOINT_COUNT; ++j)
                for (int a = 0; a < 3; ++a)
                    ge->joint_filter[h][j][a].reset();
            ge->wrist_speed_mps[h] = 0.0f;
            ge->has_prev_wrist_for_speed[h] = false;
            for (auto it = ge->disarmed.begin(); it != ge->disarmed.end();) {
                if (it->first.first == h) it = ge->disarmed.erase(it);
                else ++it;
            }
        }
    }

    // Track which gestures are active this frame (for priority suppression).
    // Gestures are sorted priority-descending; an Active gesture claims the
    // hand, blocking lower-priority gestures from entering Pending.
    bool hand_claimed[2] = {false, false};
    // Which gesture name claimed each hand (for coexist_with checks).
    // When a higher-priority gesture has coexist_with = "X", gesture X
    // stays Active instead of being cancelled.
    std::string hand_claimed_coexist[2];

    // Per-hand pre-claim by gesture NAME.  When two GestureDefs share a
    // name (e.g. the three pinch_right_click variants — standard /
    // occluded-middle / aimed-middle), one going Active should NOT be
    // preempted by another with the same name promoting later, even
    // if their iteration order shuffles.  Without this, clip 778bb2
    // fires pinch_right_click twice (standard BEGIN @ 944 ms, then
    // occluded BEGIN + standard CANCEL @ 1111 ms).  The pre-claim
    // suppresses any later same-name variant from promoting Pending →
    // Active on the same hand, but leaves cross-name preemption alone
    // (a higher-priority pinch_right_click can still cancel a lower-
    // priority pinch_select via the usual hand_claimed path).
    std::set<std::pair<int, std::string>> name_active;
    for (size_t gi = 0; gi < ge->gestures.size(); ++gi) {
        for (int h = 0; h < 2; ++h) {
            if (!ge->hand_present[h]) continue;
            if (ge->runtime[gi * 2 + h].state == ge::GestureState::Active) {
                name_active.insert({h, ge->gestures[gi].name});
            }
        }
    }

    // Names whose trigger is still held this tick (any variant, Met or
    // Indeterminate within its indeterminate_release_ms budget) — consumed
    // after the loop to clear rearm-on-release disarms only once the pose
    // has genuinely released.
    std::set<std::pair<int, std::string>> name_trigger_held;

    for (size_t gi = 0; gi < ge->gestures.size(); ++gi) {
        const auto &gdef = ge->gestures[gi];

        for (int h = 0; h < 2; ++h) {
            // Skip if gesture is locked to a specific hand
            if (gdef.hand >= 0 && gdef.hand != h) continue;

            auto &rt = ge->runtime[gi * 2 + h];
            const auto &feat = ge->features[h];
            bool present = ge->hand_present[h];

            if (!present) {
                // Hand lost — cancel any active gesture
                if (rt.state == ge::GestureState::Active) {
                    if (ge->callback) {
                        ge_event_t ev{};
                        ev.type = GE_EVENT_CANCEL;
                        ev.action = gdef.action;
                        ev.gesture_name = gdef.name.c_str();
                        ev.hand_index = h;
                        ge->callback(&ev, ge->user_data);
                    }
                }
                // Full reset including cooldown — hand reappearing after
                // a tracking-loss break should be treated as a fresh
                // session, not a continuation.
                rt = ge::GestureRuntime{};
                continue;
            }

            ge::TriggerEval trig
                = ge::evaluate_trigger(gdef.trigger_conditions, feat);
            bool trigger_met = trig == ge::TriggerEval::Met;
            bool trigger_indeterminate
                = trig == ge::TriggerEval::Indeterminate;
            ge::TriggerEval rel = gdef.release_conditions.empty()
                ? ge::TriggerEval::NotMet
                : ge::evaluate_release(gdef.release_conditions, feat);
            bool release_met = rel == ge::TriggerEval::Met;
            bool release_indeterminate = rel == ge::TriggerEval::Indeterminate;

            // Indeterminate budget: consecutive ms the evaluation that
            // matters in this state has read NaN.  Active cares about
            // release; every other state about the trigger.  Past
            // indeterminate_release_ms the gesture stops waiting for the
            // joint to come back (see GestureDef).
            bool indeterminate_now = rt.state == ge::GestureState::Active
                ? release_indeterminate : trigger_indeterminate;
            if (indeterminate_now) rt.indeterminate_ms += dt_ms;
            else rt.indeterminate_ms = 0.0f;
            bool indeterminate_expired = rt.indeterminate_ms
                >= static_cast<float>(gdef.indeterminate_release_ms);

            if (trigger_met
                || (trigger_indeterminate && !indeterminate_expired))
                name_trigger_held.insert({h, gdef.name});

            // Cooldown decay (applies whenever the gesture is idle, even
            // if the trigger is currently met — the whole point is to
            // suppress an immediate re-fire after END/CANCEL).
            if (rt.cooldown_remaining_ms > 0.0f) {
                rt.cooldown_remaining_ms -= dt_ms;
                if (rt.cooldown_remaining_ms < 0.0f)
                    rt.cooldown_remaining_ms = 0.0f;
            }

            // Loop the Idle → Pending → Active state-machine so a
            // gesture with min_hold_ms == 0 and no motion gate fires
            // BEGIN on the same tick its trigger first matches.
            // Without same-tick promotion, the gesture has to wait
            // one full frame in Pending — which lets a higher-priority
            // motion-gated gesture (scroll) overtake it on the next
            // frame and cancel it before BEGIN ever fired.  Consumer
            // wants the sequence CLICK BEGIN → CLICK CANCEL →
            // SCROLL BEGIN so it can decide on END whether a click
            // commits.  Two iterations is enough for the worst case
            // (Idle→Pending→Active in one tick).
            for (int iter = 0; iter < 2; ++iter) {
            ge::GestureState entry_state = rt.state;
            switch (rt.state) {
            case ge::GestureState::Idle:
                if (trigger_met && !hand_claimed[h]
                    && rt.cooldown_remaining_ms == 0.0f
                    && ge->disarmed.count({h, gdef.name}) == 0) {
                    rt.state = ge::GestureState::Pending;
                    rt.hold_time_ms = 0;
                    rt.has_prev_tracked = false;
                    if (gdef.track.type == ge::TrackType::Position3D) {
                        rt.pending_start_pos
                            = ge::get_tracked_position(gdef.track, feat);
                        rt.has_pending_start_pos = true;
                    } else {
                        rt.has_pending_start_pos = false;
                    }
                    if (gdef.scalar_gate_threshold > 0.0f
                        && !gdef.scalar_gate_feature.empty()) {
                        rt.pending_gate_start_scalar
                            = ge::get_feature_by_name(feat,
                                gdef.scalar_gate_feature);
                        rt.has_pending_gate_start_scalar = true;
                    } else {
                        rt.has_pending_gate_start_scalar = false;
                    }
                }
                break;

            case ge::GestureState::Pending:
                if (trigger_indeterminate && !indeterminate_expired
                    && !release_met && !hand_claimed[h]) {
                    // Zero-frame artifact mid-hold: trigger "not met" this
                    // frame, but the hold timer FREEZES rather than resets —
                    // a 1-frame artifact in a real pinch must not
                    // release-then-retrigger.  The freeze is capped by
                    // indeterminate_release_ms; past that the hold resets.
                } else if (release_met || !trigger_met) {
                    rt.state = ge::GestureState::Idle;
                    rt.hold_time_ms = 0;
                    rt.has_pending_start_pos = false;
                } else if (hand_claimed[h]) {
                    // Higher-priority gesture claimed this hand
                    rt.state = ge::GestureState::Idle;
                    rt.hold_time_ms = 0;
                    rt.has_pending_start_pos = false;
                } else {
                    rt.hold_time_ms += dt_ms;
                    bool hold_ok = rt.hold_time_ms
                                   >= static_cast<float>(gdef.min_hold_ms);

                    // Motion gate: if motion_threshold_m > 0 and the
                    // gesture tracks a 3D position, also require the
                    // tracked point to have moved past the threshold
                    // since Pending entry.  Used by `scroll` so BEGIN
                    // fires the instant the pinched hand starts
                    // moving, NOT after some hold timer.
                    bool motion_ok = true;
                    if (gdef.motion_threshold_m > 0.0f
                        && gdef.track.type == ge::TrackType::Position3D
                        && rt.has_pending_start_pos) {
                        ge::Vec3 cur = ge::get_tracked_position(gdef.track, feat);
                        float dx = cur.x - rt.pending_start_pos.x;
                        float dy = cur.y - rt.pending_start_pos.y;
                        float dz = cur.z - rt.pending_start_pos.z;
                        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                        motion_ok = dist >= gdef.motion_threshold_m;
                    }
                    // Scalar motion gate independent of `track`.
                    if (motion_ok
                        && gdef.scalar_gate_threshold > 0.0f
                        && rt.has_pending_gate_start_scalar) {
                        float cur = ge::get_feature_by_name(feat,
                            gdef.scalar_gate_feature);
                        float delta = cur - rt.pending_gate_start_scalar;
                        if (delta < 0) delta = -delta;
                        motion_ok = delta >= gdef.scalar_gate_threshold;
                    }

                    // Same-name suppression: if another GestureDef
                    // with this same name is already Active on this
                    // hand, don't promote — the active variant owns
                    // the gesture for this hand.  See the name_active
                    // pre-pass comment above for the 778bb2 bug this
                    // prevents.
                    // Same-name suppression considers BOTH hands —
                    // when the iPhone Vision tracker swaps the
                    // dominant hand mid-gesture (clip 5ca5ef has
                    // 4 left-hand packets followed by 9+ right-
                    // hand packets, both consistent with a single
                    // right_click), only one BEGIN should fire.
                    bool name_blocked
                        = (name_active.count({0, gdef.name}) > 0
                           || name_active.count({1, gdef.name}) > 0)
                          && rt.state == ge::GestureState::Pending;
                    if (hold_ok && motion_ok && !name_blocked) {
                        // Transition to Active — emit BEGIN
                        rt.state = ge::GestureState::Active;
                        hand_claimed[h] = true;
                        if (!gdef.coexist_with.empty())
                            hand_claimed_coexist[h] = gdef.coexist_with;
                        rt.active_time_ms = 0;
                        rt.has_pending_start_pos = false;

                        // Initialize tracking
                        if (gdef.track.type == ge::TrackType::Scalar) {
                            rt.prev_tracked_value = ge::get_tracked_scalar(gdef.track, feat);
                            rt.has_prev_tracked = true;
                        } else if (gdef.track.type == ge::TrackType::Position3D) {
                            rt.prev_tracked_pos = ge::get_tracked_position(gdef.track, feat);
                            rt.has_prev_tracked = true;
                        }

                        if (ge->callback) {
                            ge_event_t ev{};
                            ev.type = GE_EVENT_BEGIN;
                            ev.action = gdef.action;
                            ev.gesture_name = gdef.name.c_str();
                            ev.hand_index = h;
                            ev.position[0] = feat.pinch_midpoint.x;
                            ev.position[1] = feat.pinch_midpoint.y;
                            ev.position[2] = feat.pinch_midpoint.z;
                            ge->callback(&ev, ge->user_data);
                        }
                    }
                }
                break;

            case ge::GestureState::Active: {
                bool coexisted = hand_claimed[h]
                    && !hand_claimed_coexist[h].empty()
                    && hand_claimed_coexist[h] == gdef.name;
                if (hand_claimed[h] && !coexisted) {
                    // A higher-priority gesture has claimed this hand on
                    // this frame and does NOT coexist with us.  Cancel.
                    rt.state = ge::GestureState::Idle;
                    rt.hold_time_ms = 0;
                    rt.has_prev_tracked = false;
                    rt.cooldown_remaining_ms
                        = static_cast<float>(gdef.cooldown_ms);
                    ge->disarmed[{h, gdef.name}] = 0.0f;
                    if (gdef.name == "pinch_select") {
                        ge->pinch_select_end_recency_ms[h] = 0.0f;
                    }
                    if (gdef.name == "fist_launcher") {
                        ge->fist_launcher_end_recency_ms[h] = 0.0f;
                    }
                    if (ge->callback) {
                        ge_event_t ev{};
                        ev.type = GE_EVENT_CANCEL;
                        ev.action = gdef.action;
                        ev.gesture_name = gdef.name.c_str();
                        ev.hand_index = h;
                        ge->callback(&ev, ge->user_data);
                    }
                } else if (release_indeterminate && indeterminate_expired) {
                    // Release has been unreadable for too long (occluded or
                    // low-confidence joint) — give up rather than stay
                    // Active forever.
                    rt.state = ge::GestureState::Idle;
                    rt.hold_time_ms = 0;
                    rt.has_prev_tracked = false;
                    rt.cooldown_remaining_ms
                        = static_cast<float>(gdef.cooldown_ms);
                    ge->disarmed[{h, gdef.name}] = 0.0f;
                    if (gdef.name == "pinch_select") {
                        ge->pinch_select_end_recency_ms[h] = 0.0f;
                    }
                    if (gdef.name == "fist_launcher") {
                        ge->fist_launcher_end_recency_ms[h] = 0.0f;
                    }
                    if (ge->callback) {
                        ge_event_t ev{};
                        ev.type = GE_EVENT_CANCEL;
                        ev.action = gdef.action;
                        ev.gesture_name = gdef.name.c_str();
                        ev.hand_index = h;
                        ge->callback(&ev, ge->user_data);
                    }
                } else if (release_met
                           || (gdef.max_active_ms > 0
                               && rt.active_time_ms
                                  >= static_cast<float>(gdef.max_active_ms))) {
                    // Deactivate — emit END (release condition or max duration)
                    rt.state = ge::GestureState::Idle;
                    rt.hold_time_ms = 0;
                    rt.has_prev_tracked = false;
                    rt.cooldown_remaining_ms
                        = static_cast<float>(gdef.cooldown_ms);
                    ge->disarmed[{h, gdef.name}] = 0.0f;
                    if (gdef.name == "pinch_select") {
                        ge->pinch_select_end_recency_ms[h] = 0.0f;
                    }
                    if (gdef.name == "fist_launcher") {
                        ge->fist_launcher_end_recency_ms[h] = 0.0f;
                    }
                    if (ge->callback) {
                        ge_event_t ev{};
                        ev.type = GE_EVENT_END;
                        ev.action = gdef.action;
                        ev.gesture_name = gdef.name.c_str();
                        ev.hand_index = h;
                        ge->callback(&ev, ge->user_data);
                    }
                } else {
                    // Still active — emit UPDATE with delta
                    hand_claimed[h] = true;
                    if (!gdef.coexist_with.empty())
                        hand_claimed_coexist[h] = gdef.coexist_with;
                    rt.active_time_ms += dt_ms;

                    ge_event_t ev{};
                    ev.type = GE_EVENT_UPDATE;
                    ev.action = gdef.action;
                    ev.gesture_name = gdef.name.c_str();
                    ev.hand_index = h;
                    ev.position[0] = feat.pinch_midpoint.x;
                    ev.position[1] = feat.pinch_midpoint.y;
                    ev.position[2] = feat.pinch_midpoint.z;

                    if (gdef.track.type == ge::TrackType::Scalar) {
                        float cur = ge::get_tracked_scalar(gdef.track, feat);
                        if (std::isnan(cur)) {
                            // Indeterminate this frame: report the last real
                            // value with zero delta instead of leaking NaN
                            // into consumers.
                            ev.value = rt.prev_tracked_value;
                        } else {
                            ev.value = cur;
                            if (rt.has_prev_tracked) {
                                ev.delta[0] = cur - rt.prev_tracked_value;
                            }
                            rt.prev_tracked_value = cur;
                            rt.has_prev_tracked = true;
                        }
                    } else if (gdef.track.type == ge::TrackType::Position3D) {
                        ge::Vec3 cur = ge::get_tracked_position(gdef.track, feat);
                        if (rt.has_prev_tracked) {
                            ev.delta[0] = cur.x - rt.prev_tracked_pos.x;
                            ev.delta[1] = cur.y - rt.prev_tracked_pos.y;
                            ev.delta[2] = cur.z - rt.prev_tracked_pos.z;
                        }
                        rt.prev_tracked_pos = cur;
                        rt.has_prev_tracked = true;
                    }

                    if (ge->callback) {
                        ge->callback(&ev, ge->user_data);
                    }
                }
                break;
            }  // case Active
            }  // switch
            // End of switch on rt.state.  Only loop again if we just
            // transitioned Idle → Pending — that's the one case where
            // running the Pending case on the same tick lets a
            // 0-debounce gesture fire BEGIN immediately instead of
            // ceding the next tick to a higher-priority overtaker.
            // For any other transition we'd risk firing END right
            // after BEGIN in the same tick, which the consumer would
            // see as a 0-duration gesture.
            if (entry_state == ge::GestureState::Idle
                && rt.state == ge::GestureState::Pending
                && gdef.min_hold_ms <= 0
                && gdef.motion_threshold_m <= 0.0f) {
                continue;  // re-enter the switch with rt.state == Pending
            }
            break;
            }  // for (iter ...)
        }
    }

    // Rearm: a disarmed name becomes fireable again only after no variant
    // of it has had its trigger held for kRearmReleaseMs straight (see the
    // `disarmed` field comment).  A real double-tap's inter-beat release
    // (>= 130 ms in every recorded clip) clears this comfortably.
    constexpr float kRearmReleaseMs = 100.0f;
    for (auto it = ge->disarmed.begin(); it != ge->disarmed.end();) {
        if (name_trigger_held.count(it->first) > 0) {
            it->second = 0.0f;
            ++it;
        } else {
            it->second += dt_ms;
            if (it->second >= kRearmReleaseMs) it = ge->disarmed.erase(it);
            else ++it;
        }
    }
}

float ge_get_feature(const ge_engine_t *ge, int hand_index, const char *feature_name) {
    if (!ge || hand_index < 0 || hand_index > 1 || !feature_name) return 0.0f;
    if (!ge->hand_present[hand_index]) return 0.0f;
    return ge::get_feature_by_name(ge->features[hand_index], std::string(feature_name));
}

float ge_hand_scale(const ge_hand_t *hand) {
    return hand ? ge::compute_hand_scale(*hand) : 0.0f;
}

ge_chirality_t ge_hand_chirality(const ge_hand_t *hand) {
    if (!hand) return GE_CHIRALITY_UNKNOWN;
    // Degeneracy guard, not a tuned threshold: 0.2 proximal-phalanx lengths
    // is ~6 mm on a 30 mm bone, which is the residual noise floor of the
    // 3-joint thumb average.  A real thumb clears it by 2-4x even on a flat
    // palm, and a fist (where the thumb wraps to the palmar side) far more.
    constexpr float kMinOffset = 0.20f;
    float d = ge::palm_thumb_signed_offset(*hand);
    if (d > kMinOffset)  return GE_CHIRALITY_RIGHT;
    if (d < -kMinOffset) return GE_CHIRALITY_LEFT;
    return GE_CHIRALITY_UNKNOWN;
}

bool ge_hand_is_left(const ge_hand_t *hand) {
    return ge_hand_chirality(hand) == GE_CHIRALITY_LEFT;
}

int ge_hand_input_slot(const ge_engine_t *ge, int hand_index) {
    // The event's hand_index is an ENGINE-hand identity, which the continuity
    // remap in ge_update() may have bound to EITHER input slot (see the
    // "Hand-identity remap" block).  Report which hands[] slot currently feeds
    // this engine hand so a caller can recover the original joints.  Only
    // meaningful while the engine hand is present — engine_hand_last_bound_slot
    // is sticky across a brief dropout, but the joints in that slot are only
    // this hand's while it's actually bound this tick.
    if (!ge || hand_index < 0 || hand_index > 1) return -1;
    if (!ge->hand_present[hand_index]) return -1;
    return ge->engine_hand_last_bound_slot[hand_index];
}

float ge_pending_progress(const ge_engine_t *ge, const char *gesture_name,
                          int hand_index) {
    if (!ge || !gesture_name || hand_index < 0 || hand_index > 1) return 0.0f;
    for (size_t gi = 0; gi < ge->gestures.size(); ++gi) {
        if (ge->gestures[gi].name != gesture_name) continue;
        const ge::GestureRuntime &rt = ge->runtime[gi * 2 + hand_index];
        if (rt.state != ge::GestureState::Pending) return 0.0f;
        const int hold = ge->gestures[gi].min_hold_ms;
        if (hold <= 0) return 1.0f;
        float p = rt.hold_time_ms / static_cast<float>(hold);
        if (p < 0.0f) p = 0.0f;
        if (p > 1.0f) p = 1.0f;
        return p;
    }
    return 0.0f;
}
