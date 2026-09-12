// state_machine.h — Gesture state machine definitions.
// Internal header; not part of the public API.

#pragma once

#include "ge_features.h"
#include "gesture_engine.h"
#include <string>
#include <vector>

namespace ge {

// ---------------------------------------------------------------------------
// Gesture condition: a threshold test on a named feature
// ---------------------------------------------------------------------------

enum class CompareOp { Less, Greater, LessEq, GreaterEq };

struct Condition {
    std::string feature;
    CompareOp   op;
    float       threshold;
};

bool evaluate_condition(const Condition &c, const HandFeatures &f);

// ---------------------------------------------------------------------------
// Tracked feature: what to track for delta computation in dynamic gestures
// ---------------------------------------------------------------------------

enum class TrackType {
    None,            // static gesture (no delta tracking)
    Scalar,          // track a single named feature value
    Position3D,      // track a 3D position (palm_center or pinch_midpoint)
};

struct TrackConfig {
    TrackType   type = TrackType::None;
    std::string feature_name;   // for Scalar: which feature to track
                                // for Position3D: "palm_center" or "pinch_midpoint"
};

// ---------------------------------------------------------------------------
// Gesture definition
// ---------------------------------------------------------------------------

struct GestureDef {
    std::string             name;
    // Distinguishes same-name variants in TOML ([name.variant]) and in
    // the introspection accessors ("name.variant").  Empty when the name
    // has a single compiled definition.
    std::string             variant;
    std::vector<Condition>  trigger_conditions;
    std::vector<Condition>  release_conditions;
    int                     min_hold_ms = 50;
    // Minimum gap, in milliseconds, between this gesture's END/CANCEL
    // and the next allowed BEGIN on the same hand.  Stops flap-firing
    // when tracking jitter bounces a feature in and out of its trigger
    // band — the original release-hysteresis alone wasn't enough at the
    // ~30 Hz iPhone Vision hand-tracking cadence (a 33 ms tick can land
    // either side of a 50 mm threshold from frame to frame).  Set 0 to
    // disable (the historic behaviour).
    int                     cooldown_ms = 0;
    // Motion-gated BEGIN: if > 0 AND track.type == Position3D, the
    // Pending→Active transition additionally requires that the tracked
    // position has moved more than this many meters since the gesture
    // first entered Pending.  Used to differentiate a *held* pinch
    // (pinch_select) from a *moving* pinch (scroll) by trajectory
    // rather than by hold duration — the user's scroll motion fires
    // BEGIN the instant the pinch starts moving rather than after some
    // arbitrary 250 ms hold window.
    float                   motion_threshold_m = 0.0f;
    // Optional scalar motion gate independent of `track`.  BEGIN
    // additionally requires |scalar_gate_feature(now) -
    // scalar_gate_feature(Pending entry)| > scalar_gate_threshold.
    // Used by scroll which needs to distinguish a real slide
    // (projection changes 0.4 → 1.0) from a static hover (projection
    // stays at 1.0 for 500 ms before the user actually pinches —
    // see clip f1a911).  Position3D-based motion gates aren't
    // discriminating enough because the user's wrist drifts ~20 mm
    // even during a "static" hover.
    std::string             scalar_gate_feature;
    float                   scalar_gate_threshold = 0.0f;
    ge_action_t             action = GE_ACTION_CUSTOM;
    int                     priority = 0;
    TrackConfig             track;
    int                     hand = -1;  // -1 = either hand, 0 = left only, 1 = right only
    // If > 0, the gesture automatically releases (END) after being
    // Active for this many ms, even if release conditions aren't met.
    // Used by click-type gestures that shouldn't hold indefinitely.
    int                     max_active_ms = 0;
    // If the trigger (Pending) or release (Active) evaluation stays
    // indeterminate — a NaN feature from a zero-frame artifact or a
    // low-confidence joint — for longer than this, the gesture gives up:
    // a Pending hold resets, an Active gesture CANCELs, and a disarmed
    // name counts as released for rearm.  Without it an occluded joint
    // freezes a gesture Active forever (release reads NaN, never true).
    int                     indeterminate_release_ms = 300;
    // If non-empty, this gesture does NOT cancel the named gesture when
    // it claims the hand.  Both gestures stay Active simultaneously —
    // the coexisted gesture emits UPDATE/END as if no higher-priority
    // gesture claimed the hand.  Used by scroll to coexist with
    // pinch_select: scroll tracks projection delta while pinch_select
    // still emits its own END on physical release (ti > threshold).
    std::string             coexist_with;
};

// ---------------------------------------------------------------------------
// Per-gesture per-hand runtime state
// ---------------------------------------------------------------------------

enum class GestureState { Idle, Pending, Active };

struct GestureRuntime {
    GestureState state = GestureState::Idle;
    float        hold_time_ms = 0;      // how long trigger conditions have been met
    float        active_time_ms = 0;    // how long gesture has been Active (for max_active_ms)
    float        cooldown_remaining_ms = 0;  // decrements each tick; blocks Pending until 0
    float        indeterminate_ms = 0;  // consecutive ms the relevant evaluation read NaN
    float        prev_tracked_value = 0; // for scalar delta
    Vec3         prev_tracked_pos;       // for 3D delta
    bool         has_prev_tracked = false;
    Vec3         pending_start_pos;      // tracked position at Idle→Pending entry, for motion-gated BEGIN
    bool         has_pending_start_pos = false;
    // For scalar_gate_feature (independent of `track`).
    float        pending_gate_start_scalar = 0.0f;
    bool         has_pending_gate_start_scalar = false;
};

}  // namespace ge
