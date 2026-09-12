// gesture_engine.h — Public C API for the gesture recognition engine.
//
// Consumes hand joint data from bridge-receiver (sb_hand_t) and produces
// gesture events via a callback. Gestures are defined as configurable
// finite state machines with hysteresis on all thresholds.
//
// Thread safety: ge_update() must be called from a single thread.
// The callback fires synchronously inside ge_update().

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Hand joint data (matches bridge-receiver sb_hand_t layout)
// ---------------------------------------------------------------------------

#define GE_JOINT_COUNT 21

typedef struct ge_hand_t {
    float joints[GE_JOINT_COUNT][5];  // [joint][x, y, z, confidence, reserved]
    bool  present;                     // true if hand data is fresh this frame
} ge_hand_t;

// Joint indices (must match SB_JOINT_* in spatial_bridge.h)
enum ge_joint {
    GE_JOINT_WRIST = 0,
    GE_JOINT_THUMB_CMC,  GE_JOINT_THUMB_MCP,  GE_JOINT_THUMB_IP,   GE_JOINT_THUMB_TIP,
    GE_JOINT_INDEX_MCP,  GE_JOINT_INDEX_PIP,  GE_JOINT_INDEX_DIP,  GE_JOINT_INDEX_TIP,
    GE_JOINT_MIDDLE_MCP, GE_JOINT_MIDDLE_PIP, GE_JOINT_MIDDLE_DIP, GE_JOINT_MIDDLE_TIP,
    GE_JOINT_RING_MCP,   GE_JOINT_RING_PIP,   GE_JOINT_RING_DIP,   GE_JOINT_RING_TIP,
    GE_JOINT_PINKY_MCP,  GE_JOINT_PINKY_PIP,  GE_JOINT_PINKY_DIP,  GE_JOINT_PINKY_TIP,
};

// ---------------------------------------------------------------------------
// Gesture events
// ---------------------------------------------------------------------------

typedef enum {
    GE_EVENT_BEGIN,    // gesture just activated
    GE_EVENT_UPDATE,   // gesture is ongoing, delta values updated
    GE_EVENT_END,      // gesture deactivated normally
    GE_EVENT_CANCEL,   // gesture cancelled (e.g., hand lost)
} ge_event_type_t;

typedef enum {
    GE_ACTION_POINTER_CLICK,   // maps to BTN_LEFT press/release
    GE_ACTION_SCROLL,          // vertical scroll (delta in event.delta[1])
    GE_ACTION_WINDOW_MOVE,     // 3D position delta in event.delta[0..2]
    GE_ACTION_WINDOW_RESIZE,   // scale delta in event.delta[0]
    GE_ACTION_TOGGLE_LAUNCHER, // open/close app launcher
    GE_ACTION_POINTER_RIGHT_CLICK, // maps to BTN_RIGHT press/release
    GE_ACTION_FINE_SLIDER,     // fine-grained scalar delta in event.delta[0]
    GE_ACTION_WINDOW_CLOSE,    // close focused window
    GE_ACTION_TOGGLE_KEYBOARD, // toggle the on-screen keyboard (wvkbd v0)
    GE_ACTION_KEYBOARD_ANCHOR, // bind the typing plane to the surface under the palm
    GE_ACTION_CUSTOM,          // user-defined, check gesture name
} ge_action_t;

typedef struct ge_event_t {
    ge_event_type_t type;
    ge_action_t     action;
    const char     *gesture_name;  // pointer valid until ge_destroy() or config reload
    int             hand_index;    // engine-hand identity (0 or 1) — NOT the
                                   // hands[] input slot and NOT handedness.
                                   // The continuity remap in ge_update() may
                                   // bind this identity to either input slot;
                                   // use ge_hand_input_slot() to recover the
                                   // slot whose joints fired this event.
    float           delta[3];      // per-frame delta for dynamic gestures (x, y, z)
    float           value;         // current tracked value (for slider-type gestures)
    float           position[3];   // 3D position of the interaction point (e.g., pinch midpoint)
} ge_event_t;

// Callback fired from inside ge_update(). Do not call ge_* functions from within.
typedef void (*ge_event_callback_t)(const ge_event_t *event, void *user_data);

// ---------------------------------------------------------------------------
// Engine lifecycle
// ---------------------------------------------------------------------------

typedef struct ge_engine_impl_t ge_engine_t;

// Create the gesture engine with built-in default gestures.
ge_engine_t *ge_create(void);

// Destroy the engine and free all resources.
void ge_destroy(ge_engine_t *ge);

// Load gesture definitions from a TOML config file.
//
// Pass NULL to load from $XDG_CONFIG_HOME/spatial-os/gestures.toml (or
// $HOME/.config/spatial-os/gestures.toml).  Missing file is not an error —
// the function returns false and leaves compiled defaults in place.
//
// Returns true if at least one override was applied; false otherwise.
// Per-line parse errors are logged to stderr but do not abort loading.
bool ge_load_config(ge_engine_t *ge, const char *toml_path);

// Get a gesture's min_hold_ms (returns -1 if no gesture by that name).
// Useful for verification in tests and tooling.
int ge_gesture_min_hold_ms(const ge_engine_t *ge, const char *gesture_name);

// Get the threshold of a specific trigger or release condition.
// kind must be "trigger" or "release".  Returns NaN if the gesture, kind,
// or feature isn't found.
float ge_gesture_threshold(const ge_engine_t *ge, const char *gesture_name,
                           const char *kind, const char *feature);

// Get a gesture's cooldown_ms (returns -1 if no gesture by that name).
int ge_gesture_cooldown_ms(const ge_engine_t *ge, const char *gesture_name);

// Get a gesture's max_active_ms / indeterminate_release_ms (-1 if unknown).
int ge_gesture_max_active_ms(const ge_engine_t *ge, const char *gesture_name);
int ge_gesture_indeterminate_release_ms(const ge_engine_t *ge,
                                        const char *gesture_name);

// Every per-name accessor above resolves `gesture_name` against the first
// compiled definition with that name; pass "name.variant" to address one
// same-name variant (variants are listed by ge_gesture_variant_at).

// Enumerate compiled gesture definitions in priority-descending order.
// Returned strings are valid until ge_destroy().  ge_gesture_variant_at
// returns "" for a name with a single definition.
int         ge_gesture_count(const ge_engine_t *ge);
const char *ge_gesture_name_at(const ge_engine_t *ge, int index);
const char *ge_gesture_variant_at(const ge_engine_t *ge, int index);

// Enumerate a definition's trigger / release conditions.  kind is
// "trigger" or "release"; op comes back as less / greater / less_eq /
// greater_eq.  Returns false when index or cond_index is out of range.
int  ge_gesture_condition_count(const ge_engine_t *ge, int index,
                                const char *kind);
bool ge_gesture_condition_at(const ge_engine_t *ge, int index,
                             const char *kind, int cond_index,
                             const char **feature, const char **op,
                             float *threshold);

// Get a [filter]-section parameter (one_euro_enabled,
// one_euro_min_cutoff_hz, one_euro_beta, one_euro_d_cutoff_hz,
// min_joint_confidence).  Returns NaN for an unknown key.
float ge_filter_param(const ge_engine_t *ge, const char *key);

// Write the compiled-default gesture configuration as TOML to `f`.
// The output is the canonical sample shipped under sample-config/, suitable
// for round-tripping through ge_load_config.  Used by
// tools/print_default_config to regenerate the sample mechanically when
// defaults change.  Pass `f == NULL` to write to stdout.
void ge_print_default_config(FILE *f);

// Same shape as ge_print_default_config but for `ge`'s CURRENT values
// (compiled defaults plus any overrides applied by ge_load_config).
void ge_print_config(const ge_engine_t *ge, FILE *f);

// Set the event callback. Only one callback at a time; replaces any previous.
void ge_set_callback(ge_engine_t *ge, ge_event_callback_t cb, void *user_data);

// ---------------------------------------------------------------------------
// Per-frame update
// ---------------------------------------------------------------------------

// Call once per frame with the latest hand data for both hands.
// hands[0] = left, hands[1] = right. Set hands[i].present = false if no data.
// dt_seconds = time since last call (for velocity computation and debounce).
// Gesture events are fired synchronously via the callback during this call.
void ge_update(ge_engine_t *ge, const ge_hand_t hands[2], float dt_seconds);

// ---------------------------------------------------------------------------
// Computed features (readable after ge_update for debugging / UI)
// ---------------------------------------------------------------------------

// Get the most recently computed value of a named feature for a given hand.
// Returns 0 if the feature name is unknown or the hand is not present.
float ge_get_feature(const ge_engine_t *ge, int hand_index, const char *feature_name);

// Robust per-hand metric scale (m): mean of the index/middle/ring proximal
// phalanx. Stateless — computed directly from joints. Used by the per-user
// hand-length calibration flow and tools to read the same scale the engine
// normalizes by. Returns 0 if `hand` is NULL or joints are degenerate.
float ge_hand_scale(const ge_hand_t *hand);

// ---------------------------------------------------------------------------
// Handedness (chirality)
// ---------------------------------------------------------------------------

typedef enum {
    GE_CHIRALITY_UNKNOWN = 0,  // joints too flat / degenerate to decide
    GE_CHIRALITY_LEFT,
    GE_CHIRALITY_RIGHT,
} ge_chirality_t;

// Which physical hand these joints belong to, derived from GEOMETRY —
// stateless, and independent of the hands[] slot the tracker put them in.
//
// Do NOT infer handedness from a slot index.  The iPhone's hand_index is a
// sticky first-seen slot, not a chirality: a right hand that lands in slot 0
// once stays in slot 0 for the whole session, so `slot == 0` misreads it as
// left every frame after.  Anything whose sign depends on handedness (the
// fist-roll scrub mirror in fist_rotation.c's angle_from_dir, for one) must
// call this instead.
//
// Signed volume of the tetrahedron wrist / INDEX_MCP / PINKY_MCP / thumb:
// the palm-plane normal (INDEX_MCP-WRIST) x (PINKY_MCP-WRIST) points palmar
// on a right hand and dorsal on a left one, and the thumb is anatomically
// always palmar.  Returns GE_CHIRALITY_UNKNOWN when the thumb reads too
// close to the palm plane to call (a caller with a continuous signal should
// latch the last decided value rather than flip).
ge_chirality_t ge_hand_chirality(const ge_hand_t *hand);

// Convenience wrapper for callers that need a plain bool (e.g. the
// `is_left` mirror argument of fist_tracker_update).  UNKNOWN reads as
// false — latch ge_hand_chirality yourself if a stale value is safer than
// a wrong one.
bool ge_hand_is_left(const ge_hand_t *hand);

// Recover which input slot (index into the hands[] array passed to ge_update)
// currently feeds engine hand `hand_index`.
//
// The engine remaps the two tracker input slots onto two stable engine-hand
// identities by position continuity (so per-hand FSM state survives the iPhone
// Vision tracker swapping or dropping a slot).  Because of that remap, the
// `hand_index` on a ge_event_t is an ENGINE-hand id, which is NOT necessarily
// the hands[] slot the caller filled: a hand present only in hands[1] binds to
// engine hand 0, so its events carry hand_index == 0.
//
// A caller that needs the ORIGINAL per-joint data for the hand that fired an
// event (e.g. the compositor recomputing fist roll from raw joints) must read
// hands[ge_hand_input_slot(ge, event->hand_index)], NOT hands[event->hand_index].
//
// Returns the bound slot (0 or 1), or -1 if `hand_index` is out of range or
// that engine hand is not currently present.
int ge_hand_input_slot(const ge_engine_t *ge, int hand_index);

// Progress through a gesture's hold window, [0, 1].  Returns:
//   0.0   the named gesture is Idle, Active, or unknown.
//   x>0   the gesture is in Pending state on `hand_index`, x = hold_time / min_hold.
// Used by UI to show a hold-progress indicator (e.g., a ring filling
// above the palm for keyboard_anchor) so the user knows the system
// has seen the gesture and is waiting for the hold to complete.
float ge_pending_progress(const ge_engine_t *ge, const char *gesture_name,
                          int hand_index);

#ifdef __cplusplus
}  // extern "C"
#endif
