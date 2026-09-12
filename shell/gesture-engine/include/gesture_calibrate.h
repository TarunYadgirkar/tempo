// gesture_calibrate.h — Per-user gesture-engine calibration.
//
// Idea: a one-size threshold can't simultaneously fire reliably for User A
// and avoid false-firing for User B.  The 30-second calibration walks the
// user through three phases (idle hand, active pinches, active grabs),
// observes the noise floor and the actively-met value of each relevant
// feature, and emits a tuned gestures.toml that lands the trigger/release
// thresholds at percentages of the gap between them.
//
// Threshold formula (works for both `<` and `>` gestures):
//   trigger = active + 0.30 * (noise - active)
//   release = active + 0.70 * (noise - active)
// — both lie between the active value and the noise floor.  The release
// is further toward the noise floor than the trigger so hysteresis is in
// the correct direction regardless of comparison operator.
//
// The library is C-callable so the same code can drive the CLI tool, the
// in-shell calibration UI, and the unit test.

#pragma once

#include "gesture_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GE_CALIB_PHASE_IDLE  = 0,   // hands relaxed, not gesturing
    GE_CALIB_PHASE_PINCH = 1,   // user repeatedly pinches index+thumb
    GE_CALIB_PHASE_GRAB  = 2,   // user repeatedly grabs (thumb+index+middle)
    GE_CALIB_PHASE_COUNT
} ge_calib_phase_t;

typedef struct ge_calib_t ge_calib_t;

// Create a fresh calibration state.  Returns NULL on OOM.
ge_calib_t *gc_create(void);

// Destroy and free.
void gc_destroy(ge_calib_t *gc);

// Switch the active phase.  Subsequent gc_observe() calls accumulate
// statistics under this phase.
void gc_set_phase(ge_calib_t *gc, ge_calib_phase_t phase);

// Observe one hand frame.  Confidence-filtered features are folded into
// the running statistics for the current phase.
void gc_observe(ge_calib_t *gc, const ge_hand_t *hand);

// Resolve a (gesture_name, kind, feature) triple to its calibrated
// threshold value.  Returns false if the calibration didn't observe
// enough data for that feature in either the idle or relevant active
// phase.  On success, *out_value contains the threshold; the caller
// applies the formula direction implicitly because both trigger and
// release sit between active and noise.
//
// kind must be "trigger" or "release".
bool gc_resolve_threshold(const ge_calib_t *gc,
                          const char *gesture_name,
                          const char *kind,
                          const char *feature,
                          float *out_value);

// Emit a gestures.toml containing every threshold the calibration
// successfully resolved.  Thresholds that didn't get enough data are
// omitted (the loader will fall back to compiled defaults for those).
//
// Pass `f == NULL` to write to stdout.
void gc_emit_toml(const ge_calib_t *gc, FILE *f);

#ifdef __cplusplus
}  // extern "C"
#endif
