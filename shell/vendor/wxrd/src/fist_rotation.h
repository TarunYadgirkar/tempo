/* fist_rotation.h — Geometry + noise-robust tracking for the radial-
 * launcher fist-twist (wrist pronation/supination) measurement.
 *
 * Extracted from hand_input.c so the math can be unit tested without the
 * compositor (see tests/test_fist_rotation.c) — same pattern as
 * launcher_shm_writer.{c,h}.
 *
 * THE MEASUREMENT
 * ---------------
 * The launcher is scrubbed by ROLLING the fist about the forearm's long
 * axis, NOT by yawing the forearm left/right.  So:
 *
 *   - the forearm axis (wrist -> middle knuckle) is the stable ROLL AXIS:
 *     the wrist twists *about* it, so it barely moves during the gesture;
 *   - the across-palm "spoke" (index knuckle -> pinky knuckle) is
 *     perpendicular to that axis, so it sweeps a full angle as the wrist
 *     rolls — that swept angle is the measurement.
 *
 * WHY A STATEFUL TRACKER (and not just the raw per-frame angle)
 * -------------------------------------------------------------
 * The spoke differences the index and pinky knuckles, which are among the
 * least-reliable joints inside a closed fist (curled fingers + occlusion;
 * observed tracking confidence as low as 0.15).  A single bad frame flips
 * the spoke tens of degrees, which — with no smoothing — flings the
 * launcher highlight to a random segment and back on the next good frame
 * (user report 2026-06-28: "it often just jumps to a random item and then
 * jumps back ... can't select with any precision").  fist_tracker_t adds:
 *
 *   - deferred neutral capture: the zero reference is taken from the first
 *     frame with a non-degenerate spoke, so a frame where the knuckles are
 *     momentarily coincident/collinear can't poison the gesture;
 *   - outlier rejection: a frame whose spoke jumps more than ~70 deg from
 *     the smoothed estimate is dropped — no wrist rolls that fast in one
 *     frame — unless several consecutive frames agree (a real re-acquire);
 *   - exponential smoothing of the spoke DIRECTION (a unit vector, so
 *     there is no atan2 wraparound discontinuity to average across).
 *
 * Robustness here is purely GEOMETRIC (outlier distance + smoothing); it
 * deliberately does NOT gate on the tracker's per-joint confidence value,
 * because that confidence is itself unreliable on this hardware and a
 * confidence threshold tight enough to be useful risks never clearing —
 * which would silently freeze rotation at 0, the very symptom this fixes.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef FIST_ROTATION_H
#define FIST_ROTATION_H

#include <stdbool.h>

/* ----------------------------------------------------------------------
 * Low-level pure primitives (stateless; unit tested directly)
 * -------------------------------------------------------------------- */

/* Capture the neutral (zero-angle) reference at gesture BEGIN.
 *   out_axis    — forearm/roll axis (unit vector).
 *   out_neutral — across-palm spoke projected into the plane perpendicular
 *                 to out_axis, normalised. */
void fist_rotation_snapshot (const float wrist[3],
                             const float index_mcp[3],
                             const float middle_mcp[3],
                             const float pinky_mcp[3],
                             float out_axis[3],
                             float out_neutral[3]);

/* Signed roll angle (radians) of the current index/pinky pose relative to
 * the snapshotted neutral, about the snapshot axis.  is_left mirrors the
 * sign so the same physical supination reads the same on both hands.
 * Returns 0 when the spoke is degenerate (nearly parallel to the axis). */
float fist_rotation_angle (const float index_mcp[3],
                           const float pinky_mcp[3],
                           int is_left,
                           const float axis[3],
                           const float neutral[3]);

/* ----------------------------------------------------------------------
 * Stateful noise-robust tracker (one per active launcher gesture)
 * -------------------------------------------------------------------- */

typedef struct {
  bool  initialized;  /* neutral/axis captured from a usable frame yet?  */
  float axis[3];      /* forearm roll axis, captured once                */
  float neutral[3];   /* zero-angle spoke direction, captured once       */
  float smoothed[3];  /* EMA-smoothed current spoke direction (unit)     */
  int   outlier_run;  /* consecutive rejected frames                     */
} fist_tracker_t;

/* Clear the tracker — call at gesture BEGIN. */
void fist_tracker_reset (fist_tracker_t *t);

/* Feed one frame; returns the current smoothed roll angle (radians).
 *   is_left — non-zero for the left hand (mirrors the sign).
 * Returns 0 until a non-degenerate neutral has been captured. */
float fist_tracker_update (fist_tracker_t *t,
                           const float wrist[3], const float index_mcp[3],
                           const float middle_mcp[3], const float pinky_mcp[3],
                           int is_left);

/* Current smoothed angle WITHOUT consuming a frame (used on gesture END,
 * where the hand is already opening and a fresh sample is meaningless). */
float fist_tracker_current_angle (const fist_tracker_t *t, int is_left);

#endif /* FIST_ROTATION_H */
