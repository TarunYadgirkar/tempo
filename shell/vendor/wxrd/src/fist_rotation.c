/* fist_rotation.c — see fist_rotation.h for the contract and rationale.
 *
 * SPDX-License-Identifier: MIT
 */

#include "fist_rotation.h"
#include <math.h>

/* Below this length a vector is treated as degenerate (joints coincident
 * or the spoke nearly parallel to the axis). */
#define FR_EPS 1e-6f

/* Smoothing / robustness constants.  These are signal-conditioning
 * parameters, not gesture thresholds: they assume ~30 Hz hand updates and
 * target ~100 ms of scrub latency.
 *   - EMA_ALPHA 0.25  -> first-order time constant ~3.5 frames (~115 ms).
 *   - OUTLIER_COS 0.34 (cos 70 deg): no wrist rolls 70 deg in one 33 ms
 *     frame, so a spoke that far from the smoothed estimate is noise.
 *   - OUTLIER_MAX_RUN 6 (~200 ms): if that many consecutive frames all
 *     disagree the same way it is a genuine re-acquisition, so accept it. */
#define FR_EMA_ALPHA      0.25f
#define FR_OUTLIER_COS    0.34f
#define FR_OUTLIER_MAX_RUN 6

/* Project vec onto the plane perpendicular to axis (assumed unit length),
 * store in out.  Returns the length of the projected vector. */
static float
project_onto_plane (const float vec[3], const float axis[3], float out[3])
{
  float d = vec[0] * axis[0] + vec[1] * axis[1] + vec[2] * axis[2];
  out[0] = vec[0] - d * axis[0];
  out[1] = vec[1] - d * axis[1];
  out[2] = vec[2] - d * axis[2];
  return sqrtf (out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
}

/* Across-palm spoke (index - pinky) projected into the plane perpendicular
 * to axis, normalised into out.  Returns the pre-normalisation length (0
 * if degenerate, in which case out is left untouched). */
static float
project_unit_spoke (const float index_mcp[3], const float pinky_mcp[3],
                    const float axis[3], float out[3])
{
  float spoke[3] = {
    index_mcp[0] - pinky_mcp[0],
    index_mcp[1] - pinky_mcp[1],
    index_mcp[2] - pinky_mcp[2],
  };
  float plen = project_onto_plane (spoke, axis, out);
  if (plen < FR_EPS)
    return 0.0f;
  out[0] /= plen;
  out[1] /= plen;
  out[2] /= plen;
  return plen;
}

/* Pick an arbitrary unit vector perpendicular to `axis`.  Fallback for a
 * degenerate snapshot spoke so the neutral reference stays well-formed. */
static void
any_perpendicular (const float axis[3], float out[3])
{
  float ax = fabsf (axis[0]), ay = fabsf (axis[1]), az = fabsf (axis[2]);
  float seed[3] = { 0.0f, 0.0f, 0.0f };
  if (ax <= ay && ax <= az)
    seed[0] = 1.0f;
  else if (ay <= az)
    seed[1] = 1.0f;
  else
    seed[2] = 1.0f;

  float p[3];
  float plen = project_onto_plane (seed, axis, p);
  if (plen < FR_EPS) {
    out[0] = 1.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
    return;
  }
  out[0] = p[0] / plen;
  out[1] = p[1] / plen;
  out[2] = p[2] / plen;
}

/* Signed angle (radians) from `neutral` to unit direction `dir`, measured
 * about `axis`, with the left-hand mirror applied. */
static float
angle_from_dir (const float dir[3], int is_left,
                const float axis[3], const float neutral[3])
{
  float dot = neutral[0] * dir[0] + neutral[1] * dir[1] + neutral[2] * dir[2];
  float cx = neutral[1] * dir[2] - neutral[2] * dir[1];
  float cy = neutral[2] * dir[0] - neutral[0] * dir[2];
  float cz = neutral[0] * dir[1] - neutral[1] * dir[0];
  float cross_along = cx * axis[0] + cy * axis[1] + cz * axis[2];

  float angle = atan2f (cross_along, dot);
  if (is_left)
    angle = -angle;
  return angle;
}

void
fist_rotation_snapshot (const float wrist[3],
                        const float index_mcp[3],
                        const float middle_mcp[3],
                        const float pinky_mcp[3],
                        float out_axis[3],
                        float out_neutral[3])
{
  /* Forearm / roll axis: wrist -> middle knuckle.  Roughly collinear with
   * the forearm, so it stays put while the wrist twists about it. */
  float fwd[3] = {
    middle_mcp[0] - wrist[0],
    middle_mcp[1] - wrist[1],
    middle_mcp[2] - wrist[2],
  };
  float flen = sqrtf (fwd[0] * fwd[0] + fwd[1] * fwd[1] + fwd[2] * fwd[2]);
  if (flen < FR_EPS) {
    out_axis[0] = 0.0f;
    out_axis[1] = 0.0f;
    out_axis[2] = -1.0f;
  } else {
    out_axis[0] = fwd[0] / flen;
    out_axis[1] = fwd[1] / flen;
    out_axis[2] = fwd[2] / flen;
  }

  if (project_unit_spoke (index_mcp, pinky_mcp, out_axis, out_neutral) == 0.0f)
    any_perpendicular (out_axis, out_neutral);
}

float
fist_rotation_angle (const float index_mcp[3],
                     const float pinky_mcp[3],
                     int is_left,
                     const float axis[3],
                     const float neutral[3])
{
  float dir[3];
  if (project_unit_spoke (index_mcp, pinky_mcp, axis, dir) == 0.0f)
    return 0.0f;
  return angle_from_dir (dir, is_left, axis, neutral);
}

/* ---------------------------------------------------------------------- */

void
fist_tracker_reset (fist_tracker_t *t)
{
  t->initialized = false;
  t->axis[0] = 0.0f;     t->axis[1] = 0.0f;     t->axis[2] = -1.0f;
  t->neutral[0] = 1.0f;  t->neutral[1] = 0.0f;  t->neutral[2] = 0.0f;
  t->smoothed[0] = 1.0f; t->smoothed[1] = 0.0f; t->smoothed[2] = 0.0f;
  t->outlier_run = 0;
}

float
fist_tracker_current_angle (const fist_tracker_t *t, int is_left)
{
  if (!t->initialized)
    return 0.0f;
  return angle_from_dir (t->smoothed, is_left, t->axis, t->neutral);
}

float
fist_tracker_update (fist_tracker_t *t,
                     const float wrist[3], const float index_mcp[3],
                     const float middle_mcp[3], const float pinky_mcp[3],
                     int is_left)
{
  /* Deferred neutral capture: wait for a frame with a non-degenerate spoke
   * so a momentary collinear/coincident knuckle reading at BEGIN doesn't
   * fix a wrong zero reference for the whole gesture. */
  if (!t->initialized) {
    float axis[3], neutral[3];
    fist_rotation_snapshot (wrist, index_mcp, middle_mcp, pinky_mcp,
                            axis, neutral);
    float probe[3];
    if (project_unit_spoke (index_mcp, pinky_mcp, axis, probe) == 0.0f)
      return 0.0f; /* degenerate spoke — keep waiting */
    for (int i = 0; i < 3; i++) {
      t->axis[i] = axis[i];
      t->neutral[i] = neutral[i];
      t->smoothed[i] = neutral[i];
    }
    t->initialized = true;
    t->outlier_run = 0;
    return 0.0f;
  }

  /* Current spoke direction in the (fixed) snapshot plane. */
  float cur[3];
  if (project_unit_spoke (index_mcp, pinky_mcp, t->axis, cur) == 0.0f)
    return angle_from_dir (t->smoothed, is_left, t->axis, t->neutral);

  /* Outlier rejection: a single frame can't roll 70 deg.  Hold the
   * smoothed estimate unless a run of frames insists on the new value. */
  float cosang = cur[0] * t->smoothed[0] + cur[1] * t->smoothed[1]
               + cur[2] * t->smoothed[2];
  if (cosang < FR_OUTLIER_COS && t->outlier_run < FR_OUTLIER_MAX_RUN) {
    t->outlier_run++;
    return angle_from_dir (t->smoothed, is_left, t->axis, t->neutral);
  }
  t->outlier_run = 0;

  /* EMA toward the current direction, then renormalise (keeps it a unit
   * vector and sidesteps any atan2 wraparound). */
  float blended[3] = {
    t->smoothed[0] + FR_EMA_ALPHA * (cur[0] - t->smoothed[0]),
    t->smoothed[1] + FR_EMA_ALPHA * (cur[1] - t->smoothed[1]),
    t->smoothed[2] + FR_EMA_ALPHA * (cur[2] - t->smoothed[2]),
  };
  float blen = sqrtf (blended[0] * blended[0] + blended[1] * blended[1]
                      + blended[2] * blended[2]);
  if (blen >= FR_EPS) {
    t->smoothed[0] = blended[0] / blen;
    t->smoothed[1] = blended[1] / blen;
    t->smoothed[2] = blended[2] / blen;
  }
  return angle_from_dir (t->smoothed, is_left, t->axis, t->neutral);
}
