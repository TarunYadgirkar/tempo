/* kbd_tap_traj.h — the canonical synthetic fingertip-tap trajectory.
 *
 * A "tap" descends a fingertip toward the keyboard plane (z high → low) over
 * KBD_TAP_DOWN_FRAMES, then lifts it back over the remainder of
 * KBD_TAP_TOTAL_FRAMES.  Both the decoder's unit test (test_decoder.c) and the
 * headless e2e injector (tests/integration/kbd_tap_inject.c) drive the decoder
 * with EXACTLY this trajectory, so it lives here as a single source of truth.
 * The trajectory must satisfy the surface decoder's tap gate (decoder.c): a
 * plunge of at least TAP_PLUNGE_M peak-to-valley that bottoms within
 * TAP_NEAR_Z_M of the plane, then lifts TAP_PLUNGE_M back up.  Here the plunge
 * amplitude is KBD_TAP_Z_HIGH − KBD_TAP_Z_LOW (45 mm ≫ 18 mm) and Z_LOW ≤ 0 sits
 * at the plane, so it passes comfortably; keep those margins if the gate tunes.
 *
 * Park position: a fingertip "at rest" sits at (KBD_TAP_PARK_X/Y/Z) in
 * plane-local metres — provably off every key (the layout spans x∈[0,0.30],
 * y∈[0,0.12], see keyboard_geom.h) AND 20 cm above the plane (z ≫ 13·σ_z), so a
 * parked finger contributes zero typing intent.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SPATIAL_KEYBOARD_DECODER_KBD_TAP_TRAJ_H
#define SPATIAL_KEYBOARD_DECODER_KBD_TAP_TRAJ_H

#define KBD_TAP_TOTAL_FRAMES 24
#define KBD_TAP_DOWN_FRAMES  10
#define KBD_TAP_Z_HIGH       0.04f    /* start/end height above plane (m) */
#define KBD_TAP_Z_LOW        (-0.005f) /* deepest penetration past plane (m) */

#define KBD_TAP_PARK_X 0.5f
#define KBD_TAP_PARK_Y 0.5f
#define KBD_TAP_PARK_Z 0.2f

/* z height (plane-local metres) of the tapping finger at frame `frame`
 * (0 ≤ frame < KBD_TAP_TOTAL_FRAMES): linear descent then linear lift. */
static inline float
kbd_tap_traj_z (int frame)
{
    if (frame < KBD_TAP_DOWN_FRAMES) {
        float t = (float) frame / (float) (KBD_TAP_DOWN_FRAMES - 1);
        return KBD_TAP_Z_HIGH * (1.0f - t) + KBD_TAP_Z_LOW * t;
    }
    float t = (float) (frame - KBD_TAP_DOWN_FRAMES)
              / (float) (KBD_TAP_TOTAL_FRAMES - KBD_TAP_DOWN_FRAMES - 1);
    return KBD_TAP_Z_LOW * (1.0f - t) + KBD_TAP_Z_HIGH * t;
}

#endif /* SPATIAL_KEYBOARD_DECODER_KBD_TAP_TRAJ_H */
