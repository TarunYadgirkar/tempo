/* test_decoder.c — synthetic-fingertip unit tests for the continuous
 * pose-stream typing decoder.  Verifies that:
 *   (a) the decoder emits the keysym corresponding to a tap location,
 *   (b) only one key fires per simulated tap (no flapping),
 *   (c) cooldown prevents immediate re-emission on a held position,
 *   (d) the mid-air decoder picks the highest-velocity finger,
 *   (e) typing a real English word produces (mostly) the right chars.
 *
 * SPDX-License-Identifier: MIT
 */

#include "decoder.h"
#include "keyboard_geom.h"
#include "kbd_tap_traj.h"
#include "lm.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define XK_a 0x0061
#define XK_b 0x0062
#define XK_c 0x0063
#define XK_e 0x0065
#define XK_f 0x0066
#define XK_h 0x0068
#define XK_l 0x006c
#define XK_o 0x006f
#define XK_q 0x0071
#define XK_t 0x0074

#define DT (1.0f / 60.0f)
#define N_FINGERS 10

static void
zero_fingers (float fingertips[N_FINGERS][3], bool present[N_FINGERS])
{
    memset (fingertips, 0, sizeof (float) * N_FINGERS * 3);
    memset (present, 0, sizeof (bool) * N_FINGERS);
}

/* Simulate one finger doing a tap-down-then-lift over the given key
 * center.  ~10 frames descent (positive z dropping to negative),
 * ~10 frames lift (negative back to positive).  Returns the keysym
 * emitted within the simulated trajectory (0 if none). */
static uint32_t
simulate_tap (decoder_t *dec, int finger_idx, float cx_m, float cy_m,
              int *out_emit_frame)
{
    if (out_emit_frame) *out_emit_frame = -1;
    float fingertips[N_FINGERS][3];
    bool  present[N_FINGERS];
    zero_fingers (fingertips, present);
    /* Park all other fingers far above the plane so they don't
     * fire by accident (shared park position — see kbd_tap_traj.h). */
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        fingertips[f][0] = KBD_TAP_PARK_X;
        fingertips[f][1] = KBD_TAP_PARK_Y;
        fingertips[f][2] = KBD_TAP_PARK_Z;
    }

    uint32_t first_emit = 0;
    for (int frame = 0; frame < KBD_TAP_TOTAL_FRAMES; ++frame) {
        /* Canonical descend-then-lift trajectory, shared with the e2e
         * injector via kbd_tap_traj.h. */
        fingertips[finger_idx][0] = cx_m;
        fingertips[finger_idx][1] = cy_m;
        fingertips[finger_idx][2] = kbd_tap_traj_z (frame);
        uint32_t emitted = decoder_update (dec, fingertips, present, DT);
        if (emitted && !first_emit) {
            first_emit = emitted;
            if (out_emit_frame) *out_emit_frame = frame;
        }
    }
    return first_emit;
}

static void
test_letter_at_key_center (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    assert (dec);

    /* Find the 'a' key and tap right on its center. */
    int idx = kbd_geom_find_keysym (XK_a);
    assert (idx >= 0);
    const kbd_key_geom_t *g = kbd_geom_get (idx);
    assert (g);

    int emit_frame = -1;
    uint32_t k = simulate_tap (dec, /*left index*/ 1, g->cx_m, g->cy_m,
                               &emit_frame);
    assert (k == XK_a);
    assert (emit_frame > 0 && emit_frame < 24);
    printf ("PASS test_letter_at_key_center  (emit on frame %d)\n", emit_frame);
    decoder_destroy (dec);
}

static void
test_tap_t_then_h (void)
{
    /* "th" is a heavily-weighted bigram so the LM should not
     * suppress this transition. */
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    int idx_t = kbd_geom_find_keysym (XK_t);
    int idx_h = kbd_geom_find_keysym (XK_h);
    assert (idx_t >= 0 && idx_h >= 0);
    const kbd_key_geom_t *gt = kbd_geom_get (idx_t);
    const kbd_key_geom_t *gh = kbd_geom_get (idx_h);

    uint32_t k1 = simulate_tap (dec, 1, gt->cx_m, gt->cy_m, NULL);
    assert (k1 == XK_t);

    /* Lift between taps: 8 frames with all fingers above plane. */
    float fingertips[N_FINGERS][3];
    bool present[N_FINGERS];
    zero_fingers (fingertips, present);
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        fingertips[f][0] = 0.5f; fingertips[f][1] = 0.5f; fingertips[f][2] = 0.2f;
    }
    for (int i = 0; i < 8; ++i) decoder_update (dec, fingertips, present, DT);

    uint32_t k2 = simulate_tap (dec, 1, gh->cx_m, gh->cy_m, NULL);
    assert (k2 == XK_h);
    printf ("PASS test_tap_t_then_h\n");
    decoder_destroy (dec);
}

static void
test_held_finger_does_not_repeat (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    int idx = kbd_geom_find_keysym (XK_a);
    const kbd_key_geom_t *g = kbd_geom_get (idx);

    /* Approach: standard tap-down trajectory. */
    int emit_frame = -1;
    uint32_t first = simulate_tap (dec, 1, g->cx_m, g->cy_m, &emit_frame);
    assert (first == XK_a);

    /* Now HOLD the finger at the same key center, no lift.  Expect
     * NO further emissions over the next 30 frames (cooldown +
     * peak detection both refuse to re-fire). */
    float fingertips[N_FINGERS][3];
    bool present[N_FINGERS];
    zero_fingers (fingertips, present);
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        fingertips[f][0] = 0.5f; fingertips[f][1] = 0.5f; fingertips[f][2] = 0.2f;
    }
    fingertips[1][0] = g->cx_m;
    fingertips[1][1] = g->cy_m;
    fingertips[1][2] = -0.005f;
    int repeats = 0;
    for (int i = 0; i < 30; ++i) {
        if (decoder_update (dec, fingertips, present, DT)) repeats++;
    }
    assert (repeats == 0);
    printf ("PASS test_held_finger_does_not_repeat\n");
    decoder_destroy (dec);
}

static void
test_off_key_tap_does_not_emit_neighbor (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    /* Tap right on the boundary between 'a' and 's'.  Acceptable for
     * v1 to fire either of them; just verify SOMETHING fires (not
     * silence). */
    int idx_a = kbd_geom_find_keysym (XK_a);
    int idx_s = kbd_geom_find_keysym (0x0073 /*XK_s*/);
    const kbd_key_geom_t *ga = kbd_geom_get (idx_a);
    const kbd_key_geom_t *gs = kbd_geom_get (idx_s);
    float midx = 0.5f * (ga->cx_m + gs->cx_m);

    uint32_t k = simulate_tap (dec, 1, midx, ga->cy_m, NULL);
    assert (k == XK_a || k == 0x0073);
    printf ("PASS test_off_key_tap_does_not_emit_neighbor (got %c)\n",
            (int) k);
    decoder_destroy (dec);
}

static void
test_midair_picks_fastest_finger (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_MIDAIR);
    int idx_a = kbd_geom_find_keysym (XK_a);
    int idx_l = kbd_geom_find_keysym (XK_l);
    const kbd_key_geom_t *ga = kbd_geom_get (idx_a);
    const kbd_key_geom_t *gl = kbd_geom_get (idx_l);

    /* Two fingers are present.  Left index is parked over 'a'.
     * Right index moves DOWN over 'l' at high velocity.  In midair
     * mode, attention should be on the right index → emit 'l'. */
    float fingertips[N_FINGERS][3];
    bool present[N_FINGERS];
    zero_fingers (fingertips, present);
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        fingertips[f][0] = KBD_TAP_PARK_X;
        fingertips[f][1] = KBD_TAP_PARK_Y;
        fingertips[f][2] = KBD_TAP_PARK_Z;
    }
    /* Park left index at 'a', motionless. */
    fingertips[1][0] = ga->cx_m;
    fingertips[1][1] = ga->cy_m;
    fingertips[1][2] = 0.0f;

    uint32_t first = 0;
    for (int frame = 0; frame < KBD_TAP_TOTAL_FRAMES; ++frame) {
        /* Right index (finger 6) descends rapidly over 'l' — shared tap. */
        fingertips[6][0] = gl->cx_m;
        fingertips[6][1] = gl->cy_m;
        fingertips[6][2] = kbd_tap_traj_z (frame);

        uint32_t k = decoder_update (dec, fingertips, present, DT);
        if (k && !first) first = k;
    }
    assert (first == XK_l);
    printf ("PASS test_midair_picks_fastest_finger\n");
    decoder_destroy (dec);
}

/* The real hardware failure mode: a clean tap trajectory corrupted with
 * ±8 mm z-jitter (reconstruction noise ~10 mm).  The old prox×velocity detector
 * scored these at −20 and never emitted; the trajectory detector must still. */
static void
test_jittery_tap_still_emits (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    int idx = kbd_geom_find_keysym (XK_e);
    const kbd_key_geom_t *g = kbd_geom_get (idx);

    float fingertips[N_FINGERS][3];
    bool  present[N_FINGERS];
    zero_fingers (fingertips, present);
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        fingertips[f][0] = KBD_TAP_PARK_X;
        fingertips[f][1] = KBD_TAP_PARK_Y;
        fingertips[f][2] = KBD_TAP_PARK_Z;
    }
    /* Deterministic ±8 mm sawtooth jitter on z. */
    const float jit[6] = { 0.008f, -0.006f, 0.007f, -0.008f, 0.005f, -0.007f };
    uint32_t k = 0;
    for (int frame = 0; frame < KBD_TAP_TOTAL_FRAMES; ++frame) {
        fingertips[1][0] = g->cx_m;
        fingertips[1][1] = g->cy_m;
        fingertips[1][2] = kbd_tap_traj_z (frame) + jit[frame % 6];
        uint32_t e = decoder_update (dec, fingertips, present, DT);
        if (e && !k) k = e;
    }
    assert (k == XK_e);
    printf ("PASS test_jittery_tap_still_emits\n");
    decoder_destroy (dec);
}

/* A fast jab: down past the plane and back up in ~6 frames.  The old
 * EMA-smoothed detector could not accumulate score this fast; the trajectory
 * detector only needs one frame near the bottom. */
static void
test_fast_tap_still_emits (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    int idx = kbd_geom_find_keysym (XK_o);
    const kbd_key_geom_t *g = kbd_geom_get (idx);

    float fingertips[N_FINGERS][3];
    bool  present[N_FINGERS];
    zero_fingers (fingertips, present);
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        fingertips[f][0] = KBD_TAP_PARK_X;
        fingertips[f][1] = KBD_TAP_PARK_Y;
        fingertips[f][2] = KBD_TAP_PARK_Z;
    }
    const float zseq[8] = { 0.04f, 0.02f, -0.01f, -0.005f,
                            0.01f, 0.03f, 0.05f, 0.05f };
    uint32_t k = 0;
    for (int frame = 0; frame < 8; ++frame) {
        fingertips[1][0] = g->cx_m;
        fingertips[1][1] = g->cy_m;
        fingertips[1][2] = zseq[frame];
        uint32_t e = decoder_update (dec, fingertips, present, DT);
        if (e && !k) k = e;
    }
    assert (k == XK_o);
    printf ("PASS test_fast_tap_still_emits\n");
    decoder_destroy (dec);
}

/* Deterministic LCG → reproducible pseudo-random jitter in [-amp, +amp]. */
static unsigned g_lcg = 0x1234567u;
static float
lcg_jit (float amp)
{
    g_lcg = g_lcg * 1103515245u + 12345u;
    float u = (float) ((g_lcg >> 8) & 0xffff) / 65535.0f;
    return (u * 2.0f - 1.0f) * amp;
}

/* The garbled-output bug, fuzzed across the WHOLE ENGAGE–ARM band.  After a real
 * tap the finger rests at various heights near the plane and jitters ±10 mm (the
 * natural inter-keystroke posture).  A prior fix held only in the outer band — a
 * mid-band rest (~18 mm) still phantom-fired a neighbour ~15%/s.  Requiring a
 * real press DEPTH (not just band-crossing) must give ZERO phantoms at EVERY
 * resting height, since jitter can't drive the smoothed min to the surface and
 * back above ARM in one cycle. */
static void
test_resting_jitter_no_phantom (void)
{
    const float heights[] = { 0.000f, 0.008f, 0.012f, 0.018f, 0.022f };
    for (unsigned h = 0; h < sizeof (heights) / sizeof (heights[0]); ++h) {
        int total = 0;
        for (int trial = 0; trial < 150; ++trial) {
            decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
            int idx = kbd_geom_find_keysym (XK_a);
            const kbd_key_geom_t *g = kbd_geom_get (idx);
            assert (simulate_tap (dec, 1, g->cx_m, g->cy_m, NULL) == XK_a);

            float fingertips[N_FINGERS][3];
            bool  present[N_FINGERS];
            zero_fingers (fingertips, present);
            for (int f = 0; f < N_FINGERS; ++f) {
                present[f] = true;
                fingertips[f][0] = 0.5f; fingertips[f][1] = 0.5f; fingertips[f][2] = 0.2f;
            }
            for (int i = 0; i < 90; ++i) {   /* 1.5 s of resting jitter */
                fingertips[1][0] = g->cx_m;
                fingertips[1][1] = g->cy_m;
                fingertips[1][2] = heights[h] + lcg_jit (0.010f);
                if (decoder_update (dec, fingertips, present, DT)) total++;
            }
            decoder_destroy (dec);
        }
        if (total != 0)
            printf ("  FAIL height=%.0fmm phantoms=%d / 150 trials\n",
                    (double) heights[h] * 1000.0, total);
        assert (total == 0);
    }
    printf ("PASS test_resting_jitter_no_phantom (5 heights × 150 trials)\n");
}

/* A CLEAN on-center tap on a RARE letter (q) must win on its spatial evidence,
 * not be flipped to a common neighbour by the LM prior. */
static void
test_rare_letter_clean_tap (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    int idx = kbd_geom_find_keysym (XK_q);
    const kbd_key_geom_t *g = kbd_geom_get (idx);
    uint32_t k = simulate_tap (dec, 1, g->cx_m, g->cy_m, NULL);
    assert (k == XK_q);
    printf ("PASS test_rare_letter_clean_tap\n");
    decoder_destroy (dec);
}

/* Averaging the press position must resist a single-frame xy SPIKE at the
 * bottom that would otherwise pick a neighbour.  This is the behaviour the
 * averaging change exists for — every other test holds xy constant, so only
 * this one distinguishes "average over the press" from "xy at the min frame".
 * Tap 'a' with small xy wobble, plus a big spike to 's' exactly on the deepest
 * frame; the mean stays on 'a' (the min-frame classifier would emit 's'). */
static void
test_xy_spike_at_min_averaged_out (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    int idx_a = kbd_geom_find_keysym (XK_a);
    int idx_s = kbd_geom_find_keysym (0x0073 /*XK_s*/);
    const kbd_key_geom_t *ga = kbd_geom_get (idx_a);
    const kbd_key_geom_t *gs = kbd_geom_get (idx_s);

    float fingertips[N_FINGERS][3];
    bool  present[N_FINGERS];
    zero_fingers (fingertips, present);
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        fingertips[f][0] = KBD_TAP_PARK_X;
        fingertips[f][1] = KBD_TAP_PARK_Y;
        fingertips[f][2] = KBD_TAP_PARK_Z;
    }
    uint32_t k = 0;
    for (int frame = 0; frame < KBD_TAP_TOTAL_FRAMES; ++frame) {
        fingertips[1][0] = ga->cx_m + ((frame % 2) ? 0.003f : -0.003f);
        fingertips[1][1] = ga->cy_m;
        fingertips[1][2] = kbd_tap_traj_z (frame);
        if (frame == KBD_TAP_DOWN_FRAMES - 1)   /* the deepest (min-z) frame */
            fingertips[1][0] = gs->cx_m;         /* spike toward 's' */
        uint32_t e = decoder_update (dec, fingertips, present, DT);
        if (e && !k) k = e;
    }
    assert (k == XK_a);
    printf ("PASS test_xy_spike_at_min_averaged_out\n");
    decoder_destroy (dec);
}

/* Simulate a full-amplitude tap on 'a' shifted UP by `bias` metres
 * (valley -5 mm → -5 mm + bias).  Returns the first emitted keysym. */
static uint32_t
simulate_biased_tap_on_a (decoder_t *dec, float bias)
{
    int idx = kbd_geom_find_keysym (XK_a);
    const kbd_key_geom_t *g = kbd_geom_get (idx);

    float fingertips[N_FINGERS][3];
    bool  present[N_FINGERS];
    zero_fingers (fingertips, present);
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        fingertips[f][0] = KBD_TAP_PARK_X;
        fingertips[f][1] = KBD_TAP_PARK_Y;
        fingertips[f][2] = KBD_TAP_PARK_Z;
    }
    uint32_t k = 0;
    for (int frame = 0; frame < KBD_TAP_TOTAL_FRAMES; ++frame) {
        fingertips[1][0] = g->cx_m;
        fingertips[1][1] = g->cy_m;
        fingertips[1][2] = kbd_tap_traj_z (frame) + bias;
        uint32_t e = decoder_update (dec, fingertips, present, DT);
        if (e && !k) k = e;
    }
    return k;
}

/* THE reason this detector exists: the reconstruction reads the hand ~an inch
 * high, so a real tap bottoms out ABOVE the plane, never through it.  A plunge
 * of full amplitude bottoming at +10 mm (which the old absolute 6 mm depth gate
 * dropped) must still fire — that's the on-hardware drop bug this fixes. */
static void
test_biased_shallow_tap_still_fires (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    /* whole tap shifted UP: valley -5mm → +10mm */
    assert (simulate_biased_tap_on_a (dec, 0.015f) == XK_a);
    printf ("PASS test_biased_shallow_tap_still_fires\n");
    decoder_destroy (dec);
}

/* keyboard-auto-repeat: a sustained low-z dwell on a key yields a held-key
 * event (press without immediate release, release on lift), distinct from
 * the tap path that emits a single keysym. */
static void
test_hold_key_press_and_release_on_lift (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    int idx = kbd_geom_find_keysym (XK_a);
    const kbd_key_geom_t *g = kbd_geom_get (idx);

    float fingertips[N_FINGERS][3];
    bool  present[N_FINGERS];
    zero_fingers (fingertips, present);
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        fingertips[f][0] = KBD_TAP_PARK_X;
        fingertips[f][1] = KBD_TAP_PARK_Y;
        fingertips[f][2] = KBD_TAP_PARK_Z;
    }

    /* Descend onto 'a' (the tap trajectory's down leg)… */
    for (int frame = 0; frame < KBD_TAP_DOWN_FRAMES; ++frame) {
        fingertips[1][0] = g->cx_m;
        fingertips[1][1] = g->cy_m;
        fingertips[1][2] = kbd_tap_traj_z (frame);
        decoder_key_event_t ev;
        uint32_t k = decoder_update_ev (dec, fingertips, present, DT, &ev);
        assert (k == 0 && ev.kind == DECODER_KEY_EVENT_NONE);
    }

    /* …then DWELL pressed at the bottom for 1 s instead of lifting. */
    int hold_begins = 0, begin_frame = -1;
    for (int i = 0; i < 60; ++i) {
        fingertips[1][2] = KBD_TAP_Z_LOW;
        decoder_key_event_t ev;
        uint32_t k = decoder_update_ev (dec, fingertips, present, DT, &ev);
        assert (k == 0);   /* press without immediate release: no tap keysym */
        assert (ev.kind != DECODER_KEY_EVENT_TAP);
        assert (ev.kind != DECODER_KEY_EVENT_HOLD_END);
        if (ev.kind == DECODER_KEY_EVENT_HOLD_BEGIN) {
            hold_begins++;
            begin_frame = i;
            assert (ev.keysym == XK_a);
        }
    }
    assert (hold_begins == 1);
    /* Fired after the 0.40 s low-z dwell gate (the descent's last ~6
     * frames already count as low-z), not on touch. */
    assert (begin_frame >= 12 && begin_frame <= 30);

    /* Lift: the release arrives as HOLD_END; the same plunge must NOT
     * also fire a tap. */
    int hold_ends = 0;
    for (int i = 0; i < 12; ++i) {
        fingertips[1][2] = KBD_TAP_Z_LOW + 0.006f * (float) (i + 1);
        decoder_key_event_t ev;
        uint32_t k = decoder_update_ev (dec, fingertips, present, DT, &ev);
        assert (k == 0);
        assert (ev.kind != DECODER_KEY_EVENT_TAP);
        if (ev.kind == DECODER_KEY_EVENT_HOLD_END) {
            hold_ends++;
            assert (ev.keysym == XK_a);
        }
    }
    assert (hold_ends == 1);
    decoder_destroy (dec);

    /* Distinct from the tap path: the canonical tap trajectory reports a
     * single TAP event and no hold events. */
    dec = decoder_create (DECODER_MODE_SURFACE);
    zero_fingers (fingertips, present);
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        fingertips[f][0] = KBD_TAP_PARK_X;
        fingertips[f][1] = KBD_TAP_PARK_Y;
        fingertips[f][2] = KBD_TAP_PARK_Z;
    }
    int taps = 0, holds = 0;
    for (int frame = 0; frame < KBD_TAP_TOTAL_FRAMES; ++frame) {
        fingertips[1][0] = g->cx_m;
        fingertips[1][1] = g->cy_m;
        fingertips[1][2] = kbd_tap_traj_z (frame);
        decoder_key_event_t ev;
        uint32_t k = decoder_update_ev (dec, fingertips, present, DT, &ev);
        if (ev.kind == DECODER_KEY_EVENT_TAP) {
            taps++;
            assert (k == ev.keysym && ev.keysym == XK_a);
        } else if (ev.kind != DECODER_KEY_EVENT_NONE) {
            holds++;
        }
    }
    assert (taps == 1 && holds == 0);
    decoder_destroy (dec);
    printf ("PASS test_hold_key_press_and_release_on_lift\n");
}

/* A held key lost mid-hold (tracking dropout) must still release. */
static void
test_hold_key_releases_on_finger_loss (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    int idx = kbd_geom_find_keysym (XK_a);
    const kbd_key_geom_t *g = kbd_geom_get (idx);

    float fingertips[N_FINGERS][3];
    bool  present[N_FINGERS];
    zero_fingers (fingertips, present);
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        fingertips[f][0] = KBD_TAP_PARK_X;
        fingertips[f][1] = KBD_TAP_PARK_Y;
        fingertips[f][2] = KBD_TAP_PARK_Z;
    }
    int begins = 0, ends = 0;
    for (int frame = 0; frame < KBD_TAP_DOWN_FRAMES + 40; ++frame) {
        fingertips[1][0] = g->cx_m;
        fingertips[1][1] = g->cy_m;
        fingertips[1][2] = (frame < KBD_TAP_DOWN_FRAMES)
                           ? kbd_tap_traj_z (frame) : KBD_TAP_Z_LOW;
        decoder_key_event_t ev;
        decoder_update_ev (dec, fingertips, present, DT, &ev);
        if (ev.kind == DECODER_KEY_EVENT_HOLD_BEGIN) begins++;
    }
    assert (begins == 1);

    present[1] = false;   /* tracker drops the holding finger */
    decoder_key_event_t ev;
    decoder_update_ev (dec, fingertips, present, DT, &ev);
    if (ev.kind == DECODER_KEY_EVENT_HOLD_END) ends++;
    assert (ends == 1 && ev.keysym == XK_a);
    decoder_destroy (dec);
    printf ("PASS test_hold_key_releases_on_finger_loss\n");
}

/* --- TOML config loader ------------------------------------------ */

static void
write_file (const char *path, const char *content)
{
    FILE *f = fopen (path, "w");
    assert (f);
    fputs (content, f);
    fclose (f);
}

/* A TOML file with a non-default [surface] sigma_z_m must be picked up
 * and shift decoder_update: with a 4 mm near-plane gate, the biased
 * shallow tap (valley at +10 mm) is rejected, where the 25 mm default
 * accepts it (test_biased_shallow_tap_still_fires). */
static void
test_toml_sigma_z_override (void)
{
    char dir[256];
    const char *tmp = getenv ("TMPDIR");
    snprintf (dir, sizeof (dir), "%s/spatial-kbd-toml-XXXXXX",
              (tmp && *tmp) ? tmp : "/tmp");
    assert (mkdtemp (dir));
    char path[256];
    snprintf (path, sizeof (path), "%s/keyboard.toml", dir);
    write_file (path,
                "# test override\n"
                "[surface]\n"
                "sigma_z_m = 0.004\n");

    decoder_t *dec = decoder_create_with_config (DECODER_MODE_SURFACE, path);
    assert (dec);
    const decoder_config_t *cfg = decoder_get_config (dec);
    assert (cfg->sigma_z_m > 0.0039f && cfg->sigma_z_m < 0.0041f);
    assert (cfg->lm_weight_surface > 0.21f && cfg->lm_weight_surface < 0.23f);

    assert (simulate_biased_tap_on_a (dec, 0.015f) == 0);
    decoder_destroy (dec);

    /* Same file, unbiased tap (valley -5 mm < 4 mm gate): still fires,
     * so the override tightened the gate rather than breaking taps. */
    dec = decoder_create_with_config (DECODER_MODE_SURFACE, path);
    assert (simulate_biased_tap_on_a (dec, 0.0f) == XK_a);
    decoder_destroy (dec);

    unlink (path);
    rmdir (dir);
    printf ("PASS test_toml_sigma_z_override\n");
}

/* decoder_create (no explicit path) must read the XDG default
 * location: $XDG_CONFIG_HOME/spatial-os/keyboard.toml. */
static void
test_toml_default_xdg_path (void)
{
    const char *xdg = getenv ("XDG_CONFIG_HOME");
    assert (xdg && *xdg);   /* main() points this at a temp dir */
    char subdir[256], path[256];
    snprintf (subdir, sizeof (subdir), "%s/spatial-os", xdg);
    assert (mkdir (subdir, 0700) == 0);
    snprintf (path, sizeof (path), "%s/keyboard.toml", subdir);
    write_file (path,
                "[surface]\n"
                "sigma_z_m = 0.004\n"
                "[streaming]\n"
                "cooldown_s = 0.5\n");

    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    const decoder_config_t *cfg = decoder_get_config (dec);
    assert (cfg->sigma_z_m > 0.0039f && cfg->sigma_z_m < 0.0041f);
    assert (cfg->cooldown_s > 0.49f && cfg->cooldown_s < 0.51f);
    decoder_destroy (dec);

    assert (unlink (path) == 0);
    assert (rmdir (subdir) == 0);
    printf ("PASS test_toml_default_xdg_path\n");
}

/* An inter-key MOVE (lift, translate, descend to the next key's hover) is a
 * plunge too, but it bottoms at HOVER height, not near the plane.  It must NOT
 * fire — only a tap, which bottoms near the plane, does. */
static void
test_inter_key_move_no_fire (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    float fingertips[N_FINGERS][3];
    bool  present[N_FINGERS];
    zero_fingers (fingertips, present);
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        fingertips[f][0] = 0.15f; fingertips[f][1] = 0.06f; fingertips[f][2] = 0.2f;
    }
    /* Big up→down→up bob bottoming at 45 mm (hover height, well above NEAR_Z). */
    const float zseq[] = { 0.080f, 0.070f, 0.055f, 0.048f, 0.045f,
                           0.050f, 0.065f, 0.080f, 0.085f };
    int fires = 0;
    for (unsigned i = 0; i < sizeof (zseq) / sizeof (zseq[0]); ++i) {
        fingertips[1][2] = zseq[i];
        if (decoder_update (dec, fingertips, present, DT)) fires++;
    }
    assert (fires == 0);
    printf ("PASS test_inter_key_move_no_fire\n");
    decoder_destroy (dec);
}


/* --- Finger prominence + two-finger arbitration ------------------- */

/* A plunge with an explicitly-chosen bottom: 6-frame descent, 6-frame dwell at
 * the bottom (long enough for the decoder's z EMA to converge, far short of the
 * 0.40 s hold gate), 8-frame lift.  Amplitude clears tap_plunge_m either way,
 * so only the bottom height and the finger posture decide whether it fires. */
#define PLUNGE_AMP_M   0.033f
#define PLUNGE_FRAMES  20

static float
plunge_z (int frame, float valley_m)
{
    const float top = valley_m + PLUNGE_AMP_M;
    if (frame < 6) {
        const float t = (float) frame / 5.0f;
        return top * (1.0f - t) + valley_m * t;
    }
    if (frame < 12) return valley_m;
    float t = (float) (frame - 12) / 7.0f;
    if (t > 1.0f) t = 1.0f;
    return valley_m * (1.0f - t) + top * t;
}

static float
plunge_z_at (int frame, int start, float valley_m)
{
    const int i = frame - start;
    if (i < 0 || i >= PLUNGE_FRAMES) return valley_m + PLUNGE_AMP_M;
    return plunge_z (i, valley_m);
}

/* Left index plunges onto `g` bottoming at `valley_m`.  The other four
 * left-hand fingers either ride down with it (prom_m == 0 — the inter-key MOVE
 * posture) or sit `prom_m` above the valley (a real tap, one finger extended
 * below the hand).  Returns the first keysym emitted. */
static uint32_t
simulate_plunge_with_prominence (decoder_t *dec, const kbd_key_geom_t *g,
                                 float valley_m, float prom_m)
{
    float fingertips[N_FINGERS][3];
    bool  present[N_FINGERS];
    zero_fingers (fingertips, present);
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        fingertips[f][0] = KBD_TAP_PARK_X;
        fingertips[f][1] = KBD_TAP_PARK_Y;
        fingertips[f][2] = KBD_TAP_PARK_Z;
    }
    for (int f = 0; f < 5; ++f) {
        if (f == 1) continue;
        fingertips[f][0] = g->cx_m + 0.02f * (float) (f + 1);
        fingertips[f][1] = g->cy_m;
    }

    uint32_t k = 0;
    for (int frame = 0; frame < PLUNGE_FRAMES; ++frame) {
        const float z = plunge_z (frame, valley_m);
        fingertips[1][0] = g->cx_m;
        fingertips[1][1] = g->cy_m;
        fingertips[1][2] = z;
        for (int f = 0; f < 5; ++f) {
            if (f == 1) continue;
            fingertips[f][2] = (prom_m > 0.0f) ? valley_m + prom_m : z;
        }
        uint32_t e = decoder_update (dec, fingertips, present, DT);
        if (e && !k) k = e;
    }
    return k;
}

/* THE finding-1 false-fire: a move between keys that dips low (22 mm) has the
 * same (amplitude, valley) signature as a biased tap, so the near-plane gate
 * alone takes it.  It carries the whole hand down together, so the prominence
 * gate rejects it. */
static void
test_low_move_fingers_together_no_fire (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    const kbd_key_geom_t *g = kbd_geom_get (kbd_geom_find_keysym (XK_a));
    assert (simulate_plunge_with_prominence (dec, g, 0.022f, 0.0f) == 0);
    printf ("PASS test_low_move_fingers_together_no_fire\n");
    decoder_destroy (dec);
}

/* Same amplitude, same valley — but the tapping finger is extended 10 mm below
 * its neighbours.  That is a tap and must fire. */
static void
test_prominent_finger_tap_fires (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    const kbd_key_geom_t *g = kbd_geom_get (kbd_geom_find_keysym (XK_a));
    assert (simulate_plunge_with_prominence (dec, g, 0.022f, 0.010f) == XK_a);
    printf ("PASS test_prominent_finger_tap_fires\n");
    decoder_destroy (dec);
}

/* Two fingers, one per hand, each extended below its own hand, plunging onto
 * `ga` and `gb` with the second starting `offset` frames later so the lifts
 * land inside the 70 ms emit refractory.  Collects the emitted keysyms in
 * order. */
static int
simulate_two_finger_plunges (decoder_t *dec, const kbd_key_geom_t *ga,
                             const kbd_key_geom_t *gb, int offset,
                             uint32_t out_keys[4])
{
    const float valley = 0.000f;
    const float prom   = 0.020f;

    float fingertips[N_FINGERS][3];
    bool  present[N_FINGERS];
    zero_fingers (fingertips, present);
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        fingertips[f][0] = KBD_TAP_PARK_X;
        fingertips[f][1] = KBD_TAP_PARK_Y;
        fingertips[f][2] = valley + prom;
    }
    fingertips[1][0] = ga->cx_m; fingertips[1][1] = ga->cy_m;
    fingertips[6][0] = gb->cx_m; fingertips[6][1] = gb->cy_m;

    int n = 0;
    for (int frame = 0; frame < PLUNGE_FRAMES + offset + 24; ++frame) {
        fingertips[1][2] = plunge_z_at (frame, 0,      valley);
        fingertips[6][2] = plunge_z_at (frame, offset, valley);
        uint32_t e = decoder_update (dec, fingertips, present, DT);
        if (e && n < 4) out_keys[n++] = e;
    }
    return n;
}

/* finding-2: two fingers whose lift-offs are 2 frames apart.  The second used
 * to be dropped by the one-key-per-frame rule + the emit refractory; it must
 * now be deferred and released, in valley order. */
static void
test_two_finger_taps_both_emit_in_order (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    const kbd_key_geom_t *ga = kbd_geom_get (kbd_geom_find_keysym (XK_a));
    const kbd_key_geom_t *gl = kbd_geom_get (kbd_geom_find_keysym (XK_l));
    uint32_t keys[4] = { 0 };
    int n = simulate_two_finger_plunges (dec, ga, gl, 2, keys);
    if (n != 2 || keys[0] != XK_a || keys[1] != XK_l)
        printf ("  FAIL n=%d keys=%c,%c\n", n, (int) keys[0], (int) keys[1]);
    assert (n == 2 && keys[0] == XK_a && keys[1] == XK_l);
    printf ("PASS test_two_finger_taps_both_emit_in_order\n");
    decoder_destroy (dec);
}

/* The case the refractory exists for: the SAME key struck twice inside the
 * window is one keystroke bouncing, not two.  The defer queue must not
 * resurrect it. */
static void
test_same_key_twice_in_refractory_emits_once (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    const kbd_key_geom_t *ga = kbd_geom_get (kbd_geom_find_keysym (XK_a));
    uint32_t keys[4] = { 0 };
    int n = simulate_two_finger_plunges (dec, ga, ga, 2, keys);
    if (n != 1 || keys[0] != XK_a)
        printf ("  FAIL n=%d keys=%c,%c\n", n, (int) keys[0], (int) keys[1]);
    assert (n == 1 && keys[0] == XK_a);
    printf ("PASS test_same_key_twice_in_refractory_emits_once\n");
    decoder_destroy (dec);
}

/* Sweep the plunge bottom in 1 mm steps and report the highest bottom at which
 * a real (prominent-finger) tap still fires, and the highest at which a
 * fingers-together move FALSE-fires.  Their difference is the tap/not-tap
 * margin at the near-plane gate: with the near-plane gate alone both curves
 * have the same threshold (margin 0 — no discrimination at any height), and
 * the prominence gate pushes the move curve off the bottom of the sweep. */
#define SWEEP_MAX_MM 40
#define SWEEP_NONE   (-1)

static void
sweep_thresholds (const char *toml_path, int *out_tap_mm, int *out_move_mm)
{
    *out_tap_mm = SWEEP_NONE;
    *out_move_mm = SWEEP_NONE;
    for (int mm = 0; mm <= SWEEP_MAX_MM; ++mm) {
        const float h = (float) mm / 1000.0f;
        const kbd_key_geom_t *g = kbd_geom_get (kbd_geom_find_keysym (XK_a));

        decoder_t *d1 = decoder_create_with_config (DECODER_MODE_SURFACE, toml_path);
        if (simulate_plunge_with_prominence (d1, g, h, 0.010f)) *out_tap_mm = mm;
        decoder_destroy (d1);

        decoder_t *d2 = decoder_create_with_config (DECODER_MODE_SURFACE, toml_path);
        if (simulate_plunge_with_prominence (d2, g, h, 0.0f)) *out_move_mm = mm;
        decoder_destroy (d2);
    }
}

static void
test_near_z_margin_sweep (void)
{
    char dir[256];
    const char *tmp = getenv ("TMPDIR");
    snprintf (dir, sizeof (dir), "%s/spatial-kbd-sweep-XXXXXX",
              (tmp && *tmp) ? tmp : "/tmp");
    assert (mkdtemp (dir));
    char path[256];
    snprintf (path, sizeof (path), "%s/keyboard.toml", dir);
    write_file (path, "[surface]\ntap_prominence_m = 0.0\n");

    int tap_before = 0, move_before = 0, tap_after = 0, move_after = 0;
    sweep_thresholds (path, &tap_before, &move_before);   /* gate disabled */
    sweep_thresholds (NULL, &tap_after, &move_after);     /* compiled default */

    const int margin_before = tap_before - move_before;
    const int margin_after  = tap_after - move_after;
    printf ("  sweep 0..%d mm  prominence gate OFF: tap fires up to %d mm, "
            "move false-fires up to %d mm → margin %d mm\n",
            SWEEP_MAX_MM, tap_before, move_before, margin_before);
    printf ("  sweep 0..%d mm  prominence gate ON : tap fires up to %d mm, "
            "move never false-fires → margin %d mm\n",
            SWEEP_MAX_MM, tap_after, margin_after);

    assert (tap_before > 0 && tap_after > 0);
    assert (margin_before <= 2);        /* no discrimination without the gate */
    assert (move_after == SWEEP_NONE);  /* a move never fires with it */
    assert (margin_after >= 20);

    unlink (path);
    rmdir (dir);
    printf ("PASS test_near_z_margin_sweep\n");
}

int
main (void)
{
    /* Hermetic XDG config: a host ~/.config/spatial-os/keyboard.toml
     * must not skew the compiled-default assertions below. */
    char xdg_dir[256];
    const char *tmp = getenv ("TMPDIR");
    snprintf (xdg_dir, sizeof (xdg_dir), "%s/spatial-kbd-xdg-XXXXXX",
              (tmp && *tmp) ? tmp : "/tmp");
    assert (mkdtemp (xdg_dir));
    setenv ("XDG_CONFIG_HOME", xdg_dir, 1);

    printf ("=== decoder tests ===\n");
    test_letter_at_key_center ();
    test_tap_t_then_h ();
    test_held_finger_does_not_repeat ();
    test_off_key_tap_does_not_emit_neighbor ();
    test_midair_picks_fastest_finger ();
    test_jittery_tap_still_emits ();
    test_fast_tap_still_emits ();
    test_resting_jitter_no_phantom ();
    test_rare_letter_clean_tap ();
    test_xy_spike_at_min_averaged_out ();
    test_biased_shallow_tap_still_fires ();
    test_inter_key_move_no_fire ();
    test_hold_key_press_and_release_on_lift ();
    test_hold_key_releases_on_finger_loss ();
    test_toml_sigma_z_override ();
    test_toml_default_xdg_path ();
    test_low_move_fingers_together_no_fire ();
    test_prominent_finger_tap_fires ();
    test_two_finger_taps_both_emit_in_order ();
    test_same_key_twice_in_refractory_emits_once ();
    test_near_z_margin_sweep ();
    printf ("=== ALL DECODER TESTS PASSED ===\n");
    rmdir (xdg_dir);
    return 0;
}
