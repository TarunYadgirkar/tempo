/* decoder.c — see decoder.h.
 *
 * SURFACE mode (typing on a plane): a per-finger z-trajectory state machine
 * (decoder_update_surface) detects a tap as descend-into-band → local-min →
 * lift-off, then classifies the key at the min via Σ N(xy; μ, Σ) fused with a
 * bigram LM.  This replaced an EMA of prox×velocity scores that collapsed on
 * ~10 mm tracking jitter (the velocity sign flipped and zeroed the weight).
 *
 * MIDAIR mode: per-frame velocity-softmax attention over fingers, fused with the
 * LM and EMA-smoothed, emitting on a rising-then-falling peak.
 *
 * Thresholds/cooldown/LM weight/σ default to the typical iPhone hand-tracking
 * jitter envelope (~10 mm σ) and are overridable per-user from
 * ~/.config/spatial-os/keyboard.toml (decoder_config.c).
 */

#include "decoder.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "keyboard_geom.h"
#include "lm.h"

#define N_FINGERS 10
#define MAX_KEYS  64
/* Deepest realistic pile-up is one queued tap per finger of the other hand. */
#define DEFER_MAX 8

/* Tunables live in decoder_config_t (defaults + TOML overrides in
 * decoder_config.c). */

/* --- Surface tap detector (RELATIVE plunge, not absolute depth) --- */
/* The hand reconstruction places the whole hand ~an inch too high, so a
 * fingertip's ABSOLUTE depth vs the plane is unreliable — it barely reaches the
 * plane even when the finger physically punches through to the desk (measured
 * on hardware).  But the RELATIVE motion is faithful — a constant depth bias
 * cancels in a plunge measured peak-to-valley — so a tap is detected from the
 * plunge, not the depth:
 *   PRESS  — z falls ≥ cfg.tap_hyst_m below a tracked peak → a plunge begins.
 *   TAP    — it descends ≥ cfg.tap_plunge_m (peak→valley: a deliberate press,
 *            not ~10 mm jitter), bottoms out near the plane (valley <
 *            cfg.sigma_z_m: a real tap, not an inter-key MOVE that bottoms at
 *            hover height), then LIFTS ≥ cfg.tap_plunge_m back up (a real
 *            release, not a SETTLE that just jitters at the bottom — the old
 *            phantom-fire mode).
 * Only the loose near-plane gate (cfg.sigma_z_m) is absolute (and far more
 * generous than a couple mm), so the detector survives the reconstruction
 * depth bias.  z is lightly smoothed; cfg.tap_hyst_m debounces direction
 * flips against jitter.
 *
 * The near-plane gate alone leaves only ~2-3 mm between a deliberate-but-biased
 * tap and an inter-key MOVE that dips low, so TAP and HOLD_BEGIN carry a second,
 * also-bias-invariant gate: FINGER PROMINENCE.  At the valley a real tap has one
 * finger extended clearly below the rest of its own hand, whereas a move carries
 * the fingers down together — so the tapping finger must sit cfg.tap_prominence_m
 * below the mean of the other tracked fingers of the SAME hand (fingers 0-4 left,
 * 5-9 right).  Being hand-RELATIVE it cancels the depth bias exactly the way the
 * plunge does.  A hand with no second tracked finger has no reference, so the
 * gate is skipped rather than guessed at. */

/* --- Default per-key 2D Gaussian width ------------------------- */
/* Initial σ: half a key width — overlap with neighbours via the
 * Gaussian tail.  Refined per-user by the spatial-model-refit
 * agent task in Phase 6. */
static float
default_sigma_x (int key_idx)
{
    const kbd_key_geom_t *g = kbd_geom_get (key_idx);
    return g ? g->hw_m : 0.015f;
}
static float
default_sigma_y (int key_idx)
{
    const kbd_key_geom_t *g = kbd_geom_get (key_idx);
    return g ? g->hh_m : 0.015f;
}

/* ---------------------------------------------------------------- */

struct decoder_t {
    decoder_mode_t   mode;
    decoder_config_t cfg;
    lm_t            *lm;

    /* Previous frame's fingertip positions (for velocity). */
    float prev_xy_z[N_FINGERS][3];
    bool  prev_present[N_FINGERS];
    bool  velocity_warm;

    /* Per-key smoothed score (log domain). */
    float smoothed[MAX_KEYS];
    /* Direction tracking for peak detection: number of consecutive
     * frames the argmax key's smoothed score has been decreasing. */
    int   argmax_idx;
    float argmax_prev;
    int   argmax_falling_frames;
    float argmax_peak_score;

    /* Per-key cooldown remaining (s). */
    float cooldown_s[MAX_KEYS];

    /* Last emitted key index (for LM context). */
    int   last_emitted_idx;

    /* Global refractory remaining (s) after any emit (surface mode). */
    float emit_refractory_s;

    /* Per-finger surface tap detector (relative peak/valley plunge). */
    float tap_zs[N_FINGERS];         /* lightly-smoothed z */
    bool  tap_descending[N_FINGERS]; /* z currently in a plunge (falling) */
    float tap_peak[N_FINGERS];       /* z at the top of the current plunge */
    float tap_valley[N_FINGERS];     /* deepest z this plunge */
    float tap_valley_t[N_FINGERS];   /* now_s when that valley was recorded */
    float tap_prom[N_FINGERS];       /* at the valley: other-fingers-mean-z − z
                                      * (INFINITY when the hand has no other
                                      * tracked finger — gate skipped) */
    float tap_xy_sum[N_FINGERS][2];  /* Σ (x,y) near the valley → averaged tap pos */
    int   tap_xy_n[N_FINGERS];       /* frames summed near the valley */

    /* Monotonic surface-mode clock (s), for valley ordering. */
    float now_s;

    /* Deferred taps: a genuine second tap that completed inside the emit
     * refractory is queued in valley order and released one per update tick
     * once the refractory expires, instead of being dropped. */
    uint32_t defer_keysym[DEFER_MAX];
    float    defer_valley_t[DEFER_MAX];
    int      defer_n;

    /* Held-key state (auto-repeat).  One hold at a time: a plunge that
     * dwells pressed at its bottom for cfg.hold_dwell_s instead of
     * lifting becomes a HOLD_BEGIN; the lift becomes HOLD_END. */
    float hold_bottom_dwell_s[N_FINGERS];
    bool  hold_active;
    int   hold_key_idx;
    int   hold_finger;
};

decoder_t *
decoder_create_with_config (decoder_mode_t initial_mode, const char *toml_path)
{
    decoder_t *self = calloc (1, sizeof (*self));
    if (!self) return NULL;
    self->mode = initial_mode;
    decoder_config_defaults (&self->cfg);
    decoder_config_load (&self->cfg, toml_path);
    self->lm = lm_create ();
    if (!self->lm) { free (self); return NULL; }
    self->last_emitted_idx = -1;
    self->argmax_idx = -1;
    for (int k = 0; k < MAX_KEYS; ++k) self->smoothed[k] = -20.0f;
    return self;
}

decoder_t *
decoder_create (decoder_mode_t initial_mode)
{
    return decoder_create_with_config (initial_mode, NULL);
}

const decoder_config_t *
decoder_get_config (const decoder_t *self)
{
    return self ? &self->cfg : NULL;
}

void
decoder_destroy (decoder_t *self)
{
    if (!self) return;
    lm_destroy (self->lm);
    free (self);
}

void
decoder_set_mode (decoder_t *self, decoder_mode_t mode)
{
    if (self) self->mode = mode;
}

/* ---------------------------------------------------------------- */
/* Per-finger intent weight                                          */
/* ---------------------------------------------------------------- */

/* Surface intent is no longer a per-frame prox×velocity weight — the brittle
 * velocity gate collapsed on tracking jitter.  Surface taps are now detected by
 * the z-trajectory state machine in decoder_update_surface() below. */

static void
compute_intent_weights_midair (const decoder_t *self,
                               const float fingertips_plane[N_FINGERS][3],
                               const bool  present[N_FINGERS],
                               float       dt,
                               float       out_weight[N_FINGERS])
{
    float raw[N_FINGERS] = {0};
    float max_v = -1e9f;

    for (int f = 0; f < N_FINGERS; ++f) {
        if (!present[f]) { raw[f] = -1e9f; continue; }
        float vel = 0.0f;
        if (self->velocity_warm && self->prev_present[f] && dt > 1e-6f) {
            float dz = fingertips_plane[f][2] - self->prev_xy_z[f][2];
            vel = -dz / dt;
        }
        raw[f] = vel / self->cfg.attention_temperature;
        if (raw[f] > max_v) max_v = raw[f];
    }

    /* Softmax with numerical stability.  Output is 0 if no finger has
     * any meaningful downward velocity. */
    float sum = 0.0f;
    float exps[N_FINGERS] = {0};
    bool any = false;
    for (int f = 0; f < N_FINGERS; ++f) {
        if (raw[f] <= -1e8f) { exps[f] = 0.0f; continue; }
        if (raw[f] <= 0.0f)  { exps[f] = 0.0f; continue; }
        exps[f] = expf (raw[f] - max_v);
        sum += exps[f];
        any = true;
    }
    for (int f = 0; f < N_FINGERS; ++f) {
        out_weight[f] = (any && sum > 0.0f) ? exps[f] / sum : 0.0f;
    }
}

/* ---------------------------------------------------------------- */
/* Per-key spatial likelihood                                        */
/* ---------------------------------------------------------------- */

/* log Σ_finger w_f · N(xy_f; μ_k, Σ_k).  Diagonal Σ.  We compute the
 * sum in linear space then log it (numerically OK because w·N is
 * always small but we sum < 10 terms). */
static float
log_spatial_for_key (int key_idx,
                     const float fingertips_plane[N_FINGERS][3],
                     const float weights[N_FINGERS])
{
    const kbd_key_geom_t *g = kbd_geom_get (key_idx);
    if (!g) return -50.0f;
    const float sx = default_sigma_x (key_idx);
    const float sy = default_sigma_y (key_idx);
    const float two_sx2 = 2.0f * sx * sx;
    const float two_sy2 = 2.0f * sy * sy;
    /* Normalisation constant 1 / (2π σx σy).  Drops out of the
     * argmax for fixed-σ keys but matters when σ varies per key. */
    const float norm = 1.0f / (2.0f * 3.14159265f * sx * sy);

    float sum = 0.0f;
    for (int f = 0; f < N_FINGERS; ++f) {
        if (weights[f] <= 0.0f) continue;
        const float dx = fingertips_plane[f][0] - g->cx_m;
        const float dy = fingertips_plane[f][1] - g->cy_m;
        const float exponent = (dx*dx) / two_sx2 + (dy*dy) / two_sy2;
        sum += weights[f] * norm * expf (-exponent);
    }
    if (sum <= 1e-30f) return -50.0f;
    return logf (sum);
}

/* LM prior for key `k` given the last emitted key index `last`.  Keys the
 * character model cannot speak about (shift, the layer toggle, backspace,
 * return — see lm_is_character) take the neutral prior in both the candidate
 * and the context position, so a control key is decided on geometry against a
 * letter's real prior rather than on smoothing noise. */
static float
lm_prior_for_key (const decoder_t *self, int k, int last)
{
    const uint32_t keysym = kbd_geom_get (k)->keysym;
    if (!lm_is_character (keysym)) return lm_log_prob_neutral ();
    const int idx = lm_keysym_to_idx (keysym);
    if (idx < 0) return lm_log_prob_neutral ();
    if (last < 0) return lm_log_prob_unigram (self->lm, idx);
    const uint32_t last_keysym = kbd_geom_get (last)->keysym;
    if (!lm_is_character (last_keysym))
        return lm_log_prob_unigram (self->lm, idx);
    const int last_idx = lm_keysym_to_idx (last_keysym);
    return (last_idx >= 0) ? lm_log_prob (self->lm, last_idx, idx)
                           : lm_log_prob_unigram (self->lm, idx);
}

/* ---------------------------------------------------------------- */
/* Surface tap detector (z-trajectory local minimum)                 */
/* ---------------------------------------------------------------- */

/* Best key for a press at plane-local `xy`: spatial likelihood fused with the
 * bigram LM.  Returns the key index (-1 if none scored) and, through the
 * optional outs, its score and the runner-up.  `skip_cooldown` ignores the
 * per-key cooldown filter — used to ask which key a press physically LANDED on
 * while that key is still cooling down from the tap before it. */
static int
surface_pick_key (const decoder_t *self, const float xy[2], int K,
                  bool skip_cooldown, float *out_score,
                  int *out_second, float *out_second_score)
{
    float weights[N_FINGERS] = { 0 };
    weights[0] = 1.0f;
    float ft[N_FINGERS][3] = { { 0 } };
    ft[0][0] = xy[0];
    ft[0][1] = xy[1];

    const float alpha = self->cfg.lm_weight_surface;
    const int   last  = self->last_emitted_idx;
    int   best = -1, second = -1;
    float best_score = -1e9f, second_score = -1e9f;
    for (int k = 0; k < K; ++k) {
        if (!skip_cooldown && self->cooldown_s[k] > 0.0f) continue;
        const float log_spat = log_spatial_for_key (k, ft, weights);
        const float log_lm = lm_prior_for_key (self, k, last);
        const float fused = (1.0f - alpha) * log_spat + alpha * log_lm;
        if (fused > best_score) {
            second = best; second_score = best_score;
            best = k; best_score = fused;
        } else if (fused > second_score) {
            second = k; second_score = fused;
        }
    }
    if (out_score)        *out_score        = best_score;
    if (out_second)       *out_second       = second;
    if (out_second_score) *out_second_score = second_score;
    return best;
}

/* A tap (or hold press) fired for finger `f` at plane-local `xy`: pick the best
 * key and commit it (per-key cooldown + LM context).  Returns the keysym, or 0
 * if nothing crossed threshold.  `what` labels the debug trace. */
static uint32_t
surface_emit_tap (decoder_t *self, int f, const float xy[2], int K,
                  float plunge_m, float valley_m, const char *what)
{
    int   second = -1;
    float best_score = -1e9f, second_score = -1e9f;
    const int best = surface_pick_key (self, xy, K, false, &best_score,
                                       &second, &second_score);
    if (best >= 0 && best_score >= self->cfg.emit_threshold) {
        self->cooldown_s[best] = self->cfg.cooldown_s;
        self->last_emitted_idx = best;
        /* Diagnostic: where the tap landed vs which key it picked and the
         * runner-up — reveals systematic offset vs aim scatter vs wrong finger,
         * without fitting to any recorded session.  Gated by the widget's flag. */
        if (getenv ("SPATIAL_KEYBOARD_DEBUG")) {
            const uint32_t bk = kbd_geom_get (best)->keysym;
            const uint32_t sk = (second >= 0) ? kbd_geom_get (second)->keysym : 0;
            fprintf (stderr, "decoder: %s f=%d xy=(%.3f,%.3f) plunge=%.0fmm "
                     "valley=%.0fmm → 0x%04x '%c' (%.1f)  2nd 0x%04x '%c' (%.1f)\n",
                     what, f, (double) xy[0], (double) xy[1],
                     (double) (plunge_m * 1000.0f), (double) (valley_m * 1000.0f), bk,
                     (bk >= 0x20 && bk < 0x7f) ? (char) bk : '?', (double) best_score,
                     sk, (sk >= 0x20 && sk < 0x7f) ? (char) sk : '?',
                     (double) second_score);
        }
        return kbd_geom_get (best)->keysym;
    }
    return 0;
}

/* Per-finger RELATIVE plunge detector — see the TAP_* comment above.  Track z
 * peaks/valleys with hysteresis; a completed plunge (descend ≥ TAP_PLUNGE_M,
 * bottom near the plane, then lift ≥ TAP_PLUNGE_M) fires a tap at the average xy
 * near the valley.  Because the trigger is the RELATIVE plunge and not an
 * absolute depth, it works despite the reconstruction placing the hand an inch
 * high; the lift-off requirement rejects a settle-at-the-bottom phantom, and
 * the near-plane gate rejects an inter-key move. */
/* Mean smoothed z of the OTHER tracked fingers of finger `f`'s hand (0-4 left,
 * 5-9 right).  False when the hand has no other tracked finger. */
static bool
hand_others_mean_z (const float zs[N_FINGERS], const bool tracked[N_FINGERS],
                    int f, float *out_mean)
{
    const int base = (f < 5) ? 0 : 5;
    float sum = 0.0f;
    int   n   = 0;
    for (int o = base; o < base + 5; ++o) {
        if (o == f || !tracked[o]) continue;
        sum += zs[o];
        n++;
    }
    if (n == 0) return false;
    *out_mean = sum / (float) n;
    return true;
}

/* Record a new deepest point for finger `f`, capturing the two things the tap
 * gates read at the valley: when it happened, and how far the finger was
 * extended below the rest of its hand. */
static void
tap_mark_valley (decoder_t *self, int f, float zs,
                 const float zs_now[N_FINGERS], const bool tracked[N_FINGERS])
{
    self->tap_valley[f]   = zs;
    self->tap_valley_t[f] = self->now_s;
    float others = 0.0f;
    self->tap_prom[f] = hand_others_mean_z (zs_now, tracked, f, &others)
                        ? others - zs
                        : INFINITY;   /* no reference finger → gate skipped */
}

static bool
tap_gates_pass (const decoder_t *self, int f, float amp, float valley)
{
    return amp >= self->cfg.tap_plunge_m
           && valley < self->cfg.sigma_z_m
           && self->tap_prom[f] >= self->cfg.tap_prominence_m;
}

/* Queue a tap keysym for release once the refractory expires, ordered by the
 * valley time so a fast two-finger alternation comes out in the order it was
 * typed.  Insertion sort — the queue holds at most a few entries. */
static void
defer_push (decoder_t *self, uint32_t keysym, float valley_t)
{
    if (self->defer_n >= DEFER_MAX) {
        if (getenv ("SPATIAL_KEYBOARD_DEBUG"))
            fprintf (stderr, "decoder: defer queue full, dropping 0x%04x\n", keysym);
        return;
    }
    int i = self->defer_n;
    while (i > 0 && self->defer_valley_t[i - 1] > valley_t) {
        self->defer_keysym[i]   = self->defer_keysym[i - 1];
        self->defer_valley_t[i] = self->defer_valley_t[i - 1];
        --i;
    }
    self->defer_keysym[i]   = keysym;
    self->defer_valley_t[i] = valley_t;
    self->defer_n++;
}

static uint32_t
defer_pop (decoder_t *self)
{
    const uint32_t k = self->defer_keysym[0];
    for (int i = 1; i < self->defer_n; ++i) {
        self->defer_keysym[i - 1]   = self->defer_keysym[i];
        self->defer_valley_t[i - 1] = self->defer_valley_t[i];
    }
    self->defer_n--;
    return k;
}

/* A tap that completed while the previous key was still inside the emit
 * refractory.  Dropping it loses a genuine fast two-finger alternation, so it
 * is scored now and queued — EXCEPT when it landed on the key just emitted,
 * which is the resting-thumb double bounce the refractory exists to swallow. */
static void
surface_defer_tap (decoder_t *self, int f, const float xy[2], int K,
                   float amp, float valley)
{
    const int landed = surface_pick_key (self, xy, K, true, NULL, NULL, NULL);
    if (landed >= 0 && landed == self->last_emitted_idx) return;
    uint32_t e = surface_emit_tap (self, f, xy, K, amp, valley, "defer");
    if (e) defer_push (self, e, self->tap_valley_t[f]);
}

static void
hold_end (decoder_t *self, decoder_key_event_t *ev)
{
    if (ev) {
        ev->kind   = DECODER_KEY_EVENT_HOLD_END;
        ev->keysym = kbd_geom_get (self->hold_key_idx)->keysym;
    }
    self->cooldown_s[self->hold_key_idx] = self->cfg.cooldown_s;
    self->emit_refractory_s = self->cfg.emit_refractory_s;
    self->hold_active = false;
}

static uint32_t
decoder_update_surface (decoder_t *self,
                        const float fingertips_plane[N_FINGERS][3],
                        const bool  present[N_FINGERS],
                        float       dt_seconds,
                        decoder_key_event_t *ev)
{
    const int K = kbd_geom_key_count ();
    if (K <= 0 || K > MAX_KEYS) return 0;

    for (int k = 0; k < K; ++k) {
        if (self->cooldown_s[k] > 0.0f) {
            self->cooldown_s[k] -= dt_seconds;
            if (self->cooldown_s[k] < 0.0f) self->cooldown_s[k] = 0.0f;
        }
    }
    if (self->emit_refractory_s > 0.0f) {
        self->emit_refractory_s -= dt_seconds;
        if (self->emit_refractory_s < 0.0f) self->emit_refractory_s = 0.0f;
    }
    self->now_s += dt_seconds;

    uint32_t emitted = 0;
    if (self->defer_n > 0 && self->emit_refractory_s <= 0.0f) {
        emitted = defer_pop (self);
        self->emit_refractory_s = self->cfg.emit_refractory_s;
        if (ev && ev->kind == DECODER_KEY_EVENT_NONE) {
            ev->kind   = DECODER_KEY_EVENT_TAP;
            ev->keysym = emitted;
        }
    }

    /* Smooth every tracked finger's z FIRST: the prominence gate compares a
     * finger against the rest of its hand in the same frame, so all ten values
     * must come from the same tick. */
    float zs_now[N_FINGERS]  = { 0 };
    bool  appeared[N_FINGERS] = { false };
    for (int f = 0; f < N_FINGERS; ++f) {
        if (!present[f]) continue;
        const float z = fingertips_plane[f][2];
        if (!self->prev_present[f]) {
            self->tap_zs[f] = z;   /* seed the smoother on (re)appearance */
            appeared[f]     = true;
        } else {
            self->tap_zs[f] = self->cfg.tap_z_smooth_a * z
                              + (1.0f - self->cfg.tap_z_smooth_a) * self->tap_zs[f];
        }
        zs_now[f] = self->tap_zs[f];
    }

    for (int f = 0; f < N_FINGERS; ++f) {
        if (!present[f]) {
            if (self->hold_active && self->hold_finger == f)
                hold_end (self, ev);   /* lost mid-hold: release, don't stick */
            self->prev_present[f] = false;
            continue;
        }
        if (appeared[f]) {
            self->tap_descending[f] = false;
            self->tap_peak[f]       = zs_now[f];
            self->tap_valley[f]     = zs_now[f];
            self->tap_prom[f]       = INFINITY;
            self->hold_bottom_dwell_s[f] = 0.0f;
            self->prev_present[f]   = true;
            continue;
        }
        const float zs = zs_now[f];

        if (!self->tap_descending[f]) {
            /* Ascending / hovering: track the peak.  A fall of one hysteresis
             * below it starts a plunge (reset the xy accumulator). */
            if (zs > self->tap_peak[f]) self->tap_peak[f] = zs;
            if (zs < self->tap_peak[f] - self->cfg.tap_hyst_m) {
                self->tap_descending[f] = true;
                tap_mark_valley (self, f, zs, zs_now, present);
                self->tap_xy_sum[f][0]  = 0.0f;
                self->tap_xy_sum[f][1]  = 0.0f;
                self->tap_xy_n[f]       = 0;
                self->hold_bottom_dwell_s[f] = 0.0f;
            }
        }
        if (self->tap_descending[f]) {
            if (zs < self->tap_valley[f])
                tap_mark_valley (self, f, zs, zs_now, present);
            /* Average the xy only near the bottom (on the key), not the whole
             * up-swing where the finger is already leaving toward the next key. */
            if (zs <= self->tap_valley[f] + self->cfg.tap_hyst_m) {
                self->tap_xy_sum[f][0] += fingertips_plane[f][0];
                self->tap_xy_sum[f][1] += fingertips_plane[f][1];
                self->tap_xy_n[f]++;
                /* Dwell counts only LOW-Z bottom time (a still-falling zs
                 * trivially equals the running valley, so gate on the plane
                 * too — the descent leg is not "pressed"). */
                if (zs < self->cfg.sigma_z_m)
                    self->hold_bottom_dwell_s[f] += dt_seconds;
            } else {
                self->hold_bottom_dwell_s[f] = 0.0f;
            }

            /* A press that DWELLS at the bottom instead of lifting is a held
             * key: same amplitude + near-plane gates as a tap, with the lift
             * requirement replaced by a sustained low-z dwell.  Fires the
             * press (HOLD_BEGIN) now; the release arrives on lift. */
            if (!self->hold_active
                && self->hold_bottom_dwell_s[f] >= self->cfg.hold_dwell_s
                && tap_gates_pass (self, f, self->tap_peak[f] - self->tap_valley[f],
                                   self->tap_valley[f])
                && self->tap_xy_n[f] > 0
                && self->emit_refractory_s <= 0.0f && !emitted) {
                const float avg[2] = {
                    self->tap_xy_sum[f][0] / (float) self->tap_xy_n[f],
                    self->tap_xy_sum[f][1] / (float) self->tap_xy_n[f],
                };
                uint32_t e = surface_emit_tap (self, f, avg, K,
                                               self->tap_peak[f] - self->tap_valley[f],
                                               self->tap_valley[f], "hold");
                if (e) {
                    self->hold_active   = true;
                    self->hold_key_idx  = self->last_emitted_idx;
                    self->hold_finger   = f;
                    self->emit_refractory_s = self->cfg.emit_refractory_s;
                    if (ev) {
                        ev->kind   = DECODER_KEY_EVENT_HOLD_BEGIN;
                        ev->keysym = e;
                    }
                } else {
                    /* Nothing crossed threshold — re-arm after another dwell
                     * instead of retrying every frame. */
                    self->hold_bottom_dwell_s[f] = 0.0f;
                }
            }

            if (zs > self->tap_valley[f] + self->cfg.tap_plunge_m) {
                /* Confirmed lift-off (a real release, not settle jitter). */
                const float amp    = self->tap_peak[f] - self->tap_valley[f];
                const float valley = self->tap_valley[f];
                const bool  is_tap = tap_gates_pass (self, f, amp, valley);
                if (self->hold_active && self->hold_finger == f) {
                    hold_end (self, ev);   /* release of the held key, not a tap */
                } else if (is_tap && self->tap_xy_n[f] > 0) {
                    const float avg[2] = {
                        self->tap_xy_sum[f][0] / (float) self->tap_xy_n[f],
                        self->tap_xy_sum[f][1] / (float) self->tap_xy_n[f],
                    };
                    if (self->emit_refractory_s <= 0.0f && !emitted) {
                        uint32_t e = surface_emit_tap (self, f, avg, K, amp, valley, "tap");
                        if (e) {
                            emitted = e;
                            self->emit_refractory_s = self->cfg.emit_refractory_s;
                            if (ev && ev->kind == DECODER_KEY_EVENT_NONE) {
                                ev->kind   = DECODER_KEY_EVENT_TAP;
                                ev->keysym = e;
                            }
                        }
                    } else {
                        surface_defer_tap (self, f, avg, K, amp, valley);
                    }
                } else if (getenv ("SPATIAL_KEYBOARD_DEBUG") && amp >= self->cfg.tap_plunge_m) {
                    /* A real plunge that wasn't taken — say why (helps tuning). */
                    const char *why =
                        (valley >= self->cfg.sigma_z_m) ? "move (bottomed too high)"
                        : (self->tap_prom[f] < self->cfg.tap_prominence_m)
                            ? "move (fingers together)"
                            : "suppressed";
                    fprintf (stderr, "decoder: plunge f=%d amp=%.0fmm valley=%.0fmm"
                             " prom=%.0fmm → %s\n", f, (double) (amp * 1000.0f),
                             (double) (valley * 1000.0f),
                             (double) (self->tap_prom[f] * 1000.0f), why);
                }
                self->tap_descending[f] = false;
                self->tap_peak[f]       = zs;   /* start the next hover here */
                self->hold_bottom_dwell_s[f] = 0.0f;
            }
        }
        self->prev_present[f] = true;
    }
    return emitted;
}

/* ---------------------------------------------------------------- */
/* Mid-air update (unchanged velocity-attention + EMA + peak)        */
/* ---------------------------------------------------------------- */

static uint32_t
decoder_update_midair (decoder_t *self,
                       const float fingertips_plane[N_FINGERS][3],
                       const bool  present[N_FINGERS],
                       float       dt_seconds)
{
    const int K = kbd_geom_key_count ();
    if (K <= 0 || K > MAX_KEYS) return 0;

    /* 1. Per-finger intent weights (mid-air velocity attention). */
    float weight[N_FINGERS];
    compute_intent_weights_midair (self, fingertips_plane, present,
                                   dt_seconds, weight);

    /* 2. + 3. Per-key fused log-score. */
    const float alpha = self->cfg.lm_weight_midair;
    const int last = self->last_emitted_idx;

    /* 4. EMA-smooth.  Update cooldowns. */
    int new_argmax = -1;
    float new_argmax_score = -1e9f;
    for (int k = 0; k < K; ++k) {
        if (self->cooldown_s[k] > 0.0f) {
            self->cooldown_s[k] -= dt_seconds;
            if (self->cooldown_s[k] < 0.0f) self->cooldown_s[k] = 0.0f;
        }
        const float log_spat = log_spatial_for_key (k, fingertips_plane, weight);
        const float log_lm = lm_prior_for_key (self, k, last);
        const float log_fused = (1.0f - alpha) * log_spat + alpha * log_lm;

        /* EMA smoothing in log-space. */
        self->smoothed[k] = self->cfg.ema_alpha * log_fused
                            + (1.0f - self->cfg.ema_alpha) * self->smoothed[k];

        if (self->smoothed[k] > new_argmax_score
            && self->cooldown_s[k] <= 0.0f) {
            new_argmax_score = self->smoothed[k];
            new_argmax = k;
        }
    }

    /* 5. Peak detection on argmax. */
    uint32_t emitted = 0;
    if (new_argmax >= 0) {
        if (new_argmax == self->argmax_idx) {
            if (new_argmax_score < self->argmax_prev) {
                self->argmax_falling_frames++;
            } else {
                self->argmax_falling_frames = 0;
                if (new_argmax_score > self->argmax_peak_score)
                    self->argmax_peak_score = new_argmax_score;
            }
        } else {
            self->argmax_idx = new_argmax;
            self->argmax_falling_frames = 0;
            self->argmax_peak_score = new_argmax_score;
        }
        self->argmax_prev = new_argmax_score;

        if (self->argmax_falling_frames >= self->cfg.peak_holdoff_frames
            && self->argmax_peak_score >= self->cfg.emit_threshold)
        {
            const kbd_key_geom_t *g = kbd_geom_get (self->argmax_idx);
            emitted = g ? g->keysym : 0;
            if (emitted) {
                self->cooldown_s[self->argmax_idx] = self->cfg.cooldown_s;
                self->last_emitted_idx = self->argmax_idx;
                self->argmax_falling_frames = 0;
                self->argmax_peak_score = -1e9f;
            }
        }
    }

    /* Cache positions for next frame velocity. */
    for (int f = 0; f < N_FINGERS; ++f) {
        if (present[f]) {
            self->prev_xy_z[f][0] = fingertips_plane[f][0];
            self->prev_xy_z[f][1] = fingertips_plane[f][1];
            self->prev_xy_z[f][2] = fingertips_plane[f][2];
        }
        self->prev_present[f] = present[f];
    }
    self->velocity_warm = true;
    return emitted;
}

uint32_t
decoder_update_ev (decoder_t *self,
                   const float fingertips_plane[N_FINGERS][3],
                   const bool  present[N_FINGERS],
                   float       dt_seconds,
                   decoder_key_event_t *out_event)
{
    if (out_event) {
        out_event->kind   = DECODER_KEY_EVENT_NONE;
        out_event->keysym = 0;
    }
    if (!self) return 0;
    if (self->mode == DECODER_MODE_SURFACE)
        return decoder_update_surface (self, fingertips_plane, present,
                                       dt_seconds, out_event);
    if (self->hold_active)
        hold_end (self, out_event);   /* mode switched mid-hold: release */
    self->defer_n = 0;   /* queued surface taps are stale once mid-air owns input */
    uint32_t k = decoder_update_midair (self, fingertips_plane, present,
                                        dt_seconds);
    if (k && out_event && out_event->kind == DECODER_KEY_EVENT_NONE) {
        out_event->kind   = DECODER_KEY_EVENT_TAP;
        out_event->keysym = k;
    }
    return k;
}

uint32_t
decoder_update (decoder_t *self,
                const float fingertips_plane[N_FINGERS][3],
                const bool  present[N_FINGERS],
                float       dt_seconds)
{
    return decoder_update_ev (self, fingertips_plane, present, dt_seconds,
                              NULL);
}

float
decoder_last_key_score (const decoder_t *self, int key_idx)
{
    if (!self || key_idx < 0 || key_idx >= MAX_KEYS) return -50.0f;
    return self->smoothed[key_idx];
}

int
decoder_last_argmax (const decoder_t *self)
{
    return self ? self->argmax_idx : -1;
}
