/* decoder.h — continuous pose-stream typing decoder.
 *
 * Architecture (per project memory project-typing-decoder-continuous):
 *   per frame, per finger:
 *     intent_weight = proximity_weight × downward_velocity_weight
 *                                              (surface mode)
 *     intent_weight = softmax(downward_velocity / T)
 *                                              (mid-air mode)
 *   per frame, per key:
 *     P(key | frame) = Σ_finger intent_weight[finger]
 *                              × N(plane_xy[finger]; μ_K, Σ_K)
 *   temporal:
 *     accumulate per-key probability over a short window; emit the
 *     argmax-key when its smoothed probability peaks above threshold,
 *     then start the per-key cooldown so a single tap doesn't fire
 *     repeatedly.
 *   LM rescore:
 *     P(key | frame) is fused with the LM bigram prior P(key | last
 *     emitted) before peak detection, so unlikely sequences are
 *     suppressed (h → q, etc.) and likely ones boosted (q → u).
 *
 * NO discrete tap events.  No CTC trellis or beam search in v1 — the
 * MVP uses argmax-on-smoothed-probability with cooldown.  Both can
 * be swapped in by the agent loop without changing this API.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SPATIAL_KEYBOARD_DECODER_DECODER_H
#define SPATIAL_KEYBOARD_DECODER_DECODER_H

#include <stdbool.h>
#include <stdint.h>

#include "decoder_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DECODER_MODE_SURFACE = 0,
    DECODER_MODE_MIDAIR  = 1,
} decoder_mode_t;

typedef struct decoder_t decoder_t;

/* Tunables start at compiled defaults, then overrides from
 * ~/.config/spatial-os/keyboard.toml are merged on top (see
 * decoder_config.h).  decoder_create uses the default XDG path;
 * decoder_create_with_config takes an explicit TOML path (tests,
 * eval sweeps) — NULL/empty means the XDG default. */
decoder_t *decoder_create (decoder_mode_t initial_mode);
decoder_t *decoder_create_with_config (decoder_mode_t initial_mode,
                                       const char *toml_path);
void       decoder_destroy (decoder_t *self);

const decoder_config_t *decoder_get_config (const decoder_t *self);

void decoder_set_mode (decoder_t *self, decoder_mode_t mode);

/* Per-frame update.  fingertips_plane is an array of 10 entries
 * (left hand 5 fingers, then right hand 5 fingers) holding each
 * fingertip's position in PLANE-LOCAL coordinates:
 *   x: meters, 0 = left edge of the keyboard surface
 *   y: meters, 0 = top edge of the keyboard surface
 *   z: meters, signed distance ABOVE the plane (positive = above,
 *      negative = below — i.e., user's finger pushed through)
 * dt_seconds is the time since the previous decoder_update() call
 * (used to compute velocity).
 *
 * present[10] flags which fingertips have fresh data this frame —
 * absent fingertips contribute zero weight.
 *
 * Returns an X11 keysym (XK_a, XK_space, …) if a key was emitted
 * this frame, or 0 otherwise.  Repeated calls always return at most
 * one keysym per frame; if two fingers simultaneously tap distinct
 * keys, only the higher-weighted one fires (the other is suppressed
 * by cooldown). */
uint32_t decoder_update (decoder_t *self,
                         const float fingertips_plane[10][3],
                         const bool  present[10],
                         float       dt_seconds);

/* Held-key events (surface mode).  A tap that stays pressed at the
 * plunge bottom for cfg.hold_dwell_s becomes a HOLD: HOLD_BEGIN is the
 * press (no immediate release); HOLD_END fires when the finger lifts
 * (or is lost).  The tap path is unchanged and reports as a single
 * TAP event.  The widget turns HOLD_BEGIN…HOLD_END into key repeats. */
typedef enum {
    DECODER_KEY_EVENT_NONE = 0,
    DECODER_KEY_EVENT_TAP,
    DECODER_KEY_EVENT_HOLD_BEGIN,
    DECODER_KEY_EVENT_HOLD_END,
} decoder_key_event_kind_t;

typedef struct {
    decoder_key_event_kind_t kind;
    uint32_t                 keysym;
} decoder_key_event_t;

/* Like decoder_update, but also reports the richest key event of the
 * frame in *out_event (may be NULL).  The return value stays the tap
 * keysym (0 for hold events), so decoder_update == decoder_update_ev
 * with out_event dropped. */
uint32_t decoder_update_ev (decoder_t *self,
                            const float fingertips_plane[10][3],
                            const bool  present[10],
                            float       dt_seconds,
                            decoder_key_event_t *out_event);

/* Debug accessors (for eval_typing and unit tests). */
float decoder_last_key_score (const decoder_t *self, int key_idx);
int   decoder_last_argmax    (const decoder_t *self);

#ifdef __cplusplus
}
#endif

#endif
