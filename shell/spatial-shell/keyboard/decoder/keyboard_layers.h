/* keyboard_layers.h — sticky-shift + layer-toggle state machine sitting
 * between the decoder and the wire.
 *
 * The decoder always emits LETTERS-layer keysyms: its per-key Gaussians are
 * keyed on the physical rectangle, not on what the rectangle currently means.
 * This module turns one of those into the character the user actually asked
 * for, and swallows the taps that only moved modifier state.
 *
 * Shift cycles OFF → ONCE → CAPS → OFF.  There is deliberately no wall-clock
 * double-tap window: one plunge tap occupies ~0.4 s of the pose stream, so a
 * physical double tap lands well outside any window short enough to also mean
 * "cancel", and a timer would make caps lock a coin flip.  Two consecutive
 * shift taps therefore always lock caps, and a third clears.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SPATIAL_KEYBOARD_DECODER_KEYBOARD_LAYERS_H
#define SPATIAL_KEYBOARD_DECODER_KEYBOARD_LAYERS_H

#include <stdbool.h>
#include <stdint.h>

#include "keyboard_geom.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    kbd_layer_t layer;
    kbd_shift_t shift;
} kbd_layer_state_t;

typedef struct {
    bool        emit;           /* false: the tap only moved modifier state */
    bool        state_changed;  /* layer/shift moved — repaint the board */
    uint32_t    keysym;         /* what to put on the wire (emit only) */
    const char *label;          /* that key's current label (emit only) */
} kbd_layer_out_t;

void kbd_layer_state_reset (kbd_layer_state_t *st);

/* Feed one decoder keysym through the current state.  Never NULL-tolerant on
 * `out`; `st` NULL is a no-op that emits `base_keysym` unchanged. */
void kbd_layer_apply (kbd_layer_state_t *st, uint32_t base_keysym,
                      kbd_layer_out_t *out);

#ifdef __cplusplus
}
#endif

#endif
