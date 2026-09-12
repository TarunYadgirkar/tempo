/* keyboard_geom.h — geometric layout of the QWERTY keyboard in
 * plane-local 2D coordinates (meters).
 *
 * The decoder's spatial model needs per-key (center, half-extent)
 * tuples to initialise the per-key 2D Gaussians.  This module owns
 * that layout, kept in lockstep with keyboard_layout.c and
 * keyboard_widget.c.  All three derive from the same row/column
 * structure; future refactors should consolidate to a single source.
 *
 * Two character LAYERS share one grid of rectangles: letters (default)
 * and symbols/digits.  Only the keysym and the label differ per layer,
 * so every per-key Gaussian is layer-invariant and the spatial model
 * fitted for letters is exactly the model the symbols layer uses.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SPATIAL_KEYBOARD_DECODER_KEYBOARD_GEOM_H
#define SPATIAL_KEYBOARD_DECODER_KEYBOARD_GEOM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* X11 keysyms for the keys that are not plain characters.  Copied
 * verbatim from <X11/keysymdef.h> so nothing here needs libx11-dev. */
#define XK_BackSpace     0xff08
#define XK_Return        0xff0d
#define XK_Mode_switch   0xff7e
#define XK_Shift_L       0xffe1

typedef struct {
    uint32_t keysym;     /* X11 keysym (XK_a, XK_space, …) — LETTERS layer */
    const char *label;   /* LETTERS layer, unshifted */
    float cx_m, cy_m;    /* key center in plane-local meters */
    float hw_m, hh_m;    /* half-width / half-height (for default σ) */
} kbd_key_geom_t;

typedef enum {
    KBD_LAYER_LETTERS = 0,
    KBD_LAYER_SYMBOLS = 1,
} kbd_layer_t;

typedef enum {
    KBD_SHIFT_OFF  = 0,
    KBD_SHIFT_ONCE = 1,   /* sticky one-shot: consumed by the next character */
    KBD_SHIFT_CAPS = 2,   /* caps lock: held until shift is tapped again */
} kbd_shift_t;

/* Standard keyboard surface size (m).  Matches the visible widget
 * roughly — ~30 cm wide × ~12 cm tall, the same scale TouchInsight
 * and the iPhone soft-keyboard target. */
#define KBD_GEOM_WIDTH_M  0.30f
#define KBD_GEOM_HEIGHT_M 0.12f

/* Returns the count of keys in the canonical full QWERTY layout
 * (letters + space + return + backspace + shift + layer toggle). */
int kbd_geom_key_count (void);

/* Returns a pointer to the i-th key's geom (0 ≤ i < count).  Static
 * data; do not free.  Returns NULL on out-of-range. */
const kbd_key_geom_t *kbd_geom_get (int i);

/* Looks up the geom for a keysym; returns -1 if not found, else the
 * index into the layout.  Keysyms are the LETTERS-layer ones — the
 * decoder only ever deals in those. */
int kbd_geom_find_keysym (uint32_t keysym);

/* The keysym / label key `i` carries under (layer, shift).  keysym 0 and
 * label "" for an out-of-range index. */
uint32_t    kbd_geom_keysym_at (int i, kbd_layer_t layer, kbd_shift_t shift);
const char *kbd_geom_label_at  (int i, kbd_layer_t layer, kbd_shift_t shift);

#ifdef __cplusplus
}
#endif

#endif
