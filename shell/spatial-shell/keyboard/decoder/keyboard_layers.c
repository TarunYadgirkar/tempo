/* keyboard_layers.c — see keyboard_layers.h. */

#include "keyboard_layers.h"

#include <stddef.h>

void
kbd_layer_state_reset (kbd_layer_state_t *st)
{
    if (!st) return;
    st->layer = KBD_LAYER_LETTERS;
    st->shift = KBD_SHIFT_OFF;
}

void
kbd_layer_apply (kbd_layer_state_t *st, uint32_t base_keysym,
                 kbd_layer_out_t *out)
{
    if (!out) return;
    out->emit = true;
    out->state_changed = false;
    out->keysym = base_keysym;
    out->label = "?";
    if (!st) return;

    /* The shift rectangle only means shift on the letters layer; on the
     * symbols layer it is the '=' key (see keyboard_geom.c). */
    if (base_keysym == XK_Shift_L && st->layer == KBD_LAYER_LETTERS) {
        st->shift = st->shift == KBD_SHIFT_OFF  ? KBD_SHIFT_ONCE
                  : st->shift == KBD_SHIFT_ONCE ? KBD_SHIFT_CAPS
                                                : KBD_SHIFT_OFF;
        out->emit = false;
        out->state_changed = true;
        return;
    }

    if (base_keysym == XK_Mode_switch) {
        st->layer = st->layer == KBD_LAYER_LETTERS ? KBD_LAYER_SYMBOLS
                                                   : KBD_LAYER_LETTERS;
        if (st->shift == KBD_SHIFT_ONCE) st->shift = KBD_SHIFT_OFF;
        out->emit = false;
        out->state_changed = true;
        return;
    }

    const int idx = kbd_geom_find_keysym (base_keysym);
    if (idx < 0) return;

    out->keysym = kbd_geom_keysym_at (idx, st->layer, st->shift);
    out->label  = kbd_geom_label_at (idx, st->layer, st->shift);

    if (st->layer == KBD_LAYER_LETTERS && st->shift == KBD_SHIFT_ONCE) {
        st->shift = KBD_SHIFT_OFF;
        out->state_changed = true;
    }
}
