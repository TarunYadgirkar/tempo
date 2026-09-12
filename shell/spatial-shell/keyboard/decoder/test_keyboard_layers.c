/* test_keyboard_layers.c — shift / symbols-layer plumbing.
 *
 * Two halves:
 *   (a) the pure state machine (kbd_layer_apply) — one-shot shift, caps lock,
 *       layer toggle, and the per-layer key map;
 *   (b) an end-to-end pass where synthetic plunge taps run through the real
 *       decoder and the state machine, typing a URL that needs digits,
 *       punctuation and both layers.
 *
 * No thresholds or σ are touched here: taps land on key centres and the
 * assertions are on the character that comes out.
 *
 * SPDX-License-Identifier: MIT
 */

#include "decoder.h"
#include "kbd_tap_traj.h"
#include "keyboard_geom.h"
#include "keyboard_layers.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define DT (1.0f / 60.0f)
#define N_FINGERS 10
#define XK_space 0x0020

/* --- (a) state machine ------------------------------------------- */

static uint32_t
apply (kbd_layer_state_t *st, uint32_t base)
{
    kbd_layer_out_t out;
    kbd_layer_apply (st, base, &out);
    return out.emit ? out.keysym : 0;
}

static void
test_shift_one_shot (void)
{
    kbd_layer_state_t st;
    kbd_layer_state_reset (&st);

    kbd_layer_out_t out;
    kbd_layer_apply (&st, XK_Shift_L, &out);
    assert (!out.emit && out.state_changed);
    assert (st.shift == KBD_SHIFT_ONCE);

    assert (apply (&st, 'a') == 'A');
    assert (st.shift == KBD_SHIFT_OFF);
    assert (apply (&st, 'b') == 'b');
    printf ("PASS test_shift_one_shot\n");
}

static void
test_shift_double_tap_caps_lock (void)
{
    kbd_layer_state_t st;
    kbd_layer_state_reset (&st);

    apply (&st, XK_Shift_L);
    apply (&st, XK_Shift_L);
    assert (st.shift == KBD_SHIFT_CAPS);

    assert (apply (&st, 'a') == 'A');
    assert (apply (&st, 'b') == 'B');
    assert (apply (&st, 'z') == 'Z');
    assert (st.shift == KBD_SHIFT_CAPS);

    apply (&st, XK_Shift_L);   /* tapped again → released */
    assert (st.shift == KBD_SHIFT_OFF);
    assert (apply (&st, 'a') == 'a');
    printf ("PASS test_shift_double_tap_caps_lock\n");
}

static void
test_shift_leaves_specials_alone (void)
{
    kbd_layer_state_t st;
    kbd_layer_state_reset (&st);
    apply (&st, XK_Shift_L);
    assert (apply (&st, XK_space) == XK_space);
    assert (st.shift == KBD_SHIFT_OFF);   /* one-shot consumed by any key */
    apply (&st, XK_Shift_L);
    assert (apply (&st, XK_BackSpace) == XK_BackSpace);
    printf ("PASS test_shift_leaves_specials_alone\n");
}

static void
test_layer_toggle_maps_and_restores (void)
{
    kbd_layer_state_t st;
    kbd_layer_state_reset (&st);

    kbd_layer_out_t out;
    kbd_layer_apply (&st, XK_Mode_switch, &out);
    assert (!out.emit && out.state_changed);
    assert (st.layer == KBD_LAYER_SYMBOLS);

    /* The former "q" rectangle is the "1" key. */
    assert (apply (&st, 'q') == '1');
    assert (apply (&st, 'p') == '0');
    /* The former shift rectangle is '=' — no shift to modify here. */
    assert (apply (&st, XK_Shift_L) == '=');
    /* Layer-invariant keys keep their meaning. */
    assert (apply (&st, XK_space) == XK_space);
    assert (apply (&st, XK_BackSpace) == XK_BackSpace);

    kbd_layer_apply (&st, XK_Mode_switch, &out);
    assert (st.layer == KBD_LAYER_LETTERS);
    assert (apply (&st, 'q') == 'q');
    printf ("PASS test_layer_toggle_maps_and_restores\n");
}

/* Every character the product brief needs for URLs, email and chat has to be
 * reachable in at most one layer toggle plus one tap. */
static void
test_required_characters_are_reachable (void)
{
    const char *required = "0123456789.,?!'\":;/-_@#()=";
    for (const char *c = required; *c; ++c) {
        bool found = false;
        for (int layer = 0; layer < 2 && !found; ++layer) {
            for (int i = 0; i < kbd_geom_key_count (); ++i) {
                if (kbd_geom_keysym_at (i, (kbd_layer_t) layer,
                                        KBD_SHIFT_OFF) == (uint32_t) *c) {
                    found = true;
                    break;
                }
            }
        }
        if (!found) printf ("  MISSING '%c'\n", *c);
        assert (found);
    }
    /* And every capital, via shift on the letters layer. */
    for (char c = 'A'; c <= 'Z'; ++c) {
        int i = kbd_geom_find_keysym ((uint32_t) (c + ('a' - 'A')));
        assert (i >= 0);
        assert (kbd_geom_keysym_at (i, KBD_LAYER_LETTERS, KBD_SHIFT_ONCE)
                == (uint32_t) c);
    }
    printf ("PASS test_required_characters_are_reachable\n");
}

static void
test_labels_follow_layer_and_shift (void)
{
    const int q = kbd_geom_find_keysym ('q');
    const int sh = kbd_geom_find_keysym (XK_Shift_L);
    const int mo = kbd_geom_find_keysym (XK_Mode_switch);
    assert (q >= 0 && sh >= 0 && mo >= 0);

    assert (strcmp (kbd_geom_label_at (q, KBD_LAYER_LETTERS, KBD_SHIFT_OFF),
                    "q") == 0);
    assert (strcmp (kbd_geom_label_at (q, KBD_LAYER_LETTERS, KBD_SHIFT_ONCE),
                    "Q") == 0);
    assert (strcmp (kbd_geom_label_at (q, KBD_LAYER_SYMBOLS, KBD_SHIFT_OFF),
                    "1") == 0);
    assert (strcmp (kbd_geom_label_at (sh, KBD_LAYER_LETTERS, KBD_SHIFT_OFF),
                    "shift") == 0);
    assert (strcmp (kbd_geom_label_at (sh, KBD_LAYER_LETTERS, KBD_SHIFT_ONCE),
                    "SHIFT") == 0);
    assert (strcmp (kbd_geom_label_at (sh, KBD_LAYER_LETTERS, KBD_SHIFT_CAPS),
                    "CAPS") == 0);
    assert (strcmp (kbd_geom_label_at (sh, KBD_LAYER_SYMBOLS, KBD_SHIFT_OFF),
                    "=") == 0);
    assert (strcmp (kbd_geom_label_at (mo, KBD_LAYER_LETTERS, KBD_SHIFT_OFF),
                    "123") == 0);
    assert (strcmp (kbd_geom_label_at (mo, KBD_LAYER_SYMBOLS, KBD_SHIFT_OFF),
                    "abc") == 0);
    printf ("PASS test_labels_follow_layer_and_shift\n");
}

/* --- (b) through the real decoder --------------------------------- */

static void
park (float ft[N_FINGERS][3], bool present[N_FINGERS])
{
    for (int f = 0; f < N_FINGERS; ++f) {
        present[f] = true;
        ft[f][0] = KBD_TAP_PARK_X;
        ft[f][1] = KBD_TAP_PARK_Y;
        ft[f][2] = KBD_TAP_PARK_Z;
    }
}

/* One canonical plunge tap of the left index over key index `key_idx`, plus a
 * parked tail so the lift completes.  Returns the decoder keysym (0 if none). */
static uint32_t
tap_key (decoder_t *dec, int key_idx)
{
    const kbd_key_geom_t *g = kbd_geom_get (key_idx);
    assert (g);
    float ft[N_FINGERS][3];
    bool present[N_FINGERS];
    uint32_t first = 0;
    for (int frame = 0; frame < KBD_TAP_TOTAL_FRAMES; ++frame) {
        park (ft, present);
        ft[1][0] = g->cx_m;
        ft[1][1] = g->cy_m;
        ft[1][2] = kbd_tap_traj_z (frame);
        uint32_t e = decoder_update (dec, ft, present, DT);
        if (e && !first) first = e;
    }
    for (int frame = 0; frame < 14; ++frame) {
        park (ft, present);
        uint32_t e = decoder_update (dec, ft, present, DT);
        if (e && !first) first = e;
    }
    return first;
}

/* Index of the rectangle that carries `ch` on `layer`. */
static int
key_index_for (char ch, kbd_layer_t layer)
{
    for (int i = 0; i < kbd_geom_key_count (); ++i)
        if (kbd_geom_keysym_at (i, layer, KBD_SHIFT_OFF) == (uint32_t) ch)
            return i;
    return -1;
}

/* Type `text` the way a user would: tap the layer key whenever the next
 * character lives on the other layer, then tap the character.  Appends every
 * emitted character to `out`. */
static void
type_text (decoder_t *dec, kbd_layer_state_t *st, const char *text,
           char *out, size_t out_cap)
{
    size_t n = 0;
    for (const char *c = text; *c; ++c) {
        if (key_index_for (*c, st->layer) < 0) {
            uint32_t k = tap_key (dec, kbd_geom_find_keysym (XK_Mode_switch));
            if (k != XK_Mode_switch) {
                printf ("  layer tap decoded as 0x%04x\n", k);
                assert (k == XK_Mode_switch);
            }
            kbd_layer_out_t lo;
            kbd_layer_apply (st, k, &lo);
            assert (!lo.emit);
        }
        const int idx = key_index_for (*c, st->layer);
        assert (idx >= 0);
        uint32_t base = tap_key (dec, idx);
        kbd_layer_out_t lo;
        kbd_layer_apply (st, base, &lo);
        if (lo.emit && n + 1 < out_cap) out[n++] = (char) lo.keysym;
    }
    out[n] = '\0';
}

static void
test_url_round_trip (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    assert (dec);
    kbd_layer_state_t st;
    kbd_layer_state_reset (&st);

    const char *url = "http://a.b/c?d=1";
    char got[64];
    type_text (dec, &st, url, got, sizeof (got));
    if (strcmp (got, url) != 0)
        printf ("  FAIL want '%s' got '%s'\n", url, got);
    assert (strcmp (got, url) == 0);
    printf ("PASS test_url_round_trip ('%s')\n", got);
    decoder_destroy (dec);
}

static void
test_shifted_word_round_trip (void)
{
    decoder_t *dec = decoder_create (DECODER_MODE_SURFACE);
    kbd_layer_state_t st;
    kbd_layer_state_reset (&st);

    /* shift → "H", then "i": the one-shot must not stick. */
    uint32_t k = tap_key (dec, kbd_geom_find_keysym (XK_Shift_L));
    assert (k == XK_Shift_L);
    kbd_layer_out_t lo;
    kbd_layer_apply (&st, k, &lo);
    assert (!lo.emit && st.shift == KBD_SHIFT_ONCE);

    char got[8];
    type_text (dec, &st, "hi", got, sizeof (got));
    if (strcmp (got, "Hi") != 0) printf ("  FAIL got '%s'\n", got);
    assert (strcmp (got, "Hi") == 0);
    printf ("PASS test_shifted_word_round_trip\n");
    decoder_destroy (dec);
}

int
main (void)
{
    /* Hermetic XDG config: a host keyboard.toml must not skew the
     * compiled-default assertions (same discipline as test_decoder.c). */
    char xdg_dir[256];
    const char *tmp = getenv ("TMPDIR");
    snprintf (xdg_dir, sizeof (xdg_dir), "%s/spatial-kbd-layers-XXXXXX",
              (tmp && *tmp) ? tmp : "/tmp");
    assert (mkdtemp (xdg_dir));
    setenv ("XDG_CONFIG_HOME", xdg_dir, 1);

    printf ("=== keyboard layer tests ===\n");
    test_shift_one_shot ();
    test_shift_double_tap_caps_lock ();
    test_shift_leaves_specials_alone ();
    test_layer_toggle_maps_and_restores ();
    test_required_characters_are_reachable ();
    test_labels_follow_layer_and_shift ();
    test_url_round_trip ();
    test_shifted_word_round_trip ();
    printf ("=== ALL KEYBOARD LAYER TESTS PASSED ===\n");
    rmdir (xdg_dir);
    return 0;
}
