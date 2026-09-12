/* keyboard_geom.c — see keyboard_geom.h. */

#include "keyboard_geom.h"

#include <stddef.h>

#define XK_a             0x0061
#define XK_b             0x0062
#define XK_c             0x0063
#define XK_d             0x0064
#define XK_e             0x0065
#define XK_f             0x0066
#define XK_g             0x0067
#define XK_h             0x0068
#define XK_i             0x0069
#define XK_j             0x006a
#define XK_k             0x006b
#define XK_l             0x006c
#define XK_m             0x006d
#define XK_n             0x006e
#define XK_o             0x006f
#define XK_p             0x0070
#define XK_q             0x0071
#define XK_r             0x0072
#define XK_s             0x0073
#define XK_t             0x0074
#define XK_u             0x0075
#define XK_v             0x0076
#define XK_w             0x0077
#define XK_x             0x0078
#define XK_y             0x0079
#define XK_z             0x007a
#define XK_space         0x0020

typedef struct {
    uint32_t keysym;
    const char *label;
    float width_units;   /* 1.0 = base letter, 1.5 = wide, 5.0 = space */
    uint32_t sym_keysym; /* symbols layer; 0 = this key is layer-invariant */
    const char *sym_label;
} key_def_t;

typedef struct {
    const key_def_t *keys;
    int count;
} row_def_t;

/* Row defs (mirrors keyboard_layout.c and keyboard_widget.c).  Shift and the
 * layer toggle ARE tap targets: the decoder scores them like any other key and
 * the consumer (mac keyboard_overlay / the GTK widget) turns them into modifier
 * state via keyboard_layers.c rather than sending them on the wire.
 *
 * The symbols layer re-labels the SAME rectangles — one physical grid, two
 * character sets — so every per-key Gaussian (μ from cx/cy, σ from hw/hh) is
 * identical on both layers and the spatial model needs no second fit.  On the
 * symbols layer the shift slot carries '=' (URLs need it and shift has nothing
 * to modify there), and the layer key reads "abc". */
static const key_def_t kRow0[] = {
    {XK_q, "q", 1.0f, '1', "1"}, {XK_w, "w", 1.0f, '2', "2"},
    {XK_e, "e", 1.0f, '3', "3"}, {XK_r, "r", 1.0f, '4', "4"},
    {XK_t, "t", 1.0f, '5', "5"}, {XK_y, "y", 1.0f, '6', "6"},
    {XK_u, "u", 1.0f, '7', "7"}, {XK_i, "i", 1.0f, '8', "8"},
    {XK_o, "o", 1.0f, '9', "9"}, {XK_p, "p", 1.0f, '0', "0"},
};
static const key_def_t kRow1[] = {
    {XK_a, "a", 1.0f, '-', "-"}, {XK_s, "s", 1.0f, '_', "_"},
    {XK_d, "d", 1.0f, '/', "/"}, {XK_f, "f", 1.0f, ':', ":"},
    {XK_g, "g", 1.0f, ';', ";"}, {XK_h, "h", 1.0f, '(', "("},
    {XK_j, "j", 1.0f, ')', ")"}, {XK_k, "k", 1.0f, '@', "@"},
    {XK_l, "l", 1.0f, '#', "#"},
};
static const key_def_t kRow2[] = {
    {XK_Shift_L, "shift", 1.5f, '=', "="},
    {XK_z, "z", 1.0f, '.', "."}, {XK_x, "x", 1.0f, ',', ","},
    {XK_c, "c", 1.0f, '?', "?"}, {XK_v, "v", 1.0f, '!', "!"},
    {XK_b, "b", 1.0f, '\'', "'"}, {XK_n, "n", 1.0f, '"', "\""},
    {XK_m, "m", 1.0f, '+', "+"},
    {XK_BackSpace, "back", 1.5f, 0, NULL},
};
static const key_def_t kRow3[] = {
    {XK_Mode_switch, "123", 1.5f, 0, "abc"},
    {XK_space, "space", 5.0f, 0, NULL},
    {XK_Return, "return", 2.0f, 0, NULL},
};

#define NROWS 4
static const row_def_t kRows[NROWS] = {
    {kRow0, sizeof(kRow0)/sizeof(kRow0[0])},
    {kRow1, sizeof(kRow1)/sizeof(kRow1[0])},
    {kRow2, sizeof(kRow2)/sizeof(kRow2[0])},
    {kRow3, sizeof(kRow3)/sizeof(kRow3[0])},
};

static const char *const kUpperLetterLabels[26] = {
    "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M",
    "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z",
};

/* Lazy-build into a flat array on first access.  Static lifetime;
 * never freed. */
#define MAX_KEYS 64
static kbd_key_geom_t g_keys[MAX_KEYS];
static const key_def_t *g_defs[MAX_KEYS];
static int g_key_count = 0;
static int g_built = 0;

static void
_build (void)
{
    if (g_built) return;
    g_built = 1;

    const float row_h = KBD_GEOM_HEIGHT_M / (float)NROWS;
    for (int r = 0; r < NROWS; ++r) {
        /* Sum units in this row. */
        float total_units = 0.0f;
        for (int c = 0; c < kRows[r].count; ++c) {
            total_units += kRows[r].keys[c].width_units;
        }
        if (total_units <= 0) continue;
        const float unit_w = KBD_GEOM_WIDTH_M / total_units;

        float cursor_x = 0.0f;
        for (int c = 0; c < kRows[r].count; ++c) {
            const float kw = kRows[r].keys[c].width_units * unit_w;
            if (g_key_count >= MAX_KEYS) return;
            const int i = g_key_count++;
            kbd_key_geom_t *g = &g_keys[i];
            g_defs[i] = &kRows[r].keys[c];
            g->keysym = kRows[r].keys[c].keysym;
            g->label  = kRows[r].keys[c].label;
            g->cx_m   = cursor_x + kw * 0.5f;
            g->cy_m   = ((float)r + 0.5f) * row_h;
            g->hw_m   = kw * 0.5f;
            g->hh_m   = row_h * 0.5f;
            cursor_x += kw;
        }
    }
}

int
kbd_geom_key_count (void)
{
    _build ();
    return g_key_count;
}

const kbd_key_geom_t *
kbd_geom_get (int i)
{
    _build ();
    if (i < 0 || i >= g_key_count) return NULL;
    return &g_keys[i];
}

int
kbd_geom_find_keysym (uint32_t keysym)
{
    _build ();
    for (int i = 0; i < g_key_count; ++i) {
        if (g_keys[i].keysym == keysym) return i;
    }
    return -1;
}

static bool
is_letter (uint32_t keysym)
{
    return keysym >= XK_a && keysym <= XK_z;
}

uint32_t
kbd_geom_keysym_at (int i, kbd_layer_t layer, kbd_shift_t shift)
{
    _build ();
    if (i < 0 || i >= g_key_count) return 0;
    const uint32_t base = g_keys[i].keysym;
    if (layer == KBD_LAYER_SYMBOLS)
        return g_defs[i]->sym_keysym ? g_defs[i]->sym_keysym : base;
    if (shift != KBD_SHIFT_OFF && is_letter (base))
        return base - ('a' - 'A');
    return base;
}

const char *
kbd_geom_label_at (int i, kbd_layer_t layer, kbd_shift_t shift)
{
    _build ();
    if (i < 0 || i >= g_key_count) return "";
    const uint32_t base = g_keys[i].keysym;
    if (layer == KBD_LAYER_SYMBOLS)
        return g_defs[i]->sym_label ? g_defs[i]->sym_label : g_keys[i].label;
    if (base == XK_Shift_L)
        return shift == KBD_SHIFT_CAPS ? "CAPS"
             : shift == KBD_SHIFT_ONCE ? "SHIFT" : "shift";
    if (shift != KBD_SHIFT_OFF && is_letter (base))
        return kUpperLetterLabels[base - XK_a];
    return g_keys[i].label;
}
