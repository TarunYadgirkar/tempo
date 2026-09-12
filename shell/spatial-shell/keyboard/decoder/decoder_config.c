/* decoder_config.c — see decoder_config.h.
 *
 * Minimal TOML-subset reader (sections, key = value, '#' comments) —
 * the same subset the gesture engine's ge_load_config consumes, so the
 * two config files share one grammar and no parser dependency is added.
 *
 * SPDX-License-Identifier: MIT
 */

#include "decoder_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void
decoder_config_defaults (decoder_config_t *cfg)
{
    if (!cfg) return;
    /* Surface LM weight kept LOW so a CLEAN on-key tap wins on its
     * spatial evidence — at 0.45 the bigram/unigram prior flipped
     * clean taps on rare letters (q,z). */
    cfg->lm_weight_surface     = 0.22f;
    /* Loose near-plane gate: survives the reconstruction placing the
     * whole hand ~an inch high (see the plunge-detector comment in
     * decoder.c). */
    cfg->sigma_z_m             = 0.025f;
    cfg->tap_z_smooth_a        = 0.5f;
    cfg->tap_hyst_m            = 0.008f;
    cfg->tap_plunge_m          = 0.018f;
    /* One tap_hyst_m of finger separation: the same ~8 mm scale that
     * debounces the ~10 mm tracking jitter, so hand-relative noise alone
     * cannot satisfy the gate, while a really extended finger clears it
     * 2-3x over. */
    cfg->tap_prominence_m      = 0.008f;

    cfg->attention_temperature = 0.15f;
    /* Midair spatial signal is noisier, so the LM carries more. */
    cfg->lm_weight_midair      = 0.65f;

    /* 0.30 at 60 Hz settles in ~50 ms. */
    cfg->ema_alpha             = 0.30f;
    /* -3.0 ≈ P > 0.05 after LM fusion. */
    cfg->emit_threshold        = -3.0f;
    cfg->cooldown_s            = 0.18f;
    /* No human taps two DISTINCT keys closer than this; suppresses
     * near-simultaneous multi-finger double fires. */
    cfg->emit_refractory_s     = 0.07f;
    /* 3 frames at 60 Hz ≈ 50 ms, matches tap-and-lift kinematics. */
    cfg->peak_holdoff_frames   = 3;

    /* Longer than any tap's bottom-out (a tap lifts within ~150 ms)
     * and past the tap cooldown, so a hold can never race a tap. */
    cfg->hold_dwell_s          = 0.40f;
    cfg->hold_repeat_delay_s   = 0.30f;
    cfg->hold_repeat_hz        = 15.0f;
}

static char *
trim (char *s)
{
    while (*s == ' ' || *s == '\t') ++s;
    size_t n = strlen (s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t'
                     || s[n - 1] == '\r' || s[n - 1] == '\n'))
        s[--n] = '\0';
    return s;
}

static bool
parse_float_str (const char *s, float *out)
{
    char *end = NULL;
    double v = strtod (s, &end);
    if (end == s || (end && *end != '\0')) return false;
    *out = (float) v;
    return true;
}

static bool
parse_int_str (const char *s, int *out)
{
    char *end = NULL;
    long v = strtol (s, &end, 10);
    if (end == s || (end && *end != '\0')) return false;
    *out = (int) v;
    return true;
}

static bool
apply_key (decoder_config_t *cfg, const char *section, const char *key,
           const char *val)
{
    struct {
        const char *section;
        const char *key;
        float      *f;
        int        *i;
    } map[] = {
        { "surface",   "lm_weight",             &cfg->lm_weight_surface,     NULL },
        { "surface",   "sigma_z_m",             &cfg->sigma_z_m,             NULL },
        { "surface",   "tap_z_smooth_a",        &cfg->tap_z_smooth_a,        NULL },
        { "surface",   "tap_hyst_m",            &cfg->tap_hyst_m,            NULL },
        { "surface",   "tap_plunge_m",          &cfg->tap_plunge_m,          NULL },
        { "surface",   "tap_prominence_m",      &cfg->tap_prominence_m,      NULL },
        { "midair",    "attention_temperature", &cfg->attention_temperature, NULL },
        { "midair",    "lm_weight",             &cfg->lm_weight_midair,      NULL },
        { "streaming", "ema_alpha",             &cfg->ema_alpha,             NULL },
        { "streaming", "emit_threshold",        &cfg->emit_threshold,        NULL },
        { "streaming", "cooldown_s",            &cfg->cooldown_s,            NULL },
        { "streaming", "emit_refractory_s",     &cfg->emit_refractory_s,     NULL },
        { "streaming", "peak_holdoff_frames",   NULL, &cfg->peak_holdoff_frames },
        { "hold",      "dwell_s",               &cfg->hold_dwell_s,          NULL },
        { "hold",      "repeat_delay_s",        &cfg->hold_repeat_delay_s,   NULL },
        { "hold",      "repeat_hz",             &cfg->hold_repeat_hz,        NULL },
    };
    for (size_t i = 0; i < sizeof (map) / sizeof (map[0]); ++i) {
        if (strcmp (section, map[i].section) != 0) continue;
        if (strcmp (key, map[i].key) != 0) continue;
        bool ok = map[i].f ? parse_float_str (val, map[i].f)
                           : parse_int_str (val, map[i].i);
        if (!ok) {
            fprintf (stderr,
                     "decoder_config: [%s] %s: unparseable value '%s'\n",
                     section, key, val);
        }
        return true;
    }
    return false;
}

static bool
default_config_path (char *buf, size_t buflen)
{
    const char *xdg = getenv ("XDG_CONFIG_HOME");
    if (xdg && *xdg) {
        snprintf (buf, buflen, "%s/spatial-os/keyboard.toml", xdg);
        return true;
    }
    const char *home = getenv ("HOME");
    if (home && *home) {
        snprintf (buf, buflen, "%s/.config/spatial-os/keyboard.toml", home);
        return true;
    }
    return false;
}

bool
decoder_config_load (decoder_config_t *cfg, const char *toml_path)
{
    if (!cfg) return false;

    char pathbuf[1024];
    const char *path = toml_path;
    if (!path || !*path) {
        if (!default_config_path (pathbuf, sizeof (pathbuf))) return false;
        path = pathbuf;
    }

    FILE *in = fopen (path, "r");
    if (!in) return false;   /* missing config is normal on first run */

    char line[512];
    char section[64] = "";
    int lineno = 0;
    while (fgets (line, sizeof (line), in)) {
        ++lineno;
        /* Strip a trailing comment (unquoted '#'). */
        bool in_str = false;
        for (char *p = line; *p; ++p) {
            if (*p == '"') in_str = !in_str;
            else if (!in_str && *p == '#') { *p = '\0'; break; }
        }
        char *s = trim (line);
        if (!*s) continue;

        if (*s == '[') {
            char *end = strchr (s, ']');
            if (!end) {
                fprintf (stderr,
                         "decoder_config: %s:%d malformed section header\n",
                         path, lineno);
                continue;
            }
            *end = '\0';
            snprintf (section, sizeof (section), "%s", trim (s + 1));
            continue;
        }

        char *eq = strchr (s, '=');
        if (!eq) {
            fprintf (stderr,
                     "decoder_config: %s:%d expected key = value (got '%s')\n",
                     path, lineno, s);
            continue;
        }
        *eq = '\0';
        char *key = trim (s);
        char *val = trim (eq + 1);
        if (!apply_key (cfg, section, key, val)) {
            /* Per-key Gaussian refits and hold tunables land as later
             * tasks; unknown keys are logged, not fatal. */
            fprintf (stderr,
                     "decoder_config: %s:%d ignoring unknown key [%s] %s\n",
                     path, lineno, section, key);
        }
    }
    fclose (in);
    return true;
}
