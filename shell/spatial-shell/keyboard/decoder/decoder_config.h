/* decoder_config.h — runtime tunables for the typing decoder, loadable
 * from ~/.config/spatial-os/keyboard.toml (same discipline as the
 * gesture engine's gestures.toml: the tuning loop edits the TOML, never
 * the compiled defaults).
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SPATIAL_KEYBOARD_DECODER_DECODER_CONFIG_H
#define SPATIAL_KEYBOARD_DECODER_DECODER_CONFIG_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* All fields map 1:1 to keyboard.toml keys — see
 * spatial-shell/keyboard/sample-config/keyboard.toml for the schema
 * and the WHY behind each default. */
typedef struct {
    /* [surface] */
    float lm_weight_surface;   /* lm_weight */
    float sigma_z_m;           /* z-proximity width: a tap's valley must
                                * bottom within this of the plane.  After the
                                * plunge-detector rework this near-plane gate
                                * is the surviving z-proximity tunable, so it
                                * keeps the documented sigma_z_m key that the
                                * sweep tasks target. */
    float tap_z_smooth_a;      /* light z EMA on the trajectory */
    float tap_hyst_m;          /* z fall to start a plunge / xy window */
    float tap_plunge_m;        /* min descent AND lift amplitude for a tap */
    float tap_prominence_m;    /* at the valley, how far below the rest of its
                                * own hand the tapping finger must be extended
                                * (a tap extends one finger; an inter-key move
                                * carries the hand down together) */

    /* [midair] */
    float attention_temperature;
    float lm_weight_midair;    /* lm_weight */

    /* [streaming] */
    float ema_alpha;
    float emit_threshold;
    float cooldown_s;
    float emit_refractory_s;
    int   peak_holdoff_frames;

    /* [hold] — held-key auto-repeat (hold-backspace-to-delete). */
    float hold_dwell_s;         /* dwell_s: time pressed at the plunge bottom
                                 * before a tap-in-progress becomes a HOLD */
    float hold_repeat_delay_s;  /* repeat_delay_s: widget-side delay from the
                                 * first emit to the first repeat */
    float hold_repeat_hz;       /* repeat_hz: widget-side repeat rate */
} decoder_config_t;

void decoder_config_defaults (decoder_config_t *cfg);

/* Merge overrides from a TOML file on top of the current values in
 * *cfg.  toml_path NULL/empty means the default XDG location
 * ($XDG_CONFIG_HOME/spatial-os/keyboard.toml, falling back to
 * $HOME/.config/spatial-os/keyboard.toml).  A missing file is normal
 * (returns false, cfg untouched); malformed lines are logged to
 * stderr and skipped, the rest of the overrides still apply. */
bool decoder_config_load (decoder_config_t *cfg, const char *toml_path);

#ifdef __cplusplus
}
#endif

#endif
