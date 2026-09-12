// theme_tokens.h — mac-shell mirror of the design-token system
// (spatial-shell/theme/default.toml deep-merged with vantage.toml, the
// brand theme). mac-shell has no generator hookup yet, so the resolved
// values are pinned here; keep in lockstep when the theme changes.
//
// verified: 2026-08-29 -- spatial-shell/theme/default.toml
//                        spatial-shell/theme/vantage.toml

#pragma once

#include "ui/panel_surface.h"

namespace mac_shell {
namespace theme {

// [colors] — Vantage palette (warm near-black + signature orange).
constexpr color_rgba BG              = {20, 13, 10, 255};    // #140d0a
constexpr color_rgba SURFACE         = {27, 18, 14, 255};    // #1b120e
constexpr color_rgba SURFACE_PRESSED = {42, 24, 19, 255};    // #2a1813
constexpr color_rgba FG              = {244, 239, 224, 255}; // #f4efe0
constexpr color_rgba FG_MUTED        = {197, 178, 154, 255}; // #c5b29a
constexpr color_rgba ACCENT          = {237, 123, 12, 255};  // #ed7b0c
constexpr color_rgba ACCENT_PRESSED  = {197, 64, 10, 255};   // #c5400a
constexpr color_rgba WARNING         = {255, 179, 0, 255};   // #ffb300
constexpr color_rgba ERROR           = {214, 54, 28, 255};   // #d6361c
constexpr color_rgba BORDER          = {244, 239, 224, 255}; // #f4efe0
// mac-shell-only status-dot colors (no theme.toml counterpart yet).
constexpr color_rgba SUCCESS         = {92, 190, 96, 255};   // #5cbe60
constexpr color_rgba NEUTRAL         = {110, 100, 92, 255};  // #6e645c

// [translucency]
constexpr float SURFACE_ALPHA = 0.85f;
constexpr float BORDER_ALPHA = 0.18f;

// [grain] — vantage.toml enables it; alpha 0.07, screen blend.
constexpr float GRAIN_ALPHA = 0.07f;

// [radii] (px on CPU surfaces; PANEL_RADIUS_M is md at ~500 px/m).
constexpr int RADIUS_SM = 6;
constexpr int RADIUS_MD = 14;
constexpr int RADIUS_LG = 22;
constexpr float PANEL_RADIUS_M = 0.028f;

// [springs.*] — same constants wxrd's animation.c reads from the theme.
constexpr float SPRING_OPEN_STIFFNESS = 200.0f;
constexpr float SPRING_OPEN_DAMPING = 20.0f;
constexpr float SPRING_CLOSE_STIFFNESS = 200.0f;
constexpr float SPRING_CLOSE_DAMPING = 20.0f;
constexpr float SPRING_MOVE_STIFFNESS = 250.0f;
constexpr float SPRING_MOVE_DAMPING = 22.0f;
constexpr float SPRING_SNAP_STIFFNESS = 300.0f;
constexpr float SPRING_SNAP_DAMPING = 18.0f;

// [animations.fade] — the ≤200 ms interruptible budget for fades.
constexpr float FADE_DURATION_S = 0.16f;

// [launcher] — anti-twitch timings. The gesture engine already gates the
// fist on a 300 ms min-hold before BEGIN (>= the 250 ms pre-arm budget);
// these govern the scene-side dismiss state machine.
// Tracking loss (gesture CANCEL) keeps the menu up this long — a few dropped
// frames must not dismiss it; the fist re-acquiring within the grace resumes.
constexpr float LAUNCHER_GRACE_S = 0.6f;
// After a release commits, the menu stays visible with the chosen wedge
// highlighted this long before fading.
constexpr float LAUNCHER_COMMIT_LINGER_S = 0.3f;

// [gestures] — double-pinch on empty space within this window recalls all
// unanchored panels into an arc in front of the head (gather-panels).
constexpr float DOUBLE_PINCH_GATHER_S = 0.6f;

}  // namespace theme
}  // namespace mac_shell
