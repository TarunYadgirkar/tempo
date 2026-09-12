// font_atlas.h — embedded 8-bit-coverage glyph atlas (Monaco via CoreText at
// authoring time; see scripts/gen_font_atlas.m). Pure data: keeps scene_core
// headless-testable while every CPU surface gets antialiased type.

#pragma once

namespace mac_shell {

extern const int FONT_ATLAS_CELL_W;
extern const int FONT_ATLAS_CELL_H;
extern const int FONT_ATLAS_BASELINE;
extern const int FONT_ATLAS_FIRST;
extern const int FONT_ATLAS_LAST;
// (LAST - FIRST + 1) cells of CELL_W * CELL_H coverage bytes, rows top-down.
extern const unsigned char FONT_ATLAS[];

}  // namespace mac_shell
