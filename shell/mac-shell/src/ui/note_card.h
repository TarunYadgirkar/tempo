// note_card.h — CPU-drawn note panel (control verb `note`).
//
// Same surface primitives and Vantage tokens as render_test_card; the card is
// a title line over word-wrapped body text, with an optional thin accent rule
// down the left of the text block. Sentence case throughout — the atlas has
// one face, so hierarchy comes from size, color and spacing.

#pragma once

#include <string>
#include <vector>

#include "ui/panel_surface.h"

namespace mac_shell {

// Body text the card will draw: greedy word wrap at `max_cols` columns,
// honouring existing newlines, capped at `max_lines`. A word longer than the
// measure is split; text past the cap ends the last line with "...".
std::vector<std::string> wrap_text(const std::string &text, int max_cols,
                                   size_t max_lines);

constexpr size_t NOTE_BODY_MAX_LINES = 6;
constexpr int NOTE_DEFAULT_W_PX = 512;
constexpr int NOTE_DEFAULT_H_PX = 320;

void render_note_card(panel_surface &surface, const std::string &title,
                      const std::string &body, bool accent);

}  // namespace mac_shell
