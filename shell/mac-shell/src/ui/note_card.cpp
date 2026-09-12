// note_card.cpp — see note_card.h.

#include "ui/note_card.h"

#include <algorithm>
#include <cmath>

#include "ui/theme_tokens.h"

namespace mac_shell {

namespace {

// One padding scale for the card, used for every edge so the text block sits
// in an optically even frame (outer pad, then the same step between the rule
// and the text).
constexpr int PAD = 28;
constexpr int RULE_W = 3;
constexpr int RULE_GAP = 14;
constexpr int TITLE_SCALE = 2;
constexpr int BODY_SCALE = 1;
// Body leading: 20 px glyphs on a 28 px step is 1.4, the floor for text that
// wraps past two lines.
constexpr int BODY_STEP = 28;
constexpr int TITLE_GAP = 12;   // title baseline block → divider
constexpr int DIVIDER_GAP = 16; // divider → first body line

void push_line(std::vector<std::string> &out, std::string line) {
    while (!line.empty() && line.back() == ' ')
        line.pop_back();
    out.push_back(std::move(line));
}

void ellipsize(std::string &line, int max_cols) {
    if (max_cols <= 3) {
        line = "...";
        return;
    }
    if ((int)line.size() > max_cols - 3)
        line.resize((size_t)(max_cols - 3));
    while (!line.empty() && line.back() == ' ')
        line.pop_back();
    line += "...";
}

}  // namespace

std::vector<std::string> wrap_text(const std::string &text, int max_cols,
                                   size_t max_lines) {
    std::vector<std::string> lines;
    if (max_cols <= 0 || max_lines == 0)
        return lines;

    std::string line;
    size_t i = 0;
    bool truncated = false;
    while (i <= text.size()) {
        bool end_of_text = i == text.size();
        char c = end_of_text ? '\n' : text[i];
        if (c != '\n' && c != ' ' && c != '\t' && c != '\r') {
            size_t word_end = i;
            while (word_end < text.size() && text[word_end] != '\n' &&
                   text[word_end] != ' ' && text[word_end] != '\t' &&
                   text[word_end] != '\r')
                word_end++;
            std::string word = text.substr(i, word_end - i);
            i = word_end;
            while (!word.empty()) {
                int room = max_cols - (int)line.size();
                if (room <= 0 || (int)word.size() > room) {
                    if (!line.empty()) {
                        push_line(lines, line);
                        line.clear();
                        if (lines.size() == max_lines) {
                            truncated = true;
                            break;
                        }
                        continue;  // retry the word on the fresh line
                    }
                    // Longer than the whole measure: hard-split it.
                    line = word.substr(0, (size_t)max_cols);
                    word = word.substr((size_t)max_cols);
                    push_line(lines, line);
                    line.clear();
                    if (lines.size() == max_lines) {
                        truncated = true;
                        break;
                    }
                    continue;
                }
                line += word;
                word.clear();
            }
            if (truncated)
                break;
            continue;
        }
        if (c == '\n') {
            push_line(lines, line);
            line.clear();
            if (lines.size() == max_lines) {
                truncated = !end_of_text &&
                            text.find_first_not_of(" \t\r\n", i + 1) !=
                                std::string::npos;
                break;
            }
            if (end_of_text)
                break;
        } else if (!line.empty() && (int)line.size() < max_cols) {
            line += ' ';
        }
        i++;
    }
    if (!truncated && !line.empty() && lines.size() < max_lines)
        push_line(lines, line);

    // Drop a trailing blank produced by text that ends in a newline.
    while (!lines.empty() && lines.back().empty())
        lines.pop_back();
    if (truncated && !lines.empty())
        ellipsize(lines.back(), max_cols);
    return lines;
}

void render_note_card(panel_surface &surface, const std::string &title,
                      const std::string &body, bool accent) {
    using namespace theme;
    const int w = surface.width(), h = surface.height();
    const uint8_t surf_a = (uint8_t)std::lround(255.0f * SURFACE_ALPHA);
    color_rgba body_top = {SURFACE.r, SURFACE.g, SURFACE.b, surf_a};
    color_rgba body_bottom = {BG.r, BG.g, BG.b, surf_a};
    surface.fill({0, 0, 0, 0});
    // Corner rounding, border and focus glow are the panel shader's job, so
    // the card paints edge to edge.
    surface.fill_rounded_rect_vgrad(0, 0, (float)w, (float)h, 1.0f, body_top,
                                    body_bottom);

    const int text_x = PAD + (accent ? RULE_W + RULE_GAP : 0);
    const int text_w = w - text_x - PAD;
    if (text_w <= 0)
        return;

    const int title_h = panel_surface::glyph_height(TITLE_SCALE);
    int title_cols = text_w / panel_surface::glyph_width(TITLE_SCALE);
    std::string head = title.empty() ? "Note" : title;
    if ((int)head.size() > title_cols)
        ellipsize(head, title_cols);
    surface.draw_text(text_x, PAD, TITLE_SCALE, head, FG);

    const int divider_y = PAD + title_h + TITLE_GAP;
    color_rgba divider = {BORDER.r, BORDER.g, BORDER.b,
                          (uint8_t)std::lround(255.0f * BORDER_ALPHA)};
    surface.fill_rounded_rect((float)text_x, (float)divider_y, (float)text_w,
                              1.0f, 0.5f, divider);

    const int body_top_y = divider_y + 1 + DIVIDER_GAP;
    int room_lines = (h - PAD - body_top_y + BODY_STEP -
                      panel_surface::glyph_height(BODY_SCALE)) / BODY_STEP;
    size_t max_lines =
        (size_t)std::max(0, std::min((int)NOTE_BODY_MAX_LINES, room_lines));
    int body_cols = text_w / panel_surface::glyph_width(BODY_SCALE);
    std::vector<std::string> lines = wrap_text(body, body_cols, max_lines);

    int y = body_top_y;
    for (const auto &line : lines) {
        surface.draw_text(text_x, y, BODY_SCALE, line, FG_MUTED);
        y += BODY_STEP;
    }

    if (accent) {
        // One mark: a thin rule down the text block in Vantage orange. Its top
        // aligns with the title's cap, not its cell, so it reads as level.
        float y0 = (float)PAD + 4.0f;
        float y1 = lines.empty()
                       ? (float)(divider_y)
                       : (float)(y - BODY_STEP +
                                 panel_surface::glyph_height(BODY_SCALE) - 4);
        surface.fill_rounded_rect((float)PAD, y0, (float)RULE_W,
                                  std::fmax(y1 - y0, 8.0f), RULE_W * 0.5f,
                                  ACCENT);
    }

    surface.apply_grain(GRAIN_ALPHA);
}

}  // namespace mac_shell
