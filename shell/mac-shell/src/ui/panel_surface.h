// panel_surface.h — CPU-backed RGBA surface for internal panels and HUD
// textures. Pure C++; no CoreGraphics so the scene core stays
// headless-testable. Antialiased rounded-rect / circle / ring primitives
// (SDF coverage), film-grain, and text via the embedded CoreText-rasterised
// atlas (font_atlas.h).

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mac_shell {

struct color_rgba {
    uint8_t r, g, b, a;
};

class panel_surface {
   public:
    panel_surface(int width = 512, int height = 384);

    void resize(int width, int height);
    int width() const { return width_; }
    int height() const { return height_; }

    const uint8_t *pixels() const { return rgba_.data(); }
    uint8_t *pixels() { return rgba_.data(); }
    size_t size_bytes() const { return rgba_.size(); }

    void fill(color_rgba c);
    void fill_rect(int x, int y, int w, int h, color_rgba c);
    // "Over"-composites c (weighted by c.a * cov) onto the pixel.
    void blend_px(int x, int y, color_rgba c, float cov = 1.0f);
    void blend_rect(int x, int y, int w, int h, color_rgba c);

    // Antialiased rounded rectangle, alpha-blended. Optional vertical
    // gradient (top → bottom) via the *_vgrad variant.
    void fill_rounded_rect(float x, float y, float w, float h, float radius,
                           color_rgba c);
    void fill_rounded_rect_vgrad(float x, float y, float w, float h,
                                 float radius, color_rgba top,
                                 color_rgba bottom);
    // Antialiased rounded-rect outline (stroke centred on the edge).
    void stroke_rounded_rect(float x, float y, float w, float h, float radius,
                             float thickness, color_rgba c);
    void fill_circle(float cx, float cy, float r, color_rgba c);
    // Annular segment: radii [r0, r1], angles [a0, a1] (radians, 0 = +x,
    // CCW in surface space), slightly rounded corners. For radial menus.
    void fill_ring_segment(float cx, float cy, float r0, float r1, float a0,
                           float a1, float corner, color_rgba c);

    // Deterministic film-grain (screen blend) over every non-transparent
    // pixel — the brand's signature texture; keep alpha faint (~0.07).
    void apply_grain(float alpha);

    // Antialiased text from the embedded atlas (Monaco, 10x20 cells).
    // Integer scale; returns the x advance in pixels.
    int draw_text(int x, int y, int scale, const std::string &text,
                  color_rgba c);
    static int glyph_width(int scale);
    static int glyph_height(int scale);
    static int text_width(const std::string &text, int scale) {
        return (int)text.size() * glyph_width(scale);
    }

   private:
    int width_;
    int height_;
    std::vector<uint8_t> rgba_;
};

// Renders the standard internal test-card: title header, divider, and the
// tail of the input log. Vantage-themed; corner rounding happens in the
// panel shader, so the card paints edge-to-edge.
void render_test_card(panel_surface &surface, uint64_t handle,
                      const std::string &title,
                      const std::vector<std::string> &input_log_tail,
                      bool focused);

}  // namespace mac_shell
