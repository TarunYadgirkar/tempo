// panel_surface.cpp — see panel_surface.h.

#include "ui/panel_surface.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "ui/font_atlas.h"
#include "ui/theme_tokens.h"

namespace mac_shell {

namespace {

inline float clamp01(float v) {
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

// Rounded-rect SDF (centre c, half-extent h, radius r).
inline float rrect_sdf(float px, float py, float cx, float cy, float hx,
                       float hy, float r) {
    float qx = std::fabs(px - cx) - (hx - r);
    float qy = std::fabs(py - cy) - (hy - r);
    float ax = std::fmax(qx, 0.0f), ay = std::fmax(qy, 0.0f);
    return std::sqrt(ax * ax + ay * ay) + std::fmin(std::fmax(qx, qy), 0.0f) -
           r;
}

// Deterministic 2D hash → [0,1). Same recipe as the shader-side grain.
inline float hash01(int x, int y) {
    uint32_t h = (uint32_t)x * 374761393u + (uint32_t)y * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return (float)(h ^ (h >> 16)) / 4294967296.0f;
}

}  // namespace

panel_surface::panel_surface(int width, int height) { resize(width, height); }

void panel_surface::resize(int width, int height) {
    width_ = std::max(1, width);
    height_ = std::max(1, height);
    rgba_.assign((size_t)width_ * height_ * 4, 0);
}

void panel_surface::fill(color_rgba c) {
    fill_rect(0, 0, width_, height_, c);
}

void panel_surface::fill_rect(int x, int y, int w, int h, color_rgba c) {
    int x0 = std::max(0, x);
    int y0 = std::max(0, y);
    int x1 = std::min(width_, x + w);
    int y1 = std::min(height_, y + h);
    for (int py = y0; py < y1; py++) {
        uint8_t *row = rgba_.data() + ((size_t)py * width_ + x0) * 4;
        for (int px = x0; px < x1; px++) {
            row[0] = c.r;
            row[1] = c.g;
            row[2] = c.b;
            row[3] = c.a;
            row += 4;
        }
    }
}

void panel_surface::blend_px(int x, int y, color_rgba c, float cov) {
    if (x < 0 || y < 0 || x >= width_ || y >= height_)
        return;
    float sa = (c.a / 255.0f) * clamp01(cov);
    if (sa <= 0.0f)
        return;
    uint8_t *p = rgba_.data() + ((size_t)y * width_ + x) * 4;
    float da = p[3] / 255.0f;
    float oa = sa + da * (1.0f - sa);
    if (oa <= 0.0f)
        return;
    for (int i = 0; i < 3; i++) {
        float s = (&c.r)[i] / 255.0f;
        float d = p[i] / 255.0f;
        p[i] = (uint8_t)std::lround(
            255.0f * (s * sa + d * da * (1.0f - sa)) / oa);
    }
    p[3] = (uint8_t)std::lround(255.0f * oa);
}

void panel_surface::blend_rect(int x, int y, int w, int h, color_rgba c) {
    for (int py = y; py < y + h; py++)
        for (int px = x; px < x + w; px++)
            blend_px(px, py, c, 1.0f);
}

void panel_surface::fill_rounded_rect(float x, float y, float w, float h,
                                      float radius, color_rgba c) {
    fill_rounded_rect_vgrad(x, y, w, h, radius, c, c);
}

void panel_surface::fill_rounded_rect_vgrad(float x, float y, float w,
                                            float h, float radius,
                                            color_rgba top,
                                            color_rgba bottom) {
    float cx = x + w * 0.5f, cy = y + h * 0.5f;
    float hx = w * 0.5f, hy = h * 0.5f;
    float r = std::fmin(radius, std::fmin(hx, hy));
    int x0 = std::max(0, (int)std::floor(x - 1));
    int y0 = std::max(0, (int)std::floor(y - 1));
    int x1 = std::min(width_, (int)std::ceil(x + w + 1));
    int y1 = std::min(height_, (int)std::ceil(y + h + 1));
    for (int py = y0; py < y1; py++) {
        float t = h > 1.0f ? clamp01(((float)py + 0.5f - y) / h) : 0.0f;
        color_rgba c = {
            (uint8_t)std::lround(top.r + (bottom.r - top.r) * t),
            (uint8_t)std::lround(top.g + (bottom.g - top.g) * t),
            (uint8_t)std::lround(top.b + (bottom.b - top.b) * t),
            (uint8_t)std::lround(top.a + (bottom.a - top.a) * t)};
        for (int px = x0; px < x1; px++) {
            float d = rrect_sdf((float)px + 0.5f, (float)py + 0.5f, cx, cy,
                                hx, hy, r);
            float cov = clamp01(0.5f - d);
            if (cov > 0.0f)
                blend_px(px, py, c, cov);
        }
    }
}

void panel_surface::stroke_rounded_rect(float x, float y, float w, float h,
                                        float radius, float thickness,
                                        color_rgba c) {
    float cx = x + w * 0.5f, cy = y + h * 0.5f;
    float hx = w * 0.5f, hy = h * 0.5f;
    float r = std::fmin(radius, std::fmin(hx, hy));
    float ht = thickness * 0.5f;
    int x0 = std::max(0, (int)std::floor(x - thickness - 1));
    int y0 = std::max(0, (int)std::floor(y - thickness - 1));
    int x1 = std::min(width_, (int)std::ceil(x + w + thickness + 1));
    int y1 = std::min(height_, (int)std::ceil(y + h + thickness + 1));
    for (int py = y0; py < y1; py++) {
        for (int px = x0; px < x1; px++) {
            float d = std::fabs(rrect_sdf((float)px + 0.5f, (float)py + 0.5f,
                                          cx, cy, hx, hy, r)) -
                      ht;
            float cov = clamp01(0.5f - d);
            if (cov > 0.0f)
                blend_px(px, py, c, cov);
        }
    }
}

void panel_surface::fill_circle(float cx, float cy, float r, color_rgba c) {
    int x0 = std::max(0, (int)std::floor(cx - r - 1));
    int y0 = std::max(0, (int)std::floor(cy - r - 1));
    int x1 = std::min(width_, (int)std::ceil(cx + r + 1));
    int y1 = std::min(height_, (int)std::ceil(cy + r + 1));
    for (int py = y0; py < y1; py++)
        for (int px = x0; px < x1; px++) {
            float dx = (float)px + 0.5f - cx, dy = (float)py + 0.5f - cy;
            float d = std::sqrt(dx * dx + dy * dy) - r;
            float cov = clamp01(0.5f - d);
            if (cov > 0.0f)
                blend_px(px, py, c, cov);
        }
}

void panel_surface::fill_ring_segment(float cx, float cy, float r0, float r1,
                                      float a0, float a1, float corner,
                                      color_rgba c) {
    float amid = (a0 + a1) * 0.5f;
    float ahalf = std::fabs(a1 - a0) * 0.5f;
    float rc = (r0 + r1) * 0.5f, rw = (r1 - r0) * 0.5f;
    int x0 = std::max(0, (int)std::floor(cx - r1 - 1));
    int y0 = std::max(0, (int)std::floor(cy - r1 - 1));
    int x1 = std::min(width_, (int)std::ceil(cx + r1 + 1));
    int y1 = std::min(height_, (int)std::ceil(cy + r1 + 1));
    for (int py = y0; py < y1; py++)
        for (int px = x0; px < x1; px++) {
            float dx = (float)px + 0.5f - cx, dy = (float)py + 0.5f - cy;
            float r = std::sqrt(dx * dx + dy * dy);
            float ang = std::atan2(dy, dx);
            float da = ang - amid;
            while (da > (float)M_PI)
                da -= 2.0f * (float)M_PI;
            while (da < -(float)M_PI)
                da += 2.0f * (float)M_PI;
            float dr = std::fabs(r - rc) - rw;                // radial
            float dt = (std::fabs(da) - ahalf) * std::fmax(r, 1.0f);  // arc
            float ax = std::fmax(dr + corner, 0.0f);
            float ay = std::fmax(dt + corner, 0.0f);
            float d = std::sqrt(ax * ax + ay * ay) +
                      std::fmin(std::fmax(dr, dt), 0.0f) - corner;
            float cov = clamp01(0.5f - d);
            if (cov > 0.0f)
                blend_px(px, py, c, cov);
        }
}

void panel_surface::apply_grain(float alpha) {
    if (alpha <= 0.0f)
        return;
    for (int y = 0; y < height_; y++) {
        uint8_t *row = rgba_.data() + (size_t)y * width_ * 4;
        for (int x = 0; x < width_; x++) {
            uint8_t *p = row + (size_t)x * 4;
            if (p[3] == 0)
                continue;
            float n = hash01(x, y) * alpha;
            for (int i = 0; i < 3; i++)  // screen blend
                p[i] = (uint8_t)std::lround(p[i] + n * (255 - p[i]));
        }
    }
}

int panel_surface::glyph_width(int scale) {
    return FONT_ATLAS_CELL_W * scale;
}

int panel_surface::glyph_height(int scale) {
    return FONT_ATLAS_CELL_H * scale;
}

int panel_surface::draw_text(int x, int y, int scale, const std::string &text,
                             color_rgba c) {
    scale = std::max(1, scale);
    const int cw = FONT_ATLAS_CELL_W, ch = FONT_ATLAS_CELL_H;
    int cx = x;
    for (char ch_in : text) {
        int code = (unsigned char)ch_in;
        if (code < FONT_ATLAS_FIRST || code > FONT_ATLAS_LAST)
            code = '?';
        const unsigned char *cell =
            FONT_ATLAS + (size_t)(code - FONT_ATLAS_FIRST) * cw * ch;
        for (int gy = 0; gy < ch; gy++)
            for (int gx = 0; gx < cw; gx++) {
                float cov = cell[gy * cw + gx] / 255.0f;
                if (cov <= 0.0f)
                    continue;
                for (int sy = 0; sy < scale; sy++)
                    for (int sx = 0; sx < scale; sx++)
                        blend_px(cx + gx * scale + sx, y + gy * scale + sy, c,
                                 cov);
            }
        cx += cw * scale;
        if (cx >= width_)
            break;
    }
    return cx - x;
}

void render_test_card(panel_surface &surface, uint64_t handle,
                      const std::string &title,
                      const std::vector<std::string> &input_log_tail,
                      bool focused) {
    using namespace theme;
    const uint8_t surf_a = (uint8_t)std::lround(255.0f * SURFACE_ALPHA);
    // Translucent Vantage surface with a slightly lighter header band.
    // Corner rounding + border + focus glow happen in the panel shader.
    color_rgba body = {SURFACE.r, SURFACE.g, SURFACE.b, surf_a};
    color_rgba body_deep = {BG.r, BG.g, BG.b, surf_a};
    surface.fill({0, 0, 0, 0});
    surface.fill_rounded_rect_vgrad(0, 0, (float)surface.width(),
                                    (float)surface.height(), 1.0f, body,
                                    body_deep);

    const int pad = 18;
    // Header: accent tick + title, muted handle id right-aligned.
    color_rgba tick = focused ? ACCENT : FG_MUTED;
    surface.fill_rounded_rect((float)pad, (float)pad + 3.0f, 4.0f, 14.0f,
                              2.0f, tick);
    std::string t = title.empty() ? "panel" : title;
    surface.draw_text(pad + 14, pad, 1, t, FG);
    char id[24];
    std::snprintf(id, sizeof(id), "#%llu", (unsigned long long)handle);
    surface.draw_text(
        surface.width() - pad - panel_surface::text_width(id, 1), pad, 1, id,
        FG_MUTED);

    // Hairline divider under the header.
    color_rgba divider = {BORDER.r, BORDER.g, BORDER.b,
                          (uint8_t)std::lround(255.0f * BORDER_ALPHA)};
    surface.fill_rounded_rect((float)pad,
                              (float)(pad + panel_surface::glyph_height(1) + 8),
                              (float)(surface.width() - 2 * pad), 1.0f, 0.5f,
                              divider);

    int y = pad + panel_surface::glyph_height(1) + 18;
    for (const auto &line : input_log_tail) {
        if (y + panel_surface::glyph_height(1) >= surface.height() - pad)
            break;
        surface.draw_text(pad, y, 1, line, FG_MUTED);
        y += panel_surface::glyph_height(1) + 4;
    }

    surface.apply_grain(GRAIN_ALPHA);
}

}  // namespace mac_shell
