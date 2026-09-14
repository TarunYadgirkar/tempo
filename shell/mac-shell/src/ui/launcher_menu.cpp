// launcher_menu.cpp — see launcher_menu.h.

#include "ui/launcher_menu.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>

extern "C" {
#include "menubar_items.h"
}

#include "ui/theme_tokens.h"

namespace mac_shell {

namespace {

// Full scrub range of the wrist roll mapped across one page of wedges.
constexpr float SCRUB_SPAN_RAD = 2.0944f;  // 120 deg
// Angular hysteresis on the wedge boundary, as a fraction of one wedge.
// Without it, roll jitter on a boundary flips the focus every frame and each
// flip re-rasterises the whole 768x768 HUD surface on the CPU. Mirrors
// depth_math.h orientation_bucket_from_gravity.
constexpr float SCRUB_HYSTERESIS_FRAC = 0.15f;

constexpr int SURFACE_SIZE = 768;

std::string default_config_path() {
    if (const char *xdg = getenv("XDG_CONFIG_HOME"); xdg && xdg[0])
        return std::string(xdg) + "/spatial-os/launcher.toml";
    if (const char *home = getenv("HOME"); home && home[0])
        return std::string(home) + "/.config/spatial-os/launcher.toml";
    return "";
}

std::string trim(const std::string &s) {
    size_t a = s.find_first_not_of(" \t\r");
    if (a == std::string::npos)
        return "";
    size_t b = s.find_last_not_of(" \t\r");
    return s.substr(a, b - a + 1);
}

std::string unquote(const std::string &s) {
    if (s.size() >= 2 && s.front() == '"' && s.back() == '"')
        return s.substr(1, s.size() - 2);
    return s;
}

void json_escape(const std::string &in, std::string &out) {
    for (char c : in) {
        if (c == '"' || c == '\\')
            out += '\\';
        out += c;
    }
}

}  // namespace

launcher_menu::launcher_menu() {
    // Default radial-launcher entries. Real openable mac apps go through the
    // `app:<name>` path (on_launcher_launch strips the prefix and hands the
    // name to the `launch-app` handler / headless fallback). "Close All" is
    // a self-target: on_launcher_launch runs close_all_panels_impl, closing
    // every panel without dismissing the launcher mid-gesture. A
    // ~/.config/spatial-os/launcher.toml ([[entry]] label/target blocks)
    // overrides this list. Six entries fill one radial page (RM_ITEMS_PER_PAGE).
    entries_ = {
        {"Test Card", "internal:test-card"},
        {"Safari", "app:Safari"},
        {"Terminal", "app:Terminal"},
        {"Finder", "app:Finder"},
        {"Notes", "app:Notes"},
        {"Close All", "close-all"},
    };
    load_config(nullptr);
    rebuild_model();
    surface_.resize(SURFACE_SIZE, SURFACE_SIZE);
}

void launcher_menu::load_config(const char *toml_path) {
    std::string path = toml_path && toml_path[0] ? toml_path
                                                 : default_config_path();
    if (path.empty())
        return;
    std::ifstream f(path);
    if (!f.is_open())
        return;

    std::vector<launcher_entry> loaded;
    launcher_entry cur;
    bool in_entry = false;
    auto flush = [&]() {
        if (in_entry && !cur.label.empty() && !cur.target.empty())
            loaded.push_back(cur);
        cur = {};
    };
    std::string line;
    while (std::getline(f, line)) {
        std::string t = trim(line);
        if (t.empty() || t[0] == '#')
            continue;
        if (t == "[[entry]]") {
            flush();
            in_entry = true;
            continue;
        }
        size_t eq = t.find('=');
        if (!in_entry || eq == std::string::npos)
            continue;
        std::string key = trim(t.substr(0, eq));
        std::string val = unquote(trim(t.substr(eq + 1)));
        if (key == "label")
            cur.label = val;
        else if (key == "target")
            cur.target = val;
        else
            std::fprintf(stderr, "launcher: %s: unknown key '%s' (skipped)\n",
                         path.c_str(), key.c_str());
    }
    flush();
    if (!loaded.empty()) {
        entries_ = std::move(loaded);
        rebuild_model();
    }
}

void launcher_menu::rebuild_model() {
    std::vector<const char *> labels;
    std::vector<void *> actions;
    labels.reserve(entries_.size());
    for (size_t i = 0; i < entries_.size(); i++) {
        labels.push_back(entries_[i].label.c_str());
        actions.push_back((void *)(uintptr_t)i);
    }
    items_.assign(entries_.size(), rm_item_t{});
    size_t n = mbi_from_labels(labels.data(), actions.data(), entries_.size(),
                               items_.data(), items_.size());
    rm_model_init(&model_, items_.data(), n);
}

void launcher_menu::set_launch_callback(launch_fn fn) {
    launch_ = std::move(fn);
}

void launcher_menu::show() {
    rm_model_init(&model_, items_.data(), items_.size());
    angle_rad_ = 0.0f;
    visible_ = true;
    surface_dirty_ = true;
}

void launcher_menu::hide() {
    visible_ = false;
    rm_model_reset_commit(&model_);
}

bool launcher_menu::select(int idx_on_page) {
    if (!visible_)
        return false;
    if (idx_on_page < 0 || (size_t)idx_on_page >= rm_model_items_on_page(&model_))
        return false;
    rm_model_set_focus(&model_, idx_on_page);
    surface_dirty_ = true;
    return true;
}

bool launcher_menu::commit() {
    bool ok = commit_keep_visible();
    visible_ = false;
    return ok;
}

bool launcher_menu::commit_keep_visible() {
    if (!visible_ || model_.focused < 0)
        return false;
    rm_model_set_thumb(&model_, 1.0f);
    bool ok = model_.committed;
    if (ok) {
        int idx = model_.committed_index;
        rm_model_reset_commit(&model_);
        if (idx >= 0 && (size_t)idx < entries_.size() && launch_)
            launch_(entries_[(size_t)idx]);
    }
    rm_model_set_thumb(&model_, 0.0f);
    // visible_ untouched: the focused wedge stays highlighted through the
    // scene's commit-linger window; the scene calls hide() when it elapses.
    return ok;
}

void launcher_menu::begin_rotation() {
    fist_tracker_reset(&tracker_);
    angle_rad_ = 0.0f;
}

void launcher_menu::update_rotation(const float wrist[3],
                                    const float index_mcp[3],
                                    const float middle_mcp[3],
                                    const float pinky_mcp[3], bool is_left) {
    if (!visible_)
        return;
    angle_rad_ = fist_tracker_update(&tracker_, wrist, index_mcp, middle_mcp,
                                     pinky_mcp, is_left ? 1 : 0);
    size_t n = rm_model_items_on_page(&model_);
    if (n == 0)
        return;
    // Wedge index in continuous units; the previous wedge keeps the focus
    // until the angle clears its boundary by SCRUB_HYSTERESIS_FRAC.
    float u = (angle_rad_ + SCRUB_SPAN_RAD * 0.5f) / SCRUB_SPAN_RAD *
              (float)n;
    int idx = (int)std::floor(u);
    idx = idx < 0 ? 0 : (idx >= (int)n ? (int)n - 1 : idx);
    int prev = model_.focused;
    if (prev >= 0 && prev < (int)n && idx != prev) {
        float d = u - ((float)prev + 0.5f);  // wedge centres sit on n + 0.5
        if (std::fabs(d) <= 0.5f + SCRUB_HYSTERESIS_FRAC)
            idx = prev;
    }
    if (idx != model_.focused) {
        rm_model_set_focus(&model_, idx);
        surface_dirty_ = true;
    }
}

bool launcher_menu::end_rotation_commit() {
    if (!visible_)
        return false;
    if (model_.focused < 0) {
        hide();  // released without ever scrubbing → cancel, launch nothing
        return false;
    }
    return commit();
}

bool launcher_menu::end_rotation_commit_keep_visible() {
    if (!visible_)
        return false;
    if (model_.focused < 0) {
        hide();
        return false;
    }
    return commit_keep_visible();
}

std::string launcher_menu::status_json() const {
    std::string s = "{";
    s += std::string("\"visible\":") + (visible_ ? "true" : "false");
    char buf[96];
    std::snprintf(buf, sizeof(buf),
                  ",\"focused\":%d,\"page\":%zu,\"page_count\":%zu,"
                  "\"angle_rad\":%g",
                  model_.focused, model_.page, rm_model_page_count(&model_),
                  (double)angle_rad_);
    s += buf;
    s += ",\"entries\":[";
    for (size_t i = 0; i < entries_.size(); i++) {
        if (i)
            s += ",";
        s += "\"";
        json_escape(entries_[i].label, s);
        s += "\"";
    }
    s += "]}";
    return s;
}

// Smooth annular wedges (SDF ring segments): the fan divides into equal
// arcs with small angular gaps; the focused wedge widens outward, fills
// with the Vantage accent, and carries an outer halo.
void launcher_menu::redraw_surface() {
    using namespace theme;
    surface_.fill({0, 0, 0, 0});
    const float cx = SURFACE_SIZE * 0.5f, cy = SURFACE_SIZE * 0.5f;

    size_t n = rm_model_items_on_page(&model_);
    const float fan = 3.6652f;  // 210 deg fan, opening upward
    const float up = -0.5f * (float)M_PI;  // surface-space "up" angle
    const float r0 = 128.0f, r1 = 236.0f;
    const float gap = 0.022f;    // angular gap between wedges (rad)
    const float corner = 10.0f;  // wedge corner rounding (px)

    for (size_t i = 0; i < n; i++) {
        const rm_item_t *it = rm_model_item_at(&model_, i);
        if (!it)
            continue;
        float a0 = up - fan * 0.5f + fan * (float)i / (float)n + gap;
        float a1 = up - fan * 0.5f + fan * (float)(i + 1) / (float)n - gap;
        bool focused = (int)i == model_.focused;

        float wr0 = r0, wr1 = r1;
        if (focused) {
            wr0 -= 4.0f;
            wr1 += 16.0f;  // selected wedge scales outward
            // Outer halo.
            surface_.fill_ring_segment(cx, cy, wr0 - 8.0f, wr1 + 14.0f,
                                       a0 - 0.015f, a1 + 0.015f, corner,
                                       {ACCENT.r, ACCENT.g, ACCENT.b, 56});
        }
        const uint8_t surf_a = (uint8_t)std::lround(255.0f * SURFACE_ALPHA);
        color_rgba fill = focused
                              ? color_rgba{ACCENT.r, ACCENT.g, ACCENT.b, 246}
                              : color_rgba{SURFACE.r, SURFACE.g, SURFACE.b,
                                           surf_a};
        surface_.fill_ring_segment(cx, cy, wr0, wr1, a0, a1, corner, fill);
        // Hairline rim on unfocused wedges.
        if (!focused)
            surface_.fill_ring_segment(cx, cy, wr1 - 2.0f, wr1, a0, a1, 1.0f,
                                       {BORDER.r, BORDER.g, BORDER.b, 40});

        float amid = (a0 + a1) * 0.5f;
        float rc = (wr0 + wr1) * 0.5f;
        float lx = cx + std::cos(amid) * rc;
        float ly = cy + std::sin(amid) * rc;
        std::string label = it->label;
        int scale = 1;
        int tw = panel_surface::text_width(label, scale);
        float max_w = (a1 - a0) * rc - 12.0f;
        while (tw > (int)max_w && label.size() > 2) {
            label = label.substr(0, label.size() - 2) + "~";
            tw = panel_surface::text_width(label, scale);
        }
        color_rgba ink = focused ? color_rgba{BG.r, BG.g, BG.b, 255} : FG;
        surface_.draw_text((int)std::lround(lx - tw * 0.5f),
                           (int)std::lround(
                               ly - panel_surface::glyph_height(scale) * 0.5f),
                           scale, label, ink);
    }

    // Hub: quiet dial centre with an accent dot.
    surface_.fill_circle(cx, cy, 27.0f,
                         {SURFACE.r, SURFACE.g, SURFACE.b, 235});
    surface_.stroke_rounded_rect(cx - 27.0f, cy - 27.0f, 54.0f, 54.0f, 27.0f,
                                 1.5f, {BORDER.r, BORDER.g, BORDER.b, 46});
    surface_.fill_circle(cx, cy, 5.0f, ACCENT);

    // Page dots under the fan.
    size_t pages = rm_model_page_count(&model_);
    if (pages > 1) {
        float total_w = (float)pages * 18.0f;
        for (size_t p = 0; p < pages; p++) {
            color_rgba c = p == model_.page ? ACCENT
                                            : color_rgba{FG_MUTED.r,
                                                         FG_MUTED.g,
                                                         FG_MUTED.b, 130};
            surface_.fill_circle(cx - total_w * 0.5f + (float)p * 18.0f +
                                     9.0f,
                                 cy + r1 + 42.0f, 4.0f, c);
        }
    }
    surface_.apply_grain(GRAIN_ALPHA);
    surface_dirty_ = false;
    surface_version_++;
}

launcher_render_state launcher_menu::snapshot(uint64_t have_version) const {
    launcher_render_state rs;
    rs.visible = visible_;
    if (!visible_)
        return rs;
    if (surface_dirty_)
        const_cast<launcher_menu *>(this)->redraw_surface();
    size_t n = rm_model_items_on_page(&model_);
    for (size_t i = 0; i < n; i++) {
        const rm_item_t *it = rm_model_item_at(&model_, i);
        rs.labels.push_back(it ? it->label : "");
    }
    rs.focused = model_.focused;
    rs.angle_rad = angle_rad_;
    rs.page = model_.page;
    rs.page_count = rm_model_page_count(&model_);
    rs.surface_w = surface_.width();
    rs.surface_h = surface_.height();
    if (have_version != surface_version_)
        rs.rgba.assign(surface_.pixels(),
                       surface_.pixels() + surface_.size_bytes());
    rs.version = surface_version_;
    return rs;
}

}  // namespace mac_shell
