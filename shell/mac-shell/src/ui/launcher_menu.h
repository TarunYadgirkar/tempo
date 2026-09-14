// launcher_menu.h — radial app launcher for mac-shell.
//
// Reuses the pure-C radial-menu model (spatial-shell/radial-menu/radial_menu.c
// rm_model_*, mbi_from_labels) and the noise-robust fist-roll tracker
// (vendor/wxrd/src/fist_rotation.c). The fist_launcher gesture engages the
// menu; the roll angle scrubs the focused wedge; releasing the fist commits
// the selection through the rm_model thumb-commit path.
//
// Entries load from ~/.config/spatial-os/launcher.toml ([[entry]] blocks with
// label/target); defaults are the internal test-card plus Safari, Terminal,
// Finder, Notes via the launch-app path, and a "Close All" self-target that
// runs close_all_panels (closing every panel without dismissing the
// launcher). Targets: "internal:<title>" spawns an internal test-card panel,
// "close-all" closes every panel, anything else is handed to the app launcher.
//
// Pure C++ — the renderer consumes snapshot() (a CPU-rendered HUD texture).
// Not thread-safe on its own: the owning scene serialises all calls.

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

extern "C" {
#include "fist_rotation.h"
#include "radial_menu.h"
}

#include "ui/panel_surface.h"

namespace mac_shell {

struct launcher_entry {
    std::string label;
    std::string target;
};

struct launcher_render_state {
    bool visible = false;
    std::vector<std::string> labels;  // current page
    int focused = -1;
    float angle_rad = 0.0f;
    size_t page = 0;
    size_t page_count = 1;
    int surface_w = 0;
    int surface_h = 0;
    std::vector<uint8_t> rgba;
    uint64_t version = 0;
};

class launcher_menu {
   public:
    using launch_fn = std::function<void(const launcher_entry &)>;

    launcher_menu();

    launcher_menu(const launcher_menu &) = delete;
    launcher_menu &operator=(const launcher_menu &) = delete;

    // nullptr → $XDG_CONFIG_HOME/spatial-os/launcher.toml (then
    // ~/.config/...). Missing/empty file keeps the defaults.
    void load_config(const char *toml_path);

    void set_launch_callback(launch_fn fn);

    void show();
    void hide();  // cancel without committing
    bool visible() const { return visible_; }

    // Control-plane selection (absolute index within the current page).
    bool select(int idx_on_page);
    // Commit the focused wedge (launch + hide). False when nothing focused.
    bool commit();
    // Commit the focused wedge but stay visible with it highlighted — the
    // scene keeps the menu up for the commit-linger window, then hide()s.
    bool commit_keep_visible();

    // Gesture-driven scrubbing: reset at fist BEGIN, feed raw fist joints
    // (scene frame) each tick, commit on END.
    void begin_rotation();
    void update_rotation(const float wrist[3], const float index_mcp[3],
                         const float middle_mcp[3], const float pinky_mcp[3],
                         bool is_left);
    // A release with no wedge ever focused is a cancel: it hides the menu and
    // returns false rather than launching an arbitrary entry.
    bool end_rotation_commit();
    // END-of-fist commit that leaves the menu visible for the linger window.
    bool end_rotation_commit_keep_visible();

    int focused_index() const { return model_.focused; }
    float angle_rad() const { return angle_rad_; }
    const std::vector<launcher_entry> &entries() const { return entries_; }

    std::string status_json() const;
    // have_version: the surface version the caller already holds on the
    // GPU. Matching it skips the full RGBA copy (768x768x4 per frame).
    launcher_render_state snapshot(uint64_t have_version = 0) const;

   private:
    void rebuild_model();
    void redraw_surface();

    std::vector<launcher_entry> entries_;
    std::vector<rm_item_t> items_;
    rm_model_t model_ = {};
    fist_tracker_t tracker_ = {};
    launch_fn launch_;

    bool visible_ = false;
    float angle_rad_ = 0.0f;

    panel_surface surface_;
    bool surface_dirty_ = true;
    uint64_t surface_version_ = 1;
};

}  // namespace mac_shell
