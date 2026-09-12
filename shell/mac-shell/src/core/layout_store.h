// layout_store.h — named panel arrangements on disk (control verb `layout`).
//
// One JSON file per layout under $SPATIAL_OS_CONFIG_DIR (default
// ~/.config/spatial-os) / layouts/<name>.json. Pure C++ + POSIX so the scene
// core stays headless-testable.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mac_shell {

// One saved panel. `kind` is the wire spelling: "note", "internal" (the test
// card) or "captured" (a real mac window, restored via launch-app).
struct layout_panel {
    std::string kind;
    std::string app_id;
    std::string title;
    std::string body;   // note panels only
    bool accent = false;

    float pos[3] = {0.0f, 0.0f, 0.0f};
    float yaw = 0.0f;
    int width_px = 512;
    int height_px = 384;
    float width_m = 0.6f;

    bool has_anchor = false;
    uint8_t anchor_uuid[16] = {0};
    float anchor_offset[3] = {0.0f, 0.0f, 0.0f};
};

// [A-Za-z0-9_-]{1,40} — the name is a path component, so nothing else is
// accepted and no traversal is possible.
bool layout_name_valid(const std::string &name);

// <config dir>/layouts. Empty when neither SPATIAL_OS_CONFIG_DIR nor HOME is
// set, which every caller treats as an io_error.
std::string layout_dir();

bool layout_save(const std::string &name,
                 const std::vector<layout_panel> &panels, std::string &error);
// `error` is "not_found" when the layout does not exist.
bool layout_load(const std::string &name, std::vector<layout_panel> &out,
                 std::string &error);
std::vector<std::string> layout_list();

}  // namespace mac_shell
