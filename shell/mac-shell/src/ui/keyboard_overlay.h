// keyboard_overlay.h — spatial virtual keyboard for mac-shell.
//
// Wraps the pure-C plunge decoder (spatial-shell/keyboard/decoder) around a
// keyboard plane placed in the scene frame: anchored to the nearest horizontal
// plane in front of the user when one exists, else a default floating plane
// in front of the head, rotated with it (same constants as wxrd's
// keyboard_pose_writer.c).
// Fingertips in scene coordinates are projected into plane-local coordinates
// (the keyboard_pose_writer dot-product math) and fed to decoder_update_ev;
// TAP/HOLD events surface through the emit callback, and HOLD auto-repeat is
// driven here from the decoder's hold_repeat_* config.
//
// The decoder always names a rectangle by its letters-layer keysym; this class
// owns the shift / layer state (keyboard_layers.c) that turns that into the
// character the user asked for, swallows taps on shift and the layer toggle,
// and repaints the keycap labels whenever either moves.
//
// Pure C++ — no Metal. The renderer consumes snapshot(): the plane basis plus
// a CPU-rendered key-grid surface with per-key press/hold highlights.
//
// Not thread-safe on its own: the owning scene serialises all calls.

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "spatial_bridge.h"

#include "core/anchor_math.h"
#include "ui/panel_surface.h"

#include "keyboard_layers.h"

extern "C" {
struct decoder_t;
}

namespace mac_shell {

// X11 keysyms the decoder emits for non-letter keys (keyboard_geom.c).
constexpr uint32_t KBD_KEYSYM_SPACE = 0x20;
constexpr uint32_t KBD_KEYSYM_BACKSPACE = XK_BackSpace;
constexpr uint32_t KBD_KEYSYM_RETURN = XK_Return;

struct keyboard_plane {
    float origin[3] = {0, 0, 0};  // top-left corner, scene frame
    float right[3] = {1, 0, 0};   // plane-local +x (metres)
    float down[3] = {0, 0, 1};    // plane-local +y (top row → space row)
    float normal[3] = {0, 1, 0};  // above-plane direction
};

// Per-key state for the instanced GPU keycaps: cap rect in plane-local
// metres (gap already excluded), press highlight 1→0, hold flag.
struct keyboard_key_cell {
    float cx_m = 0.0f, cy_m = 0.0f;
    float hw_m = 0.0f, hh_m = 0.0f;
    float press = 0.0f;
    bool held = false;
};

struct keyboard_render_state {
    bool visible = false;
    keyboard_plane plane;
    float width_m = 0.0f;
    float height_m = 0.0f;
    int surface_w = 0;
    int surface_h = 0;
    std::vector<uint8_t> rgba;
    std::vector<keyboard_key_cell> keys;
    uint64_t version = 0;
};

class keyboard_overlay {
   public:
    // repeat=true for HOLD auto-repeats after the first emit.
    using emit_fn = std::function<void(uint32_t keysym, const char *label,
                                       bool repeat)>;

    keyboard_overlay();
    ~keyboard_overlay();

    keyboard_overlay(const keyboard_overlay &) = delete;
    keyboard_overlay &operator=(const keyboard_overlay &) = delete;

    void set_emit_callback(emit_fn fn);

    // Place the plane and make the keyboard visible. Anchors to the nearest
    // live horizontal plane in front of the head; with none detected the
    // default floating plane sits head-relative, oriented by head_quat (scene
    // frame). Returns true when plane-anchored, false when floating.
    bool show(const anchor_env &env, const float head_quat[4],
              const sb_plane_t *planes, int n_planes);
    void hide();
    bool visible() const { return visible_; }
    bool anchored() const { return anchored_; }

    // Per-tick: project fingertips into the plane, run the decoder, fire
    // emits and drive hold auto-repeat + highlight decay.
    void update(const sb_hand_t hands[2], const bool present[2], float dt);

    std::string status_json() const;
    // have_version: the surface version the caller already holds on the
    // GPU. Matching it skips the full RGBA copy (1200x480x4 per frame).
    keyboard_render_state snapshot(uint64_t have_version = 0) const;

    const keyboard_plane &plane() const { return plane_; }
    kbd_layer_t layer() const { return layers_.layer; }
    kbd_shift_t shift() const { return layers_.shift; }

   private:
    void redraw_surface();
    void fire(uint32_t base_keysym, bool repeat);

    decoder_t *decoder_ = nullptr;
    emit_fn emit_;

    bool visible_ = false;
    bool anchored_ = false;
    keyboard_plane plane_;

    std::vector<float> highlight_;  // per key-geom index, 1 → 0 decay
    kbd_layer_state_t layers_{};
    int hold_key_idx_ = -1;
    uint32_t hold_keysym_ = 0;       // decoder (letters-layer) keysym
    uint32_t hold_emit_keysym_ = 0;  // what auto-repeats put on the wire
    std::string hold_label_;
    float hold_elapsed_s_ = 0.0f;
    float hold_next_repeat_s_ = 0.0f;

    uint32_t last_keysym_ = 0;

    panel_surface surface_;
    bool surface_dirty_ = true;
    uint64_t surface_version_ = 1;
};

}  // namespace mac_shell
