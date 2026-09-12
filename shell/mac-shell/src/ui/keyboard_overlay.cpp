// keyboard_overlay.cpp — see keyboard_overlay.h.

#include "ui/keyboard_overlay.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "ui/theme_tokens.h"
#include "core/placement_math.h"
#include "core/vec_math.h"

extern "C" {
#include "decoder.h"
#include "keyboard_geom.h"
}

namespace mac_shell {

namespace {

// Keyboard centre sits this far in front of the user's plane projection —
// keyboard_pose_writer.c places its default plane 0.40 m out; on a real desk
// slightly closer reads better with the 30 cm surface.
constexpr float PLANE_FORWARD_M = 0.35f;
// Default floating plane, head-relative (keyboard_pose_writer.c kDefaultOrigin):
// this far along the head's horizontal forward and this far below eye level.
constexpr float DEFAULT_FORWARD_M = 0.50f;
constexpr float DEFAULT_BELOW_M = 0.20f;
// Only planes within this range of the head are anchor candidates.
constexpr float PLANE_MAX_DIST_M = 3.0f;

constexpr float HIGHLIGHT_DECAY_PER_S = 5.0f;  // ~0.2 s fade (anim budget)

// Gap between neighbouring keycaps (each side), plane metres.
constexpr float KEY_GAP_M = 0.0012f;

// Surface resolution: 4000 px/m in both axes keeps key rects square with the
// 0.30 x 0.12 m field and the atlas type crisp under mipmapping.
constexpr int SURFACE_W = 1200;
constexpr int SURFACE_H = 480;
constexpr float PX_PER_M = SURFACE_W / KBD_GEOM_WIDTH_M;

const int TIP_JOINTS[5] = {SB_JOINT_THUMB_TIP, SB_JOINT_INDEX_TIP,
                           SB_JOINT_MIDDLE_TIP, SB_JOINT_RING_TIP,
                           SB_JOINT_PINKY_TIP};

std::string vec3_json(const float v[3]) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "[%g,%g,%g]", (double)v[0], (double)v[1],
                  (double)v[2]);
    return buf;
}

}  // namespace

keyboard_overlay::keyboard_overlay() {
    decoder_ = decoder_create(DECODER_MODE_SURFACE);
    highlight_.assign((size_t)kbd_geom_key_count(), 0.0f);
    kbd_layer_state_reset(&layers_);
    surface_.resize(SURFACE_W, SURFACE_H);
}

keyboard_overlay::~keyboard_overlay() {
    if (decoder_)
        decoder_destroy(decoder_);
}

void keyboard_overlay::set_emit_callback(emit_fn fn) { emit_ = std::move(fn); }

bool keyboard_overlay::show(const anchor_env &env, const float head_quat[4],
                            const sb_plane_t *planes, int n_planes) {
    float fwd_h[3];
    head_forward_horizontal(head_quat, fwd_h);

    // Nearest live horizontal plane IN FRONT of the head. Scored on the closest
    // point inside the plane's extent, not its centre — a long desk you are
    // sitting at the end of should still win over a small one further away.
    const sb_plane_t *best = nullptr;
    float best_d = PLANE_MAX_DIST_M;
    for (int i = 0; i < n_planes; i++) {
        if (planes[i].is_removed || planes[i].alignment != 0)
            continue;
        float center[3], normal[3], p_right[3], p_fwd[3];
        plane_to_scene_basis(env, planes[i], center, normal, p_right, p_fwd);
        float to_center[3];
        v3sub(center, env.head_pos, to_center);
        if (v3dot(to_center, fwd_h) <= 0.0f)
            continue;  // behind the user
        float d[3];
        v3sub(env.head_pos, center, d);
        float perp = v3dot(d, normal);
        float in_plane[3] = {d[0] - perp * normal[0], d[1] - perp * normal[1],
                             d[2] - perp * normal[2]};
        float hx = planes[i].extent[0] * 0.5f;
        float hz = planes[i].extent[1] * 0.5f;
        float lx = std::fmax(-hx, std::fmin(hx, v3dot(in_plane, p_right)));
        float lz = std::fmax(-hz, std::fmin(hz, v3dot(in_plane, p_fwd)));
        float nearest[3] = {center[0] + p_right[0] * lx + p_fwd[0] * lz,
                            center[1] + p_right[1] * lx + p_fwd[1] * lz,
                            center[2] + p_right[2] * lx + p_fwd[2] * lz};
        float delta[3];
        v3sub(nearest, env.head_pos, delta);
        float dist = v3length(delta);
        if (dist < best_d) {
            best_d = dist;
            best = &planes[i];
        }
    }

    if (best) {
        float center[3], normal[3];
        plane_to_scene_frame(env, *best, center, normal);
        if (normal[1] < 0.0f) {  // keep "above the plane" pointing up
            normal[0] = -normal[0];
            normal[1] = -normal[1];
            normal[2] = -normal[2];
        }
        // In-plane direction toward the head = keyboard "down" (space row
        // nearest the user).
        float to_head[3];
        v3sub(env.head_pos, center, to_head);
        float k = v3dot(to_head, normal);
        float down[3] = {to_head[0] - k * normal[0],
                         to_head[1] - k * normal[1],
                         to_head[2] - k * normal[2]};
        if (v3normalize(down) < 1e-4f) {
            down[0] = 0.0f;
            down[1] = 0.0f;
            down[2] = 1.0f;
        }
        float right[3];
        v3cross(normal, down, right);
        v3normalize(right);

        // Keyboard centre: the head's projection onto the plane, stepped
        // forward (away from the user), clamped to the plane's extent.
        float head_off[3];
        v3sub(env.head_pos, center, head_off);
        float hk = v3dot(head_off, normal);
        float proj[3] = {env.head_pos[0] - hk * normal[0],
                         env.head_pos[1] - hk * normal[1],
                         env.head_pos[2] - hk * normal[2]};
        float kb_center[3] = {proj[0] - down[0] * PLANE_FORWARD_M,
                              proj[1] - down[1] * PLANE_FORWARD_M,
                              proj[2] - down[2] * PLANE_FORWARD_M};
        float half = 0.5f * std::fmax(best->extent[0], best->extent[1]);
        float off[3];
        v3sub(kb_center, center, off);
        const float *axes[2] = {right, down};
        for (const float *axis : axes) {
            float p = v3dot(off, axis);
            float lim = std::fmax(0.0f, half);
            float clamped = std::fmax(-lim, std::fmin(lim, p));
            float dcl = clamped - p;
            kb_center[0] += dcl * axis[0];
            kb_center[1] += dcl * axis[1];
            kb_center[2] += dcl * axis[2];
        }

        for (int i = 0; i < 3; i++) {
            plane_.origin[i] = kb_center[i] -
                               right[i] * (KBD_GEOM_WIDTH_M * 0.5f) -
                               down[i] * (KBD_GEOM_HEIGHT_M * 0.5f);
            plane_.right[i] = right[i];
            plane_.down[i] = down[i];
            plane_.normal[i] = normal[i];
        }
        anchored_ = true;
    } else {
        // Floating default, laid out in the HEAD's frame rather than fixed
        // scene axes: space row nearest the user, keys receding along the
        // head's forward, and horizontally centred on it.
        const float up[3] = {0.0f, 1.0f, 0.0f};
        float down[3] = {-fwd_h[0], -fwd_h[1], -fwd_h[2]};
        float right[3];
        v3cross(up, down, right);
        v3normalize(right);
        for (int i = 0; i < 3; i++) {
            plane_.normal[i] = up[i];
            plane_.down[i] = down[i];
            plane_.right[i] = right[i];
            plane_.origin[i] = env.head_pos[i] + fwd_h[i] * DEFAULT_FORWARD_M -
                               up[i] * DEFAULT_BELOW_M -
                               right[i] * (KBD_GEOM_WIDTH_M * 0.5f);
        }
        anchored_ = false;
    }

    visible_ = true;
    surface_dirty_ = true;
    return anchored_;
}

void keyboard_overlay::hide() {
    visible_ = false;
    hold_key_idx_ = -1;
    hold_keysym_ = 0;
    hold_emit_keysym_ = 0;
    std::fill(highlight_.begin(), highlight_.end(), 0.0f);
}

void keyboard_overlay::fire(uint32_t base_keysym, bool repeat) {
    int idx = kbd_geom_find_keysym(base_keysym);
    if (idx >= 0 && idx < (int)highlight_.size())
        highlight_[idx] = 1.0f;  // press state renders GPU-side per key

    // A held key keeps sending the character it resolved to at HOLD_BEGIN — a
    // one-shot shift consumed by the press must not un-capitalise the repeats.
    if (repeat) {
        if (!hold_emit_keysym_)
            return;
        last_keysym_ = hold_emit_keysym_;
        if (emit_)
            emit_(hold_emit_keysym_, hold_label_.c_str(), true);
        return;
    }

    kbd_layer_out_t out;
    kbd_layer_apply(&layers_, base_keysym, &out);
    if (out.state_changed)
        surface_dirty_ = true;
    if (!out.emit)
        return;
    last_keysym_ = out.keysym;
    hold_emit_keysym_ = out.keysym;
    hold_label_ = out.label;
    if (emit_)
        emit_(out.keysym, out.label, false);
}

void keyboard_overlay::update(const sb_hand_t hands[2], const bool present[2],
                              float dt) {
    if (!visible_ || !decoder_)
        return;

    float tips[10][3];
    bool pres[10];
    std::memset(tips, 0, sizeof(tips));
    std::memset(pres, 0, sizeof(pres));
    for (int h = 0; h < 2; h++) {
        if (!present[h])
            continue;
        for (int t = 0; t < 5; t++) {
            const float *j = hands[h].joints[TIP_JOINTS[t]];
            float d[3] = {j[0] - plane_.origin[0], j[1] - plane_.origin[1],
                          j[2] - plane_.origin[2]};
            int idx = h * 5 + t;
            tips[idx][0] = v3dot(d, plane_.right);
            tips[idx][1] = v3dot(d, plane_.down);
            tips[idx][2] = v3dot(d, plane_.normal);
            pres[idx] = true;
        }
    }

    decoder_key_event_t ev = {DECODER_KEY_EVENT_NONE, 0};
    decoder_update_ev(decoder_, tips, pres, dt, &ev);

    switch (ev.kind) {
        case DECODER_KEY_EVENT_TAP:
            fire(ev.keysym, false);
            break;
        case DECODER_KEY_EVENT_HOLD_BEGIN: {
            hold_emit_keysym_ = 0;
            fire(ev.keysym, false);
            if (!hold_emit_keysym_)
                break;  // shift / layer toggle: nothing to auto-repeat
            hold_keysym_ = ev.keysym;
            hold_key_idx_ = kbd_geom_find_keysym(ev.keysym);
            hold_elapsed_s_ = 0.0f;
            hold_next_repeat_s_ =
                decoder_get_config(decoder_)->hold_repeat_delay_s;
            break;
        }
        case DECODER_KEY_EVENT_HOLD_END:
            hold_keysym_ = 0;
            hold_emit_keysym_ = 0;
            hold_key_idx_ = -1;
            break;
        default:
            break;
    }

    if (hold_keysym_) {
        const decoder_config_t *cfg = decoder_get_config(decoder_);
        float period = cfg->hold_repeat_hz > 0.01f
                           ? 1.0f / cfg->hold_repeat_hz
                           : 0.1f;
        hold_elapsed_s_ += dt;
        while (hold_elapsed_s_ >= hold_next_repeat_s_) {
            fire(hold_keysym_, true);
            hold_next_repeat_s_ += period;
        }
    }

    for (size_t i = 0; i < highlight_.size(); i++) {
        if ((int)i == hold_key_idx_)
            continue;  // held keys stay lit until HOLD_END
        if (highlight_[i] > 0.0f)
            highlight_[i] =
                std::fmax(0.0f, highlight_[i] - HIGHLIGHT_DECAY_PER_S * dt);
    }

    if (surface_dirty_)
        redraw_surface();
}

// Static keycap texture: styled caps + labels only. Press/hold/depression
// render GPU-side per key (key_fragment_main), so this redraws on show and on
// a shift / layer change, not per keystroke.
void keyboard_overlay::redraw_surface() {
    using namespace theme;
    surface_.fill({0, 0, 0, 0});
    const int n = kbd_geom_key_count();
    const int shift_idx = kbd_geom_find_keysym(XK_Shift_L);
    const int layer_idx = kbd_geom_find_keysym(XK_Mode_switch);
    const bool shift_lit =
        layers_.layer == KBD_LAYER_LETTERS && layers_.shift != KBD_SHIFT_OFF;
    for (int i = 0; i < n; i++) {
        const kbd_key_geom_t *g = kbd_geom_get(i);
        if (!g)
            continue;
        // Armed shift and the symbols layer light their own cap, so the board
        // shows which alphabet the next tap will produce without a legend.
        const bool armed = (i == shift_idx && shift_lit) ||
                           (i == layer_idx &&
                            layers_.layer == KBD_LAYER_SYMBOLS);
        float hw = (g->hw_m - KEY_GAP_M) * PX_PER_M;
        float hh = (g->hh_m - KEY_GAP_M) * PX_PER_M;
        float x = g->cx_m * PX_PER_M - hw;
        float y = g->cy_m * PX_PER_M - hh;
        float w = hw * 2.0f, h = hh * 2.0f;
        float radius = 0.0035f * PX_PER_M;
        const char *label_c = kbd_geom_label_at(i, layers_.layer, layers_.shift);
        bool wide = std::strlen(label_c) > 1;  // space/return/back/shift/123

        // Cap body: subtle top-lit vertical gradient; wide keys sit deeper.
        color_rgba top = wide ? color_rgba{36, 25, 20, 242}
                              : color_rgba{48, 34, 27, 244};
        color_rgba bottom = wide ? color_rgba{22, 15, 12, 242}
                                 : color_rgba{30, 20, 16, 244};
        if (armed) {
            top = {ACCENT.r, ACCENT.g, ACCENT.b, 244};
            bottom = {ACCENT_PRESSED.r, ACCENT_PRESSED.g, ACCENT_PRESSED.b,
                      244};
        }
        surface_.fill_rounded_rect_vgrad(x, y, w, h, radius, top, bottom);
        // Crisp top edge highlight — the "machined" catchlight.
        surface_.fill_rounded_rect(x + radius * 0.6f, y + 1.5f,
                                   w - radius * 1.2f, 2.0f, 1.0f,
                                   {BORDER.r, BORDER.g, BORDER.b, 34});
        // Hairline rim.
        surface_.stroke_rounded_rect(x + 0.5f, y + 0.5f, w - 1.0f, h - 1.0f,
                                     radius, 1.0f,
                                     {BORDER.r, BORDER.g, BORDER.b, 26});

        // Letter caps carry their own case: the board reads lowercase until
        // shift is armed, so shift state is legible from the letters too.
        std::string label = label_c;
        int scale = wide ? 1 : 2;
        int tw = panel_surface::text_width(label, scale);
        int th = panel_surface::glyph_height(scale);
        color_rgba ink = armed ? FG : (wide ? FG_MUTED : FG);
        surface_.draw_text((int)std::lround(x + (w - tw) * 0.5f),
                           (int)std::lround(y + (h - th) * 0.5f), scale,
                           label, ink);
    }
    surface_.apply_grain(GRAIN_ALPHA);
    surface_dirty_ = false;
    surface_version_++;
}

std::string keyboard_overlay::status_json() const {
    std::string s = "{";
    s += std::string("\"visible\":") + (visible_ ? "true" : "false");
    s += std::string(",\"anchored\":") + (anchored_ ? "true" : "false");
    s += ",\"origin\":" + vec3_json(plane_.origin);
    s += ",\"right\":" + vec3_json(plane_.right);
    s += ",\"down\":" + vec3_json(plane_.down);
    s += ",\"normal\":" + vec3_json(plane_.normal);
    char buf[96];
    std::snprintf(buf, sizeof(buf), ",\"width\":%g,\"height\":%g",
                  (double)KBD_GEOM_WIDTH_M, (double)KBD_GEOM_HEIGHT_M);
    s += buf;
    s += std::string(",\"layer\":\"") +
         (layers_.layer == KBD_LAYER_SYMBOLS ? "symbols" : "letters") + "\"";
    s += std::string(",\"shift\":\"") +
         (layers_.shift == KBD_SHIFT_CAPS   ? "caps"
          : layers_.shift == KBD_SHIFT_ONCE ? "once"
                                            : "off") +
         "\"";
    if (last_keysym_) {
        std::snprintf(buf, sizeof(buf), ",\"last_keysym\":\"0x%x\"",
                      last_keysym_);
        s += buf;
    }
    s += "}";
    return s;
}

keyboard_render_state keyboard_overlay::snapshot(uint64_t have_version) const {
    keyboard_render_state rs;
    rs.visible = visible_;
    if (!visible_)
        return rs;
    rs.plane = plane_;
    rs.width_m = KBD_GEOM_WIDTH_M;
    rs.height_m = KBD_GEOM_HEIGHT_M;
    rs.surface_w = surface_.width();
    rs.surface_h = surface_.height();
    if (have_version != surface_version_)
        rs.rgba.assign(surface_.pixels(),
                       surface_.pixels() + surface_.size_bytes());
    const int n = kbd_geom_key_count();
    rs.keys.reserve((size_t)n);
    for (int i = 0; i < n; i++) {
        const kbd_key_geom_t *g = kbd_geom_get(i);
        if (!g)
            continue;
        keyboard_key_cell cell;
        cell.cx_m = g->cx_m;
        cell.cy_m = g->cy_m;
        cell.hw_m = g->hw_m - KEY_GAP_M;
        cell.hh_m = g->hh_m - KEY_GAP_M;
        cell.press = i < (int)highlight_.size() ? highlight_[i] : 0.0f;
        cell.held = i == hold_key_idx_;
        rs.keys.push_back(cell);
    }
    rs.version = surface_version_;
    return rs;
}

}  // namespace mac_shell
