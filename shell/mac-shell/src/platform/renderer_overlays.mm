// renderer_overlays.mm — ShellRenderer HUD overlays: welcome card,
// onboarding, cheat sheet, toasts.

#include "platform/renderer_internal.h"

@implementation ShellRenderer (Overlays)

// Blit an overlay texture centered at (cx, cy) in NDC with the standard
// pop (0.96 → 1 scale with show) and returns the covered window-fraction
// rect for click hit-testing.
- (mac_shell::hit_rect)blitOverlay:(id<MTLTexture>)tex
                           encoder:(id<MTLRenderCommandEncoder>)enc
                              view:(MTKView *)view
                           centerX:(float)cx
                           centerY:(float)cy
                              show:(float)show {
    mac_shell::hit_rect rect;
    if (!tex || show < 0.02f)
        return rect;
    float dw = (float)std::fmax(1.0, view.drawableSize.width);
    float dh = (float)std::fmax(1.0, view.drawableSize.height);
    float pop = 0.96f + 0.04f * show;
    float hw = (float)tex.width / dw * pop;
    float hh = (float)tex.height / dh * pop;
    mac_shell::panel_vertex verts[6] = {
        {{cx - hw, cy - hh, 0}, {0, 1}}, {{cx + hw, cy - hh, 0}, {1, 1}},
        {{cx + hw, cy + hh, 0}, {1, 0}}, {{cx - hw, cy - hh, 0}, {0, 1}},
        {{cx + hw, cy + hh, 0}, {1, 0}}, {{cx - hw, cy + hh, 0}, {0, 0}},
    };
    mac_shell::panel_uniforms pu;
    std::memset(&pu, 0, sizeof(pu));
    pu.mvp = matrix_identity_float4x4;
    pu.alpha = show;
    [enc setRenderPipelineState:_blit_pipeline];
    [enc setDepthStencilState:_depth_none];
    [enc setVertexBytes:verts length:sizeof(verts) atIndex:0];
    [enc setVertexBytes:&pu length:sizeof(pu) atIndex:1];
    [enc setFragmentBytes:&pu length:sizeof(pu) atIndex:0];
    [enc setFragmentTexture:tex atIndex:0];
    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:6];

    rect.x0 = (cx - hw + 1.0f) * 0.5f;
    rect.x1 = (cx + hw + 1.0f) * 0.5f;
    rect.y0 = (cy - hh + 1.0f) * 0.5f;
    rect.y1 = (cy + hh + 1.0f) * 0.5f;
    rect.valid = true;
    return rect;
}

// Map a texture-pixel rect (y down) inside an overlay to window fractions
// (y up) given the overlay's on-screen rect.
static mac_shell::hit_rect sub_rect(const mac_shell::hit_rect &card,
                                    int tex_w, int tex_h, float px0,
                                    float py0, float px1, float py1) {
    mac_shell::hit_rect r;
    if (!card.valid)
        return r;
    float w = card.x1 - card.x0, h = card.y1 - card.y0;
    r.x0 = card.x0 + px0 / (float)tex_w * w;
    r.x1 = card.x0 + px1 / (float)tex_w * w;
    r.y0 = card.y0 + (1.0f - py1 / (float)tex_h) * h;
    r.y1 = card.y0 + (1.0f - py0 / (float)tex_h) * h;
    r.valid = true;
    return r;
}

// Standard card background: translucent vertical-gradient surface with a
// hairline border and the brand grain.
static void paint_card_bg(mac_shell::panel_surface &surf) {
    using namespace mac_shell::theme;
    surf.fill({0, 0, 0, 0});
    const float inset = 6.0f;
    const float w = (float)surf.width() - inset * 2;
    const float h = (float)surf.height() - inset * 2;
    const uint8_t a = (uint8_t)std::lround(255.0f * 0.94f);
    surf.fill_rounded_rect_vgrad(inset, inset, w, h, (float)RADIUS_LG * 1.6f,
                                 {SURFACE.r, SURFACE.g, SURFACE.b, a},
                                 {BG.r, BG.g, BG.b, a});
    surf.stroke_rounded_rect(inset, inset, w, h, (float)RADIUS_LG * 1.6f,
                             1.4f,
                             {BORDER.r, BORDER.g, BORDER.b,
                              (uint8_t)std::lround(255.0f * BORDER_ALPHA)});
}

static void center_text(mac_shell::panel_surface &surf, int y, int scale,
                        const std::string &text, mac_shell::color_rgba c) {
    int x = (surf.width() -
             mac_shell::panel_surface::text_width(text, scale)) /
            2;
    surf.draw_text(x, y, scale, text, c);
}

// Accent tick + SPATULA wordmark, centered.
static void paint_wordmark(mac_shell::panel_surface &surf, int y) {
    using namespace mac_shell::theme;
    const std::string mark = "SPATULA";
    int tw = mac_shell::panel_surface::text_width(mark, 2);
    int x = (surf.width() - (tw + 26)) / 2;
    surf.fill_rounded_rect((float)x, (float)y + 3, 8.0f, 34.0f, 3.0f, ACCENT);
    surf.draw_text(x + 26, y, 2, mark, FG_MUTED);
}

// Welcome / empty-state / connection-lost / startup-error card.
- (void)drawWelcome:(id<MTLRenderCommandEncoder>)enc
               view:(MTKView *)view
         sceneEmpty:(bool)sceneEmpty
                 dt:(float)dt
               time:(float)t {
    using namespace mac_shell::theme;
    using mac_shell::color_rgba;
    using mac_shell::panel_surface;

    double now = CACurrentMediaTime();
    if (now - _perm_checked_at > 2.0) {
        _perm_screen = mac_shell::screen_capture_permitted();
        _perm_ax = mac_shell::accessibility_permitted();
        _perm_checked_at = now;
    }

    const bool err = !_s->startup_error.empty();
    bool want = err || (!_s->onboarding_visible &&
                        _s->welcome.card_visible(sceneEmpty));
    float show = _welcome_anim.tick(want, dt);
    if (show < 0.02f)
        return;

    const bool lost = _s->welcome.lost_wording();
    // Bonjour chip state: 1 advertising, 0 not, -1 still checking, -2 n/a.
    int bonjour = -2;
    if (!_s->replay) {
        if (_s->bonjour_matches.load() > 0)
            bonjour = 1;
        else if (_s->bonjour_browsing &&
                 now - _s->bonjour_started_at < mac_shell::BONJOUR_GRACE_S)
            bonjour = -1;
        else
            bonjour = 0;
    }
    // Pulse bucket: coarse enough that the card redraws a handful of times
    // per breath, not per frame.
    float phase = std::fmod(t / mac_shell::WELCOME_PULSE_PERIOD_S, 1.0f);
    int pulse_bucket =
        err ? 0 : (int)(phase * (float)mac_shell::WELCOME_PULSE_STEPS);

    // We are advertising and the phone still has not reached us: the macOS
    // application firewall drops inbound UDP with no error anywhere, so it is
    // the one cause the user cannot discover on their own.
    const bool firewall_hint = _s->welcome.firewall_hint(bonjour == 1);

    char sig[256];
    std::snprintf(sig, sizeof(sig), "%d|%d|%d|%s|%d|%d|%d|%d|%d", err ? 1 : 0,
                  lost ? 1 : 0, (int)_s->welcome.phase,
                  _s->local_ip.c_str(), _s->udp_port, bonjour,
                  _perm_screen ? 1 : 0, (_perm_ax ? 1 : 0) * 10 +
                  pulse_bucket, firewall_hint ? 1 : 0);
    if (_welcome_sig != sig) {
        _welcome_sig = sig;
        auto &surf = _welcome_surface;
        paint_card_bg(surf);
        paint_wordmark(surf, 62);

        if (err) {
            center_text(surf, 190, 3,
                        _s->startup_error_title.empty()
                            ? "Spatula can't start"
                            : _s->startup_error_title,
                        ERROR);
            int y = 300;
            for (const auto &line :
                 mac_shell::wrap_text(_s->startup_error, 52)) {
                center_text(surf, y, 2, line, FG);
                y += 44;
            }
            center_text(surf, y + 20, 2, "(Cmd+Q quits this window.)",
                        FG_MUTED);
        } else {
            // Pulsing status dot + headline.
            float breath =
                0.5f - 0.5f * std::cos(phase * 2.0f * (float)M_PI);
            std::string title =
                lost ? "Connection lost" : "Waiting for your iPhone";
            color_rgba title_c = lost ? WARNING : FG;
            int tw = panel_surface::text_width(title, 3) + 44;
            int tx = (surf.width() - tw) / 2;
            color_rgba dot = lost ? WARNING : ACCENT;
            dot.a = (uint8_t)std::lround(120.0f + 135.0f * breath);
            surf.fill_circle((float)tx + 12, 196.0f, 11.0f + 3.0f * breath,
                             dot);
            surf.draw_text(tx + 44, 168, 3, title, title_c);
            center_text(surf, 240, 2,
                        lost ? "Reopen SpatialBridge on your iPhone to "
                               "reconnect."
                             : "Stream from SpatialBridge to bring this "
                               "room to life.",
                        FG_MUTED);

            // This Mac's address, large — what the phone needs on networks
            // where Bonjour is blocked.
            center_text(surf, 330, 1, "THIS MAC ON YOUR WIFI", FG_MUTED);
            if (!_s->local_ip.empty()) {
                char addr[64];
                std::snprintf(addr, sizeof(addr), "%s : %d",
                              _s->local_ip.c_str(), _s->udp_port);
                center_text(surf, 362, 4, addr, FG);
            } else {
                center_text(surf, 372, 3, "No WiFi - join a network first",
                            WARNING);
            }

            // Three steps.
            const char *steps[3] = {
                "1   Open SpatialBridge on your iPhone",
                "2   Tap this Mac under 'Macs on your network'",
                "3   Point the phone at your room",
            };
            int sx = (surf.width() - 900) / 2 + 10;
            for (int i = 0; i < 3; i++)
                surf.draw_text(sx, 496 + i * 54, 2, steps[i],
                               i == 0 ? FG : FG_MUTED);

            if (firewall_hint)
                center_text(surf, 664, 1,
                            "Still nothing? macOS Firewall may be blocking "
                            "Spatula - System Settings > Network > Firewall "
                            "> Options",
                            WARNING);

            // Status chips: Bonjour / Screen Recording / Accessibility.
            struct chip {
                const char *label;
                std::string status;
                color_rgba dot_c;
            } chips[3];
            chips[0].label = "BONJOUR";
            switch (bonjour) {
                case 1:
                    chips[0].status = "advertising";
                    chips[0].dot_c = SUCCESS;
                    break;
                case -1:
                    chips[0].status = "checking...";
                    chips[0].dot_c = NEUTRAL;
                    break;
                case -2:
                    chips[0].status = "replay mode";
                    chips[0].dot_c = NEUTRAL;
                    break;
                default:
                    chips[0].status = "off - mDNS blocked on this network?";
                    chips[0].dot_c = WARNING;
            }
            chips[1].label = "SCREEN RECORDING";
            chips[1].status =
                _perm_screen ? "granted" : "off - app panels use cards";
            chips[1].dot_c = _perm_screen ? SUCCESS
                                          : WARNING;
            chips[2].label = "ACCESSIBILITY";
            chips[2].status =
                _perm_ax ? "granted" : "off - needed to type into apps";
            chips[2].dot_c = _perm_ax ? SUCCESS
                                      : NEUTRAL;

            const int chip_w = 370, chip_h = 88, gap = 26;
            int cx0 = (surf.width() - (chip_w * 3 + gap * 2)) / 2;
            for (int i = 0; i < 3; i++) {
                int x = cx0 + i * (chip_w + gap);
                int y = 706;
                surf.fill_rounded_rect((float)x, (float)y, (float)chip_w,
                                       (float)chip_h, (float)RADIUS_MD,
                                       {SURFACE.r, SURFACE.g, SURFACE.b,
                                        220});
                surf.stroke_rounded_rect(
                    (float)x, (float)y, (float)chip_w, (float)chip_h,
                    (float)RADIUS_MD, 1.0f,
                    {BORDER.r, BORDER.g, BORDER.b,
                     (uint8_t)std::lround(255.0f * BORDER_ALPHA)});
                surf.fill_circle((float)x + 26, (float)y + 30, 6.0f,
                                 chips[i].dot_c);
                surf.draw_text(x + 44, y + 20, 1, chips[i].label, FG);
                surf.draw_text(x + 44, y + 50, 1, chips[i].status,
                               FG_MUTED);
            }
        }
        surf.apply_grain(GRAIN_ALPHA);

        if (!_welcome_tex)
            _welcome_tex =
                [self makeMippedTextureW:(NSUInteger)surf.width()
                                       h:(NSUInteger)surf.height()];
        [self uploadTexture:_welcome_tex
                       rgba:surf.pixels()
                          w:(NSUInteger)surf.width()
                          h:(NSUInteger)surf.height()];
    }

    [self blitOverlay:_welcome_tex
              encoder:enc
                 view:view
              centerX:0.0f
              centerY:0.08f
                 show:show];
}

// One-at-a-time toast above the dock. The action button (System Settings
// deep link) is hit-tested via renderer_state::toast_button.
- (void)drawToast:(id<MTLRenderCommandEncoder>)enc
             view:(MTKView *)view
               dt:(float)dt {
    using namespace mac_shell::theme;
    using mac_shell::panel_surface;

    auto toasts = _s->world->snapshot_toasts();
    bool want = !toasts.empty() && !_s->onboarding_visible;
    if (want) {
        _toast_cached_text = toasts[0].text;
        _toast_cached_action =
            toasts[0].action ==
            mac_shell::toast_action::open_screen_recording_settings;
    }
    float show = _toast_anim.tick(want, dt);
    _s->toast_rect = {};
    _s->toast_button = {};
    if (show < 0.02f || _toast_cached_text.empty())
        return;

    const bool action = _toast_cached_action;
    const int btn_w = 230, btn_h = 56;
    float btn_px[4] = {0, 0, 0, 0};
    std::string sig = (action ? "a|" : "-|") + _toast_cached_text;
    if (_toast_sig != sig) {
        _toast_sig = sig;
        auto &surf = _toast_surface;
        surf.fill({0, 0, 0, 0});
        const float inset = 4.0f;
        const uint8_t a = (uint8_t)std::lround(255.0f * 0.94f);
        surf.fill_rounded_rect_vgrad(
            inset, inset, (float)surf.width() - inset * 2,
            (float)surf.height() - inset * 2, (float)RADIUS_LG,
            {SURFACE_PRESSED.r, SURFACE_PRESSED.g, SURFACE_PRESSED.b, a},
            {BG.r, BG.g, BG.b, a});
        surf.stroke_rounded_rect(
            inset, inset, (float)surf.width() - inset * 2,
            (float)surf.height() - inset * 2, (float)RADIUS_LG, 1.2f,
            {ACCENT.r, ACCENT.g, ACCENT.b, 90});

        int text_w = surf.width() - 60 - (action ? btn_w + 40 : 0);
        auto lines = mac_shell::wrap_text(
            _toast_cached_text,
            (size_t)std::max(8, text_w / panel_surface::glyph_width(2)));
        if (lines.size() > 3)
            lines.resize(3);
        int y0 = (surf.height() - (int)lines.size() * 38 + 8) / 2;
        for (size_t i = 0; i < lines.size(); i++)
            surf.draw_text(30, y0 + (int)i * 38, 2, lines[i], FG);

        if (action) {
            int bx = surf.width() - btn_w - 26;
            int by = (surf.height() - btn_h) / 2;
            surf.fill_rounded_rect((float)bx, (float)by, (float)btn_w,
                                   (float)btn_h, (float)RADIUS_MD, ACCENT);
            std::string label = "SETTINGS";
            surf.draw_text(
                bx + (btn_w - panel_surface::text_width(label, 2)) / 2,
                by + (btn_h - panel_surface::glyph_height(2)) / 2, 2, label,
                BG);
        }
        surf.apply_grain(GRAIN_ALPHA);

        if (!_toast_tex)
            _toast_tex = [self makeMippedTextureW:(NSUInteger)surf.width()
                                                h:(NSUInteger)surf.height()];
        [self uploadTexture:_toast_tex
                       rgba:surf.pixels()
                          w:(NSUInteger)surf.width()
                          h:(NSUInteger)surf.height()];
    }
    if (action) {
        btn_px[0] = (float)(_toast_surface.width() - btn_w - 26);
        btn_px[1] = (float)((_toast_surface.height() - btn_h) / 2);
        btn_px[2] = btn_px[0] + (float)btn_w;
        btn_px[3] = btn_px[1] + (float)btn_h;
    }

    // Sits above the dock strip.
    float dh = (float)std::fmax(1.0, view.drawableSize.height);
    float dock_top = -1.0f + 2.0f * (float)mac_shell::DOCK_TEX_H / dh;
    float hh = (float)_toast_tex.height / dh;
    mac_shell::hit_rect rect = [self blitOverlay:_toast_tex
                                         encoder:enc
                                            view:view
                                         centerX:0.0f
                                         centerY:dock_top + hh + 0.03f
                                            show:show];
    _s->toast_rect = rect;
    if (action)
        _s->toast_button =
            sub_rect(rect, _toast_surface.width(), _toast_surface.height(),
                     btn_px[0], btn_px[1], btn_px[2], btn_px[3]);
}

// Gesture cheat sheet (static content, rendered once).
- (void)drawCheatSheet:(id<MTLRenderCommandEncoder>)enc
                  view:(MTKView *)view
                    dt:(float)dt {
    using namespace mac_shell::theme;
    using mac_shell::panel_surface;

    bool want = _s->cheat_visible && !_s->onboarding_visible;
    float show = _cheat_anim.tick(want, dt);
    if (show < 0.02f)
        return;

    if (!_cheat_tex) {
        auto &surf = _cheat_surface;
        paint_card_bg(surf);
        center_text(surf, 54, 3, "Gestures", FG);

        struct row {
            const char *gesture;
            const char *action;
        };
        const row rows[5] = {
            {"pinch", "focus a panel"},
            {"fist, then roll wrist", "open the launcher"},
            {"palm-down hold", "summon the keyboard"},
            {"double-pinch the air", "gather your windows"},
        };
        int y = 170;
        for (const auto &r : rows) {
            surf.fill_circle(74.0f, (float)y + 20, 5.0f, ACCENT);
            surf.draw_text(100, y, 2, r.gesture, FG);
            surf.draw_text(560, y, 2, r.action, FG_MUTED);
            y += 74;
        }
        center_text(surf, y + 14, 1,
                    "? in the dock or Cmd+/ toggles this card - Esc or a "
                    "click closes it",
                    FG_MUTED);
        surf.apply_grain(GRAIN_ALPHA);
        _cheat_tex = [self makeMippedTextureW:(NSUInteger)surf.width()
                                            h:(NSUInteger)surf.height()];
        [self uploadTexture:_cheat_tex
                       rgba:surf.pixels()
                          w:(NSUInteger)surf.width()
                          h:(NSUInteger)surf.height()];
    }

    [self blitOverlay:_cheat_tex
              encoder:enc
                 view:view
              centerX:0.0f
              centerY:0.1f
                 show:show];
}

// First-launch onboarding: one modal card explaining both macOS grants in
// plain language before any TCC prompt ever fires (the prompts themselves
// still fire on first actual need).
- (void)drawOnboarding:(id<MTLRenderCommandEncoder>)enc
                  view:(MTKView *)view
                    dt:(float)dt {
    using namespace mac_shell::theme;
    using mac_shell::panel_surface;

    bool want = _s->onboarding_visible;
    float show = _onboard_anim.tick(want, dt);
    _s->onboard_button = {};
    if (show < 0.02f)
        return;

    // Dim the scene behind the modal.
    {
        simd_float3 quad[6] = {
            {-1, -1, 0}, {1, -1, 0}, {1, 1, 0},
            {-1, -1, 0}, {1, 1, 0},  {-1, 1, 0},
        };
        mac_shell::solid_uniforms su;
        su.mvp = matrix_identity_float4x4;
        su.color = simd_make_float4(0, 0, 0, 0.55f * show);
        [enc setRenderPipelineState:_solid_pipeline];
        [enc setDepthStencilState:_depth_none];
        [enc setVertexBytes:quad length:sizeof(quad) atIndex:0];
        [enc setVertexBytes:&su length:sizeof(su) atIndex:1];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle
                vertexStart:0
                vertexCount:6];
    }

    const int btn_w = 360, btn_h = 84;
    const int btn_x = (mac_shell::ONBOARD_TEX_W - btn_w) / 2;
    const int btn_y = mac_shell::ONBOARD_TEX_H - 148;
    if (!_onboard_tex) {
        auto &surf = _onboard_surface;
        paint_card_bg(surf);
        paint_wordmark(surf, 56);
        center_text(surf, 122, 3, "Welcome to Spatula", FG);

        int y = 210;
        for (const auto &line : mac_shell::wrap_text(
                 "Your iPhone becomes the eyes of this Mac - it tracks the "
                 "room and streams what it sees into this window.",
                 48)) {
            center_text(surf, y, 2, line, FG_MUTED);
            y += 42;
        }
        y += 26;
        center_text(surf, y, 1,
                    "TWO MACOS PERMISSIONS YOU'LL BE ASKED ABOUT LATER", FG);
        y += 44;

        auto grant = [&](const char *title, const std::string &detail) {
            surf.fill_circle(96.0f, (float)y + 18, 6.0f, ACCENT);
            surf.draw_text(120, y, 2, title, FG);
            y += 42;
            for (const auto &line : mac_shell::wrap_text(detail, 56)) {
                surf.draw_text(120, y, 1, line, FG_MUTED);
                y += 28;
            }
            y += 22;
        };
        grant("Screen Recording",
              "Lets Spatula bring your Mac apps into the room as floating "
              "panels. macOS asks the first time you open an app panel - "
              "click Allow, then relaunch when prompted.");
        grant("Accessibility",
              "Lets your hands type and click inside those panels. macOS "
              "asks the first time you tap inside one.");

        surf.fill_rounded_rect((float)btn_x, (float)btn_y, (float)btn_w,
                               (float)btn_h, (float)RADIUS_MD, ACCENT);
        std::string label = "Get started";
        surf.draw_text(
            btn_x + (btn_w - panel_surface::text_width(label, 2)) / 2,
            btn_y + (btn_h - panel_surface::glyph_height(2)) / 2, 2, label,
            BG);
        surf.apply_grain(GRAIN_ALPHA);
        _onboard_tex = [self makeMippedTextureW:(NSUInteger)surf.width()
                                              h:(NSUInteger)surf.height()];
        [self uploadTexture:_onboard_tex
                       rgba:surf.pixels()
                          w:(NSUInteger)surf.width()
                          h:(NSUInteger)surf.height()];
    }

    mac_shell::hit_rect card = [self blitOverlay:_onboard_tex
                                         encoder:enc
                                            view:view
                                         centerX:0.0f
                                         centerY:0.05f
                                            show:show];
    _s->onboard_button = sub_rect(card, mac_shell::ONBOARD_TEX_W,
                                  mac_shell::ONBOARD_TEX_H, (float)btn_x,
                                  (float)btn_y, (float)(btn_x + btn_w),
                                  (float)(btn_y + btn_h));
}

@end
