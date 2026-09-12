// renderer_dock.mm — ShellRenderer screen-space HUD: radial launcher
// blit and the dock/status strip.

#include "platform/renderer_internal.h"

@implementation ShellRenderer (Dock)

- (void)drawLauncher:(id<MTLRenderCommandEncoder>)enc
              aspect:(float)aspect
                  dt:(float)dt {
    _perf.enter();
    auto ln = _s->world->snapshot_launcher(_launcher_tex ? _launcher_tex_version
                                                         : 0);
    _perf.leave();
    float show = _launcher_anim.tick(ln.visible, dt);
    if (show < 0.02f)
        return;
    if (ln.visible && !ln.rgba.empty() &&
        _launcher_tex_version != ln.version) {
        if (!_launcher_tex ||
            _launcher_tex.width != (NSUInteger)ln.surface_w)
            _launcher_tex = [self makeMippedTextureW:(NSUInteger)ln.surface_w
                                                   h:(NSUInteger)ln.surface_h];
        _perf.enter();
        [self uploadTexture:_launcher_tex
                       rgba:ln.rgba.data()
                          w:(NSUInteger)ln.surface_w
                          h:(NSUInteger)ln.surface_h];
        _perf.leave();
        _launcher_tex_version = ln.version;
    }
    if (!_launcher_tex)
        return;

    float scale = 0.92f + 0.08f * show;
    float half_h = 0.62f * scale;
    float half_w = half_h / aspect;  // square on screen
    mac_shell::panel_vertex verts[6] = {
        {{-half_w, -half_h, 0}, {0, 1}}, {{half_w, -half_h, 0}, {1, 1}},
        {{half_w, half_h, 0}, {1, 0}},   {{-half_w, -half_h, 0}, {0, 1}},
        {{half_w, half_h, 0}, {1, 0}},   {{-half_w, half_h, 0}, {0, 0}},
    };
    mac_shell::panel_uniforms pu;
    std::memset(&pu, 0, sizeof(pu));
    pu.mvp = matrix_identity_float4x4;  // vertices already in NDC
    pu.alpha = show;
    [enc setRenderPipelineState:_blit_pipeline];
    [enc setDepthStencilState:_depth_none];
    [enc setVertexBytes:verts length:sizeof(verts) atIndex:0];
    [enc setVertexBytes:&pu length:sizeof(pu) atIndex:1];
    [enc setFragmentBytes:&pu length:sizeof(pu) atIndex:0];
    [enc setFragmentTexture:_launcher_tex atIndex:0];
    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:6];
}

// Dock/status strip: a floating translucent bar — running panels (click to
// focus, orange dot marks focus), the Vantage wordmark, clock, packet rate,
// and tracking quality as a colored dot. CPU-rendered, redrawn only when the
// content signature changes; blitted along the bottom edge.
- (void)drawDockWithEncoder:(id<MTLRenderCommandEncoder>)enc
                     panels:(const std::vector<mac_shell::scene::render_panel>
                                 &)panels
                       view:(MTKView *)view {
    using namespace mac_shell::theme;
    using mac_shell::color_rgba;
    using mac_shell::panel_surface;

    time_t now = time(nullptr);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    char clock_str[16];
    std::snprintf(clock_str, sizeof(clock_str), "%02d:%02d", tm_now.tm_hour,
                  tm_now.tm_min);
    float rate = _s->receiver ? sb_get_packet_rate(_s->receiver) : 0.0f;
    // The displayed number follows the bucket, not the raw rate: folding a
    // per-packet-noisy integer into the cache signature redrew the whole
    // dock surface on the CPU most frames.
    int rate_shown = (int)std::lround(rate / mac_shell::DOCK_RATE_BUCKET) *
                     mac_shell::DOCK_RATE_BUCKET;
    float quality = _s->world->tracking_quality();

    // Connection chip: wording + dot color from the welcome model's phase.
    char conn_text[48];
    color_rgba conn_dot;
    switch (_s->welcome.phase) {
        case mac_shell::link_phase::connected:
            std::snprintf(conn_text, sizeof(conn_text),
                          "IPHONE CONNECTED  %d PKT/S", rate_shown);
            conn_dot = SUCCESS;
            break;
        case mac_shell::link_phase::lost:
            std::snprintf(conn_text, sizeof(conn_text), "STREAM LOST");
            conn_dot = ERROR;
            break;
        default:
            std::snprintf(conn_text, sizeof(conn_text), "WAITING");
            conn_dot = WARNING;
    }

    // Tracking-quality wording beside its dot.
    const char *trk_text = quality < 0.0f    ? "NO POSE"
                           : quality >= 0.9f ? "TRACKING OK"
                           : quality >= 0.4f ? "TRACKING LIMITED"
                                             : "TRACKING LOST";
    color_rgba trk = quality < 0.0f    ? NEUTRAL
                     : quality >= 0.9f ? SUCCESS
                     : quality >= 0.4f ? WARNING
                                       : ERROR;

    // Frames arriving but no 0x0A intrinsics ever seen: the passthrough is
    // being stretched across the hardcoded 60 deg frustum, so it is visibly
    // mis-scaled. Say so rather than letting the user think the camera is
    // just bad.
    const int64_t frame_age_ms =
        _s->receiver ? sb_get_frame_age_ms(_s->receiver) : -1;
    const bool calib_pending =
        !_have_intrinsics && frame_age_ms >= 0 &&
        (float)frame_age_ms / 1000.0f <= mac_shell::FRAME_STALE_AFTER_S;

    // Hands are optional on the wire: when the phone streams but never sees a
    // hand, every gesture silently does nothing — say so in the dock.
    int hands_seen = 0;
    for (int slot = 0; slot < 2; slot++) {
        sb_hand_t tmp;
        if (_s->world->hand_joints(slot, tmp))
            hands_seen++;
    }
    const bool show_hands = rate > 0.0f;
    char hands_text[16];
    if (hands_seen == 0)
        std::snprintf(hands_text, sizeof(hands_text), "NO HANDS");
    else
        std::snprintf(hands_text, sizeof(hands_text), "HANDS %d", hands_seen);

    std::string sig = clock_str;
    char num[96];
    std::snprintf(num, sizeof(num), "|%s|%s|%d|%s", conn_text, trk_text,
                  calib_pending ? 1 : 0, show_hands ? hands_text : "");
    sig += num;
    for (const auto &p : panels) {
        sig += "|" + p.title;
        sig += p.focused ? "*" : "";
    }

    if (sig != _dock_sig) {
        _dock_sig = sig;
        auto &surf = _dock_surface;
        surf.fill({0, 0, 0, 0});

        // Floating bar, inset from the window edges.
        const float bar_x = 10, bar_y = 10;
        const float bar_w = (float)surf.width() - 20;
        const float bar_h = (float)surf.height() - 20;
        const uint8_t surf_a = (uint8_t)std::lround(255.0f * SURFACE_ALPHA);
        surf.fill_rounded_rect_vgrad(
            bar_x, bar_y, bar_w, bar_h, (float)RADIUS_LG,
            {SURFACE.r, SURFACE.g, SURFACE.b, surf_a},
            {BG.r, BG.g, BG.b, surf_a});
        surf.stroke_rounded_rect(
            bar_x, bar_y, bar_w, bar_h, (float)RADIUS_LG, 1.2f,
            {BORDER.r, BORDER.g, BORDER.b,
             (uint8_t)std::lround(255.0f * BORDER_ALPHA)});

        const int mid_y = surf.height() / 2;
        const int text_y = mid_y - panel_surface::glyph_height(1) / 2;

        // Wordmark: accent tick + VANTAGE.
        int x = (int)bar_x + 18;
        surf.fill_rounded_rect((float)x, (float)mid_y - 7, 4.0f, 14.0f, 2.0f,
                               ACCENT);
        x += 12;
        x += surf.draw_text(x, text_y, 1, "VANTAGE", FG_MUTED);
        x += 22;

        // Connection chip: colored dot + plain wording in a quiet pill.
        {
            int chip_w = 16 + 14 + panel_surface::text_width(conn_text, 1) +
                         16;
            const int chip_h = 42;
            surf.fill_rounded_rect((float)x, (float)(mid_y - chip_h / 2),
                                   (float)chip_w, (float)chip_h,
                                   (float)RADIUS_MD,
                                   {SURFACE.r, SURFACE.g, SURFACE.b, 160});
            surf.stroke_rounded_rect(
                (float)x, (float)(mid_y - chip_h / 2), (float)chip_w,
                (float)chip_h, (float)RADIUS_MD, 1.0f,
                {conn_dot.r, conn_dot.g, conn_dot.b, 90});
            surf.fill_circle((float)(x + 18), (float)mid_y, 4.5f, conn_dot);
            surf.draw_text(x + 30, text_y, 1, conn_text, FG_MUTED);
            x += chip_w + 14;
        }

        // Calibration-pending chip, same budget the panel entries respect so
        // it can never run into the right-hand status cluster.
        if (calib_pending) {
            const char *calib_text = "CALIBRATING";
            int chip_w =
                16 + 14 + panel_surface::text_width(calib_text, 1) + 16;
            const int chip_h = 42;
            if (x + chip_w <= surf.width() - 380) {
                surf.fill_rounded_rect((float)x, (float)(mid_y - chip_h / 2),
                                       (float)chip_w, (float)chip_h,
                                       (float)RADIUS_MD,
                                       {SURFACE.r, SURFACE.g, SURFACE.b, 160});
                surf.stroke_rounded_rect(
                    (float)x, (float)(mid_y - chip_h / 2), (float)chip_w,
                    (float)chip_h, (float)RADIUS_MD, 1.0f,
                    {WARNING.r, WARNING.g, WARNING.b, 90});
                surf.fill_circle((float)(x + 18), (float)mid_y, 4.5f, WARNING);
                surf.draw_text(x + 30, text_y, 1, calib_text, FG_MUTED);
                x += chip_w + 14;
            }
        }

        if (show_hands) {
            color_rgba hd = hands_seen ? SUCCESS : NEUTRAL;
            int chip_w =
                16 + 14 + panel_surface::text_width(hands_text, 1) + 16;
            const int chip_h = 42;
            if (x + chip_w <= surf.width() - 380) {
                surf.fill_rounded_rect((float)x, (float)(mid_y - chip_h / 2),
                                       (float)chip_w, (float)chip_h,
                                       (float)RADIUS_MD,
                                       {SURFACE.r, SURFACE.g, SURFACE.b, 160});
                surf.stroke_rounded_rect(
                    (float)x, (float)(mid_y - chip_h / 2), (float)chip_w,
                    (float)chip_h, (float)RADIUS_MD, 1.0f,
                    {hd.r, hd.g, hd.b, 90});
                surf.fill_circle((float)(x + 18), (float)mid_y, 4.5f, hd);
                surf.draw_text(x + 30, text_y, 1, hands_text, FG_MUTED);
                x += chip_w + 14;
            }
        }

        // Panel entries: quiet pills; the focused one carries an accent dot
        // and brighter ink.
        _s->dock_hits.clear();
        const int entry_w = 150, entry_h = 42, gap = 10;
        for (const auto &p : panels) {
            if (x + entry_w > surf.width() - 380)
                break;  // leave room for the status cluster
            color_rgba fill = p.focused
                                  ? color_rgba{SURFACE_PRESSED.r,
                                               SURFACE_PRESSED.g,
                                               SURFACE_PRESSED.b, 255}
                                  : color_rgba{SURFACE.r, SURFACE.g,
                                               SURFACE.b, 160};
            surf.fill_rounded_rect((float)x, (float)(mid_y - entry_h / 2),
                                   (float)entry_w, (float)entry_h,
                                   (float)RADIUS_MD, fill);
            if (p.focused)
                surf.stroke_rounded_rect((float)x,
                                         (float)(mid_y - entry_h / 2),
                                         (float)entry_w, (float)entry_h,
                                         (float)RADIUS_MD, 1.2f,
                                         {ACCENT.r, ACCENT.g, ACCENT.b, 140});
            int tx = x + 16;
            if (p.focused) {
                surf.fill_circle((float)(x + 16), (float)mid_y, 4.0f, ACCENT);
                tx += 14;
            }
            std::string label = p.title.empty() ? "panel" : p.title;
            size_t max_chars =
                (size_t)((entry_w - (tx - x) - 10) /
                         panel_surface::glyph_width(1));
            if (label.size() > max_chars && max_chars > 1)
                label = label.substr(0, max_chars - 1) + "~";
            surf.draw_text(tx, text_y, 1, label, p.focused ? FG : FG_MUTED);
            mac_shell::dock_entry_hit hit;
            hit.x0 = (float)x / (float)surf.width();
            hit.x1 = (float)(x + entry_w) / (float)surf.width();
            hit.handle = p.handle;
            _s->dock_hits.push_back(hit);
            x += entry_w + gap;
        }

        // Status cluster, right-aligned: "?" help pill, clock, tracking
        // dot + wording.
        int rx = surf.width() - (int)bar_x - 18;
        {
            const int help_w = 42, help_h = 42;
            rx -= help_w;
            surf.fill_rounded_rect((float)rx, (float)(mid_y - help_h / 2),
                                   (float)help_w, (float)help_h,
                                   (float)RADIUS_MD,
                                   {SURFACE.r, SURFACE.g, SURFACE.b, 200});
            surf.stroke_rounded_rect(
                (float)rx, (float)(mid_y - help_h / 2), (float)help_w,
                (float)help_h, (float)RADIUS_MD, 1.0f,
                {BORDER.r, BORDER.g, BORDER.b,
                 (uint8_t)std::lround(255.0f * BORDER_ALPHA)});
            surf.draw_text(
                rx + (help_w - panel_surface::glyph_width(1)) / 2, text_y, 1,
                "?", FG);
            _s->dock_help_x0 = (float)rx / (float)surf.width();
            _s->dock_help_x1 = (float)(rx + help_w) / (float)surf.width();
            rx -= 18;
        }
        rx -= panel_surface::text_width(clock_str, 1);
        surf.draw_text(rx, text_y, 1, clock_str, FG);
        rx -= 20;
        surf.fill_circle((float)rx, (float)mid_y, 5.0f, trk);
        rx -= 12;
        rx -= panel_surface::text_width(trk_text, 1);
        surf.draw_text(rx, text_y, 1, trk_text, FG_MUTED);

        surf.apply_grain(GRAIN_ALPHA);

        if (!_dock_tex)
            _dock_tex = [self makeMippedTextureW:(NSUInteger)surf.width()
                                               h:(NSUInteger)surf.height()];
        [self uploadTexture:_dock_tex
                       rgba:surf.pixels()
                          w:(NSUInteger)surf.width()
                          h:(NSUInteger)surf.height()];
    }
    if (!_dock_tex)
        return;

    float draw_h = (float)std::fmax(1.0, view.drawableSize.height);
    float h_ndc = 2.0f * (float)mac_shell::DOCK_TEX_H / draw_h;
    _s->dock_h_frac = h_ndc * 0.5f;
    mac_shell::panel_vertex verts[6] = {
        {{-1, -1, 0}, {0, 1}},          {{1, -1, 0}, {1, 1}},
        {{1, -1 + h_ndc, 0}, {1, 0}},   {{-1, -1, 0}, {0, 1}},
        {{1, -1 + h_ndc, 0}, {1, 0}},   {{-1, -1 + h_ndc, 0}, {0, 0}},
    };
    mac_shell::panel_uniforms pu;
    std::memset(&pu, 0, sizeof(pu));
    pu.mvp = matrix_identity_float4x4;
    pu.alpha = 1.0f;
    [enc setRenderPipelineState:_blit_pipeline];
    [enc setDepthStencilState:_depth_none];
    [enc setVertexBytes:verts length:sizeof(verts) atIndex:0];
    [enc setVertexBytes:&pu length:sizeof(pu) atIndex:1];
    [enc setFragmentBytes:&pu length:sizeof(pu) atIndex:0];
    [enc setFragmentTexture:_dock_tex atIndex:0];
    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:6];
}

@end
