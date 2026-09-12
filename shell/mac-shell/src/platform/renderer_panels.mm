// renderer_panels.mm — ShellRenderer panel path: texture cache, spring
// animation, analytic shadow + SDF rounded draw.

#include "platform/renderer_internal.h"

@implementation ShellRenderer (Panels)

- (id<MTLTexture>)textureForPanel:(const mac_shell::scene::render_panel &)rp {
    if (rp.kind == mac_shell::panel_kind::captured_window) {
        if (!_s->capture)
            return nil;
        CVPixelBufferRef pb = (CVPixelBufferRef)
            _s->capture->copy_latest_pixel_buffer(rp.handle);
        if (!pb) {
            auto it = _panel_textures.find(rp.handle);
            return it != _panel_textures.end() ? it->second.tex : nil;
        }
        CVMetalTextureRef cv_tex = nil;
        size_t w = CVPixelBufferGetWidth(pb);
        size_t h = CVPixelBufferGetHeight(pb);
        CVReturn rc = CVMetalTextureCacheCreateTextureFromImage(
            kCFAllocatorDefault, _tex_cache, pb, nil, MTLPixelFormatBGRA8Unorm,
            w, h, 0, &cv_tex);
        id<MTLTexture> tex = nil;
        if (rc == kCVReturnSuccess && cv_tex) {
            tex = CVMetalTextureGetTexture(cv_tex);
            auto &entry = _panel_textures[rp.handle];
            if (entry.cv_tex)
                _frame_cf_release.push_back(entry.cv_tex);
            entry = {tex, 0, cv_tex};
        }
        // The command buffer reading this IOSurface commits later in the
        // frame; the completion handler drops the reference.
        _frame_cf_release.push_back(pb);
        return tex ? tex
                   : (_panel_textures.count(rp.handle)
                          ? _panel_textures[rp.handle].tex
                          : nil);
    }

    auto it = _panel_textures.find(rp.handle);
    bool needs_upload = it == _panel_textures.end() ||
                        it->second.version != rp.surface_version ||
                        it->second.tex.width != (NSUInteger)rp.width_px ||
                        it->second.tex.height != (NSUInteger)rp.height_px;
    if (!needs_upload)
        return it->second.tex;
    if (rp.rgba.empty())
        return it != _panel_textures.end() ? it->second.tex : nil;

    id<MTLTexture> tex =
        (it != _panel_textures.end() &&
         it->second.tex.width == (NSUInteger)rp.width_px &&
         it->second.tex.height == (NSUInteger)rp.height_px)
            ? it->second.tex
            : nil;
    if (!tex)
        tex = [self makeMippedTextureW:(NSUInteger)rp.width_px
                                     h:(NSUInteger)rp.height_px];
    _perf.enter();
    [self uploadTexture:tex
                   rgba:rp.rgba.data()
                      w:(NSUInteger)rp.width_px
                      h:(NSUInteger)rp.height_px];
    _perf.leave();
    _panel_textures[rp.handle] = {tex, rp.surface_version};
    return tex;
}

// ------------------------------------------------------------------
// panels: spring animation + shadow + SDF draw
// ------------------------------------------------------------------

// Compose the animated model matrix: keep the scene's rotation basis,
// scale it, spring the translation, and add the grab lift/tilt.
- (simd_float4x4)animatedModelFor:(const mac_shell::scene::render_panel &)rp
                             anim:(mac_shell::panel_anim &)a {
    simd_float4x4 m = mac_shell::model_from_rows(rp.m);
    float s = std::fmax(0.01f, a.scale.position) *
              (1.0f + 0.02f * a.glow.position);

    simd_float3 right = simd_make_float3(m.columns[0].x, m.columns[0].y,
                                         m.columns[0].z);
    simd_float3 up = simd_make_float3(m.columns[1].x, m.columns[1].y,
                                      m.columns[1].z);
    simd_float3 fwd = simd_make_float3(m.columns[2].x, m.columns[2].y,
                                       m.columns[2].z);

    // Grab: tilt back slightly around the panel's local X and lift toward
    // the viewer along its normal.
    float lift = a.lift.position;
    if (lift > 0.001f) {
        float ang = mac_shell::GRAB_TILT_RAD * lift;
        float c = std::cos(ang), sn = std::sin(ang);
        simd_float3 up2 = up * c + fwd * sn;
        simd_float3 fwd2 = fwd * c - up * sn;
        up = up2;
        fwd = fwd2;
    }

    simd_float4x4 out;
    out.columns[0] = simd_make_float4(right * s, 0);
    out.columns[1] = simd_make_float4(up * s, 0);
    out.columns[2] = simd_make_float4(fwd * s, 0);
    simd_float3 t = simd_make_float3(a.tx.position, a.ty.position,
                                     a.tz.position) +
                    fwd * (lift * mac_shell::GRAB_LIFT_M);
    out.columns[3] = simd_make_float4(t, 1);
    return out;
}

- (void)drawPanel:(const mac_shell::scene::render_panel &)rp
             anim:(mac_shell::panel_anim &)a
          texture:(id<MTLTexture>)tex
          encoder:(id<MTLRenderCommandEncoder>)enc
        viewProj:(simd_float4x4)view_proj
             time:(float)t {
    float hw = rp.width_m * 0.5f, hh = rp.height_m * 0.5f;
    simd_float4x4 model = [self animatedModelFor:rp anim:a];
    simd_float4x4 mvp = simd_mul(view_proj, model);

    mac_shell::panel_uniforms u;
    std::memset(&u, 0, sizeof(u));
    u.half_size = simd_make_float2(hw, hh);
    u.corner_radius =
        std::fmin(mac_shell::theme::PANEL_RADIUS_M, std::fmin(hw, hh) * 0.5f);
    u.focus_glow = std::fmax(0.0f, a.glow.position);
    u.dim = std::fmax(0.0f, 1.0f - std::fmax(a.glow.position, 0.0f)) * 0.5f;
    u.alpha = mac_shell::clamp01(a.alpha.position);
    u.time = t;

    // Shadow: a larger quad slightly behind the panel, offset down-right in
    // panel-local space; grabbed panels cast a wider, softer shadow.
    {
        float grow = 1.0f + 0.16f + 0.1f * a.lift.position;
        float shw = hw * grow, shh = hh * grow;
        float ox = 0.010f, oy = -0.016f - 0.01f * a.lift.position;
        mac_shell::panel_vertex sverts[6] = {
            {{-shw + ox, -shh + oy, -0.006f}, {0, 1}},
            {{shw + ox, -shh + oy, -0.006f}, {1, 1}},
            {{shw + ox, shh + oy, -0.006f}, {1, 0}},
            {{-shw + ox, -shh + oy, -0.006f}, {0, 1}},
            {{shw + ox, shh + oy, -0.006f}, {1, 0}},
            {{-shw + ox, shh + oy, -0.006f}, {0, 0}},
        };
        mac_shell::panel_uniforms su = u;
        su.mvp = mvp;
        su.half_size = simd_make_float2(hw, hh);
        su.shadow = 0.38f + 0.1f * a.lift.position;
        [enc setRenderPipelineState:_shadow_pipeline];
        [enc setDepthStencilState:_depth_read];
        [enc setVertexBytes:sverts length:sizeof(sverts) atIndex:0];
        [enc setVertexBytes:&su length:sizeof(su) atIndex:1];
        [enc setFragmentBytes:&su length:sizeof(su) atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle
                vertexStart:0
                vertexCount:6];
    }

    mac_shell::panel_vertex verts[6] = {
        {{-hw, -hh, 0}, {0, 1}}, {{hw, -hh, 0}, {1, 1}},
        {{hw, hh, 0}, {1, 0}},   {{-hw, -hh, 0}, {0, 1}},
        {{hw, hh, 0}, {1, 0}},   {{-hw, hh, 0}, {0, 0}},
    };
    u.mvp = mvp;
    [enc setRenderPipelineState:_panel_pipeline];
    [enc setDepthStencilState:_depth_test];
    [enc setVertexBytes:verts length:sizeof(verts) atIndex:0];
    [enc setVertexBytes:&u length:sizeof(u) atIndex:1];
    [enc setFragmentBytes:&u length:sizeof(u) atIndex:0];
    [enc setFragmentBytes:&_occl length:sizeof(_occl) atIndex:1];
    [enc setFragmentTexture:tex atIndex:0];
    [enc setFragmentTexture:(_occl.band_m > 0.0f && _depth_map_tex)
                                ? _depth_map_tex
                                : _depth_map_dummy
                    atIndex:1];
    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:6];
}

- (void)drawPanels:(std::vector<mac_shell::scene::render_panel> &)panels
           encoder:(id<MTLRenderCommandEncoder>)enc
          viewProj:(simd_float4x4)view_proj
              view:(simd_float4x4)view_mat
                dt:(float)dt
              time:(float)t {
    using mac_shell::panel_anim;
    using namespace mac_shell::theme;

    // Aim feedback rides the focus glow at a partial level: the pinch
    // candidate gets the accent border and rim without claiming to be
    // focused, so the user sees the target before committing to the pinch.
    constexpr float AIM_GLOW = 0.45f;
    auto glow_target = [](const mac_shell::scene::render_panel &rp) {
        return rp.focused ? 1.0f : (rp.aimed ? AIM_GLOW : 0.0f);
    };

    // Advance / create animation state for live panels.
    for (auto &rp : panels) {
        auto it = _panel_anims.find(rp.handle);
        if (it == _panel_anims.end()) {
            panel_anim a;
            a.tx.init(rp.m[12], SPRING_MOVE_STIFFNESS, SPRING_MOVE_DAMPING);
            a.ty.init(rp.m[13], SPRING_MOVE_STIFFNESS, SPRING_MOVE_DAMPING);
            a.tz.init(rp.m[14], SPRING_MOVE_STIFFNESS, SPRING_MOVE_DAMPING);
            a.scale.init(0.85f, SPRING_OPEN_STIFFNESS, SPRING_OPEN_DAMPING);
            a.scale.target = 1.0f;
            a.alpha.init(0.0f, SPRING_OPEN_STIFFNESS, SPRING_OPEN_DAMPING);
            a.alpha.target = 1.0f;
            a.glow.init(glow_target(rp), SPRING_OPEN_STIFFNESS,
                        SPRING_OPEN_DAMPING);
            a.lift.init(0.0f, SPRING_SNAP_STIFFNESS, SPRING_SNAP_DAMPING);
            it = _panel_anims.emplace(rp.handle, std::move(a)).first;
        }
        panel_anim &a = it->second;
        a.closing = false;
        a.tx.target = rp.m[12];
        a.ty.target = rp.m[13];
        a.tz.target = rp.m[14];
        a.glow.target = glow_target(rp);
        a.lift.target = rp.grabbed ? 1.0f : 0.0f;
        a.scale.target = 1.0f;
        a.alpha.target = 1.0f;
    }

    // Panels that vanished start their close animation on cached state.
    for (auto &kv : _panel_anims) {
        bool live = false;
        for (auto &rp : panels)
            if (rp.handle == kv.first)
                live = true;
        if (!live && !kv.second.closing) {
            kv.second.closing = true;
            kv.second.scale.stiffness = SPRING_CLOSE_STIFFNESS;
            kv.second.scale.damping = SPRING_CLOSE_DAMPING;
            kv.second.scale.target = 0.9f;
            kv.second.alpha.stiffness = SPRING_CLOSE_STIFFNESS;
            kv.second.alpha.damping = SPRING_CLOSE_DAMPING;
            kv.second.alpha.target = 0.0f;
            auto tex_it = _panel_textures.find(kv.first);
            if (tex_it != _panel_textures.end())
                kv.second.cached_tex = tex_it->second.tex;
        }
    }

    // Step springs.
    for (auto &kv : _panel_anims) {
        mac_shell::panel_anim &a = kv.second;
        a.tx.step(dt);
        a.ty.step(dt);
        a.tz.step(dt);
        a.scale.step(dt);
        a.alpha.step(dt);
        a.glow.step(dt);
        a.lift.step(dt);
    }

    // Drop close animations that have faded out.
    for (auto it = _panel_anims.begin(); it != _panel_anims.end();) {
        if (it->second.closing && it->second.alpha.position < 0.02f)
            it = _panel_anims.erase(it);
        else
            ++it;
    }

    // Draw order: translucent panels back-to-front by animated view z.
    struct draw_item {
        const mac_shell::scene::render_panel *rp;
        mac_shell::panel_anim *anim;
        id<MTLTexture> tex;
        float vz;
    };
    std::vector<draw_item> items;
    simd_float4 vrow2 = simd_make_float4(
        view_mat.columns[0].z, view_mat.columns[1].z, view_mat.columns[2].z,
        view_mat.columns[3].z);
    auto view_z = [&](const mac_shell::panel_anim &a) {
        return vrow2.x * a.tx.position + vrow2.y * a.ty.position +
               vrow2.z * a.tz.position + vrow2.w;
    };
    for (auto &rp : panels) {
        auto it = _panel_anims.find(rp.handle);
        if (it == _panel_anims.end())
            continue;
        id<MTLTexture> tex = [self textureForPanel:rp];
        if (!tex)
            continue;
        // Refresh the cached snapshot for a potential close animation; the
        // pixels are already on the GPU, so drop them before the copy.
        std::vector<uint8_t>().swap(rp.rgba);
        it->second.cached = rp;
        items.push_back({&rp, &it->second, tex, view_z(it->second)});
    }
    for (auto &kv : _panel_anims) {
        if (!kv.second.closing || !kv.second.cached_tex)
            continue;
        items.push_back({&kv.second.cached, &kv.second, kv.second.cached_tex,
                         view_z(kv.second)});
    }
    std::sort(items.begin(), items.end(),
              [](const draw_item &a, const draw_item &b) {
                  return a.vz < b.vz;  // farther (more negative) first
              });
    for (auto &item : items)
        [self drawPanel:*item.rp
                   anim:*item.anim
                texture:item.tex
                encoder:enc
               viewProj:view_proj
                   time:t];
}

@end
