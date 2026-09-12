// renderer_keyboard.mm — ShellRenderer virtual keyboard: glass plate +
// instanced keycaps + plane shadow.

#include "platform/renderer_internal.h"

@implementation ShellRenderer (Keyboard)

- (void)drawKeyboard:(id<MTLRenderCommandEncoder>)enc
            keyboard:(const mac_shell::keyboard_render_state &)kb
            viewProj:(simd_float4x4)view_proj
                  dt:(float)dt
                time:(float)t {
    float show = _kbd_anim.tick(kb.visible, dt);
    if (show < 0.02f || kb.width_m <= 0)
        return;

    // rgba is empty when the snapshot saw our cached version and skipped the
    // copy; the texture on hand is then already current.
    if (!kb.rgba.empty() &&
        (!_kbd_tex || _kbd_tex_version != kb.version ||
         _kbd_tex.width != (NSUInteger)kb.surface_w ||
         _kbd_tex.height != (NSUInteger)kb.surface_h)) {
        if (!_kbd_tex || _kbd_tex.width != (NSUInteger)kb.surface_w ||
            _kbd_tex.height != (NSUInteger)kb.surface_h)
            _kbd_tex = [self makeMippedTextureW:(NSUInteger)kb.surface_w
                                              h:(NSUInteger)kb.surface_h];
        _perf.enter();
        [self uploadTexture:_kbd_tex
                       rgba:kb.rgba.data()
                          w:(NSUInteger)kb.surface_w
                          h:(NSUInteger)kb.surface_h];
        _perf.leave();
        _kbd_tex_version = kb.version;
    }
    if (!_kbd_tex)
        return;

    const auto &pl = kb.plane;
    simd_float3 right = simd_make_float3(pl.right[0], pl.right[1],
                                         pl.right[2]);
    simd_float3 down = simd_make_float3(pl.down[0], pl.down[1], pl.down[2]);
    simd_float3 normal = simd_make_float3(pl.normal[0], pl.normal[1],
                                          pl.normal[2]);
    simd_float3 origin = simd_make_float3(pl.origin[0], pl.origin[1],
                                          pl.origin[2]);
    simd_float3 centre = origin + right * (kb.width_m * 0.5f) +
                         down * (kb.height_m * 0.5f);

    // Appear: the plate rises from the plane while fading in.
    float rise = (1.0f - show) * -0.02f;
    float plate_lift = mac_shell::KBD_PLATE_LIFT_M * show;

    // Plane-local model matrix (x = right, y = down, z = normal).
    auto plane_model = [&](simd_float3 at) {
        simd_float4x4 m;
        m.columns[0] = simd_make_float4(right, 0);
        m.columns[1] = simd_make_float4(down, 0);
        m.columns[2] = simd_make_float4(normal, 0);
        m.columns[3] = simd_make_float4(at, 1);
        return m;
    };

    float phw = kb.width_m * 0.5f + mac_shell::KBD_PLATE_MARGIN_M;
    float phh = kb.height_m * 0.5f + mac_shell::KBD_PLATE_MARGIN_M;
    mac_shell::panel_vertex quad[6] = {
        {{-phw, -phh, 0}, {0, 0}}, {{phw, -phh, 0}, {1, 0}},
        {{phw, phh, 0}, {1, 1}},   {{-phw, -phh, 0}, {0, 0}},
        {{phw, phh, 0}, {1, 1}},   {{-phw, phh, 0}, {0, 1}},
    };

    // 1. soft shadow on the anchor plane under the plate.
    {
        mac_shell::panel_uniforms u;
        std::memset(&u, 0, sizeof(u));
        u.mvp = simd_mul(view_proj,
                         plane_model(centre + normal * 0.0005f));
        u.half_size = simd_make_float2(phw * 0.96f, phh * 0.96f);
        u.corner_radius = 0.02f;
        u.alpha = show;
        u.shadow = 0.4f;
        [enc setRenderPipelineState:_shadow_pipeline];
        [enc setDepthStencilState:_depth_read];
        [enc setVertexBytes:quad length:sizeof(quad) atIndex:0];
        [enc setVertexBytes:&u length:sizeof(u) atIndex:1];
        [enc setFragmentBytes:&u length:sizeof(u) atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle
                vertexStart:0
                vertexCount:6];
    }

    // 2. glass base plate.
    {
        mac_shell::panel_uniforms u;
        std::memset(&u, 0, sizeof(u));
        u.mvp = simd_mul(
            view_proj,
            plane_model(centre + normal * (plate_lift + rise)));
        u.half_size = simd_make_float2(phw, phh);
        u.corner_radius = 0.016f;
        u.alpha = show;
        u.grain = mac_shell::theme::GRAIN_ALPHA;
        u.time = t;
        [enc setRenderPipelineState:_glass_pipeline];
        [enc setDepthStencilState:_depth_test];
        [enc setVertexBytes:quad length:sizeof(quad) atIndex:0];
        [enc setVertexBytes:&u length:sizeof(u) atIndex:1];
        [enc setFragmentBytes:&u length:sizeof(u) atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle
                vertexStart:0
                vertexCount:6];
    }

    // 3. instanced keycaps above the plate.
    if (!kb.keys.empty()) {
        std::vector<mac_shell::key_instance> inst;
        inst.reserve(kb.keys.size());
        for (const auto &k : kb.keys) {
            mac_shell::key_instance ki;
            ki.center = simd_make_float2(k.cx_m, k.cy_m);
            ki.half_size = simd_make_float2(k.hw_m, k.hh_m);
            ki.press = k.press;
            ki.hold = k.held ? 1.0f : 0.0f;
            ki.pad = simd_make_float2(0, 0);
            inst.push_back(ki);
        }
        mac_shell::kbd_uniforms u;
        std::memset(&u, 0, sizeof(u));
        u.view_proj = view_proj;
        simd_float3 key_origin =
            origin + normal * (plate_lift + rise);
        u.origin = simd_make_float4(key_origin, 0);
        u.right = simd_make_float4(right, 0);
        u.down = simd_make_float4(down, 0);
        u.normal = simd_make_float4(normal, 0);
        u.plane_size = simd_make_float2(kb.width_m, kb.height_m);
        u.lift = mac_shell::KBD_KEY_LIFT_M;
        u.time = t;
        [enc setRenderPipelineState:_key_pipeline];
        [enc setDepthStencilState:_depth_test];
        [enc setVertexBytes:inst.data()
                     length:sizeof(mac_shell::key_instance) * inst.size()
                    atIndex:0];
        [enc setVertexBytes:&u length:sizeof(u) atIndex:1];
        [enc setFragmentBytes:&u length:sizeof(u) atIndex:0];
        [enc setFragmentTexture:_kbd_tex atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle
                vertexStart:0
                vertexCount:6
              instanceCount:inst.size()];
    }
}

@end
