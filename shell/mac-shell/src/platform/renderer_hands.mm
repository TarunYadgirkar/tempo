// renderer_hands.mm — Hand skeleton, fingertip reticles and summon arc.

#include "platform/renderer_internal.h"

namespace {
float joint_alpha(const float *joint) {
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(joint[i]))
            return 0.0f;
    }
    return mac_shell::clamp01((joint[3] - 0.15f) / 0.60f);
}

simd_float3 hand_tint(int slot) {
    return slot == 0 ? simd_make_float3(0.20f, 0.68f, 0.90f)
                     : simd_make_float3(0.98f, 0.80f, 0.10f);
}
}  // namespace

@implementation ShellRenderer (Hands)

- (void)drawHands:(id<MTLRenderCommandEncoder>)enc
         keyboard:(const mac_shell::keyboard_render_state &)kb
         viewProj:(simd_float4x4)view_proj
             view:(simd_float4x4)view_mat
               dt:(float)dt
             time:(float)t {
    mac_shell::hand_uniforms hu;
    std::memset(&hu, 0, sizeof(hu));
    hu.view_proj = view_proj;
    hu.cam_right = simd_make_float4(view_mat.columns[0].x,
                                    view_mat.columns[1].x,
                                    view_mat.columns[2].x, 0);
    hu.cam_up = simd_make_float4(view_mat.columns[0].y, view_mat.columns[1].y,
                                 view_mat.columns[2].y, 0);
    hu.view_row2 = simd_make_float4(view_mat.columns[0].z,
                                    view_mat.columns[1].z,
                                    view_mat.columns[2].z,
                                    view_mat.columns[3].z);
    float zs = mac_shell::RENDER_FAR_Z /
               (mac_shell::RENDER_NEAR_Z - mac_shell::RENDER_FAR_Z);
    hu.depth_consts = simd_make_float2(zs, mac_shell::RENDER_NEAR_Z * zs);
    hu.time = t;

    auto contact_glow = [&](const float j[3]) -> float {
        if (!kb.visible)
            return 0.0f;
        const auto &pl = kb.plane;
        float d[3] = {j[0] - pl.origin[0], j[1] - pl.origin[1],
                      j[2] - pl.origin[2]};
        float x = d[0] * pl.right[0] + d[1] * pl.right[1] + d[2] * pl.right[2];
        float y = d[0] * pl.down[0] + d[1] * pl.down[1] + d[2] * pl.down[2];
        float z = d[0] * pl.normal[0] + d[1] * pl.normal[1] +
                  d[2] * pl.normal[2];
        if (x < -0.03f || x > kb.width_m + 0.03f || y < -0.03f ||
            y > kb.height_m + 0.03f)
            return 0.0f;
        float g = 1.0f - std::fabs(z) / 0.035f;
        return mac_shell::clamp01(g);
    };

    if (!_s->world->hand_overlay()) {
        [self drawHandsMinimal:enc
                      uniforms:hu
                            dt:dt
                  contactGlow:contact_glow];
        return;
    }

    // ---- hand overlay on: full 21-joint skeleton ----
    for (int slot = 0; slot < 2; slot++) {
        sb_hand_t hand;
        if (!_s->world->hand_joints(slot, hand))
            continue;

        // Bones as capsules. Anatomically impossible spans (tracking
        // glitches, synthetic fixtures) are dropped rather than drawn as
        // arm-length rods.
        std::vector<mac_shell::capsule_instance> caps;
        caps.reserve(mac_shell::N_BONES);
        const simd_float3 bone_tint = hand_tint(slot);
        for (int b = 0; b < mac_shell::N_BONES; b++) {
            const float *a = hand.joints[mac_shell::BONES[b][0]];
            const float *c = hand.joints[mac_shell::BONES[b][1]];
            const float alpha = std::min(joint_alpha(a), joint_alpha(c));
            if (alpha <= 0.0f)
                continue;
            float dx = c[0] - a[0], dy = c[1] - a[1], dz = c[2] - a[2];
            if (dx * dx + dy * dy + dz * dz > 0.15f * 0.15f)
                continue;
            mac_shell::capsule_instance ci;
            ci.a_radius = simd_make_float4(a[0], a[1], a[2], 0.0018f);
            ci.b = simd_make_float4(c[0], c[1], c[2], 0);
            ci.color = simd_make_float4(bone_tint, 0.45f * alpha);
            caps.push_back(ci);
        }
        if (!caps.empty()) {
            [enc setRenderPipelineState:_capsule_pipeline];
            [enc setDepthStencilState:_depth_read];
            [enc setVertexBytes:caps.data()
                         length:sizeof(mac_shell::capsule_instance) *
                                caps.size()
                        atIndex:0];
            [enc setVertexBytes:&hu length:sizeof(hu) atIndex:1];
            [enc setFragmentBytes:&hu length:sizeof(hu) atIndex:0];
            [enc drawPrimitives:MTLPrimitiveTypeTriangle
                    vertexStart:0
                    vertexCount:6
                  instanceCount:caps.size()];
        }

        mac_shell::sphere_instance spheres[SB_HAND_JOINT_COUNT];
        int count = 0;
        for (int j = 0; j < SB_HAND_JOINT_COUNT; j++) {
            const float *joint = hand.joints[j];
            const float alpha = joint_alpha(joint);
            if (mac_shell::is_fingertip(j) || alpha <= 0.0f)
                continue;
            spheres[count].pos_radius =
                simd_make_float4(joint[0], joint[1], joint[2], 0.0028f);
            spheres[count].color = simd_make_float4(bone_tint, 0.55f * alpha);
            spheres[count].glow = simd_make_float4(0, 0, 0, 0);
            ++count;
        }
        if (count == 0)
            continue;
        [enc setRenderPipelineState:_sphere_pipeline];
        [enc setDepthStencilState:_depth_read];
        [enc setVertexBytes:spheres length:sizeof(spheres[0]) * count atIndex:0];
        [enc setVertexBytes:&hu length:sizeof(hu) atIndex:1];
        [enc setFragmentBytes:&hu length:sizeof(hu) atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle
                vertexStart:0
                vertexCount:6
              instanceCount:count];
    }
    [self drawHandsMinimal:enc uniforms:hu dt:dt contactGlow:contact_glow];
}

- (void)drawHandsMinimal:(id<MTLRenderCommandEncoder>)enc
                uniforms:(mac_shell::hand_uniforms)hu
                      dt:(float)dt
             contactGlow:(const std::function<float(const float *)> &)contact {
    using namespace mac_shell;
    for (int slot = 0; slot < 2; slot++) {
        sb_hand_t hand;
        if (!_s->world->hand_joints(slot, hand))
            continue;
        sphere_instance tips[5];
        int count = 0;
        for (int i = 0; i < 5; i++) {
            const float *joint = hand.joints[FINGERTIPS[i]];
            const float alpha = joint_alpha(joint);
            if (alpha <= 0.0f)
                continue;
            // Reticle marks the exact point tested by the typing decoder.
            tips[count].pos_radius =
                simd_make_float4(joint[0], joint[1], joint[2], 0.0065f);
            tips[count].color = simd_make_float4(hand_tint(slot), 0.95f * alpha);
            tips[count].glow = simd_make_float4(contact(joint), 0, 0, 0);
            ++count;
        }
        if (count == 0)
            continue;
        [enc setRenderPipelineState:_glow_pipeline];
        [enc setDepthStencilState:_depth_read];
        [enc setVertexBytes:tips length:sizeof(tips[0]) * count atIndex:0];
        [enc setVertexBytes:&hu length:sizeof(hu) atIndex:1];
        [enc setFragmentBytes:&hu length:sizeof(hu) atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle
                vertexStart:0
                vertexCount:6
              instanceCount:count];
    }
}

// ------------------------------------------------------------------
// HUD: summon arc, launcher, dock
// ------------------------------------------------------------------

- (void)drawSummonArcs:(id<MTLRenderCommandEncoder>)enc
              viewProj:(simd_float4x4)view_proj
                  view:(simd_float4x4)view_mat {
    mac_shell::scene::hud_arc arcs[2];
    _s->world->hud_arcs(arcs);
    simd_float3 cam_right = simd_make_float3(view_mat.columns[0].x,
                                             view_mat.columns[1].x,
                                             view_mat.columns[2].x);
    simd_float3 cam_up = simd_make_float3(view_mat.columns[0].y,
                                          view_mat.columns[1].y,
                                          view_mat.columns[2].y);
    for (const auto &arc : arcs) {
        // Same visibility band as wxrd's hand_anchor_progress: hidden until
        // the hold is clearly deliberate.
        if (!arc.active || arc.progress < 0.20f)
            continue;
        constexpr int SEGS = 48;
        constexpr float R = 0.05f;
        constexpr float W = 0.007f;  // ring thickness
        simd_float3 center =
            simd_make_float3(arc.pos[0], arc.pos[1] + 0.10f, arc.pos[2]);

        auto ring = [&](float t0, float t1, simd_float4 color) {
            simd_float3 verts[(SEGS + 1) * 2];
            int n = 0;
            for (int i = 0; i <= SEGS; i++) {
                float f = t0 + (t1 - t0) * (float)i / (float)SEGS;
                float a = -0.5f * (float)M_PI + f * 2.0f * (float)M_PI;
                simd_float3 dir = cam_right * std::cos(a) +
                                  cam_up * std::sin(a);
                verts[n++] = center + dir * (R - W * 0.5f);
                verts[n++] = center + dir * (R + W * 0.5f);
            }
            mac_shell::solid_uniforms su;
            su.mvp = view_proj;
            su.color = color;
            [enc setRenderPipelineState:_solid_pipeline];
            [enc setDepthStencilState:_depth_none];
            [enc setVertexBytes:verts
                         length:sizeof(simd_float3) * (NSUInteger)n
                        atIndex:0];
            [enc setVertexBytes:&su length:sizeof(su) atIndex:1];
            [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip
                    vertexStart:0
                    vertexCount:(NSUInteger)n];
        };
        simd_float3 accent = mac_shell::srgb3(mac_shell::theme::ACCENT);
        ring(0.0f, 1.0f, simd_make_float4(1, 1, 1, 0.10f));  // track
        ring(0.0f, std::fmin(arc.progress, 1.0f),
             simd_make_float4(accent, 0.95f));
    }
}

@end
