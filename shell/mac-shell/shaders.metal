// shaders.metal — mac-shell render pipelines: passthrough background quad,
// SDF rounded panels (+ analytic soft shadows, focus glow), glass plates,
// instanced keyboard keycaps with press depression, hand spheres/capsule
// bones as impostors, and solid-color primitives.
//
// Palette constants mirror spatial-shell/theme vantage.toml via
// src/theme_tokens.h — keep in lockstep.

#include <metal_stdlib>
using namespace metal;

struct frame_uniforms {
    float4x4 view_proj;
};

// Vantage palette (linearised sRGB approximations are fine at these levels).
constant float3 VANTAGE_ACCENT = float3(0.929, 0.482, 0.047);   // #ed7b0c
constant float3 VANTAGE_ACCENT_DEEP = float3(0.773, 0.251, 0.039); // #c5400a
constant float3 VANTAGE_BORDER = float3(0.957, 0.937, 0.878);   // #f4efe0
constant float3 VANTAGE_SURFACE = float3(0.106, 0.071, 0.055);  // #1b120e
// Matches the MTKView clearColor in app_main.mm — a faded passthrough must
// land exactly on the background it is dissolving into.
constant float3 VANTAGE_BG = float3(0.078, 0.051, 0.039);       // #140d0a

// Rounded-rectangle SDF (centred, half-extent b, corner radius r).
static float rrect_sdf(float2 p, float2 b, float r) {
    float2 q = abs(p) - b + r;
    return length(max(q, 0.0)) + min(max(q.x, q.y), 0.0) - r;
}

static float hash21(float2 p) {
    float3 p3 = fract(float3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

// ---------------------------------------------------------------------------
// Passthrough: head-locked fullscreen quad, drawn first. Sized to the
// camera's true angular extent from the streamed intrinsics, and writing
// per-pixel depth from the LiDAR depth map so real-world geometry occludes
// panels. Formulas mirror src/depth_math.h — keep both in sync.
// ---------------------------------------------------------------------------

struct fs_out {
    float4 position [[position]];
    float2 uv;
};

struct passthrough_uniforms {
    float4 uv_m;           // row-major 2x2: uv_cam-0.5 = M*(uv_scr-0.5)+off
                           // (angular scale + phone-orientation rotation)
    float2 uv_offset;      // principal-point shift, image fractions
    float2 depth_near_far; // renderer projection near/far (metres)
    float have_depth;      // 0 → no LiDAR: everything at the far sentinel
    float depth_push_m;    // soft occlusion: depth written band/2 behind the
                           // LiDAR reading (0 = hard cutoff)
    float fade;            // 1 = live frame, 0 = stalled → clear colour
    float pad;
};

constant float DEPTH_INVALID_ZBUF = 0.9999;

vertex fs_out passthrough_vertex(uint vid [[vertex_id]]) {
    // Fullscreen triangle.
    float2 pos[3] = {float2(-1.0, -1.0), float2(3.0, -1.0), float2(-1.0, 3.0)};
    fs_out out;
    out.position = float4(pos[vid], 0.999, 1.0);
    out.uv = float2((pos[vid].x + 1.0) * 0.5, 1.0 - (pos[vid].y + 1.0) * 0.5);
    return out;
}

struct pt_out {
    float4 color [[color(0)]];
    float depth [[depth(any)]];
};

fragment pt_out passthrough_fragment(fs_out in [[stage_in]],
                                     texture2d<float> tex [[texture(0)]],
                                     texture2d<float> depth_tex [[texture(1)]],
                                     constant passthrough_uniforms &u
                                         [[buffer(0)]]) {
    constexpr sampler s(mag_filter::linear, min_filter::linear,
                        address::clamp_to_edge);
    constexpr sampler sd(mag_filter::nearest, min_filter::nearest,
                         address::clamp_to_edge);
    // Rotation-aware mapping (depth_math.h passthrough_uv_mapping): the same
    // matrix carries the intrinsics scale AND the snapped phone-orientation
    // roll, and the depth map is colour-aligned so it uses the SAME uv.
    float2 p = in.uv - 0.5;
    float2 uv = 0.5 + float2(dot(u.uv_m.xy, p), dot(u.uv_m.zw, p)) +
                u.uv_offset;
    pt_out out;
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) {
        out.color = float4(mix(VANTAGE_BG, float3(0.0), u.fade), 1.0);
        out.depth = DEPTH_INVALID_ZBUF;
        return out;
    }
    out.color = float4(mix(VANTAGE_BG, tex.sample(s, uv).rgb, u.fade), 1.0);
    out.depth = DEPTH_INVALID_ZBUF;
    if (u.have_depth > 0.5) {
        float d = depth_tex.sample(sd, uv).r;
        if (d > 0.0) {  // 0 == no reading: must never occlude
            float near_z = u.depth_near_far.x, far_z = u.depth_near_far.y;
            d = max(d + u.depth_push_m, near_z);
            float zs = far_z / (near_z - far_z);
            out.depth = min((zs * (near_z - d)) / d, DEPTH_INVALID_ZBUF);
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// Panels: textured quads with rounded-corner SDF, hairline border, animated
// focus glow (Vantage orange), unfocused dimming, and an analytic soft
// shadow pass (no blur passes). Vertex positions are metres in panel-local
// space; `local` carries them to the fragment for the SDF.
// ---------------------------------------------------------------------------

struct panel_vertex {
    float3 position;
    float2 uv;
};

struct panel_v_out {
    float4 position [[position]];
    float2 uv;
    float2 local;
};

struct panel_uniforms {
    float4x4 mvp;
    float2 half_size;     // quad half-extent, metres
    float corner_radius;  // metres
    float focus_glow;     // 0..1, spring-animated
    float dim;            // 0 focused → 1 fully dimmed
    float alpha;          // overall opacity (open/close fades)
    float grain;          // film-grain amount (glass plate)
    float time;           // seconds, for the glow breathing
    float shadow;         // shadow pass strength
    float pad0, pad1, pad2;
};

vertex panel_v_out panel_vertex_main(
    const device panel_vertex *verts [[buffer(0)]],
    constant panel_uniforms &u [[buffer(1)]], uint vid [[vertex_id]]) {
    panel_v_out out;
    out.position = u.mvp * float4(verts[vid].position, 1.0);
    out.uv = verts[vid].uv;
    out.local = verts[vid].position.xy;
    return out;
}

// Soft LiDAR occlusion parameters for the panel fragment (mirrors
// depth_math.h occlusion_visibility / zbuffer_to_depth — keep in sync).
// band <= 0 disables the pass (hard z-buffer only).
struct occlusion_uniforms {
    float4 uv_m;         // screen→camera uv mapping (same as passthrough)
    float2 uv_offset;
    float2 viewport;     // drawable size, pixels
    float2 depth_consts; // x = zs, y = near_z
    float band_m;        // soft band width (metres); <= 0 → off
    float pad;
};

// Fragment visibility against the (colour-aligned) LiDAR depth map:
// attenuates over a band centred on the reading instead of a hard cutoff.
static float occlusion_visibility(float4 frag_pos, texture2d<float> depth_tex,
                                  constant occlusion_uniforms &o) {
    if (o.band_m <= 0.0)
        return 1.0;
    constexpr sampler sd(mag_filter::nearest, min_filter::nearest,
                         address::clamp_to_edge);
    float2 suv = frag_pos.xy / o.viewport;  // fragment coords: y down, like uv
    float2 p = suv - 0.5;
    float2 cuv = 0.5 + float2(dot(o.uv_m.xy, p), dot(o.uv_m.zw, p)) +
                 o.uv_offset;
    if (cuv.x < 0.0 || cuv.x > 1.0 || cuv.y < 0.0 || cuv.y > 1.0)
        return 1.0;
    float dl = depth_tex.sample(sd, cuv).r;
    if (dl <= 0.0)
        return 1.0;  // no reading never occludes
    // Recover the fragment's metric depth from its z-buffer value.
    float zs = o.depth_consts.x, near_z = o.depth_consts.y;
    float df = (zs * near_z) / (frag_pos.z + zs);
    float t = (df - (dl - o.band_m * 0.5)) / o.band_m;
    return 1.0 - clamp(t, 0.0, 1.0);
}

fragment float4 panel_fragment_main(panel_v_out in [[stage_in]],
                                    texture2d<float> tex [[texture(0)]],
                                    texture2d<float> depth_tex [[texture(1)]],
                                    constant panel_uniforms &u [[buffer(0)]],
                                    constant occlusion_uniforms &o
                                        [[buffer(1)]]) {
    constexpr sampler s(mag_filter::linear, min_filter::linear,
                        mip_filter::linear, address::clamp_to_edge);
    float4 c = tex.sample(s, in.uv);

    float d = rrect_sdf(in.local, u.half_size, u.corner_radius);
    float aa = max(fwidth(d), 1e-4);
    float coverage = 1.0 - smoothstep(-aa, aa, d);

    // Unfocused panels recede: dim + slight desaturation.
    float lum = dot(c.rgb, float3(0.299, 0.587, 0.114));
    c.rgb = mix(c.rgb, mix(c.rgb, float3(lum), 0.35) * 0.6, u.dim);

    // Hairline border just inside the edge (theme border @ 0.18 alpha).
    float border_w = 0.0022;
    float border = 1.0 - smoothstep(0.0, border_w + aa, abs(d + border_w));
    c.rgb = mix(c.rgb, VANTAGE_BORDER,
                border * (0.18 + 0.25 * u.focus_glow));

    // Focus: an orange glow that hugs the inner edge and breathes gently.
    float breathe = 0.85 + 0.15 * sin(u.time * 2.2);
    float glow = exp(-abs(d) / 0.012) * u.focus_glow * breathe;
    c.rgb += VANTAGE_ACCENT * glow * 0.55;

    float vis = occlusion_visibility(in.position, depth_tex, o);
    return float4(c.rgb, c.a * coverage * u.alpha * vis);
}

// Screen-space blit (dock / launcher HUD textures): plain sample × alpha.
fragment float4 blit_fragment(panel_v_out in [[stage_in]],
                              texture2d<float> tex [[texture(0)]],
                              constant panel_uniforms &u [[buffer(0)]]) {
    constexpr sampler s(mag_filter::linear, min_filter::linear,
                        mip_filter::linear, address::clamp_to_edge);
    float4 c = tex.sample(s, in.uv);
    return float4(c.rgb, c.a * u.alpha);
}

// Analytic soft shadow: drawn on a slightly larger quad behind the panel.
// Penumbra from the SDF distance — no blur passes needed.
fragment float4 shadow_fragment(panel_v_out in [[stage_in]],
                                constant panel_uniforms &u [[buffer(0)]]) {
    float d = rrect_sdf(in.local, u.half_size, u.corner_radius * 2.0);
    float penumbra = 0.045;
    float a = 1.0 - smoothstep(-penumbra * 0.4, penumbra, d);
    return float4(0.0, 0.0, 0.0, a * u.shadow * u.alpha);
}

// Glass plate (keyboard base): translucent Vantage surface, accent-warmed
// border glow, film grain. No texture.
fragment float4 glass_fragment(panel_v_out in [[stage_in]],
                               constant panel_uniforms &u [[buffer(0)]]) {
    float d = rrect_sdf(in.local, u.half_size, u.corner_radius);
    float aa = max(fwidth(d), 1e-4);
    float coverage = 1.0 - smoothstep(-aa, aa, d);

    float3 c = VANTAGE_SURFACE * 0.9;
    // Depth cue: slightly deeper toward the far (top) edge.
    c *= 1.0 - 0.25 * (0.5 + in.local.y / max(u.half_size.y * 2.0, 1e-4));

    float border = exp(-abs(d) / 0.0045);
    c += mix(VANTAGE_BORDER * 0.22, VANTAGE_ACCENT * 0.55,
             0.35 + 0.25 * sin(u.time * 1.7)) *
         border * (0.5 + 0.5 * u.focus_glow);

    float n = hash21(floor(in.local * 2200.0));
    c += n * u.grain * (1.0 - c);  // screen blend

    return float4(c, 0.82 * coverage * u.alpha);
}

// ---------------------------------------------------------------------------
// Keyboard keycaps: one instance per key, expanded in the keyboard plane
// basis and lifted along the plane normal. A press depresses the cap toward
// the plane and blooms it toward the accent; holds pulse.
// ---------------------------------------------------------------------------

struct key_instance {
    float2 center;     // plane-local metres
    float2 half_size;  // plane-local metres (cap extent, gap excluded)
    float press;       // 0..1 highlight (1 = just struck)
    float hold;        // 1 while held (auto-repeat)
    float2 pad;
};

struct kbd_uniforms {
    float4x4 view_proj;
    float4 origin;      // xyz = plane top-left corner (world)
    float4 right;       // xyz = plane +x
    float4 down;        // xyz = plane +y (top row → space row)
    float4 normal;      // xyz = above-plane
    float2 plane_size;  // metres (uv mapping of the label texture)
    float lift;         // resting cap lift above the plane, metres
    float time;
};

struct key_v_out {
    float4 position [[position]];
    float2 uv;      // into the label texture
    float2 local;   // metres from key centre
    float2 half_size;
    float press;
    float hold;
};

vertex key_v_out key_vertex_main(const device key_instance *keys [[buffer(0)]],
                                 constant kbd_uniforms &u [[buffer(1)]],
                                 uint vid [[vertex_id]],
                                 uint iid [[instance_id]]) {
    key_instance k = keys[iid];
    // Two-triangle quad from vertex id, with a small AA margin.
    float2 corner = float2((vid == 1 || vid == 2 || vid == 4) ? 1.0 : -1.0,
                           (vid >= 2 && vid != 3) ? 1.0 : -1.0);
    float2 margin = k.half_size + 0.0012;
    float2 p = k.center + corner * margin;
    float lift = u.lift * (1.0 - 0.85 * k.press);
    float3 world = u.origin.xyz + u.right.xyz * p.x + u.down.xyz * p.y +
                   u.normal.xyz * lift;
    key_v_out out;
    out.position = u.view_proj * float4(world, 1.0);
    out.uv = p / u.plane_size;
    out.local = corner * margin;
    out.half_size = k.half_size;
    out.press = k.press;
    out.hold = k.hold;
    return out;
}

fragment float4 key_fragment_main(key_v_out in [[stage_in]],
                                  texture2d<float> tex [[texture(0)]],
                                  constant kbd_uniforms &u [[buffer(0)]]) {
    constexpr sampler s(mag_filter::linear, min_filter::linear,
                        mip_filter::linear, address::clamp_to_edge);
    // The CPU surface draws the styled cap + label; clip to the cap SDF so
    // the quad margin stays clean at glancing angles.
    float d = rrect_sdf(in.local, in.half_size, 0.0035);
    float aa = max(fwidth(d), 1e-4);
    float coverage = 1.0 - smoothstep(-aa, aa, d);

    float4 c = tex.sample(s, in.uv);

    // Press bloom: cap surges toward the accent then decays with `press`.
    float bloom = in.press * (0.55 + 0.45 * exp(-abs(d) / 0.004));
    c.rgb = mix(c.rgb, VANTAGE_ACCENT, bloom * 0.75);
    c.rgb += VANTAGE_ACCENT * bloom * 0.35;

    // Hold: pulse the deep accent so auto-repeat reads at a glance.
    float pulse = in.hold * (0.5 + 0.5 * sin(u.time * 9.0));
    c.rgb = mix(c.rgb, VANTAGE_ACCENT_DEEP, pulse * 0.6);

    return float4(c.rgb, c.a * coverage);
}

// ---------------------------------------------------------------------------
// Hands: soft-sphere impostors for joints, capsule impostors for bones.
// Billboarded quads; the fragment shades an analytic sphere/capsule and
// writes its true depth so hands sort correctly against panels.
// ---------------------------------------------------------------------------

struct hand_uniforms {
    float4x4 view_proj;
    float4 cam_right;   // world-space camera basis
    float4 cam_up;
    float4 view_row2;   // third row of the view matrix (gives view z)
    float2 depth_consts; // x = zs, y = near*zs (perspective_rh terms)
    float time;
    float pad;
};

struct sphere_instance {
    float4 pos_radius;  // xyz world, w radius
    float4 color;       // rgb, a = alpha
    float4 glow;        // x = contact glow 0..1
};

struct sphere_v_out {
    float4 position [[position]];
    float2 quad;  // -1..1
    float3 color;
    float alpha;
    float glow;
    float radius;
    float vz_center;
};

struct impostor_out {
    float4 color [[color(0)]];
    float depth [[depth(any)]];
};

static float impostor_depth(float vz, float2 dc) {
    // perspective_rh: clip.z = zs*vz + near*zs, clip.w = -vz.
    return (dc.x * vz + dc.y) / (-vz);
}

vertex sphere_v_out sphere_vertex_main(
    const device sphere_instance *inst [[buffer(0)]],
    constant hand_uniforms &u [[buffer(1)]], uint vid [[vertex_id]],
    uint iid [[instance_id]]) {
    sphere_instance sp = inst[iid];
    float2 corner = float2((vid == 1 || vid == 2 || vid == 4) ? 1.0 : -1.0,
                           (vid >= 2 && vid != 3) ? 1.0 : -1.0);
    float r = sp.pos_radius.w;
    float3 world = sp.pos_radius.xyz + u.cam_right.xyz * corner.x * r +
                   u.cam_up.xyz * corner.y * r;
    sphere_v_out out;
    out.position = u.view_proj * float4(world, 1.0);
    out.quad = corner;
    out.color = sp.color.rgb;
    out.alpha = sp.color.a;
    out.glow = sp.glow.x;
    out.radius = r;
    out.vz_center = dot(u.view_row2, float4(sp.pos_radius.xyz, 1.0));
    return out;
}

fragment impostor_out sphere_fragment_main(sphere_v_out in [[stage_in]],
                                           constant hand_uniforms &u
                                               [[buffer(0)]]) {
    float r2 = dot(in.quad, in.quad);
    if (r2 > 1.0)
        discard_fragment();
    float nz = sqrt(1.0 - r2);
    // Soft top-left key light + rim.
    float3 n = float3(in.quad.x, in.quad.y, nz);
    float3 l = normalize(float3(-0.35, 0.55, 0.75));
    float diff = 0.35 + 0.65 * max(dot(n, l), 0.0);
    float rim = pow(1.0 - nz, 2.0) * 0.35;
    float3 c = in.color * diff + rim;
    // Fingertip near the key plane: warm up toward the accent.
    c = mix(c, VANTAGE_ACCENT, in.glow * 0.8);
    c += VANTAGE_ACCENT * in.glow * 0.4;
    float edge = 1.0 - smoothstep(0.82, 1.0, sqrt(r2));

    impostor_out out;
    out.color = float4(c, in.alpha * max(edge, 0.15));
    out.depth = impostor_depth(in.vz_center + nz * in.radius,
                               u.depth_consts);
    return out;
}

// Reticles read depth without covering the view with opaque spheres.
fragment float4 sphere_glow_fragment_main(sphere_v_out in [[stage_in]],
                                          constant hand_uniforms &u
                                              [[buffer(0)]]) {
    float r = length(in.quad);
    float aa = max(fwidth(r), 0.015);
    float ring = 1.0 - smoothstep(0.055, 0.055 + aa, abs(r - 0.70));
    float center = 1.0 - smoothstep(0.13, 0.13 + aa, r);
    float halo = (1.0 - smoothstep(0.72, 1.0, r)) * in.glow * 0.12;
    float a = in.alpha * max(max(ring * 0.85, center), halo);
    if (a < 0.001)
        discard_fragment();
    float3 c = mix(in.color, VANTAGE_ACCENT, in.glow * 0.55);
    return float4(c, a);
}

struct capsule_instance {
    float4 a_radius;  // xyz end A, w radius
    float4 b;         // xyz end B
    float4 color;     // rgb, a = alpha
};

struct capsule_v_out {
    float4 position [[position]];
    float3 world;
    float3 a;
    float3 b;
    float radius;
    float3 color;
    float alpha;
};

vertex capsule_v_out capsule_vertex_main(
    const device capsule_instance *inst [[buffer(0)]],
    constant hand_uniforms &u [[buffer(1)]], uint vid [[vertex_id]],
    uint iid [[instance_id]]) {
    capsule_instance cp = inst[iid];
    float r = cp.a_radius.w;
    float3 a = cp.a_radius.xyz, b = cp.b.xyz;
    float3 axis = b - a;
    float len = max(length(axis), 1e-5);
    axis /= len;
    // Camera-facing side vector.
    float3 view_dir = normalize(cross(u.cam_right.xyz, u.cam_up.xyz));
    float3 side = normalize(cross(axis, view_dir));
    float2 corner = float2((vid == 1 || vid == 2 || vid == 4) ? 1.0 : -1.0,
                           (vid >= 2 && vid != 3) ? 1.0 : -1.0);
    float3 along = axis * (len * 0.5 + r);
    float3 centre = (a + b) * 0.5;
    float3 world = centre + along * corner.y + side * (r * 1.2) * corner.x;
    capsule_v_out out;
    out.position = u.view_proj * float4(world, 1.0);
    out.world = world;
    out.a = a;
    out.b = b;
    out.radius = r;
    out.color = cp.color.rgb;
    out.alpha = cp.color.a;
    return out;
}

fragment impostor_out capsule_fragment_main(capsule_v_out in [[stage_in]],
                                            constant hand_uniforms &u
                                                [[buffer(0)]]) {
    // Distance from the fragment's world position to the bone segment.
    float3 pa = in.world - in.a, ba = in.b - in.a;
    float h = clamp(dot(pa, ba) / max(dot(ba, ba), 1e-8), 0.0, 1.0);
    float d = length(pa - ba * h);
    float t = d / in.radius;
    if (t > 1.0)
        discard_fragment();
    float nz = sqrt(max(1.0 - t * t, 0.0));
    float diff = 0.4 + 0.6 * nz;
    float3 c = in.color * diff;
    float edge = 1.0 - smoothstep(0.8, 1.0, t);

    impostor_out out;
    out.color = float4(c, in.alpha * max(edge, 0.1));
    float vz = dot(u.view_row2, float4(in.world, 1.0));
    out.depth = impostor_depth(vz + nz * in.radius, u.depth_consts);
    return out;
}

// ---------------------------------------------------------------------------
// Solid-color primitives (lines, arcs, translucent strips).
// ---------------------------------------------------------------------------

struct solid_uniforms {
    float4x4 mvp;
    float4 color;
};

struct solid_v_out {
    float4 position [[position]];
    float4 color;
};

vertex solid_v_out solid_vertex_main(const device float3 *verts [[buffer(0)]],
                                     constant solid_uniforms &u [[buffer(1)]],
                                     uint vid [[vertex_id]]) {
    solid_v_out out;
    out.position = u.mvp * float4(verts[vid], 1.0);
    out.color = u.color;
    return out;
}

fragment float4 solid_fragment_main(solid_v_out in [[stage_in]]) {
    return in.color;
}
