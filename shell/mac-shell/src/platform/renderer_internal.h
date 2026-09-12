// renderer_internal.h — private seam shared by the renderer's translation
// units (renderer.mm + the renderer_*.mm categories + app_main.mm). Not
// part of the public API; include platform/renderer.h for that.

#pragma once

#import <AppKit/AppKit.h>
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>
#import <CoreVideo/CoreVideo.h>
#import <ImageIO/ImageIO.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#include <mach-o/dyld.h>

#include <arpa/inet.h>
#include <dns_sd.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <functional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <simd/simd.h>

#include "platform/capture.h"
#include "platform/frame_grab.h"
#include "platform/png_write.h"
#include "platform/port_retry.h"
#include "core/depth_math.h"
#include "core/hud_state.h"
#include "platform/renderer.h"
#include "core/scene.h"
#include "shaders_metal.h"
#include "ui/theme_tokens.h"

namespace mac_shell {

// ------------------------------------------------------------------
// matrix helpers (column-major simd, Metal clip space)
// ------------------------------------------------------------------

inline simd_float4x4 perspective_rh(float fovy, float aspect, float near_z,
                             float far_z) {
    float ys = 1.0f / std::tan(fovy * 0.5f);
    float xs = ys / aspect;
    float zs = far_z / (near_z - far_z);
    return simd_float4x4{
        simd_make_float4(xs, 0, 0, 0), simd_make_float4(0, ys, 0, 0),
        simd_make_float4(0, 0, zs, -1),
        simd_make_float4(0, 0, near_z * zs, 0)};
}

inline simd_float4x4 translation(simd_float3 t) {
    simd_float4x4 m = matrix_identity_float4x4;
    m.columns[3] = simd_make_float4(t.x, t.y, t.z, 1);
    return m;
}

inline simd_float4x4 rotation_from_quat(simd_float4 q) {  // xyzw
    simd_quatf sq = simd_quaternion(q.x, q.y, q.z, q.w);
    return simd_matrix4x4(sq);
}

// mac-shell panel matrices are row-major with row-vector convention
// (rows = right/up/front/translation); the equivalent column-major matrix
// has those rows as columns.
inline simd_float4x4 model_from_rows(const float m[16]) {
    return simd_float4x4{
        simd_make_float4(m[0], m[1], m[2], m[3]),
        simd_make_float4(m[4], m[5], m[6], m[7]),
        simd_make_float4(m[8], m[9], m[10], m[11]),
        simd_make_float4(m[12], m[13], m[14], m[15])};
}

// ------------------------------------------------------------------
// spring animation (port of vendor/wxrd/src/animation.c wxrd_spring —
// critically-damped harmonic oscillator, frame-rate independent,
// interruptible by construction: set_target mid-flight just re-aims it)
// ------------------------------------------------------------------

struct spring1 {
    float position = 0, velocity = 0, target = 0;
    float stiffness = theme::SPRING_MOVE_STIFFNESS;
    float damping = theme::SPRING_MOVE_DAMPING;

    void init(float value, float k, float c) {
        position = target = value;
        velocity = 0;
        stiffness = k;
        damping = c;
    }
    void step(float dt) {
        float force = -stiffness * (position - target) - damping * velocity;
        velocity += force * dt;
        position += velocity * dt;
    }
    bool settled() const {
        return std::fabs(position - target) < 0.001f &&
               std::fabs(velocity) < 0.01f;
    }
};

// Per-panel animation state (open / close / move / focus / grab).
struct panel_anim {
    spring1 tx, ty, tz;  // translation toward the scene target
    spring1 scale;       // open 0.85 → 1, close → 0.9
    spring1 alpha;       // open 0 → 1, close → 0
    spring1 glow;        // focus ring 0..1
    spring1 lift;        // grab lift/tilt 0..1
    bool closing = false;
    scene::render_panel cached;      // last snapshot, drives the close fade
    id<MTLTexture> cached_tex = nil;
};

inline float clamp01(float v) { return std::fmax(0.0f, std::fmin(1.0f, v)); }

// Keyboard / launcher / HUD-card show-hide spring. tick() re-aims the spring
// on a visibility change, steps it, and returns the clamped show factor.
struct overlay_anim {
    spring1 show;  // 0 hidden → 1 shown
    bool visible = false;

    float tick(bool want, float dt) {
        if (want != visible) {
            visible = want;
            show.target = want ? 1.0f : 0.0f;
        }
        show.step(dt);
        return clamp01(show.position);
    }
};

// ------------------------------------------------------------------
// GPU-side uniform layouts (must match shaders.metal)
// ------------------------------------------------------------------

struct panel_uniforms {
    simd_float4x4 mvp;
    simd_float2 half_size;
    float corner_radius;
    float focus_glow;
    float dim;
    float alpha;
    float grain;
    float time;
    float shadow;
    float pad0, pad1, pad2;
};

struct passthrough_uniforms {
    simd_float4 uv_m;  // row-major 2x2 screen→camera uv (scale + orientation)
    simd_float2 uv_offset;
    simd_float2 depth_near_far;
    float have_depth;
    float depth_push;  // soft occlusion: written depth pushed band/2 back
    float fade;        // 1 = live frame, 0 = stalled → clear colour
    float pad;
};

// Soft LiDAR occlusion parameters for the panel fragment (shaders.metal
// occlusion_uniforms — keep in sync). band_m <= 0 disables the pass.
struct occlusion_uniforms {
    simd_float4 uv_m;
    simd_float2 uv_offset;
    simd_float2 viewport;
    simd_float2 depth_consts;  // zs, near_z
    float band_m;
    float pad;
};

struct key_instance {
    simd_float2 center;
    simd_float2 half_size;
    float press;
    float hold;
    simd_float2 pad;
};

struct kbd_uniforms {
    simd_float4x4 view_proj;
    simd_float4 origin;
    simd_float4 right;
    simd_float4 down;
    simd_float4 normal;
    simd_float2 plane_size;
    float lift;
    float time;
};

struct hand_uniforms {
    simd_float4x4 view_proj;
    simd_float4 cam_right;
    simd_float4 cam_up;
    simd_float4 view_row2;
    simd_float2 depth_consts;
    float time;
    float pad;
};

struct sphere_instance {
    simd_float4 pos_radius;
    simd_float4 color;
    simd_float4 glow;
};

struct capsule_instance {
    simd_float4 a_radius;
    simd_float4 b;
    simd_float4 color;
};

struct solid_uniforms {
    simd_float4x4 mvp;
    simd_float4 color;
};

struct panel_vertex {
    simd_float3 position;
    simd_float2 uv;
};

constexpr float RENDER_NEAR_Z = 0.05f;
constexpr float RENDER_FAR_Z = 100.0f;
constexpr float RENDER_FOVY_RAD = 60.0f * (float)M_PI / 180.0f;

constexpr int DOCK_TEX_W = 1024;
constexpr int DOCK_TEX_H = 84;
// Packet rate is quantised to this before it reaches the dock's cache
// signature, so only a real rate change repaints the strip.
constexpr int DOCK_RATE_BUCKET = 10;

// Keyboard plate metrics (metres).
constexpr float KBD_PLATE_MARGIN_M = 0.014f;
constexpr float KBD_PLATE_LIFT_M = 0.008f;
constexpr float KBD_KEY_LIFT_M = 0.0045f;

// Grab feedback: lift toward the viewer + slight tilt.
constexpr float GRAB_LIFT_M = 0.03f;
constexpr float GRAB_TILT_RAD = -0.09f;

// SPATULA_MAC_PERF=1: time the scene-snapshot + texture-upload path and
// print mean/max to stderr at most once every PERF_REPORT_S. Disabled it
// costs one predictable branch per region.
constexpr double PERF_REPORT_S = 5.0;

struct perf_accum {
    bool on = false;
    void enable() { on = getenv("SPATULA_MAC_PERF") != nullptr; }
    void enter() {
        if (on)
            t0_ = std::chrono::steady_clock::now();
    }
    void leave() {
        if (!on)
            return;
        frame_s_ += std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - t0_)
                        .count();
    }
    // Fold the accumulated regions into the running window and report when
    // PERF_REPORT_S has elapsed. `now` is the renderer's frame clock.
    void end_frame(double now) {
        if (!on)
            return;
        sum_s_ += frame_s_;
        max_s_ = std::fmax(max_s_, frame_s_);
        frames_++;
        frame_s_ = 0.0;
        if (last_report_s_ == 0.0)
            last_report_s_ = now;
        if (now - last_report_s_ < PERF_REPORT_S || frames_ == 0)
            return;
        std::fprintf(stderr,
                     "mac-shell perf: snapshot+upload mean %.3f ms max %.3f "
                     "ms over %d frames\n",
                     sum_s_ / frames_ * 1e3, max_s_ * 1e3, frames_);
        sum_s_ = max_s_ = 0.0;
        frames_ = 0;
        last_report_s_ = now;
    }

   private:
    std::chrono::steady_clock::time_point t0_;
    double frame_s_ = 0, sum_s_ = 0, max_s_ = 0, last_report_s_ = 0;
    int frames_ = 0;
};

// Dock hit-region (fractions of the window) for click-to-focus.
struct dock_entry_hit {
    float x0, x1;  // fraction of window width
    uint64_t handle;
};

// ------------------------------------------------------------------
// HUD overlays: welcome card / onboarding / cheat sheet / toasts
// ------------------------------------------------------------------

// CPU-surface sizes (device px — on Retina these render at half in points,
// matching the dock's convention).
constexpr int WELCOME_TEX_W = 1240, WELCOME_TEX_H = 880;
constexpr int ONBOARD_TEX_W = 1040, ONBOARD_TEX_H = 800;
constexpr int CHEAT_TEX_W = 960, CHEAT_TEX_H = 660;
constexpr int TOAST_TEX_W = 1160, TOAST_TEX_H = 150;

// "Waiting for your iPhone" pulse — one breath per this period.
constexpr float WELCOME_PULSE_PERIOD_S = 2.4f;
// Redraw granularity of the pulsing card (buckets per pulse period).
constexpr int WELCOME_PULSE_STEPS = 24;
// Bonjour browse warm-up before the chip claims "not advertised".
constexpr float BONJOUR_GRACE_S = 3.0f;
// WiFi IP re-probe cadence (network changes while the card is up).
constexpr float IP_REFRESH_S = 5.0f;

// Screen-fraction rect (x0,y0,x1,y1; y up from the bottom) for overlay
// click targets.
struct hit_rect {
    float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    bool valid = false;
    bool contains(float fx, float fy) const {
        return valid && fx >= x0 && fx <= x1 && fy >= y0 && fy <= y1;
    }
};

// First WiFi-ish IPv4 (mirrors run-mac.sh: en0, en1, en2, then any
// non-loopback interface).
std::string wifi_ipv4();

// The user-facing "Computer Name" (System Settings > General > About),
// falling back to the short hostname. NOT gethostname(): on DHCP networks
// that yields a generated name like "wifi-10-45-211-166", which is what the
// iPhone would then show in its Mac list.
std::string computer_name();

// Greedy word wrap for CPU-surface text.
std::vector<std::string> wrap_text(const std::string &text, size_t max_chars);

// Hand skeleton bone pairs (Vision framework joint order).
constexpr int BONES[][2] = {
    {0, 1},   {1, 2},   {2, 3},   {3, 4},    // thumb
    {0, 5},   {5, 6},   {6, 7},   {7, 8},    // index
    {0, 9},   {9, 10},  {10, 11}, {11, 12},  // middle
    {0, 13},  {13, 14}, {14, 15}, {15, 16},  // ring
    {0, 17},  {17, 18}, {18, 19}, {19, 20},  // pinky
};
constexpr int N_BONES = sizeof(BONES) / sizeof(BONES[0]);
constexpr int FINGERTIPS[] = {4, 8, 12, 16, 20};

inline bool is_fingertip(int j) {
    for (int t : FINGERTIPS)
        if (t == j)
            return true;
    return false;
}

inline simd_float3 srgb3(color_rgba c) {
    return simd_make_float3(c.r / 255.0f, c.g / 255.0f, c.b / 255.0f);
}

struct panel_texture {
    id<MTLTexture> tex;
    uint64_t version;
    CVMetalTextureRef cv_tex = nullptr;  // captured panels: owns tex's backing
};

// MTKView keeps up to this many frames in flight; CPU-written textures are
// ring-buffered per frame and a semaphore gates encoding so replaceRegion
// never touches a texture the GPU is still reading.
constexpr int FRAMES_IN_FLIGHT = 3;

struct renderer_state {
    scene *world = nullptr;
    sb_receiver_t *receiver = nullptr;
    capture_manager *capture = nullptr;
    std::atomic<bool> *running = nullptr;

    // fly-cam
    simd_float3 cam_pos = {0, 0, 0};
    float cam_yaw = 0, cam_pitch = 0;
    std::set<unsigned short> keys_down;
    float drag_dx = 0, drag_dy = 0;

    double last_time = 0;

    // SPATULA_MAC_SCREENSHOT: one detached grab, screenshot_at seconds in.
    // Everything else (the `screenshot` verb) arms `grabber` directly.
    std::string screenshot_path;
    double screenshot_at = 2.0;
    bool screenshot_done = false;
    frame_grabber *grabber = nullptr;
    double exit_after = 0;
    double start_time = 0;

    // Dock hit-testing (written by the render delegate, read by mouseDown —
    // both on the main thread).
    std::vector<dock_entry_hit> dock_hits;
    float dock_h_frac = 0.0f;
    float dock_help_x0 = 0.0f, dock_help_x1 = 0.0f;  // "?" pill

    // ---- boot facts (renderer_boot_info) ----
    int udp_port = 9898;
    bool replay = false;
    std::string startup_error_title;
    std::string startup_error;
    port_retry *retry = nullptr;

    // ---- welcome / connection state ----
    welcome_model welcome;
    std::string local_ip;
    double last_ip_check = -1e9;
    // Bonjour advertisement watch: our own registration seen coming back
    // through a local browse. Written from the mDNS dispatch queue, read by
    // the render thread.
    std::atomic<int> bonjour_matches{0};
    std::string bonjour_expected;  // "Spatula (<hostname>)"
    bool bonjour_browsing = false;
    double bonjour_started_at = 0;

    // ---- overlays ----
    bool onboarding_visible = false;
    bool cheat_visible = false;
    bool autoshow_cheat_after_onboarding = false;
    bool persist_first_run = false;  // NSUserDefaults writes allowed
    hit_rect onboard_button;
    hit_rect toast_rect;
    hit_rect toast_button;
};

// DNSServiceRegister reply: logs conflicts. Required (not optional) because
// dns_sd rejects a null callback next to kDNSServiceFlagsNoAutoRename.
void bonjour_register_reply(DNSServiceRef, DNSServiceFlags,
                            DNSServiceErrorType err, const char *name,
                            const char *, const char *, void *);

// DNSServiceBrowse reply: count adds/removes of instances matching the
// expected registration name (any interface).
void bonjour_browse_reply(DNSServiceRef, DNSServiceFlags flags, uint32_t,
                          DNSServiceErrorType err, const char *name,
                          const char *, const char *, void *ctx);

}  // namespace mac_shell

using mac_shell::renderer_state;

// Stop the AppKit run loop and wake it so the stop takes effect promptly
// (app_main.mm).
void stop_app_loop();

// ------------------------------------------------------------------
// Renderer delegate
// ------------------------------------------------------------------

@interface ShellRenderer : NSObject <MTKViewDelegate>
- (instancetype)initWithView:(MTKView *)view state:(renderer_state *)state;
@end

// Ivars live in this class extension so the drawing categories below
// (separate translation units) can reach them.
@interface ShellRenderer () {
@public
    renderer_state *_s;
    id<MTLDevice> _device;
    id<MTLCommandQueue> _queue;
    id<MTLRenderPipelineState> _passthrough_pipeline;
    id<MTLRenderPipelineState> _panel_pipeline;    // SDF rounded + glow
    id<MTLRenderPipelineState> _blit_pipeline;     // HUD textures
    id<MTLRenderPipelineState> _shadow_pipeline;   // analytic soft shadow
    id<MTLRenderPipelineState> _glass_pipeline;    // keyboard base plate
    id<MTLRenderPipelineState> _key_pipeline;      // instanced keycaps
    id<MTLRenderPipelineState> _sphere_pipeline;   // hand joints (debug)
    id<MTLRenderPipelineState> _glow_pipeline;     // fingertip reticles
    id<MTLRenderPipelineState> _capsule_pipeline;  // hand bones (debug)
    id<MTLRenderPipelineState> _solid_pipeline;
    id<MTLDepthStencilState> _depth_test;
    id<MTLDepthStencilState> _depth_read;          // test, no write (shadows)
    id<MTLDepthStencilState> _depth_none;
    id<MTLDepthStencilState> _depth_write_always;  // passthrough depth fill
    id<MTLTexture> _passthrough_tex;  // slot of _passthrough_ring drawn this frame
    id<MTLTexture> _passthrough_ring[mac_shell::FRAMES_IN_FLIGHT];
    bool _have_frame;
    // Capture timestamp of the passthrough frame currently on the GPU. The
    // view matrix is built from the head pose that was current *then*, not
    // from the newest pose, so world-locked panels stay glued to the image.
    uint64_t _passthrough_ts;
    // One toast per stall, not one per frame; cleared when frames resume.
    bool _frame_stall_notified;
    id<MTLTexture> _depth_map_tex;   // R32Float LiDAR metres (ring slot in use)
    id<MTLTexture> _depth_ring[mac_shell::FRAMES_IN_FLIGHT];
    dispatch_semaphore_t _frame_sem;
    int _ring_index;
    // CF objects (captured-panel pixel buffers, superseded CVMetalTextures)
    // whose IOSurfaces this frame's command buffer reads; released in its
    // completion handler.
    std::vector<CFTypeRef> _frame_cf_release;
    id<MTLTexture> _depth_map_dummy; // 1x1 zero → never occludes
    bool _have_depth_map;
    sb_intrinsics_t _intrinsics;
    bool _have_intrinsics;
    id<MTLTexture> _kbd_tex;
    uint64_t _kbd_tex_version;
    id<MTLTexture> _launcher_tex;
    uint64_t _launcher_tex_version;
    id<MTLTexture> _dock_tex;
    mac_shell::panel_surface _dock_surface;
    std::string _dock_sig;
    CVMetalTextureCacheRef _tex_cache;
    std::unordered_map<uint64_t, mac_shell::panel_texture> _panel_textures;
    std::unordered_map<uint64_t, mac_shell::panel_anim> _panel_anims;
    mac_shell::overlay_anim _kbd_anim;
    mac_shell::overlay_anim _launcher_anim;
    mac_shell::perf_accum _perf;
    // Phone-orientation compensation (depth_math.h): snapped 90° roll bucket
    // with hysteresis; rolls both the view matrix and the passthrough UVs.
    int _orient_bucket;
    // Per-frame soft-occlusion uniforms shared by the panel draws.
    mac_shell::occlusion_uniforms _occl;
    // Temporal EMA of the LiDAR depth map (invalid readings stay invalid).
    std::vector<float> _depth_ema;
    // scene::depth_snapshot version already uploaded; 0 = none.
    uint64_t _depth_version;
    // HUD overlays: welcome/error card, onboarding, cheat sheet, toast.
    mac_shell::overlay_anim _welcome_anim;
    mac_shell::overlay_anim _onboard_anim;
    mac_shell::overlay_anim _cheat_anim;
    mac_shell::overlay_anim _toast_anim;
    mac_shell::panel_surface _welcome_surface;
    mac_shell::panel_surface _onboard_surface;
    mac_shell::panel_surface _cheat_surface;
    mac_shell::panel_surface _toast_surface;
    id<MTLTexture> _welcome_tex;
    id<MTLTexture> _onboard_tex;
    id<MTLTexture> _cheat_tex;
    id<MTLTexture> _toast_tex;
    std::string _welcome_sig;
    std::string _toast_sig;
    // Toast content held through the fade-out after dismissal.
    std::string _toast_cached_text;
    bool _toast_cached_action;
    // Cached TCC probes (a per-frame preflight is wasteful).
    bool _perm_screen, _perm_ax;
    double _perm_checked_at;
    DNSServiceRef _bonjour_ref;  // browse (verifies our own registration)
    DNSServiceRef _bonjour_reg;  // our _spatialbridge._udp advertisement
    dispatch_queue_t _mdns_queue;
}

- (id<MTLLibrary>)loadLibrary;
- (void)startBonjour;
// RGBA8 texture with a full mip chain (crisp minified panel content).
- (id<MTLTexture>)makeMippedTextureW:(NSUInteger)w h:(NSUInteger)h;
- (void)uploadTexture:(id<MTLTexture>)tex
                 rgba:(const uint8_t *)rgba
                    w:(NSUInteger)w
                    h:(NSUInteger)h;
- (simd_float4x4)viewMatrixWithDt:(float)dt;
- (void)updatePassthrough;
- (void)updateDepthMap;
- (void)releaseFrameObjects;
+ (bool)writeTexture:(id<MTLTexture>)tex
               toPNG:(const std::string &)path
                 err:(std::string &)err;
@end

// renderer_panels.mm — spring animation + shadow + SDF draw.
@interface ShellRenderer (Panels)
- (id<MTLTexture>)textureForPanel:(const mac_shell::scene::render_panel &)rp;
- (simd_float4x4)animatedModelFor:(const mac_shell::scene::render_panel &)rp
                             anim:(mac_shell::panel_anim &)a;
- (void)drawPanel:(const mac_shell::scene::render_panel &)rp
             anim:(mac_shell::panel_anim &)a
          texture:(id<MTLTexture>)tex
          encoder:(id<MTLRenderCommandEncoder>)enc
         viewProj:(simd_float4x4)view_proj
             time:(float)t;
- (void)drawPanels:(std::vector<mac_shell::scene::render_panel> &)panels
           encoder:(id<MTLRenderCommandEncoder>)enc
          viewProj:(simd_float4x4)view_proj
              view:(simd_float4x4)view_mat
                dt:(float)dt
              time:(float)t;
@end

// renderer_keyboard.mm — glass plate + instanced keycaps + plane shadow.
@interface ShellRenderer (Keyboard)
- (void)drawKeyboard:(id<MTLRenderCommandEncoder>)enc
            keyboard:(const mac_shell::keyboard_render_state &)kb
            viewProj:(simd_float4x4)view_proj
                  dt:(float)dt
                time:(float)t;
@end

// renderer_hands.mm — hand orbs / skeleton + keyboard-summon arcs.
@interface ShellRenderer (Hands)
- (void)drawHands:(id<MTLRenderCommandEncoder>)enc
         keyboard:(const mac_shell::keyboard_render_state &)kb
         viewProj:(simd_float4x4)view_proj
             view:(simd_float4x4)view_mat
               dt:(float)dt
             time:(float)t;
- (void)drawHandsMinimal:(id<MTLRenderCommandEncoder>)enc
                uniforms:(mac_shell::hand_uniforms)hu
                      dt:(float)dt
             contactGlow:(const std::function<float(const float *)> &)contact;
- (void)drawSummonArcs:(id<MTLRenderCommandEncoder>)enc
              viewProj:(simd_float4x4)view_proj
                  view:(simd_float4x4)view_mat;
@end

// renderer_dock.mm — radial launcher blit + dock/status strip.
@interface ShellRenderer (Dock)
- (void)drawLauncher:(id<MTLRenderCommandEncoder>)enc
              aspect:(float)aspect
                  dt:(float)dt;
- (void)drawDockWithEncoder:(id<MTLRenderCommandEncoder>)enc
                     panels:(const std::vector<mac_shell::scene::render_panel>
                                 &)panels
                       view:(MTKView *)view;
@end

// renderer_overlays.mm — welcome card, toast, cheat sheet, onboarding.
@interface ShellRenderer (Overlays)
- (mac_shell::hit_rect)blitOverlay:(id<MTLTexture>)tex
                           encoder:(id<MTLRenderCommandEncoder>)enc
                              view:(MTKView *)view
                           centerX:(float)cx
                           centerY:(float)cy
                              show:(float)show;
- (void)drawWelcome:(id<MTLRenderCommandEncoder>)enc
               view:(MTKView *)view
         sceneEmpty:(bool)sceneEmpty
                 dt:(float)dt
               time:(float)t;
- (void)drawToast:(id<MTLRenderCommandEncoder>)enc
             view:(MTKView *)view
               dt:(float)dt;
- (void)drawCheatSheet:(id<MTLRenderCommandEncoder>)enc
                  view:(MTKView *)view
                    dt:(float)dt;
- (void)drawOnboarding:(id<MTLRenderCommandEncoder>)enc
                  view:(MTKView *)view
                    dt:(float)dt;
@end
