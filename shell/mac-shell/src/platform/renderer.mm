// renderer.mm — see renderer.h.
//
// Visual layer: SDF rounded panels with analytic soft shadows and
// spring-animated open/close/move/focus (wxrd's critically-damped spring
// system, ported from vendor/wxrd/src/animation.c and driven by the theme
// spring constants), instanced keyboard keycaps with press depression,
// hand joints/bones as sphere/capsule impostors, and the Vantage HUD.

#include "platform/renderer_internal.h"

#include <SystemConfiguration/SystemConfiguration.h>

namespace mac_shell {

// First WiFi-ish IPv4 (mirrors run-mac.sh: en0, en1, en2, then any
// non-loopback interface).
std::string wifi_ipv4() {
    struct ifaddrs *ifs = nullptr;
    if (getifaddrs(&ifs) != 0 || !ifs)
        return "";
    auto addr_of = [&](const char *want) -> std::string {
        for (struct ifaddrs *it = ifs; it; it = it->ifa_next) {
            if (!it->ifa_addr || it->ifa_addr->sa_family != AF_INET)
                continue;
            if (!(it->ifa_flags & IFF_UP) || (it->ifa_flags & IFF_LOOPBACK))
                continue;
            if (want && std::strcmp(it->ifa_name, want) != 0)
                continue;
            char buf[INET_ADDRSTRLEN] = {0};
            auto *sin = reinterpret_cast<struct sockaddr_in *>(it->ifa_addr);
            if (inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf)))
                return buf;
        }
        return "";
    };
    std::string ip;
    for (const char *cand : {"en0", "en1", "en2"}) {
        ip = addr_of(cand);
        if (!ip.empty())
            break;
    }
    if (ip.empty())
        ip = addr_of(nullptr);
    freeifaddrs(ifs);
    return ip;
}

std::string computer_name() {
    std::string name;
    if (CFStringRef cf = SCDynamicStoreCopyComputerName(nullptr, nullptr)) {
        char buf[256] = {0};
        if (CFStringGetCString(cf, buf, sizeof(buf), kCFStringEncodingUTF8))
            name = buf;
        CFRelease(cf);
    }
    if (name.empty()) {
        char host[256] = {0};
        gethostname(host, sizeof(host) - 1);
        if (char *dot = std::strchr(host, '.'))
            *dot = '\0';
        name = host;
    }
    return name;
}

// Greedy word wrap for CPU-surface text.
std::vector<std::string> wrap_text(const std::string &text,
                                   size_t max_chars) {
    std::vector<std::string> lines;
    std::string line, word;
    auto flush_word = [&]() {
        if (word.empty())
            return;
        if (!line.empty() && line.size() + 1 + word.size() > max_chars) {
            lines.push_back(line);
            line.clear();
        }
        if (!line.empty())
            line += ' ';
        line += word;
        word.clear();
    };
    for (char c : text) {
        if (c == ' ')
            flush_word();
        else
            word += c;
    }
    flush_word();
    if (!line.empty())
        lines.push_back(line);
    return lines;
}

// DNSServiceRegister reply. Nothing to record — NoAutoRename pins the name
// to what we asked for — but dns_sd rejects a null callback alongside that
// flag, and a conflict is worth a log line.
void bonjour_register_reply(DNSServiceRef, DNSServiceFlags,
                            DNSServiceErrorType err, const char *name,
                            const char *, const char *, void *) {
    if (err != kDNSServiceErr_NoError)
        std::fprintf(stderr,
                     "renderer: Bonjour registration of '%s' failed (%d) — "
                     "another Spatula on this Mac?\n",
                     name ? name : "?", (int)err);
}

// DNSServiceBrowse reply: count adds/removes of instances matching the
// expected registration name (any interface).
void bonjour_browse_reply(DNSServiceRef, DNSServiceFlags flags, uint32_t,
                          DNSServiceErrorType err, const char *name,
                          const char *, const char *, void *ctx) {
    if (err != kDNSServiceErr_NoError || !name)
        return;
    auto *s = static_cast<renderer_state *>(ctx);
    if (!s->bonjour_expected.empty() && s->bonjour_expected != name)
        return;
    if (flags & kDNSServiceFlagsAdd)
        s->bonjour_matches.fetch_add(1);
    else if (s->bonjour_matches.load() > 0)
        s->bonjour_matches.fetch_sub(1);
}

}  // namespace mac_shell

@implementation ShellRenderer

- (instancetype)initWithView:(MTKView *)view state:(renderer_state *)state {
    self = [super init];
    if (!self)
        return nil;
    _s = state;
    _device = view.device;
    _queue = [_device newCommandQueue];
    _have_frame = false;
    _passthrough_ts = 0;
    _frame_stall_notified = false;
    _frame_sem = dispatch_semaphore_create(mac_shell::FRAMES_IN_FLIGHT);
    _ring_index = 0;
    CVMetalTextureCacheCreate(kCFAllocatorDefault, nil, _device, nil,
                              &_tex_cache);

    id<MTLLibrary> lib = [self loadLibrary];
    if (!lib)
        return nil;

    auto make_pipeline = [&](NSString *vfn, NSString *ffn, bool blend,
                             MTKView *v) -> id<MTLRenderPipelineState> {
        MTLRenderPipelineDescriptor *d =
            [[MTLRenderPipelineDescriptor alloc] init];
        d.vertexFunction = [lib newFunctionWithName:vfn];
        d.fragmentFunction = [lib newFunctionWithName:ffn];
        d.colorAttachments[0].pixelFormat = v.colorPixelFormat;
        d.depthAttachmentPixelFormat = v.depthStencilPixelFormat;
        if (blend) {
            d.colorAttachments[0].blendingEnabled = YES;
            d.colorAttachments[0].sourceRGBBlendFactor =
                MTLBlendFactorSourceAlpha;
            d.colorAttachments[0].destinationRGBBlendFactor =
                MTLBlendFactorOneMinusSourceAlpha;
            d.colorAttachments[0].sourceAlphaBlendFactor =
                MTLBlendFactorSourceAlpha;
            d.colorAttachments[0].destinationAlphaBlendFactor =
                MTLBlendFactorOneMinusSourceAlpha;
        }
        NSError *err = nil;
        id<MTLRenderPipelineState> p =
            [_device newRenderPipelineStateWithDescriptor:d error:&err];
        if (!p)
            std::fprintf(stderr, "renderer: pipeline %s failed: %s\n",
                         vfn.UTF8String,
                         err.localizedDescription.UTF8String);
        return p;
    };

    _passthrough_pipeline = make_pipeline(@"passthrough_vertex",
                                          @"passthrough_fragment", false, view);
    _panel_pipeline =
        make_pipeline(@"panel_vertex_main", @"panel_fragment_main", true, view);
    _blit_pipeline =
        make_pipeline(@"panel_vertex_main", @"blit_fragment", true, view);
    _shadow_pipeline =
        make_pipeline(@"panel_vertex_main", @"shadow_fragment", true, view);
    _glass_pipeline =
        make_pipeline(@"panel_vertex_main", @"glass_fragment", true, view);
    _key_pipeline =
        make_pipeline(@"key_vertex_main", @"key_fragment_main", true, view);
    _sphere_pipeline = make_pipeline(@"sphere_vertex_main",
                                     @"sphere_fragment_main", true, view);
    _glow_pipeline = make_pipeline(@"sphere_vertex_main",
                                   @"sphere_glow_fragment_main", true, view);
    _capsule_pipeline = make_pipeline(@"capsule_vertex_main",
                                      @"capsule_fragment_main", true, view);
    _solid_pipeline =
        make_pipeline(@"solid_vertex_main", @"solid_fragment_main", true, view);
    if (!_passthrough_pipeline || !_panel_pipeline || !_blit_pipeline ||
        !_shadow_pipeline || !_glass_pipeline || !_key_pipeline ||
        !_sphere_pipeline || !_glow_pipeline || !_capsule_pipeline ||
        !_solid_pipeline)
        return nil;

    MTLDepthStencilDescriptor *dd = [[MTLDepthStencilDescriptor alloc] init];
    dd.depthCompareFunction = MTLCompareFunctionLess;
    dd.depthWriteEnabled = YES;
    _depth_test = [_device newDepthStencilStateWithDescriptor:dd];
    dd.depthCompareFunction = MTLCompareFunctionLess;
    dd.depthWriteEnabled = NO;
    _depth_read = [_device newDepthStencilStateWithDescriptor:dd];
    dd.depthCompareFunction = MTLCompareFunctionAlways;
    dd.depthWriteEnabled = NO;
    _depth_none = [_device newDepthStencilStateWithDescriptor:dd];
    dd.depthCompareFunction = MTLCompareFunctionAlways;
    dd.depthWriteEnabled = YES;
    _depth_write_always = [_device newDepthStencilStateWithDescriptor:dd];

    // 1x1 zero depth map: "no LiDAR reading anywhere" → never occludes.
    {
        MTLTextureDescriptor *td = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatR32Float
                                         width:1
                                        height:1
                                     mipmapped:NO];
        td.usage = MTLTextureUsageShaderRead;
        _depth_map_dummy = [_device newTextureWithDescriptor:td];
        float zero = 0.0f;
        [_depth_map_dummy replaceRegion:MTLRegionMake2D(0, 0, 1, 1)
                            mipmapLevel:0
                              withBytes:&zero
                            bytesPerRow:4];
    }
    _have_depth_map = false;
    _have_intrinsics = false;
    _orient_bucket = 0;
    std::memset(&_occl, 0, sizeof(_occl));
    _kbd_tex_version = 0;
    _launcher_tex_version = 0;
    _perf.enable();
    _dock_surface.resize(mac_shell::DOCK_TEX_W, mac_shell::DOCK_TEX_H);

    _kbd_anim.show.init(0.0f, mac_shell::theme::SPRING_OPEN_STIFFNESS,
                        mac_shell::theme::SPRING_OPEN_DAMPING);
    _launcher_anim.show.init(0.0f, mac_shell::theme::SPRING_OPEN_STIFFNESS,
                             mac_shell::theme::SPRING_OPEN_DAMPING);
    for (mac_shell::overlay_anim *oa :
         {&_welcome_anim, &_onboard_anim, &_cheat_anim, &_toast_anim})
        oa->show.init(0.0f, mac_shell::theme::SPRING_OPEN_STIFFNESS,
                      mac_shell::theme::SPRING_OPEN_DAMPING);
    _welcome_surface.resize(mac_shell::WELCOME_TEX_W,
                            mac_shell::WELCOME_TEX_H);
    _onboard_surface.resize(mac_shell::ONBOARD_TEX_W,
                            mac_shell::ONBOARD_TEX_H);
    _cheat_surface.resize(mac_shell::CHEAT_TEX_W, mac_shell::CHEAT_TEX_H);
    _toast_surface.resize(mac_shell::TOAST_TEX_W, mac_shell::TOAST_TEX_H);
    _toast_cached_action = false;
    _perm_screen = mac_shell::screen_capture_permitted();
    _perm_ax = mac_shell::accessibility_permitted();
    _perm_checked_at = CACurrentMediaTime();

    // Welcome-card facts: this Mac's WiFi IP, plus our own Bonjour
    // advertisement and a browse that reads it back — the chip goes green
    // only once mDNSResponder is actually answering for us, which is what
    // the iPhone's browse depends on.
    _s->local_ip = mac_shell::wifi_ipv4();
    _s->last_ip_check = CACurrentMediaTime();
    _bonjour_ref = nullptr;
    _bonjour_reg = nullptr;
    _mdns_queue = nullptr;
    if (!_s->replay) {
        // Serial: both DNSService callbacks read bonjour_expected.
        _mdns_queue = dispatch_queue_create("com.spatialos.spatula.mdns",
                                            DISPATCH_QUEUE_SERIAL);
        _s->bonjour_expected =
            std::string("Spatula (") + mac_shell::computer_name() + ")";
        [self startBonjour];
        DNSServiceErrorType err = DNSServiceBrowse(
            &_bonjour_ref, 0, 0, "_spatialbridge._udp", nullptr,
            mac_shell::bonjour_browse_reply, _s);
        if (err == kDNSServiceErr_NoError) {
            DNSServiceSetDispatchQueue(_bonjour_ref, _mdns_queue);
            _s->bonjour_browsing = true;
        } else {
            _bonjour_ref = nullptr;
        }
        _s->bonjour_started_at = CACurrentMediaTime();
    }

    _s->start_time = CACurrentMediaTime();
    _s->last_time = _s->start_time;
    return self;
}

// Advertise this shell as `Spatula (<Computer Name>)` on the UDP port it
// actually bound, so launching from the Dock is as discoverable as launching
// from run-mac.sh. Skipped in replay mode (nothing to connect to) and under
// SPATULA_MAC_NO_BONJOUR=1 (test/screenshot runs must not pollute the
// user's phone list); headless never builds a renderer at all.
- (void)startBonjour {
    if (_bonjour_reg || _s->replay || !_mdns_queue)
        return;
    // Never advertise a port we failed to bind — the phone would tap a card
    // that goes nowhere. The retry re-calls this once the port opens.
    if (!_s->receiver)
        return;
    const char *off = getenv("SPATULA_MAC_NO_BONJOUR");
    if (off && std::strcmp(off, "1") == 0)
        return;
    // NoAutoRename: a rename would leave bonjour_expected — and so the
    // self-check browse — pointing at a name we no longer own. A conflict
    // means a second Spatula on this Mac, which must not double-advertise.
    DNSServiceErrorType err = DNSServiceRegister(
        &_bonjour_reg, kDNSServiceFlagsNoAutoRename, 0,
        _s->bonjour_expected.c_str(), "_spatialbridge._udp", nullptr, nullptr,
        htons((uint16_t)_s->udp_port), 0, nullptr,
        mac_shell::bonjour_register_reply, nullptr);
    if (err == kDNSServiceErr_NoError) {
        DNSServiceSetDispatchQueue(_bonjour_reg, _mdns_queue);
    } else {
        _bonjour_reg = nullptr;
        std::fprintf(stderr,
                     "renderer: Bonjour registration failed (%d) — the iPhone "
                     "will need this Mac's IP typed in\n",
                     (int)err);
    }
}

- (void)dealloc {
    if (_bonjour_reg)
        DNSServiceRefDeallocate(_bonjour_reg);
    if (_bonjour_ref)
        DNSServiceRefDeallocate(_bonjour_ref);
    for (auto &kv : _panel_textures)
        if (kv.second.cv_tex)
            CFRelease(kv.second.cv_tex);
    for (CFTypeRef o : _frame_cf_release)
        CFRelease(o);
    if (_tex_cache)
        CFRelease(_tex_cache);
}

- (id<MTLLibrary>)loadLibrary {
    NSError *err = nil;
    // A precompiled metallib can be supplied, but the default path is runtime
    // compilation of the embedded shaders.metal source (the offline metal
    // toolchain is an optional Xcode download).
    if (const char *env = getenv("SPATULA_MAC_METALLIB")) {
        NSString *path = [NSString stringWithUTF8String:env];
        id<MTLLibrary> lib =
            [_device newLibraryWithURL:[NSURL fileURLWithPath:path]
                                 error:&err];
        if (lib)
            return lib;
        std::fprintf(stderr, "renderer: cannot load metallib %s: %s\n",
                     path.UTF8String, err.localizedDescription.UTF8String);
    }
    id<MTLLibrary> lib = [_device
        newLibraryWithSource:[NSString
                                 stringWithUTF8String:MAC_SHELL_SHADER_SOURCE]
                     options:nil
                       error:&err];
    if (!lib)
        std::fprintf(stderr, "renderer: shader compile failed: %s\n",
                     err.localizedDescription.UTF8String);
    return lib;
}

// Required by MTKViewDelegate; the projection is rebuilt from
// drawableSize every frame, so there is nothing to do here.
- (void)mtkView:(MTKView *)view drawableSizeWillChange:(CGSize)size {
}

// ------------------------------------------------------------------
// texture upload helpers
// ------------------------------------------------------------------

// RGBA8 texture with a full mip chain (crisp minified panel content).
- (id<MTLTexture>)makeMippedTextureW:(NSUInteger)w h:(NSUInteger)h {
    MTLTextureDescriptor *td = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                     width:w
                                    height:h
                                 mipmapped:YES];
    td.usage = MTLTextureUsageShaderRead;
    return [_device newTextureWithDescriptor:td];
}

- (void)uploadTexture:(id<MTLTexture>)tex
                 rgba:(const uint8_t *)rgba
                    w:(NSUInteger)w
                    h:(NSUInteger)h {
    [tex replaceRegion:MTLRegionMake2D(0, 0, w, h)
           mipmapLevel:0
             withBytes:rgba
           bytesPerRow:w * 4];
    if (tex.mipmapLevelCount > 1) {
        id<MTLCommandBuffer> cmd = [_queue commandBuffer];
        id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
        [blit generateMipmapsForTexture:tex];
        [blit endEncoding];
        [cmd commit];
    }
}

// ------------------------------------------------------------------
// per-frame
// ------------------------------------------------------------------

- (simd_float4x4)viewMatrixWithDt:(float)dt {
    // Pose/frame time alignment: poses arrive in one datagram, camera frames
    // are chunk-reassembled and JPEG-decoded, so the image on screen is always
    // older than the newest pose. Drawing world-locked panels against the
    // newest pose makes them swim across the passthrough on every head turn,
    // so the view comes from the pose that was current when THIS frame was
    // captured. No frame yet (or one older than the pose ring) falls back to
    // the newest pose, which is also the fly-cam-free live path at startup.
    float head_pos[3], head_quat[4];
    float lag_ms = 0.0f;
    bool have_head =
        _have_frame && _passthrough_ts != 0 &&
        _s->world->head_pose_at(_passthrough_ts, head_pos, head_quat, &lag_ms);
    if (!have_head) {
        lag_ms = 0.0f;
        have_head = _s->world->head_pose(head_pos, head_quat);
    }
    if (have_head) {
        _s->world->set_view_lag_ms(lag_ms);
        // Phone-orientation compensation: derive the camera's gravity roll
        // from the raw pose, snap it to the nearest 90° bucket (with
        // hysteresis), and roll-correct the view so the horizon stays level
        // however the phone is held. The passthrough UV mapping rotates by
        // the SAME bucket (drawInMTKView), so world-anchored content stays
        // glued to the image through an orientation change.
        float uxuy[2];
        if (_s->world->head_up_in_camera(uxuy))
            _orient_bucket = mac_shell::orientation_bucket_from_gravity(
                uxuy[0], uxuy[1], _orient_bucket);

        simd_float4x4 rot = mac_shell::rotation_from_quat(simd_make_float4(
            head_quat[0], head_quat[1], head_quat[2], head_quat[3]));
        simd_float4x4 rot_inv = simd_transpose(rot);
        simd_float4x4 view = simd_mul(
            rot_inv, mac_shell::translation(simd_make_float3(
                         -head_pos[0], -head_pos[1], -head_pos[2])));
        if (_orient_bucket != 0) {
            float a = mac_shell::orientation_bucket_angle(_orient_bucket);
            float ca = std::cos(a), sa = std::sin(a);
            simd_float4x4 rz = matrix_identity_float4x4;
            rz.columns[0] = simd_make_float4(ca, sa, 0, 0);
            rz.columns[1] = simd_make_float4(-sa, ca, 0, 0);
            view = simd_mul(rz, view);
        }
        return view;
    }
    _orient_bucket = 0;  // fly-cam is already upright

    // Fly-cam. Mouse drag → yaw/pitch, WASD (+QE vertical) → translate.
    _s->cam_yaw -= _s->drag_dx * 0.005f;
    _s->cam_pitch -= _s->drag_dy * 0.005f;
    _s->cam_pitch = std::fmax(-1.5f, std::fmin(1.5f, _s->cam_pitch));
    _s->drag_dx = _s->drag_dy = 0;

    float cy = std::cos(_s->cam_yaw), sy = std::sin(_s->cam_yaw);
    float cp = std::cos(_s->cam_pitch), sp = std::sin(_s->cam_pitch);
    simd_float3 fwd = {-sy * cp, sp, -cy * cp};
    simd_float3 right = {cy, 0, -sy};
    simd_float3 up = {0, 1, 0};

    const float speed = 1.5f * dt;
    auto down = [&](unsigned short k) { return _s->keys_down.count(k) > 0; };
    if (down(13)) _s->cam_pos += fwd * speed;    // W
    if (down(1)) _s->cam_pos -= fwd * speed;     // S
    if (down(0)) _s->cam_pos -= right * speed;   // A
    if (down(2)) _s->cam_pos += right * speed;   // D
    if (down(12)) _s->cam_pos -= up * speed;     // Q
    if (down(14)) _s->cam_pos += up * speed;     // E

    simd_float4x4 ry = mac_shell::rotation_from_quat(
        simd_make_float4(0, std::sin(_s->cam_yaw * 0.5f), 0,
                         std::cos(_s->cam_yaw * 0.5f)));
    simd_float4x4 rx = mac_shell::rotation_from_quat(
        simd_make_float4(std::sin(_s->cam_pitch * 0.5f), 0, 0,
                         std::cos(_s->cam_pitch * 0.5f)));
    simd_float4x4 rot = simd_mul(ry, rx);
    return simd_mul(simd_transpose(rot),
                    mac_shell::translation(-_s->cam_pos));
}

- (void)updatePassthrough {
    sb_frame_t frame;
    if (!_s->receiver || !sb_get_latest_frame(_s->receiver, &frame))
        return;
    __strong id<MTLTexture> &slot = _passthrough_ring[_ring_index];
    if (!slot || slot.width != frame.width || slot.height != frame.height) {
        MTLTextureDescriptor *td = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                         width:frame.width
                                        height:frame.height
                                     mipmapped:NO];
        td.usage = MTLTextureUsageShaderRead;
        slot = [_device newTextureWithDescriptor:td];
    }
    [slot replaceRegion:MTLRegionMake2D(0, 0, frame.width, frame.height)
            mipmapLevel:0
              withBytes:frame.rgba
            bytesPerRow:frame.width * 4];
    _passthrough_tex = slot;
    _passthrough_ts = frame.timestamp_ns;
    _have_frame = true;
    sb_free_frame(&frame);
}

- (void)updateDepthMap {
    if (!_s->receiver)
        return;
    sb_intrinsics_t intr;
    if (sb_get_latest_intrinsics(_s->receiver, &intr)) {
        _intrinsics = intr;
        _have_intrinsics = true;
    }
    sb_depth_t depth;
    if (!sb_get_latest_depth(_s->receiver, &depth))
        return;
    __strong id<MTLTexture> &slot = _depth_ring[_ring_index];
    if (!slot || slot.width != depth.width || slot.height != depth.height) {
        MTLTextureDescriptor *td = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatR32Float
                                         width:depth.width
                                        height:depth.height
                                     mipmapped:NO];
        td.usage = MTLTextureUsageShaderRead;
        slot = [_device newTextureWithDescriptor:td];
    }
    // Temporal EMA: blends LiDAR noise out of static surfaces (reduces
    // occlusion-edge crawl). Invalid readings (0) pass through as invalid so
    // stale depth never lingers where the sensor lost the surface.
    const size_t count = (size_t)depth.width * depth.height;
    if (_depth_ema.size() != count) {
        _depth_ema.assign(depth.depth, depth.depth + count);
    } else {
        constexpr float K = 0.6f;  // weight of the new frame
        for (size_t i = 0; i < count; i++) {
            float n = depth.depth[i];
            float &e = _depth_ema[i];
            if (n <= 0.0f || e <= 0.0f)
                e = n;
            else
                e += K * (n - e);
        }
    }
    [slot replaceRegion:MTLRegionMake2D(0, 0, depth.width, depth.height)
            mipmapLevel:0
              withBytes:_depth_ema.data()
            bytesPerRow:depth.width * 4];
    _depth_map_tex = slot;
    _have_depth_map = true;
    sb_free_depth(&depth);
}

- (void)releaseFrameObjects {
    for (CFTypeRef o : _frame_cf_release)
        CFRelease(o);
    _frame_cf_release.clear();
}

- (void)drawInMTKView:(MTKView *)view {
    double now = CACurrentMediaTime();
    float dt = (float)(now - _s->last_time);
    _s->last_time = now;
    if (dt <= 0 || dt > 0.25f)
        dt = 1.0f / 60.0f;
    float t = (float)(now - _s->start_time);

    if (_s->running && !_s->running->load()) {
        stop_app_loop();
        return;
    }
    if (_s->exit_after > 0 && now - _s->start_time > _s->exit_after) {
        _s->running->store(false);
        stop_app_loop();
        return;
    }

    if (_s->retry && _s->retry->poll(now)) {
        _s->receiver = _s->retry->receiver();
        _s->startup_error.clear();
        _s->startup_error_title.clear();
        [self startBonjour];
    }

    _s->world->tick(dt);

    // Gate on a free in-flight slot before any CPU→GPU texture write.
    dispatch_semaphore_wait(_frame_sem, DISPATCH_TIME_FOREVER);
    _ring_index = (_ring_index + 1) % mac_shell::FRAMES_IN_FLIGHT;
    [self updatePassthrough];
    [self updateDepthMap];

    // Connection state machine: drives the welcome card and the
    // stream-dropped toast.
    float pkt_rate = _s->receiver ? sb_get_packet_rate(_s->receiver) : 0.0f;
    _s->welcome.tick(pkt_rate, dt);
    if (_s->welcome.just_lost)
        _s->world->post_toast(
            "iPhone stream dropped - reopen SpatialBridge on your phone to "
            "reconnect.");
    _s->welcome.consume_events();
    if (_s->welcome.phase != mac_shell::link_phase::connected &&
        now - _s->last_ip_check > mac_shell::IP_REFRESH_S) {
        _s->local_ip = mac_shell::wifi_ipv4();
        _s->last_ip_check = now;
    }

    _perf.enter();
    auto panels = _s->world->snapshot_render_panels(
        [self](uint64_t handle) -> uint64_t {
            auto it = self->_panel_textures.find(handle);
            return it != self->_panel_textures.end() && it->second.tex
                       ? it->second.version
                       : 0;
        });
    _perf.leave();
    if (_s->capture) {
        std::vector<uint64_t> live;
        for (auto &p : panels)
            live.push_back(p.handle);
        _s->capture->gc(live);
        // Drop cached textures for dead panels (the close animation holds
        // its own reference in panel_anim::cached_tex).
        for (auto it = _panel_textures.begin(); it != _panel_textures.end();) {
            bool alive = false;
            for (auto h : live)
                if (h == it->first)
                    alive = true;
            if (alive) {
                ++it;
                continue;
            }
            if (it->second.cv_tex)
                _frame_cf_release.push_back(it->second.cv_tex);
            it = _panel_textures.erase(it);
        }
    }

    MTLRenderPassDescriptor *rpd = view.currentRenderPassDescriptor;
    id<CAMetalDrawable> drawable = view.currentDrawable;
    if (!rpd || !drawable) {
        [self releaseFrameObjects];
        dispatch_semaphore_signal(_frame_sem);
        return;
    }

    id<MTLCommandBuffer> cmd = [_queue commandBuffer];
    id<MTLRenderCommandEncoder> enc =
        [cmd renderCommandEncoderWithDescriptor:rpd];

    float aspect = (float)view.drawableSize.width /
                   (float)std::fmax(1.0, view.drawableSize.height);
    simd_float4x4 proj = mac_shell::perspective_rh(
        mac_shell::RENDER_FOVY_RAD, aspect, mac_shell::RENDER_NEAR_Z,
        mac_shell::RENDER_FAR_Z);
    simd_float4x4 view_mat = [self viewMatrixWithDt:dt];
    simd_float4x4 view_proj = simd_mul(proj, view_mat);

    // 1. passthrough (head-locked, rearmost). Writes per-pixel depth from the
    // LiDAR map so real-world geometry closer than a panel occludes it. The
    // UV mapping folds in the intrinsics scale AND the snapped orientation
    // roll from viewMatrixWithDt; the colour-aligned depth map samples with
    // the SAME mapping.
    int occl_mode = _s->world->depth_occlusion_mode();  // 0 off,1 hard,2 soft
    std::memset(&_occl, 0, sizeof(_occl));

    // Camera frames can die while ARKit keeps posing. Fade the frozen photo
    // out rather than letting world-locked panels swim over it, and say why
    // once — the phone is the thing the user has to go fix.
    float pt_fade = 1.0f;
    if (_have_frame) {
        const int64_t age_ms =
            _s->receiver ? sb_get_frame_age_ms(_s->receiver) : -1;
        if (age_ms >= 0) {
            pt_fade = mac_shell::passthrough_fade((float)age_ms / 1000.0f);
            const bool stalled =
                (float)age_ms / 1000.0f > mac_shell::FRAME_STALE_AFTER_S;
            if (stalled && !_frame_stall_notified) {
                _frame_stall_notified = true;
                _s->world->post_toast(
                    "Camera stream stalled - check the phone");
            } else if (!stalled) {
                _frame_stall_notified = false;
            }
        }
    }

    if (_have_frame && _passthrough_tex && pt_fade > 0.0f) {
        mac_shell::passthrough_uniforms pu;
        std::memset(&pu, 0, sizeof(pu));
        float tan_half_y = std::tan(mac_shell::RENDER_FOVY_RAD * 0.5f);
        float tan_half_x = tan_half_y * aspect;
        float uv_m[4] = {1, 0, 0, 1}, offset[2] = {0, 0};
        if (_have_intrinsics)
            mac_shell::passthrough_uv_mapping(_intrinsics, tan_half_x,
                                              tan_half_y, _orient_bucket,
                                              uv_m, offset);
        pu.uv_m = simd_make_float4(uv_m[0], uv_m[1], uv_m[2], uv_m[3]);
        pu.uv_offset = simd_make_float2(offset[0], offset[1]);
        pu.fade = pt_fade;
        pu.depth_near_far = simd_make_float2(mac_shell::RENDER_NEAR_Z,
                                             mac_shell::RENDER_FAR_Z);
        bool use_depth = occl_mode > 0 && _have_depth_map;
        pu.have_depth = use_depth ? 1.0f : 0.0f;
        pu.depth_push = occl_mode == 2
                            ? mac_shell::OCCLUSION_SOFT_BAND_M * 0.5f
                            : 0.0f;
        if (occl_mode == 2 && use_depth) {
            float zs = mac_shell::RENDER_FAR_Z /
                       (mac_shell::RENDER_NEAR_Z - mac_shell::RENDER_FAR_Z);
            _occl.uv_m = pu.uv_m;
            _occl.uv_offset = pu.uv_offset;
            _occl.viewport =
                simd_make_float2((float)view.drawableSize.width,
                                 (float)view.drawableSize.height);
            _occl.depth_consts =
                simd_make_float2(zs, mac_shell::RENDER_NEAR_Z);
            _occl.band_m = mac_shell::OCCLUSION_SOFT_BAND_M;
        }
        [enc setRenderPipelineState:_passthrough_pipeline];
        [enc setDepthStencilState:_depth_write_always];
        [enc setFragmentTexture:_passthrough_tex atIndex:0];
        [enc setFragmentTexture:_have_depth_map ? _depth_map_tex
                                                : _depth_map_dummy
                        atIndex:1];
        [enc setFragmentBytes:&pu length:sizeof(pu) atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle
                vertexStart:0
                vertexCount:3];
    }

    // 2. panels (world space, spring-animated, sorted back-to-front).
    [self drawPanels:panels
             encoder:enc
            viewProj:view_proj
                view:view_mat
                  dt:dt
                time:t];

    // 3. virtual keyboard: glass plate + instanced keycaps.
    _perf.enter();
    const auto kb = _s->world->snapshot_keyboard(_kbd_tex ? _kbd_tex_version
                                                          : 0);
    _perf.leave();
    [self drawKeyboard:enc keyboard:kb viewProj:view_proj dt:dt time:t];

    // 4. hands: fingertip reticles with optional joints and bones.
    [self drawHands:enc
           keyboard:kb
           viewProj:view_proj
               view:view_mat
                 dt:dt
               time:t];

    // 5. keyboard-summon hold-progress arc above the palm (billboarded).
    [self drawSummonArcs:enc viewProj:view_proj view:view_mat];

    // 6. radial launcher (screen-space HUD).
    [self drawLauncher:enc aspect:aspect dt:dt];

    // 7. dock/status HUD strip (screen-space, bottom edge).
    [self drawDockWithEncoder:enc panels:panels view:view];

    // 8. welcome / connection-lost / startup-error card.
    bool scene_empty = panels.empty() && !_s->world->keyboard_visible() &&
                       !_s->world->launcher_visible();
    [self drawWelcome:enc view:view sceneEmpty:scene_empty dt:dt time:t];

    // 9. toast, cheat sheet, and (topmost, modal) onboarding.
    [self drawToast:enc view:view dt:dt];
    [self drawCheatSheet:enc view:view dt:dt];
    [self drawOnboarding:enc view:view dt:dt];

    [enc endEncoding];

    // Framebuffer grabs. The verification env grab (SPATULA_MAC_SCREENSHOT_AT
    // seconds in, once) arms the same queue the `screenshot` control verb
    // uses, so both are served by whichever frame comes next.
    if (_s->grabber) {
        if (!_s->screenshot_path.empty() && !_s->screenshot_done &&
            now - _s->start_time > _s->screenshot_at) {
            _s->screenshot_done = true;
            _s->grabber->submit_detached(_s->screenshot_path);
        }
        std::vector<mac_shell::frame_grabber::request> shots =
            _s->grabber->take_pending();
        if (!shots.empty()) {
            id<MTLTexture> target = drawable.texture;
            mac_shell::frame_grabber *grabber = _s->grabber;
            [cmd addCompletedHandler:^(id<MTLCommandBuffer> _Nonnull) {
              for (const auto &shot : shots) {
                  std::string err;
                  bool ok = [ShellRenderer writeTexture:target
                                                  toPNG:shot.path
                                                    err:err];
                  grabber->complete(shot.id, ok, err);
              }
            }];
        }
    }

    {
        dispatch_semaphore_t sem = _frame_sem;
        std::vector<CFTypeRef> cf = std::move(_frame_cf_release);
        _frame_cf_release.clear();
        [cmd addCompletedHandler:^(id<MTLCommandBuffer> _Nonnull) {
          for (CFTypeRef o : cf)
              CFRelease(o);
          dispatch_semaphore_signal(sem);
        }];
    }
    [cmd presentDrawable:drawable];
    [cmd commit];
    CVMetalTextureCacheFlush(_tex_cache, 0);
    _perf.end_frame(now);
}

+ (bool)writeTexture:(id<MTLTexture>)tex
               toPNG:(const std::string &)path
                 err:(std::string &)err {
    NSUInteger w = tex.width, h = tex.height;
    if (w == 0 || h == 0) {
        err = "empty_frame";
        return false;
    }
    std::vector<uint8_t> data(w * h * 4);
    [tex getBytes:data.data()
        bytesPerRow:w * 4
         fromRegion:MTLRegionMake2D(0, 0, w, h)
        mipmapLevel:0];
    // BGRA → RGBA
    for (size_t i = 0; i < data.size(); i += 4)
        std::swap(data[i], data[i + 2]);
    bool ok = mac_shell::write_rgba_png(data.data(), (int)w, (int)h, w * 4,
                                        path, err);
    if (ok)
        std::fprintf(stderr, "renderer: screenshot written to %s\n",
                     path.c_str());
    else
        std::fprintf(stderr, "renderer: screenshot %s failed: %s\n",
                     path.c_str(), err.c_str());
    return ok;
}

@end
