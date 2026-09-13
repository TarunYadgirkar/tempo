// capture.mm — see capture.h.

#import <AppKit/AppKit.h>
#import <ApplicationServices/ApplicationServices.h>
#import <CoreGraphics/CoreGraphics.h>
#import <CoreMedia/CoreMedia.h>
#import <CoreVideo/CoreVideo.h>
#import <ScreenCaptureKit/ScreenCaptureKit.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

#include "platform/capture.h"
#include "platform/png_write.h"
#include "core/scene.h"

// ------------------------------------------------------------------
// stream output shim: keeps the latest CVPixelBuffer per stream
// ------------------------------------------------------------------

@interface CaptureStreamOutput : NSObject <SCStreamOutput, SCStreamDelegate>
@property(nonatomic, assign) CVPixelBufferRef latest;  // retained
@property(nonatomic, strong) NSRecursiveLock *lock;
// Invoked from didStopWithError (source app quit, window destroyed, SCK
// error). Cleared before deliberate stopCapture calls so gc/teardown never
// re-enter the degrade path.
@property(nonatomic, copy) void (^onStopped)(NSString *reason);
@end

@implementation CaptureStreamOutput
- (instancetype)init {
    self = [super init];
    if (self)
        _lock = [[NSRecursiveLock alloc] init];
    return self;
}
- (void)stream:(SCStream *)stream
    didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
                   ofType:(SCStreamOutputType)type {
    (void)stream;
    if (type != SCStreamOutputTypeScreen)
        return;
    CVPixelBufferRef pb = CMSampleBufferGetImageBuffer(sampleBuffer);
    if (!pb)
        return;
    [self.lock lock];
    if (self.latest)
        CVPixelBufferRelease(self.latest);
    self.latest = CVPixelBufferRetain(pb);
    [self.lock unlock];
}
- (void)stream:(SCStream *)stream didStopWithError:(NSError *)error {
    (void)stream;
    NSString *reason =
        error ? error.localizedDescription : @"stream stopped";
    std::fprintf(stderr, "capture: stream stopped: %s\n", reason.UTF8String);
    if (self.onStopped)
        self.onStopped(reason);
}
- (CVPixelBufferRef)copyLatest {
    [self.lock lock];
    CVPixelBufferRef pb = self.latest ? CVPixelBufferRetain(self.latest) : nil;
    [self.lock unlock];
    return pb;
}
- (void)dealloc {
    if (_latest)
        CVPixelBufferRelease(_latest);
}
@end

namespace mac_shell {

namespace {

bool headless_mode() {
    const char *e = getenv("SPATULA_MAC_HEADLESS");
    return e && std::strcmp(e, "1") == 0;
}

// On by default for interactive runs: the Accessibility grant is the user's
// consent, and `open` cannot pass an opt-in env var to the installed app.
bool inject_enabled() {
    const char *e = getenv("SPATULA_MAC_INJECT");
    if (e)
        return std::strcmp(e, "1") == 0;
    return !headless_mode();
}

constexpr double GEOMETRY_POLL_INTERVAL_S = 1.0;

// One stream configuration shape everywhere (initial capture + live resize).
SCStreamConfiguration *make_stream_config(size_t width, size_t height) {
    SCStreamConfiguration *config = [[SCStreamConfiguration alloc] init];
    config.width = width;
    config.height = height;
    config.pixelFormat = kCVPixelFormatType_32BGRA;
    config.minimumFrameInterval = CMTimeMake(1, 30);
    config.queueDepth = 3;
    config.showsCursor = NO;
    return config;
}

// Post a down/up event pair to a pid, releasing both events.
void post_updown(int32_t pid, CGEventRef down, CGEventRef up) {
    CGEventPostToPid(pid, down);
    CGEventPostToPid(pid, up);
    CFRelease(down);
    CFRelease(up);
}

void stop_stream(SCStream *stream) {
    if (stream)
        [stream stopCaptureWithCompletionHandler:^(NSError *_Nullable) {}];
}

// Teardown variant that waits for the stop to land. stopCapture is an async
// XPC round-trip to replayd; a process that exits before it completes leaves
// replayd holding the stream, so the screen-recording indicator stays lit
// until the daemon is restarted. Bounded so a wedged daemon cannot hang quit.
constexpr int64_t STOP_TIMEOUT_NS = 2LL * NSEC_PER_SEC;

void stop_stream_sync(SCStream *stream) {
    if (!stream)
        return;
    dispatch_semaphore_t done = dispatch_semaphore_create(0);
    [stream stopCaptureWithCompletionHandler:^(NSError *_Nullable) {
      dispatch_semaphore_signal(done);
    }];
    dispatch_semaphore_wait(
        done, dispatch_time(DISPATCH_TIME_NOW, STOP_TIMEOUT_NS));
}

CGPoint cg_window_point(const CGRect &frame, float x, float y) {
    screen_point p = window_point(frame.origin.x, frame.origin.y, x, y);
    return CGPointMake(p.x, p.y);
}

// The geometry poll only refreshes stream_entry::window_frame once a second,
// so a window dragged since the last tick would take a click at its old
// position — into whatever is there now. This is the synchronous single-window
// read (kCGWindowBounds is the same rect SCWindow.frame reports), cheap enough
// to run per injected event; false leaves the caller on its cached frame.
bool live_window_frame(uint32_t window_id, CGRect &out) {
    if (!window_id)
        return false;
    CFArrayRef list = CGWindowListCopyWindowInfo(
        kCGWindowListOptionIncludingWindow, (CGWindowID)window_id);
    if (!list)
        return false;
    bool ok = false;
    if (CFArrayGetCount(list) > 0) {
        CFDictionaryRef d =
            (CFDictionaryRef)CFArrayGetValueAtIndex(list, 0);
        CFDictionaryRef bounds =
            (CFDictionaryRef)CFDictionaryGetValue(d, kCGWindowBounds);
        ok = bounds && CGRectMakeWithDictionaryRepresentation(bounds, &out);
    }
    CFRelease(list);
    return ok;
}

// An unresponsive app can sit on an AX request until the messaging timeout;
// keep that well under a frame's worth of user-visible delay.
constexpr float AX_TIMEOUT_S = 0.25f;
constexpr double AX_FRAME_MATCH_TOLERANCE_PT = 1.0;

bool ax_frame(AXUIElementRef w, CGRect &out) {
    CFTypeRef pos_v = nullptr, size_v = nullptr;
    CGPoint pos = CGPointZero;
    CGSize size = CGSizeZero;
    bool ok =
        AXUIElementCopyAttributeValue(w, kAXPositionAttribute, &pos_v) ==
            kAXErrorSuccess &&
        AXUIElementCopyAttributeValue(w, kAXSizeAttribute, &size_v) ==
            kAXErrorSuccess &&
        AXValueGetValue((AXValueRef)pos_v, kAXValueTypeCGPoint, &pos) &&
        AXValueGetValue((AXValueRef)size_v, kAXValueTypeCGSize, &size);
    if (pos_v)
        CFRelease(pos_v);
    if (size_v)
        CFRelease(size_v);
    out = CGRectMake(pos.x, pos.y, size.width, size.height);
    return ok;
}

bool ax_title_is(AXUIElementRef w, const std::string &want) {
    if (want.empty())
        return false;
    CFTypeRef v = nullptr;
    if (AXUIElementCopyAttributeValue(w, kAXTitleAttribute, &v) !=
            kAXErrorSuccess ||
        !v)
        return false;
    bool ok = CFGetTypeID(v) == CFStringGetTypeID() &&
              want == [(__bridge NSString *)v UTF8String];
    CFRelease(v);
    return ok;
}

bool frames_match(const CGRect &a, const CGRect &b) {
    return std::abs(a.origin.x - b.origin.x) <= AX_FRAME_MATCH_TOLERANCE_PT &&
           std::abs(a.origin.y - b.origin.y) <= AX_FRAME_MATCH_TOLERANCE_PT &&
           std::abs(a.size.width - b.size.width) <=
               AX_FRAME_MATCH_TOLERANCE_PT &&
           std::abs(a.size.height - b.size.height) <=
               AX_FRAME_MATCH_TOLERANCE_PT;
}

// Best-effort: make the captured source window its app's main + focused window
// over the Accessibility API and raise it, WITHOUT activating the app — which
// would yank the user's frontmost app away from Spatula. False means the
// caller should fall back to activating.
//
// AXUIElement has no public windowID lookup, so the app's AX window list has to
// be identified some other way. Title first: an app's AX frame and its
// kCGWindowBounds are not always the same rect (observed on a live Electron
// window reporting AX (0,33 1512x949) for a CG (224,227 1064x668) window), so
// frame matching alone silently misses. Frame is the tiebreak for untitled or
// duplicate-titled windows, and a sole window is taken as-is.
bool ax_focus_window(int32_t pid, const CGRect &want,
                     const std::string &want_title) {
    AXUIElementRef app = AXUIElementCreateApplication((pid_t)pid);
    if (!app)
        return false;
    AXUIElementSetMessagingTimeout(app, AX_TIMEOUT_S);
    CFArrayRef windows = nullptr;
    if (AXUIElementCopyAttributeValue(app, kAXWindowsAttribute,
                                      (CFTypeRef *)&windows) !=
            kAXErrorSuccess ||
        !windows) {
        CFRelease(app);
        return false;
    }

    CFIndex count = CFArrayGetCount(windows);
    AXUIElementRef by_title = nullptr, by_frame = nullptr;
    int title_hits = 0;
    for (CFIndex i = 0; i < count; i++) {
        AXUIElementRef w = (AXUIElementRef)CFArrayGetValueAtIndex(windows, i);
        if (ax_title_is(w, want_title)) {
            title_hits++;
            if (!by_title)
                by_title = w;
        }
        CGRect f = CGRectZero;
        if (!by_frame && ax_frame(w, f) && frames_match(f, want))
            by_frame = w;
    }
    AXUIElementRef target = title_hits == 1 ? by_title : nullptr;
    if (!target)
        target = by_frame;
    if (!target && count == 1)
        target = (AXUIElementRef)CFArrayGetValueAtIndex(windows, 0);

    bool ok = false;
    if (target) {
        bool main_ok =
            AXUIElementSetAttributeValue(target, kAXMainAttribute,
                                         kCFBooleanTrue) == kAXErrorSuccess;
        bool focus_ok =
            AXUIElementSetAttributeValue(target, kAXFocusedAttribute,
                                         kCFBooleanTrue) == kAXErrorSuccess;
        AXUIElementPerformAction(target, kAXRaiseAction);
        ok = main_ok || focus_ok;
    }
    CFRelease(windows);
    CFRelease(app);
    return ok;
}

// launch-app with no matching window: open (or reopen) the app, then poll
// shareable content until its window exists.
constexpr int LAUNCH_ATTEMPTS = 12;
constexpr int64_t LAUNCH_RETRY_NS = 500 * NSEC_PER_MSEC;
constexpr CGFloat MIN_WINDOW_PT = 64;

NSURL *app_url_for(const std::string &target) {
    NSString *t = [NSString stringWithUTF8String:target.c_str()];
    NSWorkspace *ws = [NSWorkspace sharedWorkspace];
    if (NSURL *u = [ws URLForApplicationWithBundleIdentifier:t])
        return u;
    NSString *name = [t.pathExtension isEqualToString:@"app"]
                         ? t
                         : [t stringByAppendingPathExtension:@"app"];
    NSArray<NSString *> *dirs = @[
        @"/Applications", @"/System/Applications",
        @"/System/Applications/Utilities", @"/System/Library/CoreServices",
        [NSHomeDirectory() stringByAppendingPathComponent:@"Applications"]
    ];
    for (NSString *dir in dirs) {
        NSString *p = [dir stringByAppendingPathComponent:name];
        if ([[NSFileManager defaultManager] fileExistsAtPath:p])
            return [NSURL fileURLWithPath:p];
    }
    return nil;
}

// A running app receives the reopen event and makes a new window; a
// stopped one launches. activates=NO so the user's focus stays put.
bool open_app(const std::string &target) {
    NSURL *url = app_url_for(target);
    if (!url)
        return false;
    NSWorkspaceOpenConfiguration *cfg =
        [NSWorkspaceOpenConfiguration configuration];
    cfg.activates = NO;
    std::string name = target;
    [[NSWorkspace sharedWorkspace]
        openApplicationAtURL:url
               configuration:cfg
           completionHandler:^(NSRunningApplication *, NSError *err) {
             if (err)
                 std::fprintf(stderr, "capture: open '%s' failed: %s\n",
                              name.c_str(),
                              err.localizedDescription.UTF8String);
           }];
    std::fprintf(stderr, "capture: opening '%s' for capture\n",
                 target.c_str());
    return true;
}

// Prefer a window on the current Space, then the largest one.
double window_rank(SCWindow *w) {
    return (w.isOnScreen ? 1e12 : 0.0) + w.frame.size.width * w.frame.size.height;
}

}  // namespace

bool screen_capture_permitted() { return CGPreflightScreenCaptureAccess(); }

bool accessibility_permitted() { return AXIsProcessTrusted(); }

struct source_window_info {
    CGRect frame = CGRectZero;  // global screen coords, re-read live
    uint32_t window_id = 0;
    std::string title;
};

struct stream_entry {
    SCStream *stream = nil;
    CaptureStreamOutput *output = nil;
    CGRect window_frame = CGRectZero;  // global screen coords, kept fresh by
                                       // the geometry poll
    int32_t pid = 0;
    uint32_t window_id = 0;  // CGWindowID, for geometry-poll matching
    std::string title;
    geometry_miss_tracker misses;
};

// Lifetime: capture_manager owns the only strong ref; every SCK completion
// block / stream callback holds a weak_ptr and bails once the manager is
// gone. `world` is guarded separately (world_lock) so a block that outlived
// ~capture_manager but still holds impl alive never touches a scene that is
// being torn down. Lock order: world_lock → mutex → (scene mutex via world
// calls happens only with mutex released, since the scene calls back into
// the injector under its own mutex).
struct capture_manager::impl {
    std::shared_mutex world_lock;
    scene *world = nullptr;
    std::mutex mutex;
    std::map<uint64_t, stream_entry> streams;
    dispatch_queue_t sample_queue =
        dispatch_queue_create("mac-shell.capture", DISPATCH_QUEUE_SERIAL);
    double last_geometry_poll = 0;
    bool geometry_poll_inflight = false;

    template <class F>
    void with_world(F &&fn) {
        std::shared_lock<std::shared_mutex> guard(world_lock);
        if (world)
            fn(*world);
    }

    void handle_stream_ended(uint64_t handle, const std::string &title);
    // Source window of a captured panel, with the frame re-read live so an
    // injected event cannot aim at where the window was up to a poll interval
    // ago. False when the handle is not a captured stream.
    bool source_window(uint64_t handle, source_window_info &out);
    // Taken by value: the completion block below captures a reference-typed
    // parameter by reference, so a temporary weak_ptr materialised at the
    // call site would be dead by the time ScreenCaptureKit replies.
    void poll_geometry(std::weak_ptr<impl> weak);
};

bool capture_manager::impl::source_window(uint64_t handle,
                                          source_window_info &out) {
    {
        std::lock_guard<std::mutex> lock(mutex);
        auto it = streams.find(handle);
        if (it == streams.end())
            return false;
        out.frame = it->second.window_frame;
        out.window_id = it->second.window_id;
        out.title = it->second.title;
    }
    CGRect live = CGRectZero;
    if (live_window_frame(out.window_id, live))
        out.frame = live;
    return true;
}

// Source stream ended underneath us (app quit, window closed, SCK error):
// drop the stream and replace the panel with a labelled test card.
void capture_manager::impl::handle_stream_ended(uint64_t handle,
                                                const std::string &title) {
    SCStream *stream = nil;
    {
        std::lock_guard<std::mutex> lock(mutex);
        auto it = streams.find(handle);
        if (it == streams.end())
            return;  // already gc'd / torn down deliberately
        it->second.output.onStopped = nil;
        stream = it->second.stream;
        streams.erase(it);
    }
    stop_stream(stream);  // no-op when SCK already stopped it; required when
                          // the geometry poll noticed the window vanish first
    std::fprintf(stderr, "capture: '%s' ended — degrading panel %llu\n",
                 title.c_str(), (unsigned long long)handle);
    with_world([&](scene &w) {
        w.close_panel(handle);
        w.spawn_panel("test-card", "capture ended: " + title);
        w.post_toast("'" + title +
                     "' closed on the Mac - its panel became a placeholder.");
    });
}

// Rate-limited source-window geometry poll, piggybacked on gc() (called once
// per rendered frame). Keeps window_frame fresh for click injection, resizes
// the stream + panel when the source window resizes, and degrades the panel
// when the window is gone but SCK never delivered didStopWithError.
void capture_manager::impl::poll_geometry(std::weak_ptr<impl> weak) {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (streams.empty() || geometry_poll_inflight)
            return;
        double now = CFAbsoluteTimeGetCurrent();
        if (now - last_geometry_poll < GEOMETRY_POLL_INTERVAL_S)
            return;
        last_geometry_poll = now;
        geometry_poll_inflight = true;
    }

    [SCShareableContent
        getShareableContentExcludingDesktopWindows:YES
                               onScreenWindowsOnly:NO
                                 completionHandler:^(
                                     SCShareableContent *content,
                                     NSError *error) {
      std::shared_ptr<impl> im = weak.lock();
      if (!im)
          return;
      struct resize_op {
          uint64_t handle;
          SCStream *stream;
          int width, height;
      };
      struct ended_op {
          uint64_t handle;
          std::string title;
      };
      std::vector<resize_op> resizes;
      std::vector<ended_op> ended;

      {
          std::lock_guard<std::mutex> lock(im->mutex);
          im->geometry_poll_inflight = false;
          if (error || !content)
              return;  // transient; retry next interval
          for (auto &kv : im->streams) {
              stream_entry &e = kv.second;
              SCWindow *found = nil;
              for (SCWindow *w in content.windows) {
                  if ((uint32_t)w.windowID == e.window_id) {
                      found = w;
                      break;
                  }
              }
              if (e.misses.note(found != nil)) {
                  ended.push_back({kv.first, e.title});
                  continue;
              }
              if (!found)
                  continue;
              CGRect f = found.frame;
              bool resized =
                  std::abs(f.size.width - e.window_frame.size.width) >= 1.0 ||
                  std::abs(f.size.height - e.window_frame.size.height) >= 1.0;
              e.window_frame = f;  // keeps click mapping fresh on plain moves
              if (resized)
                  resizes.push_back(
                      {kv.first, e.stream,
                       (int)(f.size.width * CAPTURE_SCALE),
                       (int)(f.size.height * CAPTURE_SCALE)});
          }
      }

      for (auto &op : ended)
          im->handle_stream_ended(op.handle, op.title);

      for (auto &op : resizes) {
          SCStreamConfiguration *config =
              make_stream_config((size_t)op.width, (size_t)op.height);
          [op.stream
              updateConfiguration:config
                completionHandler:^(NSError *_Nullable cfg_err) {
                  if (cfg_err)
                      std::fprintf(
                          stderr,
                          "capture: updateConfiguration failed: %s\n",
                          cfg_err.localizedDescription.UTF8String);
                }];
          im->with_world([&](scene &w) {
              w.resize_panel(op.handle, op.width, op.height);
          });
          std::fprintf(stderr, "capture: panel %llu resized to %dx%d\n",
                       (unsigned long long)op.handle, op.width, op.height);
      }
    }];
}

capture_manager::capture_manager(scene &scene_ref)
    : impl_(std::make_shared<impl>()) {
    impl_->world = &scene_ref;
}

capture_manager::~capture_manager() {
    std::unique_lock<std::shared_mutex> world_guard(impl_->world_lock);
    impl_->world = nullptr;
    std::lock_guard<std::mutex> lock(impl_->mutex);
    for (auto &kv : impl_->streams) {
        kv.second.output.onStopped = nil;  // deliberate stop, no degrade
        stop_stream_sync(kv.second.stream);
    }
    impl_->streams.clear();
}

bool capture_manager::launch_app(const std::string &target) {
    // Permission gate. Interactive runs request access once — that pops the
    // OS prompt / deep-links System Settings; the grant only applies after a
    // relaunch. Headless runs (CI) preflight only, so the TCC prompt never
    // blocks a test. Either way degrade to an internal test card now.
    if (!CGPreflightScreenCaptureAccess()) {
        static bool requested = false;
        if (!headless_mode() && !requested) {
            requested = true;
            CGRequestScreenCaptureAccess();
            std::fprintf(
                stderr,
                "capture: requested Screen Recording access — enable "
                "mac-shell under System Settings > Privacy & Security > "
                "Screen Recording, then relaunch\n");
        }
        std::fprintf(stderr,
                     "capture: Screen Recording permission not granted — "
                     "spawning internal panel for '%s'\n",
                     target.c_str());
        impl_->with_world([&](scene &w) {
            w.spawn_panel("test-card", "no-capture: " + target);
            w.post_toast(
                "Screen Recording is off - '" + target +
                    "' opened as a placeholder. Enable it in System Settings "
                    "> Privacy & Security.",
                toast_action::open_screen_recording_settings);
        });
        return false;
    }

    attempt_capture(impl_, target, LAUNCH_ATTEMPTS);
    return true;
}

void capture_manager::attempt_capture(std::weak_ptr<impl> weak,
                                      std::string want, int attempts_left) {
    [SCShareableContent
        getShareableContentExcludingDesktopWindows:YES
                               onScreenWindowsOnly:NO
                                 completionHandler:^(
                                     SCShareableContent *content,
                                     NSError *error) {
      std::shared_ptr<impl> im = weak.lock();
      if (!im)
          return;
      if (error || !content) {
          std::fprintf(stderr, "capture: shareable content failed: %s\n",
                       error ? error.localizedDescription.UTF8String
                             : "nil content");
          im->with_world([&](scene &w) {
              w.spawn_panel("test-card", "no-capture: " + want);
          });
          return;
      }

      NSString *needle =
          [[NSString stringWithUTF8String:want.c_str()] lowercaseString];
      SCWindow *match = nil;
      for (SCWindow *w in content.windows) {
          if (w.windowLayer != 0 || w.frame.size.width < MIN_WINDOW_PT ||
              w.frame.size.height < MIN_WINDOW_PT)
              continue;
          NSString *bundle =
              [w.owningApplication.bundleIdentifier lowercaseString];
          NSString *app_name =
              [w.owningApplication.applicationName lowercaseString];
          NSString *title = [w.title lowercaseString];
          bool hit = (bundle && [bundle isEqualToString:needle]) ||
                     (app_name && [app_name containsString:needle]) ||
                     (title && title.length > 0 &&
                      [title containsString:needle]);
          if (hit && (!match || window_rank(w) > window_rank(match)))
              match = w;
      }
      if (!match) {
          bool retry = !headless_mode() && attempts_left > 0 &&
                       (attempts_left < LAUNCH_ATTEMPTS || open_app(want));
          if (retry) {
              dispatch_after(
                  dispatch_time(DISPATCH_TIME_NOW, LAUNCH_RETRY_NS),
                  dispatch_get_global_queue(QOS_CLASS_UTILITY, 0), ^{
                    attempt_capture(weak, want, attempts_left - 1);
                  });
              return;
          }
          std::fprintf(stderr, "capture: no window matching '%s'\n",
                       want.c_str());
          im->with_world([&](scene &w) {
              w.spawn_panel("test-card", "no-window: " + want);
          });
          return;
      }

      // Already on a panel: bring that panel back rather than stream the
      // same window twice (every launcher pick used to add a duplicate).
      {
          uint64_t existing = 0;
          {
              std::lock_guard<std::mutex> lock(im->mutex);
              for (const auto &kv : im->streams)
                  if (kv.second.window_id == (uint32_t)match.windowID) {
                      existing = kv.first;
                      break;
                  }
          }
          if (existing) {
              std::fprintf(stderr, "capture: '%s' is already panel %llu, recalling\n",
                           want.c_str(), (unsigned long long)existing);
              im->with_world([&](scene &w) { w.recall_panel(existing); });
              return;
          }
      }

      // Desktop-independent window filter: captures ONLY the chosen window,
      // never the whole display.
      SCContentFilter *filter = [[SCContentFilter alloc]
          initWithDesktopIndependentWindow:match];
      SCStreamConfiguration *config = make_stream_config(
          (size_t)(match.frame.size.width * CAPTURE_SCALE),
          (size_t)(match.frame.size.height * CAPTURE_SCALE));

      CaptureStreamOutput *output = [[CaptureStreamOutput alloc] init];
      SCStream *stream = [[SCStream alloc] initWithFilter:filter
                                            configuration:config
                                                 delegate:output];
      NSError *add_err = nil;
      if (![stream addStreamOutput:output
                              type:SCStreamOutputTypeScreen
                sampleHandlerQueue:im->sample_queue
                             error:&add_err]) {
          std::fprintf(stderr, "capture: addStreamOutput failed: %s\n",
                       add_err.localizedDescription.UTF8String);
          im->with_world([&](scene &w) {
              w.spawn_panel("test-card", "no-capture: " + want);
          });
          return;
      }

      std::string app_id =
          match.owningApplication.bundleIdentifier
              ? match.owningApplication.bundleIdentifier.UTF8String
              : want;
      std::string title =
          match.title.length ? match.title.UTF8String : app_id;
      int32_t pid = (int32_t)match.owningApplication.processID;
      uint64_t handle = 0;
      im->with_world([&](scene &w) {
          handle = w.spawn_captured_panel(app_id, title, pid,
                                          (int)config.width,
                                          (int)config.height);
      });
      if (!handle)
          return;  // manager tearing down; the stream never started

      {
          std::lock_guard<std::mutex> lock(im->mutex);
          im->streams[handle] = {stream,
                                 output,
                                 match.frame,
                                 pid,
                                 (uint32_t)match.windowID,
                                 title,
                                 {}};
      }
      output.onStopped = ^(NSString *) {
        if (std::shared_ptr<impl> self = weak.lock())
            self->handle_stream_ended(handle, title);
      };

      [stream startCaptureWithCompletionHandler:^(NSError *start_err) {
        std::shared_ptr<impl> self = weak.lock();
        if (!self)
            return;
        if (start_err) {
            std::fprintf(stderr, "capture: startCapture failed: %s\n",
                         start_err.localizedDescription.UTF8String);
            self->handle_stream_ended(handle, title);
        } else {
            std::fprintf(stderr,
                         "capture: streaming '%s' (pid %d) as panel %llu\n",
                         title.c_str(), pid, (unsigned long long)handle);
        }
      }];
    }];
}

void *capture_manager::copy_latest_pixel_buffer(uint64_t handle) {
    CaptureStreamOutput *output = nil;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto it = impl_->streams.find(handle);
        if (it == impl_->streams.end())
            return nullptr;
        output = it->second.output;
    }
    return output ? (void *)[output copyLatest] : nullptr;
}

bool capture_manager::capture_panel_png(uint64_t handle,
                                        const std::string &path,
                                        std::string &err) {
    if (CVPixelBufferRef pb =
            (CVPixelBufferRef)copy_latest_pixel_buffer(handle)) {
        bool ok = false;
        if (CVPixelBufferLockBaseAddress(pb, kCVPixelBufferLock_ReadOnly) ==
            kCVReturnSuccess) {
            size_t w = CVPixelBufferGetWidth(pb);
            size_t h = CVPixelBufferGetHeight(pb);
            size_t stride = CVPixelBufferGetBytesPerRow(pb);
            const uint8_t *src =
                (const uint8_t *)CVPixelBufferGetBaseAddress(pb);
            if (src && w > 0 && h > 0) {
                // SCK hands us 32BGRA; write_rgba_png wants RGBA, and the
                // repacked copy also drops any row padding.
                std::vector<uint8_t> rgba(w * h * 4);
                for (size_t y = 0; y < h; y++) {
                    const uint8_t *sp = src + y * stride;
                    uint8_t *dp = rgba.data() + y * w * 4;
                    for (size_t x = 0; x < w; x++, sp += 4, dp += 4) {
                        dp[0] = sp[2];
                        dp[1] = sp[1];
                        dp[2] = sp[0];
                        dp[3] = sp[3];
                    }
                }
                ok = write_rgba_png(rgba.data(), (int)w, (int)h, w * 4, path,
                                    err);
            } else {
                err = "no_frame";
            }
            CVPixelBufferUnlockBaseAddress(pb, kCVPixelBufferLock_ReadOnly);
        } else {
            err = "lock_failed";
        }
        CVPixelBufferRelease(pb);
        return ok;
    }

    bool ok = false;
    err = "no_such_window";
    impl_->with_world([&](scene &w) {
        for (auto &rp : w.snapshot_render_panels()) {
            if (rp.handle != handle)
                continue;
            if (rp.rgba.empty() || rp.width_px <= 0 || rp.height_px <= 0) {
                err = "no_frame";
                return;
            }
            ok = write_rgba_png(rp.rgba.data(), rp.width_px, rp.height_px,
                                (size_t)rp.width_px * 4, path, err);
            return;
        }
    });
    return ok;
}

void capture_manager::gc(const std::vector<uint64_t> &live_handles) {
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        for (auto it = impl_->streams.begin(); it != impl_->streams.end();) {
            bool alive = false;
            for (uint64_t h : live_handles)
                if (h == it->first)
                    alive = true;
            if (alive) {
                ++it;
                continue;
            }
            it->second.output.onStopped = nil;  // deliberate stop
            stop_stream(it->second.stream);
            it = impl_->streams.erase(it);
        }
    }
    impl_->poll_geometry(impl_);
}

void capture_manager::install_input_injector() {
    std::weak_ptr<impl> weak = impl_;
    impl_->world->set_input_injector([weak](uint64_t handle, int32_t pid,
                                            const inject_event &ev) {
        std::shared_ptr<impl> im = weak.lock();
        if (!im)
            return;
        if (!inject_enabled()) {
            static bool warned = false;
            if (!warned) {
                std::fprintf(stderr,
                             "capture: input injection disabled "
                             "(SPATULA_MAC_INJECT=0 or headless)\n");
                warned = true;
            }
            return;
        }
        // CGEvent posts need Accessibility trust. Prompt once (interactive
        // runs only — the prompt deep-links System Settings), then drop
        // events until granted. The injector runs on the control-socket
        // thread; the prompting call must run on the main queue.
        if (!AXIsProcessTrusted()) {
            static bool prompted = false;
            if (!prompted) {
                prompted = true;
                if (!headless_mode()) {
                    dispatch_async(dispatch_get_main_queue(), ^{
                      NSDictionary *opts = @{
                          (__bridge NSString *)kAXTrustedCheckOptionPrompt :
                              @YES
                      };
                      AXIsProcessTrustedWithOptions(
                          (__bridge CFDictionaryRef)opts);
                    });
                }
                std::fprintf(
                    stderr,
                    "capture: Accessibility permission not granted — enable "
                    "mac-shell under System Settings > Privacy & Security > "
                    "Accessibility to inject input\n");
            }
            return;
        }
        switch (ev.type) {
            case inject_event::kind::text: {
                NSString *text =
                    [NSString stringWithUTF8String:ev.text.c_str()];
                // One key event per composed character so surrogate pairs and
                // combining sequences reach the app intact.
                [text enumerateSubstringsInRange:NSMakeRange(0, text.length)
                                         options:
                                             NSStringEnumerationByComposedCharacterSequences
                                      usingBlock:^(NSString *sub, NSRange,
                                                   NSRange, BOOL *) {
                                        unichar buf[8];
                                        NSUInteger n = sub.length;
                                        if (n > 8)
                                            n = 8;
                                        [sub getCharacters:buf
                                                     range:NSMakeRange(0, n)];
                                        CGEventRef down =
                                            CGEventCreateKeyboardEvent(nil, 0,
                                                                       true);
                                        CGEventRef up =
                                            CGEventCreateKeyboardEvent(nil, 0,
                                                                       false);
                                        CGEventKeyboardSetUnicodeString(
                                            down, n, buf);
                                        CGEventKeyboardSetUnicodeString(up, n,
                                                                        buf);
                                        post_updown(pid, down, up);
                                      }];
                break;
            }
            case inject_event::kind::key: {
                uint16_t code = 0;
                if (!keycode_for_name(ev.text, code)) {
                    std::fprintf(stderr,
                                 "capture: dropping unknown key name '%s' "
                                 "(not a known name or a keycode in 0-%lu)\n",
                                 ev.text.c_str(), MAX_VIRTUAL_KEYCODE);
                    return;
                }
                CGEventRef down = CGEventCreateKeyboardEvent(nil, code, true);
                CGEventRef up = CGEventCreateKeyboardEvent(nil, code, false);
                post_updown(pid, down, up);
                break;
            }
            case inject_event::kind::focus: {
                source_window_info src;
                if (!im->source_window(handle, src))
                    return;
                // Fire-and-forget off-thread: the scene calls the injector on
                // its own thread holding its mutex, and an AX round-trip
                // blocks for as long as the target app takes to answer.
                CGRect frame = src.frame;
                std::string title = src.title;
                dispatch_async(
                    dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^{
                      if (ax_focus_window(pid, frame, title))
                          return;
                      // AX refused (app not scriptable, or no window matched).
                      // Activating is the only way left to give the app a key
                      // window, at the cost of the user's focus.
                      std::fprintf(stderr,
                                   "capture: AX focus failed for panel %llu — "
                                   "activating pid %d instead\n",
                                   (unsigned long long)handle, pid);
                      dispatch_async(dispatch_get_main_queue(), ^{
                        if (NSRunningApplication *app = [NSRunningApplication
                                runningApplicationWithProcessIdentifier:(
                                    pid_t)pid])
                            [app activateWithOptions:0];
                      });
                    });
                break;
            }
            case inject_event::kind::click: {
                source_window_info src;
                if (!im->source_window(handle, src))
                    return;
                CGPoint pt = cg_window_point(src.frame, ev.x, ev.y);
                CGMouseButton btn = ev.button == 1 ? kCGMouseButtonRight
                                                   : kCGMouseButtonCenter;
                CGEventType down_t = kCGEventLeftMouseDown,
                            up_t = kCGEventLeftMouseUp;
                if (ev.button == 1) {
                    down_t = kCGEventRightMouseDown;
                    up_t = kCGEventRightMouseUp;
                } else if (ev.button == 2) {
                    down_t = kCGEventOtherMouseDown;
                    up_t = kCGEventOtherMouseUp;
                }
                if (ev.button == 0)
                    btn = kCGMouseButtonLeft;
                CGEventRef down = CGEventCreateMouseEvent(nil, down_t, pt, btn);
                CGEventRef up = CGEventCreateMouseEvent(nil, up_t, pt, btn);
                post_updown(pid, down, up);
                break;
            }
            case inject_event::kind::scroll: {
                source_window_info src;
                if (!im->source_window(handle, src))
                    return;
                // Scroll carries deltas only, so aim at the window's centre;
                // without an explicit location the event lands under the
                // physical cursor, i.e. in some other app.
                CGPoint pt = cg_window_point(
                    src.frame,
                    (float)(src.frame.size.width * CAPTURE_SCALE / 2),
                    (float)(src.frame.size.height * CAPTURE_SCALE / 2));
                CGEventRef ev_scroll = CGEventCreateScrollWheelEvent(
                    nil, kCGScrollEventUnitLine, 2, (int32_t)ev.y,
                    (int32_t)ev.x);
                CGEventSetLocation(ev_scroll, pt);
                CGEventPostToPid(pid, ev_scroll);
                CFRelease(ev_scroll);
                break;
            }
        }
    });
}

}  // namespace mac_shell
