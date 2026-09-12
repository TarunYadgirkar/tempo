// app_main.mm — window / app bootstrap: the MTKView input subclass, window
// delegate, menu bar, and run_renderer_app (the AppKit event loop).

#include "platform/renderer_internal.h"

// Stop the AppKit run loop. -[NSApplication stop:] only takes effect once
// the loop next wakes, so post a no-op event to wake it immediately — the
// draw callback is not an event, and a quiet window would otherwise sit
// until the user moves the mouse.
void stop_app_loop() {
    [NSApp stop:nil];
    [NSApp postEvent:[NSEvent otherEventWithType:NSEventTypeApplicationDefined
                                        location:NSZeroPoint
                                   modifierFlags:0
                                       timestamp:0
                                    windowNumber:0
                                         context:nil
                                         subtype:0
                                           data1:0
                                           data2:0]
             atStart:YES];
}

// ------------------------------------------------------------------
// ShellView: MTKView + fly-cam input
// ------------------------------------------------------------------

@interface ShellView : MTKView
@property(nonatomic, assign) renderer_state *state;
@end

@implementation ShellView
- (BOOL)acceptsFirstResponder {
    return YES;
}
- (void)keyDown:(NSEvent *)event {
    renderer_state *s = self.state;
    if (s && event.keyCode == 53 && s->cheat_visible) {  // Esc
        s->cheat_visible = false;
        return;
    }
    if (!event.isARepeat)
        self.state->keys_down.insert(event.keyCode);
}
- (void)keyUp:(NSEvent *)event {
    self.state->keys_down.erase(event.keyCode);
}
- (void)mouseDragged:(NSEvent *)event {
    self.state->drag_dx += (float)event.deltaX;
    self.state->drag_dy += (float)event.deltaY;
}
- (void)mouseDown:(NSEvent *)event {
    renderer_state *s = self.state;
    if (!s || !s->world)
        return;
    NSPoint p = [self convertPoint:event.locationInWindow fromView:nil];
    CGFloat w = self.bounds.size.width, h = self.bounds.size.height;
    if (w <= 0 || h <= 0)
        return;
    float fx = (float)(p.x / w);
    float fy = (float)(p.y / h);  // 0 = bottom

    // First-launch onboarding is modal: only "Get started" does anything.
    if (s->onboarding_visible) {
        if (s->onboard_button.contains(fx, fy)) {
            s->onboarding_visible = false;
            if (s->persist_first_run)
                [[NSUserDefaults standardUserDefaults]
                    setBool:YES
                     forKey:@"spatula.hasSeenOnboarding"];
            if (s->autoshow_cheat_after_onboarding)
                s->cheat_visible = true;
        }
        return;
    }

    // Any click closes the cheat sheet.
    if (s->cheat_visible) {
        s->cheat_visible = false;
        return;
    }

    // Toast: its action button deep-links System Settings; clicking the
    // toast anywhere else just dismisses it.
    if (s->toast_rect.contains(fx, fy)) {
        if (s->toast_button.contains(fx, fy)) {
            NSURL *url = [NSURL
                URLWithString:@"x-apple.systempreferences:com.apple."
                              @"preference.security?Privacy_ScreenCapture"];
            if (url)
                [[NSWorkspace sharedWorkspace] openURL:url];
        }
        s->world->dismiss_toast();
        return;
    }

    // Dock strip: "?" pill toggles the cheat sheet, pills focus panels.
    if (s->dock_h_frac <= 0 || fy > s->dock_h_frac)
        return;
    if (s->dock_help_x1 > 0 && fx >= s->dock_help_x0 &&
        fx <= s->dock_help_x1) {
        s->cheat_visible = !s->cheat_visible;
        return;
    }
    for (const auto &e : s->dock_hits) {
        if (fx >= e.x0 && fx <= e.x1) {
            s->world->focus_panel(e.handle);
            return;
        }
    }
}
@end

// ------------------------------------------------------------------
// window / app bootstrap
// ------------------------------------------------------------------

@interface ShellWindowDelegate : NSObject <NSWindowDelegate>
@property(nonatomic, assign) std::atomic<bool> *running;
@end

@implementation ShellWindowDelegate
- (void)windowWillClose:(NSNotification *)notification {
    (void)notification;
    if (self.running)
        self.running->store(false);
    stop_app_loop();
}
@end

// Menu-bar targets: About (version + commit), clean Quit through the same
// shutdown path as closing the window, Help → gesture cheat sheet.
@interface SpatulaMenuActions : NSObject
@property(nonatomic, assign) renderer_state *state;
@end

@implementation SpatulaMenuActions
- (void)showAbout:(id)sender {
    (void)sender;
    NSString *ver = [[NSBundle mainBundle]
        objectForInfoDictionaryKey:@"CFBundleShortVersionString"];
    if (!ver.length)
        ver = @"dev";
#ifndef MAC_SHELL_COMMIT
#define MAC_SHELL_COMMIT "unknown"
#endif
    NSAlert *alert = [[NSAlert alloc] init];
    alert.messageText = @"Spatula";
    alert.informativeText = [NSString
        stringWithFormat:@"Version %@ - commit %s\n\nThe Mac side of the "
                         @"Spatula spatial shell. Your iPhone (SpatialBridge) "
                         @"is the eyes; this window is the room.",
                         ver, MAC_SHELL_COMMIT];
    [alert addButtonWithTitle:@"OK"];
    [alert runModal];
}
- (void)quitShell:(id)sender {
    (void)sender;
    if (self.state && self.state->running)
        self.state->running->store(false);
    stop_app_loop();
}
- (void)toggleCheatSheet:(id)sender {
    (void)sender;
    if (self.state)
        self.state->cheat_visible = !self.state->cheat_visible;
}
@end

namespace mac_shell {

int run_renderer_app(scene &scene_ref, sb_receiver_t *receiver,
                     capture_manager *capture, std::atomic<bool> &running,
                     const renderer_boot_info &boot) {
    @autoreleasepool {
        static renderer_state state;
        state.world = &scene_ref;
        state.receiver = receiver;
        state.capture = capture;
        state.running = &running;
        state.udp_port = boot.udp_port;
        state.replay = boot.replay;
        state.startup_error_title = boot.startup_error_title;
        state.startup_error = boot.startup_error;
        state.retry = boot.retry;
        state.grabber = boot.grabber;
        if (const char *p = getenv("SPATULA_MAC_SCREENSHOT"))
            state.screenshot_path = p;
        if (const char *p = getenv("SPATULA_MAC_SCREENSHOT_AT"))
            state.screenshot_at = atof(p);
        if (const char *p = getenv("SPATULA_MAC_EXIT_AFTER"))
            state.exit_after = atof(p);

        // First-run aids (onboarding card, cheat-sheet auto-show), persisted
        // via NSUserDefaults. Verification runs (screenshot / timed exit)
        // suppress them and never touch persistence unless
        // SPATULA_MAC_FIRST_RUN forces a state: 1 = act like a fresh
        // install, 0 = act like a veteran.
        bool verification =
            !state.screenshot_path.empty() || state.exit_after > 0;
        first_run_prefs prefs;
        prefs.seen_onboarding = true;
        prefs.session_count = CHEATSHEET_AUTO_SESSIONS + 1;
        const char *fr = getenv("SPATULA_MAC_FIRST_RUN");
        if (fr && std::strcmp(fr, "1") == 0) {
            prefs = first_run_prefs{};
        } else if (fr && std::strcmp(fr, "cheat") == 0) {
            // Onboarding done, cheat sheet still auto-showing (screenshots).
            prefs.seen_onboarding = true;
            prefs.session_count = 1;
        } else if ((!fr || std::strcmp(fr, "0") != 0) && !verification) {
            NSUserDefaults *d = [NSUserDefaults standardUserDefaults];
            prefs.seen_onboarding =
                [d boolForKey:@"spatula.hasSeenOnboarding"];
            NSInteger n = [d integerForKey:@"spatula.sessionCount"] + 1;
            [d setInteger:n forKey:@"spatula.sessionCount"];
            prefs.session_count = (int)n;
            state.persist_first_run = true;
        }
        state.onboarding_visible = should_show_onboarding(prefs);
        state.autoshow_cheat_after_onboarding =
            should_autoshow_cheatsheet(prefs);
        state.cheat_visible =
            !state.onboarding_visible && should_autoshow_cheatsheet(prefs);

        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];

        // Menu bar: About / Quit (Cmd+Q) and Help → cheat sheet (Cmd+/).
        SpatulaMenuActions *menu_actions = [[SpatulaMenuActions alloc] init];
        menu_actions.state = &state;
        NSMenu *menubar = [[NSMenu alloc] init];
        NSMenuItem *app_item = [[NSMenuItem alloc] init];
        NSMenu *app_menu = [[NSMenu alloc] init];
        NSMenuItem *about_item =
            [[NSMenuItem alloc] initWithTitle:@"About Spatula"
                                       action:@selector(showAbout:)
                                keyEquivalent:@""];
        about_item.target = menu_actions;
        [app_menu addItem:about_item];
        [app_menu addItem:[NSMenuItem separatorItem]];
        NSMenuItem *quit_item =
            [[NSMenuItem alloc] initWithTitle:@"Quit Spatula"
                                       action:@selector(quitShell:)
                                keyEquivalent:@"q"];
        quit_item.target = menu_actions;
        [app_menu addItem:quit_item];
        app_item.submenu = app_menu;
        [menubar addItem:app_item];
        NSMenuItem *help_item = [[NSMenuItem alloc] init];
        NSMenu *help_menu = [[NSMenu alloc] initWithTitle:@"Help"];
        NSMenuItem *cheat_item =
            [[NSMenuItem alloc] initWithTitle:@"Gesture Cheat Sheet"
                                       action:@selector(toggleCheatSheet:)
                                keyEquivalent:@"/"];
        cheat_item.target = menu_actions;
        [help_menu addItem:cheat_item];
        help_item.submenu = help_menu;
        [menubar addItem:help_item];
        [NSApp setMainMenu:menubar];
        NSApp.helpMenu = help_menu;

        NSRect frame = NSMakeRect(120, 120, 1280, 800);
        NSWindow *window = [[NSWindow alloc]
            initWithContentRect:frame
                      styleMask:NSWindowStyleMaskTitled |
                                NSWindowStyleMaskClosable |
                                NSWindowStyleMaskResizable
                        backing:NSBackingStoreBuffered
                          defer:NO];
        window.title = @"Spatula mac-shell";

        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (!device) {
            std::fprintf(stderr, "renderer: no Metal device\n");
            return 1;
        }
        ShellView *view = [[ShellView alloc] initWithFrame:frame
                                                    device:device];
        view.state = &state;
        view.colorPixelFormat = MTLPixelFormatBGRA8Unorm;
        view.depthStencilPixelFormat = MTLPixelFormatDepth32Float;
        // Vantage void (#140d0a).
        view.clearColor = MTLClearColorMake(0.078, 0.051, 0.039, 1.0);
        view.clearDepth = 1.0;
        view.preferredFramesPerSecond = 60;
        if (!state.screenshot_path.empty())
            view.framebufferOnly = NO;

        ShellRenderer *renderer = [[ShellRenderer alloc] initWithView:view
                                                                state:&state];
        if (!renderer)
            return 1;
        view.delegate = renderer;

        ShellWindowDelegate *wd = [[ShellWindowDelegate alloc] init];
        wd.running = &running;
        window.delegate = wd;

        window.contentView = view;
        [window makeKeyAndOrderFront:nil];
        [window makeFirstResponder:view];
        [NSApp activateIgnoringOtherApps:YES];
        [NSApp run];
    }
    return 0;
}

}  // namespace mac_shell
