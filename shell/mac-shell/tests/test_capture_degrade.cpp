// test_capture_degrade.cpp — capture permission degrade e2e.
//
// Starts a headless mac-shell (no replay needed) and asserts the permission
// surface over the control socket: `permissions` reports scriptable 0/1
// flags, `stats` replies, and `launch-app` for a window that cannot be
// captured (no Screen Recording grant in CI, or no such window when a grant
// exists) degrades to an internal test-card fallback panel instead of
// failing. Also pins the header-only injection helpers that need no SCK: the
// geometry-poll debounce, the click coordinate mapping, and key-name parsing.
// Headless-safe: nothing here ever triggers a TCC prompt, and nothing here
// posts a CGEvent.

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include "platform/capture.h"

extern char **environ;

static int g_failures = 0;

#define CHECK_MSG(cond, msg)                                                  \
    do {                                                                      \
        if (!(cond)) {                                                        \
            std::fprintf(stderr, "FAIL %s:%d: %s (%s)\n", __FILE__, __LINE__, \
                         #cond, (msg));                                       \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

struct ctl_client {
    int fd = -1;
    std::string pending;

    bool connect_path(const std::string &path, int timeout_ms) {
        struct sockaddr_un addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
        int waited = 0;
        while (waited <= timeout_ms) {
            fd = socket(AF_UNIX, SOCK_STREAM, 0);
            if (fd >= 0 &&
                connect(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0)
                return true;
            if (fd >= 0)
                close(fd);
            fd = -1;
            usleep(100 * 1000);
            waited += 100;
        }
        return false;
    }

    bool recv_line(std::string &out, int timeout_ms = 5000) {
        while (true) {
            size_t nl = pending.find('\n');
            if (nl != std::string::npos) {
                out = pending.substr(0, nl);
                pending.erase(0, nl + 1);
                return true;
            }
            struct pollfd pfd = {fd, POLLIN, 0};
            int rc = poll(&pfd, 1, timeout_ms);
            if (rc <= 0)
                return false;
            char buf[4096];
            ssize_t r = read(fd, buf, sizeof(buf));
            if (r <= 0)
                return false;
            pending.append(buf, (size_t)r);
        }
    }

    std::string request(const std::string &line) {
        std::string out = line + "\n";
        if (write(fd, out.data(), out.size()) != (ssize_t)out.size())
            return "<send-failed>";
        std::string reply;
        if (!recv_line(reply))
            return "<recv-timeout>";
        return reply;
    }

    void close_fd() {
        if (fd >= 0)
            close(fd);
        fd = -1;
    }
};

static bool contains(const std::string &s, const std::string &sub) {
    return s.find(sub) != std::string::npos;
}

static void test_geometry_miss_debounce() {
    mac_shell::geometry_miss_tracker t;
    CHECK_MSG(!t.note(true), "present window must not end");
    CHECK_MSG(!t.note(false), "a single miss must not end the stream");
    CHECK_MSG(!t.note(true), "reappearing window resets the miss count");
    CHECK_MSG(!t.note(false), "first miss after reset");
    CHECK_MSG(t.note(false), "second consecutive miss ends the stream");
    CHECK_MSG(t.note(false), "stays ended while still missing");
}

// window_point is the click-injection mapping: surface-local capture px →
// global screen point. SCWindow.frame (what the geometry poll caches) and
// kCGWindowBounds (the live re-read) are the same top-left-origin rect for a
// given windowID, and CGEventCreateMouseEvent takes that same space — so the
// mapping is a pure translate with no Y flip at any origin.
static void test_window_point() {
    using mac_shell::window_point;
    mac_shell::screen_point p = window_point(120.0, 33.0, 0.0, 0.0);
    CHECK_MSG(p.x == 120.0 && p.y == 33.0, "origin maps to the frame origin");

    // 2560x1664 capture surface over a 1280x832 window (CAPTURE_SCALE = 2).
    p = window_point(120.0, 33.0, 2560.0, 1664.0);
    CHECK_MSG(p.x == 1400.0 && p.y == 865.0, "bottom-right corner");
    p = window_point(120.0, 33.0, 1280.0, 832.0);
    CHECK_MSG(p.x == 760.0 && p.y == 449.0, "centre");

    // A display placed above/left of the main one gives the window frame a
    // negative origin in the same global space; the translate carries it.
    p = window_point(-1512.0, -400.0, 200.0, 100.0);
    CHECK_MSG(p.x == -1412.0 && p.y == -350.0, "negative multi-display origin");

    // Y grows downward on both sides — a larger surface y must map to a
    // larger screen y, which is what a mistaken bottom-left assumption breaks.
    CHECK_MSG(window_point(0, 0, 0, 200).y > window_point(0, 0, 0, 0).y,
              "surface y grows downward in screen space");
}

static void test_keycode_for_name() {
    using mac_shell::keycode_for_name;
    uint16_t code = 0xffff;
    CHECK_MSG(keycode_for_name("Return", code) && code == 36, "named key");
    CHECK_MSG(keycode_for_name("Up", code) && code == 126, "named arrow");
    CHECK_MSG(keycode_for_name("36", code) && code == 36, "decimal keycode");
    CHECK_MSG(keycode_for_name("0", code) && code == 0, "keycode zero");

    // Everything below used to fall through strtoul and post a keycode.
    CHECK_MSG(!keycode_for_name("", code), "empty name must not post 'a'");
    CHECK_MSG(!keycode_for_name("a", code), "unknown name");
    CHECK_MSG(!keycode_for_name("-1", code), "negative wraps, must be dropped");
    CHECK_MSG(!keycode_for_name("  5", code), "leading space");
    CHECK_MSG(!keycode_for_name("+3", code), "signed literal");
    CHECK_MSG(!keycode_for_name("0x41", code), "hex is not a keycode");
    CHECK_MSG(!keycode_for_name("5x", code), "trailing garbage");
    CHECK_MSG(!keycode_for_name("99999", code), "out of keycode range");
    CHECK_MSG(!keycode_for_name("return", code), "names are case-sensitive");
}

int main() {
    test_geometry_miss_debounce();
    test_window_point();
    test_keycode_for_name();

    std::string dir = "/tmp";
    if (const char *t = getenv("TMPDIR"); t && t[0])
        dir = t;
    if (dir.back() == '/')
        dir.pop_back();
    std::string sock = dir + "/mac-shell-degrade-" +
                       std::to_string((long)getpid()) + ".sock";

    std::vector<std::string> env_strs;
    for (char **e = environ; *e; e++) {
        if (std::strncmp(*e, "SPATIAL_OS_SOCK=", 16) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_HEADLESS=", 21) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_NO_BONJOUR=", 23) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_INJECT=", 19) == 0)
            continue;
        env_strs.push_back(*e);
    }
    env_strs.push_back("SPATIAL_OS_SOCK=" + sock);
    env_strs.push_back("SPATULA_MAC_HEADLESS=1");
    env_strs.push_back("SPATULA_MAC_NO_BONJOUR=1");
    std::vector<char *> envp;
    for (auto &s : env_strs)
        envp.push_back(const_cast<char *>(s.c_str()));
    envp.push_back(nullptr);

    // Live-UDP mode on an unprivileged high port; no session file needed.
    const char *bin = MAC_SHELL_BIN;
    std::vector<char *> argv = {const_cast<char *>(bin),
                                const_cast<char *>("--port"),
                                const_cast<char *>("29897"), nullptr};

    pid_t child = -1;
    int rc = posix_spawn(&child, bin, nullptr, nullptr, argv.data(),
                         envp.data());
    if (rc != 0) {
        std::fprintf(stderr, "posix_spawn(%s) failed: %s\n", bin,
                     std::strerror(rc));
        return 1;
    }

    ctl_client c;
    CHECK_MSG(c.connect_path(sock, 10000), "control socket never came up");

    // ---- permissions: scriptable 0/1 flags ----
    std::string perms = c.request("permissions");
    int sc = -1, ax = -1;
    CHECK_MSG(std::sscanf(perms.c_str(),
                          "ok screen_capture=%d accessibility=%d", &sc,
                          &ax) == 2,
              perms.c_str());
    CHECK_MSG(sc == 0 || sc == 1, perms.c_str());
    CHECK_MSG(ax == 0 || ax == 1, perms.c_str());

    // ---- stats: packet rate + panel count ----
    std::string st = c.request("stats");
    float rate = -1.0f;
    unsigned long panels = 999;
    CHECK_MSG(std::sscanf(st.c_str(), "ok packet_rate=%f panels=%lu", &rate,
                          &panels) == 2,
              st.c_str());
    CHECK_MSG(panels == 0, st.c_str());

    // ---- launch-app degrade: no permission (or no such window) must fall
    // back to an internal test-card panel, never an error or a dead panel.
    std::string la = c.request("launch-app zz-degrade-no-such-window-zz");
    CHECK_MSG(la == "ok", la.c_str());
    const char *want_label =
        sc == 0 ? "no-capture: zz-degrade-no-such-window-zz"
                : "no-window: zz-degrade-no-such-window-zz";
    std::string lw;
    bool degraded = false;
    for (int i = 0; i < 50 && !degraded; i++) {  // window lookup is async
        lw = c.request("list-windows");
        degraded = contains(lw, want_label);
        if (!degraded)
            usleep(100 * 1000);
    }
    CHECK_MSG(degraded, lw.c_str());
    CHECK_MSG(contains(lw, "\"app_id\":\"test-card\""), lw.c_str());

    st = c.request("stats");
    CHECK_MSG(contains(st, "panels=1"), st.c_str());

    c.close_fd();
    kill(child, SIGTERM);
    int status = 0;
    waitpid(child, &status, 0);
    CHECK_MSG(WIFEXITED(status) && WEXITSTATUS(status) == 0,
              "mac-shell did not exit cleanly");

    if (g_failures) {
        std::fprintf(stderr, "%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("test_capture_degrade: all checks passed\n");
    return 0;
}
