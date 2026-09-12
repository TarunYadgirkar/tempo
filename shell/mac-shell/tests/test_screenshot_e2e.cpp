// test_screenshot_e2e.cpp — the `screenshot` control verb against a real
// window.
//
// Every other mac-shell harness is headless, but the whole-scene form of the
// verb grabs the Metal drawable, so this one spawns a WINDOWED instance
// instead. No display attached (ssh, a CI box) → exit 77, which the ctest
// entry maps to SKIP rather than a failure.
//
// Live mode on a private UDP port with nothing sent to it, NOT --replay: a
// recorded session's hand packets drive the radial launcher, which captures
// real mac windows as panels — so handles would not be deterministic and the
// test would open other people's apps.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include <CoreGraphics/CoreGraphics.h>

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

namespace {

constexpr int SKIP_EXIT = 77;

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

    bool send_line(const std::string &line) {
        std::string out = line + "\n";
        return write(fd, out.data(), out.size()) == (ssize_t)out.size();
    }

    bool recv_line(std::string &out, int timeout_ms = 8000) {
        while (true) {
            size_t nl = pending.find('\n');
            if (nl != std::string::npos) {
                out = pending.substr(0, nl);
                pending.erase(0, nl + 1);
                return true;
            }
            struct pollfd pfd = {fd, POLLIN, 0};
            if (poll(&pfd, 1, timeout_ms) <= 0)
                return false;
            char buf[4096];
            ssize_t r = read(fd, buf, sizeof(buf));
            if (r <= 0)
                return false;
            pending.append(buf, (size_t)r);
        }
    }

    std::string request(const std::string &line) {
        if (!send_line(line))
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

bool has_display() {
    uint32_t count = 0;
    return CGGetActiveDisplayList(0, nullptr, &count) == kCGErrorSuccess &&
           count > 0;
}

std::string path_of(const std::string &reply) {
    const std::string key = "ok path=";
    if (reply.rfind(key, 0) != 0)
        return {};
    return reply.substr(key.size());
}

// PNG magic + the IHDR dimensions, which live at a fixed offset in every PNG.
bool png_dimensions(const std::string &path, uint32_t &w, uint32_t &h) {
    std::FILE *f = std::fopen(path.c_str(), "rb");
    if (!f)
        return false;
    uint8_t head[24];
    bool ok = std::fread(head, 1, sizeof(head), f) == sizeof(head);
    std::fclose(f);
    static const uint8_t MAGIC[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a,
                                     '\n'};
    if (!ok || std::memcmp(head, MAGIC, sizeof(MAGIC)) != 0 ||
        std::memcmp(head + 12, "IHDR", 4) != 0)
        return false;
    w = ((uint32_t)head[16] << 24) | ((uint32_t)head[17] << 16) |
        ((uint32_t)head[18] << 8) | head[19];
    h = ((uint32_t)head[20] << 24) | ((uint32_t)head[21] << 16) |
        ((uint32_t)head[22] << 8) | head[23];
    return true;
}

// A composited frame is at least a few hundred pixels on the short side; a
// 1x1 or an empty texture would still be "a PNG".
constexpr uint32_t MIN_PLAUSIBLE_PX = 128;

}  // namespace

int main() {
    if (!has_display()) {
        std::printf("test_screenshot_e2e: no active display — skipping\n");
        return SKIP_EXIT;
    }

    std::string dir = "/tmp";
    if (const char *t = getenv("TMPDIR"); t && t[0])
        dir = t;
    if (dir.back() == '/')
        dir.pop_back();
    std::string tag = std::to_string((long)getpid());
    std::string sock = dir + "/mac-shell-shot-e2e-" + tag + ".sock";
    std::string cfg = dir + "/mac-shell-shot-e2e-cfg-" + tag;
    std::string custom_leaf = "/mac-shell-shot-e2e-custom-" + tag + ".png";
    std::string custom = dir + custom_leaf;
    std::string link = dir + "/mac-shell-shot-e2e-link-" + tag + ".png";
    int port = 21000 + (int)(getpid() % 2000);

    if (system(("mkdir -p '" + cfg + "'").c_str()) != 0) {
        std::fprintf(stderr, "cannot create %s\n", cfg.c_str());
        return 1;
    }

    std::vector<std::string> env_strs;
    for (char **e = environ; *e; e++) {
        if (std::strncmp(*e, "SPATIAL_OS_SOCK=", 16) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_HEADLESS=", 21) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_NO_BONJOUR=", 23) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_EXIT_AFTER=", 23) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_SCREENSHOT", 22) == 0 ||
            std::strncmp(*e, "XDG_CONFIG_HOME=", 16) == 0)
            continue;
        env_strs.push_back(*e);
    }
    env_strs.push_back("SPATIAL_OS_SOCK=" + sock);
    env_strs.push_back("SPATULA_MAC_NO_BONJOUR=1");
    // Backstop: the window closes itself even if this test dies mid-run.
    env_strs.push_back("SPATULA_MAC_EXIT_AFTER=90");
    env_strs.push_back("XDG_CONFIG_HOME=" + cfg);
    std::vector<char *> envp;
    for (auto &s : env_strs)
        envp.push_back(const_cast<char *>(s.c_str()));
    envp.push_back(nullptr);

    std::string port_str = std::to_string(port);
    const char *bin = MAC_SHELL_BIN;
    std::vector<char *> argv = {const_cast<char *>(bin),
                                const_cast<char *>("--port"),
                                const_cast<char *>(port_str.c_str()), nullptr};
    pid_t child = -1;
    int rc = posix_spawn(&child, bin, nullptr, nullptr, argv.data(),
                         envp.data());
    if (rc != 0) {
        std::fprintf(stderr, "posix_spawn(%s) failed: %s\n", bin,
                     std::strerror(rc));
        return 1;
    }

    ctl_client c;
    if (!c.connect_path(sock, 20000)) {
        std::fprintf(stderr,
                     "control socket never came up — no window server for a "
                     "GUI process? skipping\n");
        kill(child, SIGTERM);
        waitpid(child, nullptr, 0);
        return SKIP_EXIT;
    }
    CHECK_MSG(c.request("version") == "ok proto=1", "version");

    // ---- default path ----
    std::string reply = c.request("screenshot");
    std::string shot = path_of(reply);
    CHECK_MSG(!shot.empty(), reply.c_str());
    uint32_t w = 0, h = 0;
    if (!shot.empty()) {
        CHECK_MSG(shot.rfind(dir + "/spatula-shot-", 0) == 0, shot.c_str());
        CHECK_MSG(png_dimensions(shot, w, h), shot.c_str());
        CHECK_MSG(w >= MIN_PLAUSIBLE_PX && h >= MIN_PLAUSIBLE_PX,
                  (std::to_string(w) + "x" + std::to_string(h)).c_str());
        unlink(shot.c_str());
    }

    // ---- re-armable: the second call is served by a later frame ----
    // The reply names the realpath, which on macOS resolves $TMPDIR's
    // /var → /private/var symlink.
    reply = c.request("screenshot " + custom);
    std::string custom_shot = path_of(reply);
    CHECK_MSG(!custom_shot.empty(), reply.c_str());
    CHECK_MSG(custom_shot.size() >= custom_leaf.size() &&
                  custom_shot.compare(custom_shot.size() - custom_leaf.size(),
                                      custom_leaf.size(), custom_leaf) == 0,
              custom_shot.c_str());
    uint32_t w2 = 0, h2 = 0;
    CHECK_MSG(png_dimensions(custom, w2, h2), custom.c_str());
    CHECK_MSG(w2 == w && h2 == h, "second grab differs in size");
    unlink(custom.c_str());

    // ---- refusals ----
    reply = c.request("screenshot /etc/x.png");
    CHECK_MSG(reply == "err bad_path outside_allowed_roots", reply.c_str());
    CHECK_MSG(access("/etc/x.png", F_OK) != 0, "/etc/x.png was written");

    reply = c.request("screenshot relative.png");
    CHECK_MSG(reply == "err bad_path not_absolute", reply.c_str());

    // A symlink at the target is refused, not followed (O_NOFOLLOW).
    if (symlink((dir + "/mac-shell-shot-e2e-victim-" + tag + ".png").c_str(),
                link.c_str()) == 0) {
        reply = c.request("screenshot " + link);
        CHECK_MSG(reply == "err capture_failed open_failed", reply.c_str());
        unlink(link.c_str());
    }

    // ---- per-window tier ----
    CHECK_MSG(c.request("launch shot-target") == "ok", "launch");
    reply = c.request("screenshot 1");
    std::string panel_shot = path_of(reply);
    CHECK_MSG(!panel_shot.empty(), reply.c_str());
    if (!panel_shot.empty()) {
        uint32_t pw = 0, ph = 0;
        CHECK_MSG(png_dimensions(panel_shot, pw, ph), panel_shot.c_str());
        CHECK_MSG(pw >= MIN_PLAUSIBLE_PX && ph >= MIN_PLAUSIBLE_PX,
                  (std::to_string(pw) + "x" + std::to_string(ph)).c_str());
        unlink(panel_shot.c_str());
    }
    reply = c.request("screenshot 999999");
    CHECK_MSG(reply == "err capture_failed no_such_window", reply.c_str());

    reply = c.request("screenshot 1 scale=0.5");
    CHECK_MSG(reply.rfind("err unsupported", 0) == 0, reply.c_str());

    c.close_fd();
    kill(child, SIGTERM);
    waitpid(child, nullptr, 0);
    system(("rm -rf '" + cfg + "'").c_str());

    if (g_failures == 0)
        std::printf("test_screenshot_e2e: all checks passed (%ux%u frame)\n", w,
                    h);
    return g_failures == 0 ? 0 : 1;
}
