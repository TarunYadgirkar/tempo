// test_ui_e2e.cpp — headless end-to-end for the keyboard + launcher verbs.
//
// Spawns mac-shell headless in LIVE mode (UDP on a private port), streams
// wire-protocol hand packets at it, and drives the control socket:
//   keyboard show → status (plane basis) → synthetic fingertip plunge taps
//   (kbd_tap_traj conventions) over real keys → typed chars appear in the
//   focused panel's input log via dump-state;
//   launcher show/select/commit/status → panel spawn + events.
// Live UDP (rather than --replay) lets the test read the actual keyboard
// plane back from `keyboard status` before synthesising the trajectories.
//
// Deliberately streams NO pose packets: without a camera pose the receiver's
// bone-length hand reconstruction (hr_reconstruct) passes joints through
// verbatim, so the synthetic fingertips land exactly on the keys. The scene
// frame then equals the world frame and the floating keyboard plane sits at
// its head-relative defaults. The posed path is covered by tests/test_keyboard
// (direct injection) and the gesture-eval synthetic clips.

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

extern "C" {
#include "kbd_tap_traj.h"
#include "keyboard_geom.h"
}

extern char **environ;

static int g_failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, \
                         #cond);                                          \
            g_failures++;                                                 \
        }                                                                 \
    } while (0)

#define CHECK_MSG(cond, msg)                                                  \
    do {                                                                      \
        if (!(cond)) {                                                        \
            std::fprintf(stderr, "FAIL %s:%d: %s (%s)\n", __FILE__, __LINE__, \
                         #cond, (msg));                                       \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

// ---------------------------------------------------------------------------
// wire-protocol packet builders
// ---------------------------------------------------------------------------

static void put_u8(std::vector<uint8_t> &b, uint8_t v) { b.push_back(v); }
static void put_u64(std::vector<uint8_t> &b, uint64_t v) {
    for (int i = 0; i < 8; i++)
        b.push_back((uint8_t)(v >> (8 * i)));
}
static void put_f32(std::vector<uint8_t> &b, float v) {
    uint32_t u;
    std::memcpy(&u, &v, 4);
    for (int i = 0; i < 4; i++)
        b.push_back((uint8_t)(u >> (8 * i)));
}

// 21 joints, [x,y,z,confidence,reserved] each.
static std::vector<uint8_t> hand_packet(uint64_t ts_ns, uint8_t hand_index,
                                        const float joints[21][3]) {
    std::vector<uint8_t> p;
    put_u8(p, 0x05);
    put_u64(p, ts_ns);
    put_u8(p, hand_index);
    put_u8(p, 21);
    for (int j = 0; j < 21; j++) {
        put_f32(p, joints[j][0]);
        put_f32(p, joints[j][1]);
        put_f32(p, joints[j][2]);
        put_f32(p, 0.9f);
        put_f32(p, 0.0f);
    }
    return p;
}

// ---------------------------------------------------------------------------
// synthetic typing hand (mirrors tests/test_keyboard.cpp)
// ---------------------------------------------------------------------------

struct plane_basis {
    float origin[3], right[3], down[3], normal[3];
};

static void plane_to_world(const plane_basis &b, float x, float y, float z,
                           float out[3]) {
    for (int i = 0; i < 3; i++)
        out[i] = b.origin[i] + b.right[i] * x + b.down[i] * y +
                 b.normal[i] * z;
}

static void build_typing_joints(const plane_basis &b,
                                const float tips_plane[5][3],
                                float joints[21][3]) {
    std::memset(joints, 0, sizeof(float) * 21 * 3);
    // Vision joint indices.
    const int TIP[5] = {4, 8, 12, 16, 20};
    struct chain {
        int backs[3];  // dip, pip, mcp behind each tip
        float spread;
    };
    const chain CH[5] = {{{3, 2, 1}, -0.10f},
                         {{7, 6, 5}, -0.03f},
                         {{11, 10, 9}, 0.03f},
                         {{15, 14, 13}, 0.09f},
                         {{19, 18, 17}, 0.15f}};
    for (int t = 0; t < 5; t++) {
        plane_to_world(b, tips_plane[t][0], tips_plane[t][1],
                       tips_plane[t][2], joints[TIP[t]]);
        for (int k = 0; k < 3; k++) {
            float step = 0.03f * (float)(k + 1);
            joints[CH[t].backs[k]][0] =
                joints[TIP[t]][0] + CH[t].spread * 0.2f * (float)(k + 1);
            joints[CH[t].backs[k]][1] = joints[TIP[t]][1] + step;
            joints[CH[t].backs[k]][2] = joints[TIP[t]][2] + step;
        }
    }
    for (int i = 0; i < 3; i++)
        joints[0][i] = (joints[5][i] + joints[17][i]) * 0.5f +
                       (i == 1 ? 0.06f : 0.0f);
}

static void park_tips(float tips[5][3]) {
    for (int t = 0; t < 5; t++) {
        tips[t][0] = KBD_TAP_PARK_X + 0.05f * (float)t;
        tips[t][1] = KBD_TAP_PARK_Y;
        tips[t][2] = KBD_TAP_PARK_Z;
    }
}

// ---------------------------------------------------------------------------
// control-socket client (same shape as test_control_e2e.cpp)
// ---------------------------------------------------------------------------

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

static bool contains(const std::string &s, const std::string &sub) {
    return s.find(sub) != std::string::npos;
}

static bool parse_vec3(const std::string &json, const char *key,
                       float out[3]) {
    std::string pat = std::string("\"") + key + "\":[";
    size_t at = json.find(pat);
    if (at == std::string::npos)
        return false;
    return std::sscanf(json.c_str() + at + pat.size(), "%f,%f,%f", &out[0],
                       &out[1], &out[2]) == 3;
}

// ---------------------------------------------------------------------------

int main() {
    std::string dir = "/tmp";
    if (const char *t = getenv("TMPDIR"); t && t[0])
        dir = t;
    if (dir.back() == '/')
        dir.pop_back();
    std::string sock = dir + "/mac-shell-ui-e2e-" +
                       std::to_string((long)getpid()) + ".sock";
    std::string cfg = dir + "/mac-shell-ui-e2e-cfg-" +
                      std::to_string((long)getpid());
    int port = 19000 + (int)(getpid() % 2000);

    // Isolated config dir → compiled-default gesture/keyboard/launcher
    // configs regardless of the developer's real ~/.config.
    std::string mk = "mkdir -p '" + cfg + "'";
    CHECK(system(mk.c_str()) == 0);

    std::vector<std::string> env_strs;
    for (char **e = environ; *e; e++) {
        if (std::strncmp(*e, "SPATIAL_OS_SOCK=", 16) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_HEADLESS=", 21) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_NO_BONJOUR=", 23) == 0 ||
            std::strncmp(*e, "XDG_CONFIG_HOME=", 16) == 0)
            continue;
        env_strs.push_back(*e);
    }
    env_strs.push_back("SPATIAL_OS_SOCK=" + sock);
    env_strs.push_back("SPATULA_MAC_HEADLESS=1");
    env_strs.push_back("SPATULA_MAC_NO_BONJOUR=1");
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

    // UDP sender to the live shell.
    int udp = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in dst;
    std::memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons((uint16_t)port);
    dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    uint64_t ts = 1000000000ull;
    auto send_pkt = [&](const std::vector<uint8_t> &p) {
        sendto(udp, p.data(), p.size(), 0, (struct sockaddr *)&dst,
               sizeof(dst));
    };

    ctl_client c;
    CHECK_MSG(c.connect_path(sock, 10000), "control socket never came up");
    CHECK(c.request("version") == "ok proto=1");

    // Focused panel to type into.
    CHECK(c.request("launch typing-target") == "ok");
    CHECK(c.request("focus 1") == "ok");

    // ---- keyboard: show → status → plunge taps → typed chars ----
    std::string st = c.request("keyboard status");
    CHECK_MSG(contains(st, "\"visible\":false"), st.c_str());
    CHECK(c.request("keyboard show") == "ok");
    st = c.request("keyboard status");
    CHECK_MSG(contains(st, "\"visible\":true"), st.c_str());
    // No planes were streamed → floating default plane.
    CHECK_MSG(contains(st, "\"anchored\":false"), st.c_str());

    plane_basis b{};
    CHECK_MSG(parse_vec3(st, "origin", b.origin), st.c_str());
    CHECK(parse_vec3(st, "right", b.right));
    CHECK(parse_vec3(st, "down", b.down));
    CHECK(parse_vec3(st, "normal", b.normal));

    const uint32_t word[2] = {'h', 'i'};
    for (uint32_t keysym : word) {
        int idx = kbd_geom_find_keysym(keysym);
        assert(idx >= 0);
        const kbd_key_geom_t *g = kbd_geom_get(idx);
        float tips[5][3];
        float joints[21][3];
        for (int f = 0; f < KBD_TAP_TOTAL_FRAMES; f++) {
            park_tips(tips);
            tips[1][0] = g->cx_m;
            tips[1][1] = g->cy_m;
            tips[1][2] = kbd_tap_traj_z(f);
            build_typing_joints(b, tips, joints);
            send_pkt(hand_packet(ts, 0, joints));
            ts += 16000000ull;
            usleep(16 * 1000);
        }
        float park[5][3];
        park_tips(park);
        build_typing_joints(b, park, joints);
        for (int f = 0; f < 14; f++) {
            send_pkt(hand_packet(ts, 0, joints));
            ts += 16000000ull;
            usleep(16 * 1000);
        }
    }

    std::string ds;
    bool typed = false;
    for (int i = 0; i < 50 && !typed; i++) {
        ds = c.request("dump-state");
        typed = contains(ds, "type:h") && contains(ds, "type:i");
        if (!typed)
            usleep(100 * 1000);
    }
    CHECK_MSG(typed, ds.c_str());

    CHECK(c.request("keyboard hide") == "ok");
    st = c.request("keyboard status");
    CHECK_MSG(contains(st, "\"visible\":false"), st.c_str());
    CHECK(contains(c.request("keyboard bogus"), "err parse_error"));

    // ---- launcher: show/select/commit/status ----
    st = c.request("launcher status");
    CHECK_MSG(contains(st, "\"visible\":false"), st.c_str());
    CHECK(contains(st, "\"Test Card\""));  // compiled defaults
    CHECK(c.request("launcher show") == "ok");
    CHECK(c.request("launcher select 0") == "ok");
    st = c.request("launcher status");
    CHECK_MSG(contains(st, "\"focused\":0"), st.c_str());
    CHECK(contains(c.request("launcher select 99"), "err bad_index"));

    ctl_client sub;
    CHECK(sub.connect_path(sock, 2000));
    CHECK(sub.request("subscribe") == "ok");

    CHECK(c.request("launcher commit") == "ok");
    st = c.request("launcher status");
    CHECK_MSG(contains(st, "\"visible\":false"), st.c_str());
    // Default entry 0 = internal test-card → a new panel maps.
    std::string lw = c.request("list-windows");
    CHECK_MSG(contains(lw, "test-card"), lw.c_str());

    std::string ev;
    bool saw_map = false;
    while (sub.recv_line(ev, 3000)) {
        if (ev.rfind("event window-map", 0) == 0) {
            saw_map = true;
            break;
        }
    }
    CHECK_MSG(saw_map, "no window-map event after launcher commit");

    CHECK(contains(c.request("launcher commit"), "err no_selection"));

    sub.close_fd();
    c.close_fd();
    close(udp);

    kill(child, SIGTERM);
    int status = 0;
    for (int i = 0; i < 50; i++) {
        if (waitpid(child, &status, WNOHANG) == child) {
            child = -1;
            break;
        }
        usleep(100 * 1000);
    }
    if (child > 0) {
        kill(child, SIGKILL);
        waitpid(child, &status, 0);
        std::fprintf(stderr, "FAIL: mac-shell did not exit on SIGTERM\n");
        g_failures++;
    }
    unlink(sock.c_str());

    if (g_failures) {
        std::fprintf(stderr, "test_ui_e2e: %d FAILURES\n", g_failures);
        return 1;
    }
    std::printf("test_ui_e2e: all tests passed\n");
    return 0;
}
