// test_control_e2e.cpp — end-to-end control-plane test.
//
// Synthesises a wire-protocol session file (pose + plane packets, the
// [u32 len][payload] .bin framing bridge-receiver replays), starts a headless
// mac-shell on it, and drives the unix control socket with a client speaking
// the wire protocol: launch, list-windows, move, anchor, type, dump-state,
// subscribe, plus the error paths. Asserts the reply shapes control.c
// produces.

#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

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
// session .bin synthesis (wire protocol packets, docs/wire-protocol.md)
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

static std::vector<uint8_t> pose_packet(uint64_t ts_ns) {
    std::vector<uint8_t> p;
    put_u8(p, 0x01);
    put_u64(p, ts_ns);
    put_f32(p, 0.0f);  // pos
    put_f32(p, 0.0f);
    put_f32(p, 0.0f);
    put_f32(p, 0.0f);  // rot xyzw identity
    put_f32(p, 0.0f);
    put_f32(p, 0.0f);
    put_f32(p, 1.0f);
    put_f32(p, 1.0f);  // quality normal
    return p;
}

static std::vector<uint8_t> plane_packet(uint64_t ts_ns, uint8_t id,
                                         float cx, float cy, float cz,
                                         float nx, float ny, float nz,
                                         float w, float h, uint8_t alignment) {
    std::vector<uint8_t> p;
    put_u8(p, 0x02);
    put_u64(p, ts_ns);
    for (int i = 0; i < 16; i++)
        put_u8(p, id);
    put_f32(p, cx);
    put_f32(p, cy);
    put_f32(p, cz);
    put_f32(p, nx);
    put_f32(p, ny);
    put_f32(p, nz);
    put_f32(p, w);
    put_f32(p, h);
    put_u8(p, alignment);
    put_u8(p, 0);  // is_removed
    return p;
}

static bool write_session(const std::string &path) {
    FILE *f = std::fopen(path.c_str(), "wb");
    if (!f)
        return false;
    auto frame = [&](const std::vector<uint8_t> &pkt) {
        uint32_t len = (uint32_t)pkt.size();
        uint8_t hdr[4] = {(uint8_t)len, (uint8_t)(len >> 8),
                          (uint8_t)(len >> 16), (uint8_t)(len >> 24)};
        std::fwrite(hdr, 1, 4, f);
        std::fwrite(pkt.data(), 1, pkt.size(), f);
    };

    const uint64_t base = 1000000000ull;
    const uint64_t step = 16000000ull;  // 16 ms

    // Desk (horizontal) + wall (vertical), sent up-front and refreshed once a
    // second so replay loops keep them alive.
    for (int i = 0; i < 180; i++) {  // ~2.9 s of session, then loops
        uint64_t ts = base + step * (uint64_t)i;
        frame(pose_packet(ts));
        if (i % 60 == 0) {
            frame(plane_packet(ts, 0x11, 0.0f, -0.8f, -1.0f, 0, 1, 0, 2.0f,
                               1.2f, 0));
            frame(plane_packet(ts, 0x22, 0.0f, 0.0f, -2.0f, 0, 0, 1, 2.0f,
                               2.0f, 1));
        }
    }
    std::fclose(f);
    return true;
}

// ---------------------------------------------------------------------------
// control-socket client
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

    // Reads one '\n'-terminated line, waiting up to timeout_ms.
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

    // Request/reply helper.
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

// Extracts the first "pos":[x,y,z] triple following the given anchor string.
static bool parse_pos_after(const std::string &json, const std::string &after,
                            float out[3]) {
    size_t at = json.find(after);
    if (at == std::string::npos)
        return false;
    size_t p = json.find("\"pos\":[", at);
    if (p == std::string::npos)
        return false;
    return std::sscanf(json.c_str() + p + 7, "%f,%f,%f", &out[0], &out[1],
                       &out[2]) == 3;
}

// With neither SPATIAL_OS_SOCK nor TMPDIR set there is no per-user socket
// location; the shell must exit non-zero instead of binding /tmp.
static void test_refuses_world_writable_socket(const char *bin,
                                               const std::string &session) {
    CHECK(write_session(session));
    struct stat st;
    bool tmp_sock_before = lstat("/tmp/spatial-os.sock", &st) == 0;

    std::vector<std::string> env_strs;
    for (char **e = environ; *e; e++) {
        if (std::strncmp(*e, "SPATIAL_OS_SOCK=", 16) == 0 ||
            std::strncmp(*e, "TMPDIR=", 7) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_NO_BONJOUR=", 23) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_HEADLESS=", 21) == 0)
            continue;
        env_strs.push_back(*e);
    }
    env_strs.push_back("SPATULA_MAC_HEADLESS=1");
    env_strs.push_back("SPATULA_MAC_NO_BONJOUR=1");
    std::vector<char *> envp;
    for (auto &s : env_strs)
        envp.push_back(const_cast<char *>(s.c_str()));
    envp.push_back(nullptr);
    std::vector<char *> argv = {const_cast<char *>(bin),
                                const_cast<char *>("--replay"),
                                const_cast<char *>(session.c_str()), nullptr};

    pid_t child = -1;
    int rc = posix_spawn(&child, bin, nullptr, nullptr, argv.data(),
                         envp.data());
    CHECK_MSG(rc == 0, std::strerror(rc));
    if (rc != 0)
        return;
    int status = 0;
    bool exited = false;
    for (int i = 0; i < 100; i++) {
        if (waitpid(child, &status, WNOHANG) == child) {
            exited = true;
            break;
        }
        usleep(100 * 1000);
    }
    if (!exited) {
        kill(child, SIGKILL);
        waitpid(child, &status, 0);
        CHECK_MSG(false, "mac-shell kept running without a socket dir");
    } else {
        CHECK_MSG(WIFEXITED(status) && WEXITSTATUS(status) != 0,
                  "expected non-zero exit when no socket dir is available");
    }
    bool tmp_sock_after = lstat("/tmp/spatial-os.sock", &st) == 0;
    CHECK_MSG(tmp_sock_after == tmp_sock_before,
              "mac-shell bound /tmp/spatial-os.sock");
    unlink(session.c_str());
}

// ---------------------------------------------------------------------------
// frame-stall observability
// ---------------------------------------------------------------------------

// A 16x16 JPEG. The receiver only stores a frame if libjpeg decodes it, so the
// stall test needs a real one to prove frames actually landed before they
// stopped.
static const uint8_t TINY_JPEG[] = {
    0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, 0x4a, 0x46, 0x49, 0x46, 0x00, 0x01,
    0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xff, 0xdb, 0x00, 0x43,
    0x00, 0x14, 0x0e, 0x0f, 0x12, 0x0f, 0x0d, 0x14, 0x12, 0x10, 0x12, 0x17,
    0x15, 0x14, 0x18, 0x1e, 0x32, 0x21, 0x1e, 0x1c, 0x1c, 0x1e, 0x3d, 0x2c,
    0x2e, 0x24, 0x32, 0x49, 0x40, 0x4c, 0x4b, 0x47, 0x40, 0x46, 0x45, 0x50,
    0x5a, 0x73, 0x62, 0x50, 0x55, 0x6d, 0x56, 0x45, 0x46, 0x64, 0x88, 0x65,
    0x6d, 0x77, 0x7b, 0x81, 0x82, 0x81, 0x4e, 0x60, 0x8d, 0x97, 0x8c, 0x7d,
    0x96, 0x73, 0x7e, 0x81, 0x7c, 0xff, 0xdb, 0x00, 0x43, 0x01, 0x15, 0x17,
    0x17, 0x1e, 0x1a, 0x1e, 0x3b, 0x21, 0x21, 0x3b, 0x7c, 0x53, 0x46, 0x53,
    0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c,
    0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c,
    0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c,
    0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c,
    0x7c, 0x7c, 0xff, 0xc0, 0x00, 0x11, 0x08, 0x00, 0x10, 0x00, 0x10, 0x03,
    0x01, 0x22, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01, 0xff, 0xc4, 0x00,
    0x15, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0xff, 0xc4, 0x00, 0x14,
    0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xc4, 0x00, 0x14, 0x01, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x03, 0xff, 0xc4, 0x00, 0x14, 0x11, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xff, 0xda, 0x00, 0x0c, 0x03, 0x01, 0x00, 0x02, 0x11, 0x03,
    0x11, 0x00, 0x3f, 0x00, 0x92, 0x00, 0x8a, 0xff, 0xd9};

static void put_u32(std::vector<uint8_t> &b, uint32_t v) {
    for (int i = 0; i < 4; i++)
        b.push_back((uint8_t)(v >> (8 * i)));
}

static std::vector<uint8_t> frame_packet(uint64_t ts_ns) {
    std::vector<uint8_t> p;
    put_u8(p, 0x03);
    put_u64(p, ts_ns);
    put_u32(p, 16);  // width
    put_u32(p, 16);  // height
    put_u32(p, (uint32_t)sizeof(TINY_JPEG));
    p.insert(p.end(), TINY_JPEG, TINY_JPEG + sizeof(TINY_JPEG));
    return p;
}

// Frames for the first ~100 ms, then poses alone for 4 s — the failure the
// passthrough guard exists for. No 0x0A packet either, so have_intrinsics
// stays 0.
static bool write_stall_session(const std::string &path) {
    FILE *f = std::fopen(path.c_str(), "wb");
    if (!f)
        return false;
    auto frame = [&](const std::vector<uint8_t> &pkt) {
        uint32_t len = (uint32_t)pkt.size();
        uint8_t hdr[4] = {(uint8_t)len, (uint8_t)(len >> 8),
                          (uint8_t)(len >> 16), (uint8_t)(len >> 24)};
        std::fwrite(hdr, 1, 4, f);
        std::fwrite(pkt.data(), 1, pkt.size(), f);
    };
    const uint64_t base = 1000000000ull;
    const uint64_t step = 16000000ull;  // 16 ms
    for (int i = 0; i < 250; i++) {     // 4.0 s of poses
        uint64_t ts = base + step * (uint64_t)i;
        frame(pose_packet(ts));
        if (i < 6)
            frame(frame_packet(ts));
    }
    std::fclose(f);
    return true;
}

static long long stats_field(const std::string &s, const char *key) {
    size_t p = s.find(key);
    if (p == std::string::npos)
        return -999;
    long long v = -999;
    std::sscanf(s.c_str() + p + std::strlen(key), "%lld", &v);
    return v;
}

static void test_frame_stall(const char *bin, const std::string &dir) {
    std::string session = dir + "/mac-shell-stall-" +
                          std::to_string((long)getpid()) + ".bin";
    std::string sock = dir + "/mac-shell-stall-" +
                       std::to_string((long)getpid()) + ".sock";
    CHECK(write_stall_session(session));

    std::vector<std::string> env_strs;
    for (char **e = environ; *e; e++) {
        if (std::strncmp(*e, "SPATIAL_OS_SOCK=", 16) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_HEADLESS=", 21) == 0)
            continue;
        env_strs.push_back(*e);
    }
    env_strs.push_back("SPATIAL_OS_SOCK=" + sock);
    env_strs.push_back("SPATULA_MAC_HEADLESS=1");
    std::vector<char *> envp;
    for (auto &s : env_strs)
        envp.push_back(const_cast<char *>(s.c_str()));
    envp.push_back(nullptr);

    // --once: without looping the frames never come back, so the age only
    // ever grows.
    std::vector<char *> argv = {const_cast<char *>(bin),
                                const_cast<char *>("--replay"),
                                const_cast<char *>(session.c_str()),
                                const_cast<char *>("--once"), nullptr};
    pid_t child = -1;
    int rc = posix_spawn(&child, bin, nullptr, nullptr, argv.data(),
                         envp.data());
    CHECK_MSG(rc == 0, std::strerror(rc));
    if (rc != 0) {
        unlink(session.c_str());
        return;
    }

    ctl_client c;
    CHECK_MSG(c.connect_path(sock, 10000), "stall-run socket never came up");
    if (c.fd >= 0) {
        // Frames did land: without this the "stalled" assertion below would
        // also pass on a session whose JPEGs never decoded.
        std::string early = c.request("stats");
        CHECK_MSG(stats_field(early, "frame_age_ms=") >= 0, early.c_str());

        usleep(2000 * 1000);

        std::string late = c.request("stats");
        CHECK_MSG(stats_field(late, "frame_age_ms=") >= 1000, late.c_str());
        CHECK_MSG(stats_field(late, "have_intrinsics=") == 0, late.c_str());
        // Poses are still flowing — the point of the guard is that packet
        // rate alone cannot tell you the camera died.
        double rate = 0.0;
        size_t p = late.find("packet_rate=");
        if (p != std::string::npos)
            std::sscanf(late.c_str() + p + 12, "%lf", &rate);
        CHECK_MSG(rate > 0.0, late.c_str());
    }

    kill(child, SIGTERM);
    int status = 0;
    waitpid(child, &status, 0);
    unlink(session.c_str());
    unlink(sock.c_str());
}

int main() {
    std::string dir = "/tmp";
    if (const char *t = getenv("TMPDIR"); t && t[0])
        dir = t;
    if (dir.back() == '/')
        dir.pop_back();
    std::string session = dir + "/mac-shell-e2e-" +
                          std::to_string((long)getpid()) + ".bin";
    std::string sock = dir + "/mac-shell-e2e-" +
                       std::to_string((long)getpid()) + ".sock";
    CHECK(write_session(session));

    // Spawn headless mac-shell on the replayed session.
    std::vector<std::string> env_strs;
    for (char **e = environ; *e; e++) {
        if (std::strncmp(*e, "SPATIAL_OS_SOCK=", 16) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_NO_BONJOUR=", 23) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_HEADLESS=", 21) == 0)
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

    const char *bin = MAC_SHELL_BIN;
    std::vector<char *> argv = {const_cast<char *>(bin),
                                const_cast<char *>("--replay"),
                                const_cast<char *>(session.c_str()), nullptr};

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

    // ---- meta ----
    CHECK(c.request("version") == "ok proto=1");

    // ---- launch → internal test-card panel ----
    CHECK(c.request("launch test-card-alpha") == "ok");
    std::string lw = c.request("list-windows");
    CHECK_MSG(contains(lw, "\"handle\":1"), lw.c_str());
    CHECK_MSG(contains(lw, "test-card-alpha"), lw.c_str());
    CHECK_MSG(contains(lw, "\"focused\":true"), lw.c_str());
    CHECK_MSG(contains(lw, "\"anchor\":null"), lw.c_str());

    // ---- world state: wait for the replayed planes to arrive ----
    std::string lp;
    for (int i = 0; i < 100; i++) {
        lp = c.request("list-planes");
        if (contains(lp, "11111111111111111111111111111111") &&
            contains(lp, "22222222222222222222222222222222"))
            break;
        usleep(100 * 1000);
    }
    CHECK_MSG(contains(lp, "11111111111111111111111111111111"), lp.c_str());
    CHECK_MSG(contains(lp, "\"alignment\":1"), lp.c_str());

    std::string hp = c.request("head-pose");
    CHECK_MSG(contains(hp, "\"tracking\":true"), hp.c_str());
    // Frames are labelled: head-pose is RAW (+ scene_*), window pos is SCENE.
    CHECK_MSG(contains(hp, "\"frame\":\"raw\""), hp.c_str());
    CHECK_MSG(contains(hp, "\"scene_pos\":["), hp.c_str());
    CHECK_MSG(contains(hp, "\"scene_rot\":["), hp.c_str());

    // ---- move + anchor ----
    CHECK(c.request("move 1 0 -0.75 -1") == "ok");
    CHECK(c.request("anchor 1 closest-horizontal") ==
          "ok anchor=11111111111111111111111111111111");
    usleep(300 * 1000);  // a few ticks so the anchor transform lands

    std::string ds = c.request("dump-state");
    CHECK_MSG(contains(ds, "\"anchor\":\"11111111"), ds.c_str());
    CHECK_MSG(contains(ds, "\"frame\":\"scene\""), ds.c_str());
    float pos[3] = {0, 0, 0};
    CHECK_MSG(parse_pos_after(ds, "\"handle\":1", pos), ds.c_str());
    CHECK_MSG(std::fabs(pos[1] - (-0.78f)) < 0.02f, ds.c_str());
    CHECK_MSG(std::fabs(pos[2] - (-1.0f)) < 0.02f, ds.c_str());

    CHECK(c.request("query 1") ==
          "ok anchor=11111111111111111111111111111111");
    CHECK(c.request("clear-anchor 1") == "ok");
    CHECK(c.request("query 1") == "ok no_anchor");

    // anchor closest-wall from mid-air → no_anchor.
    CHECK(c.request("move 1 0 0 -1") == "ok");
    CHECK(c.request("anchor 1 closest-wall") == "ok no_anchor");

    // explicit uuid anchor.
    CHECK(c.request("anchor 1 22222222222222222222222222222222") ==
          "ok anchor=22222222222222222222222222222222");
    CHECK(c.request("clear-anchor 1") == "ok");

    // ---- hand skeleton overlay toggles at runtime ----
    CHECK(c.request("hands overlay status") == "ok overlay=on");  // default
    CHECK(c.request("hands overlay off") == "ok overlay=off");
    CHECK(c.request("hands overlay status") == "ok overlay=off");
    CHECK(c.request("hands overlay on") == "ok overlay=on");
    CHECK(contains(c.request("hands overlay sideways"), "err parse_error"));

    // ---- input routed to the focused internal panel ----
    CHECK(c.request("focus 1") == "ok");
    CHECK(c.request("type hello world") == "ok");
    CHECK(c.request("key Return") == "ok");
    CHECK(c.request("click 1 10 20") == "ok");
    CHECK(c.request("scroll 0 -3") == "ok");
    ds = c.request("dump-state");
    CHECK_MSG(contains(ds, "type:hello world"), ds.c_str());
    CHECK_MSG(contains(ds, "key:Return"), ds.c_str());
    CHECK_MSG(contains(ds, "click:10,20 btn=0"), ds.c_str());

    // ---- resize ----
    CHECK(c.request("resize 1 640 480") == "ok");
    lw = c.request("list-windows");
    CHECK_MSG(contains(lw, "\"size\":[640,480]"), lw.c_str());

    // ---- error paths ----
    CHECK(c.request("move 99 0 0 0") == "err no_such_window");
    CHECK(c.request("bogus-verb 1") == "err parse_error bogus-verb 1");
    CHECK(c.request("a11y 1") == "err unimplemented");

    // ---- screenshot: per-window works headless (the panel's own pixels),
    // the whole-scene form needs a renderer ----
    std::string shot = c.request("screenshot 1");
    CHECK_MSG(shot.rfind("ok path=", 0) == 0, shot.c_str());
    if (shot.rfind("ok path=", 0) == 0) {
        std::string p = shot.substr(8);
        std::FILE *f = std::fopen(p.c_str(), "rb");
        CHECK_MSG(f != nullptr, p.c_str());
        if (f) {
            unsigned char magic[8] = {0};
            CHECK(std::fread(magic, 1, 8, f) == 8);
            std::fclose(f);
            CHECK(magic[0] == 0x89 && magic[1] == 'P' && magic[2] == 'N' &&
                  magic[3] == 'G');
            unlink(p.c_str());
        }
    }
    CHECK(c.request("screenshot 99") == "err capture_failed no_such_window");
    CHECK(c.request("screenshot") ==
          "err unimplemented renderer not running");
    CHECK(c.request("screenshot /etc/x.png") ==
          "err bad_path outside_allowed_roots");

    // ---- subscribe event stream on a second connection ----
    ctl_client sub;
    CHECK(sub.connect_path(sock, 2000));
    CHECK(sub.request("subscribe") == "ok");
    CHECK(c.request("launch test-card-beta") == "ok");
    std::string ev;
    bool saw_map = false;
    while (sub.recv_line(ev, 3000)) {
        if (ev == "event window-map handle=2") {
            saw_map = true;
            break;
        }
    }
    CHECK_MSG(saw_map, "no window-map event on subscribed conn");

    // no-focus guard: close everything, then type.
    CHECK(c.request("close 1") == "ok");
    CHECK(c.request("close 2") == "ok");
    lw = c.request("list-windows");
    CHECK_MSG(contains(lw, "\"windows\":[]"), lw.c_str());
    std::string tr = c.request("type nobody-home");
    CHECK_MSG(contains(tr, "err no_focus"), tr.c_str());

    // ---- panel cap: MAX_PANELS (32) launches ok, the next is refused ----
    for (int i = 0; i < 32; i++) {
        std::string r = c.request("launch cap-" + std::to_string(i));
        CHECK_MSG(r == "ok", r.c_str());
    }
    CHECK(c.request("launch cap-overflow") == "err resource_exhausted");
    CHECK(contains(c.request("launch-app cap-overflow"),
                   "err resource_exhausted"));

    bool saw_unmap = false;
    while (sub.recv_line(ev, 3000)) {
        if (ev == "event window-unmap handle=1") {
            saw_unmap = true;
            break;
        }
    }
    CHECK_MSG(saw_unmap, "no window-unmap event on subscribed conn");

    sub.close_fd();
    c.close_fd();

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

    unlink(session.c_str());
    unlink(sock.c_str());

    test_refuses_world_writable_socket(bin, session);
    test_frame_stall(bin, dir);

    if (g_failures) {
        std::fprintf(stderr, "test_control_e2e: %d FAILURES\n", g_failures);
        return 1;
    }
    std::printf("test_control_e2e: all tests passed\n");
    return 0;
}
