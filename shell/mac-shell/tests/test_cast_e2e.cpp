// test_cast_e2e.cpp — the `cast` control verb over a real socket, against two
// synthesised replay sessions:
//   * planes only        → cast falls back to the ARKit plane list (source=plane)
//   * planes + LiDAR     → cast marches the depth map instead (source=depth)
// Same harness shape as tests/test_hack_verbs_e2e.cpp.

#include <cassert>
#include <cmath>
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

extern char **environ;

static int g_failures = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, \
                         #cond);                                         \
            g_failures++;                                                \
        }                                                                \
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
// session .bin synthesis (wire framing: u32 LE length, then the packet)
// ---------------------------------------------------------------------------

static void put_u8(std::vector<uint8_t> &b, uint8_t v) { b.push_back(v); }
static void put_u16(std::vector<uint8_t> &b, uint16_t v) {
    b.push_back((uint8_t)v);
    b.push_back((uint8_t)(v >> 8));
}
static void put_u32(std::vector<uint8_t> &b, uint32_t v) {
    for (int i = 0; i < 4; i++)
        b.push_back((uint8_t)(v >> (8 * i)));
}
static void put_u64(std::vector<uint8_t> &b, uint64_t v) {
    for (int i = 0; i < 8; i++)
        b.push_back((uint8_t)(v >> (8 * i)));
}
static void put_f32(std::vector<uint8_t> &b, float v) {
    uint32_t u;
    std::memcpy(&u, &v, 4);
    put_u32(b, u);
}

// Head at the origin, identity rotation: scene forward is -Z.
static std::vector<uint8_t> pose_packet(uint64_t ts_ns) {
    std::vector<uint8_t> p;
    put_u8(p, 0x01);
    put_u64(p, ts_ns);
    for (int i = 0; i < 3; i++)
        put_f32(p, 0.0f);
    put_f32(p, 0.0f);
    put_f32(p, 0.0f);
    put_f32(p, 0.0f);
    put_f32(p, 1.0f);
    put_f32(p, 1.0f);
    return p;
}

// A 2.0 x 1.2 m horizontal plane centred at (0, cy, -1).
static std::vector<uint8_t> plane_packet(uint64_t ts_ns, float cy) {
    std::vector<uint8_t> p;
    put_u8(p, 0x02);
    put_u64(p, ts_ns);
    for (int i = 0; i < 16; i++)
        put_u8(p, 0x11);
    put_f32(p, 0.0f);
    put_f32(p, cy);
    put_f32(p, -1.0f);
    put_f32(p, 0.0f);
    put_f32(p, 1.0f);
    put_f32(p, 0.0f);
    put_f32(p, 2.0f);
    put_f32(p, 1.2f);
    put_u8(p, 0);  // horizontal
    put_u8(p, 0);
    return p;
}

static constexpr int DW = 48, DH = 36;
// Half-FOV tangents 24/20 across and 18/20 down — wide enough that a ray
// 39 deg below the axis still lands inside the depth map.
static constexpr float DFX = 20.0f, DFY = 20.0f;
static constexpr float FLOOR_Y = -0.8f, WALL_Z = -2.0f;

static std::vector<uint8_t> intrinsics_packet(uint64_t ts_ns) {
    std::vector<uint8_t> p;
    put_u8(p, 0x0A);
    put_u64(p, ts_ns);
    put_f32(p, DFX);
    put_f32(p, DFY);
    put_f32(p, (float)DW * 0.5f);
    put_f32(p, (float)DH * 0.5f);
    put_f32(p, (float)DW);
    put_f32(p, (float)DH);
    return p;
}

// 0x0B is always chunked on the wire; a map this small fits one chunk.
// Payload sub-header: u16 w, u16 h, u8 enc=1, f32 z_min, f32 z_max, then
// u16 quantized samples (0 = no reading).
static std::vector<uint8_t> depth_packet(uint64_t ts_ns) {
    const float z_min = 0.1f, z_max = 8.0f;
    std::vector<uint8_t> payload;
    put_u16(payload, (uint16_t)DW);
    put_u16(payload, (uint16_t)DH);
    put_u8(payload, 1);
    put_f32(payload, z_min);
    put_f32(payload, z_max);
    const float scale = (z_max - z_min) / 65535.0f;
    for (int y = 0; y < DH; y++) {
        for (int x = 0; x < DW; x++) {
            (void)x;
            float ry = -((float)y + 0.5f - (float)DH * 0.5f) / DFY;
            float d = -WALL_Z;
            if (ry < -1e-6f) {
                float d_floor = FLOOR_Y / ry;
                if (d_floor < d)
                    d = d_floor;
            }
            uint32_t q = (uint32_t)((d - z_min) / scale + 0.5f);
            if (q < 1)
                q = 1;
            if (q > 65535)
                q = 65535;
            put_u16(payload, (uint16_t)q);
        }
    }
    std::vector<uint8_t> p;
    put_u8(p, 0x0B);
    put_u64(p, ts_ns);
    put_u32(p, (uint32_t)payload.size());  // total_size
    put_u32(p, 0);                         // offset
    put_u16(p, (uint16_t)payload.size());  // chunk length
    p.insert(p.end(), payload.begin(), payload.end());
    return p;
}

static bool write_session(const std::string &path, bool with_depth) {
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
    const uint64_t base = 1000000000ull, step = 16000000ull;
    for (int i = 0; i < 180; i++) {
        uint64_t ts = base + step * (uint64_t)i;
        frame(pose_packet(ts));
        if (i % 60 == 0)
            frame(plane_packet(ts, FLOOR_Y));
        if (with_depth && i % 30 == 10) {
            frame(intrinsics_packet(ts));
            frame(depth_packet(ts));
        }
    }
    std::fclose(f);
    return true;
}

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
    bool recv_line(std::string &out, int timeout_ms = 5000) {
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
            char buf[8192];
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

static bool field_vec3(const std::string &json, const std::string &key,
                       float out[3]) {
    size_t at = json.find("\"" + key + "\":[");
    if (at == std::string::npos)
        return false;
    return std::sscanf(json.c_str() + at + key.size() + 4, "%f,%f,%f", &out[0],
                       &out[1], &out[2]) == 3;
}

// ---------------------------------------------------------------------------

static void test_plane_fallback(ctl_client &c) {
    // The head ray is horizontal and the only plane is horizontal, so the two
    // never meet: `cast` answers rather than erroring.
    std::string r = c.request("cast");
    std::fprintf(stderr, "cast (head ray, planes only): %s\n", r.c_str());
    CHECK_MSG(r.rfind("ok {", 0) == 0, r.c_str());
    CHECK_MSG(contains(r, "\"source\":\"none\""), r.c_str());
    CHECK_MSG(contains(r, "\"hit\":null"), r.c_str());
    CHECK_MSG(contains(r, "\"kind\":null"), r.c_str());

    // Aimed down at the plane: no depth map in this session, so the plane
    // list answers.
    r = c.request("cast 0 0 0 0 -1 -1");
    std::fprintf(stderr, "cast (down-forward, planes only): %s\n", r.c_str());
    CHECK_MSG(r.rfind("ok {", 0) == 0, r.c_str());
    CHECK_MSG(contains(r, "\"source\":\"plane\""), r.c_str());
    CHECK_MSG(contains(r, "\"kind\":\"horizontal\""), r.c_str());
    float hit[3] = {0, 0, 0}, n[3] = {0, 0, 0};
    CHECK_MSG(field_vec3(r, "hit", hit), r.c_str());
    CHECK_MSG(std::fabs(hit[1] - FLOOR_Y) < 0.02f, r.c_str());
    CHECK_MSG(std::fabs(hit[2] - (-0.8f)) < 0.02f, r.c_str());
    CHECK_MSG(field_vec3(r, "normal", n), r.c_str());
    CHECK_MSG(std::fabs(n[1] - 1.0f) < 0.02f, r.c_str());

    // Error paths.
    CHECK(contains(c.request("cast 0 0 0"), "err parse_error"));
    CHECK(contains(c.request("cast 0 0 0 0 -1 -1 extra"), "err parse_error"));
    CHECK(contains(c.request("cast nope"), "err parse_error"));
}

static void test_depth_source(ctl_client &c) {
    // Head ray straight ahead: the plane list cannot answer (its only plane is
    // horizontal), but the depth map has a wall at z = -2.
    std::string r = c.request("cast");
    std::fprintf(stderr, "cast (head ray, with depth): %s\n", r.c_str());
    CHECK_MSG(contains(r, "\"source\":\"depth\""), r.c_str());
    CHECK_MSG(contains(r, "\"kind\":\"vertical\""), r.c_str());
    float hit[3] = {0, 0, 0};
    CHECK_MSG(field_vec3(r, "hit", hit), r.c_str());
    CHECK_MSG(std::fabs(hit[2] - WALL_Z) < 0.1f, r.c_str());

    r = c.request("cast 0 0 0 0 -0.8 -1");
    std::fprintf(stderr, "cast (down-forward, with depth): %s\n", r.c_str());
    CHECK_MSG(contains(r, "\"source\":\"depth\""), r.c_str());
    CHECK_MSG(contains(r, "\"kind\":\"horizontal\""), r.c_str());
    CHECK_MSG(field_vec3(r, "hit", hit), r.c_str());
    CHECK_MSG(std::fabs(hit[1] - FLOOR_Y) < 0.06f, r.c_str());
}

// ---------------------------------------------------------------------------

static std::string tmp_dir() {
    std::string dir = "/tmp";
    if (const char *t = getenv("TMPDIR"); t && t[0])
        dir = t;
    if (dir.back() == '/')
        dir.pop_back();
    return dir;
}

static void run_session(const std::string &session, const std::string &sock,
                        void (*body)(ctl_client &)) {
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
        g_failures++;
        return;
    }

    ctl_client c;
    CHECK_MSG(c.connect_path(sock, 10000), "control socket never came up");
    if (c.fd >= 0) {
        CHECK(c.request("version") == "ok proto=1");
        // Let the replay run far enough in that the first depth frame and its
        // intrinsics have landed.
        usleep(800 * 1000);
        body(c);
        c.close_fd();
    }

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
}

int main() {
    std::string dir = tmp_dir();
    std::string tag = std::to_string((long)getpid());

    std::string plain = dir + "/mac-shell-cast-plain-" + tag + ".bin";
    std::string deep = dir + "/mac-shell-cast-depth-" + tag + ".bin";
    CHECK(write_session(plain, false));
    CHECK(write_session(deep, true));

    run_session(plain, dir + "/mac-shell-cast-a-" + tag + ".sock",
                test_plane_fallback);
    run_session(deep, dir + "/mac-shell-cast-b-" + tag + ".sock",
                test_depth_source);

    unlink(plain.c_str());
    unlink(deep.c_str());

    if (g_failures) {
        std::fprintf(stderr, "test_cast_e2e: %d FAILURES\n", g_failures);
        return 1;
    }
    std::printf("test_cast_e2e: all tests passed\n");
    return 0;
}
