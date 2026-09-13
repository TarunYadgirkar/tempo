// test_hands_e2e.cpp — `frame-export` and `hands-inject` over a real control
// socket, against a headless mac-shell replaying a synthesised session that
// carries all four streams the mac-side hand tracker needs: head pose (0x01),
// camera intrinsics (0x0A), a chunked JPEG camera frame (0x09) and a chunked
// LiDAR depth map (0x0B).
//
// What it pins:
//   * frame-export writes latest.jpg / latest.json / latest.depth, and the
//     sidecar carries the frame timestamp, the image size, the intrinsics,
//     the SCENE-frame head pose at capture and the depth geometry;
//   * latest.depth is exactly width*height float32 metres and reads back as
//     the values the wire carried (the tracker indexes it raw, so a stride or
//     dequantisation mistake here is silent and total);
//   * the directory containment rule the verb shares with `screenshot`;
//   * hands-inject replaces the phone's hands for the gesture engine, `hands
//     status` says which source is live, `hands dump` reads the joints back,
//     and the takeover expires so a dead tracker falls back to the phone.

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
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include "jpeg_fixture.h"

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
// session .bin synthesis
// ---------------------------------------------------------------------------

static const uint32_t CHUNK_PAYLOAD = 1200;  // must match the iOS sender
static const uint16_t DEPTH_W = 32, DEPTH_H = 24;
static const float DEPTH_Z_MIN = 0.1f, DEPTH_Z_MAX = 8.0f;
static const float DEPTH_VALUE_M = 1.5f;

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

static std::vector<uint8_t> pose_packet(uint64_t ts_ns) {
    std::vector<uint8_t> p;
    put_u8(p, 0x01);
    put_u64(p, ts_ns);
    put_f32(p, 0.0f);  // pos
    put_f32(p, 0.0f);
    put_f32(p, 0.0f);
    put_f32(p, 0.0f);  // rot xyzw = identity
    put_f32(p, 0.0f);
    put_f32(p, 0.0f);
    put_f32(p, 1.0f);
    put_f32(p, 1.0f);  // tracking quality
    return p;
}

static std::vector<uint8_t> intrinsics_packet(uint64_t ts_ns) {
    std::vector<uint8_t> p;
    put_u8(p, 0x0A);
    put_u64(p, ts_ns);
    put_f32(p, 1440.0f);  // fx
    put_f32(p, 1440.0f);  // fy
    put_f32(p, 960.0f);   // cx
    put_f32(p, 540.0f);   // cy
    put_f32(p, 1920.0f);  // image_width
    put_f32(p, 1080.0f);  // image_height
    return p;
}

// One 0x09/0x0B chunk. Header: type, ts, total, offset, chunk length.
static std::vector<uint8_t> chunk_packet(uint8_t type, uint64_t ts_ns,
                                         const uint8_t *payload, uint32_t total,
                                         uint32_t offset, uint16_t len) {
    std::vector<uint8_t> p;
    put_u8(p, type);
    put_u64(p, ts_ns);
    put_u32(p, total);
    put_u32(p, offset);
    put_u16(p, len);
    p.insert(p.end(), payload + offset, payload + offset + len);
    return p;
}

static void append_chunked(std::vector<std::vector<uint8_t>> &out, uint8_t type,
                           uint64_t ts_ns, const std::vector<uint8_t> &payload) {
    const uint32_t total = (uint32_t)payload.size();
    for (uint32_t off = 0; off < total; off += CHUNK_PAYLOAD) {
        const uint16_t len =
            (uint16_t)std::min<uint32_t>(CHUNK_PAYLOAD, total - off);
        out.push_back(
            chunk_packet(type, ts_ns, payload.data(), total, off, len));
    }
}

static std::vector<uint8_t> depth_payload() {
    std::vector<uint8_t> p;
    put_u16(p, DEPTH_W);
    put_u16(p, DEPTH_H);
    put_u8(p, 1);  // encoding: u16 linear
    put_f32(p, DEPTH_Z_MIN);
    put_f32(p, DEPTH_Z_MAX);
    const float scale = (DEPTH_Z_MAX - DEPTH_Z_MIN) / 65535.0f;
    const uint16_t q = (uint16_t)std::lround((DEPTH_VALUE_M - DEPTH_Z_MIN) / scale);
    for (int i = 0; i < DEPTH_W * DEPTH_H; i++)
        put_u16(p, q);
    return p;
}

// The replayed depth value as the receiver dequantises it, which is what the
// exported .bin must contain byte for byte.
static float expected_depth_m() {
    const float scale = (DEPTH_Z_MAX - DEPTH_Z_MIN) / 65535.0f;
    const uint16_t q = (uint16_t)std::lround((DEPTH_VALUE_M - DEPTH_Z_MIN) / scale);
    return DEPTH_Z_MIN + (float)q * scale;
}

static bool write_session(const std::string &path) {
    std::vector<std::vector<uint8_t>> packets;
    const uint64_t t0 = 1000000000000000000ull;  // arbitrary Unix-epoch ns
    const std::vector<uint8_t> jpeg(TEST_JPEG, TEST_JPEG + sizeof(TEST_JPEG));
    const std::vector<uint8_t> depth = depth_payload();

    // ~30 Hz for a second: every frame gets a pose ahead of it so the pose
    // ring can interpolate the capture instant, and the depth map is stamped
    // with the SAME timestamp as its colour frame (the sender's contract).
    for (int i = 0; i < 30; i++) {
        const uint64_t ts = t0 + (uint64_t)i * 33000000ull;
        packets.push_back(pose_packet(ts));
        packets.push_back(intrinsics_packet(ts));
        append_chunked(packets, 0x09, ts + 1000ull, jpeg);
        append_chunked(packets, 0x0B, ts + 1000ull, depth);
    }

    FILE *f = std::fopen(path.c_str(), "wb");
    if (!f)
        return false;
    for (const auto &pkt : packets) {
        uint32_t n = (uint32_t)pkt.size();
        uint8_t len[4] = {(uint8_t)n, (uint8_t)(n >> 8), (uint8_t)(n >> 16),
                          (uint8_t)(n >> 24)};
        std::fwrite(len, 1, 4, f);
        std::fwrite(pkt.data(), 1, pkt.size(), f);
    }
    std::fclose(f);
    return true;
}

// ---------------------------------------------------------------------------
// control client
// ---------------------------------------------------------------------------

struct ctl_client {
    int fd = -1;
    std::string pending;

    bool connect_path(const std::string &path, int timeout_ms) {
        for (int waited = 0; waited <= timeout_ms; waited += 100) {
            fd = socket(AF_UNIX, SOCK_STREAM, 0);
            if (fd < 0)
                return false;
            struct sockaddr_un addr {};
            addr.sun_family = AF_UNIX;
            std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
            if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0)
                return true;
            close(fd);
            fd = -1;
            usleep(100 * 1000);
        }
        return false;
    }
    bool recv_line(std::string &out, int timeout_ms = 5000) {
        for (;;) {
            size_t nl = pending.find('\n');
            if (nl != std::string::npos) {
                out = pending.substr(0, nl);
                pending.erase(0, nl + 1);
                return true;
            }
            struct pollfd pfd = {fd, POLLIN, 0};
            if (poll(&pfd, 1, timeout_ms) <= 0)
                return false;
            char buf[65536];
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

static std::string read_file(const std::string &path) {
    FILE *f = std::fopen(path.c_str(), "rb");
    if (!f)
        return {};
    std::string out;
    char buf[65536];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0)
        out.append(buf, n);
    std::fclose(f);
    return out;
}

// ---------------------------------------------------------------------------
// frame-export
// ---------------------------------------------------------------------------

static void test_frame_export(ctl_client &c, const std::string &export_dir) {
    std::string r = c.request("frame-export status");
    CHECK_MSG(r.rfind("ok off", 0) == 0, r.c_str());

    // The verb is a file-write primitive on an agent-reachable socket, so it
    // inherits `screenshot`'s containment rule.
    r = c.request("frame-export on relative/path");
    CHECK_MSG(r == "err not_absolute frame-export on relative/path", r.c_str());
    r = c.request("frame-export on /etc/spatula-frames");
    CHECK_MSG(contains(r, "err "), r.c_str());
    CHECK_MSG(!contains(r, "ok"), r.c_str());
    r = c.request("frame-export sideways");
    CHECK_MSG(contains(r, "err parse_error"), r.c_str());

    r = c.request("frame-export on " + export_dir);
    CHECK_MSG(contains(r, "ok on dir=") && contains(r, "hz=15"), r.c_str());

    // Wait for the first published frame. The sidecar is renamed last, so its
    // presence means the .jpg and .depth beside it are complete.
    std::string json;
    for (int waited = 0; waited < 8000; waited += 100) {
        json = read_file(export_dir + "/latest.json");
        if (!json.empty() && json.find('}') != std::string::npos)
            break;
        usleep(100 * 1000);
    }
    CHECK_MSG(!json.empty(), "no latest.json was ever written");
    if (json.empty())
        return;
    std::fprintf(stderr, "sidecar: %s", json.c_str());

    char imgbuf[64];
    std::snprintf(imgbuf, sizeof(imgbuf), "\"width\":%u,\"height\":%u",
                  TEST_JPEG_W, TEST_JPEG_H);
    CHECK_MSG(contains(json, imgbuf), json.c_str());
    CHECK_MSG(contains(json, "\"file\":\"latest.jpg\""), json.c_str());
    CHECK_MSG(contains(json, "\"t_ns\":1000000000"), json.c_str());

    // Intrinsics arrive as their own packet and are in ARKit capture pixels,
    // not the JPEG's — the tracker rescales, so both numbers must be here.
    CHECK_MSG(contains(json, "\"fx\":1440"), json.c_str());
    CHECK_MSG(contains(json, "\"cx\":960"), json.c_str());
    CHECK_MSG(contains(json, "\"image_width\":1920"), json.c_str());

    // The pose at capture, in the scene frame, is what makes the landmarks
    // land where the panels are.
    CHECK_MSG(contains(json, "\"head\":{\"frame\":\"scene\""), json.c_str());
    CHECK_MSG(contains(json, "\"quat\":["), json.c_str());

    char depthbuf[80];
    std::snprintf(depthbuf, sizeof(depthbuf),
                  "\"depth\":{\"width\":%u,\"height\":%u", DEPTH_W, DEPTH_H);
    CHECK_MSG(contains(json, depthbuf), json.c_str());
    CHECK_MSG(contains(json, "\"format\":\"float32\""), json.c_str());

    const std::string jpg = read_file(export_dir + "/latest.jpg");
    CHECK_MSG(jpg.size() > 100, "latest.jpg is implausibly small");
    CHECK_MSG(jpg.size() >= 2 && (uint8_t)jpg[0] == 0xFF &&
                  (uint8_t)jpg[1] == 0xD8,
              "latest.jpg is not a JPEG");

    const std::string raw = read_file(export_dir + "/latest.depth");
    CHECK(raw.size() == (size_t)DEPTH_W * DEPTH_H * sizeof(float));
    if (raw.size() == (size_t)DEPTH_W * DEPTH_H * sizeof(float)) {
        std::vector<float> d(DEPTH_W * DEPTH_H);
        std::memcpy(d.data(), raw.data(), raw.size());
        const float want = expected_depth_m();
        bool all_match = true;
        for (float v : d)
            if (std::fabs(v - want) > 1e-4f)
                all_match = false;
        CHECK_MSG(all_match, "latest.depth values are not the replayed metres");
    }

    // The export keeps running, so a second sidecar must appear with a higher
    // seq — a one-shot that silently stops is the failure mode that would
    // strand the tracker on a stale frame.
    const std::string first = json;
    for (int waited = 0; waited < 4000; waited += 100) {
        json = read_file(export_dir + "/latest.json");
        if (!json.empty() && json != first)
            break;
        usleep(100 * 1000);
    }
    CHECK_MSG(json != first, "frame-export published only one frame");

    r = c.request("frame-export off");
    CHECK_MSG(r.rfind("ok off frames=", 0) == 0, r.c_str());
    r = c.request("frame-export status");
    CHECK_MSG(r.rfind("ok off", 0) == 0, r.c_str());
}

// ---------------------------------------------------------------------------
// hands-inject
// ---------------------------------------------------------------------------

// A pinch-shaped hand: thumb tip (landmark 4) and index tip (landmark 8)
// almost touching, half a metre in front of the head.
static std::string pinch_payload(float x_offset) {
    std::string s = "{\"t\":1,\"hands\":[{\"chirality\":\"right\","
                    "\"confidence\":0.95,\"joints\":[";
    for (int mp = 0; mp < 21; mp++) {
        float x = x_offset, y = -0.10f + 0.005f * (float)mp, z = -0.50f;
        if (mp == 4)
            x = x_offset + 0.010f;
        if (mp == 8)
            x = x_offset + 0.012f;
        char buf[80];
        std::snprintf(buf, sizeof(buf), "%s[%.4f,%.4f,%.4f]", mp ? "," : "",
                      (double)x, (double)y, (double)z);
        s += buf;
    }
    return s + "]}]}";
}

static void test_hands_inject(ctl_client &c) {
    std::string r = c.request("hands status");
    CHECK_MSG(contains(r, "source=phone"), r.c_str());
    CHECK_MSG(contains(r, "overlay="), r.c_str());

    // The replayed session carries no 0x05 hands, so nothing is tracked yet.
    r = c.request("hands dump");
    CHECK_MSG(contains(r, "\"source\":\"phone\""), r.c_str());
    CHECK_MSG(contains(r, "\"hands\":[]"), r.c_str());

    CHECK(c.request("hands-inject " + pinch_payload(0.2f)) == "ok");
    r = c.request("hands status");
    CHECK_MSG(contains(r, "source=mac"), r.c_str());
    // `hands status` answers straight from the held injection; the joints
    // only reach hand_raw_ (and the gesture engine) on the next tick.
    usleep(80 * 1000);

    r = c.request("hands dump");
    CHECK_MSG(contains(r, "\"source\":\"mac\""), r.c_str());
    CHECK_MSG(contains(r, "\"frame\":\"scene\""), r.c_str());
    // Right hand -> scene slot 1: the injected slots are fixed by chirality,
    // unlike the phone's first-seen-order slots.
    CHECK_MSG(contains(r, "\"slot\":1"), r.c_str());
    // Landmark 0 (the wrist) reads back at the x it was sent at, which is the
    // whole point: the joints survived the MediaPipe -> SB reorder and came
    // back through it unchanged.
    CHECK_MSG(contains(r, "\"joints\":[[0.20000,"), r.c_str());

    // `aim` sees the injected pinch midpoint, which is the proof that the
    // joints reached the gesture path and not just an introspection buffer.
    CHECK(c.request("hands-inject " + pinch_payload(0.2f)) == "ok");
    usleep(80 * 1000);
    r = c.request("aim");
    CHECK_MSG(contains(r, "\"hands\":1"), r.c_str());
    CHECK_MSG(!contains(r, "\"aim\":null"), r.c_str());

    // Malformed payloads are refused without disturbing the live injection.
    CHECK(contains(c.request("hands-inject {\"t\":1}"), "err bad_json"));
    CHECK(contains(c.request("hands-inject nonsense"), "err bad_json"));
    CHECK(contains(c.request("hands status"), "source=mac"));

    // A tracker that dies must hand control back rather than freeze the
    // user's hands in mid-air.
    usleep(400 * 1000);
    r = c.request("hands status");
    CHECK_MSG(contains(r, "source=phone"), r.c_str());
    r = c.request("hands dump");
    CHECK_MSG(contains(r, "\"source\":\"phone\""), r.c_str());
}

// ---------------------------------------------------------------------------

int main() {
    std::string tmp = "/tmp";
    if (const char *t = getenv("TMPDIR"); t && t[0])
        tmp = t;
    if (tmp.back() == '/')
        tmp.pop_back();

    const std::string tag = std::to_string((long)getpid());
    const std::string session = tmp + "/tempo-hands-" + tag + ".bin";
    const std::string sock = tmp + "/tempo-hands.sock";
    const std::string export_dir = tmp + "/tempo-hands-export-" + tag;
    unlink(sock.c_str());
    CHECK(write_session(session));

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
    if (c.fd >= 0) {
        CHECK(c.request("version") == "ok proto=1");
        test_frame_export(c, export_dir);
        test_hands_inject(c);
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

    unlink(session.c_str());
    unlink(sock.c_str());
    unlink((export_dir + "/latest.jpg").c_str());
    unlink((export_dir + "/latest.json").c_str());
    unlink((export_dir + "/latest.depth").c_str());
    rmdir(export_dir.c_str());

    if (g_failures) {
        std::fprintf(stderr, "hands_e2e: %d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("hands_e2e: all checks passed\n");
    return 0;
}
