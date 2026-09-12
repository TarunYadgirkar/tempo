// test_hack_verbs_e2e.cpp — control-socket end-to-end for the three verbs
// added on top of the wxrd wire layer: `note` / `note-update`, `aim`, and
// `layout save|load|list`. Same shape as tests/test_control_e2e.cpp: a
// synthesised session .bin, a headless mac-shell on it, and a client that
// speaks the line protocol. Layout files land in a throwaway
// SPATIAL_OS_CONFIG_DIR, never the user's real ~/.config.

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
// session .bin synthesis (docs/wire-protocol.md framing)
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
    for (int i = 0; i < 3; i++)
        put_f32(p, 0.0f);
    put_f32(p, 0.0f);
    put_f32(p, 0.0f);
    put_f32(p, 0.0f);
    put_f32(p, 1.0f);
    put_f32(p, 1.0f);
    return p;
}

static std::vector<uint8_t> plane_packet(uint64_t ts_ns, uint8_t id, float cy,
                                         uint8_t alignment) {
    std::vector<uint8_t> p;
    put_u8(p, 0x02);
    put_u64(p, ts_ns);
    for (int i = 0; i < 16; i++)
        put_u8(p, id);
    put_f32(p, 0.0f);
    put_f32(p, cy);
    put_f32(p, -1.0f);
    put_f32(p, 0.0f);
    put_f32(p, 1.0f);
    put_f32(p, 0.0f);
    put_f32(p, 2.0f);
    put_f32(p, 1.2f);
    put_u8(p, alignment);
    put_u8(p, 0);
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
    const uint64_t base = 1000000000ull, step = 16000000ull;
    for (int i = 0; i < 180; i++) {
        uint64_t ts = base + step * (uint64_t)i;
        frame(pose_packet(ts));
        if (i % 60 == 0)
            frame(plane_packet(ts, 0x11, -0.8f, 0));
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

// ---------------------------------------------------------------------------

static void test_notes(ctl_client &c) {
    std::string r = c.request(
        R"(note {"title":"Standup notes","body":"Ship the aim verb, then the layout store.","accent":true})");
    CHECK_MSG(r == "ok handle=1", r.c_str());

    std::string lw = c.request("list-windows");
    CHECK_MSG(contains(lw, "\"app_id\":\"note\""), lw.c_str());
    CHECK_MSG(contains(lw, "\"title\":\"Standup notes\""), lw.c_str());
    CHECK_MSG(contains(lw, "\"size\":[512,320]"), lw.c_str());

    // A note without the accent flag is still a note.
    r = c.request(R"(note {"title":"Second","body":"No accent here."})");
    CHECK_MSG(r == "ok handle=2", r.c_str());

    CHECK(c.request(R"(note-update 1 {"title":"Standup notes, revised"})") ==
          "ok");
    CHECK(c.request(R"(note-update 1 {"body":"Both verbs landed."})") == "ok");
    CHECK(c.request(R"(note-update 1 {"accent":false})") == "ok");
    lw = c.request("list-windows");
    CHECK_MSG(contains(lw, "Standup notes, revised"), lw.c_str());

    // Error paths.
    r = c.request(R"(note-update 99 {"title":"nobody"})");
    CHECK_MSG(r == "err no_such_window", r.c_str());
    r = c.request(R"(note-update 1 {})");
    CHECK_MSG(contains(r, "err bad_json"), r.c_str());
    r = c.request("note-update 1");
    CHECK_MSG(contains(r, "err parse_error"), r.c_str());
    r = c.request(R"(note {"title":"only a title"})");
    CHECK_MSG(r == "err bad_json title and body are required strings",
              r.c_str());
    r = c.request(R"(note {"title":7,"body":"x"})");
    CHECK_MSG(r == "err bad_json title must be a string", r.c_str());
    r = c.request(R"(note {"title":"x","body":"y","accent":"yes"})");
    CHECK_MSG(r == "err bad_json accent must be true or false", r.c_str());
    r = c.request("note not-json-at-all");
    CHECK_MSG(contains(r, "err bad_json"), r.c_str());
}

static void test_aim(ctl_client &c) {
    std::string r = c.request("aim");
    std::fprintf(stderr, "aim reply: %s\n", r.c_str());
    CHECK_MSG(r.rfind("ok {", 0) == 0, r.c_str());
    // The replayed session carries no hands, so every vector is null and the
    // reply still answers rather than erroring.
    CHECK_MSG(contains(r, "\"hands\":0"), r.c_str());
    CHECK_MSG(contains(r, "\"aim\":null"), r.c_str());
    CHECK_MSG(contains(r, "\"ray_origin\":null"), r.c_str());
    CHECK_MSG(contains(r, "\"ray_dir\":null"), r.c_str());
    CHECK_MSG(contains(r, "\"pinching\":false"), r.c_str());
    CHECK_MSG(contains(r, "\"aimed_handle\":null"), r.c_str());
    CHECK_MSG(contains(r, "\"hit\":null"), r.c_str());
}

static void test_layouts(ctl_client &c) {
    CHECK(c.request("move 1 0.5 -0.2 -1.5") == "ok");
    CHECK(c.request("move 2 -0.5 -0.2 -1.5") == "ok");

    std::string r = c.request("layout save desk");
    CHECK_MSG(r == "ok saved=2", r.c_str());
    r = c.request("layout list");
    CHECK_MSG(r == "ok {\"layouts\":[\"desk\"]}", r.c_str());

    // Bad names never reach the filesystem.
    CHECK(c.request("layout save ../etc") == "err bad_name use 1-40 of [A-Za-z0-9_-]");
    CHECK(c.request("layout save two words") ==
          "err bad_name use 1-40 of [A-Za-z0-9_-]");
    CHECK(c.request("layout load nope") == "err not_found nope");
    CHECK(contains(c.request("layout bogus x"), "err parse_error"));

    CHECK(c.request("close 1") == "ok");
    CHECK(c.request("close 2") == "ok");
    std::string lw = c.request("list-windows");
    CHECK_MSG(contains(lw, "\"windows\":[]"), lw.c_str());

    // Load re-creates both notes at their saved poses, closing nothing.
    r = c.request("layout load desk");
    CHECK_MSG(r == "ok restored=2 skipped=0", r.c_str());
    usleep(200 * 1000);
    lw = c.request("list-windows");
    CHECK_MSG(contains(lw, "Standup notes, revised"), lw.c_str());
    CHECK_MSG(contains(lw, "\"app_id\":\"note\""), lw.c_str());
    float pos[3] = {0, 0, 0};
    CHECK_MSG(parse_pos_after(lw, "\"handle\":3", pos), lw.c_str());
    CHECK_MSG(std::fabs(pos[0] - 0.5f) < 0.01f, lw.c_str());
    CHECK_MSG(std::fabs(pos[2] - (-1.5f)) < 0.01f, lw.c_str());

    // Loading again adds a second copy rather than replacing the first.
    r = c.request("layout load desk");
    CHECK_MSG(r == "ok restored=2 skipped=0", r.c_str());
    lw = c.request("list-windows");
    CHECK_MSG(contains(lw, "\"handle\":6"), lw.c_str());
}

int main() {
    std::string dir = "/tmp";
    if (const char *t = getenv("TMPDIR"); t && t[0])
        dir = t;
    if (dir.back() == '/')
        dir.pop_back();
    std::string tag = std::to_string((long)getpid());
    std::string session = dir + "/mac-shell-hack-" + tag + ".bin";
    std::string sock = dir + "/mac-shell-hack-" + tag + ".sock";
    std::string cfg = dir + "/mac-shell-hack-cfg-" + tag;
    CHECK(write_session(session));

    std::vector<std::string> env_strs;
    for (char **e = environ; *e; e++) {
        if (std::strncmp(*e, "SPATIAL_OS_SOCK=", 16) == 0 ||
            std::strncmp(*e, "SPATIAL_OS_CONFIG_DIR=", 22) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_NO_BONJOUR=", 23) == 0 ||
            std::strncmp(*e, "SPATULA_MAC_HEADLESS=", 21) == 0)
            continue;
        env_strs.push_back(*e);
    }
    env_strs.push_back("SPATIAL_OS_SOCK=" + sock);
    env_strs.push_back("SPATIAL_OS_CONFIG_DIR=" + cfg);
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
        test_notes(c);
        test_aim(c);
        test_layouts(c);
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
    unlink((cfg + "/layouts/desk.json").c_str());
    rmdir((cfg + "/layouts").c_str());
    rmdir(cfg.c_str());

    if (g_failures) {
        std::fprintf(stderr, "test_hack_verbs_e2e: %d FAILURES\n", g_failures);
        return 1;
    }
    std::printf("test_hack_verbs_e2e: all tests passed\n");
    return 0;
}
