// test_port_retry_e2e.cpp — a busy UDP port must not be a permanent
// condition.
//
// Occupies the port first, spawns headless mac-shell on it (which comes up
// deaf and says so), then frees the port and asserts the shell rebinds on
// its own — `stats` reports a live packet_rate within a few seconds, with no
// relaunch. Before the retry loop this stayed at 0 forever.

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

    std::string request(const std::string &line) {
        std::string out = line + "\n";
        if (write(fd, out.data(), out.size()) != (ssize_t)out.size())
            return "<send-failed>";
        while (true) {
            size_t nl = pending.find('\n');
            if (nl != std::string::npos) {
                std::string reply = pending.substr(0, nl);
                pending.erase(0, nl + 1);
                return reply;
            }
            struct pollfd pfd = {fd, POLLIN, 0};
            if (poll(&pfd, 1, 5000) <= 0)
                return "<recv-timeout>";
            char buf[4096];
            ssize_t r = read(fd, buf, sizeof(buf));
            if (r <= 0)
                return "<eof>";
            pending.append(buf, (size_t)r);
        }
    }

    void close_fd() {
        if (fd >= 0)
            close(fd);
        fd = -1;
    }
};

static double packet_rate_of(const std::string &stats) {
    size_t at = stats.find("packet_rate=");
    if (at == std::string::npos)
        return -1.0;
    return atof(stats.c_str() + at + std::strlen("packet_rate="));
}

static std::string read_file(const std::string &path) {
    std::string out;
    FILE *f = fopen(path.c_str(), "r");
    if (!f)
        return out;
    char buf[4096];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
        out.append(buf, n);
    fclose(f);
    return out;
}

int main() {
    std::string dir = "/tmp";
    if (const char *t = getenv("TMPDIR"); t && t[0])
        dir = t;
    if (dir.back() == '/')
        dir.pop_back();
    std::string tag = std::to_string((long)getpid());
    std::string sock = dir + "/mac-shell-retry-" + tag + ".sock";
    std::string log = dir + "/mac-shell-retry-" + tag + ".log";
    int port = 21000 + (int)(getpid() % 2000);

    // ---- occupy the port (no SO_REUSEADDR: the shell's bind must fail) ----
    int squatter = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in bind_addr;
    std::memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons((uint16_t)port);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    CHECK_MSG(squatter >= 0 && bind(squatter, (struct sockaddr *)&bind_addr,
                                    sizeof(bind_addr)) == 0,
              "could not occupy the test port");
    // Without CLOEXEC the spawned shell inherits this fd and keeps the port
    // bound after we close it here, so it could never rebind.
    fcntl(squatter, F_SETFD, FD_CLOEXEC);

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

    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_addopen(&fa, STDERR_FILENO, log.c_str(),
                                     O_WRONLY | O_CREAT | O_TRUNC, 0600);

    std::string port_str = std::to_string(port);
    const char *bin = MAC_SHELL_BIN;
    std::vector<char *> argv = {const_cast<char *>(bin),
                                const_cast<char *>("--port"),
                                const_cast<char *>(port_str.c_str()), nullptr};
    pid_t child = -1;
    int rc = posix_spawn(&child, bin, &fa, nullptr, argv.data(), envp.data());
    posix_spawn_file_actions_destroy(&fa);
    if (rc != 0) {
        std::fprintf(stderr, "posix_spawn(%s) failed: %s\n", bin,
                     std::strerror(rc));
        return 1;
    }

    // ---- it comes up, deaf, and says why ----
    ctl_client c;
    CHECK_MSG(c.connect_path(sock, 10000),
              "shell exited instead of coming up on a busy port");
    std::string stats = c.request("stats");
    CHECK_MSG(packet_rate_of(stats) == 0.0, stats.c_str());
    std::string logged = read_file(log);
    CHECK_MSG(logged.find("cannot bind UDP port") != std::string::npos,
              "the shell did not report the busy port");

    // ---- free it; the shell should rebind within a few retry ticks ----
    close(squatter);

    int udp = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in dst;
    std::memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons((uint16_t)port);
    dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    const uint8_t probe[8] = {0x01, 0, 0, 0, 0, 0, 0, 0};

    bool live = false;
    for (int i = 0; i < 60 && !live; i++) {  // ~6 s
        for (int k = 0; k < 6; k++)
            sendto(udp, probe, sizeof(probe), 0, (struct sockaddr *)&dst,
                   sizeof(dst));
        usleep(100 * 1000);
        stats = c.request("stats");
        live = packet_rate_of(stats) > 0.0;
    }
    CHECK_MSG(live, stats.c_str());

    close(udp);
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
    if (g_failures)
        std::fprintf(stderr, "--- shell log ---\n%s", read_file(log).c_str());
    unlink(sock.c_str());
    unlink(log.c_str());

    if (g_failures) {
        std::fprintf(stderr, "test_port_retry_e2e: %d FAILURES\n", g_failures);
        return 1;
    }
    std::printf("test_port_retry_e2e: all tests passed\n");
    return 0;
}
