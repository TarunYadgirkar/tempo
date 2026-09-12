// test_control_socket_busy.cpp — control_server must not steal a live
// control socket.
//
// clear_stale_socket() used to unlink whatever socket sat at the path, so a
// second Spatula silently took the control plane away from the running one
// (which then held a listen fd nobody could reach). Asserts the live case is
// refused with a message, the listener survives, and a genuinely stale
// socket file is still cleaned up.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "core/scene.h"
#include "platform/control_server.h"

static int g_failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,  \
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

static void fill_addr(struct sockaddr_un &addr, const std::string &path) {
    std::memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
}

static bool is_socket(const std::string &path) {
    struct stat st;
    return lstat(path.c_str(), &st) == 0 && S_ISSOCK(st.st_mode);
}

int main() {
    std::string dir = "/tmp";
    if (const char *t = getenv("TMPDIR"); t && t[0])
        dir = t;
    if (dir.back() == '/')
        dir.pop_back();
    std::string path =
        dir + "/mac-shell-busy-" + std::to_string((long)getpid()) + ".sock";
    unlink(path.c_str());
    setenv("SPATIAL_OS_SOCK", path.c_str(), 1);

    struct sockaddr_un addr;
    fill_addr(addr, path);

    // ---- a live listener at the path: start() must refuse ----
    int listener = socket(AF_UNIX, SOCK_STREAM, 0);
    CHECK(listener >= 0);
    CHECK(bind(listener, (struct sockaddr *)&addr, sizeof(addr)) == 0);
    CHECK(listen(listener, 4) == 0);

    mac_shell::scene world;
    {
        mac_shell::control_server server(world);
        CHECK_MSG(!server.start(),
                  "control_server bound over a live control socket");
        CHECK_MSG(server.last_error().find("Another Spatula") !=
                      std::string::npos,
                  server.last_error().c_str());
        server.stop();
    }
    CHECK_MSG(is_socket(path), "the live socket was unlinked");

    // The original listener still accepts — it was never stolen.
    int client = socket(AF_UNIX, SOCK_STREAM, 0);
    CHECK(client >= 0);
    CHECK_MSG(connect(client, (struct sockaddr *)&addr, sizeof(addr)) == 0,
              "the pre-existing listener stopped answering");
    close(client);

    // ---- same path, no listener: the stale file is cleaned up ----
    close(listener);
    CHECK_MSG(is_socket(path), "closing the listener removed the socket file");
    {
        mac_shell::control_server server(world);
        CHECK_MSG(server.start(), server.last_error().c_str());
        server.stop();
    }
    unlink(path.c_str());

    if (g_failures) {
        std::fprintf(stderr, "test_control_socket_busy: %d FAILURES\n",
                     g_failures);
        return 1;
    }
    std::printf("test_control_socket_busy: all tests passed\n");
    return 0;
}
