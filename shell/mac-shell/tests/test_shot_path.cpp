// test_shot_path.cpp — the `screenshot` verb's path policy (shot_path.cpp).
//
// The verb is a file-write primitive reachable from the control socket, so
// the containment rules are the security boundary: everything here is a
// refusal case except the two roots.

#include <cstdio>
#include <cstdlib>
#include <string>

#include <sys/stat.h>
#include <unistd.h>

#include <chrono>

#include "platform/frame_grab.h"
#include "platform/shot_path.h"

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

static std::string tmpdir() {
    const char *t = getenv("TMPDIR");
    std::string d = (t && t[0]) ? t : "/tmp";
    if (d.back() == '/')
        d.pop_back();
    return d;
}

int main() {
    using namespace mac_shell;

    std::string out, err;

    // ---- default path ----
    std::string def = default_shot_path();
    CHECK_MSG(def.rfind(tmpdir() + "/spatula-shot-", 0) == 0, def.c_str());
    CHECK(def.size() > 4 && def.compare(def.size() - 4, 4, ".png") == 0);
    // ---- accepted: inside $TMPDIR / $HOME ----
    CHECK_MSG(resolve_shot_path(tmpdir() + "/shot.png", out, err), err.c_str());
    CHECK_MSG(out.size() > 8 && out.compare(out.size() - 9, 9, "/shot.png") == 0,
              out.c_str());

    const char *home = getenv("HOME");
    if (home && home[0]) {
        CHECK_MSG(resolve_shot_path(std::string(home) + "/shot.png", out, err),
                  err.c_str());
    }

    // ---- refused ----
    CHECK(!resolve_shot_path("/etc/x.png", out, err));
    CHECK_MSG(err == "outside_allowed_roots", err.c_str());

    CHECK(!resolve_shot_path("/x.png", out, err));
    CHECK_MSG(err == "outside_allowed_roots", err.c_str());

    CHECK(!resolve_shot_path("shot.png", out, err));
    CHECK_MSG(err == "not_absolute", err.c_str());

    CHECK(!resolve_shot_path("", out, err));
    CHECK_MSG(err == "not_absolute", err.c_str());

    CHECK(!resolve_shot_path(tmpdir() + "/", out, err));
    CHECK_MSG(err == "bad_basename", err.c_str());

    CHECK(!resolve_shot_path(tmpdir() + "/no-such-dir-here/x.png", out, err));
    CHECK_MSG(err == "no_such_dir", err.c_str());

    // .. inside an allowed root still has to land inside one (the exact
    // refusal depends on how deep $TMPDIR sits: the climb may leave the roots
    // or land somewhere that doesn't exist).
    CHECK(!resolve_shot_path(tmpdir() + "/../../../etc/x.png", out, err));
    CHECK_MSG(err == "outside_allowed_roots" || err == "no_such_dir",
              err.c_str());

    // A symlinked directory cannot smuggle the target out of the roots:
    // containment is checked on the realpath of the parent.
    std::string link = tmpdir() + "/spatula-shot-escape-" +
                       std::to_string((long)getpid());
    if (symlink("/etc", link.c_str()) == 0) {
        CHECK(!resolve_shot_path(link + "/x.png", out, err));
        CHECK_MSG(err == "outside_allowed_roots", err.c_str());
        unlink(link.c_str());
    }

    // frame_grabber: a renderer that has stopped presenting must fail fast
    // (the control thread is single; a full timeout would stall every client).
    {
        mac_shell::frame_grabber g;
        std::string err;
        auto t0 = std::chrono::steady_clock::now();
        CHECK(!g.grab("/tmp/x.png", std::chrono::milliseconds(50), err));
        CHECK_MSG(err == "timeout", err.c_str());  // never drawn yet: waits
        g.take_pending();
        usleep((mac_shell::RENDERER_IDLE_MS + 50) * 1000);
        CHECK(!g.grab("/tmp/x.png", std::chrono::milliseconds(1000), err));
        CHECK_MSG(err == "renderer_idle", err.c_str());
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - t0)
                      .count();
        CHECK(ms < 900);  // the idle refusal did not wait out the timeout
    }

    if (g_failures == 0)
        std::printf("test_shot_path: all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}
