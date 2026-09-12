// shot_path.cpp — see shot_path.h.

#include "platform/shot_path.h"

#include <chrono>
#include <cstdlib>
#include <vector>

#include <limits.h>
#include <stdlib.h>

namespace mac_shell {

namespace {

std::string realpath_of(const std::string &p) {
    char buf[PATH_MAX];
    if (!realpath(p.c_str(), buf))
        return {};
    return buf;
}

bool contained_in(const std::string &path, const std::string &root) {
    if (root.empty())
        return false;
    if (path == root)
        return true;
    return path.size() > root.size() && path.compare(0, root.size(), root) == 0 &&
           path[root.size()] == '/';
}

std::string env_root(const char *name) {
    const char *v = getenv(name);
    if (!v || !v[0])
        return {};
    return realpath_of(v);
}

}  // namespace

std::string default_shot_path() {
    const char *tmp = getenv("TMPDIR");
    if (!tmp || !tmp[0])
        return {};
    std::string dir = tmp;
    if (dir.back() != '/')
        dir += '/';
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count();
    return dir + "spatula-shot-" + std::to_string((long long)ms) + ".png";
}

bool resolve_shot_path(const std::string &in, std::string &out,
                       std::string &err) {
    if (in.empty() || in[0] != '/') {
        err = "not_absolute";
        return false;
    }
    size_t slash = in.find_last_of('/');
    std::string dir = slash == 0 ? "/" : in.substr(0, slash);
    std::string base = in.substr(slash + 1);
    if (base.empty() || base == "." || base == "..") {
        err = "bad_basename";
        return false;
    }
    std::string real_dir = realpath_of(dir);
    if (real_dir.empty()) {
        err = "no_such_dir";
        return false;
    }
    if (!contained_in(real_dir, env_root("TMPDIR")) &&
        !contained_in(real_dir, env_root("HOME"))) {
        err = "outside_allowed_roots";
        return false;
    }
    out = real_dir == "/" ? "/" + base : real_dir + "/" + base;
    return true;
}

}  // namespace mac_shell
