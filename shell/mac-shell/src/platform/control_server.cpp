// control_server.cpp — see control_server.h.

#include "platform/control_server.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>

#include "core/json_lite.h"
#include "core/layout_store.h"
#include "core/scene.h"
#include "platform/frame_grab.h"
#include "platform/shot_path.h"

namespace mac_shell {

namespace {
constexpr int POLL_TIMEOUT_MS = 20;  // also paces event broadcasts
// `layout load` waits this long for a relaunched app's capture stream to
// register its panel before giving up on that entry and counting it skipped.
constexpr int RELAUNCH_WAIT_MS = 300;
constexpr int RELAUNCH_POLL_MS = 25;

// Reads {"title":str,"body":str,"accent":bool} into a patch. Every key is
// optional here; the `note` verb is the one that insists on title + body.
bool parse_note_json(const std::string &text, scene::note_patch &out,
                     std::string &error) {
    json_value doc;
    if (!json_parse(text, doc, error))
        return false;
    if (!doc.is_object()) {
        error = "expected a json object";
        return false;
    }
    if (const json_value *v = doc.find("title")) {
        if (!v->is_string()) {
            error = "title must be a string";
            return false;
        }
        out.set_title = true;
        out.title = v->str;
    }
    if (const json_value *v = doc.find("body")) {
        if (!v->is_string()) {
            error = "body must be a string";
            return false;
        }
        out.set_body = true;
        out.body = v->str;
    }
    if (const json_value *v = doc.find("accent")) {
        if (v->type != json_value::kind::boolean) {
            error = "accent must be true or false";
            return false;
        }
        out.set_accent = true;
        out.accent = v->boolean;
    }
    return true;
}

// Splits "<sub> <rest>" — e.g. "save kitchen" — on the first run of spaces.
void split_first_word(const std::string &in, std::string &word,
                      std::string &rest) {
    size_t sp = in.find_first_of(" \t");
    if (sp == std::string::npos) {
        word = in;
        rest.clear();
        return;
    }
    word = in.substr(0, sp);
    size_t at = in.find_first_not_of(" \t", sp);
    rest = at == std::string::npos ? "" : in.substr(at);
}
}

std::string resolve_control_sock_path() {
    const char *env = getenv("SPATIAL_OS_SOCK");
    if (env && env[0])
        return env;
    const char *tmp = getenv("TMPDIR");
    if (tmp && tmp[0]) {
        std::string p = tmp;
        if (p.back() != '/')
            p += '/';
        return p + "spatial-os.sock";
    }
    // Deliberately no /tmp fallback: mirrors control.c — the control plane
    // launches panels and injects input, so a predictable world-traversable
    // path would let any local user drive it.
    return "";
}

namespace {
// True when something is still accepting on the unix socket at path.
bool socket_has_listener(const std::string &path) {
    struct sockaddr_un addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    if (path.size() >= sizeof(addr.sun_path))
        return false;
    std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0)
        return false;
    bool live = connect(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0;
    close(fd);
    return live;
}

// Removes a leftover socket at path. Never follows symlinks and never
// removes anything that isn't a socket, so a planted link or file at the
// path can't be turned into an unlink primitive — and never removes one
// that still answers, which would silently steal the control plane out
// from under a running Spatula.
bool clear_stale_socket(const std::string &path, std::string &error) {
    struct stat st;
    if (lstat(path.c_str(), &st) < 0) {
        if (errno == ENOENT)
            return true;
        error = "cannot stat the control socket at " + path;
        return false;
    }
    if (!S_ISSOCK(st.st_mode)) {
        std::fprintf(stderr,
                     "control: %s exists and is not a socket — refusing to "
                     "remove it\n",
                     path.c_str());
        error = path + " exists and is not a socket.";
        return false;
    }
    if (socket_has_listener(path)) {
        std::fprintf(stderr,
                     "control: control socket busy (another Spatula?): %s\n",
                     path.c_str());
        error = "Another Spatula already owns the control socket at " + path +
                ". Quit it, then relaunch.";
        return false;
    }
    if (unlink(path.c_str()) < 0 && errno != ENOENT) {
        std::fprintf(stderr, "control: unlink(%s) failed: %s\n", path.c_str(),
                     std::strerror(errno));
        error = "cannot remove the stale control socket at " + path;
        return false;
    }
    return true;
}
}  // namespace

struct control_server::conn {
    control_server *server = nullptr;
    int fd = -1;
    ctl_conn_t *cc = nullptr;
    bool want_write = false;
    bool closed = false;
};

control_server::control_server(scene &scene_ref) : scene_(scene_ref) {}

control_server::~control_server() { stop(); }

bool control_server::start() {
    error_.clear();
    sock_path_ = resolve_control_sock_path();
    if (sock_path_.empty()) {
        std::fprintf(stderr,
                     "control: SPATIAL_OS_SOCK and TMPDIR unset — refusing to "
                     "bind the control socket in a world-accessible location; "
                     "control plane disabled\n");
        error_ = "No per-user location for the control socket (neither "
                 "SPATIAL_OS_SOCK nor TMPDIR is set).";
        return false;
    }

    struct sockaddr_un addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    if (sock_path_.size() >= sizeof(addr.sun_path)) {
        std::fprintf(stderr, "control: socket path too long: %s\n",
                     sock_path_.c_str());
        return false;
    }
    std::strncpy(addr.sun_path, sock_path_.c_str(),
                 sizeof(addr.sun_path) - 1);

    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) {
        std::perror("control: socket");
        return false;
    }
    fcntl(fd, F_SETFL, O_NONBLOCK);
    fcntl(fd, F_SETFD, FD_CLOEXEC);

    if (!clear_stale_socket(sock_path_, error_)) {
        close(fd);
        return false;
    }

    // The socket inode is born owner-only (umask) so there is no window
    // between bind() and chmod() where another user could connect.
    mode_t old_umask = umask(077);
    int bind_rc = bind(fd, (struct sockaddr *)&addr, sizeof(addr));
    int bind_errno = errno;
    umask(old_umask);
    if (bind_rc < 0) {
        std::fprintf(stderr, "control: bind(%s) failed: %s\n",
                     sock_path_.c_str(), std::strerror(bind_errno));
        error_ = "Cannot bind the control socket at " + sock_path_ + ": " +
                 std::strerror(bind_errno);
        close(fd);
        return false;
    }
    if (chmod(sock_path_.c_str(), 0600) < 0)
        std::fprintf(stderr, "control: chmod 0600 on %s failed (continuing)\n",
                     sock_path_.c_str());
    if (listen(fd, 8) < 0) {
        std::perror("control: listen");
        close(fd);
        unlink(sock_path_.c_str());
        return false;
    }

    if (pipe(wake_pipe_) < 0) {
        std::perror("control: pipe");
        close(fd);
        unlink(sock_path_.c_str());
        return false;
    }
    fcntl(wake_pipe_[0], F_SETFL, O_NONBLOCK);

    listen_fd_ = fd;
    running_ = true;
    thread_ = std::thread(&control_server::run, this);
    std::fprintf(stderr, "control: listening on %s\n", sock_path_.c_str());
    return true;
}

void control_server::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (wake_pipe_[1] >= 0) {
        char b = 'q';
        (void)write(wake_pipe_[1], &b, 1);
    }
    if (thread_.joinable())
        thread_.join();
    for (auto *c : conns_) {
        if (c->cc)
            ctl_conn_destroy(c->cc);
        if (c->fd >= 0)
            close(c->fd);
        delete c;
    }
    conns_.clear();
    if (listen_fd_ >= 0) {
        close(listen_fd_);
        listen_fd_ = -1;
    }
    for (int i = 0; i < 2; i++) {
        if (wake_pipe_[i] >= 0) {
            close(wake_pipe_[i]);
            wake_pipe_[i] = -1;
        }
    }
    if (!sock_path_.empty())
        unlink(sock_path_.c_str());
}

// ------------------------------------------------------------------
// service loop
// ------------------------------------------------------------------

void control_server::run() {
    while (running_) {
        std::vector<struct pollfd> fds;
        fds.push_back({listen_fd_, POLLIN, 0});
        fds.push_back({wake_pipe_[0], POLLIN, 0});
        for (auto *c : conns_) {
            short ev = POLLIN;
            if (c->want_write)
                ev |= POLLOUT;
            fds.push_back({c->fd, ev, 0});
        }

        int rc = poll(fds.data(), (nfds_t)fds.size(), POLL_TIMEOUT_MS);
        if (!running_)
            break;
        if (rc < 0) {
            if (errno == EINTR)
                continue;
            std::perror("control: poll");
            break;
        }

        if (fds[0].revents & POLLIN)
            accept_conns();

        // conns_ indices line up with fds[2..]; handle_conn may mark closed.
        for (size_t i = 0; i < conns_.size() && i + 2 < fds.size(); i++) {
            if (fds[i + 2].revents)
                handle_conn(*conns_[i], fds[i + 2].revents);
        }
        // Sweep closed connections.
        for (size_t i = conns_.size(); i-- > 0;) {
            if (conns_[i]->closed)
                close_conn(i);
        }

        broadcast_events();
    }
}

void control_server::accept_conns() {
    while (true) {
        int cfd = accept(listen_fd_, nullptr, nullptr);
        if (cfd < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                break;
            if (errno == EINTR)
                continue;
            std::perror("control: accept");
            break;
        }
        fcntl(cfd, F_SETFL, O_NONBLOCK);
        fcntl(cfd, F_SETFD, FD_CLOEXEC);

        // Same-uid peer check (control.c uses SO_PEERCRED; macOS equivalent
        // is getpeereid). The control plane injects input — a different-uid
        // peer must never drive it.
        uid_t peer_uid = (uid_t)-1;
        gid_t peer_gid = (gid_t)-1;
        if (getpeereid(cfd, &peer_uid, &peer_gid) < 0 ||
            peer_uid != geteuid()) {
            std::fprintf(stderr, "control: rejecting peer uid %d (self %d)\n",
                         (int)peer_uid, (int)geteuid());
            close(cfd);
            continue;
        }

        conn *c = new conn();
        c->server = this;
        c->fd = cfd;
        c->cc = ctl_conn_create(&control_server::on_line_trampoline, c);
        if (!c->cc) {
            close(cfd);
            delete c;
            continue;
        }
        conns_.push_back(c);
    }
}

void control_server::handle_conn(conn &c, short revents) {
    if (revents & (POLLHUP | POLLERR | POLLNVAL)) {
        c.closed = true;
        return;
    }
    if (revents & POLLIN) {
        char buf[2048];
        while (true) {
            ssize_t r = read(c.fd, buf, sizeof(buf));
            if (r > 0) {
                ctl_conn_feed(c.cc, buf, (size_t)r);
                if ((size_t)r < sizeof(buf))
                    break;
                continue;
            }
            if (r == 0) {
                c.closed = true;
                return;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                break;
            c.closed = true;
            return;
        }
        if (ctl_conn_reply_overflow(c.cc)) {
            c.closed = true;
            return;
        }
    }
    flush_conn(c);
}

void control_server::flush_conn(conn &c) {
    const char *buf;
    size_t n;
    while ((n = ctl_conn_pending(c.cc, &buf)) > 0) {
        ssize_t w = write(c.fd, buf, n);
        if (w > 0) {
            ctl_conn_consumed(c.cc, (size_t)w);
            continue;
        }
        if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            c.want_write = true;
            return;
        }
        c.closed = true;  // EPIPE / peer gone
        return;
    }
    c.want_write = false;
}

void control_server::close_conn(size_t idx) {
    conn *c = conns_[idx];
    if (c->cc)
        ctl_conn_destroy(c->cc);
    if (c->fd >= 0)
        close(c->fd);
    delete c;
    conns_.erase(conns_.begin() + (ptrdiff_t)idx);
}

void control_server::broadcast_events() {
    std::vector<std::string> events = scene_.drain_events();
    if (events.empty())
        return;
    bool any_subscriber = false;
    for (auto *c : conns_) {
        if (ctl_conn_subscribed(c->cc)) {
            any_subscriber = true;
            break;
        }
    }
    if (!any_subscriber)
        return;
    for (auto *c : conns_) {
        if (!ctl_conn_subscribed(c->cc))
            continue;
        for (const auto &line : events)
            ctl_conn_queue_event(c->cc, line.c_str());
        flush_conn(*c);
    }
}

// ------------------------------------------------------------------
// `screenshot` — see control_server.h
// ------------------------------------------------------------------

bool control_server::handle_screenshot(conn &c, const char *line) {
    static const char VERB[] = "screenshot";
    constexpr size_t VERB_LEN = sizeof(VERB) - 1;
    if (std::strncmp(line, VERB, VERB_LEN) != 0)
        return false;
    if (line[VERB_LEN] != '\0' && line[VERB_LEN] != ' ' &&
        line[VERB_LEN] != '\t')
        return false;

    std::string arg = line + VERB_LEN;
    size_t first = arg.find_first_not_of(" \t");
    size_t last = arg.find_last_not_of(" \t");
    arg = first == std::string::npos ? "" : arg.substr(first, last - first + 1);

    char rbuf[512];
    auto reply_err = [&](const char *code, const char *detail) {
        ctl_reply_err(rbuf, sizeof(rbuf), code, detail);
        ctl_conn_queue(c.cc, rbuf);
    };

    // wxrd's form is `screenshot <handle> [region=…] [scale=…]`; ours adds a
    // bare/whole-scene form. A leading digit run is the handle form, and a
    // path is required to be absolute, so the two never collide.
    uint64_t handle = 0;
    bool is_handle = !arg.empty() && arg[0] >= '0' && arg[0] <= '9';
    if (is_handle) {
        size_t i = 0;
        while (i < arg.size() && arg[i] >= '0' && arg[i] <= '9')
            handle = handle * 10 + (uint64_t)(arg[i++] - '0');
        if (i != arg.size()) {
            reply_err("unsupported", "region= and scale= are not implemented");
            return true;
        }
    }

    std::string path = default_shot_path();
    if (!is_handle && !arg.empty()) {
        std::string why;
        if (!resolve_shot_path(arg, path, why)) {
            reply_err("bad_path", why.c_str());
            return true;
        }
    }
    if (path.empty()) {
        reply_err("bad_path", "no TMPDIR for the default screenshot path");
        return true;
    }

    std::string err;
    bool ok;
    if (is_handle) {
        if (!window_shot_) {
            reply_err("unsupported",
                      "per-window capture needs the ScreenCaptureKit build");
            return true;
        }
        ok = window_shot_(handle, path, err);
    } else {
        if (!screenshot_) {
            reply_err("unimplemented", "renderer not running");
            return true;
        }
        ok = screenshot_(path, err);
    }
    if (!ok) {
        // `timeout` is its own code so an agent can distinguish "the shell is
        // not presenting frames" from "that path was refused".
        if (err == "timeout")
            reply_err("timeout", "no frame presented within 1s");
        else
            reply_err("capture_failed", err.empty() ? nullptr : err.c_str());
        return true;
    }
    // Built by hand rather than ctl_reply_ok_kv: a PATH_MAX path does not fit
    // that helper's caller-supplied buffer.
    std::string ok_line = "ok path=" + path;
    ctl_conn_queue(c.cc, ok_line.c_str());
    return true;
}

uint64_t control_server::relaunch_captured(const layout_panel &saved) {
    if (!launch_app_ || saved.app_id.empty())
        return 0;
    std::vector<uint64_t> before = scene_.panel_handles();
    launch_app_(saved.app_id);
    // The capture layer registers the panel from its own stream callback, so
    // the new handle shows up a few ticks after launch_app_ returns.
    for (int waited = 0; waited <= RELAUNCH_WAIT_MS;
         waited += RELAUNCH_POLL_MS) {
        for (uint64_t h : scene_.panel_handles()) {
            if (std::find(before.begin(), before.end(), h) == before.end())
                return h;
        }
        usleep(RELAUNCH_POLL_MS * 1000);
    }
    return 0;
}

// ------------------------------------------------------------------
// verb dispatch (mirrors control.c against the mac-shell scene)
// ------------------------------------------------------------------

void control_server::on_line_trampoline(ctl_conn_t *cc, const char *line,
                                        void *user) {
    (void)cc;
    conn *c = static_cast<conn *>(user);
    c->server->on_line(*c, line);
}

void control_server::on_line(conn &c, const char *line) {
    if (line[0] == '\0')
        return;  // ignore blank keep-alive lines

    // Shared reply helpers for the mac-shell extension verbs below.
    auto queue = [&](const char *reply) { ctl_conn_queue(c.cc, reply); };
    auto queue_ok = [&]() {
        char rbuf[64];
        ctl_reply_ok(rbuf, sizeof(rbuf));
        queue(rbuf);
    };
    auto queue_err = [&](const char *code, const char *detail) {
        char rbuf[256];
        ctl_reply_err(rbuf, sizeof(rbuf), code, detail);
        queue(rbuf);
    };
    auto queue_ok_str = [&](const std::string &tail) {
        std::string r = "ok " + tail;
        queue(r.c_str());
    };

    // `screenshot` must be intercepted too: the vendored parser insists on a
    // handle, and the whole-scene form takes an optional path instead.
    if (handle_screenshot(c, line))
        return;

    // `launch-app` is a mac-shell extension the vendored parser doesn't know;
    // intercept it before ctl_request_parse.
    if (std::strncmp(line, "launch-app", 10) == 0 &&
        (line[10] == ' ' || line[10] == '\t')) {
        const char *target = line + 10;
        while (*target == ' ' || *target == '\t')
            target++;
        if (target[0] == '\0') {
            queue_err("parse_error", line);
        } else if (scene_.panel_count() >= MAX_PANELS) {
            queue_err("resource_exhausted", nullptr);
        } else if (!launch_app_) {
            queue_err("unimplemented", "renderer not running");
        } else {
            launch_app_(target);  // fallback handled inside the handler
            queue_ok();
        }
        return;
    }

    // `permissions` / `stats` are mac-shell extensions the vendored parser
    // doesn't know either; both are bare verbs.
    auto is_bare_verb = [&](const char *verb) {
        size_t n = std::strlen(verb);
        if (std::strncmp(line, verb, n) != 0)
            return false;
        for (const char *p = line + n; *p; p++)
            if (*p != ' ' && *p != '\t')
                return false;
        return true;
    };
    if (is_bare_verb("permissions")) {
        queue_ok_str(permissions_ ? permissions_()
                                  : "screen_capture=0 accessibility=0");
        return;
    }
    if (is_bare_verb("gather-panels")) {
        char rbuf[64];
        std::snprintf(rbuf, sizeof(rbuf), "ok gathered=%d",
                      scene_.gather_panels());
        queue(rbuf);
        return;
    }
    // `close-all` (alias `reset`): close every open panel — note, captured
    // window and internal test-card — in one shot. The launcher is not a
    // panel and stays up, so this is safe to fire from the launcher's own
    // commit. Replies `ok closed=<n>`; idempotent (0 when nothing is open).
    if (is_bare_verb("close-all") || is_bare_verb("reset")) {
        char rbuf[64];
        std::snprintf(rbuf, sizeof(rbuf), "ok closed=%d",
                      scene_.close_all_panels());
        queue(rbuf);
        return;
    }
    if (is_bare_verb("aim")) {
        queue_ok_str(scene_.aim_json());
        return;
    }
    if (is_bare_verb("cast")) {
        queue_ok_str(scene_.cast_json(nullptr, nullptr));
        return;
    }
    if (is_bare_verb("floor")) {
        queue_ok_str(scene_.floor_json());
        return;
    }
    if (is_bare_verb("stats")) {
        if (stats_)
            queue_ok_str(stats_());
        else
            queue_err("unimplemented", nullptr);
        return;
    }

    // `keyboard <show|hide|status>` and `launcher <show|hide|select N|commit|
    // status>` are mac-shell extensions too.
    auto sub_verb = [&](const char *verb, std::string &out_arg) {
        size_t n = std::strlen(verb);
        if (std::strncmp(line, verb, n) != 0 ||
            (line[n] != ' ' && line[n] != '\t'))
            return false;
        const char *p = line + n;
        while (*p == ' ' || *p == '\t')
            p++;
        out_arg = p;
        while (!out_arg.empty() &&
               (out_arg.back() == ' ' || out_arg.back() == '\t'))
            out_arg.pop_back();
        return true;
    };
    std::string arg;
    if (sub_verb("cast", arg)) {
        float o[3], d[3];
        char tail;
        if (std::sscanf(arg.c_str(), "%f %f %f %f %f %f %c", &o[0], &o[1],
                        &o[2], &d[0], &d[1], &d[2], &tail) != 6) {
            queue_err("parse_error", line);
            return;
        }
        queue_ok_str(scene_.cast_json(o, d));
        return;
    }
    if (sub_verb("pose", arg)) {
        unsigned long long handle = 0;
        float pos[3], quat[4];
        char tail;
        if (std::sscanf(arg.c_str(), "%llu %f %f %f %f %f %f %f %c", &handle,
                        &pos[0], &pos[1], &pos[2], &quat[0], &quat[1],
                        &quat[2], &quat[3], &tail) != 8) {
            queue_err("parse_error", line);
            return;
        }
        float len2 = quat[0] * quat[0] + quat[1] * quat[1] +
                     quat[2] * quat[2] + quat[3] * quat[3];
        if (!(len2 > 1e-8f) || !std::isfinite(len2)) {
            queue_err("bad_quat", "orientation must be a non-zero xyzw quaternion");
            return;
        }
        if (!scene_.pose_panel((uint64_t)handle, pos, quat))
            queue_err("no_such_window", nullptr);
        else
            queue_ok();
        return;
    }
    if (sub_verb("depth-occlusion", arg)) {
        if (arg == "off" || arg == "0")
            scene_.set_depth_occlusion_mode(0);
        else if (arg == "on" || arg == "1")
            scene_.set_depth_occlusion_mode(1);
        else if (arg == "soft")
            scene_.set_depth_occlusion_mode(2);
        else if (arg != "status") {
            queue_err("parse_error", line);
            return;
        }
        static const char *NAMES[3] = {"off", "on", "soft"};
        char rbuf[64];
        std::snprintf(rbuf, sizeof(rbuf), "ok mode=%s",
                      NAMES[scene_.depth_occlusion_mode()]);
        queue(rbuf);
        return;
    }
    if (sub_verb("hands", arg)) {
        if (arg == "dump") {
            queue_ok_str(scene_.hands_dump_json());
            return;
        }
        if (arg == "overlay on" || arg == "overlay 1")
            scene_.set_hand_overlay(true);
        else if (arg == "overlay off" || arg == "overlay 0")
            scene_.set_hand_overlay(false);
        else if (arg != "overlay status" && arg != "status") {
            queue_err("parse_error", line);
            return;
        }
        queue_ok_str(std::string("overlay=") +
                     (scene_.hand_overlay() ? "on" : "off") + " " +
                     scene_.hands_source_status());
        return;
    }
    if (sub_verb("hands-inject", arg)) {
        std::string err;
        if (scene_.hands_inject(arg, err))
            queue_ok();
        else
            queue_err("bad_json", err.c_str());
        return;
    }
    if (sub_verb("frame-export", arg)) {
        if (!frame_export_) {
            queue_err("unimplemented", "no frame source");
            return;
        }
        std::string reply, err;
        if (frame_export_(arg, reply, err))
            queue_ok_str(reply);
        else
            queue_err(err.empty() ? "parse_error" : err.c_str(), line);
        return;
    }
    if (sub_verb("keyboard", arg)) {
        if (arg == "show") {
            scene_.keyboard_show();
            queue_ok();
        } else if (arg == "hide") {
            scene_.keyboard_hide();
            queue_ok();
        } else if (arg == "status") {
            queue_ok_str(scene_.keyboard_status_json());
        } else {
            queue_err("parse_error", line);
        }
        return;
    }
    if (sub_verb("launcher", arg)) {
        if (arg == "show") {
            scene_.launcher_show();
            queue_ok();
        } else if (arg == "hide") {
            scene_.launcher_hide();
            queue_ok();
        } else if (arg == "commit") {
            if (scene_.launcher_commit())
                queue_ok();
            else
                queue_err("no_selection", "show + select before commit");
        } else if (arg == "status") {
            queue_ok_str(scene_.launcher_status_json());
        } else if (arg.rfind("select", 0) == 0) {
            const char *num = arg.c_str() + 6;
            char *end = nullptr;
            long idx = std::strtol(num, &end, 10);
            if (end == num || *end != '\0' ||
                !scene_.launcher_select((int)idx))
                queue_err("bad_index", arg.c_str());
            else
                queue_ok();
        } else {
            queue_err("parse_error", line);
        }
        return;
    }

    if (sub_verb("note-update", arg)) {
        std::string handle_str, json;
        split_first_word(arg, handle_str, json);
        char *end = nullptr;
        unsigned long long handle = std::strtoull(handle_str.c_str(), &end, 10);
        if (handle_str.empty() || !end || *end != '\0' || json.empty()) {
            queue_err("parse_error", line);
            return;
        }
        scene::note_patch patch;
        std::string why;
        if (!parse_note_json(json, patch, why)) {
            queue_err("bad_json", why.c_str());
        } else if (!patch.set_title && !patch.set_body && !patch.set_accent) {
            queue_err("bad_json", "no title, body or accent to update");
        } else if (!scene_.update_note((uint64_t)handle, patch)) {
            queue_err("no_such_window", nullptr);
        } else {
            queue_ok();
        }
        return;
    }
    if (sub_verb("note", arg)) {
        scene::note_patch patch;
        std::string why;
        if (!parse_note_json(arg, patch, why)) {
            queue_err("bad_json", why.c_str());
            return;
        }
        if (!patch.set_title || !patch.set_body) {
            queue_err("bad_json", "title and body are required strings");
            return;
        }
        uint64_t h = scene_.spawn_note_panel(patch.title, patch.body,
                                             patch.accent);
        if (h == 0) {
            queue_err("resource_exhausted", nullptr);
            return;
        }
        char rbuf[64];
        std::snprintf(rbuf, sizeof(rbuf), "ok handle=%llu",
                      (unsigned long long)h);
        queue(rbuf);
        return;
    }
    if (sub_verb("layout", arg)) {
        std::string sub, name;
        split_first_word(arg, sub, name);
        if (sub == "list") {
            std::string json = "{\"layouts\":[";
            bool first = true;
            for (const auto &n : layout_list()) {
                if (!first)
                    json += ",";
                first = false;
                json += "\"" + n + "\"";
            }
            queue_ok_str(json + "]}");
            return;
        }
        if (sub != "save" && sub != "load") {
            queue_err("parse_error", line);
            return;
        }
        if (!layout_name_valid(name)) {
            queue_err("bad_name", "use 1-40 of [A-Za-z0-9_-]");
            return;
        }
        std::string why;
        char rbuf[96];
        if (sub == "save") {
            std::vector<layout_panel> panels = scene_.capture_layout();
            if (!layout_save(name, panels, why)) {
                queue_err("io_error", why.c_str());
                return;
            }
            std::snprintf(rbuf, sizeof(rbuf), "ok saved=%zu", panels.size());
            queue(rbuf);
            return;
        }
        std::vector<layout_panel> panels;
        if (!layout_load(name, panels, why)) {
            queue_err(why == "not_found" ? "not_found" : "bad_layout",
                      why == "not_found" ? name.c_str() : why.c_str());
            return;
        }
        int restored = 0, skipped = 0;
        for (const auto &lp : panels) {
            uint64_t h = 0;
            if (lp.kind == "note")
                h = scene_.spawn_note_panel(lp.title, lp.body, lp.accent);
            else if (lp.kind == "captured")
                h = relaunch_captured(lp);
            else
                h = scene_.spawn_panel(
                    lp.app_id.empty() ? "test-card" : lp.app_id, lp.title);
            if (h != 0 && scene_.apply_layout_pose(h, lp))
                restored++;
            else
                skipped++;
        }
        std::snprintf(rbuf, sizeof(rbuf), "ok restored=%d skipped=%d",
                      restored, skipped);
        queue(rbuf);
        return;
    }

    ctl_request_t req;
    if (!ctl_request_parse(line, &req)) {
        queue_err("parse_error", line);
        return;
    }
    dispatch(c, req);
}

void control_server::dispatch(conn &c, const ctl_request_t &req) {
    char rbuf[512];

    auto reply = [&](const char *line) { ctl_conn_queue(c.cc, line); };
    auto reply_ok = [&]() {
        ctl_reply_ok(rbuf, sizeof(rbuf));
        reply(rbuf);
    };
    auto reply_err = [&](const char *code, const char *detail) {
        ctl_reply_err(rbuf, sizeof(rbuf), code, detail);
        reply(rbuf);
    };
    auto reply_ok_json = [&](const std::string &json) {
        std::string line = "ok " + json;
        reply(line.c_str());
    };

    switch (req.kind) {
        case CTL_REQ_VERSION:
            ctl_reply_ok_kv(rbuf, sizeof(rbuf), "proto", "1");
            reply(rbuf);
            return;

        case CTL_REQ_SUBSCRIBE:
            ctl_conn_set_subscribed(c.cc, true);
            reply_ok();
            return;

        case CTL_REQ_LIST_WINDOWS:
            reply_ok_json(scene_.windows_json(false));
            return;

        case CTL_REQ_LIST_PLANES:
            reply_ok_json(scene_.planes_json());
            return;

        case CTL_REQ_HEAD_POSE:
            reply_ok_json(scene_.head_pose_json());
            return;

        case CTL_REQ_DUMP_STATE:
            reply_ok_json(scene_.dump_state_json());
            return;

        case CTL_REQ_MOVE:
        case CTL_REQ_MOVE_REL:
            if (!scene_.move_panel(req.handle, req.vec,
                                   req.kind == CTL_REQ_MOVE_REL)) {
                reply_err("no_such_window", nullptr);
                return;
            }
            reply_ok();
            return;

        case CTL_REQ_ANCHOR: {
            int mode = req.anchor_mode == CTL_ANCHOR_UUID           ? 0
                       : req.anchor_mode == CTL_ANCHOR_CLOSEST_WALL ? 1
                                                                    : 2;
            uint8_t out_uuid[16];
            int rc = scene_.anchor_panel(req.handle, mode, req.anchor_uuid,
                                         out_uuid);
            if (rc < 0) {
                reply_err("no_such_window", nullptr);
            } else if (rc == 0) {
                ctl_reply_ok_no_anchor(rbuf, sizeof(rbuf));
                reply(rbuf);
            } else {
                ctl_reply_ok_anchor(rbuf, sizeof(rbuf), out_uuid);
                reply(rbuf);
            }
            return;
        }

        case CTL_REQ_CLEAR_ANCHOR:
            if (!scene_.clear_anchor(req.handle)) {
                reply_err("no_such_window", nullptr);
                return;
            }
            reply_ok();
            return;

        case CTL_REQ_QUERY: {  // legacy anchor-query
            uint8_t out_uuid[16];
            int rc = scene_.query_anchor(req.handle, out_uuid);
            if (rc < 0) {
                reply_err("no_such_window", nullptr);
            } else if (rc == 0) {
                ctl_reply_ok_no_anchor(rbuf, sizeof(rbuf));
                reply(rbuf);
            } else {
                ctl_reply_ok_anchor(rbuf, sizeof(rbuf), out_uuid);
                reply(rbuf);
            }
            return;
        }

        case CTL_REQ_FOCUS:
            if (!scene_.focus_panel(req.handle)) {
                reply_err("no_such_window", nullptr);
                return;
            }
            reply_ok();
            return;

        case CTL_REQ_CLOSE:
            if (!scene_.close_panel(req.handle)) {
                reply_err("no_such_window", nullptr);
                return;
            }
            reply_ok();
            return;

        case CTL_REQ_RESIZE:
            if (!scene_.resize_panel(req.handle, req.width, req.height)) {
                reply_err("no_such_window", nullptr);
                return;
            }
            reply_ok();
            return;

        case CTL_REQ_TYPE:
            if (!scene_.type_text(req.text)) {
                reply_err("no_focus", "focus a window before typing");
                return;
            }
            reply_ok();
            return;

        case CTL_REQ_KEY: {
            std::string name;
            if (req.keysym != 0) {
                char buf[16];
                std::snprintf(buf, sizeof(buf), "0x%x", req.keysym);
                name = buf;
            } else {
                name = req.text;
            }
            if (name.empty()) {
                reply_err("bad_keysym", req.text);
                return;
            }
            if (!scene_.key_press(name)) {
                reply_err("no_focus", "focus a window before sending keys");
                return;
            }
            reply_ok();
            return;
        }

        case CTL_REQ_SCROLL:
            scene_.scroll(req.scroll[0], req.scroll[1]);
            reply_ok();
            return;

        case CTL_REQ_CLICK:
            if (!scene_.click_panel(req.handle, req.click_xy[0],
                                    req.click_xy[1], req.button)) {
                reply_err("no_such_window", nullptr);
                return;
            }
            reply_ok();
            return;

        case CTL_REQ_LAUNCH: {
            // Stage 1: `launch` spawns an internal test-card panel named
            // after the command. Stage 2 adds `launch-app` semantics via
            // ScreenCaptureKit for real mac windows.
            if (scene_.spawn_panel("test-card", req.text) == 0) {
                reply_err("resource_exhausted", nullptr);
                return;
            }
            reply_ok();
            return;
        }

        // CTL_REQ_SCREENSHOT never lands here — handle_screenshot() answers
        // every `screenshot` line before the parser sees it.
        case CTL_REQ_THEME_RELOAD:
        case CTL_REQ_A11Y:
        case CTL_REQ_A11Y_ACTION:
            reply_err("unimplemented", nullptr);
            return;

        case CTL_REQ_UNKNOWN:
        default:
            reply_err("bad_request", nullptr);
            return;
    }
}

}  // namespace mac_shell
