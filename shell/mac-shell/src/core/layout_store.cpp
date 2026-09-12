// layout_store.cpp — see layout_store.h.

#include "core/layout_store.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include "core/anchor_math.h"
#include "core/json_lite.h"

namespace mac_shell {

namespace {

constexpr size_t NAME_MAX_LEN = 40;
// A layout is a few dozen small records; anything larger is not one of ours.
constexpr size_t FILE_MAX_BYTES = 1u << 20;

std::string num(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g", v);
    return buf;
}

std::string vec(const float *v, int n) {
    std::string s = "[";
    for (int i = 0; i < n; i++) {
        if (i)
            s += ",";
        s += num((double)v[i]);
    }
    return s + "]";
}

std::string quoted(const std::string &v) {
    std::string s = "\"";
    json_escape(v, s);
    return s + "\"";
}

bool mkdir_p(const std::string &path) {
    for (size_t i = 1; i <= path.size(); i++) {
        if (i != path.size() && path[i] != '/')
            continue;
        std::string part = path.substr(0, i);
        if (mkdir(part.c_str(), 0700) < 0 && errno != EEXIST)
            return false;
    }
    return true;
}

std::string file_path(const std::string &name) {
    std::string dir = layout_dir();
    return dir.empty() ? "" : dir + "/" + name + ".json";
}

bool read_file(const std::string &path, std::string &out) {
    FILE *f = std::fopen(path.c_str(), "rb");
    if (!f)
        return false;
    char buf[4096];
    size_t n;
    out.clear();
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) {
        out.append(buf, n);
        if (out.size() > FILE_MAX_BYTES) {
            std::fclose(f);
            return false;
        }
    }
    std::fclose(f);
    return true;
}

std::string str_field(const json_value &obj, const char *key) {
    const json_value *v = obj.find(key);
    return v && v->is_string() ? v->str : std::string();
}

}  // namespace

bool layout_name_valid(const std::string &name) {
    if (name.empty() || name.size() > NAME_MAX_LEN)
        return false;
    for (char c : name) {
        bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '-';
        if (!ok)
            return false;
    }
    return true;
}

std::string layout_dir() {
    const char *cfg = getenv("SPATIAL_OS_CONFIG_DIR");
    if (cfg && cfg[0])
        return std::string(cfg) + "/layouts";
    const char *home = getenv("HOME");
    if (home && home[0])
        return std::string(home) + "/.config/spatial-os/layouts";
    return "";
}

bool layout_save(const std::string &name,
                 const std::vector<layout_panel> &panels,
                 std::string &error) {
    std::string dir = layout_dir();
    if (dir.empty()) {
        error = "no HOME for the layout directory";
        return false;
    }
    if (!mkdir_p(dir)) {
        error = std::strerror(errno);
        return false;
    }

    std::string out = "{\"version\":1,\"panels\":[";
    for (size_t i = 0; i < panels.size(); i++) {
        const layout_panel &p = panels[i];
        if (i)
            out += ",";
        out += "{\"kind\":" + quoted(p.kind);
        out += ",\"app_id\":" + quoted(p.app_id);
        out += ",\"title\":" + quoted(p.title);
        if (p.kind == "note") {
            out += ",\"body\":" + quoted(p.body);
            out += std::string(",\"accent\":") + (p.accent ? "true" : "false");
        }
        out += ",\"pos\":" + vec(p.pos, 3);
        out += ",\"yaw\":" + num((double)p.yaw);
        float wh[2] = {(float)p.width_px, (float)p.height_px};
        out += ",\"size\":" + vec(wh, 2);
        out += ",\"width_m\":" + num((double)p.width_m);
        if (p.has_anchor) {
            out += ",\"anchor\":" + quoted(uuid_to_hex(p.anchor_uuid));
            out += ",\"anchor_offset\":" + vec(p.anchor_offset, 3);
        } else {
            out += ",\"anchor\":null";
        }
        out += "}";
    }
    out += "]}\n";

    // Write-then-rename: a crash mid-save leaves the previous layout intact
    // rather than a half-written file the loader would reject.
    std::string tmp = file_path(name) + ".tmp";
    FILE *f = std::fopen(tmp.c_str(), "wb");
    if (!f) {
        error = std::strerror(errno);
        return false;
    }
    bool ok = std::fwrite(out.data(), 1, out.size(), f) == out.size();
    ok = (std::fclose(f) == 0) && ok;
    if (!ok || rename(tmp.c_str(), file_path(name).c_str()) < 0) {
        error = std::strerror(errno);
        unlink(tmp.c_str());
        return false;
    }
    return true;
}

bool layout_load(const std::string &name, std::vector<layout_panel> &out,
                 std::string &error) {
    std::string path = file_path(name);
    std::string text;
    if (path.empty() || !read_file(path, text)) {
        error = "not_found";
        return false;
    }
    json_value doc;
    if (!json_parse(text, doc, error))
        return false;
    const json_value *panels = doc.find("panels");
    if (!panels || !panels->is_array()) {
        error = "no panels array";
        return false;
    }

    out.clear();
    for (const json_value &item : panels->items) {
        if (!item.is_object())
            continue;
        layout_panel p;
        p.kind = str_field(item, "kind");
        if (p.kind.empty())
            continue;
        p.app_id = str_field(item, "app_id");
        p.title = str_field(item, "title");
        p.body = str_field(item, "body");
        if (const json_value *a = item.find("accent"))
            p.accent = a->type == json_value::kind::boolean && a->boolean;
        if (const json_value *v = item.find("pos"))
            v->floats(p.pos, 3);
        if (const json_value *v = item.find("yaw"); v && v->is_number())
            p.yaw = (float)v->number;
        float wh[2] = {(float)p.width_px, (float)p.height_px};
        if (const json_value *v = item.find("size"); v && v->floats(wh, 2)) {
            p.width_px = (int)wh[0];
            p.height_px = (int)wh[1];
        }
        if (const json_value *v = item.find("width_m"); v && v->is_number())
            p.width_m = (float)v->number;
        if (const json_value *v = item.find("anchor"); v && v->is_string() &&
            parse_uuid_hex(v->str.c_str(), p.anchor_uuid)) {
            p.has_anchor = true;
            if (const json_value *o = item.find("anchor_offset"))
                o->floats(p.anchor_offset, 3);
        }
        out.push_back(std::move(p));
    }
    return true;
}

std::vector<std::string> layout_list() {
    std::vector<std::string> names;
    std::string dir = layout_dir();
    if (dir.empty())
        return names;
    DIR *d = opendir(dir.c_str());
    if (!d)
        return names;
    const std::string suffix = ".json";
    while (struct dirent *e = readdir(d)) {
        std::string n = e->d_name;
        if (n.size() <= suffix.size() ||
            n.compare(n.size() - suffix.size(), suffix.size(), suffix) != 0)
            continue;
        n.resize(n.size() - suffix.size());
        if (layout_name_valid(n))
            names.push_back(n);
    }
    closedir(d);
    std::sort(names.begin(), names.end());
    return names;
}

}  // namespace mac_shell
