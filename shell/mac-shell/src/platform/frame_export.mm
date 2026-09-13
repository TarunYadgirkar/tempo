// frame_export.mm — see frame_export.h.

#import <CoreGraphics/CoreGraphics.h>
#import <Foundation/Foundation.h>
#import <ImageIO/ImageIO.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#include <cerrno>
#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

#include "platform/frame_export.h"
#include "platform/shot_path.h"

#include "core/scene.h"

namespace mac_shell {
namespace {

// Visually lossless enough for landmark detection at a fraction of the bytes
// of a PNG; the tracker never sees the original JPEG anyway (the receiver
// decoded it), so this is a second generation either way.
constexpr float JPEG_QUALITY = 0.85f;

struct cf_guard {
    CFTypeRef ref = nullptr;
    ~cf_guard() {
        if (ref)
            CFRelease(ref);
    }
};

uint64_t steady_ns() {
    return (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool write_all(int fd, const uint8_t *p, size_t n) {
    while (n > 0) {
        ssize_t w = write(fd, p, n);
        if (w <= 0)
            return false;
        p += (size_t)w;
        n -= (size_t)w;
    }
    return true;
}

// Write `bytes` to <dir>/<name> by way of <dir>/.<name>.tmp + rename, so a
// reader polling the final name never sees a partial file. O_NOFOLLOW for the
// same reason png_write.mm uses it: a symlink planted at the temp name must
// be refused, not followed.
bool atomic_write(const std::string &dir, const char *name,
                  const uint8_t *bytes, size_t n) {
    const std::string final_path = dir + "/" + name;
    const std::string tmp_path = dir + "/." + name + ".tmp";
    ::unlink(tmp_path.c_str());
    int fd = ::open(tmp_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_NOFOLLOW,
                    0600);
    if (fd < 0)
        return false;
    bool ok = write_all(fd, bytes, n);
    ::close(fd);
    if (!ok || ::rename(tmp_path.c_str(), final_path.c_str()) != 0) {
        ::unlink(tmp_path.c_str());
        return false;
    }
    return true;
}

bool encode_jpeg(const uint8_t *rgba, int width, int height,
                 std::vector<uint8_t> &out) {
    cf_guard cs{CGColorSpaceCreateDeviceRGB()};
    if (!cs.ref)
        return false;
    const size_t row_bytes = (size_t)width * 4;
    cf_guard data{CFDataCreate(nullptr, rgba, (CFIndex)(row_bytes * (size_t)height))};
    if (!data.ref)
        return false;
    cf_guard provider{
        CGDataProviderCreateWithCFData((CFDataRef)data.ref)};
    if (!provider.ref)
        return false;
    cf_guard image{CGImageCreate(
        (size_t)width, (size_t)height, 8, 32, row_bytes,
        (CGColorSpaceRef)cs.ref,
        kCGBitmapByteOrderDefault | kCGImageAlphaNoneSkipLast,
        (CGDataProviderRef)provider.ref, nullptr, false,
        kCGRenderingIntentDefault)};
    if (!image.ref)
        return false;

    CFMutableDataRef sink = CFDataCreateMutable(nullptr, 0);
    if (!sink)
        return false;
    cf_guard sink_guard{sink};
    cf_guard dest{CGImageDestinationCreateWithData(
        sink, (__bridge CFStringRef)UTTypeJPEG.identifier, 1, nullptr)};
    if (!dest.ref)
        return false;
    NSDictionary *props = @{
        (__bridge NSString *)kCGImageDestinationLossyCompressionQuality :
            @(JPEG_QUALITY)
    };
    CGImageDestinationAddImage((CGImageDestinationRef)dest.ref,
                               (CGImageRef)image.ref,
                               (__bridge CFDictionaryRef)props);
    if (!CGImageDestinationFinalize((CGImageDestinationRef)dest.ref))
        return false;
    const uint8_t *p = CFDataGetBytePtr(sink);
    out.assign(p, p + CFDataGetLength(sink));
    return !out.empty();
}

void append_float(std::string &s, double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6g", v);
    s += buf;
}

}  // namespace

bool frame_export::enable(const std::string &dir, std::string &err) {
    if (dir.empty() || dir[0] != '/') {
        err = "not_absolute";
        return false;
    }
    // Create before resolving: the caller naming a fresh directory is the
    // normal case, and resolve_shot_path canonicalises the PARENT, which has
    // to exist for the containment check to mean anything.
    if (::mkdir(dir.c_str(), 0700) != 0 && errno != EEXIST) {
        err = "mkdir_failed";
        return false;
    }
    std::string resolved;
    if (!resolve_shot_path(dir + "/latest.json", resolved, err))
        return false;
    // resolved names the sidecar; the directory is its parent.
    const size_t slash = resolved.find_last_of('/');
    if (slash == std::string::npos || slash == 0) {
        err = "no_such_dir";
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    dir_ = resolved.substr(0, slash);
    frames_ = 0;
    last_write_ns_ = 0;
    warned_ = false;
    enabled_.store(true, std::memory_order_relaxed);
    return true;
}

void frame_export::disable() {
    enabled_.store(false, std::memory_order_relaxed);
}

std::string frame_export::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    char buf[PATH_MAX + 96];
    if (!enabled_.load(std::memory_order_relaxed)) {
        std::snprintf(buf, sizeof(buf), "off frames=%llu",
                      (unsigned long long)frames_);
        return buf;
    }
    std::snprintf(buf, sizeof(buf), "on dir=%s frames=%llu hz=%d",
                  dir_.c_str(), (unsigned long long)frames_,
                  FRAME_EXPORT_MAX_HZ);
    return buf;
}

void frame_export::offer_frame(const sb_frame_t &frame, const scene &world,
                               sb_receiver_t *rx) {
    if (!enabled_.load(std::memory_order_relaxed))
        return;
    if (!frame.rgba || frame.width == 0 || frame.height == 0)
        return;

    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_.load(std::memory_order_relaxed))
        return;

    const uint64_t now = steady_ns();
    const uint64_t min_gap = 1000000000ull / (uint64_t)FRAME_EXPORT_MAX_HZ;
    if (last_write_ns_ != 0 && now - last_write_ns_ < min_gap)
        return;

    std::vector<uint8_t> jpeg;
    if (!encode_jpeg(frame.rgba, (int)frame.width, (int)frame.height, jpeg)) {
        if (!warned_) {
            std::fprintf(stderr, "frame-export: JPEG encode failed\n");
            warned_ = true;
        }
        return;
    }

    scene::depth_snapshot depth;
    const bool have_depth = world.snapshot_depth(0, depth) &&
                            !depth.depth.empty() && depth.width > 0 &&
                            depth.height > 0;

    sb_intrinsics_t intr{};
    bool have_intr = rx && sb_get_latest_intrinsics(rx, &intr);
    if (!have_intr && have_depth && depth.have_intrinsics) {
        intr = depth.intr;
        have_intr = true;
    }

    float head_pos[3], head_quat[4];
    const bool have_head =
        world.head_pose_at(frame.timestamp_ns, head_pos, head_quat, nullptr);

    if (have_depth) {
        const uint8_t *bytes =
            reinterpret_cast<const uint8_t *>(depth.depth.data());
        if (!atomic_write(dir_, "latest.depth", bytes,
                          depth.depth.size() * sizeof(float)))
            return;
    }
    if (!atomic_write(dir_, "latest.jpg", jpeg.data(), jpeg.size()))
        return;

    std::string json = "{\"t_ns\":";
    json += std::to_string((unsigned long long)frame.timestamp_ns);
    json += ",\"seq\":";
    json += std::to_string((unsigned long long)(frames_ + 1));
    json += ",\"image\":{\"width\":" + std::to_string(frame.width);
    json += ",\"height\":" + std::to_string(frame.height);
    json += ",\"file\":\"latest.jpg\"}";

    json += ",\"head\":";
    if (have_head) {
        json += "{\"frame\":\"scene\",\"pos\":[";
        for (int i = 0; i < 3; i++) {
            if (i)
                json += ',';
            append_float(json, head_pos[i]);
        }
        json += "],\"quat\":[";
        for (int i = 0; i < 4; i++) {
            if (i)
                json += ',';
            append_float(json, head_quat[i]);
        }
        json += "]}";
    } else {
        json += "null";
    }

    json += ",\"intrinsics\":";
    if (have_intr) {
        json += "{\"fx\":";
        append_float(json, intr.fx);
        json += ",\"fy\":";
        append_float(json, intr.fy);
        json += ",\"cx\":";
        append_float(json, intr.cx);
        json += ",\"cy\":";
        append_float(json, intr.cy);
        json += ",\"image_width\":";
        append_float(json, intr.image_width);
        json += ",\"image_height\":";
        append_float(json, intr.image_height);
        json += '}';
    } else {
        json += "null";
    }

    json += ",\"depth\":";
    if (have_depth) {
        json += "{\"width\":" + std::to_string(depth.width);
        json += ",\"height\":" + std::to_string(depth.height);
        json += ",\"file\":\"latest.depth\",\"format\":\"float32\",\"t_ns\":";
        json += std::to_string((unsigned long long)depth.ts_ns);
        json += '}';
    } else {
        json += "null";
    }
    json += "}\n";

    if (!atomic_write(dir_, "latest.json",
                      reinterpret_cast<const uint8_t *>(json.data()),
                      json.size()))
        return;

    frames_++;
    last_write_ns_ = now;
    warned_ = false;
}

frame_export &frame_export_instance() {
    static frame_export instance;
    return instance;
}

bool frame_export_verb(frame_export &fx, const std::string &arg,
                       std::string &reply, std::string &err) {
    if (arg == "off") {
        fx.disable();
        reply = fx.status();
        return true;
    }
    if (arg == "status") {
        reply = fx.status();
        return true;
    }
    if (arg.rfind("on", 0) == 0 &&
        (arg.size() == 2 || arg[2] == ' ' || arg[2] == '\t')) {
        size_t i = 2;
        while (i < arg.size() && (arg[i] == ' ' || arg[i] == '\t'))
            i++;
        const std::string dir = arg.substr(i);
        if (dir.empty()) {
            err = "bad_path";
            return false;
        }
        if (!fx.enable(dir, err))
            return false;
        reply = fx.status();
        return true;
    }
    err = "parse_error";
    return false;
}

}  // namespace mac_shell
