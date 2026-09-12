// png_write.mm — see png_write.h.

#import <CoreGraphics/CoreGraphics.h>
#import <Foundation/Foundation.h>
#import <ImageIO/ImageIO.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#include <fcntl.h>
#include <unistd.h>

#include "platform/png_write.h"

namespace mac_shell {

namespace {

struct cf_guard {
    CFTypeRef ref = nullptr;
    ~cf_guard() {
        if (ref)
            CFRelease(ref);
    }
};

struct fd_guard {
    int fd = -1;
    ~fd_guard() {
        if (fd >= 0)
            close(fd);
    }
};

bool write_all(int fd, const uint8_t *p, size_t n) {
    while (n > 0) {
        ssize_t w = write(fd, p, n);
        if (w <= 0)
            return false;
        p += w;
        n -= (size_t)w;
    }
    return true;
}

}  // namespace

bool write_rgba_png(const uint8_t *rgba, int width, int height,
                    size_t row_bytes, const std::string &path,
                    std::string &err) {
    if (!rgba || width <= 0 || height <= 0) {
        err = "empty_frame";
        return false;
    }

    CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
    cf_guard cs_guard{cs};
    CGContextRef ctx = CGBitmapContextCreate(
        const_cast<uint8_t *>(rgba), (size_t)width, (size_t)height, 8,
        row_bytes, cs,
        kCGImageAlphaNoneSkipLast | kCGBitmapByteOrderDefault);
    if (!ctx) {
        err = "bitmap_failed";
        return false;
    }
    cf_guard ctx_guard{ctx};
    CGImageRef img = CGBitmapContextCreateImage(ctx);
    if (!img) {
        err = "encode_failed";
        return false;
    }
    cf_guard img_guard{img};

    CFMutableDataRef data = CFDataCreateMutable(kCFAllocatorDefault, 0);
    if (!data) {
        err = "encode_failed";
        return false;
    }
    cf_guard data_guard{data};
    CGImageDestinationRef dst = CGImageDestinationCreateWithData(
        data, (__bridge CFStringRef)UTTypePNG.identifier, 1, nil);
    if (!dst) {
        err = "encode_failed";
        return false;
    }
    cf_guard dst_guard{dst};
    CGImageDestinationAddImage(dst, img, nil);
    if (!CGImageDestinationFinalize(dst)) {
        err = "encode_failed";
        return false;
    }

    fd_guard out{open(path.c_str(),
                      O_WRONLY | O_CREAT | O_TRUNC | O_NOFOLLOW | O_CLOEXEC,
                      0600)};
    if (out.fd < 0) {
        err = "open_failed";
        return false;
    }
    if (!write_all(out.fd, CFDataGetBytePtr(data),
                   (size_t)CFDataGetLength(data))) {
        err = "write_failed";
        return false;
    }
    return true;
}

}  // namespace mac_shell
