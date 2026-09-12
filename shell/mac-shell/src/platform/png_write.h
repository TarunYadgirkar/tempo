// png_write.h — RGBA8 → PNG on disk, shared by the framebuffer grab
// (renderer.mm) and the per-panel grab (capture.mm).
//
// Encodes to memory and then writes through an O_NOFOLLOW descriptor, so a
// symlink planted at the target is refused instead of followed — the reason
// this is not just CGImageDestinationCreateWithURL.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace mac_shell {

// `rgba` is height*row_bytes of 8-bit RGBA (alpha ignored). On failure
// returns false with `err` set to a one-word reason.
bool write_rgba_png(const uint8_t *rgba, int width, int height,
                    size_t row_bytes, const std::string &path,
                    std::string &err);

}  // namespace mac_shell
