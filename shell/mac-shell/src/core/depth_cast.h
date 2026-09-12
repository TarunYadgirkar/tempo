// depth_cast.h — ray/surface hit against the LiDAR depth map.
//
// The agent's "put it on the desk" resolution used to intersect the head ray
// with ARKit's plane list, which is sparse and lags the room (a desk can be
// missing entirely, a bed can come back classified vertical). The depth map
// has none of those gaps: every pixel is a metric reading of whatever is
// actually there.
//
// Everything here is plain inputs — a depth buffer, the intrinsics it was
// captured under and the camera pose at that instant — so it is testable with
// a synthetic image and no receiver, renderer or Metal device.

#pragma once

#include "spatial_bridge.h"

namespace mac_shell {

// One depth frame, already brought into the SCENE frame by the caller.
//
// `depth` is row-major metres measured ALONG THE OPTICAL AXIS (not range from
// the camera) — the same convention shaders.metal's occlusion pass compares
// against, and the one sb_depth_t carries; 0 means "no reading".
// `intr` is in pixels of intr.image_width x intr.image_height, which is
// ARKit's capture resolution and usually larger than the depth map; the two
// are related by a pure scale because the depth map is colour-aligned.
// `cam_quat` is the ARKit camera basis: +X right, +Y up, -Z the view
// direction.
struct depth_cast_input {
    const float *depth = nullptr;
    int width = 0;
    int height = 0;
    sb_intrinsics_t intr{};
    float cam_pos[3] = {0.0f, 0.0f, 0.0f};
    float cam_quat[4] = {0.0f, 0.0f, 0.0f, 1.0f};
};

struct depth_cast_result {
    float point[3] = {0.0f, 0.0f, 0.0f};
    float distance_m = 0.0f;
    float normal[3] = {0.0f, 0.0f, 0.0f};
};

// A sample this close to the ray counts as a hit outright, before the
// behind-the-surface crossing test gets a chance to fire.
constexpr float DEPTH_CAST_TOLERANCE_M = 0.03f;
// March granularity. 1 cm over the 8 m range is 800 samples — a few
// microseconds, and fine enough that a grazing ray cannot step over a surface
// it should have clipped.
constexpr float DEPTH_CAST_STEP_M = 0.01f;
constexpr float DEPTH_CAST_MIN_M = 0.05f;
constexpr float DEPTH_CAST_MAX_M = 8.0f;

// |ny| above this is a floor/desk/shelf, below DEPTH_CAST_VERTICAL_NY is a
// wall, and anything between is a slanted surface (a laptop lid, a ramp).
constexpr float DEPTH_CAST_HORIZONTAL_NY = 0.8f;
constexpr float DEPTH_CAST_VERTICAL_NY = 0.3f;

// March `origin` + `dir` (SCENE frame, metres; `dir` need not be unit) through
// the depth map and report the first surface it meets: the first sample it
// passes within DEPTH_CAST_TOLERANCE_M of, else the point where it crosses
// behind the depth surface. The normal comes from finite differences of the
// neighbouring depth samples, oriented back toward `origin`.
//
// False when the inputs are degenerate or the ray leaves the depth map's
// field of view without meeting anything.
bool depth_cast_ray(const depth_cast_input &in, const float origin[3],
                    const float dir[3], depth_cast_result &out);

// "horizontal" | "vertical" | "slanted" for a unit normal.
const char *surface_kind(const float normal[3]);

}  // namespace mac_shell
