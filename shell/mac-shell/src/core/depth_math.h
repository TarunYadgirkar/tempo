// depth_math.h — pure math for LiDAR depth occlusion and intrinsics-correct
// passthrough sizing. Kept Metal-free so tests/test_depth_math.cpp can pin the
// numbers the shader in shaders.metal reproduces (keep both in sync).

#pragma once

#include <algorithm>
#include <cmath>

#include "spatial_bridge.h"

namespace mac_shell {

// Depth-buffer value the passthrough writes where no valid LiDAR reading
// exists (sb_depth_t depth==0 must never occlude) — just inside the far
// plane so panels always win.
constexpr float DEPTH_INVALID_ZBUF = 0.9999f;

// Metres → non-linear depth-buffer value under the renderer's reverse-less
// perspective projection (renderer.mm perspective_rh): zbuf(near)=0,
// zbuf(far)=1, monotonically increasing in between. d <= 0 is the sb_depth_t
// "no reading" sentinel.
inline float depth_to_zbuffer(float d_m, float near_z, float far_z) {
    if (d_m <= 0.0f || !(d_m == d_m))
        return DEPTH_INVALID_ZBUF;
    d_m = std::max(d_m, near_z);
    float zs = far_z / (near_z - far_z);
    float z = (zs * (near_z - d_m)) / d_m;
    return std::min(z, DEPTH_INVALID_ZBUF);
}

// UV transform placing the camera image at its true angular size inside the
// render frustum: uv_cam = 0.5 + (uv_screen - 0.5) * scale + offset. scale is
// the ratio of the render frustum's half-FOV tangents to the camera's
// (per-axis); offset accounts for the principal point being off-centre.
// Samples outside [0,1] are outside the camera's FOV (rendered black, never
// occluding). Falls back to identity when the intrinsics are degenerate.
inline void passthrough_uv_transform(const sb_intrinsics_t &intr,
                                     float tan_half_render_x,
                                     float tan_half_render_y, float out_scale[2],
                                     float out_offset[2]) {
    out_scale[0] = 1.0f;
    out_scale[1] = 1.0f;
    out_offset[0] = 0.0f;
    out_offset[1] = 0.0f;
    if (intr.fx <= 1.0f || intr.fy <= 1.0f || intr.image_width <= 1.0f ||
        intr.image_height <= 1.0f)
        return;
    float tan_cam_x = (intr.image_width * 0.5f) / intr.fx;
    float tan_cam_y = (intr.image_height * 0.5f) / intr.fy;
    if (tan_cam_x < 1e-4f || tan_cam_y < 1e-4f)
        return;
    out_scale[0] = tan_half_render_x / tan_cam_x;
    out_scale[1] = tan_half_render_y / tan_cam_y;
    out_offset[0] = (intr.cx - intr.image_width * 0.5f) / intr.image_width;
    out_offset[1] = (intr.cy - intr.image_height * 0.5f) / intr.image_height;
}

// ---------------------------------------------------------------------------
// Phone-orientation compensation. The ARKit sensor image is landscape; when
// the phone is held portrait the camera's roll about its optical axis (versus
// gravity) is ~±90°. We snap that roll to the nearest 90° bucket (with
// hysteresis so it can't flicker at the 45° boundaries), roll-correct the
// view matrix by the bucket angle, and rotate the passthrough UV mapping by
// the same angle so image and world content stay registered.
// ---------------------------------------------------------------------------

// Gravity-roll inputs: ux = dot(world_up, camera_right),
// uy = dot(world_up, camera_up) — world up projected into the camera's image
// plane. Roll angle a = atan2(ux, uy): 0 = landscape-native, ±90° = portrait,
// 180° = upside-down.
//
// Returns the orientation bucket b in {0,1,2,3} (roll snapped to b*90°).
// `prev_bucket` sticks unless the roll moves more than 45° + hysteresis_deg
// away from its centre; a near-vertical optical axis (gravity almost parallel
// to the view direction, |proj| tiny) also keeps the previous bucket.
inline int orientation_bucket_from_gravity(float ux, float uy, int prev_bucket,
                                           float hysteresis_deg = 12.0f) {
    if (prev_bucket < 0 || prev_bucket > 3)
        prev_bucket = 0;
    float mag2 = ux * ux + uy * uy;
    if (mag2 < 0.05f * 0.05f)  // camera pointing straight up/down
        return prev_bucket;
    float a_deg = std::atan2(ux, uy) * 57.29577951308232f;  // -180..180
    // Angular distance from the previous bucket's centre, wrapped to ±180.
    float d = a_deg - (float)prev_bucket * 90.0f;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    if (std::fabs(d) <= 45.0f + hysteresis_deg)
        return prev_bucket;
    int b = (int)std::lround(a_deg / 90.0f);
    return ((b % 4) + 4) % 4;
}

// Snapped roll angle for a bucket, radians.
inline float orientation_bucket_angle(int bucket) {
    return (float)bucket * 1.5707963267948966f;
}

// Rotation-aware generalisation of passthrough_uv_transform. Maps screen UV
// (u right, v down, roll-corrected render camera) to camera-image UV:
//   uv_cam = 0.5 + M * (uv_screen - 0.5) + offset
// with M row-major [m00 m01; m10 m11] in out_m = {m00, m01, m10, m11}.
// bucket = orientation_bucket_from_gravity result; bucket 0 reproduces the
// diagonal scale of passthrough_uv_transform exactly. The depth map is
// aligned to the colour image, so the SAME mapping must be used for both.
inline void passthrough_uv_mapping(const sb_intrinsics_t &intr,
                                   float tan_half_render_x,
                                   float tan_half_render_y, int bucket,
                                   float out_m[4], float out_offset[2]) {
    float scale[2];
    passthrough_uv_transform(intr, tan_half_render_x, tan_half_render_y, scale,
                             out_offset);
    // Ratios thx/tcx etc. recovered from the diagonal-scale fallback.
    float sx = scale[0], sy = scale[1];
    // tan_hy/tcx and tan_hx/tcy for the swapped (portrait) buckets.
    float sxy = sx, syx = sy;
    if (intr.fx > 1.0f && intr.fy > 1.0f && intr.image_width > 1.0f &&
        intr.image_height > 1.0f) {
        float tcx = (intr.image_width * 0.5f) / intr.fx;
        float tcy = (intr.image_height * 0.5f) / intr.fy;
        if (tcx >= 1e-4f && tcy >= 1e-4f) {
            sxy = tan_half_render_y / tcx;
            syx = tan_half_render_x / tcy;
        }
    }
    // cos/sin of bucket*90°, exact.
    static const float COS[4] = {1, 0, -1, 0};
    static const float SIN[4] = {0, 1, 0, -1};
    int b = ((bucket % 4) + 4) % 4;
    float ca = COS[b], sa = SIN[b];
    out_m[0] = ca * sx;   // m00: thx*cos/tcx
    out_m[1] = -sa * sxy; // m01: -thy*sin/tcx
    out_m[2] = sa * syx;  // m10: thx*sin/tcy
    out_m[3] = ca * sy;   // m11: thy*cos/tcy
}

// ---------------------------------------------------------------------------
// Soft depth occlusion. Instead of a hard z-buffer cutoff, occlusion fades
// over a depth band around the LiDAR reading: the passthrough writes its
// depth pushed back by band/2 (so partially-occluded panel fragments still
// rasterise), and the panel fragment attenuates alpha by this visibility
// factor. Keeps crawling edges from LiDAR noise soft instead of shimmering.
// ---------------------------------------------------------------------------

constexpr float OCCLUSION_SOFT_BAND_M = 0.05f;

// Fraction of the panel fragment that stays visible given its own metric
// depth and the LiDAR reading at the same pixel. band <= 0 → hard step.
// d_lidar <= 0 (no reading) never occludes.
inline float occlusion_visibility(float d_frag_m, float d_lidar_m,
                                  float band_m) {
    if (d_lidar_m <= 0.0f || !(d_lidar_m == d_lidar_m))
        return 1.0f;
    if (band_m <= 0.0f)
        return d_frag_m <= d_lidar_m ? 1.0f : 0.0f;
    float t = (d_frag_m - (d_lidar_m - band_m * 0.5f)) / band_m;
    return 1.0f - std::min(1.0f, std::max(0.0f, t));
}

// Inverse of depth_to_zbuffer (shader-side: panel fragment recovers its own
// metric depth from the interpolated depth-buffer value).
inline float zbuffer_to_depth(float z, float near_z, float far_z) {
    float zs = far_z / (near_z - far_z);
    return (zs * near_z) / (z + zs);
}

}  // namespace mac_shell
