// test_depth_math.cpp — pins the LiDAR occlusion depth conversion and the
// intrinsics-correct passthrough UV transform (src/core/depth_math.h; the shader in
// shaders.metal mirrors these formulas).

#include <cmath>
#include <cstdio>

#include "core/depth_math.h"

using namespace mac_shell;

static int g_failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, \
                         #cond);                                          \
            g_failures++;                                                 \
        }                                                                 \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                              \
    do {                                                                   \
        float va = (a), vb = (b);                                          \
        if (std::fabs(va - vb) > (eps)) {                                  \
            std::fprintf(stderr, "FAIL %s:%d: %s=%g != %s=%g\n", __FILE__, \
                         __LINE__, #a, (double)va, #b, (double)vb);        \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

int main() {
    const float near_z = 0.05f, far_z = 100.0f;

    // Endpoints of the projection: near plane → 0, far plane → clamped just
    // inside 1.
    CHECK_NEAR(depth_to_zbuffer(near_z, near_z, far_z), 0.0f, 1e-6f);
    CHECK_NEAR(depth_to_zbuffer(far_z, near_z, far_z), DEPTH_INVALID_ZBUF,
               2e-4f);

    // Monotonically increasing in metres.
    float prev = -1.0f;
    for (float d = near_z; d < 10.0f; d += 0.1f) {
        float z = depth_to_zbuffer(d, near_z, far_z);
        CHECK(z > prev);
        CHECK(z >= 0.0f && z <= DEPTH_INVALID_ZBUF);
        prev = z;
    }

    // The sentinel cases never occlude: invalid (0), negative, NaN, and
    // beyond-far all map to the far side of every panel.
    CHECK_NEAR(depth_to_zbuffer(0.0f, near_z, far_z), DEPTH_INVALID_ZBUF, 0);
    CHECK_NEAR(depth_to_zbuffer(-1.0f, near_z, far_z), DEPTH_INVALID_ZBUF, 0);
    CHECK_NEAR(depth_to_zbuffer(NAN, near_z, far_z), DEPTH_INVALID_ZBUF, 0);
    CHECK_NEAR(depth_to_zbuffer(500.0f, near_z, far_z), DEPTH_INVALID_ZBUF, 0);

    // A closer-than-near reading clamps to the near plane instead of blowing
    // past the depth range.
    CHECK_NEAR(depth_to_zbuffer(0.001f, near_z, far_z), 0.0f, 1e-6f);

    // Occlusion ordering: a real-world return at 1 m must occlude a panel at
    // 2 m and not one at 0.5 m (panel depth uses the same projection).
    float wall = depth_to_zbuffer(1.0f, near_z, far_z);
    CHECK(wall < depth_to_zbuffer(2.0f, near_z, far_z));
    CHECK(wall > depth_to_zbuffer(0.5f, near_z, far_z));

    // Intrinsics → UV transform. Camera with a 60° horizontal FOV rendered
    // through a frustum with the same tangents: identity scale, no offset.
    sb_intrinsics_t intr{};
    intr.image_width = 1920.0f;
    intr.image_height = 1440.0f;
    intr.fx = (intr.image_width * 0.5f) / std::tan(0.5235988f);  // 30° half
    intr.fy = intr.fx;
    intr.cx = intr.image_width * 0.5f;
    intr.cy = intr.image_height * 0.5f;
    float scale[2], offset[2];
    float tan_x = std::tan(0.5235988f);
    float tan_y = tan_x * (intr.image_height / intr.image_width);
    passthrough_uv_transform(intr, tan_x, tan_y, scale, offset);
    CHECK_NEAR(scale[0], 1.0f, 1e-4f);
    CHECK_NEAR(offset[0], 0.0f, 1e-6f);
    CHECK_NEAR(offset[1], 0.0f, 1e-6f);

    // A render frustum twice as wide as the camera sees the image at half the
    // screen: uv scale 2 (screen edges sample outside the camera image).
    passthrough_uv_transform(intr, 2.0f * tan_x, tan_y, scale, offset);
    CHECK_NEAR(scale[0], 2.0f, 1e-4f);
    CHECK_NEAR(scale[1], 1.0f, 1e-4f);

    // Principal point offset shifts the image.
    intr.cx = intr.image_width * 0.5f + 96.0f;  // +5% of width
    passthrough_uv_transform(intr, tan_x, tan_y, scale, offset);
    CHECK_NEAR(offset[0], 0.05f, 1e-4f);

    // Degenerate intrinsics fall back to identity.
    sb_intrinsics_t bad{};
    passthrough_uv_transform(bad, tan_x, tan_y, scale, offset);
    CHECK_NEAR(scale[0], 1.0f, 0);
    CHECK_NEAR(offset[0], 0.0f, 0);

    // -----------------------------------------------------------------
    // Phone-orientation buckets: gravity projected into the camera image
    // plane → snapped 90° roll, with hysteresis at the 45° boundaries.
    // -----------------------------------------------------------------

    // Cardinal orientations from any previous state.
    for (int prev = 0; prev < 4; prev++) {
        CHECK(orientation_bucket_from_gravity(0.0f, 1.0f, prev) == 0);
        CHECK(orientation_bucket_from_gravity(1.0f, 0.0f, prev) == 1);
        CHECK(orientation_bucket_from_gravity(0.0f, -1.0f, prev) == 2);
        CHECK(orientation_bucket_from_gravity(-1.0f, 0.0f, prev) == 3);
    }

    // Hysteresis: a roll just past 45° keeps the previous bucket; well past
    // the 45°+hys boundary it flips — and once flipped, coming back to just
    // under 45° does NOT flip back (no flicker at the boundary).
    {
        auto uxuy_at = [](float deg, float *ux, float *uy) {
            *ux = std::sin(deg * (float)M_PI / 180.0f);
            *uy = std::cos(deg * (float)M_PI / 180.0f);
        };
        float ux, uy;
        uxuy_at(50.0f, &ux, &uy);  // 45 < 50 < 45+12
        CHECK(orientation_bucket_from_gravity(ux, uy, 0) == 0);  // sticks
        uxuy_at(60.0f, &ux, &uy);  // beyond 57
        CHECK(orientation_bucket_from_gravity(ux, uy, 0) == 1);  // flips
        uxuy_at(40.0f, &ux, &uy);  // back under 45, still within 90∓57
        CHECK(orientation_bucket_from_gravity(ux, uy, 1) == 1);  // sticks
        uxuy_at(30.0f, &ux, &uy);  // beyond 90-57=33 → back to landscape
        CHECK(orientation_bucket_from_gravity(ux, uy, 1) == 0);
    }

    // Optical axis near-vertical (gravity ⟂ image plane): keep the previous
    // bucket rather than guessing from noise.
    CHECK(orientation_bucket_from_gravity(0.01f, 0.01f, 3) == 3);

    // Bucket angle helper.
    CHECK_NEAR(orientation_bucket_angle(0), 0.0f, 0);
    CHECK_NEAR(orientation_bucket_angle(1), (float)M_PI_2, 1e-6f);
    CHECK_NEAR(orientation_bucket_angle(2), (float)M_PI, 1e-6f);

    // -----------------------------------------------------------------
    // Rotated UV mapping: bucket 0 reproduces the diagonal scale; portrait
    // buckets swap axes with the cross tangent ratios; centre maps to
    // centre; the depth map (colour-aligned) shares the mapping by
    // construction.
    // -----------------------------------------------------------------
    {
        sb_intrinsics_t it{};
        it.image_width = 1920.0f;
        it.image_height = 1440.0f;
        it.fx = (it.image_width * 0.5f) / std::tan(0.5235988f);
        it.fy = it.fx;
        it.cx = it.image_width * 0.5f;
        it.cy = it.image_height * 0.5f;
        float thx = std::tan(0.5235988f);
        float thy = thx * 0.75f;
        float tcx = (it.image_width * 0.5f) / it.fx;
        float tcy = (it.image_height * 0.5f) / it.fy;

        float m[4], off[2], sc[2], off2[2];
        passthrough_uv_mapping(it, thx, thy, 0, m, off);
        passthrough_uv_transform(it, thx, thy, sc, off2);
        CHECK_NEAR(m[0], sc[0], 1e-6f);
        CHECK_NEAR(m[1], 0.0f, 1e-6f);
        CHECK_NEAR(m[2], 0.0f, 1e-6f);
        CHECK_NEAR(m[3], sc[1], 1e-6f);
        CHECK_NEAR(off[0], off2[0], 1e-6f);

        // Bucket 1 (90° roll): screen x samples along camera y and vice
        // versa, scaled by the cross ratios.
        passthrough_uv_mapping(it, thx, thy, 1, m, off);
        CHECK_NEAR(m[0], 0.0f, 1e-6f);
        CHECK_NEAR(m[1], -thy / tcx, 1e-5f);
        CHECK_NEAR(m[2], thx / tcy, 1e-5f);
        CHECK_NEAR(m[3], 0.0f, 1e-6f);

        // Bucket 2 (180°): negated diagonal.
        passthrough_uv_mapping(it, thx, thy, 2, m, off);
        CHECK_NEAR(m[0], -sc[0], 1e-5f);
        CHECK_NEAR(m[3], -sc[1], 1e-5f);

        // Screen centre always maps to the (principal-point-shifted) image
        // centre regardless of bucket.
        for (int b = 0; b < 4; b++) {
            passthrough_uv_mapping(it, thx, thy, b, m, off);
            float u = 0.5f + m[0] * 0.0f + m[1] * 0.0f + off[0];
            float v = 0.5f + m[2] * 0.0f + m[3] * 0.0f + off[1];
            CHECK_NEAR(u, 0.5f, 1e-6f);
            CHECK_NEAR(v, 0.5f, 1e-6f);
        }
    }

    // -----------------------------------------------------------------
    // Soft occlusion band + z-buffer inversion.
    // -----------------------------------------------------------------

    // Hard mode (band 0): step at the reading.
    CHECK_NEAR(occlusion_visibility(0.9f, 1.0f, 0.0f), 1.0f, 0);
    CHECK_NEAR(occlusion_visibility(1.1f, 1.0f, 0.0f), 0.0f, 0);
    // No reading never occludes.
    CHECK_NEAR(occlusion_visibility(5.0f, 0.0f, 0.05f), 1.0f, 0);
    CHECK_NEAR(occlusion_visibility(5.0f, -1.0f, 0.05f), 1.0f, 0);
    // Soft band: fully visible in front of the band, half at the reading,
    // gone past the far edge, monotonic in between.
    const float band = OCCLUSION_SOFT_BAND_M;
    CHECK_NEAR(occlusion_visibility(1.0f - band, 1.0f, band), 1.0f, 1e-6f);
    CHECK_NEAR(occlusion_visibility(1.0f, 1.0f, band), 0.5f, 1e-6f);
    CHECK_NEAR(occlusion_visibility(1.0f + band, 1.0f, band), 0.0f, 1e-6f);
    {
        float prev_v = 2.0f;
        for (float d = 0.9f; d < 1.1f; d += 0.005f) {
            float v = occlusion_visibility(d, 1.0f, band);
            CHECK(v <= prev_v);
            prev_v = v;
        }
    }

    // zbuffer_to_depth inverts depth_to_zbuffer across the range.
    for (float d = 0.1f; d < 5.0f; d += 0.13f) {
        float z = depth_to_zbuffer(d, near_z, far_z);
        CHECK_NEAR(zbuffer_to_depth(z, near_z, far_z), d, 1e-2f);
    }

    if (g_failures) {
        std::fprintf(stderr, "test_depth_math: %d FAILURES\n", g_failures);
        return 1;
    }
    std::printf("test_depth_math: all tests passed\n");
    return 0;
}
