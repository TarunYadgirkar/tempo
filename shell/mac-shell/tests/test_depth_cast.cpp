// test_depth_cast.cpp — depth_cast against a synthetic room: a floor at
// y = -0.8 and a wall at z = -2, seen from a camera at the origin looking
// along -Z. Pins the hit point, the estimated normal and the kind bucket the
// `cast` verb reports.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "core/depth_cast.h"

static int g_failures = 0;

#define CHECK(cond)                                                      \
    do {                                                                 \
        if (!(cond)) {                                                   \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, \
                         #cond);                                         \
            g_failures++;                                                \
        }                                                                \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                              \
    do {                                                                   \
        double va = (double)(a), vb = (double)(b);                         \
        if (!(std::fabs(va - vb) <= (eps))) {                              \
            std::fprintf(stderr, "FAIL %s:%d: %s (%g) ~= %s (%g)\n",       \
                         __FILE__, __LINE__, #a, va, #b, vb);              \
            g_failures++;                                                  \
        }                                                                  \
    } while (0)

namespace {

constexpr int W = 192, H = 144;
constexpr float FLOOR_Y = -0.8f;
constexpr float WALL_Z = -2.0f;

mac_shell::depth_cast_input make_room(std::vector<float> &buf) {
    mac_shell::depth_cast_input in;
    // Wide enough that a ray 39 deg below the axis is still inside the frame:
    // half-FOV tangents are 96/80 across and 72/80 down.
    in.intr.fx = 80.0f;
    in.intr.fy = 80.0f;
    in.intr.cx = (float)W * 0.5f;
    in.intr.cy = (float)H * 0.5f;
    in.intr.image_width = (float)W;
    in.intr.image_height = (float)H;

    // Camera at the origin, identity rotation: +X right, +Y up, -Z forward,
    // so camera space and scene space coincide and the expected numbers can
    // be read straight off the geometry.
    buf.assign((size_t)W * H, 0.0f);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            // Pixel ray, in camera space, scaled so its optical-axis
            // component is exactly 1 — the depth stored is then the metric
            // distance ALONG THE AXIS, matching sb_depth_t.
            float rx = ((float)x + 0.5f - in.intr.cx) / in.intr.fx;
            float ry = -((float)y + 0.5f - in.intr.cy) / in.intr.fy;
            float d_wall = -WALL_Z;
            float d = d_wall;
            if (ry < -1e-6f) {
                float d_floor = FLOOR_Y / ry;  // ry * d == FLOOR_Y
                if (d_floor < d)
                    d = d_floor;
            }
            (void)rx;
            buf[(size_t)y * W + (size_t)x] = d;
        }
    }
    in.depth = buf.data();
    in.width = W;
    in.height = H;
    return in;
}

void test_wall() {
    std::vector<float> buf;
    mac_shell::depth_cast_input in = make_room(buf);

    const float origin[3] = {0.0f, 0.0f, 0.0f};
    const float dir[3] = {0.0f, 0.0f, -1.0f};
    mac_shell::depth_cast_result r;
    CHECK(mac_shell::depth_cast_ray(in, origin, dir, r));
    CHECK_NEAR(r.point[0], 0.0f, 0.02f);
    CHECK_NEAR(r.point[1], 0.0f, 0.02f);
    CHECK_NEAR(r.point[2], WALL_Z, 0.02f);
    CHECK_NEAR(r.distance_m, 2.0f, 0.02f);
    // Faces the caster: +Z.
    CHECK_NEAR(r.normal[0], 0.0f, 0.05f);
    CHECK_NEAR(r.normal[1], 0.0f, 0.05f);
    CHECK_NEAR(r.normal[2], 1.0f, 0.05f);
    CHECK(std::string(mac_shell::surface_kind(r.normal)) == "vertical");
}

void test_floor() {
    std::vector<float> buf;
    mac_shell::depth_cast_input in = make_room(buf);

    // Down-and-forward: meets the floor at z = -1, well short of the wall.
    const float origin[3] = {0.0f, 0.0f, 0.0f};
    const float dir[3] = {0.0f, -0.8f, -1.0f};
    mac_shell::depth_cast_result r;
    CHECK(mac_shell::depth_cast_ray(in, origin, dir, r));
    CHECK_NEAR(r.point[0], 0.0f, 0.02f);
    CHECK_NEAR(r.point[1], FLOOR_Y, 0.02f);
    CHECK_NEAR(r.point[2], -1.0f, 0.03f);
    CHECK_NEAR(r.distance_m, std::sqrt(0.64f + 1.0f), 0.03f);
    CHECK_NEAR(r.normal[0], 0.0f, 0.05f);
    CHECK_NEAR(r.normal[1], 1.0f, 0.05f);
    CHECK_NEAR(r.normal[2], 0.0f, 0.05f);
    CHECK(std::string(mac_shell::surface_kind(r.normal)) == "horizontal");

    // Off to one side: the floor hit tracks x the same way.
    const float dir2[3] = {0.5f, -0.8f, -1.0f};
    mac_shell::depth_cast_result r2;
    CHECK(mac_shell::depth_cast_ray(in, origin, dir2, r2));
    CHECK_NEAR(r2.point[1], FLOOR_Y, 0.02f);
    CHECK_NEAR(r2.point[0], 0.5f, 0.03f);
    CHECK(std::string(mac_shell::surface_kind(r2.normal)) == "horizontal");
}

void test_no_hit() {
    std::vector<float> buf;
    mac_shell::depth_cast_input in = make_room(buf);
    mac_shell::depth_cast_result r;

    // Backwards: the depth map has nothing behind the camera.
    const float origin[3] = {0.0f, 0.0f, 0.0f};
    const float back[3] = {0.0f, 0.0f, 1.0f};
    CHECK(!mac_shell::depth_cast_ray(in, origin, back, r));

    // Straight up: every pixel it could project into is off the top of the
    // map, so there is no reading to meet.
    const float up[3] = {0.0f, 1.0f, 0.0f};
    CHECK(!mac_shell::depth_cast_ray(in, origin, up, r));

    // Degenerate direction.
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    CHECK(!mac_shell::depth_cast_ray(in, origin, zero, r));

    // No buffer at all.
    mac_shell::depth_cast_input empty = in;
    empty.depth = nullptr;
    const float fwd[3] = {0.0f, 0.0f, -1.0f};
    CHECK(!mac_shell::depth_cast_ray(empty, origin, fwd, r));
}

void test_offset_origin() {
    std::vector<float> buf;
    mac_shell::depth_cast_input in = make_room(buf);

    // A ray that does not start at the camera still lands on the surface:
    // the projection is done per marched point, not once for the origin.
    const float origin[3] = {0.3f, 0.2f, -0.5f};
    const float dir[3] = {0.0f, 0.0f, -1.0f};
    mac_shell::depth_cast_result r;
    CHECK(mac_shell::depth_cast_ray(in, origin, dir, r));
    CHECK_NEAR(r.point[2], WALL_Z, 0.03f);
    CHECK_NEAR(r.point[0], 0.3f, 0.001f);
    CHECK_NEAR(r.distance_m, 1.5f, 0.03f);
    CHECK(std::string(mac_shell::surface_kind(r.normal)) == "vertical");
}

void test_kind_buckets() {
    const float flat[3] = {0.0f, 1.0f, 0.0f};
    const float wall[3] = {1.0f, 0.0f, 0.0f};
    const float lid[3] = {0.0f, 0.5f, 0.866f};
    CHECK(std::string(mac_shell::surface_kind(flat)) == "horizontal");
    CHECK(std::string(mac_shell::surface_kind(wall)) == "vertical");
    CHECK(std::string(mac_shell::surface_kind(lid)) == "slanted");
    // Down-facing counts as horizontal too (a ceiling, or the underside of a
    // desk): the bucket is about the surface, not which way it faces.
    const float ceiling[3] = {0.0f, -1.0f, 0.0f};
    CHECK(std::string(mac_shell::surface_kind(ceiling)) == "horizontal");
}

}  // namespace

int main() {
    test_wall();
    test_floor();
    test_no_hit();
    test_offset_origin();
    test_kind_buckets();
    if (g_failures) {
        std::fprintf(stderr, "depth_cast: %d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("depth_cast: OK\n");
    return 0;
}
