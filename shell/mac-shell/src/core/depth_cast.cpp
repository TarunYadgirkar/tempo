#include "core/depth_cast.h"

#include <cmath>
#include <cstddef>

#include "core/vec_math.h"

namespace mac_shell {
namespace {

// Neighbour offset the normal's finite differences are taken over. Two pixels
// rides over single-sample LiDAR noise without smearing across a surface edge.
constexpr int NORMAL_STEP_PX = 2;
// How far a neighbour may sit from the centre sample before it counts as a
// different surface rather than a slope on this one. A fixed metric budget
// cannot work: NORMAL_STEP_PX subtends a fixed ANGLE, so the depth step a
// continuous surface produces grows with distance and with how steeply the
// surface is tilted away. The budget is therefore d * (k / f) — the lateral
// span those pixels cover at that range — times the steepest slope still
// treated as continuous, with a floor for LiDAR noise at close range.
constexpr float NORMAL_MAX_SLOPE = 6.0f;  // ~80 deg off perpendicular
constexpr float NORMAL_DISCONTINUITY_FLOOR_M = 0.03f;
// Bisection budget for the behind-the-surface crossing; 16 halvings take a
// 1 cm bracket under 0.2 micrometres.
constexpr int CROSSING_REFINE_STEPS = 16;

// Camera axes in the scene frame plus the intrinsics-pixel -> depth-pixel
// scale (the depth map is colour-aligned, so one scale covers both axes'
// resolutions).
struct cam_basis {
    float right[3], up[3], fwd[3];
    float sx = 1.0f, sy = 1.0f;
};

bool make_basis(const depth_cast_input &in, cam_basis &b) {
    if (!in.depth || in.width < 4 || in.height < 4)
        return false;
    const sb_intrinsics_t &k = in.intr;
    if (!(k.fx > 1.0f) || !(k.fy > 1.0f) || !(k.image_width > 1.0f) ||
        !(k.image_height > 1.0f))
        return false;
    const float ex[3] = {1.0f, 0.0f, 0.0f};
    const float ey[3] = {0.0f, 1.0f, 0.0f};
    const float ez[3] = {0.0f, 0.0f, -1.0f};
    quat_rotate_vec(in.cam_quat, ex, b.right);
    quat_rotate_vec(in.cam_quat, ey, b.up);
    quat_rotate_vec(in.cam_quat, ez, b.fwd);
    b.sx = (float)in.width / k.image_width;
    b.sy = (float)in.height / k.image_height;
    return v3finite(b.right) && v3finite(b.up) && v3finite(b.fwd);
}

// Scene point -> depth-map pixel, plus its distance along the optical axis.
// False when the point is behind the camera or outside the depth map.
bool project(const depth_cast_input &in, const cam_basis &b, const float p[3],
             float &u, float &v, float &z) {
    float rel[3];
    v3sub(p, in.cam_pos, rel);
    z = v3dot(rel, b.fwd);
    if (!(z > 1e-3f))
        return false;
    float x = v3dot(rel, b.right), y = v3dot(rel, b.up);
    u = (in.intr.cx + in.intr.fx * (x / z)) * b.sx;
    v = (in.intr.cy - in.intr.fy * (y / z)) * b.sy;  // image v runs downward
    if (!std::isfinite(u) || !std::isfinite(v))
        return false;
    return u >= 0.0f && v >= 0.0f && u < (float)in.width &&
           v < (float)in.height;
}

bool sample(const depth_cast_input &in, float u, float v, float &d) {
    int ix = (int)u, iy = (int)v;
    if (ix < 0 || iy < 0 || ix >= in.width || iy >= in.height)
        return false;
    d = in.depth[(size_t)iy * (size_t)in.width + (size_t)ix];
    return d > 0.0f && std::isfinite(d);
}

// Depth-map pixel + its reading -> scene point.
void unproject(const depth_cast_input &in, const cam_basis &b, float u, float v,
               float d, float out[3]) {
    float px = u / b.sx, py = v / b.sy;
    float x = (px - in.intr.cx) / in.intr.fx * d;
    float y = -(py - in.intr.cy) / in.intr.fy * d;
    for (int i = 0; i < 3; i++)
        out[i] = in.cam_pos[i] + x * b.right[i] + y * b.up[i] + d * b.fwd[i];
}

// Signed gap between the ray point at `t` and the depth surface behind that
// pixel: negative in front of the surface, positive behind it. False where the
// ray has no reading to compare against (off the map, or an invalid sample).
bool ray_gap(const depth_cast_input &in, const cam_basis &b,
             const float origin[3], const float dir[3], float t, float &gap) {
    float p[3];
    for (int i = 0; i < 3; i++)
        p[i] = origin[i] + t * dir[i];
    float u, v, z, d;
    if (!project(in, b, p, u, v, z) || !sample(in, u, v, d))
        return false;
    gap = z - d;
    return true;
}

// Cross product of the local finite differences, oriented so it points back
// toward `toward`. One-sided differences stand in where a neighbour is missing
// or sits on the far side of a depth discontinuity.
bool estimate_normal(const depth_cast_input &in, const cam_basis &b, float u,
                     float v, float d_center, const float toward[3],
                     float out[3]) {
    const float k = (float)NORMAL_STEP_PX;
    float center[3];
    unproject(in, b, u, v, d_center, center);

    // Focal length expressed in DEPTH-map pixels (the intrinsics are in the
    // larger colour image's pixels), so k / f_px is the angle the step spans.
    float f_px = std::fmin(in.intr.fx * b.sx, in.intr.fy * b.sy);
    float tol = NORMAL_DISCONTINUITY_FLOOR_M;
    if (f_px > 1e-3f)
        tol = std::fmax(tol, NORMAL_MAX_SLOPE * d_center * k / f_px);

    auto neighbour = [&](float du, float dv, float p[3]) {
        float d;
        if (!sample(in, u + du, v + dv, d))
            return false;
        if (std::fabs(d - d_center) > tol)
            return false;
        unproject(in, b, u + du, v + dv, d, p);
        return true;
    };

    // Each axis' difference always points toward increasing u / increasing v,
    // so the cross product's handedness does not depend on which neighbours
    // survived.
    auto axis = [&](float du, float dv, float o[3]) {
        float plus[3], minus[3];
        bool hp = neighbour(du, dv, plus), hm = neighbour(-du, -dv, minus);
        if (hp && hm)
            v3sub(plus, minus, o);
        else if (hp)
            v3sub(plus, center, o);
        else if (hm)
            v3sub(center, minus, o);
        else
            return false;
        return true;
    };

    float dx[3], dy[3];
    if (!axis(k, 0.0f, dx) || !axis(0.0f, k, dy))
        return false;
    v3cross(dx, dy, out);
    if (v3normalize(out) <= 1e-9f)
        return false;
    if (v3dot(out, toward) < 0.0f)
        for (int i = 0; i < 3; i++)
            out[i] = -out[i];
    return v3finite(out);
}

}  // namespace

bool depth_cast_ray(const depth_cast_input &in, const float origin[3],
                    const float dir[3], depth_cast_result &out) {
    cam_basis b;
    if (!make_basis(in, b) || !v3finite(origin) || !v3finite(dir))
        return false;
    float d0[3] = {dir[0], dir[1], dir[2]};
    if (v3normalize(d0) <= 1e-6f)
        return false;

    float hit_t = -1.0f;
    float prev_t = 0.0f, prev_gap = 0.0f;
    bool have_prev = false;
    for (float t = DEPTH_CAST_MIN_M; t <= DEPTH_CAST_MAX_M;
         t += DEPTH_CAST_STEP_M) {
        float gap;
        if (!ray_gap(in, b, origin, d0, t, gap)) {
            have_prev = false;
            continue;
        }
        if (std::fabs(gap) <= DEPTH_CAST_TOLERANCE_M) {
            hit_t = t;
            break;
        }
        if (have_prev && prev_gap < 0.0f && gap > 0.0f) {
            float lo = prev_t, hi = t;
            for (int i = 0; i < CROSSING_REFINE_STEPS; i++) {
                float mid = (lo + hi) * 0.5f, g;
                if (!ray_gap(in, b, origin, d0, mid, g))
                    break;
                if (g < 0.0f)
                    lo = mid;
                else
                    hi = mid;
            }
            hit_t = hi;
            break;
        }
        prev_t = t;
        prev_gap = gap;
        have_prev = true;
    }
    if (hit_t < 0.0f)
        return false;

    float p[3];
    for (int i = 0; i < 3; i++)
        p[i] = origin[i] + hit_t * d0[i];
    float u, v, z, d;
    if (!project(in, b, p, u, v, z) || !sample(in, u, v, d))
        return false;

    // Solve for the t where the ray's optical-axis depth equals the reading,
    // so the hit lands exactly on the surface AND exactly on the ray — the
    // march's 1 cm granularity would otherwise show up in the answer. Rays
    // running nearly perpendicular to the optical axis have no such solution
    // and keep the marched t.
    float rel[3];
    v3sub(origin, in.cam_pos, rel);
    float z0 = v3dot(rel, b.fwd), dz = v3dot(d0, b.fwd);
    float t_hit = hit_t;
    if (std::fabs(dz) > 1e-3f) {
        float t_solved = (d - z0) / dz;
        if (t_solved >= DEPTH_CAST_MIN_M && t_solved <= DEPTH_CAST_MAX_M)
            t_hit = t_solved;
    }
    for (int i = 0; i < 3; i++)
        out.point[i] = origin[i] + t_hit * d0[i];
    out.distance_m = t_hit;
    if (!v3finite(out.point))
        return false;

    float toward[3];
    v3sub(origin, out.point, toward);
    if (!estimate_normal(in, b, u, v, d, toward, out.normal)) {
        // No usable neighbourhood (a lone valid sample at a depth edge): the
        // surface still exists, so answer with the normal that faces the ray
        // rather than throwing the hit away.
        for (int i = 0; i < 3; i++)
            out.normal[i] = -d0[i];
    }
    return true;
}

const char *surface_kind(const float normal[3]) {
    float ny = std::fabs(normal[1]);
    if (ny > DEPTH_CAST_HORIZONTAL_NY)
        return "horizontal";
    if (ny < DEPTH_CAST_VERTICAL_NY)
        return "vertical";
    return "slanted";
}

}  // namespace mac_shell
