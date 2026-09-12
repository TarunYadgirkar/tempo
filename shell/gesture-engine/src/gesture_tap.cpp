// gesture_tap.cpp — Per-fingertip plane-tap detector.

#include "gesture_tap.h"
#include "ge_features.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <new>

namespace {

constexpr int kFingertipCount = 5;
constexpr int kHandCount      = 2;

const int kFingertipJoints[kFingertipCount] = {
    GE_JOINT_THUMB_TIP,
    GE_JOINT_INDEX_TIP,
    GE_JOINT_MIDDLE_TIP,
    GE_JOINT_RING_TIP,
    GE_JOINT_PINKY_TIP,
};

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float a, float b, float c) : x(a), y(b), z(c) {}
    Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
    float dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3 &o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    float length() const { return std::sqrt(dot(*this)); }
    Vec3 normalized() const {
        float l = length();
        return l > 1e-8f ? Vec3{x / l, y / l, z / l} : Vec3{};
    }
};

}  // anonymous namespace

struct ge_tap_listener_t {
    Vec3  plane_origin;
    Vec3  plane_normal;     // unit
    Vec3  plane_axis_x;     // unit, in-plane
    Vec3  plane_axis_y;     // unit, in-plane (= normal × axis_x)
    float slab_half_m;
    float lift_m;

    ge_tap_callback_t cb;
    void             *user_data;

    // Per-fingertip per-hand state.
    bool  has_prev[kHandCount][kFingertipCount];
    float prev_signed_dist[kHandCount][kFingertipCount];  // along normal
    bool  suppressed[kHandCount][kFingertipCount];

    ge_tap_listener_t() {
        std::memset(has_prev, 0, sizeof(has_prev));
        std::memset(prev_signed_dist, 0, sizeof(prev_signed_dist));
        std::memset(suppressed, 0, sizeof(suppressed));
    }
};

ge_tap_listener_t *ge_tap_listener_create(const float plane_origin[3],
                                          const float plane_normal[3],
                                          const float plane_axis_x[3],
                                          float slab_half_thickness_m,
                                          float lift_threshold_m,
                                          ge_tap_callback_t cb,
                                          void *user_data) {
    if (!plane_origin || !plane_normal || !plane_axis_x || !cb) return nullptr;

    auto *l = new (std::nothrow) ge_tap_listener_t();
    if (!l) return nullptr;

    l->plane_origin = {plane_origin[0], plane_origin[1], plane_origin[2]};
    l->plane_normal = Vec3{plane_normal[0], plane_normal[1], plane_normal[2]}.normalized();
    l->plane_axis_x = Vec3{plane_axis_x[0], plane_axis_x[1], plane_axis_x[2]}.normalized();
    l->plane_axis_y = l->plane_normal.cross(l->plane_axis_x).normalized();

    l->slab_half_m = slab_half_thickness_m > 0 ? slab_half_thickness_m : 0.005f;
    l->lift_m      = lift_threshold_m > 0      ? lift_threshold_m      : 0.01f;
    l->cb          = cb;
    l->user_data   = user_data;
    return l;
}

void ge_tap_listener_destroy(ge_tap_listener_t *l) {
    delete l;
}

void ge_tap_listener_update(ge_tap_listener_t *l,
                            const ge_hand_t hands[2],
                            float dt_s) {
    if (!l) return;
    (void)dt_s;  // velocity here is per-frame delta; absolute timing isn't
                 // needed for the reversal test.

    for (int h = 0; h < kHandCount; ++h) {
        if (!hands[h].present) {
            // Reset state for this hand so we don't fire spurious taps on
            // the next frame after re-acquisition.
            for (int f = 0; f < kFingertipCount; ++f) {
                l->has_prev[h][f]    = false;
                l->suppressed[h][f]  = false;
            }
            continue;
        }
        for (int f = 0; f < kFingertipCount; ++f) {
            int j = kFingertipJoints[f];
            // Skip low-confidence joints (engine policy).
            if (hands[h].joints[j][3] < 0.2f) {
                l->has_prev[h][f] = false;
                continue;
            }
            Vec3 tip{hands[h].joints[j][0],
                     hands[h].joints[j][1],
                     hands[h].joints[j][2]};
            Vec3 from_origin = tip - l->plane_origin;
            float d = from_origin.dot(l->plane_normal);   // signed distance

            // Suppression: a fingertip that just tapped must rise above the
            // lift threshold before it can tap again.
            if (l->suppressed[h][f]) {
                if (d > l->lift_m) {
                    l->suppressed[h][f] = false;
                }
                l->prev_signed_dist[h][f] = d;
                l->has_prev[h][f] = true;
                continue;
            }

            if (l->has_prev[h][f]) {
                float prev = l->prev_signed_dist[h][f];
                bool was_descending = (d < prev);

                // Tap criteria:
                //   1. The fingertip is currently inside the slab |d| <= slab_half.
                //   2. It was descending toward the plane on the previous step.
                //   3. (Optional sanity) the previous position was at or above
                //      the upper edge of the slab — i.e. the fingertip
                //      actually approached from above, not horizontally.
                bool in_slab = std::fabs(d) <= l->slab_half_m;
                bool came_from_above = prev >= -l->slab_half_m;
                if (in_slab && was_descending && came_from_above) {
                    // 2D contact in plane coordinates.
                    Vec3 in_plane = from_origin;  // includes the small normal component
                    float xy[2];
                    xy[0] = in_plane.dot(l->plane_axis_x);
                    xy[1] = in_plane.dot(l->plane_axis_y);
                    l->cb(h, f, xy, l->user_data);
                    l->suppressed[h][f] = true;
                }
            }
            l->prev_signed_dist[h][f] = d;
            l->has_prev[h][f] = true;
        }
    }
}
