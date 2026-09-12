// vec_math.h — minimal float[3]/quat helpers for the mac-shell scene core.
// Mirrors the file-static helpers in vendor/wxrd/src/anchors.c so the ported
// anchor math produces bit-identical results without graphene.

#pragma once

#include <cmath>

namespace mac_shell {

// Rotate vector v by unit quaternion q = (x,y,z,w). Rodrigues form — copy of
// anchors.c:quat_rotate_vec so the port stays numerically identical.
inline void quat_rotate_vec(const float q[4], const float v[3], float out[3]) {
    float qx = q[0], qy = q[1], qz = q[2], qw = q[3];
    float t0 = qy * v[2] - qz * v[1] + qw * v[0];
    float t1 = qz * v[0] - qx * v[2] + qw * v[1];
    float t2 = qx * v[1] - qy * v[0] + qw * v[2];
    out[0] = v[0] + 2.0f * (qy * t2 - qz * t1);
    out[1] = v[1] + 2.0f * (qz * t0 - qx * t2);
    out[2] = v[2] + 2.0f * (qx * t1 - qy * t0);
}

inline float v3dot(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline void v3cross(const float a[3], const float b[3], float o[3]) {
    float x = a[1] * b[2] - a[2] * b[1];
    float y = a[2] * b[0] - a[0] * b[2];
    float z = a[0] * b[1] - a[1] * b[0];
    o[0] = x;
    o[1] = y;
    o[2] = z;
}

inline float v3length(const float v[3]) { return std::sqrt(v3dot(v, v)); }

inline bool v3finite(const float v[3]) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

// Degenerate (near-zero) input is left untouched, matching anchors.c;
// non-finite input collapses to +Z so a bad packet can never propagate NaN
// into a pose. Returns 0 in both cases so callers' `< eps` checks still fire.
inline float v3normalize(float v[3]) {
    float l = v3length(v);
    if (!std::isfinite(l)) {
        v[0] = 0.0f;
        v[1] = 0.0f;
        v[2] = 1.0f;
        return 0.0f;
    }
    if (l > 1e-6f) {
        v[0] /= l;
        v[1] /= l;
        v[2] /= l;
    }
    return l;
}

inline void v3sub(const float a[3], const float b[3], float o[3]) {
    o[0] = a[0] - b[0];
    o[1] = a[1] - b[1];
    o[2] = a[2] - b[2];
}

inline void v3copy(const float src[3], float dst[3]) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

// Orientation quaternion (xyzw) from the orthonormal row basis of a
// row-major, row-vector-convention 4x4 (rows = right/up/front/translation).
// Standard Shepperd branching on the largest diagonal term.
inline void quat_from_row_matrix(const float m[16], float out[4]) {
    float trace = m[0] + m[5] + m[10];
    if (trace > 0.0f) {
        float sq = std::sqrt(trace + 1.0f) * 2.0f;
        out[3] = 0.25f * sq;
        out[0] = (m[6] - m[9]) / sq;
        out[1] = (m[8] - m[2]) / sq;
        out[2] = (m[1] - m[4]) / sq;
    } else if (m[0] > m[5] && m[0] > m[10]) {
        float sq = std::sqrt(1.0f + m[0] - m[5] - m[10]) * 2.0f;
        out[3] = (m[6] - m[9]) / sq;
        out[0] = 0.25f * sq;
        out[1] = (m[4] + m[1]) / sq;
        out[2] = (m[8] + m[2]) / sq;
    } else if (m[5] > m[10]) {
        float sq = std::sqrt(1.0f + m[5] - m[0] - m[10]) * 2.0f;
        out[3] = (m[8] - m[2]) / sq;
        out[0] = (m[4] + m[1]) / sq;
        out[1] = 0.25f * sq;
        out[2] = (m[9] + m[6]) / sq;
    } else {
        float sq = std::sqrt(1.0f + m[10] - m[0] - m[5]) * 2.0f;
        out[3] = (m[1] - m[4]) / sq;
        out[0] = (m[8] + m[2]) / sq;
        out[1] = (m[9] + m[6]) / sq;
        out[2] = 0.25f * sq;
    }
}

// Yaw-only (gravity-aligned) form of an orientation: the rotation about +Y
// that preserves q's forward (-Z) heading, with pitch and roll dropped. Used
// to capture a world origin whose frame keeps +Y as world up, so every
// {0,1,0} constant downstream still means "up". Looking straight up/down
// collapses to identity, matching head_forward_horizontal's fallback.
inline void quat_yaw_only(const float q[4], float out[4]) {
    const float fwd_local[3] = {0.0f, 0.0f, -1.0f};
    float fwd[3];
    quat_rotate_vec(q, fwd_local, fwd);
    float h = std::sqrt(fwd[0] * fwd[0] + fwd[2] * fwd[2]);
    if (!(h > 1e-4f)) {
        out[0] = out[1] = out[2] = 0.0f;
        out[3] = 1.0f;
        return;
    }
    float yaw = std::atan2(-fwd[0], -fwd[2]);
    out[0] = 0.0f;
    out[1] = std::sin(yaw * 0.5f);
    out[2] = 0.0f;
    out[3] = std::cos(yaw * 0.5f);
}

// Hamilton product out = a * b (xyzw order).
inline void quat_mul(const float a[4], const float b[4], float out[4]) {
    float x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    float y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    float z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    float w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    out[0] = x;
    out[1] = y;
    out[2] = z;
    out[3] = w;
}

// Shortest-arc interpolation between two unit quaternions (xyzw). Falls back
// to a normalised lerp when the inputs are nearly parallel, where the sin()
// denominator loses all precision.
inline void quat_slerp(const float a[4], const float b[4], float t,
                       float out[4]) {
    float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    float sign = dot < 0.0f ? -1.0f : 1.0f;
    dot *= sign;
    float wa = 1.0f - t, wb = t;
    if (dot < 0.9995f) {
        float c = dot < -1.0f ? -1.0f : (dot > 1.0f ? 1.0f : dot);
        float theta = std::acos(c);
        float s = std::sin(theta);
        wa = std::sin((1.0f - t) * theta) / s;
        wb = std::sin(t * theta) / s;
    }
    wb *= sign;
    float q[4];
    for (int i = 0; i < 4; i++)
        q[i] = a[i] * wa + b[i] * wb;
    float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] +
                          q[3] * q[3]);
    if (!(len > 1e-8f)) {
        out[0] = out[1] = out[2] = 0.0f;
        out[3] = 1.0f;
        return;
    }
    for (int i = 0; i < 4; i++)
        out[i] = q[i] / len;
}

}  // namespace mac_shell
