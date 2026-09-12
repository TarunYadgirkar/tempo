// hand_reconstruct.c — Bone-constrained hand joint depth reconstruction.
//
// Ports the Python algorithm from scripts/reconstruct_mediapipe.py to C.
// Phase 1: Ray-sphere intersection (thumb cascade + temporal, finger proximity)
// Phase 2: MCP spacing validation (16-combo enumeration)
// Phase 3: Finger coplanarity + angle limits, thumb angle validation
// Phase 4: Inter-finger collision avoidance

#include "hand_reconstruct.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ── Vec3 helpers ──

typedef float v3[3];

static inline void v3_copy(v3 dst, const v3 src) { dst[0]=src[0]; dst[1]=src[1]; dst[2]=src[2]; }
static inline void v3_sub(v3 r, const v3 a, const v3 b) { r[0]=a[0]-b[0]; r[1]=a[1]-b[1]; r[2]=a[2]-b[2]; }
static inline void v3_add(v3 r, const v3 a, const v3 b) { r[0]=a[0]+b[0]; r[1]=a[1]+b[1]; r[2]=a[2]+b[2]; }
static inline void v3_scale(v3 r, const v3 a, float s) { r[0]=a[0]*s; r[1]=a[1]*s; r[2]=a[2]*s; }
static inline void v3_mad(v3 r, const v3 a, const v3 b, float s) { r[0]=a[0]+b[0]*s; r[1]=a[1]+b[1]*s; r[2]=a[2]+b[2]*s; }
static inline float v3_dot(const v3 a, const v3 b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
static inline float v3_len(const v3 a) { return sqrtf(v3_dot(a,a)); }
static inline float v3_dist(const v3 a, const v3 b) { v3 d; v3_sub(d,a,b); return v3_len(d); }
static inline void v3_cross(v3 r, const v3 a, const v3 b) {
    r[0]=a[1]*b[2]-a[2]*b[1]; r[1]=a[2]*b[0]-a[0]*b[2]; r[2]=a[0]*b[1]-a[1]*b[0];
}
static inline int v3_normalize(v3 r, const v3 a) {
    float l = v3_len(a);
    if (l < 1e-8f) return 0;
    v3_scale(r, a, 1.0f/l);
    return 1;
}

// ── 3x3 rotation matrix (row-major) ──

typedef float mat3[9]; // [row*3+col]

static void quat_to_mat3(mat3 m, const float q[4]) {
    float x=q[0], y=q[1], z=q[2], w=q[3];
    m[0]=1-2*(y*y+z*z); m[1]=2*(x*y-w*z);   m[2]=2*(x*z+w*y);
    m[3]=2*(x*y+w*z);   m[4]=1-2*(x*x+z*z); m[5]=2*(y*z-w*x);
    m[6]=2*(x*z-w*y);   m[7]=2*(y*z+w*x);   m[8]=1-2*(x*x+y*y);
}

static void mat3_mul_v3(v3 r, const mat3 m, const v3 v) {
    r[0]=m[0]*v[0]+m[1]*v[1]+m[2]*v[2];
    r[1]=m[3]*v[0]+m[4]*v[1]+m[5]*v[2];
    r[2]=m[6]*v[0]+m[7]*v[1]+m[8]*v[2];
}

// R^T * v (transpose multiply — camera-from-world transform)
static void mat3T_mul_v3(v3 r, const mat3 m, const v3 v) {
    r[0]=m[0]*v[0]+m[3]*v[1]+m[6]*v[2];
    r[1]=m[1]*v[0]+m[4]*v[1]+m[7]*v[2];
    r[2]=m[2]*v[0]+m[5]*v[1]+m[8]*v[2];
}

// ── Constants ──

#define EPS 1e-8f
#define GAP_THRESHOLD_NS 150000000ULL

// Joint indices (same as spatial_bridge.h sb_hand_joint)
enum {
    J_WRIST=0,
    J_THUMB_CMC=1, J_THUMB_MCP=2, J_THUMB_IP=3, J_THUMB_TIP=4,
    J_INDEX_MCP=5, J_INDEX_PIP=6, J_INDEX_DIP=7, J_INDEX_TIP=8,
    J_MIDDLE_MCP=9, J_MIDDLE_PIP=10, J_MIDDLE_DIP=11, J_MIDDLE_TIP=12,
    J_RING_MCP=13, J_RING_PIP=14, J_RING_DIP=15, J_RING_TIP=16,
    J_PINKY_MCP=17, J_PINKY_PIP=18, J_PINKY_DIP=19, J_PINKY_TIP=20,
};

// Bone lengths (meters, dataset medians)
typedef struct { int parent; int child; float len; } bone_t;
static const bone_t BONES[] = {
    {0,1,0.0403f},{1,2,0.0391f},{2,3,0.0352f},{3,4,0.0302f},
    {0,5,0.0768f},{5,6,0.0424f},{6,7,0.0232f},{7,8,0.0213f},
    {0,9,0.0737f},{9,10,0.0472f},{10,11,0.0277f},{11,12,0.0230f},
    {0,13,0.0719f},{13,14,0.0441f},{14,15,0.0265f},{15,16,0.0182f},
    {0,17,0.0722f},{17,18,0.0328f},{18,19,0.0216f},{19,20,0.0156f},
};
#define BONE_COUNT 20

static float get_bone_len(int parent, int child) {
    for (int i = 0; i < BONE_COUNT; i++)
        if (BONES[i].parent == parent && BONES[i].child == child)
            return BONES[i].len;
    return 0.04f;
}

// Kinematic chains: [chain][segment] = {parent, child}
typedef struct { int p, c; } seg_t;
static const seg_t THUMB_CHAIN[] = {{0,1},{1,2},{2,3},{3,4}};
static const seg_t INDEX_CHAIN[] = {{0,5},{5,6},{6,7},{7,8}};
static const seg_t MIDDLE_CHAIN[] = {{0,9},{9,10},{10,11},{11,12}};
static const seg_t RING_CHAIN[] = {{0,13},{13,14},{14,15},{15,16}};
static const seg_t PINKY_CHAIN[] = {{0,17},{17,18},{18,19},{19,20}};
static const seg_t *FINGER_CHAINS[] = {INDEX_CHAIN, MIDDLE_CHAIN, RING_CHAIN, PINKY_CHAIN};

static const int MCP_INDICES[] = {5, 9, 13, 17};
static const int MCP_ADJ[][2] = {{5,9},{9,13},{13,17}};
#define MCP_MIN_SPACING 0.003f
#define MCP_MAX_SPACING 0.040f

#define THUMB_IP_CASCADE_LIMIT 1.0472f // 60 degrees in radians

// ── Temporal state ──

struct hr_state_t {
    float bone_dir[4][3]; // thumb joints 1-4, world-space bone direction
    uint64_t last_ts;
    int valid;
};

hr_state_t *hr_state_create(void) {
    hr_state_t *s = (hr_state_t *)calloc(1, sizeof(hr_state_t));
    return s;
}

void hr_state_destroy(hr_state_t *s) {
    free(s);
}

// ── Ray-sphere intersection ──

// Reconstruct child using bone constraint + proximity disambiguation.
// child_raw, parent: camera space. Returns result in camera space.
static void ray_sphere_proximity(v3 result, const v3 child_raw, const v3 parent, float bone) {
    float ray_len = v3_len(child_raw);
    if (ray_len < EPS) { v3_copy(result, parent); return; }

    v3 ray; v3_scale(ray, child_raw, 1.0f/ray_len);
    float b_coeff = -2.0f * v3_dot(ray, parent);
    float c_coeff = v3_dot(parent, parent) - bone * bone;
    float disc = b_coeff * b_coeff - 4.0f * c_coeff;

    if (disc < 0) {
        float t_closest = v3_dot(ray, parent);
        v3 closest; v3_scale(closest, ray, t_closest);
        v3 dir; v3_sub(dir, closest, parent);
        if (!v3_normalize(dir, dir)) v3_copy(dir, ray);
        v3_mad(result, parent, dir, bone);
        return;
    }

    float sd = sqrtf(disc);
    float t1 = (-b_coeff - sd) * 0.5f;
    float t2 = (-b_coeff + sd) * 0.5f;
    v3 sol_a, sol_b;
    v3_scale(sol_a, ray, t1);
    v3_scale(sol_b, ray, t2);

    int va = -sol_a[2] > 0.01f;
    int vb = -sol_b[2] > 0.01f;
    if (va && !vb) { v3_copy(result, sol_a); return; }
    if (vb && !va) { v3_copy(result, sol_b); return; }
    if (!va && !vb) { v3_copy(result, parent); return; }

    float da = v3_dist(sol_a, child_raw);
    float db = v3_dist(sol_b, child_raw);
    v3_copy(result, da <= db ? sol_a : sol_b);
}

// Compute both ray-sphere solutions (closer first).
static void ray_sphere_both(v3 sol_a, v3 sol_b, const v3 child_raw, const v3 parent, float bone) {
    float ray_len = v3_len(child_raw);
    if (ray_len < EPS) { v3_copy(sol_a, parent); v3_copy(sol_b, parent); return; }

    v3 ray; v3_scale(ray, child_raw, 1.0f/ray_len);
    float b_coeff = -2.0f * v3_dot(ray, parent);
    float c_coeff = v3_dot(parent, parent) - bone * bone;
    float disc = b_coeff * b_coeff - 4.0f * c_coeff;

    if (disc < 0) {
        float t_closest = v3_dot(ray, parent);
        v3 closest; v3_scale(closest, ray, t_closest);
        v3 dir; v3_sub(dir, closest, parent);
        if (!v3_normalize(dir, dir)) v3_copy(dir, ray);
        v3 fb; v3_mad(fb, parent, dir, bone);
        v3_copy(sol_a, fb); v3_copy(sol_b, fb);
        return;
    }

    float sd = sqrtf(disc);
    v3_scale(sol_a, ray, (-b_coeff - sd) * 0.5f);
    v3_scale(sol_b, ray, (-b_coeff + sd) * 0.5f);
}

// Temporal pick: choose the solution whose bone direction best matches previous frame.
static void temporal_or_proximity(v3 result, const v3 sol_a, const v3 sol_b,
                                  const v3 parent_cam, const v3 child_raw,
                                  const mat3 cam_R, const float *prev_bd) {
    if (prev_bd) {
        v3 da, db; v3_sub(da, sol_a, parent_cam); v3_sub(db, sol_b, parent_cam);
        v3 da_n, db_n;
        if (v3_normalize(da_n, da) && v3_normalize(db_n, db)) {
            v3 da_w, db_w;
            mat3_mul_v3(da_w, cam_R, da_n);
            mat3_mul_v3(db_w, cam_R, db_n);
            float dot_a = v3_dot(da_w, prev_bd);
            float dot_b = v3_dot(db_w, prev_bd);
            v3_copy(result, dot_a >= dot_b ? sol_a : sol_b);
            return;
        }
    }
    float da = v3_dist(sol_a, child_raw);
    float db = v3_dist(sol_b, child_raw);
    v3_copy(result, da <= db ? sol_a : sol_b);
}

// Store world-space bone direction for temporal use.
static void store_bone_dir(float out_bd[3], const v3 child_cam, const v3 parent_cam, const mat3 cam_R) {
    v3 bd; v3_sub(bd, child_cam, parent_cam);
    v3 bd_n;
    if (v3_normalize(bd_n, bd)) {
        mat3_mul_v3(out_bd, cam_R, bd_n);
    } else {
        out_bd[0] = out_bd[1] = out_bd[2] = 0;
    }
}

// Compute angle between two directions around a hinge axis (unsigned, in radians).
static float signed_hinge_angle(const v3 from_dir, const v3 to_dir, const v3 hinge, int *is_flex) {
    float cos_a = v3_dot(from_dir, to_dir);
    if (cos_a > 1.0f) cos_a = 1.0f;
    if (cos_a < -1.0f) cos_a = -1.0f;
    v3 cv; v3_cross(cv, from_dir, to_dir);
    float ss = v3_dot(cv, hinge);
    *is_flex = ss > 0;
    return atan2f(fabsf(ss), cos_a);
}

// Rodrigues rotation: rotate v around axis by angle (cos_a, sin_a).
static void rodrigues(v3 result, const v3 v_in, const v3 axis, float cos_a, float sin_a) {
    v3 cross_part; v3_cross(cross_part, axis, v_in);
    float dot_part = v3_dot(axis, v_in);
    result[0] = v_in[0]*cos_a + cross_part[0]*sin_a + axis[0]*dot_part*(1-cos_a);
    result[1] = v_in[1]*cos_a + cross_part[1]*sin_a + axis[1]*dot_part*(1-cos_a);
    result[2] = v_in[2]*cos_a + cross_part[2]*sin_a + axis[2]*dot_part*(1-cos_a);
}

// ── Main reconstruction ──

bool hr_reconstruct(float joints[HR_JOINT_COUNT][5],
                    const float cam_quat[4],
                    const float cam_pos[3],
                    uint64_t timestamp_ns,
                    hr_state_t *state) {
    // Guard: skip if no camera pose (identity quat = uninitialized)
    float qlen = cam_quat[0]*cam_quat[0] + cam_quat[1]*cam_quat[1] +
                 cam_quat[2]*cam_quat[2] + cam_quat[3]*cam_quat[3];
    if (qlen < 0.5f) return false;

    // Gap detection
    int have_temporal = 0;
    if (state && state->valid) {
        if (timestamp_ns > state->last_ts &&
            timestamp_ns - state->last_ts <= GAP_THRESHOLD_NS) {
            have_temporal = 1;
        } else {
            state->valid = 0;
        }
    }

    mat3 cam_R;
    quat_to_mat3(cam_R, cam_quat);

    // Convert world → camera space
    v3 raw_cam[21], corrected[21];
    for (int j = 0; j < 21; j++) {
        v3 w = {joints[j][0], joints[j][1], joints[j][2]};
        v3 rel; v3_sub(rel, w, cam_pos);
        mat3T_mul_v3(raw_cam[j], cam_R, rel);
        v3_copy(corrected[j], raw_cam[j]);
    }

    // ── Phase 1a: Thumb chain (cascade + temporal) ──

    // CMC
    {
        float bone = get_bone_len(J_WRIST, J_THUMB_CMC);
        if (have_temporal) {
            v3 sa, sb;
            ray_sphere_both(sa, sb, raw_cam[J_THUMB_CMC], corrected[J_WRIST], bone);
            temporal_or_proximity(corrected[J_THUMB_CMC], sa, sb,
                                 corrected[J_WRIST], raw_cam[J_THUMB_CMC],
                                 cam_R, state->bone_dir[0]);
        } else {
            ray_sphere_proximity(corrected[J_THUMB_CMC], raw_cam[J_THUMB_CMC], corrected[J_WRIST], bone);
        }
    }

    // MCP with cascade validation
    {
        float bone_mcp = get_bone_len(J_THUMB_CMC, J_THUMB_MCP);
        float bone_ip = get_bone_len(J_THUMB_MCP, J_THUMB_IP);
        v3 sa, sb;
        ray_sphere_both(sa, sb, raw_cam[J_THUMB_MCP], corrected[J_THUMB_CMC], bone_mcp);

        if (have_temporal) {
            temporal_or_proximity(corrected[J_THUMB_MCP], sa, sb,
                                 corrected[J_THUMB_CMC], raw_cam[J_THUMB_MCP],
                                 cam_R, state->bone_dir[1]);
        } else {
            // Cascade: try both MCP solutions, pick better downstream IP angle
            v3 ip_a, ip_b;
            ray_sphere_proximity(ip_a, raw_cam[J_THUMB_IP], sa, bone_ip);
            ray_sphere_proximity(ip_b, raw_cam[J_THUMB_IP], sb, bone_ip);

            // IP angle from each cascade
            v3 ctm_a, mti_a, ctm_b, mti_b;
            v3_sub(ctm_a, sa, corrected[J_THUMB_CMC]);
            v3_sub(mti_a, ip_a, sa);
            v3_sub(ctm_b, sb, corrected[J_THUMB_CMC]);
            v3_sub(mti_b, ip_b, sb);
            v3 ctm_an, mti_an, ctm_bn, mti_bn;
            float ang_a = (float)M_PI, ang_b = (float)M_PI;
            if (v3_normalize(ctm_an, ctm_a) && v3_normalize(mti_an, mti_a)) {
                float d = v3_dot(ctm_an, mti_an);
                if (d > 1.0f) d = 1.0f;
                if (d < -1.0f) d = -1.0f;
                ang_a = acosf(d);
            }
            if (v3_normalize(ctm_bn, ctm_b) && v3_normalize(mti_bn, mti_b)) {
                float d = v3_dot(ctm_bn, mti_bn);
                if (d > 1.0f) d = 1.0f;
                if (d < -1.0f) d = -1.0f;
                ang_b = acosf(d);
            }

            if (ang_a < THUMB_IP_CASCADE_LIMIT && ang_b < THUMB_IP_CASCADE_LIMIT) {
                float da = v3_dist(sa, raw_cam[J_THUMB_MCP]);
                float db = v3_dist(sb, raw_cam[J_THUMB_MCP]);
                v3_copy(corrected[J_THUMB_MCP], da <= db ? sa : sb);
            } else if (ang_a < THUMB_IP_CASCADE_LIMIT) {
                v3_copy(corrected[J_THUMB_MCP], sa);
            } else if (ang_b < THUMB_IP_CASCADE_LIMIT) {
                v3_copy(corrected[J_THUMB_MCP], sb);
            } else {
                v3_copy(corrected[J_THUMB_MCP], ang_a <= ang_b ? sa : sb);
            }
        }
    }

    // IP and TIP
    for (int seg = 2; seg < 4; seg++) {
        int pi = THUMB_CHAIN[seg].p, ci = THUMB_CHAIN[seg].c;
        float bone = get_bone_len(pi, ci);
        if (have_temporal) {
            v3 sa, sb;
            ray_sphere_both(sa, sb, raw_cam[ci], corrected[pi], bone);
            temporal_or_proximity(corrected[ci], sa, sb,
                                 corrected[pi], raw_cam[ci],
                                 cam_R, state->bone_dir[seg]);
        } else {
            ray_sphere_proximity(corrected[ci], raw_cam[ci], corrected[pi], bone);
        }
    }

    // Store temporal state for thumb
    if (state) {
        for (int seg = 0; seg < 4; seg++) {
            int pi = THUMB_CHAIN[seg].p, ci = THUMB_CHAIN[seg].c;
            store_bone_dir(state->bone_dir[seg], corrected[ci], corrected[pi], cam_R);
        }
        state->last_ts = timestamp_ns;
        state->valid = 1;
    }

    // ── Phase 1b: Finger chains (pure proximity) ──

    for (int fc = 0; fc < 4; fc++) {
        const seg_t *chain = FINGER_CHAINS[fc];
        for (int seg = 0; seg < 4; seg++) {
            int pi = chain[seg].p, ci = chain[seg].c;
            float bone = get_bone_len(pi, ci);
            ray_sphere_proximity(corrected[ci], raw_cam[ci], corrected[pi], bone);
        }
    }

    // ── Phase 2: MCP spacing validation ──

    {
        int spacing_ok = 1;
        for (int i = 0; i < 3; i++) {
            float d = v3_dist(corrected[MCP_ADJ[i][0]], corrected[MCP_ADJ[i][1]]);
            if (d > MCP_MAX_SPACING) { spacing_ok = 0; break; }
        }

        if (!spacing_ok) {
            v3 mcp_sols[4][2]; // [mcp_index][solution]
            for (int i = 0; i < 4; i++) {
                int mcp = MCP_INDICES[i];
                float bone = get_bone_len(J_WRIST, mcp);
                ray_sphere_both(mcp_sols[i][0], mcp_sols[i][1], raw_cam[mcp], corrected[J_WRIST], bone);
            }

            v3 best_combo[4];
            float best_score = 1e30f;
            int found = 0;

            for (int bits = 0; bits < 16; bits++) {
                v3 combo[4];
                for (int i = 0; i < 4; i++)
                    v3_copy(combo[i], mcp_sols[i][(bits >> i) & 1]);

                int valid = 1;
                for (int i = 0; i < 3; i++) {
                    float d = v3_dist(combo[MCP_ADJ[i][0] == MCP_INDICES[0] ? 0 :
                                            MCP_ADJ[i][0] == MCP_INDICES[1] ? 1 :
                                            MCP_ADJ[i][0] == MCP_INDICES[2] ? 2 : 3],
                                     combo[MCP_ADJ[i][1] == MCP_INDICES[1] ? 1 :
                                            MCP_ADJ[i][1] == MCP_INDICES[2] ? 2 : 3]);
                    if (d > MCP_MAX_SPACING || d < MCP_MIN_SPACING) { valid = 0; break; }
                }
                if (!valid) continue;

                float score = 0;
                for (int i = 0; i < 4; i++)
                    score += v3_dist(combo[i], raw_cam[MCP_INDICES[i]]);

                if (score < best_score) {
                    best_score = score;
                    for (int i = 0; i < 4; i++) v3_copy(best_combo[i], combo[i]);
                    found = 1;
                }
            }

            if (found) {
                int changed[4] = {0,0,0,0};
                for (int i = 0; i < 4; i++) {
                    if (v3_dist(corrected[MCP_INDICES[i]], best_combo[i]) > 0.001f) {
                        v3_copy(corrected[MCP_INDICES[i]], best_combo[i]);
                        changed[i] = 1;
                    }
                }
                // Re-cascade finger chains from corrected MCPs
                for (int fc = 0; fc < 4; fc++) {
                    if (!changed[fc]) continue;
                    const seg_t *chain = FINGER_CHAINS[fc];
                    for (int seg = 1; seg < 4; seg++) {
                        int pi = chain[seg].p, ci = chain[seg].c;
                        float bone = get_bone_len(pi, ci);
                        ray_sphere_proximity(corrected[ci], raw_cam[ci], corrected[pi], bone);
                    }
                }
            }
        }
    }

    // ── Phase 3: Finger coplanarity + angle limits ──

    {
        v3 v1, v2, pn;
        v3_sub(v1, corrected[J_INDEX_MCP], corrected[J_WRIST]);
        v3_sub(v2, corrected[J_PINKY_MCP], corrected[J_WRIST]);
        v3_cross(pn, v1, v2);
        float pn_len = v3_len(pn);

        if (pn_len > EPS) {
            v3_scale(pn, pn, 1.0f/pn_len);

            const float MCP_MAX_EXT = 0.7854f;   // 45 deg
            const float MCP_MAX_FLEX = 1.3963f;   // 80 deg
            const float MCP_ABD_MAX = 0.2618f;    // 15 deg
            const float PIP_MAX_EXT = 0.0873f;    // 5 deg
            const float PIP_MAX_FLEX = 2.0944f;    // 120 deg
            const float DIP_MAX_EXT = 0.1745f;    // 10 deg
            const float DIP_MAX_FLEX = 1.5708f;    // 90 deg

            for (int fc = 0; fc < 4; fc++) {
                const seg_t *chain = FINGER_CHAINS[fc];
                int mcp_idx = chain[0].c;
                int pip_idx = chain[1].c;

                v3 wtm, m2p;
                v3_sub(wtm, corrected[mcp_idx], corrected[J_WRIST]);
                v3_sub(m2p, corrected[pip_idx], corrected[mcp_idx]);
                float wtm_l = v3_len(wtm), m2p_l = v3_len(m2p);
                if (wtm_l < EPS || m2p_l < EPS) continue;

                v3 wtm_d, m2p_d;
                v3_scale(wtm_d, wtm, 1.0f/wtm_l);
                v3_scale(m2p_d, m2p, 1.0f/m2p_l);

                v3 hinge;
                v3_cross(hinge, wtm_d, pn);
                float ha_l = v3_len(hinge);
                if (ha_l < EPS) continue;
                v3_scale(hinge, hinge, 1.0f/ha_l);

                // MCP abduction clamping
                v3 m2p_palm; // m2p projected into palm plane
                float m2p_pn_dot = v3_dot(m2p_d, pn);
                v3_mad(m2p_palm, m2p_d, pn, -m2p_pn_dot);
                float m2p_palm_l = v3_len(m2p_palm);
                if (m2p_palm_l > EPS) {
                    v3_scale(m2p_palm, m2p_palm, 1.0f/m2p_palm_l);
                    v3 ref_palm;
                    float wtm_pn_dot = v3_dot(wtm_d, pn);
                    v3_mad(ref_palm, wtm_d, pn, -wtm_pn_dot);
                    float ref_l = v3_len(ref_palm);
                    if (ref_l > EPS) {
                        v3_scale(ref_palm, ref_palm, 1.0f/ref_l);
                        float cos_abd = v3_dot(ref_palm, m2p_palm);
                        if (cos_abd > 1.0f) cos_abd = 1.0f;
                        if (cos_abd < -1.0f) cos_abd = -1.0f;
                        v3 cr; v3_cross(cr, ref_palm, m2p_palm);
                        float sin_abd = v3_dot(cr, pn);
                        float abd = atan2f(sin_abd, cos_abd);

                        if (fabsf(abd) > MCP_ABD_MAX) {
                            float abd_c = abd > 0 ? MCP_ABD_MAX : -MCP_ABD_MAX;
                            v3 new_palm;
                            rodrigues(new_palm, ref_palm, pn, cosf(abd_c), sinf(abd_c));
                            v3 flex_comp; v3_scale(flex_comp, pn, m2p_pn_dot);
                            v3 new_m2p; v3_mad(new_m2p, flex_comp, new_palm, m2p_palm_l);
                            float new_l = v3_len(new_m2p);
                            if (new_l > EPS) {
                                v3_scale(m2p_d, new_m2p, 1.0f/new_l);
                                float pip_bone = get_bone_len(mcp_idx, pip_idx);
                                v3_mad(corrected[pip_idx], corrected[mcp_idx], m2p_d, pip_bone);
                            }
                        }
                    }
                }

                // MCP flexion — try alternative before clamp
                {
                    int is_flex;
                    float mcp_ang = signed_hinge_angle(wtm_d, m2p_d, hinge, &is_flex);
                    int exceeds = (is_flex && mcp_ang > MCP_MAX_FLEX) || (!is_flex && mcp_ang > MCP_MAX_EXT);

                    if (exceeds) {
                        float pip_bone = get_bone_len(mcp_idx, pip_idx);
                        v3 sa, sb;
                        ray_sphere_both(sa, sb, raw_cam[pip_idx], corrected[mcp_idx], pip_bone);
                        v3 alt;
                        float da = v3_dist(sa, raw_cam[pip_idx]);
                        float db = v3_dist(sb, raw_cam[pip_idx]);
                        v3_copy(alt, da <= db ? sb : sa); // pick the OTHER one

                        v3 alt_d; v3_sub(alt_d, alt, corrected[mcp_idx]);
                        v3 alt_dn;
                        if (v3_normalize(alt_dn, alt_d)) {
                            // Remove lateral for coplanarity
                            float lat = v3_dot(alt_dn, hinge);
                            v3 alt_ip; v3_mad(alt_ip, alt_dn, hinge, -lat);
                            v3 alt_ipn;
                            if (v3_normalize(alt_ipn, alt_ip)) {
                                int if2;
                                float a2 = signed_hinge_angle(wtm_d, alt_ipn, hinge, &if2);
                                int alt_ok = !(if2 && a2 > MCP_MAX_FLEX) && !(!if2 && a2 > MCP_MAX_EXT);
                                if (alt_ok) {
                                    v3_copy(m2p_d, alt_ipn);
                                    float pb = get_bone_len(mcp_idx, pip_idx);
                                    v3_mad(corrected[pip_idx], corrected[mcp_idx], m2p_d, pb);
                                    exceeds = 0;
                                }
                            }
                        }
                    }

                    if (exceeds) {
                        float target = is_flex ? MCP_MAX_FLEX : MCP_MAX_EXT;
                        if (mcp_ang > target) mcp_ang = target;
                        float sgn = is_flex ? 1.0f : -1.0f;
                        v3 new_dir;
                        rodrigues(new_dir, wtm_d, hinge, cosf(mcp_ang), sinf(mcp_ang) * sgn);
                        float pb = get_bone_len(mcp_idx, pip_idx);
                        v3_mad(corrected[pip_idx], corrected[mcp_idx], new_dir, pb);
                        v3_copy(m2p_d, new_dir);
                    }
                }

                // Recompute hinge
                v3_cross(hinge, m2p_d, pn);
                ha_l = v3_len(hinge);
                if (ha_l < EPS) continue;
                v3_scale(hinge, hinge, 1.0f/ha_l);

                // PIP, DIP, TIP: coplanarity + angle validation
                // seg_i maps: seg=1→0(no limits), seg=2→1(PIP), seg=3→2(DIP)
                const float seg_lim[][2] = {{0,0}, {PIP_MAX_EXT, PIP_MAX_FLEX}, {DIP_MAX_EXT, DIP_MAX_FLEX}};
                for (int seg = 1; seg < 4; seg++) {
                    int seg_i = seg - 1;  // Python-compatible index
                    int pi = chain[seg].p, ci = chain[seg].c;
                    float bone = get_bone_len(pi, ci);

                    v3 to_child; v3_sub(to_child, corrected[ci], corrected[pi]);
                    float lat = v3_dot(to_child, hinge);
                    v3 ip_vec; v3_mad(ip_vec, to_child, hinge, -lat);
                    v3 ip_dir;
                    if (!v3_normalize(ip_dir, ip_vec)) continue;

                    int within = 1;
                    if (seg_i > 0) {
                        int gp = chain[seg-1].p;
                        v3 pd; v3_sub(pd, corrected[pi], corrected[gp]);
                        v3 pdn;
                        if (v3_normalize(pdn, pd)) {
                            int is_f;
                            float ang = signed_hinge_angle(pdn, ip_dir, hinge, &is_f);
                            within = !(is_f && ang > seg_lim[seg_i][1]) &&
                                     !(!is_f && ang > seg_lim[seg_i][0]);

                            if (!within) {
                                // Try alternative
                                v3 sa2, sb2;
                                ray_sphere_both(sa2, sb2, raw_cam[ci], corrected[pi], bone);
                                float d_a = v3_dist(sa2, raw_cam[ci]);
                                float d_b = v3_dist(sb2, raw_cam[ci]);
                                v3 alt2; v3_copy(alt2, d_a <= d_b ? sb2 : sa2);
                                v3 alt_tc; v3_sub(alt_tc, alt2, corrected[pi]);
                                float alt_lat = v3_dot(alt_tc, hinge);
                                v3 alt_ip2; v3_mad(alt_ip2, alt_tc, hinge, -alt_lat);
                                v3 alt_ipn2;
                                if (v3_normalize(alt_ipn2, alt_ip2)) {
                                    int if2;
                                    float a2 = signed_hinge_angle(pdn, alt_ipn2, hinge, &if2);
                                    if (!(if2 && a2 > seg_lim[seg_i][1]) &&
                                        !(!if2 && a2 > seg_lim[seg_i][0])) {
                                        v3_copy(ip_dir, alt_ipn2);
                                        within = 1;
                                    }
                                }
                            }

                            if (!within) {
                                // Clamp
                                float target = is_f ? seg_lim[seg_i][1] : seg_lim[seg_i][0];
                                if (ang > target) ang = target;
                                float sgn = is_f ? 1.0f : -1.0f;
                                rodrigues(ip_dir, pdn, hinge, cosf(ang), sinf(ang)*sgn);
                            }
                        }
                    }

                    v3_mad(corrected[ci], corrected[pi], ip_dir, bone);
                }
            }
        }
    }

    // ── Phase 3 (cont): Thumb angle validation ──

    {
        v3 ctm, mti;
        v3_sub(ctm, corrected[J_THUMB_MCP], corrected[J_THUMB_CMC]);
        v3_sub(mti, corrected[J_THUMB_IP], corrected[J_THUMB_MCP]);
        float ctm_l = v3_len(ctm), mti_l = v3_len(mti);

        if (ctm_l > EPS && mti_l > EPS) {
            v3 ctm_d, mti_d;
            v3_scale(ctm_d, ctm, 1.0f/ctm_l);
            v3_scale(mti_d, mti, 1.0f/mti_l);

            v3 tcn; v3_cross(tcn, ctm, mti);
            float tcn_l = v3_len(tcn);

            if (tcn_l > EPS) {
                v3_scale(tcn, tcn, 1.0f/tcn_l);
                const float THUMB_EXT = 0.1745f;  // 10 deg
                const float THUMB_FLEX = 1.3963f;  // 80 deg

                // IP angle validation
                int is_f;
                float ang = signed_hinge_angle(ctm_d, mti_d, tcn, &is_f);
                int exceeds = (is_f && ang > THUMB_FLEX) || (!is_f && ang > THUMB_EXT);

                if (exceeds) {
                    float bone_ip = get_bone_len(J_THUMB_MCP, J_THUMB_IP);
                    v3 sa, sb;
                    ray_sphere_both(sa, sb, raw_cam[J_THUMB_IP], corrected[J_THUMB_MCP], bone_ip);
                    float da = v3_dist(sa, raw_cam[J_THUMB_IP]);
                    float db = v3_dist(sb, raw_cam[J_THUMB_IP]);
                    v3 alt; v3_copy(alt, da <= db ? sb : sa);
                    v3 alt_d; v3_sub(alt_d, alt, corrected[J_THUMB_MCP]);
                    v3 alt_dn;
                    if (v3_normalize(alt_dn, alt_d)) {
                        int if2;
                        float a2 = signed_hinge_angle(ctm_d, alt_dn, tcn, &if2);
                        if (!(if2 && a2 > THUMB_FLEX) && !(!if2 && a2 > THUMB_EXT)) {
                            v3_copy(corrected[J_THUMB_IP], alt);
                            v3_sub(mti, corrected[J_THUMB_IP], corrected[J_THUMB_MCP]);
                            mti_l = v3_len(mti);
                            if (mti_l > EPS) v3_scale(mti_d, mti, 1.0f/mti_l);
                            v3_cross(tcn, ctm, mti);
                            tcn_l = v3_len(tcn);
                            if (tcn_l > EPS) v3_scale(tcn, tcn, 1.0f/tcn_l);
                            exceeds = 0;
                        }
                    }
                }

                // TIP: project into curl plane + angle validation
                v3 ip_to_tip;
                v3_sub(ip_to_tip, corrected[J_THUMB_TIP], corrected[J_THUMB_IP]);
                float tip_lat = v3_dot(ip_to_tip, tcn);
                v3 tip_ip; v3_mad(tip_ip, ip_to_tip, tcn, -tip_lat);
                float tip_ip_l = v3_len(tip_ip);
                float bone_tip = get_bone_len(J_THUMB_IP, J_THUMB_TIP);

                if (tip_ip_l > EPS) {
                    v3 tip_ipd;
                    v3_scale(tip_ipd, tip_ip, 1.0f/tip_ip_l);

                    int is_f2;
                    float ang2 = signed_hinge_angle(mti_d, tip_ipd, tcn, &is_f2);
                    int exc2 = (is_f2 && ang2 > THUMB_FLEX) || (!is_f2 && ang2 > THUMB_EXT);

                    if (exc2) {
                        v3 sa, sb;
                        ray_sphere_both(sa, sb, raw_cam[J_THUMB_TIP], corrected[J_THUMB_IP], bone_tip);
                        float da = v3_dist(sa, raw_cam[J_THUMB_TIP]);
                        float db = v3_dist(sb, raw_cam[J_THUMB_TIP]);
                        v3 alt; v3_copy(alt, da <= db ? sb : sa);
                        v3 alt_to; v3_sub(alt_to, alt, corrected[J_THUMB_IP]);
                        float alt_lat = v3_dot(alt_to, tcn);
                        v3 alt_ip; v3_mad(alt_ip, alt_to, tcn, -alt_lat);
                        v3 alt_ipn;
                        if (v3_normalize(alt_ipn, alt_ip)) {
                            int if3;
                            float a3 = signed_hinge_angle(mti_d, alt_ipn, tcn, &if3);
                            if (!(if3 && a3 > THUMB_FLEX) && !(!if3 && a3 > THUMB_EXT)) {
                                v3_copy(tip_ipd, alt_ipn);
                                exc2 = 0;
                            }
                        }
                    }

                    if (exc2) {
                        float target = is_f2 ? THUMB_FLEX : THUMB_EXT;
                        if (ang2 > target) ang2 = target;
                        float sgn = is_f2 ? 1.0f : -1.0f;
                        rodrigues(tip_ipd, mti_d, tcn, cosf(ang2), sinf(ang2)*sgn);
                    }

                    v3_mad(corrected[J_THUMB_TIP], corrected[J_THUMB_IP], tip_ipd, bone_tip);
                }
            }
        }
    }

    // ── Phase 4: Collision avoidance ──

    {
        const int adj[][4] = {
            {J_INDEX_DIP, J_INDEX_TIP, J_MIDDLE_DIP, J_MIDDLE_TIP},
            {J_MIDDLE_DIP, J_MIDDLE_TIP, J_RING_DIP, J_RING_TIP},
            {J_RING_DIP, J_RING_TIP, J_PINKY_DIP, J_PINKY_TIP},
        };
        for (int i = 0; i < 3; i++) {
            int da=adj[i][0], ta=adj[i][1], db=adj[i][2], tb=adj[i][3];
            if (v3_dist(corrected[ta], corrected[tb]) >= 0.006f) continue;
            int pairs[2][2] = {{da,ta},{db,tb}};
            for (int p = 0; p < 2; p++) {
                int dip=pairs[p][0], tip=pairs[p][1];
                int pip = dip - 1;
                v3 pd, cd;
                v3_sub(pd, corrected[dip], corrected[pip]);
                v3_sub(cd, corrected[tip], corrected[dip]);
                float pd_l = v3_len(pd), cd_l = v3_len(cd);
                if (pd_l > EPS && cd_l > EPS) {
                    v3 pdn; v3_scale(pdn, pd, 1.0f/pd_l);
                    v3 cdn; v3_scale(cdn, cd, 1.0f/cd_l);
                    float curl = 1.0f - v3_dot(pdn, cdn);
                    if (curl > 0.1f) {
                        float bone = get_bone_len(dip, tip);
                        v3_mad(corrected[tip], corrected[dip], pdn, bone);
                        break;
                    }
                }
            }
        }
    }

    // ── Convert camera → world and write back ──

    for (int j = 0; j < 21; j++) {
        v3 world;
        mat3_mul_v3(world, cam_R, corrected[j]);
        joints[j][0] = world[0] + cam_pos[0];
        joints[j][1] = world[1] + cam_pos[1];
        joints[j][2] = world[2] + cam_pos[2];
    }

    return true;
}
