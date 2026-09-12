// hand_reconstruct.h — Bone-constrained hand joint depth reconstruction.
//
// Pure C API. No dependencies beyond math.h.
// Corrects 21-joint hand positions using bone-length constraints + ray-sphere
// intersection. Fixes the depth ambiguity in LiDAR-based hand tracking where
// joints are inflated 1.3-8.4x along the camera ray.
//
// Thread safety: hr_state_t is NOT thread-safe. Use one state per hand per thread.

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HR_JOINT_COUNT 21

// Per-hand temporal state for thumb disambiguation.
typedef struct hr_state_t hr_state_t;

hr_state_t *hr_state_create(void);
void hr_state_destroy(hr_state_t *s);

// Reconstruct hand joints in-place.
//
// joints:       21x5 floats [joint][x,y,z,confidence,reserved] in ARKit world space.
//               Modified in-place. Confidence and reserved fields are untouched.
// cam_quat:     Camera rotation quaternion [x,y,z,w]. If w==0 and xyz==0 (no pose
//               available yet), reconstruction is skipped and joints are unchanged.
// cam_pos:      Camera position [x,y,z] in ARKit world space (meters).
// timestamp_ns: Hand packet timestamp (nanoseconds). Used for temporal gap detection.
// state:        Per-hand temporal state. May be NULL for stateless (proximity-only) mode.
//
// Returns true if reconstruction was applied, false if skipped (no camera pose).
bool hr_reconstruct(float joints[HR_JOINT_COUNT][5],
                    const float cam_quat[4],
                    const float cam_pos[3],
                    uint64_t timestamp_ns,
                    hr_state_t *state);

#ifdef __cplusplus
}
#endif
