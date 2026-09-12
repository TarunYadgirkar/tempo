// gesture_tap.h — Per-fingertip tap-against-plane detection.
//
// Distinct from the priority-FSM gestures in gesture_engine.h: the tap
// listener is per-target-plane and per-fingertip, not per-hand.  Created
// by callers that want a "tap a key" feel — the spatial keyboard widget
// in particular.
//
// Detection: a fingertip taps the plane when its velocity component along
// the plane normal reverses (downward → upward) within a thin slab around
// the plane.  After firing, that fingertip is suppressed until it lifts
// above the plane by `lift_threshold_m` so a single tap can't double-fire
// from tracking jitter.
//
// Coordinate convention: plane_normal points away from the surface
// (i.e. out toward the user's finger before contact).  Downward motion
// is "negative along normal" in the math.

#pragma once

#include "gesture_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ge_tap_listener_t ge_tap_listener_t;

// Callback fired exactly once when a fingertip taps the configured plane.
//   hand           : 0 = left, 1 = right
//   fingertip_idx  : 0=thumb, 1=index, 2=middle, 3=ring, 4=pinky
//   plane_xy       : contact point in plane-local 2D coordinates (metres)
//                    where (0,0) is the plane origin and the basis is the
//                    plane axes supplied at create time.
typedef void (*ge_tap_callback_t)(int hand,
                                  int fingertip_idx,
                                  const float plane_xy[2],
                                  void *user_data);

// Create a tap listener for a single target plane.
//
//   plane_origin  : world-space point on the plane.
//   plane_normal  : world-space normal, normalised (out toward fingers).
//   plane_axis_x  : world-space axis defining +x in plane-local 2D.
//                   The +y axis is computed as normal × axis_x.
//   slab_half_thickness_m : half-thickness of the velocity-reversal
//                           detection slab.  Default 0.005 m (5 mm).
//   lift_threshold_m      : how far above the plane a fingertip must
//                           rise before it can tap again.  Default 0.01 m.
ge_tap_listener_t *ge_tap_listener_create(const float plane_origin[3],
                                          const float plane_normal[3],
                                          const float plane_axis_x[3],
                                          float slab_half_thickness_m,
                                          float lift_threshold_m,
                                          ge_tap_callback_t cb,
                                          void *user_data);

// Free a tap listener.
void ge_tap_listener_destroy(ge_tap_listener_t *l);

// Feed one frame of hand data.  Looks at the 5 fingertips on each
// present hand, computes velocity from the previous frame, and emits
// taps via the callback.  dt_s = seconds since the previous call.
void ge_tap_listener_update(ge_tap_listener_t *l,
                            const ge_hand_t hands[2],
                            float dt_s);

#ifdef __cplusplus
}  // extern "C"
#endif
