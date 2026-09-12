// world_model.h — Internal C++ API for the seqlock/mutex world state store.
// Not part of the public C API; only used within the bridge-receiver library.

#pragma once

#include "spatial_bridge.h"

struct WorldModel;

// Lifecycle
WorldModel *world_model_create();
void world_model_destroy(WorldModel *wm);

// Write side (UDP/file thread)
void world_model_update_pose(WorldModel *wm, const sb_pose_t &pose);
void world_model_update_plane(WorldModel *wm, const sb_plane_t &plane);
// Takes ownership of frame.rgba; caller must not free it after this call.
void world_model_update_frame(WorldModel *wm, const sb_frame_t &frame);
// Takes ownership of depth.depth; caller must not free it after this call.
void world_model_update_depth(WorldModel *wm, const sb_depth_t &depth);
void world_model_update_hand(WorldModel *wm, const sb_hand_t &hand);
void world_model_update_intrinsics(WorldModel *wm, const sb_intrinsics_t &intr);
// Call once per received packet for rate tracking.
void world_model_record_packet(WorldModel *wm);

// Read side (consumer threads) — maps directly to the public C API
bool world_model_get_pose(WorldModel *wm, sb_pose_t *out);
// Level-triggered (unlike world_model_get_pose): always the latest epoch seen.
uint32_t world_model_get_session_epoch(WorldModel *wm);
bool world_model_get_frame(WorldModel *wm, sb_frame_t *out);
bool world_model_get_depth(WorldModel *wm, sb_depth_t *out);
int world_model_get_planes(WorldModel *wm, sb_plane_t *out, int max_planes);
bool world_model_get_hand(WorldModel *wm, int hand_index, sb_hand_t *out);
bool world_model_get_intrinsics(WorldModel *wm, sb_intrinsics_t *out);
// Non-destructive, unlike world_model_get_frame: age of the newest ingested
// frame regardless of who consumed it. -1 = none has ever arrived.
int64_t world_model_get_frame_age_ms(WorldModel *wm);
float world_model_get_packet_rate(WorldModel *wm);
