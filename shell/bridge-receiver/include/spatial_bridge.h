// spatial_bridge.h — Public C API for the bridge-receiver library.
//
// Thread safety: all functions are safe to call from any thread after sb_start().
// sb_receiver_t is an opaque handle; never access its internals directly.
//
// Coordinate system: ARKit right-handed, Y-up, -Z forward (identical to OpenXR LOCAL).
// Quaternions are stored in xyzw order throughout.

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Structs
// ---------------------------------------------------------------------------

// Decoded 0x01 pose packet.
typedef struct sb_pose_t {
    uint64_t timestamp_ns;   // nanoseconds since Unix epoch
    float pos[3];            // position: x, y, z (meters, ARKit world space)
    float rot[4];            // rotation quaternion: x, y, z, w
    float tracking_quality;  // 0.0 = lost, 0.5 = limited, 1.0 = normal
    // ARKit world-frame generation this pose is expressed in. Bumped by the
    // sender on every session.run(.resetTracking); 0 = pre-epoch sender.
    uint32_t session_epoch;
} sb_pose_t;

// Decoded 0x02 plane anchor packet.
typedef struct sb_plane_t {
    uint64_t timestamp_ns;  // nanoseconds since Unix epoch
    uint8_t uuid[16];       // raw UUID bytes as ARKit provides
    float center[3];        // plane center: x, y, z (meters, ARKit world space)
    float normal[3];        // unit normal vector in world space: x, y, z
    float extent[2];        // plane size: [width, height] in meters
    uint8_t alignment;      // 0 = horizontal, 1 = vertical
    uint8_t is_removed;     // 1 = plane no longer tracked, must remove from model
} sb_plane_t;

// Number of hand joints provided by Apple Vision framework.
#define SB_HAND_JOINT_COUNT 21

// Joint indices (Apple Vision framework VNHumanHandPoseObservation order).
// Each finger has: MCP (knuckle), PIP, DIP, TIP (fingertip).
enum sb_hand_joint {
    SB_JOINT_WRIST = 0,
    SB_JOINT_THUMB_CMC,
    SB_JOINT_THUMB_MCP,
    SB_JOINT_THUMB_IP,
    SB_JOINT_THUMB_TIP,
    SB_JOINT_INDEX_MCP,
    SB_JOINT_INDEX_PIP,
    SB_JOINT_INDEX_DIP,
    SB_JOINT_INDEX_TIP,
    SB_JOINT_MIDDLE_MCP,
    SB_JOINT_MIDDLE_PIP,
    SB_JOINT_MIDDLE_DIP,
    SB_JOINT_MIDDLE_TIP,
    SB_JOINT_RING_MCP,
    SB_JOINT_RING_PIP,
    SB_JOINT_RING_DIP,
    SB_JOINT_RING_TIP,
    SB_JOINT_PINKY_MCP,
    SB_JOINT_PINKY_PIP,
    SB_JOINT_PINKY_DIP,
    SB_JOINT_PINKY_TIP,
};

// Decoded 0x05 hand joint packet.
// Joints are in ARKit world space (meters, right-handed Y-up).
// Per-joint layout: {x, y, z, confidence, reserved}.
typedef struct sb_hand_t {
    uint64_t timestamp_ns;
    uint8_t hand_index;                    // 0 = left, 1 = right
    uint8_t joint_count;                   // always SB_HAND_JOINT_COUNT (21)
    float joints[SB_HAND_JOINT_COUNT][5];  // [joint_idx][x,y,z,confidence,reserved]
} sb_hand_t;

// Decoded camera frame (JPEG decompressed to RGBA). Produced from either a
// 0x03 single-datagram frame or a 0x09 chunked frame reassembled from sub-MTU
// pieces; both paths converge here, so consumers are transport-agnostic.
// Always free with sb_free_frame() — never with raw free().
typedef struct sb_frame_t {
    uint64_t timestamp_ns;  // nanoseconds since Unix epoch
    uint32_t width;         // pixels
    uint32_t height;        // pixels
    uint8_t *rgba;          // decoded RGBA pixels, row-major, 4 bytes/pixel
    uint32_t rgba_size;     // total bytes: width * height * 4
} sb_frame_t;

// Decoded 0x0A camera intrinsics packet (ARKit pinhole model).
// fx/fy/cx/cy are in pixels of image_width x image_height (ARKit's native
// capture resolution, NOT the downscaled JPEG resolution). Only the ratios
// image_width/fx and (image_width/2 - cx)/fx are consumed downstream, and those
// are scale-invariant. fx/fy drift per frame with autofocus; cx/cy are stable.
typedef struct sb_intrinsics_t {
    uint64_t timestamp_ns;  // nanoseconds since Unix epoch
    float fx;               // focal length x, pixels
    float fy;               // focal length y, pixels
    float cx;               // principal point x, pixels
    float cy;               // principal point y, pixels
    float image_width;      // resolution the intrinsics are normalised to, pixels
    float image_height;
} sb_intrinsics_t;

// Decoded 0x0B depth map (LiDAR sceneDepth, dequantized to metres). Aligned to
// the same camera image as the 0x09 colour frame and tagged with the SAME
// timestamp. Produced by reassembling 0x0B chunks. depth[i]==0.0f means "no /
// invalid reading" — it must NOT occlude. Always free with sb_free_depth().
typedef struct sb_depth_t {
    uint64_t timestamp_ns;  // ns since Unix epoch (matches the paired colour frame)
    uint32_t width;         // depth columns
    uint32_t height;        // depth rows
    float    z_min;         // metres mapped to the smallest valid quantized value
    float    z_max;         // metres mapped to the largest quantized value
    float   *depth;         // row-major metres, width*height floats; 0.0f == invalid
    uint32_t depth_count;   // width * height
} sb_depth_t;

// Opaque receiver handle.
typedef struct sb_receiver_impl_t sb_receiver_t;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

// Create a receiver listening on the given UDP port.
// Returns NULL on failure (logs reason to stderr).
sb_receiver_t *sb_receiver_create(uint16_t port);

// Create an offline receiver that replays a recorded .bin session file.
// If loop is true, the file is replayed indefinitely.
// Packets are replayed with inter-packet timing preserved from the original timestamps.
// Returns NULL on failure.
sb_receiver_t *sb_receiver_from_file(const char *path, bool loop);

// Free all resources. Calls sb_stop() first if not already stopped.
void sb_receiver_destroy(sb_receiver_t *r);

// Start the background receive/replay thread.
void sb_start(sb_receiver_t *r);

// Stop the background thread and drain any pending state. Blocks until the
// thread has exited.
void sb_stop(sb_receiver_t *r);

// ---------------------------------------------------------------------------
// World state queries (non-blocking; safe from any thread after sb_start())
// ---------------------------------------------------------------------------

// Copy the latest pose into *out.
// Returns false if no new pose has arrived since the last successful call.
bool sb_get_latest_pose(sb_receiver_t *r, sb_pose_t *out);

// Latest ARKit world-frame generation seen on the wire. Unlike
// sb_get_latest_pose() this is level-triggered: it always returns the current
// value, so a consumer can compare it every frame. 0 means no pose yet, or a
// pre-epoch sender (assume continuity). A change to a different nonzero value
// means ARKit re-initialised its world origin — re-anchor, do not drift.
uint32_t sb_get_session_epoch(sb_receiver_t *r);

// Copy the latest decoded camera frame into *out.
// Returns false if no new frame has arrived since the last successful call.
// On true, the caller MUST release the frame with sb_free_frame() when done.
bool sb_get_latest_frame(sb_receiver_t *r, sb_frame_t *out);

// Copy the latest decoded depth map into *out.
// Returns false if no new depth map has arrived since the last successful call.
// On true, the caller MUST release it with sb_free_depth() when done.
bool sb_get_latest_depth(sb_receiver_t *r, sb_depth_t *out);

// Copy up to max_planes plane descriptors into out[].
// Returns the number of planes written (0 if none).
int sb_get_planes(sb_receiver_t *r, sb_plane_t *out, int max_planes);

// Copy the latest hand joint data for the given hand (0=left, 1=right) into *out.
// Returns false if no hand data has arrived for this hand since the last call.
bool sb_get_hand(sb_receiver_t *r, int hand_index, sb_hand_t *out);

// Copy the latest camera intrinsics into *out.
// NOT edge-triggered: returns the most recent intrinsics on every call, and
// false only until the first 0x0A packet has ever arrived. Consumers (the
// passthrough quad sizing and the driver FOV) want the latest known value each
// frame, not a one-shot event.
bool sb_get_latest_intrinsics(sb_receiver_t *r, sb_intrinsics_t *out);

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

// Average packet rate over the last 1 second (all packet types combined).
float sb_get_packet_rate(sb_receiver_t *r);

// Milliseconds since the newest camera frame was ingested; -1 if none has ever
// arrived. Unlike sb_get_latest_frame() this neither consumes nor is gated on
// consumption, so a stalled camera stream stays visible to callers that never
// read frames (the control socket's `stats`) and to the one that does. Poses
// can keep flowing while frames die — sb_get_packet_rate() alone cannot tell
// you that, and world-locked UI over a frozen photo is nauseating.
int64_t sb_get_frame_age_ms(sb_receiver_t *r);

// ---------------------------------------------------------------------------
// Memory management
// ---------------------------------------------------------------------------

// Release an sb_frame_t returned by sb_get_latest_frame().
// Frees the rgba buffer allocated by the JPEG decoder. Never use raw free().
void sb_free_frame(sb_frame_t *f);

// Release an sb_depth_t returned by sb_get_latest_depth().
// Frees the depth buffer. Never use raw free().
void sb_free_depth(sb_depth_t *d);

// ---------------------------------------------------------------------------
// Cross-process pose sharing (POSIX shared memory + seqlock)
//
// The process that owns the bridge-receiver (wxrd) creates a writer.
// The process that needs poses (Monado driver) opens a reader.
// The shared memory name should be "/spatial_bridge_pose".
// ---------------------------------------------------------------------------

typedef struct sb_shm_writer_impl_t sb_shm_writer_t;
typedef struct sb_shm_reader_impl_t sb_shm_reader_t;

// Create a shared memory writer.  name must start with '/'.
// Removes any stale segment from a prior crash before creating.
sb_shm_writer_t *sb_shm_writer_create(const char *name);

// Write a pose into the shared memory (seqlock-protected, lock-free).
void sb_shm_writer_update(sb_shm_writer_t *w, const sb_pose_t *pose);

// Write camera intrinsics into the shared memory. Uses a SEPARATE seqlock from
// the pose, so pose and intrinsics update independently (different rates).
void sb_shm_writer_update_intrinsics(sb_shm_writer_t *w,
                                     const sb_intrinsics_t *intr);

// Destroy the writer and unlink the shared memory segment.
void sb_shm_writer_destroy(sb_shm_writer_t *w);

// Open an existing shared memory segment for reading.
// Returns NULL if the segment doesn't exist yet (writer not started).
sb_shm_reader_t *sb_shm_reader_open(const char *name);

// Read the latest pose from shared memory.
// Returns false if no new pose since the last successful call, or if
// the writer hasn't written any pose yet.
bool sb_shm_reader_get_pose(sb_shm_reader_t *r, sb_pose_t *out);

// Read the latest camera intrinsics from shared memory.
// NOT edge-triggered (latest-wins); returns false only if no intrinsics have
// ever been written. Used by the Monado driver to keep its render FOV matched
// to the live camera so the passthrough quad and frustum stay the same size.
bool sb_shm_reader_get_intrinsics(sb_shm_reader_t *r, sb_intrinsics_t *out);

// Close the reader (does NOT unlink — only the writer unlinks).
void sb_shm_reader_destroy(sb_shm_reader_t *r);

#ifdef __cplusplus
}  // extern "C"
#endif
