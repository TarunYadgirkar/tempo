/* pose_shm.h — POSIX shm channel carrying per-frame fingertip data
 * from wxrd to the keyboard widget's decoder.
 *
 * Single-writer (wxrd), single-reader (spatial-keyboard) by design.
 * wxrd computes the fingertip-on-plane projection from the bridge-
 * receiver joint stream + the keyboard's anchored or floating plane,
 * writes it here at the same rate the gesture engine runs (~60 Hz).
 * The decoder polls each animation frame.
 *
 * The seqlock pattern (write: seq++; payload; seq++) lets the
 * reader retry if a write happens during a read.  Since wxrd writes
 * only one channel and the payload is small (~96 B), real contention
 * is rare.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SPATIAL_KEYBOARD_DECODER_POSE_SHM_H
#define SPATIAL_KEYBOARD_DECODER_POSE_SHM_H

#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPATIAL_KBD_POSE_SHM_PATH "/spatial_keyboard_pose"

/* Fixed shm object size, independent of the struct size, so EVERY process —
 * whatever its struct version — ftruncate()s to the SAME value.  That removes
 * the check-then-act race where a smaller-struct process could shrink the
 * object under a larger process's live mmap (SIGBUS).  Must stay >= the struct
 * size (asserted below); new fields grow into the slack, never resizing the
 * object. */
#define SPATIAL_KBD_POSE_SHM_BYTES 4096

/* Wire layout — keep in sync with wxrd-side writer.  Padding /
 * alignment kept explicit so the struct is binary-compatible across
 * the wxrd ↔ widget boundary. */
typedef struct {
    /* Seqlock: odd while writing, even when stable. */
    _Atomic uint64_t seq;
    /* Source timestamp (ns since CLOCK_MONOTONIC). */
    uint64_t timestamp_ns;
    /* True if the writer has the keyboard's plane available. */
    uint8_t  plane_valid;
    uint8_t  mode;            /* 0 = surface, 1 = midair */
    uint8_t  hand_present[2]; /* [left, right] */
    uint8_t  reserved[3];
    /* Per-finger plane-local coords.  Index 0..4 = left hand fingers
     * (thumb, index, middle, ring, pinky); 5..9 = right hand.
     * x, y in metres (keyboard plane-local, origin at top-left); z
     * is signed distance ABOVE the plane (positive = above). */
    float    fingertip[10][3];
    /* Keyboard plane field size in metres, published by the writer so the
     * reader derives the plane-local → surface mapping instead of hardcoding a
     * copy that silently desyncs if wxrd ever changes the plane size.  Zero
     * until the first write — readers must fall back to a sane default. */
    float    plane_w;
    float    plane_h;
} spatial_kbd_pose_t;

/* This struct and the inlined copy in wxrd's keyboard_pose_writer.c share the
 * same shm and MUST stay byte-identical.  Pin the size so any field drift on
 * either side fails to compile instead of silently corrupting reads. */
_Static_assert (sizeof (spatial_kbd_pose_t) == 152,
                "spatial_kbd_pose_t size changed — sync pose_shm.h with "
                "vendor/wxrd/src/keyboard_pose_writer.c");
_Static_assert (sizeof (spatial_kbd_pose_t) <= SPATIAL_KBD_POSE_SHM_BYTES,
                "struct exceeds the fixed shm size — bump "
                "SPATIAL_KBD_POSE_SHM_BYTES");

/* Open the shm region for read (called by widget).  On first call,
 * the region is created if it doesn't exist (zero-init); subsequent
 * callers just attach.  Returns NULL on failure (errno set). */
const spatial_kbd_pose_t *pose_shm_open_reader (void);
/* Symmetric for the writer side (called by wxrd). */
spatial_kbd_pose_t       *pose_shm_open_writer (void);

/* Atomically snapshot a stable copy of *src into *out.  Returns true
 * if a consistent snapshot was obtained, false after too many
 * concurrent-write retries. */
bool pose_shm_read_snapshot (const spatial_kbd_pose_t *src,
                             spatial_kbd_pose_t *out);

/* Writer helper: bracket a write with begin/end so the seqlock is
 * advanced atomically. */
void pose_shm_write_begin (spatial_kbd_pose_t *dst);
void pose_shm_write_end   (spatial_kbd_pose_t *dst);

#ifdef __cplusplus
}
#endif

#endif
