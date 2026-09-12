/* pose_shm.c — see pose_shm.h. */

#include "pose_shm.h"

#include <errno.h>
#include <fcntl.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

static void *
_open_map (int flags, mode_t mode)
{
    int fd = shm_open (SPATIAL_KBD_POSE_SHM_PATH, flags, mode);
    if (fd < 0) return NULL;
    /* Truncate/map to the FIXED object size, not sizeof — every process uses the
     * same value, so there's no size-mismatch race that could shrink the object
     * under another process's live mmap (SIGBUS). */
    if (ftruncate (fd, SPATIAL_KBD_POSE_SHM_BYTES) < 0) {
        close (fd);
        return NULL;
    }
    void *p = mmap (NULL, SPATIAL_KBD_POSE_SHM_BYTES,
                    PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close (fd);
    if (p == MAP_FAILED) return NULL;
    return p;
}

const spatial_kbd_pose_t *
pose_shm_open_reader (void)
{
    void *p = _open_map (O_RDWR | O_CREAT, 0666);
    return (const spatial_kbd_pose_t *) p;
}

spatial_kbd_pose_t *
pose_shm_open_writer (void)
{
    void *p = _open_map (O_RDWR | O_CREAT, 0666);
    return (spatial_kbd_pose_t *) p;
}

bool
pose_shm_read_snapshot (const spatial_kbd_pose_t *src, spatial_kbd_pose_t *out)
{
    if (!src || !out) return false;
    for (int attempt = 0; attempt < 16; ++attempt) {
        uint64_t s1 = atomic_load_explicit (&src->seq, memory_order_acquire);
        if (s1 & 1ULL) continue;
        memcpy (out, src, sizeof (*out));
        uint64_t s2 = atomic_load_explicit (&src->seq, memory_order_acquire);
        if (s1 == s2) return true;
    }
    return false;
}

void
pose_shm_write_begin (spatial_kbd_pose_t *dst)
{
    if (!dst) return;
    atomic_fetch_add_explicit (&dst->seq, 1, memory_order_release);
}

void
pose_shm_write_end (spatial_kbd_pose_t *dst)
{
    if (!dst) return;
    atomic_fetch_add_explicit (&dst->seq, 1, memory_order_release);
}
