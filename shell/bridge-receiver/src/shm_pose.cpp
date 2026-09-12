// shm_pose.cpp — Cross-process pose sharing via POSIX shared memory + seqlock.
//
// Writer (wxrd compositor) creates /spatial_bridge_pose shared memory,
// updates it each frame with the latest ARKit pose.
// Reader (Monado driver) opens the same segment and reads poses lock-free.
//
// Memory layout: a seqlock (atomic uint32_t) followed by an sb_pose_t.
// The seqlock pattern is identical to WorldModel's pose seqlock.

#include "spatial_bridge.h"

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <new>      // std::nothrow (libc++ does not include it transitively)
#include <thread>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

// ---------------------------------------------------------------------------
// Shared memory layout — must be identical in writer and reader
// ---------------------------------------------------------------------------

struct ShmPoseLayout {
    std::atomic<uint32_t> seq;  // seqlock: even = idle, odd = write in progress
    uint32_t _pad;              // align pose to 8-byte boundary
    sb_pose_t pose;             // ~40 bytes

    // Camera intrinsics share the same segment but use an INDEPENDENT seqlock,
    // so pose (60 Hz) and intrinsics (their own rate) never block each other.
    // Appended after the pose so the pose layout/offsets are unchanged.
    std::atomic<uint32_t> intr_seq;
    uint32_t _pad2;             // align intrinsics (leads with uint64_t) to 8
    sb_intrinsics_t intrinsics;
};

static_assert(sizeof(std::atomic<uint32_t>) == sizeof(uint32_t),
              "atomic<uint32_t> must be lock-free and same size as uint32_t");

static constexpr size_t SHM_SIZE = sizeof(ShmPoseLayout);

// A writer that dies between its two seq increments leaves the segment odd
// forever; an uncapped reader would then spin the compositor thread for good.
static constexpr int SHM_SEQLOCK_MAX_SPINS = 64;

// Returns false if the value was never written (seq still 0) or the seqlock
// never settled within SHM_SEQLOCK_MAX_SPINS.
template <typename T>
static bool seqlock_read(const std::atomic<uint32_t> &seq, const T &src, T *out) {
    for (int spin = 0; spin < SHM_SEQLOCK_MAX_SPINS; ++spin) {
        uint32_t seq1 = seq.load(std::memory_order_acquire);
        if (seq1 & 1u) {
            std::this_thread::yield();
            continue;
        }
        T snapshot = src;
        std::atomic_thread_fence(std::memory_order_acquire);
        if (seq.load(std::memory_order_acquire) != seq1) {
            std::this_thread::yield();
            continue;
        }
        if (seq1 == 0) return false;
        *out = snapshot;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Writer
// ---------------------------------------------------------------------------

struct sb_shm_writer_impl_t {
    ShmPoseLayout *layout;  // mmap'd shared memory
    int fd;                 // shm file descriptor
    char name[128];         // shm segment name (e.g. "/spatial_bridge_pose")
};

extern "C" sb_shm_writer_t *sb_shm_writer_create(const char *name) {
    if (!name || name[0] != '/') {
        std::fprintf(stderr, "sb_shm_writer_create: name must start with '/' (got '%s')\n",
                     name ? name : "(null)");
        return nullptr;
    }

    // Remove stale segment from a previous crash
    shm_unlink(name);

    int fd = shm_open(name, O_CREAT | O_RDWR, 0644);
    if (fd < 0) {
        std::fprintf(stderr, "sb_shm_writer_create: shm_open('%s') failed: %s\n", name,
                     std::strerror(errno));
        return nullptr;
    }

    if (ftruncate(fd, static_cast<off_t>(SHM_SIZE)) < 0) {
        std::fprintf(stderr, "sb_shm_writer_create: ftruncate failed: %s\n", std::strerror(errno));
        close(fd);
        shm_unlink(name);
        return nullptr;
    }

    void *mem = mmap(nullptr, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (mem == MAP_FAILED) {
        std::fprintf(stderr, "sb_shm_writer_create: mmap failed: %s\n", std::strerror(errno));
        close(fd);
        shm_unlink(name);
        return nullptr;
    }

    // Zero-init the shared memory (seq=0, pose zeroed)
    std::memset(mem, 0, SHM_SIZE);

    auto *w = new (std::nothrow) sb_shm_writer_impl_t();
    if (!w) {
        munmap(mem, SHM_SIZE);
        close(fd);
        shm_unlink(name);
        return nullptr;
    }

    w->layout = static_cast<ShmPoseLayout *>(mem);
    w->fd     = fd;
    std::snprintf(w->name, sizeof(w->name), "%s", name);

    return w;
}

extern "C" void sb_shm_writer_update(sb_shm_writer_t *w, const sb_pose_t *pose) {
    if (!w || !pose) return;

    ShmPoseLayout *l = w->layout;

    // Seqlock write: odd = write in progress, even = idle.  The first increment
    // must be acq_rel: release alone would let the pose store float above it.
    l->seq.fetch_add(1, std::memory_order_acq_rel);  // seq → odd
    l->pose = *pose;
    l->seq.fetch_add(1, std::memory_order_release);  // seq → even
}

extern "C" void sb_shm_writer_update_intrinsics(sb_shm_writer_t *w,
                                                const sb_intrinsics_t *intr) {
    if (!w || !intr) return;

    ShmPoseLayout *l = w->layout;

    // Independent seqlock from the pose: odd = write in progress, even = idle.
    l->intr_seq.fetch_add(1, std::memory_order_acq_rel);  // intr_seq → odd
    l->intrinsics = *intr;
    l->intr_seq.fetch_add(1, std::memory_order_release);  // intr_seq → even
}

extern "C" void sb_shm_writer_destroy(sb_shm_writer_t *w) {
    if (!w) return;

    if (w->layout) {
        munmap(w->layout, SHM_SIZE);
    }
    if (w->fd >= 0) {
        close(w->fd);
    }
    // Remove the segment so it doesn't linger after shutdown
    shm_unlink(w->name);

    delete w;
}

// ---------------------------------------------------------------------------
// Reader
// ---------------------------------------------------------------------------

struct sb_shm_reader_impl_t {
    ShmPoseLayout *layout;  // mmap'd shared memory (read-only)
    int fd;                 // shm file descriptor
};

extern "C" sb_shm_reader_t *sb_shm_reader_open(const char *name) {
    if (!name || name[0] != '/') {
        std::fprintf(stderr, "sb_shm_reader_open: name must start with '/'\n");
        return nullptr;
    }

    int fd = shm_open(name, O_RDONLY, 0);
    if (fd < 0) {
        // Not an error during startup — the writer may not exist yet
        return nullptr;
    }

    void *mem = mmap(nullptr, SHM_SIZE, PROT_READ, MAP_SHARED, fd, 0);
    if (mem == MAP_FAILED) {
        std::fprintf(stderr, "sb_shm_reader_open: mmap failed: %s\n", std::strerror(errno));
        close(fd);
        return nullptr;
    }

    auto *r = new (std::nothrow) sb_shm_reader_impl_t();
    if (!r) {
        munmap(mem, SHM_SIZE);
        close(fd);
        return nullptr;
    }

    r->layout = static_cast<ShmPoseLayout *>(mem);
    r->fd     = fd;

    return r;
}

extern "C" bool sb_shm_reader_get_pose(sb_shm_reader_t *r, sb_pose_t *out) {
    if (!r || !out) return false;

    // Unlike sb_get_latest_pose(), this is NOT edge-triggered: the Monado
    // driver needs the latest pose every frame for smooth tracking.
    const ShmPoseLayout *l = r->layout;
    return seqlock_read(l->seq, l->pose, out);
}

extern "C" bool sb_shm_reader_get_intrinsics(sb_shm_reader_t *r, sb_intrinsics_t *out) {
    if (!r || !out) return false;

    // Latest-wins (NOT edge-triggered).
    const ShmPoseLayout *l = r->layout;
    return seqlock_read(l->intr_seq, l->intrinsics, out);
}

extern "C" void sb_shm_reader_destroy(sb_shm_reader_t *r) {
    if (!r) return;

    if (r->layout) {
        munmap(const_cast<ShmPoseLayout *>(r->layout), SHM_SIZE);
    }
    if (r->fd >= 0) {
        close(r->fd);
    }
    delete r;
}
