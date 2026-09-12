// world_model.cpp — Thread-safe world state storage for bridge-receiver.
//
// Pose updates use the seqlock pattern (atomic sequence counter) for lock-free,
// low-latency reads on the compositor hot path.
//
// Plane state uses a mutex-protected flat array (up to MAX_PLANES entries).
// This is acceptable because plane updates arrive at ~1-5 Hz.
//
// Frame state uses a mutex + pointer swap; the JPEG rgba buffer is owned by
// the world model until the caller retrieves it via sb_get_latest_frame().

#include "world_model.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>

#include "spatial_bridge.h"
#include "hand_reconstruct.h"

// ---------------------------------------------------------------------------
// WorldModel implementation
// ---------------------------------------------------------------------------

static constexpr int MAX_PLANES = 64;

struct WorldModel {
    // --- Pose (seqlock) ---
    // seq is even when no write is in progress; odd means a write is active.
    // Readers spin until seq is even and stable across the read.
    std::atomic<uint32_t> pose_seq{0};
    sb_pose_t pose{};
    // "seen" sequence: the last pose_seq value that sb_get_latest_pose() observed.
    // Stored per-receiver (not per-caller) — callers who want edge-triggered
    // behaviour should track their own last-seen timestamp instead.
    std::atomic<uint32_t> pose_last_seen{0};
    // Mirrors pose.session_epoch, but level-triggered: consumers poll it every
    // frame to notice an ARKit world re-init, which the edge-triggered
    // world_model_get_pose() would hide whenever no new pose arrived.
    std::atomic<uint32_t> session_epoch{0};

    // --- Planes (mutex) ---
    mutable std::mutex planes_mutex;
    sb_plane_t planes[MAX_PLANES]{};
    int plane_count{0};

    // --- Hands (mutex, two entries: left=0, right=1) ---
    mutable std::mutex hands_mutex;
    sb_hand_t hands[2]{};
    bool hand_fresh[2]{false, false};
    // Per-hand temporal state for bone-length depth reconstruction (one per
    // hand). Only touched on the single packet thread in update_hand.
    hr_state_t *hr_state[2]{nullptr, nullptr};

    // --- Intrinsics (mutex, sticky latest — NOT edge-triggered) ---
    mutable std::mutex intrinsics_mutex;
    sb_intrinsics_t intrinsics{};
    bool intrinsics_valid{false};

    // --- Frame (mutex + pointer) ---
    mutable std::mutex frame_mutex;
    sb_frame_t pending_frame{};  // rgba is non-null when a new frame is ready
    bool frame_fresh{false};
    // Ingest time of the newest frame, independent of consumption: the single
    // consumer clears frame_fresh, so nothing else could otherwise tell a live
    // stream from one that died on a frame the renderer already drew.
    std::atomic<int64_t> last_frame_ns{-1};

    // --- Depth map (mutex + pointer; mirrors Frame; edge-triggered) ---
    mutable std::mutex depth_mutex;
    sb_depth_t pending_depth{};  // depth is non-null when a new map is ready
    bool depth_fresh{false};

    // --- Packet rate (lock-free, non-destructive reads) ---
    // Ring of fixed-duration buckets: the UDP thread bumps the current
    // bucket, readers sum the completed ones.  Reads must not mutate the
    // window — mac-shell polls this every rendered frame *and* the control
    // socket's `stats` verb reads it from another thread.  The previous
    // query-time windowing rolled the window on every call, so whichever
    // caller arrived second saw only the packets since the other's call
    // (~16 ms at 60 fps) and reported 0.0 pkt/s on a live 58 pkt/s link.
    // Still no mutex: the pre-2026 mutex-protected 512-entry ring added
    // lock contention on every received packet (~200+/sec).
    static constexpr int RATE_BUCKETS = 11;                 // 10 whole + current
    static constexpr int64_t RATE_BUCKET_NS = 100000000LL;  // 100 ms
    std::atomic<uint64_t> rate_count{0};  // monotonic total, diagnostics only
    std::atomic<uint64_t> rate_bucket_count[RATE_BUCKETS]{};
    std::atomic<int64_t> rate_bucket_slot[RATE_BUCKETS]{};
};

// ---------------------------------------------------------------------------
// Factory / destructor
// ---------------------------------------------------------------------------

WorldModel *world_model_create() {
    auto *wm = new (std::nothrow) WorldModel();
    if (!wm) {
        std::fprintf(stderr, "spatial_bridge: out of memory allocating WorldModel\n");
        return wm;
    }
    wm->hr_state[0] = hr_state_create();
    wm->hr_state[1] = hr_state_create();
    // -1 marks "never written": slot 0 is a real (if long-past) bucket index.
    for (int i = 0; i < WorldModel::RATE_BUCKETS; ++i)
        wm->rate_bucket_slot[i].store(-1, std::memory_order_relaxed);
    return wm;
}

void world_model_destroy(WorldModel *wm) {
    if (!wm) return;
    // Release any pending frame buffer.
    std::lock_guard<std::mutex> lk(wm->frame_mutex);
    if (wm->pending_frame.rgba) {
        std::free(wm->pending_frame.rgba);
        wm->pending_frame.rgba = nullptr;
    }
    {
        std::lock_guard<std::mutex> dlk(wm->depth_mutex);
        if (wm->pending_depth.depth) {
            std::free(wm->pending_depth.depth);
            wm->pending_depth.depth = nullptr;
        }
    }
    hr_state_destroy(wm->hr_state[0]);
    hr_state_destroy(wm->hr_state[1]);
    delete wm;
}

// ---------------------------------------------------------------------------
// Pose updates (called from UDP/file thread)
// ---------------------------------------------------------------------------

void world_model_update_pose(WorldModel *wm, const sb_pose_t &pose) {
    // seqlock write: set odd before writing, even after.
    // load+store is cheaper than fetch_add on x86 (no LOCK prefix)
    // because there is a single writer.
    uint32_t s = wm->pose_seq.load(std::memory_order_relaxed);
    wm->pose_seq.store(s + 1, std::memory_order_release);
    wm->pose = pose;
    wm->pose_seq.store(s + 2, std::memory_order_release);
    wm->session_epoch.store(pose.session_epoch, std::memory_order_relaxed);
}

uint32_t world_model_get_session_epoch(WorldModel *wm) {
    return wm->session_epoch.load(std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Plane updates (called from UDP/file thread)
// ---------------------------------------------------------------------------

void world_model_update_plane(WorldModel *wm, const sb_plane_t &plane) {
    std::lock_guard<std::mutex> lk(wm->planes_mutex);

    if (plane.is_removed) {
        // Evict by UUID — find and swap-remove.
        for (int i = 0; i < wm->plane_count; ++i) {
            if (std::memcmp(wm->planes[i].uuid, plane.uuid, 16) == 0) {
                wm->planes[i] = wm->planes[wm->plane_count - 1];
                --wm->plane_count;
                return;
            }
        }
        // Not found — ignore spurious remove.
        return;
    }

    // Update existing or insert new.
    for (int i = 0; i < wm->plane_count; ++i) {
        if (std::memcmp(wm->planes[i].uuid, plane.uuid, 16) == 0) {
            wm->planes[i] = plane;
            return;
        }
    }
    if (wm->plane_count < MAX_PLANES) {
        wm->planes[wm->plane_count++] = plane;
    } else {
        std::fprintf(stderr, "spatial_bridge: plane table full (%d entries), dropping\n",
                     MAX_PLANES);
    }
}

// ---------------------------------------------------------------------------
// Frame updates (called from UDP/file thread)
// ---------------------------------------------------------------------------

static int64_t steady_now_ns();

// Takes ownership of rgba (caller must not free it).
void world_model_update_frame(WorldModel *wm, const sb_frame_t &frame) {
    void *old_rgba = nullptr;
    {
        std::lock_guard<std::mutex> lk(wm->frame_mutex);
        old_rgba = wm->pending_frame.rgba;
        wm->pending_frame = frame;
        wm->frame_fresh   = true;
    }
    wm->last_frame_ns.store(steady_now_ns(), std::memory_order_relaxed);
    std::free(old_rgba);
}

// Takes ownership of depth.depth (malloc'd float buffer). Mirrors update_frame.
void world_model_update_depth(WorldModel *wm, const sb_depth_t &depth) {
    void *old_depth = nullptr;
    {
        std::lock_guard<std::mutex> lk(wm->depth_mutex);
        old_depth = wm->pending_depth.depth;
        wm->pending_depth = depth;
        wm->depth_fresh   = true;
    }
    std::free(old_depth);
}

// ---------------------------------------------------------------------------
// Hand updates (called from UDP/file thread)
// ---------------------------------------------------------------------------

void world_model_update_hand(WorldModel *wm, const sb_hand_t &hand) {
    int idx = hand.hand_index <= 1 ? hand.hand_index : 0;

    // Bone-length depth reconstruction. The iPhone streams raw LiDAR-unprojected
    // joints, which are roughly metric but ~10 mm noisy — too noisy for the
    // gesture engine's metric thresholds. hr_reconstruct snaps each joint onto
    // its camera ray at the anatomically-correct bone length. See
    // docs/hand-tracking.md.
    //
    // It reads the latest pose (wm->pose) directly: update_pose and update_hand
    // both run on the single packet-dispatch thread, so there is no concurrent
    // writer here — the pose seqlock guards only the other-thread compositor
    // reader. With no pose yet (zero-initialised, identity-zero quat),
    // hr_reconstruct no-ops and joints pass through verbatim, which is what the
    // pose-free wire-decode tests rely on.
    sb_hand_t h = hand;  // mutable copy for in-place reconstruction
    hr_reconstruct(h.joints, wm->pose.rot, wm->pose.pos, h.timestamp_ns,
                   wm->hr_state[idx]);

    std::lock_guard<std::mutex> lk(wm->hands_mutex);
    wm->hands[idx]      = h;
    wm->hand_fresh[idx] = true;
}

// ---------------------------------------------------------------------------
// Intrinsics updates (called from UDP/file thread)
// ---------------------------------------------------------------------------

void world_model_update_intrinsics(WorldModel *wm, const sb_intrinsics_t &intr) {
    std::lock_guard<std::mutex> lk(wm->intrinsics_mutex);
    wm->intrinsics       = intr;
    wm->intrinsics_valid = true;
}

// ---------------------------------------------------------------------------
// Packet rate tracking
// ---------------------------------------------------------------------------

// Bucket index for "now". Only the UDP thread writes buckets, so the
// read-modify-write below needs no CAS.
static int64_t steady_now_ns() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               now.time_since_epoch())
        .count();
}

static int64_t rate_slot_now() {
    return steady_now_ns() / WorldModel::RATE_BUCKET_NS;
}

void world_model_record_packet(WorldModel *wm) {
    wm->rate_count.fetch_add(1, std::memory_order_relaxed);

    const int64_t slot = rate_slot_now();
    const int idx = static_cast<int>(slot % WorldModel::RATE_BUCKETS);
    if (wm->rate_bucket_slot[idx].load(std::memory_order_relaxed) != slot) {
        // Recycling a bucket from an older revolution: restart its count.
        wm->rate_bucket_count[idx].store(1, std::memory_order_relaxed);
        wm->rate_bucket_slot[idx].store(slot, std::memory_order_release);
    } else {
        wm->rate_bucket_count[idx].fetch_add(1, std::memory_order_relaxed);
    }
}

// ---------------------------------------------------------------------------
// Query functions (called from consumer threads)
// ---------------------------------------------------------------------------

bool world_model_get_pose(WorldModel *wm, sb_pose_t *out) {
    // Seqlock read loop.
    // Invariant: we only accept a snapshot when seq1 == seq2 and both are even,
    // meaning no write was in progress during our struct read.
    //
    // Note: do NOT use 'continue' inside a do-while here — that would jump to the
    // condition check with seq2 uninitialized, which is undefined behaviour. Use
    // an explicit while(true) + inner if/else to guarantee seq2 is always
    // initialised before comparison.
    sb_pose_t snapshot;
    uint32_t seq1, seq2;
    while (true) {
        seq1 = wm->pose_seq.load(std::memory_order_acquire);
        if (seq1 & 1u) {
            // Write in progress — spin without reading the struct.
            continue;
        }
        // Copy the struct while seq is even.
        snapshot = wm->pose;
        // Acquire fence: ensure all struct loads complete before we re-read seq.
        std::atomic_thread_fence(std::memory_order_acquire);
        seq2 = wm->pose_seq.load(std::memory_order_acquire);
        if (seq1 == seq2) break;  // consistent read — accept snapshot
        // seq changed during our read — retry.
    }

    // Return false if no new pose has arrived since the last successful call.
    uint32_t last = wm->pose_last_seen.load(std::memory_order_relaxed);
    if (seq1 == last) return false;

    *out = snapshot;
    wm->pose_last_seen.store(seq1, std::memory_order_relaxed);
    return true;
}

bool world_model_get_frame(WorldModel *wm, sb_frame_t *out) {
    std::lock_guard<std::mutex> lk(wm->frame_mutex);
    if (!wm->frame_fresh || !wm->pending_frame.rgba) return false;
    *out = wm->pending_frame;
    // Transfer ownership to caller — clear our pointer so we don't double-free.
    wm->pending_frame.rgba = nullptr;
    wm->pending_frame      = {};
    wm->frame_fresh        = false;
    return true;
}

bool world_model_get_depth(WorldModel *wm, sb_depth_t *out) {
    std::lock_guard<std::mutex> lk(wm->depth_mutex);
    if (!wm->depth_fresh || !wm->pending_depth.depth) return false;
    *out = wm->pending_depth;
    // Transfer ownership to caller — clear our pointer so we don't double-free.
    wm->pending_depth.depth = nullptr;
    wm->pending_depth       = {};
    wm->depth_fresh         = false;
    return true;
}

int world_model_get_planes(WorldModel *wm, sb_plane_t *out, int max_planes) {
    std::lock_guard<std::mutex> lk(wm->planes_mutex);
    int n = (wm->plane_count < max_planes) ? wm->plane_count : max_planes;
    std::memcpy(out, wm->planes, sizeof(sb_plane_t) * static_cast<size_t>(n));
    return n;
}

bool world_model_get_hand(WorldModel *wm, int hand_index, sb_hand_t *out) {
    if (hand_index < 0 || hand_index > 1) return false;
    std::lock_guard<std::mutex> lk(wm->hands_mutex);
    if (!wm->hand_fresh[hand_index]) return false;
    *out                       = wm->hands[hand_index];
    wm->hand_fresh[hand_index] = false;
    return true;
}

bool world_model_get_intrinsics(WorldModel *wm, sb_intrinsics_t *out) {
    std::lock_guard<std::mutex> lk(wm->intrinsics_mutex);
    if (!wm->intrinsics_valid) return false;
    *out = wm->intrinsics;  // latest-wins; NOT edge-triggered
    return true;
}

int64_t world_model_get_frame_age_ms(WorldModel *wm) {
    const int64_t last = wm->last_frame_ns.load(std::memory_order_relaxed);
    if (last < 0) return -1;
    const int64_t age_ns = steady_now_ns() - last;
    return age_ns < 0 ? 0 : age_ns / 1000000;
}

float world_model_get_packet_rate(WorldModel *wm) {
    // Sum the completed buckets only — the in-progress one is a partial
    // sample and would drag the reported rate down by however far into it
    // the caller happens to land. Purely a read: any number of threads may
    // call this as often as they like without disturbing each other.
    const int64_t slot_now = rate_slot_now();
    const int64_t oldest = slot_now - (WorldModel::RATE_BUCKETS - 1);

    uint64_t sum = 0;
    for (int i = 0; i < WorldModel::RATE_BUCKETS; ++i) {
        const int64_t slot = wm->rate_bucket_slot[i].load(std::memory_order_acquire);
        if (slot >= oldest && slot < slot_now)
            sum += wm->rate_bucket_count[i].load(std::memory_order_relaxed);
    }

    // Whole-bucket window, so a stream that just started ramps up over the
    // first second rather than reading high on a fraction of a bucket.
    constexpr float window_s = static_cast<float>(WorldModel::RATE_BUCKETS - 1) *
                               static_cast<float>(WorldModel::RATE_BUCKET_NS) / 1e9f;
    return static_cast<float>(sum) / window_s;
}
