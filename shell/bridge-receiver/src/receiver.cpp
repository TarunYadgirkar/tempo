// receiver.cpp — UDP socket receiver and file replay for bridge-receiver.
//
// Parses iPhone ARKit UDP packets (wire protocol defined in CLAUDE.md / wire-protocol.md)
// and pushes decoded data into the WorldModel.
//
// Two modes:
//   Live: opens a UDP socket, receives datagrams in a background thread.
//   File: reads a .bin file of recorded datagrams, replays with original timing.
//
// All multi-byte reads from packet buffers use memcpy to avoid strict-aliasing UB.
// No exceptions — errors are printed to stderr and the packet is discarded.

#include "spatial_bridge.h"
#include "world_model.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// JPEG decompression via libjpeg (libjpeg-turbo provides this API).
#include <jpeglib.h>
#include <setjmp.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

static constexpr uint8_t PKT_POSE  = 0x01;
static constexpr uint8_t PKT_PLANE = 0x02;
static constexpr uint8_t PKT_FRAME = 0x03;
static constexpr uint8_t PKT_MESH  = 0x04;
static constexpr uint8_t PKT_HAND  = 0x05;
// 0x06/0x07/0x08 are RETIRED recording-control types (may appear in old .bin
// files) — never reuse them.  0x09 is the chunked camera frame (below).
static constexpr uint8_t PKT_FRAME_CHUNK = 0x09;
static constexpr uint8_t PKT_INTRINSICS  = 0x0A;
static constexpr uint8_t PKT_DEPTH       = 0x0B;
// 0x0C is the ONLY Mac->iPhone type: a 1 Hz liveness beacon so the phone can
// tell "my datagrams are landing" from "I am shouting into a void".  Every
// other type flows iPhone->Mac.
static constexpr uint8_t PKT_HEARTBEAT      = 0x0C;
static constexpr size_t HEARTBEAT_SIZE      = 13;
static constexpr int64_t HEARTBEAT_PERIOD_MS = 1000;

// Chunked-frame (0x09) wire layout.  A JPEG frame is split into
// FRAME_CHUNK_PAYLOAD-byte pieces, each sent as its own standalone datagram
// so it stays under the ~1472-byte MTU and never IP-fragments (the whole
// point — see the reassembly notes).  Header (19 bytes):
//   [0]      type 0x09
//   [1..8]   timestamp_ns (u64)  — frame id, identical across a frame's chunks
//   [9..12]  total_size  (u32)   — total JPEG byte count
//   [13..16] offset      (u32)   — byte offset of this chunk in the JPEG
//   [17..18] chunk_len   (u16)   — payload bytes that follow
//   [19..]   payload
// FRAME_CHUNK_PAYLOAD MUST match the iOS sender (UDPStreamer.swift).
static constexpr size_t FRAME_CHUNK_HEADER_SIZE = 19;
static constexpr uint32_t FRAME_CHUNK_PAYLOAD   = 1200;
// Reject absurd total_size early (a 640x480 q=high JPEG is well under this).
static constexpr uint32_t MAX_FRAME_BYTES = 1u << 20;  // 1 MiB
// Cap on decoded JPEG dimensions.  A SOF header is attacker-controlled: a
// 65535x65535 claim would make width*height*4 overflow uint32 and malloc a
// buffer far smaller than the scanlines libjpeg then writes into it.
static constexpr uint32_t MAX_JPEG_DIM        = 4096;
static constexpr uint64_t MAX_JPEG_RGBA_BYTES = uint64_t(MAX_JPEG_DIM) * MAX_JPEG_DIM * 4u;
// Boundary (ns) separating a "late straggler" from a "stale in-progress id" on
// a BACKWARD timestamp step during reassembly.  Frames are ~33 ms apart at
// 30 Hz; 2 s is vast headroom for reorder/clock skew.  A chunk older than the
// in-progress frame by <= this is a straggler from a just-superseded frame
// (ignored); older by more than this means the in-progress id is itself bogus
// (e.g. a bit-flip put a prior chunk's timestamp far in the future) and must be
// abandoned so real traffic isn't locked out.  Forward steps are NEVER rejected
// (see parse_frame_chunk) — only this backward case consults the threshold.
static constexpr uint64_t FRAME_TS_MAX_SKEW_NS = 2000000000ULL;  // 2 s

// Pose grew a u32 session_epoch tail; 41-byte senders/recordings stay valid and
// decode as epoch 0 ("unknown", assume world-frame continuity).
static constexpr size_t POSE_PACKET_SIZE_V1 = 41;
static constexpr size_t POSE_PACKET_SIZE    = 45;
static constexpr size_t PLANE_PACKET_SIZE = 59;
static constexpr size_t FRAME_HEADER_SIZE = 21;
// Hand packet: 1 (type) + 8 (ts) + 1 (hand_index) + 1 (joint_count) + 21*5*4 (joints) = 431
static constexpr size_t HAND_PACKET_SIZE = 431;
// Intrinsics packet: 1 (type) + 8 (ts) + 6*4 (fx,fy,cx,cy,img_w,img_h) = 33
static constexpr size_t INTRINSICS_PACKET_SIZE = 33;
// Depth (0x0B) reassembled-payload sub-header: u16 w, u16 h, u8 enc, f32 z_min, f32 z_max.
static constexpr size_t DEPTH_SUBHEADER_SIZE = 13;
// Generous cap on a reassembled depth payload (sub-header + up to a 1024x1024 u16 plane).
static constexpr uint32_t MAX_DEPTH_BYTES = DEPTH_SUBHEADER_SIZE + (2u * 1024u * 1024u);

// Maximum UDP datagram we'll accept (slightly above a full mesh chunk with 600 verts).
// 41 + 12*600 + 12*200 (vertices + triangles) = 41 + 7200 + 2400 = 9641 bytes.
// Add a generous margin.
static constexpr size_t MAX_DATAGRAM = 65536;

// ---------------------------------------------------------------------------
// Little-endian helpers (all use memcpy — no aliasing UB)
// ---------------------------------------------------------------------------

static inline uint16_t read_u16_le(const uint8_t *p) {
    uint16_t v;
    std::memcpy(&v, p, sizeof(v));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    v = __builtin_bswap16(v);
#endif
    return v;
}

static inline uint32_t read_u32_le(const uint8_t *p) {
    uint32_t v;
    std::memcpy(&v, p, sizeof(v));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    v = __builtin_bswap32(v);
#endif
    return v;
}

static inline uint64_t read_u64_le(const uint8_t *p) {
    uint64_t v;
    std::memcpy(&v, p, sizeof(v));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    v = __builtin_bswap64(v);
#endif
    return v;
}

static inline float read_f32_le(const uint8_t *p) {
    uint32_t bits = read_u32_le(p);
    float v;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

static inline void write_u32_le(uint8_t *p, uint32_t v) {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    v = __builtin_bswap32(v);
#endif
    std::memcpy(p, &v, sizeof(v));
}

static inline void write_u64_le(uint8_t *p, uint64_t v) {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    v = __builtin_bswap64(v);
#endif
    std::memcpy(p, &v, sizeof(v));
}

static inline bool all_finite(const float *v, size_t n) {
    for (size_t i = 0; i < n; ++i)
        if (!std::isfinite(v[i])) return false;
    return true;
}

// Packets with NaN/Inf or degenerate geometry are dropped before they reach the
// world model; consumers (driver, compositor) assume finite values.
static thread_local uint64_t g_dropped_invalid = 0;

static void drop_invalid(const char *what) {
    if (g_dropped_invalid++ % 1000 == 0)
        std::fprintf(stderr,
                     "spatial_bridge: dropping %s packet with invalid floats (%llu so far)\n", what,
                     (unsigned long long)g_dropped_invalid);
}

// ---------------------------------------------------------------------------
// JPEG decompression (libjpeg C API)
// ---------------------------------------------------------------------------

// Custom error manager to prevent libjpeg from calling exit() on error.
struct JpegErrorMgr {
    struct jpeg_error_mgr pub;  // must be first
    jmp_buf setjmp_buf;
};

static void jpeg_error_exit_cb(j_common_ptr cinfo) {
    auto *mgr = reinterpret_cast<JpegErrorMgr *>(cinfo->err);
    longjmp(mgr->setjmp_buf, 1);
}

struct JpegDecoder {
    jpeg_decompress_struct cinfo{};
    JpegErrorMgr jerr{};
    std::vector<uint8_t> row_buf;
    uint8_t *rgba{nullptr};
    uint64_t oversize_count{0};
    bool initialized{false};

    JpegDecoder() {
        cinfo.err = jpeg_std_error(&jerr.pub);
        jerr.pub.error_exit = jpeg_error_exit_cb;
    }
    ~JpegDecoder() {
        std::free(rgba);
        if (cinfo.mem) jpeg_destroy_decompress(&cinfo);
    }
    JpegDecoder(const JpegDecoder &) = delete;
    JpegDecoder &operator=(const JpegDecoder &) = delete;
};

// State belongs to one worker and is reused across its frames.
static uint8_t *jpeg_decode_rgba(JpegDecoder &decoder, const uint8_t *jpeg_data,
                                 uint32_t jpeg_size, uint32_t *out_width,
                                 uint32_t *out_height, uint32_t *out_rgba_size) {
    auto &cinfo = decoder.cinfo;
    auto &jerr = decoder.jerr;
    if (setjmp(jerr.setjmp_buf)) {
        char msg[JMSG_LENGTH_MAX];
        (*cinfo.err->format_message)(reinterpret_cast<j_common_ptr>(&cinfo), msg);
        std::fprintf(stderr, "spatial_bridge: JPEG decode error: %s\n", msg);
        std::free(decoder.rgba);
        decoder.rgba = nullptr;
        jpeg_abort_decompress(&cinfo);
        return nullptr;
    }
    if (!decoder.initialized) {
        jpeg_create_decompress(&cinfo);
        decoder.initialized = true;
    }

    jpeg_mem_src(&cinfo, jpeg_data, jpeg_size);

    if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
        std::fprintf(stderr, "spatial_bridge: JPEG read_header failed\n");
        jpeg_abort_decompress(&cinfo);
        return nullptr;
    }

    jpeg_calc_output_dimensions(&cinfo);
    {
        uint64_t w      = cinfo.output_width;
        uint64_t h      = cinfo.output_height;
        uint64_t nbytes = w * h * 4u;
        if (w == 0 || h == 0 || w > MAX_JPEG_DIM || h > MAX_JPEG_DIM ||
            nbytes > MAX_JPEG_RGBA_BYTES) {
            auto &oversize_count = decoder.oversize_count;
            if (oversize_count++ % 1000 == 0)
                std::fprintf(stderr,
                             "spatial_bridge: rejecting JPEG with dimensions %llux%llu "
                             "(cap %ux%u; %llu rejected so far)\n",
                             (unsigned long long)w, (unsigned long long)h, MAX_JPEG_DIM,
                             MAX_JPEG_DIM, (unsigned long long)oversize_count);
            jpeg_abort_decompress(&cinfo);
            return nullptr;
        }
    }

    // Request RGBA output (libjpeg natively produces JCS_EXT_RGBA if compiled
    // with libjpeg-turbo; fall back to RGB and manually expand otherwise).
    // We try RGBA first; if the library rejects it we fall back to RGB.
    cinfo.out_color_space = JCS_EXT_RGBA;
    jpeg_start_decompress(&cinfo);

    bool is_rgba = (cinfo.output_components == 4);
    if (!is_rgba) {
        // Restart with RGB and convert manually below.
        jpeg_abort_decompress(&cinfo);
        cinfo.out_color_space = JCS_RGB;
        jpeg_start_decompress(&cinfo);
    }

    uint32_t width      = static_cast<uint32_t>(cinfo.output_width);
    uint32_t height     = static_cast<uint32_t>(cinfo.output_height);
    int comps           = cinfo.output_components;  // 3 or 4
    size_t row_stride   = static_cast<size_t>(width) * static_cast<size_t>(comps);
    size_t rgba_size    = static_cast<size_t>(width) * height * 4u;

    // Allocate output buffer (always RGBA — 4 bytes/pixel).
    decoder.rgba = static_cast<uint8_t *>(std::malloc(rgba_size));
    auto *rgba = decoder.rgba;
    if (!rgba) {
        std::fprintf(stderr, "spatial_bridge: out of memory for JPEG output (%zu bytes)\n",
                     rgba_size);
        jpeg_abort_decompress(&cinfo);
        return nullptr;
    }

    // Row buffer for decompression (one scanline at a time).
    // Reuse the worker-owned allocation.
    auto &row_buf = decoder.row_buf;
    if (row_buf.size() < row_stride)
        row_buf.resize(row_stride);
    JSAMPROW row_ptr = row_buf.data();

    uint32_t y = 0;
    while (cinfo.output_scanline < cinfo.output_height) {
        jpeg_read_scanlines(&cinfo, &row_ptr, 1);
        uint8_t *dst = rgba + y * width * 4u;
        if (is_rgba) {
            std::memcpy(dst, row_buf.data(), width * 4u);
        } else {
            // Expand RGB -> RGBA (set alpha = 0xFF).
            const uint8_t *src = row_buf.data();
            for (uint32_t x = 0; x < width; ++x) {
                dst[x * 4 + 0] = src[x * 3 + 0];
                dst[x * 4 + 1] = src[x * 3 + 1];
                dst[x * 4 + 2] = src[x * 3 + 2];
                dst[x * 4 + 3] = 0xFF;
            }
        }
        ++y;
    }

    jpeg_finish_decompress(&cinfo);

    *out_width     = width;
    *out_height    = height;
    *out_rgba_size = static_cast<uint32_t>(rgba_size);
    decoder.rgba = nullptr;
    return rgba;
}

// ---------------------------------------------------------------------------
// Packet parsers
// ---------------------------------------------------------------------------

static void parse_pose(const uint8_t *buf, size_t len, WorldModel *wm) {
    if (len < POSE_PACKET_SIZE_V1) {
        std::fprintf(stderr, "spatial_bridge: pose packet too short (%zu bytes)\n", len);
        return;
    }
    sb_pose_t p{};
    p.timestamp_ns     = read_u64_le(buf + 1);
    p.pos[0]           = read_f32_le(buf + 9);
    p.pos[1]           = read_f32_le(buf + 13);
    p.pos[2]           = read_f32_le(buf + 17);
    p.rot[0]           = read_f32_le(buf + 21);  // x
    p.rot[1]           = read_f32_le(buf + 25);  // y
    p.rot[2]           = read_f32_le(buf + 29);  // z
    p.rot[3]           = read_f32_le(buf + 33);  // w
    p.tracking_quality = read_f32_le(buf + 37);
    p.session_epoch    = len >= POSE_PACKET_SIZE ? read_u32_le(buf + 41) : 0u;
    if (!all_finite(p.pos, 3) || !all_finite(p.rot, 4) || !std::isfinite(p.tracking_quality)) {
        drop_invalid("pose");
        return;
    }
    // A degenerate quaternion (notably all-zero, which passes the finite check)
    // is dropped rather than replaced with identity: identity is a valid-looking
    // orientation the consumer cannot tell apart from a real one, and every
    // downstream user inverts this quat (scene origin capture latches a zero
    // inverse). Dropping leaves the last good pose in the seqlock instead.
    float qlen = std::sqrt(p.rot[0] * p.rot[0] + p.rot[1] * p.rot[1] +
                           p.rot[2] * p.rot[2] + p.rot[3] * p.rot[3]);
    if (qlen < 1e-6f) {
        drop_invalid("pose");
        return;
    }
    for (int i = 0; i < 4; ++i)
        p.rot[i] /= qlen;
    world_model_update_pose(wm, p);
}

static void parse_plane(const uint8_t *buf, size_t len, WorldModel *wm) {
    if (len < PLANE_PACKET_SIZE) {
        std::fprintf(stderr, "spatial_bridge: plane packet too short (%zu bytes)\n", len);
        return;
    }
    sb_plane_t pl{};
    pl.timestamp_ns = read_u64_le(buf + 1);
    std::memcpy(pl.uuid, buf + 9, 16);
    pl.center[0]  = read_f32_le(buf + 25);
    pl.center[1]  = read_f32_le(buf + 29);
    pl.center[2]  = read_f32_le(buf + 33);
    pl.normal[0]  = read_f32_le(buf + 37);
    pl.normal[1]  = read_f32_le(buf + 41);
    pl.normal[2]  = read_f32_le(buf + 45);
    pl.extent[0]  = read_f32_le(buf + 49);  // width
    pl.extent[1]  = read_f32_le(buf + 53);  // height
    pl.alignment  = buf[57];
    pl.is_removed = buf[58];
    if (!all_finite(pl.center, 3) || !all_finite(pl.normal, 3) || !all_finite(pl.extent, 2)) {
        drop_invalid("plane");
        return;
    }
    // Remove packets carry zeroed geometry; only live planes need a usable shape.
    if (!pl.is_removed) {
        float n2 = pl.normal[0] * pl.normal[0] + pl.normal[1] * pl.normal[1] +
                   pl.normal[2] * pl.normal[2];
        if (!(n2 > 0.0f) || !(pl.extent[0] > 0.0f) || !(pl.extent[1] > 0.0f)) {
            drop_invalid("plane");
            return;
        }
    }
    world_model_update_plane(wm, pl);
}

/* parse_frame and run_jpeg_worker are defined further down, after
 * sb_receiver_impl_t is in scope (they touch its jpeg_* fields). */
static void parse_mesh(const uint8_t *buf, size_t len, WorldModel * /*wm*/) {
    // Mesh chunks are parsed structurally but not stored (LiDAR-only, optional feature).
    // Minimum header: 1 + 8 + 16 + 4 + 4 + 4 + 4 = 41 bytes.
    if (len < 41) {
        std::fprintf(stderr, "spatial_bridge: mesh packet too short (%zu bytes)\n", len);
        return;
    }
    // Silently accept; implement mesh storage in a future phase.
    (void)buf;
}

static void parse_hand(const uint8_t *buf, size_t len, WorldModel *wm) {
    if (len < HAND_PACKET_SIZE) {
        std::fprintf(stderr, "spatial_bridge: hand packet too short (%zu bytes, need %zu)\n", len,
                     HAND_PACKET_SIZE);
        return;
    }
    sb_hand_t h{};
    h.timestamp_ns = read_u64_le(buf + 1);
    h.hand_index   = buf[9];
    h.joint_count  = buf[10];

    if (h.hand_index > 1) {
        std::fprintf(stderr, "spatial_bridge: invalid hand_index %u\n", h.hand_index);
        return;
    }
    if (h.joint_count != SB_HAND_JOINT_COUNT) {
        std::fprintf(stderr, "spatial_bridge: unexpected joint_count %u (expected %d)\n",
                     h.joint_count, SB_HAND_JOINT_COUNT);
        return;
    }

    // Read 21 joints, 5 floats each (x, y, z, confidence, reserved)
    size_t offset = 11;
    for (int j = 0; j < SB_HAND_JOINT_COUNT; ++j) {
        for (int f = 0; f < 5; ++f) {
            h.joints[j][f] = read_f32_le(buf + offset);
            offset += 4;
        }
    }
    if (!all_finite(&h.joints[0][0], SB_HAND_JOINT_COUNT * 5)) {
        drop_invalid("hand");
        return;
    }
    world_model_update_hand(wm, h);

    static thread_local int hand_pkt_count = 0;
    hand_pkt_count++;
    if (hand_pkt_count == 1 || (hand_pkt_count % 300 == 0)) {
        std::fprintf(stderr,
                     "spatial_bridge: hand packet #%d: hand_index=%u "
                     "wrist=(%.3f,%.3f,%.3f) conf=%.2f\n",
                     hand_pkt_count, h.hand_index, (double)h.joints[0][0], (double)h.joints[0][1],
                     (double)h.joints[0][2], (double)h.joints[0][3]);
    }
}

static void parse_intrinsics(const uint8_t *buf, size_t len, WorldModel *wm) {
    if (len < INTRINSICS_PACKET_SIZE) {
        std::fprintf(stderr, "spatial_bridge: intrinsics packet too short (%zu bytes, need %zu)\n",
                     len, INTRINSICS_PACKET_SIZE);
        return;
    }
    sb_intrinsics_t in{};
    in.timestamp_ns = read_u64_le(buf + 1);
    in.fx           = read_f32_le(buf + 9);
    in.fy           = read_f32_le(buf + 13);
    in.cx           = read_f32_le(buf + 17);
    in.cy           = read_f32_le(buf + 21);
    in.image_width  = read_f32_le(buf + 25);
    in.image_height = read_f32_le(buf + 29);
    world_model_update_intrinsics(wm, in);
}

// ---------------------------------------------------------------------------
// sb_receiver_impl_t — the opaque handle.
// ---------------------------------------------------------------------------

enum class ReceiverMode { kUdp, kFile };

struct sb_receiver_impl_t {
    ReceiverMode mode;
    WorldModel *world_model;
    std::thread recv_thread;
    std::atomic<bool> stop_flag{false};
    std::mutex stop_mutex;
    std::condition_variable stop_cv;

    // UDP mode
    int sock_fd{-1};
    uint16_t port{0};

    // File mode
    std::string file_path;
    bool loop_file{false};

    // JPEG decode worker.  Previously parse_frame() decoded JPEG inline
    // on the recv thread; a single 640x480 q=60 decode is 5-15ms during
    // which the kernel UDP buffer fills up and silently drops pose +
    // plane packets (observed: 200+ kernel-level drops over a session).
    // Move decode to a dedicated worker so the recv loop can drain the
    // socket at line speed; if the worker is still busy when a new
    // frame arrives, drop the OLD pending one (camera latency >>
    // tracking latency).
    std::thread jpeg_thread;
    std::mutex jpeg_mutex;
    std::condition_variable jpeg_cv;
    std::vector<uint8_t> jpeg_pending;  // raw JPEG bytes
    uint64_t jpeg_pending_ts{0};
    uint32_t jpeg_pending_w{0};
    uint32_t jpeg_pending_h{0};
    bool jpeg_pending_set{false};
    uint64_t jpeg_dropped{0};  // diagnostic counter

    // Chunked-frame (0x09) reassembly.  Single-frame buffer: chunks are
    // accumulated by byte offset and, once every piece has arrived, the
    // completed JPEG is handed to the same decode worker as whole 0x03
    // frames.  Only ONE frame is assembled at a time — a chunk bearing a
    // newer timestamp abandons any incomplete current frame (a late frame
    // is useless for passthrough).  Touched only on the recv thread.
    uint64_t reasm_frame_id{0};      // timestamp being assembled (0 = none)
    uint32_t reasm_total{0};         // total JPEG size for that frame
    uint32_t reasm_recv{0};          // bytes received so far
    std::vector<uint8_t> reasm_buf;  // sized to reasm_total
    std::vector<bool> reasm_have;    // per-chunk-index presence (dedup)
    uint64_t reasm_dropped{0};       // frames abandoned incomplete (diag)
    // Loss-distribution diagnostics (recv thread only).  When a frame is
    // abandoned incomplete we accumulate how close it got: comparing the
    // average "% of chunks received" tells us WHICH fix the 0x09 loss needs —
    // ~95% means frames die on 1-2 stray lost chunks (→ small redundancy / FEC
    // wins big), ~50% means bursty multi-chunk loss (→ pacing / buffer / fewer
    // chunks).  completed vs dropped gives the true delivered frame rate.
    uint64_t reasm_completed{0};          // frames fully assembled + decoded
    uint64_t reasm_abandon_recv_sum{0};   // sum of bytes-received over abandoned frames
    uint64_t reasm_abandon_total_sum{0};  // sum of total-bytes over abandoned frames

    // Chunked-depth (0x0B) reassembly — INDEPENDENT of the 0x09 colour path so
    // interleaved colour/depth chunks never clobber each other. Recv thread only.
    uint64_t depth_reasm_frame_id{0};
    uint32_t depth_reasm_total{0};
    uint32_t depth_reasm_recv{0};
    std::vector<uint8_t> depth_reasm_buf;
    std::vector<bool> depth_reasm_have;
    uint64_t depth_reasm_completed{0};
    uint64_t depth_reasm_dropped{0};
};

static void enqueue_jpeg(sb_receiver_impl_t *rx, const uint8_t *jpeg, size_t n,
                         uint64_t timestamp_ns);

// Parse a frame packet on the recv thread: copy the JPEG bytes into
// the worker's pending slot and signal.  Does NOT decode inline.  If
// the worker hasn't picked up the previous frame yet, the old one is
// dropped (camera latency tolerance is much higher than tracking,
// and keeping recv-thread responsive matters more).
static void parse_frame(const uint8_t *buf, size_t len, sb_receiver_impl_t *rx) {
    if (len < FRAME_HEADER_SIZE) {
        std::fprintf(stderr, "spatial_bridge: frame packet too short (%zu bytes)\n", len);
        return;
    }
    uint64_t timestamp_ns = read_u64_le(buf + 1);
    uint32_t width        = read_u32_le(buf + 9);
    uint32_t height       = read_u32_le(buf + 13);
    uint32_t jpeg_size    = read_u32_le(buf + 17);

    if (len < FRAME_HEADER_SIZE + jpeg_size) {
        std::fprintf(stderr, "spatial_bridge: frame packet truncated (have %zu, need %zu)\n", len,
                     FRAME_HEADER_SIZE + static_cast<size_t>(jpeg_size));
        return;
    }
    (void)width;
    (void)height;  // worker reads dimensions from the JPEG itself
    enqueue_jpeg(rx, buf + FRAME_HEADER_SIZE, jpeg_size, timestamp_ns);
}

// Hand a fully-assembled JPEG to the decode worker (shared tail of both the
// whole-frame 0x03 path and the chunked 0x09 path).  Caller passes the JPEG
// bytes; width/height are 0 because the worker reads them from the JPEG.
static void enqueue_jpeg(sb_receiver_impl_t *rx, const uint8_t *jpeg, size_t n,
                         uint64_t timestamp_ns) {
    {
        std::lock_guard<std::mutex> lk(rx->jpeg_mutex);
        if (rx->jpeg_pending_set) rx->jpeg_dropped++;
        rx->jpeg_pending.assign(jpeg, jpeg + n);
        rx->jpeg_pending_ts  = timestamp_ns;
        rx->jpeg_pending_w   = 0;
        rx->jpeg_pending_h   = 0;
        rx->jpeg_pending_set = true;
    }
    rx->jpeg_cv.notify_one();
}

// Parse a chunked-frame (0x09) packet on the recv thread.  Accumulates the
// chunk into the single-frame reassembly buffer; when the frame is complete
// it is enqueued for decode.  Out-of-order and duplicate chunks are handled;
// a chunk with a newer timestamp abandons an incomplete current frame.
static void parse_frame_chunk(const uint8_t *buf, size_t len, sb_receiver_impl_t *rx) {
    if (len < FRAME_CHUNK_HEADER_SIZE) {
        std::fprintf(stderr, "spatial_bridge: frame-chunk too short (%zu bytes)\n", len);
        return;
    }
    uint64_t ts     = read_u64_le(buf + 1);
    uint32_t total  = read_u32_le(buf + 9);
    uint32_t offset = read_u32_le(buf + 13);
    uint16_t clen   = read_u16_le(buf + 17);

    if (len < FRAME_CHUNK_HEADER_SIZE + clen) {
        std::fprintf(stderr, "spatial_bridge: frame-chunk truncated (have %zu, need %zu)\n", len,
                     FRAME_CHUNK_HEADER_SIZE + static_cast<size_t>(clen));
        return;
    }
    if (total == 0 || total > MAX_FRAME_BYTES) {
        std::fprintf(stderr, "spatial_bridge: frame-chunk bad total_size %u\n", total);
        return;
    }
    if (clen == 0 || static_cast<uint64_t>(offset) + clen > total) {
        std::fprintf(stderr,
                     "spatial_bridge: frame-chunk range out of bounds "
                     "(offset=%u len=%u total=%u)\n",
                     offset, clen, total);
        return;
    }

    if (ts != rx->reasm_frame_id) {
        // Decide whether this chunk's timestamp should (re)start reassembly.
        // The ONLY timestamp we refuse to act on is a SMALL backward step: a
        // chunk slightly older than the in-progress frame is a genuine late
        // straggler from a frame we already superseded — ignore it.  EVERY
        // other case adopts `ts` and starts a fresh frame:
        //   - forward, small gap: the normal frame->frame advance (~33 ms);
        //   - forward, large gap (>skew): a legitimate resume after a real
        //     stall (the freeze itself, app backgrounding, ARKit relocalise) —
        //     wall time genuinely jumped, so this IS the current frame;
        //   - backward, large gap (>skew): the in-progress id is implausibly
        //     far ahead of real traffic, i.e. it was a corrupt far-future id —
        //     abandon it and adopt this (real) older timestamp.
        // Crucially we never *reject* a forward jump: doing so (an earlier
        // attempt did) re-wedges passthrough permanently when a partial frame
        // is stranded at T_old and real traffic resumes >skew later — every
        // resumed chunk looks "too far future" and is dropped forever.  A
        // relative gap cannot tell a corrupt future id from a legitimate resume
        // (both are far ahead of the stale id), so we adopt both.  A genuinely
        // corrupt far-future id therefore costs at most ONE abandoned frame: the
        // next real chunk is >skew *behind* it and triggers the backward-recover
        // branch, self-healing in ~33 ms.  Only relative gaps are compared (both
        // ends share the CLOCK_REALTIME ns epoch) — no wall-clock syscall here.
        if (rx->reasm_frame_id != 0) {
            bool forward = ts > rx->reasm_frame_id;
            uint64_t gap = forward ? (ts - rx->reasm_frame_id) : (rx->reasm_frame_id - ts);
            if (!forward && gap <= FRAME_TS_MAX_SKEW_NS)
                return;  // genuine late straggler from a superseded frame — ignore
            if (rx->reasm_recv < rx->reasm_total) {
                rx->reasm_dropped++;
                rx->reasm_abandon_recv_sum += rx->reasm_recv;
                rx->reasm_abandon_total_sum += rx->reasm_total;
            }
        }
        rx->reasm_frame_id = ts;
        rx->reasm_total    = total;
        rx->reasm_recv     = 0;
        rx->reasm_buf.resize(total);
        size_t nchunks =
            (static_cast<size_t>(total) + FRAME_CHUNK_PAYLOAD - 1) / FRAME_CHUNK_PAYLOAD;
        rx->reasm_have.assign(nchunks, false);
    }
    if (total != rx->reasm_total) return;  // inconsistent total within a frame id

    // Offset MUST be a chunk boundary: the dedup "have" bitset is indexed by
    // offset/FRAME_CHUNK_PAYLOAD, but the payload is memcpy'd to the raw byte
    // offset.  A non-multiple offset would alias two distinct offsets onto one
    // index — either silently dropping bytes (permanent stall) or leaving an
    // unwritten gap while reasm_recv still reaches total (a corrupt frame that
    // completes).  Reject it so the offset<->index bijection always holds.
    if (offset % FRAME_CHUNK_PAYLOAD != 0) {
        std::fprintf(stderr, "spatial_bridge: frame-chunk misaligned offset %u\n", offset);
        return;
    }
    size_t idx = offset / FRAME_CHUNK_PAYLOAD;
    if (idx >= rx->reasm_have.size() || rx->reasm_have[idx])
        return;  // out-of-range index or duplicate chunk
    std::memcpy(rx->reasm_buf.data() + offset, buf + FRAME_CHUNK_HEADER_SIZE, clen);
    rx->reasm_have[idx] = true;
    rx->reasm_recv += clen;

    if (rx->reasm_recv >= rx->reasm_total) {
        enqueue_jpeg(rx, rx->reasm_buf.data(), rx->reasm_buf.size(), rx->reasm_frame_id);
        rx->reasm_completed++;
        rx->reasm_frame_id = 0;  // complete; next chunk begins a fresh frame
    }
}

// Decode a completed 0x0B payload: validate the 13-byte sub-header, dequantize
// the u16 plane to float metres (0 == invalid), and publish. Runs on the recv
// thread — dequantizing ~50k values is well under a millisecond (unlike JPEG
// decode), so no separate worker thread is needed.
static void publish_depth(const uint8_t *payload, size_t n, uint64_t ts,
                          WorldModel *wm) {
    if (n < DEPTH_SUBHEADER_SIZE) {
        std::fprintf(stderr, "spatial_bridge: depth payload too short (%zu bytes)\n", n);
        return;
    }
    uint16_t width    = read_u16_le(payload + 0);
    uint16_t height   = read_u16_le(payload + 2);
    uint8_t  encoding = payload[4];
    float    z_min    = read_f32_le(payload + 5);
    float    z_max    = read_f32_le(payload + 9);

    if (encoding != 1 || width == 0 || height == 0) {
        std::fprintf(stderr, "spatial_bridge: depth bad sub-header (enc=%u %ux%u)\n",
                     encoding, width, height);
        return;
    }
    size_t count = static_cast<size_t>(width) * height;
    if (n != DEPTH_SUBHEADER_SIZE + count * 2) {
        std::fprintf(stderr, "spatial_bridge: depth size mismatch (have %zu, need %zu)\n",
                     n, DEPTH_SUBHEADER_SIZE + count * 2);
        return;
    }
    if (!(z_max > z_min)) {
        std::fprintf(stderr, "spatial_bridge: depth bad range [%f,%f]\n",
                     (double)z_min, (double)z_max);
        return;
    }

    float *depth = static_cast<float *>(std::malloc(count * sizeof(float)));
    if (!depth) {
        std::fprintf(stderr, "spatial_bridge: depth malloc(%zu) failed\n", count * sizeof(float));
        return;
    }
    const uint8_t *q     = payload + DEPTH_SUBHEADER_SIZE;
    const float    scale = (z_max - z_min) / 65535.0f;
    for (size_t i = 0; i < count; ++i) {
        uint16_t v = read_u16_le(q + i * 2);
        depth[i]   = (v == 0) ? 0.0f : (z_min + static_cast<float>(v) * scale);
    }

    sb_depth_t d{};
    d.timestamp_ns = ts;
    d.width        = width;
    d.height       = height;
    d.z_min        = z_min;
    d.z_max        = z_max;
    d.depth        = depth;  // ownership transfers to world_model
    d.depth_count  = static_cast<uint32_t>(count);
    world_model_update_depth(wm, d);
}

// Parse a chunked-depth (0x0B) packet. Reassembly is identical to
// parse_frame_chunk but uses an INDEPENDENT buffer set; on completion the
// payload is dequantized inline (not handed to the JPEG worker).
static void parse_depth_chunk(const uint8_t *buf, size_t len, sb_receiver_impl_t *rx) {
    if (len < FRAME_CHUNK_HEADER_SIZE) {
        std::fprintf(stderr, "spatial_bridge: depth-chunk too short (%zu bytes)\n", len);
        return;
    }
    uint64_t ts     = read_u64_le(buf + 1);
    uint32_t total  = read_u32_le(buf + 9);
    uint32_t offset = read_u32_le(buf + 13);
    uint16_t clen   = read_u16_le(buf + 17);

    if (len < FRAME_CHUNK_HEADER_SIZE + clen) {
        std::fprintf(stderr, "spatial_bridge: depth-chunk truncated (have %zu, need %zu)\n", len,
                     FRAME_CHUNK_HEADER_SIZE + static_cast<size_t>(clen));
        return;
    }
    if (total < DEPTH_SUBHEADER_SIZE || total > MAX_DEPTH_BYTES) {
        std::fprintf(stderr, "spatial_bridge: depth-chunk bad total_size %u\n", total);
        return;
    }
    if (clen == 0 || static_cast<uint64_t>(offset) + clen > total) {
        std::fprintf(stderr,
                     "spatial_bridge: depth-chunk range out of bounds "
                     "(offset=%u len=%u total=%u)\n",
                     offset, clen, total);
        return;
    }

    if (ts != rx->depth_reasm_frame_id) {
        // Same supersede/abandon logic as parse_frame_chunk (see its comment).
        if (rx->depth_reasm_frame_id != 0) {
            bool forward = ts > rx->depth_reasm_frame_id;
            uint64_t gap = forward ? (ts - rx->depth_reasm_frame_id)
                                   : (rx->depth_reasm_frame_id - ts);
            if (!forward && gap <= FRAME_TS_MAX_SKEW_NS)
                return;  // genuine late straggler from a superseded depth frame
            if (rx->depth_reasm_recv < rx->depth_reasm_total)
                rx->depth_reasm_dropped++;
        }
        rx->depth_reasm_frame_id = ts;
        rx->depth_reasm_total    = total;
        rx->depth_reasm_recv     = 0;
        rx->depth_reasm_buf.resize(total);
        size_t nchunks =
            (static_cast<size_t>(total) + FRAME_CHUNK_PAYLOAD - 1) / FRAME_CHUNK_PAYLOAD;
        rx->depth_reasm_have.assign(nchunks, false);
    }
    if (total != rx->depth_reasm_total) return;  // inconsistent total within a frame id

    if (offset % FRAME_CHUNK_PAYLOAD != 0) {
        std::fprintf(stderr, "spatial_bridge: depth-chunk misaligned offset %u\n", offset);
        return;
    }
    size_t idx = offset / FRAME_CHUNK_PAYLOAD;
    if (idx >= rx->depth_reasm_have.size() || rx->depth_reasm_have[idx])
        return;  // out-of-range index or duplicate chunk
    std::memcpy(rx->depth_reasm_buf.data() + offset, buf + FRAME_CHUNK_HEADER_SIZE, clen);
    rx->depth_reasm_have[idx] = true;
    rx->depth_reasm_recv += clen;

    if (rx->depth_reasm_recv >= rx->depth_reasm_total) {
        publish_depth(rx->depth_reasm_buf.data(), rx->depth_reasm_buf.size(),
                      rx->depth_reasm_frame_id, rx->world_model);
        rx->depth_reasm_completed++;
        rx->depth_reasm_frame_id = 0;  // complete; next chunk begins a fresh frame
    }
}

// JPEG decode worker thread.  Runs the decode + world_model_update
// off the recv thread so the kernel UDP buffer can be drained
// continuously.
static void run_jpeg_worker(sb_receiver_impl_t *rx) {
    JpegDecoder decoder;
    std::vector<uint8_t> jpeg;
    while (!rx->stop_flag.load(std::memory_order_relaxed)) {
        uint64_t timestamp_ns = 0;
        uint32_t width = 0, height = 0;
        {
            std::unique_lock<std::mutex> lk(rx->jpeg_mutex);
            rx->jpeg_cv.wait(lk, [&] {
                return rx->jpeg_pending_set || rx->stop_flag.load(std::memory_order_relaxed);
            });
            if (rx->stop_flag.load(std::memory_order_relaxed)) break;
            jpeg.swap(rx->jpeg_pending);
            timestamp_ns         = rx->jpeg_pending_ts;
            width                = rx->jpeg_pending_w;
            height               = rx->jpeg_pending_h;
            rx->jpeg_pending_set = false;
        }

        uint32_t decoded_w = 0, decoded_h = 0, rgba_size = 0;
        uint8_t *rgba =
            jpeg_decode_rgba(decoder, jpeg.data(), jpeg.size(), &decoded_w, &decoded_h, &rgba_size);

        // Clear jpeg data but keep the allocation for reuse.  On the next
        // iteration the swap() inside the lock donates this pre-allocated
        // buffer to jpeg_pending, so enqueue_jpeg's assign() reuses it
        // instead of malloc'ing fresh each frame.
        jpeg.clear();

        if (!rgba) continue;

        sb_frame_t frame{};
        frame.timestamp_ns = timestamp_ns;
        frame.width        = decoded_w ? decoded_w : width;
        frame.height       = decoded_h ? decoded_h : height;
        frame.rgba         = rgba;
        frame.rgba_size    = rgba_size;
        // world_model_update_frame takes ownership of rgba.
        world_model_update_frame(rx->world_model, frame);
    }
}

// ---------------------------------------------------------------------------
// Dispatch a single datagram
// ---------------------------------------------------------------------------

// Read system-wide IP fragment-reassembly counters from /proc/net/snmp
// (Linux).  Returns false if unavailable.  Parses the column index of each
// field from the "Ip:" header row so it's robust across kernel versions.
//
// Why this matters: a 30-50 KB camera frame UDP datagram IP-fragments into
// ~30 packets; the kernel holds the fragments in a reassembly buffer until
// all arrive or ipfrag_time (default 30s) elapses.  If fragments are lost
// in bursty Wi-Fi — or the reassembly buffer saturates under the load of
// many large datagrams — whole frames are dropped while tiny single-packet
// poses are unaffected.  That matches the "video freezes, head tracking fine"
// symptom.  This is a HYPOTHESIS, not yet confirmed: ReasmFails climbing in
// lockstep with a freeze would be strong corroboration (the counter is
// system-wide, but on a dedicated streaming box it's almost all ours), whereas
// flat ReasmFails during a freeze would point elsewhere (e.g. per-socket
// SO_RXQ_OVFL — see the recvmsg loop — or plain chunk loss).  We have not yet
// captured this reading from hardware.
static bool read_ip_reasm_stats(uint64_t *reqds, uint64_t *oks, uint64_t *fails) {
    std::FILE *f = std::fopen("/proc/net/snmp", "r");
    if (!f) return false;
    char hdr[4096], val[4096];
    bool found = false;
    while (std::fgets(hdr, sizeof hdr, f)) {
        if (std::strncmp(hdr, "Ip: ", 4) != 0) continue;
        if (!std::fgets(val, sizeof val, f) || std::strncmp(val, "Ip: ", 4) != 0) break;
        int idx = 0, ri = -1, oi = -1, fi = -1;
        char *save = nullptr;
        for (char *tok = strtok_r(hdr, " \t\n", &save); tok;
             tok       = strtok_r(nullptr, " \t\n", &save), ++idx) {
            if (std::strcmp(tok, "ReasmReqds") == 0)
                ri = idx;
            else if (std::strcmp(tok, "ReasmOKs") == 0)
                oi = idx;
            else if (std::strcmp(tok, "ReasmFails") == 0)
                fi = idx;
        }
        if (ri < 0 || fi < 0) break;
        idx         = 0;
        char *save2 = nullptr;
        for (char *tok = strtok_r(val, " \t\n", &save2); tok;
             tok       = strtok_r(nullptr, " \t\n", &save2), ++idx) {
            if (idx == ri)
                *reqds = std::strtoull(tok, nullptr, 10);
            else if (idx == oi)
                *oks = std::strtoull(tok, nullptr, 10);
            else if (idx == fi)
                *fails = std::strtoull(tok, nullptr, 10);
        }
        found = true;
        break;
    }
    std::fclose(f);
    return found;
}

static void dispatch_packet(const uint8_t *buf, size_t len, sb_receiver_impl_t *rx) {
    if (len == 0) return;
    WorldModel *wm = rx->world_model;
    world_model_record_packet(wm);

    // Debug: log every unique packet type we receive
    static thread_local uint32_t pkt_type_counts[256] = {};
    pkt_type_counts[buf[0]]++;
    static thread_local int total_dispatch = 0;
    total_dispatch++;
    if (total_dispatch == 1 || (total_dispatch % 6000 == 0)) {
        std::fprintf(stderr,
                     "spatial_bridge: dispatch stats (%d total): "
                     "0x01=%u 0x02=%u 0x03=%u 0x04=%u 0x05=%u 0x09=%u 0x0A=%u 0x0B=%u\n",
                     total_dispatch, pkt_type_counts[1], pkt_type_counts[2], pkt_type_counts[3],
                     pkt_type_counts[4], pkt_type_counts[5], pkt_type_counts[9],
                     pkt_type_counts[10], pkt_type_counts[11]);
        uint64_t rq = 0, ok = 0, fl = 0;
        static thread_local uint64_t last_reqds = 0, last_fails = 0;
        if (read_ip_reasm_stats(&rq, &ok, &fl)) {
            std::fprintf(stderr,
                         "spatial_bridge: IP reassembly: reqds+%llu fails+%llu "
                         "(cumulative fails=%llu) — fails climbing while a 0x03 frame "
                         "count stalls is consistent with frames lost to fragment "
                         "reassembly (cross-check SO_RXQ_OVFL drops below before concluding)\n",
                         (unsigned long long)(rq - last_reqds),
                         (unsigned long long)(fl - last_fails), (unsigned long long)fl);
            last_reqds = rq;
            last_fails = fl;
        }
        // 0x09 reassembly health.  reasm_dropped = frames abandoned incomplete
        // (a chunk lost in flight, or a newer frame superseded a partial one);
        // jpeg_dropped = assembled frames the decode worker couldn't keep up
        // with.  Both are written only on this (recv) thread, so the read is
        // race-free.  A climbing reasm_dropped with a frozen 0x09 count is the
        // signal that chunk loss — not the kernel — is starving passthrough.
        // avg % of chunks an abandoned frame had received before it was given
        // up — the discriminator between "1-2 stray losses" (→ redundancy/FEC)
        // and "bursty multi-chunk loss" (→ pacing / bigger buffer / fewer
        // chunks).  completed vs dropped is the true delivered frame rate.
        double abandon_pct =
            rx->reasm_abandon_total_sum
                ? 100.0 * (double)rx->reasm_abandon_recv_sum / (double)rx->reasm_abandon_total_sum
                : 0.0;
        std::fprintf(stderr,
                     "spatial_bridge: frame reassembly: completed=%llu dropped=%llu "
                     "jpeg_dropped=%llu; abandoned frames had avg %.0f%% of their chunks\n",
                     (unsigned long long)rx->reasm_completed, (unsigned long long)rx->reasm_dropped,
                     (unsigned long long)rx->jpeg_dropped, abandon_pct);
        if (rx->depth_reasm_completed || rx->depth_reasm_dropped)
            std::fprintf(stderr,
                         "spatial_bridge: depth reassembly: completed=%llu dropped=%llu\n",
                         (unsigned long long)rx->depth_reasm_completed,
                         (unsigned long long)rx->depth_reasm_dropped);
    }

    switch (buf[0]) {
        case PKT_POSE:
            parse_pose(buf, len, wm);
            break;
        case PKT_PLANE:
            parse_plane(buf, len, wm);
            break;
        case PKT_FRAME:
            parse_frame(buf, len, rx);
            break;
        case PKT_FRAME_CHUNK:
            parse_frame_chunk(buf, len, rx);
            break;
        case PKT_MESH:
            parse_mesh(buf, len, wm);
            break;
        case PKT_HAND:
            parse_hand(buf, len, wm);
            break;
        case PKT_INTRINSICS:
            parse_intrinsics(buf, len, wm);
            break;
        case PKT_DEPTH:
            parse_depth_chunk(buf, len, rx);
            break;
        default:
            // Unknown packet types (including the retired 0x06/0x07/0x08
            // recording-control / marker / label-vocab packets that may
            // still be present in older recorded .bin files) are silently
            // skipped so historical recordings remain replayable.
            break;
    }
}

// ---------------------------------------------------------------------------
// File format helpers
//
// .bin file layout: a sequence of length-prefixed datagrams.
//   [uint32_t datagram_len][datagram_len bytes payload]
// The iPhone record-session.sh script writes the file in this format.
// Timing is reconstructed from the timestamps inside the packets.
// ---------------------------------------------------------------------------

// Read exactly n bytes from fd. Returns false on EOF or error.
static bool read_exact(FILE *fp, void *out, size_t n) {
    size_t got = std::fread(out, 1, n, fp);
    return got == n;
}

static void run_file_loop(sb_receiver_impl_t *rx) {
    std::vector<uint8_t> buf(MAX_DATAGRAM);

    auto replay_file = [&]() -> bool {
        FILE *fp = std::fopen(rx->file_path.c_str(), "rb");
        if (!fp) {
            std::fprintf(stderr, "spatial_bridge: cannot open %s: %s\n", rx->file_path.c_str(),
                         std::strerror(errno));
            return false;
        }

        uint64_t first_pkt_ts = 0;
        bool first_pkt_seen   = false;
        auto wall_start       = std::chrono::steady_clock::now();

        while (!rx->stop_flag.load(std::memory_order_relaxed)) {
            uint32_t pkt_len = 0;
            if (!read_exact(fp, &pkt_len, 4)) break;  // EOF or error
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
            pkt_len = __builtin_bswap32(pkt_len);
#endif
            if (pkt_len == 0 || pkt_len > MAX_DATAGRAM) {
                std::fprintf(stderr, "spatial_bridge: invalid record pkt_len=%u\n", pkt_len);
                break;
            }
            buf.resize(pkt_len);
            if (!read_exact(fp, buf.data(), pkt_len)) break;

            // Reconstruct inter-packet timing from the embedded timestamp.
            if (pkt_len >= 9) {  // all packet types have ts at offset 1
                uint64_t pkt_ts = read_u64_le(buf.data() + 1);
                if (!first_pkt_seen) {
                    first_pkt_ts   = pkt_ts;
                    first_pkt_seen = true;
                }
                // Sleep until the packet's relative time.
                uint64_t relative_ns = pkt_ts - first_pkt_ts;
                auto target          = wall_start + std::chrono::nanoseconds(relative_ns);
                auto now             = std::chrono::steady_clock::now();
                if (target > now) {
                    std::unique_lock<std::mutex> lock(rx->stop_mutex);
                    if (rx->stop_cv.wait_until(lock, target, [&] {
                            return rx->stop_flag.load(std::memory_order_relaxed);
                        })) break;
                }
            }

            dispatch_packet(buf.data(), pkt_len, rx);
        }

        std::fclose(fp);
        return true;
    };

    do {
        replay_file();
    } while (rx->loop_file && !rx->stop_flag.load(std::memory_order_relaxed));
}

// ---------------------------------------------------------------------------
// UDP receive loop
// ---------------------------------------------------------------------------

/* Only a datagram that looks like a real iPhone packet may retarget the
 * heartbeat.  Otherwise a single byte from anywhere on the LAN repoints the
 * Mac's only outbound stream at an arbitrary address. */
static bool is_known_inbound_type(const uint8_t *buf, size_t len) {
    if (len < 9) return false;  // every type carries a type byte + u64 ts
    switch (buf[0]) {
        case PKT_POSE:
        case PKT_PLANE:
        case PKT_FRAME:
        case PKT_MESH:
        case PKT_HAND:
        case PKT_FRAME_CHUNK:
        case PKT_INTRINSICS:
        case PKT_DEPTH:
            return true;
        default:
            return false;
    }
}

/* Build + fire one 0x0C liveness datagram at `peer`.  Best-effort: MSG_DONTWAIT
 * plus a swallowed error, because a heartbeat that blocks (or that kills the
 * loop on a transient ENETUNREACH) would cost us the pose stream it exists to
 * report on. */
static void send_heartbeat(sb_receiver_impl_t *rx, const struct sockaddr_storage &peer,
                           socklen_t peer_len) {
    uint8_t out[HEARTBEAT_SIZE];
    out[0] = PKT_HEARTBEAT;
    uint64_t now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
    write_u64_le(out + 1, now_ns);
    float rate = world_model_get_packet_rate(rx->world_model);
    uint32_t rate_bits;
    std::memcpy(&rate_bits, &rate, 4);
    write_u32_le(out + 9, rate_bits);

    ssize_t sent = sendto(rx->sock_fd, out, sizeof(out), MSG_DONTWAIT,
                          reinterpret_cast<const struct sockaddr *>(&peer), peer_len);
    if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        std::fprintf(stderr, "spatial_bridge: heartbeat sendto: %s\n", std::strerror(errno));
    }
}

static void run_udp_loop(sb_receiver_impl_t *rx) {
    std::vector<uint8_t> buf(MAX_DATAGRAM);

    /* Cumulative kernel-side drop counter; reported by SO_RXQ_OVFL cmsg. */
    uint32_t last_dropped = 0;

    /* Last datagram's sender — the heartbeat target.  Captured from recvmsg
     * rather than configured, so the Mac never needs to know the phone's IP. */
    struct sockaddr_storage peer = {};
    socklen_t peer_len           = 0;
    auto last_heartbeat          = std::chrono::steady_clock::now();

    struct pollfd pfd;
    pfd.fd = rx->sock_fd;
    pfd.events = POLLIN;

    while (!rx->stop_flag.load(std::memory_order_relaxed)) {
        int sel = poll(&pfd, 1, 100);  // 100 ms timeout
        if (sel < 0) {
            if (errno == EINTR) continue;
            std::fprintf(stderr, "spatial_bridge: poll error: %s\n", std::strerror(errno));
            break;
        }

        /* Checked on every iteration (including the 100 ms poll timeout) so the
         * beacon keeps its cadence even when inbound traffic stalls. */
        auto now = std::chrono::steady_clock::now();
        if (peer_len > 0 &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_heartbeat).count() >=
                HEARTBEAT_PERIOD_MS) {
            last_heartbeat = now;
            send_heartbeat(rx, peer, peer_len);
        }

        if (sel == 0) continue;  // timeout — loop to check stop_flag

        /* recvmsg lets us read the SO_RXQ_OVFL ancillary data so we can
         * detect kernel-side drops.  If pose updates are dropping (because
         * 100-200 KB JPEG camera frames fill the recv buffer faster than
         * we drain it), the symptom upstream looks like "world is frozen". */
        struct sockaddr_storage src = {};
        struct iovec iov;
        iov.iov_base = buf.data();
        iov.iov_len  = buf.size();
        char ctrl[CMSG_SPACE(sizeof(uint32_t))];
        struct msghdr msg  = {};
        msg.msg_name       = &src;
        msg.msg_namelen    = sizeof(src);
        msg.msg_iov        = &iov;
        msg.msg_iovlen     = 1;
        msg.msg_control    = ctrl;
        msg.msg_controllen = sizeof(ctrl);

        ssize_t n = recvmsg(rx->sock_fd, &msg, 0);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            std::fprintf(stderr, "spatial_bridge: recvmsg error: %s\n", std::strerror(errno));
            break;
        }
#ifdef SO_RXQ_OVFL
        // Kernel UDP-drop counter (Linux-only ancillary data). Skipped on
        // platforms that lack SO_RXQ_OVFL (e.g. macOS dev builds).
        for (struct cmsghdr *c = CMSG_FIRSTHDR(&msg); c != nullptr; c = CMSG_NXTHDR(&msg, c)) {
            if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_RXQ_OVFL) {
                uint32_t dropped = 0;
                std::memcpy(&dropped, CMSG_DATA(c), sizeof(dropped));
                if (dropped > last_dropped) {
                    std::fprintf(stderr,
                                 "spatial_bridge: WARNING — kernel dropped %u "
                                 "UDP packets since last report (total=%u). "
                                 "Likely cause: JPEG decode on recv thread "
                                 "starving the socket.\n",
                                 dropped - last_dropped, dropped);
                    last_dropped = dropped;
                }
            }
        }
#else
        (void)last_dropped;
#endif

        if (msg.msg_namelen > 0 && msg.msg_namelen <= sizeof(src) &&
            is_known_inbound_type(buf.data(), static_cast<size_t>(n))) {
            peer     = src;
            peer_len = msg.msg_namelen;
        }

        dispatch_packet(buf.data(), static_cast<size_t>(n), rx);
    }
}

// ---------------------------------------------------------------------------
// Public C API implementation
// ---------------------------------------------------------------------------

sb_receiver_t *sb_receiver_create(uint16_t port) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::fprintf(stderr, "spatial_bridge: socket(): %s\n", std::strerror(errno));
        return nullptr;
    }

    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    /* Request a large UDP recv buffer.  The kernel will silently clamp to
     * net.core.rmem_max — query it back so the user can tell whether the
     * clamp is biting.  Default rmem_max on Ubuntu 24.04 is 212992 bytes,
     * which is smaller than a single high-quality JPEG camera frame.  The
     * caller can raise the cap with:
     *   sudo sysctl -w net.core.rmem_max=8388608
     * Combined with a per-packet enable of SO_RXQ_OVFL below, we can
     * detect kernel-side drops (which manifest as render stalls upstream
     * because pose updates land in the same shared kernel buffer as
     * 100-200 KB camera frames). */
    int want_rcvbuf = 4 * 1024 * 1024; /* 4 MiB */
    setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &want_rcvbuf, sizeof(want_rcvbuf));
    int got_rcvbuf    = 0;
    socklen_t got_len = sizeof(got_rcvbuf);
    if (getsockopt(fd, SOL_SOCKET, SO_RCVBUF, &got_rcvbuf, &got_len) == 0) {
        /* Linux returns 2x the value actually applied (it accounts for
         * per-packet overhead), so divide by 2 for the "logical" size. */
        std::fprintf(stderr,
                     "spatial_bridge: UDP SO_RCVBUF requested=%d granted=%d "
                     "(kernel doubles for overhead; sysctl net.core.rmem_max "
                     "caps this)\n",
                     want_rcvbuf, got_rcvbuf / 2);
    }
    /* Enable per-packet drop counter via SO_RXQ_OVFL.  Each recvmsg can
     * then report the cumulative drop count via cmsg, so we can warn
     * loudly when the kernel is dropping packets. */
#ifdef SO_RXQ_OVFL
    int ovfl = 1;
    setsockopt(fd, SOL_SOCKET, SO_RXQ_OVFL, &ovfl, sizeof(ovfl));
#endif

    struct sockaddr_in addr = {};
    addr.sin_family         = AF_INET;
    addr.sin_port           = htons(port);
    addr.sin_addr.s_addr    = INADDR_ANY;

    if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "spatial_bridge: bind(:%u): %s\n", port, std::strerror(errno));
        close(fd);
        return nullptr;
    }

    WorldModel *wm = world_model_create();
    if (!wm) {
        close(fd);
        return nullptr;
    }

    auto *rx = new (std::nothrow) sb_receiver_impl_t();
    if (!rx) {
        world_model_destroy(wm);
        close(fd);
        return nullptr;
    }
    rx->mode        = ReceiverMode::kUdp;
    rx->world_model = wm;
    rx->sock_fd     = fd;
    rx->port        = port;
    return rx;
}

sb_receiver_t *sb_receiver_from_file(const char *path, bool loop) {
    if (!path) {
        std::fprintf(stderr, "spatial_bridge: sb_receiver_from_file: null path\n");
        return nullptr;
    }

    WorldModel *wm = world_model_create();
    if (!wm) return nullptr;

    auto *rx = new (std::nothrow) sb_receiver_impl_t();
    if (!rx) {
        world_model_destroy(wm);
        return nullptr;
    }
    rx->mode        = ReceiverMode::kFile;
    rx->world_model = wm;
    rx->file_path   = path;
    rx->loop_file   = loop;
    return rx;
}

void sb_receiver_destroy(sb_receiver_t *r) {
    if (!r) return;
    sb_stop(r);
    if (r->sock_fd >= 0) {
        close(r->sock_fd);
        r->sock_fd = -1;
    }
    world_model_destroy(r->world_model);
    delete r;
}

void sb_start(sb_receiver_t *r) {
    if (!r) return;
    r->stop_flag.store(false, std::memory_order_relaxed);
    r->jpeg_thread = std::thread(run_jpeg_worker, r);
    if (r->mode == ReceiverMode::kUdp) {
        r->recv_thread = std::thread(run_udp_loop, r);
    } else {
        r->recv_thread = std::thread(run_file_loop, r);
    }
}

void sb_stop(sb_receiver_t *r) {
    if (!r) return;
    {
        std::scoped_lock lock(r->stop_mutex, r->jpeg_mutex);
        r->stop_flag.store(true, std::memory_order_relaxed);
    }
    r->stop_cv.notify_all();
    r->jpeg_cv.notify_all();
    if (r->recv_thread.joinable()) {
        r->recv_thread.join();
    }
    if (r->jpeg_thread.joinable()) {
        r->jpeg_thread.join();
    }
}

bool sb_get_latest_pose(sb_receiver_t *r, sb_pose_t *out) {
    if (!r || !out) return false;
    return world_model_get_pose(r->world_model, out);
}

uint32_t sb_get_session_epoch(sb_receiver_t *r) {
    if (!r) return 0u;
    return world_model_get_session_epoch(r->world_model);
}

bool sb_get_latest_frame(sb_receiver_t *r, sb_frame_t *out) {
    if (!r || !out) return false;
    return world_model_get_frame(r->world_model, out);
}

int sb_get_planes(sb_receiver_t *r, sb_plane_t *out, int max_planes) {
    if (!r || !out || max_planes <= 0) return 0;
    return world_model_get_planes(r->world_model, out, max_planes);
}

bool sb_get_hand(sb_receiver_t *r, int hand_index, sb_hand_t *out) {
    if (!r || !out) return false;
    return world_model_get_hand(r->world_model, hand_index, out);
}

bool sb_get_latest_intrinsics(sb_receiver_t *r, sb_intrinsics_t *out) {
    if (!r || !out) return false;
    return world_model_get_intrinsics(r->world_model, out);
}

bool sb_get_latest_depth(sb_receiver_t *r, sb_depth_t *out) {
    if (!r || !out) return false;
    return world_model_get_depth(r->world_model, out);
}

float sb_get_packet_rate(sb_receiver_t *r) {
    if (!r) return 0.0f;
    return world_model_get_packet_rate(r->world_model);
}

int64_t sb_get_frame_age_ms(sb_receiver_t *r) {
    if (!r) return -1;
    return world_model_get_frame_age_ms(r->world_model);
}

void sb_free_depth(sb_depth_t *d) {
    if (!d || !d->depth) return;
    std::free(d->depth);
    d->depth       = nullptr;
    d->depth_count = 0;
}

void sb_free_frame(sb_frame_t *f) {
    if (!f || !f->rgba) return;
    std::free(f->rgba);
    f->rgba      = nullptr;
    f->rgba_size = 0;
}
