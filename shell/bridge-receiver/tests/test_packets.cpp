// test_packets.cpp — Unit tests for packet parsing and the seqlock.
//
// Tests run entirely without live hardware:
//   - Packet bytes are constructed by hand matching the wire protocol.
//   - Written to a temp file and opened with sb_receiver_from_file().
//   - Seqlock correctness is validated with a concurrent stress test.
//
// No external test framework — uses <cassert> and prints pass/fail to stdout.

#include "spatial_bridge.h"
#include "jpeg_fixture.h"

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

// ---------------------------------------------------------------------------
// Helpers — little-endian write into byte buffer (mirrors wire protocol)
// ---------------------------------------------------------------------------

static void write_u8(std::vector<uint8_t> &buf, uint8_t v) { buf.push_back(v); }

static void write_u32_le(std::vector<uint8_t> &buf, uint32_t v) {
    buf.push_back(static_cast<uint8_t>(v >> 0));
    buf.push_back(static_cast<uint8_t>(v >> 8));
    buf.push_back(static_cast<uint8_t>(v >> 16));
    buf.push_back(static_cast<uint8_t>(v >> 24));
}

static void write_u16_le(std::vector<uint8_t> &buf, uint16_t v) {
    buf.push_back(static_cast<uint8_t>(v >> 0));
    buf.push_back(static_cast<uint8_t>(v >> 8));
}

static void write_u64_le(std::vector<uint8_t> &buf, uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        buf.push_back(static_cast<uint8_t>(v >> (8 * i)));
    }
}

static void write_f32_le(std::vector<uint8_t> &buf, float f) {
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    write_u32_le(buf, bits);
}

// ---------------------------------------------------------------------------
// .bin file helpers
//
// Format: repeated [uint32_t len (LE)][len bytes payload]
// ---------------------------------------------------------------------------

static void write_bin_record(std::vector<uint8_t> &file_buf, const std::vector<uint8_t> &pkt) {
    write_u32_le(file_buf, static_cast<uint32_t>(pkt.size()));
    file_buf.insert(file_buf.end(), pkt.begin(), pkt.end());
}

static std::string write_temp_file(const std::vector<uint8_t> &data) {
    char tmpl[] = "/tmp/sb_test_XXXXXX";
    int fd      = mkstemp(tmpl);
    assert(fd >= 0 && "mkstemp failed");
    std::string path(tmpl);
    FILE *fp = fdopen(fd, "wb");
    assert(fp && "fdopen failed");
    std::fwrite(data.data(), 1, data.size(), fp);
    std::fclose(fp);
    return path;
}

// ---------------------------------------------------------------------------
// Test 1: Pose packet size is exactly 45 bytes (41 for a pre-epoch sender)
// ---------------------------------------------------------------------------

static void test_pose_packet_size() {
    // 1 (type) + 8 (ts) + 3*4 (pos) + 4*4 (rot) + 4 (quality) = 41
    static_assert(1 + 8 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 == 41, "pre-epoch pose size must be 41");
    // + 4 (session_epoch) = 45
    static_assert(41 + 4 == 45, "pose packet size must be 45");
    std::printf("PASS test_pose_packet_size\n");
}

// ---------------------------------------------------------------------------
// Test 2: Plane packet size is exactly 59 bytes
// ---------------------------------------------------------------------------

static void test_plane_packet_size() {
    // 1 + 8 + 16 + 12 (center) + 12 (normal) + 8 (extent) + 1 + 1 = 59
    static_assert(1 + 8 + 16 + 12 + 12 + 8 + 1 + 1 == 59, "plane packet size must be 59");
    std::printf("PASS test_plane_packet_size\n");
}

// ---------------------------------------------------------------------------
// Test 3: Little-endian encoding — 0x01020304 must decode as [04 03 02 01]
// ---------------------------------------------------------------------------

static void test_little_endian_encoding() {
    uint32_t v = 0x01020304u;
    std::vector<uint8_t> buf;
    write_u32_le(buf, v);
    assert(buf.size() == 4);
    assert(buf[0] == 0x04 && "LE byte 0");
    assert(buf[1] == 0x03 && "LE byte 1");
    assert(buf[2] == 0x02 && "LE byte 2");
    assert(buf[3] == 0x01 && "LE byte 3");
    std::printf("PASS test_little_endian_encoding\n");
}

// ---------------------------------------------------------------------------
// Test 4: Pose packet roundtrip through file replay
// ---------------------------------------------------------------------------

static void test_pose_roundtrip() {
    const uint64_t ts = 1700000000000000000ULL;  // some timestamp_ns
    const float px = 1.5f, py = -0.3f, pz = 2.7f;
    const float rx = 0.1f, ry = 0.2f, rz = 0.3f, rw = 0.9274f;
    const float quality = 1.0f;

    // Build a 0x01 packet.
    std::vector<uint8_t> pkt;
    write_u8(pkt, 0x01);
    write_u64_le(pkt, ts);
    write_f32_le(pkt, px);
    write_f32_le(pkt, py);
    write_f32_le(pkt, pz);
    write_f32_le(pkt, rx);
    write_f32_le(pkt, ry);
    write_f32_le(pkt, rz);
    write_f32_le(pkt, rw);
    write_f32_le(pkt, quality);
    assert(pkt.size() == 41);

    std::vector<uint8_t> file_buf;
    write_bin_record(file_buf, pkt);
    std::string path = write_temp_file(file_buf);

    sb_receiver_t *rx_h = sb_receiver_from_file(path.c_str(), false);
    assert(rx_h && "sb_receiver_from_file failed");

    sb_start(rx_h);
    // Wait for the replay thread to finish.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sb_stop(rx_h);

    sb_pose_t pose{};
    bool got = sb_get_latest_pose(rx_h, &pose);
    assert(got && "expected a pose");
    assert(pose.timestamp_ns == ts);
    assert(std::abs(pose.pos[0] - px) < 1e-6f);
    assert(std::abs(pose.pos[1] - py) < 1e-6f);
    assert(std::abs(pose.pos[2] - pz) < 1e-6f);
    // parse_pose normalizes the quaternion, so compare against the unit form
    // of what was written rather than the raw bytes.
    const float qn = std::sqrt(rx * rx + ry * ry + rz * rz + rw * rw);
    assert(std::abs(pose.rot[0] - rx / qn) < 1e-6f);
    assert(std::abs(pose.rot[1] - ry / qn) < 1e-6f);
    assert(std::abs(pose.rot[2] - rz / qn) < 1e-6f);
    assert(std::abs(pose.rot[3] - rw / qn) < 1e-6f);
    assert(std::abs(pose.tracking_quality - quality) < 1e-6f);

    sb_receiver_destroy(rx_h);
    std::remove(path.c_str());
    std::printf("PASS test_pose_roundtrip\n");
}

// ---------------------------------------------------------------------------
// Test 5: Plane packet roundtrip through file replay
// ---------------------------------------------------------------------------

static void test_plane_roundtrip() {
    const uint64_t ts = 9000000000000000000ULL;
    uint8_t uuid[16];
    for (int i = 0; i < 16; ++i) uuid[i] = static_cast<uint8_t>(i * 7 + 3);
    const float cx = 0.5f, cy = 1.0f, cz = -2.0f;
    const float nx = 0.0f, ny = 1.0f, nz = 0.0f;
    const float ew = 1.2f, eh = 0.8f;
    const uint8_t alignment  = 0;  // horizontal
    const uint8_t is_removed = 0;

    std::vector<uint8_t> pkt;
    write_u8(pkt, 0x02);
    write_u64_le(pkt, ts);
    pkt.insert(pkt.end(), uuid, uuid + 16);
    write_f32_le(pkt, cx);
    write_f32_le(pkt, cy);
    write_f32_le(pkt, cz);
    write_f32_le(pkt, nx);
    write_f32_le(pkt, ny);
    write_f32_le(pkt, nz);
    write_f32_le(pkt, ew);
    write_f32_le(pkt, eh);
    write_u8(pkt, alignment);
    write_u8(pkt, is_removed);
    assert(pkt.size() == 59);

    std::vector<uint8_t> file_buf;
    write_bin_record(file_buf, pkt);
    std::string path = write_temp_file(file_buf);

    sb_receiver_t *rx_h = sb_receiver_from_file(path.c_str(), false);
    assert(rx_h);
    sb_start(rx_h);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sb_stop(rx_h);

    sb_plane_t planes[8];
    int n = sb_get_planes(rx_h, planes, 8);
    assert(n == 1 && "expected 1 plane");
    assert(planes[0].timestamp_ns == ts);
    assert(std::memcmp(planes[0].uuid, uuid, 16) == 0);
    assert(std::abs(planes[0].center[0] - cx) < 1e-6f);
    assert(std::abs(planes[0].center[1] - cy) < 1e-6f);
    assert(std::abs(planes[0].center[2] - cz) < 1e-6f);
    assert(std::abs(planes[0].normal[0] - nx) < 1e-6f);
    assert(std::abs(planes[0].normal[1] - ny) < 1e-6f);
    assert(std::abs(planes[0].normal[2] - nz) < 1e-6f);
    assert(std::abs(planes[0].extent[0] - ew) < 1e-6f);
    assert(std::abs(planes[0].extent[1] - eh) < 1e-6f);
    assert(planes[0].alignment == alignment);
    assert(planes[0].is_removed == is_removed);

    sb_receiver_destroy(rx_h);
    std::remove(path.c_str());
    std::printf("PASS test_plane_roundtrip\n");
}

// ---------------------------------------------------------------------------
// Test 6: Known multi-byte value roundtrip — verify exact byte order
// ---------------------------------------------------------------------------

static void test_known_u64_byte_order() {
    // Write 0xDEADBEEFCAFEBABE as a timestamp, read back as encoded bytes.
    const uint64_t known = 0xDEADBEEFCAFEBABEULL;
    std::vector<uint8_t> pkt;
    write_u8(pkt, 0x01);       // pose type
    write_u64_le(pkt, known);  // timestamp at offset 1
    // Pad the rest of the 41 bytes with zeros, except rot_w=1 at offset 33 —
    // parse_pose drops degenerate quaternions, and this test is about the
    // timestamp bytes, not the rotation.
    while (pkt.size() < 33) write_u8(pkt, 0x00);
    write_f32_le(pkt, 1.0f);  // rot_w
    while (pkt.size() < 41) write_u8(pkt, 0x00);

    // Verify raw byte order in the buffer (little-endian).
    assert(pkt[1] == 0xBE && "LE byte 0");
    assert(pkt[2] == 0xBA && "LE byte 1");
    assert(pkt[3] == 0xFE && "LE byte 2");
    assert(pkt[4] == 0xCA && "LE byte 3");
    assert(pkt[5] == 0xEF && "LE byte 4");
    assert(pkt[6] == 0xBE && "LE byte 5");
    assert(pkt[7] == 0xAD && "LE byte 6");
    assert(pkt[8] == 0xDE && "LE byte 7");

    // Now roundtrip through file replay and verify the decoded value.
    std::vector<uint8_t> file_buf;
    write_bin_record(file_buf, pkt);
    std::string path = write_temp_file(file_buf);

    sb_receiver_t *rx_h = sb_receiver_from_file(path.c_str(), false);
    assert(rx_h);
    sb_start(rx_h);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sb_stop(rx_h);

    sb_pose_t pose{};
    bool got = sb_get_latest_pose(rx_h, &pose);
    assert(got);
    assert(pose.timestamp_ns == known && "u64 roundtrip byte order mismatch");

    sb_receiver_destroy(rx_h);
    std::remove(path.c_str());
    std::printf("PASS test_known_u64_byte_order\n");
}

// ---------------------------------------------------------------------------
// Test 7: Seqlock concurrent stress test
//
// 8 reader threads + 1 writer thread. The writer updates poses at ~60Hz for 3
// seconds. Readers spin calling sb_get_latest_pose() and check that each
// returned pose is internally consistent (pos/rot values were all written in
// the same "atomic" update).
//
// We detect torn reads by encoding a generation counter into both pos[0] and
// rot[0] and verifying they match every time a pose is returned.
// ---------------------------------------------------------------------------

static std::atomic<bool> g_stress_stop{false};
static std::atomic<int> g_torn_reads{0};
static std::atomic<int> g_total_reads{0};

// Create a minimal valid 0x01 pose packet with a given generation.
static std::vector<uint8_t> make_pose_packet(uint32_t gen) {
    float fgen;
    std::memcpy(&fgen, &gen, sizeof(fgen));  // reinterpret bits for uniqueness
    uint64_t ts = static_cast<uint64_t>(gen) * 1000000ULL;
    std::vector<uint8_t> pkt;
    write_u8(pkt, 0x01);
    write_u64_le(pkt, ts);
    write_f32_le(pkt, fgen);  // pos[0]
    write_f32_le(pkt, fgen);  // pos[1]
    write_f32_le(pkt, fgen);  // pos[2]
    // parse_pose normalizes the quaternion, so rot can no longer carry the
    // generation bits; tracking_quality does, and it sits at the far end of
    // sb_pose_t, so pos[0]-vs-quality spans the whole payload.
    write_f32_le(pkt, 0.0f);  // rot[0]
    write_f32_le(pkt, 0.0f);  // rot[1]
    write_f32_le(pkt, 0.0f);  // rot[2]
    write_f32_le(pkt, 1.0f);  // rot[3]
    write_f32_le(pkt, fgen);  // quality
    return pkt;
}

static void test_seqlock_stress() {
    // Build a .bin file with N_PACKETS distinct poses.
    static constexpr int N_PACKETS = 200;
    std::vector<uint8_t> file_buf;
    for (int i = 1; i <= N_PACKETS; ++i) {
        auto pkt = make_pose_packet(static_cast<uint32_t>(i));
        write_bin_record(file_buf, pkt);
    }
    std::string path = write_temp_file(file_buf);

    // We create a separate receiver for the stress test; the readers query it.
    // The file replays at full speed (no inter-packet delay because timestamps
    // grow by 1ms each packet — very fast replay).
    sb_receiver_t *rx_h = sb_receiver_from_file(path.c_str(), /*loop=*/true);
    assert(rx_h);

    g_stress_stop.store(false);
    g_torn_reads.store(0);
    g_total_reads.store(0);

    // Launch 8 reader threads.
    static constexpr int N_READERS = 8;
    std::vector<std::thread> readers;
    readers.reserve(N_READERS);
    for (int t = 0; t < N_READERS; ++t) {
        readers.emplace_back([rx_h]() {
            while (!g_stress_stop.load(std::memory_order_relaxed)) {
                sb_pose_t pose{};
                if (sb_get_latest_pose(rx_h, &pose)) {
                    g_total_reads.fetch_add(1, std::memory_order_relaxed);
                    // pos[] and tracking_quality carry the same bit pattern
                    // (see make_pose_packet) and sit at opposite ends of
                    // sb_pose_t — a mismatch means a torn write.
                    uint32_t p0, r0;
                    std::memcpy(&p0, &pose.pos[0], 4);
                    std::memcpy(&r0, &pose.tracking_quality, 4);
                    if (p0 != r0) {
                        g_torn_reads.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }
        });
    }

    sb_start(rx_h);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    g_stress_stop.store(true);
    sb_stop(rx_h);

    for (auto &t : readers) t.join();

    int torn  = g_torn_reads.load();
    int total = g_total_reads.load();
    std::printf("  seqlock stress: %d total reads, %d torn reads\n", total, torn);
    assert(torn == 0 && "seqlock torn read detected!");

    sb_receiver_destroy(rx_h);
    std::remove(path.c_str());
    std::printf("PASS test_seqlock_stress\n");
}

// ---------------------------------------------------------------------------
// Test 8: sb_get_latest_pose returns false when called twice without a new pose
// ---------------------------------------------------------------------------

static void test_pose_edge_triggered() {
    std::vector<uint8_t> pkt = make_pose_packet(42);
    std::vector<uint8_t> file_buf;
    write_bin_record(file_buf, pkt);
    std::string path = write_temp_file(file_buf);

    sb_receiver_t *rx_h = sb_receiver_from_file(path.c_str(), false);
    assert(rx_h);
    sb_start(rx_h);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sb_stop(rx_h);

    sb_pose_t p{};
    bool first  = sb_get_latest_pose(rx_h, &p);
    bool second = sb_get_latest_pose(rx_h, &p);
    assert(first && "first call should return true");
    assert(!second && "second call with no new pose should return false");

    sb_receiver_destroy(rx_h);
    std::remove(path.c_str());
    std::printf("PASS test_pose_edge_triggered\n");
}

// ---------------------------------------------------------------------------
// Test 8b: 45-byte pose carries session_epoch; 41-byte pose still parses
// ---------------------------------------------------------------------------

// Replay one datagram from a .bin and return the receiver (started + stopped).
static sb_receiver_t *replay_one(const std::vector<uint8_t> &pkt, std::string *path_out) {
    std::vector<uint8_t> file_buf;
    write_bin_record(file_buf, pkt);
    *path_out = write_temp_file(file_buf);
    sb_receiver_t *rx_h = sb_receiver_from_file(path_out->c_str(), false);
    assert(rx_h);
    sb_start(rx_h);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sb_stop(rx_h);
    return rx_h;
}

static void test_pose_session_epoch() {
    std::vector<uint8_t> v2 = make_pose_packet(3);
    write_u32_le(v2, 7u);  // session_epoch tail
    assert(v2.size() == 45);

    std::string path;
    sb_receiver_t *rx_h = replay_one(v2, &path);
    sb_pose_t p{};
    assert(sb_get_latest_pose(rx_h, &p) && "45-byte pose must parse");
    assert(p.session_epoch == 7u && "epoch must decode from offset 41");
    assert(sb_get_session_epoch(rx_h) == 7u && "epoch getter must see 7");
    // Level-triggered: unlike sb_get_latest_pose it keeps answering.
    assert(sb_get_session_epoch(rx_h) == 7u && "epoch getter must not be edge-triggered");
    sb_receiver_destroy(rx_h);
    std::remove(path.c_str());

    // A pre-epoch 41-byte sender still parses; its epoch reads 0. The stored
    // epoch mirrors whatever the last pose carried (rather than latching the
    // last nonzero value) so sb_pose_t::session_epoch and sb_get_session_epoch()
    // can never disagree; 0 means "unknown", which consumers treat as
    // continuity and never as a world-frame reset.
    std::vector<uint8_t> v1 = make_pose_packet(3);
    assert(v1.size() == 41);
    rx_h = replay_one(v1, &path);
    sb_pose_t p1{};
    assert(sb_get_latest_pose(rx_h, &p1) && "41-byte pose must still parse");
    assert(p1.pos[0] == p.pos[0] && "41-byte body must decode identically");
    assert(p1.session_epoch == 0u && "pre-epoch pose must report epoch 0");
    assert(sb_get_session_epoch(rx_h) == 0u && "pre-epoch session must report epoch 0");
    sb_receiver_destroy(rx_h);
    std::remove(path.c_str());

    std::printf("PASS test_pose_session_epoch\n");
}

// ---------------------------------------------------------------------------
// Test 9: Hand packet size is exactly 431 bytes
// ---------------------------------------------------------------------------

static void test_hand_packet_size() {
    // 1 (type) + 8 (ts) + 1 (hand_index) + 1 (joint_count) + 21*5*4 (joints) = 431
    static_assert(1 + 8 + 1 + 1 + 21 * 5 * 4 == 431, "hand packet size must be 431");
    std::printf("PASS test_hand_packet_size\n");
}

// ---------------------------------------------------------------------------
// Test 10: Hand packet roundtrip through file replay
// ---------------------------------------------------------------------------

static void test_hand_roundtrip() {
    const uint64_t ts         = 1800000000000000000ULL;
    const uint8_t hand_index  = 1;  // right hand
    const uint8_t joint_count = 21;

    // Build a 0x05 packet with known joint values.
    // Each joint's x,y,z = joint_index * 0.01, confidence = 0.9, reserved = 0.
    std::vector<uint8_t> pkt;
    write_u8(pkt, 0x05);
    write_u64_le(pkt, ts);
    write_u8(pkt, hand_index);
    write_u8(pkt, joint_count);
    for (int j = 0; j < 21; ++j) {
        float val = static_cast<float>(j) * 0.01f;
        write_f32_le(pkt, val);         // x
        write_f32_le(pkt, val + 1.0f);  // y
        write_f32_le(pkt, val + 2.0f);  // z
        write_f32_le(pkt, 0.9f);        // confidence
        write_f32_le(pkt, 0.0f);        // reserved
    }
    assert(pkt.size() == 431);

    std::vector<uint8_t> file_buf;
    write_bin_record(file_buf, pkt);
    std::string path = write_temp_file(file_buf);

    sb_receiver_t *rx_h = sb_receiver_from_file(path.c_str(), false);
    assert(rx_h && "sb_receiver_from_file failed");

    sb_start(rx_h);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sb_stop(rx_h);

    // Left hand should have no data.
    sb_hand_t left{};
    assert(!sb_get_hand(rx_h, 0, &left) && "left hand should have no data");

    // Right hand should have our data.
    sb_hand_t right{};
    bool got = sb_get_hand(rx_h, 1, &right);
    assert(got && "expected right hand data");
    assert(right.timestamp_ns == ts);
    assert(right.hand_index == 1);
    assert(right.joint_count == 21);

    // Verify a few joints.
    for (int j = 0; j < 21; ++j) {
        float expected_x = static_cast<float>(j) * 0.01f;
        assert(std::abs(right.joints[j][0] - expected_x) < 1e-6f);
        assert(std::abs(right.joints[j][1] - (expected_x + 1.0f)) < 1e-6f);
        assert(std::abs(right.joints[j][2] - (expected_x + 2.0f)) < 1e-6f);
        assert(std::abs(right.joints[j][3] - 0.9f) < 1e-6f);
        assert(std::abs(right.joints[j][4] - 0.0f) < 1e-6f);
    }

    // Edge-triggered: second call should return false.
    assert(!sb_get_hand(rx_h, 1, &right) && "second call should return false");

    sb_receiver_destroy(rx_h);
    std::remove(path.c_str());
    std::printf("PASS test_hand_roundtrip\n");
}

// ---------------------------------------------------------------------------
// Chunked-frame (0x09) reassembly tests
//
// 0x09 header: type | ts(u64) | total(u32) | offset(u32) | len(u16) | payload.
// MUST match FRAME_CHUNK_PAYLOAD in receiver.cpp.
// ---------------------------------------------------------------------------

static const uint32_t FX_CHUNK_PAYLOAD = 1200;

static std::vector<uint8_t> make_frame_chunk(uint64_t ts, uint32_t total, uint32_t offset,
                                             const uint8_t *payload, uint16_t len) {
    std::vector<uint8_t> pkt;
    write_u8(pkt, 0x09);
    write_u64_le(pkt, ts);
    write_u32_le(pkt, total);
    write_u32_le(pkt, offset);
    write_u16_le(pkt, len);
    pkt.insert(pkt.end(), payload, payload + len);
    return pkt;
}

// Build the list of chunk packets for the fixture JPEG at timestamp ts.
static std::vector<std::vector<uint8_t>> fixture_chunks(uint64_t ts) {
    std::vector<std::vector<uint8_t>> chunks;
    for (uint32_t off = 0; off < TEST_JPEG_LEN; off += FX_CHUNK_PAYLOAD) {
        uint16_t len = static_cast<uint16_t>(std::min(FX_CHUNK_PAYLOAD, TEST_JPEG_LEN - off));
        chunks.push_back(make_frame_chunk(ts, TEST_JPEG_LEN, off, TEST_JPEG + off, len));
    }
    return chunks;
}

// Replay `records` and return the latest decoded frame.  `run_ms` is how long
// to let the file replay run before stopping — file replay honors each packet's
// embedded timestamp, so a test that deliberately spaces packets in time (e.g.
// the far-future-timestamp wedge test) must allow enough wall-clock for the
// later packets to actually be delivered.
static bool replay_and_get_frame_ms(const std::vector<std::vector<uint8_t>> &records,
                                    sb_frame_t *out, int run_ms) {
    std::vector<uint8_t> file_buf;
    for (const auto &r : records) write_bin_record(file_buf, r);
    std::string path  = write_temp_file(file_buf);
    sb_receiver_t *rx = sb_receiver_from_file(path.c_str(), false);
    assert(rx && "sb_receiver_from_file failed");
    sb_start(rx);
    std::this_thread::sleep_for(std::chrono::milliseconds(run_ms));
    sb_stop(rx);
    bool got = sb_get_latest_frame(rx, out);
    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    return got;
}

static bool replay_and_get_frame(const std::vector<std::vector<uint8_t>> &records,
                                 sb_frame_t *out) {
    return replay_and_get_frame_ms(records, out, 200);
}

// Build a single whole-frame 0x03 datagram wrapping the fixture JPEG.
static std::vector<uint8_t> make_frame_0x03(uint64_t ts, uint32_t w, uint32_t h,
                                            const uint8_t *jpeg, uint32_t n) {
    std::vector<uint8_t> pkt;
    write_u8(pkt, 0x03);
    write_u64_le(pkt, ts);
    write_u32_le(pkt, w);
    write_u32_le(pkt, h);
    write_u32_le(pkt, n);
    pkt.insert(pkt.end(), jpeg, jpeg + n);
    return pkt;
}

// Build a minimal valid-size 0x01 pose datagram (zero-filled body).  Used only
// to anchor file-replay timing (parse_pose ignores the zeros for our purposes).
static std::vector<uint8_t> make_pose_anchor(uint64_t ts) {
    std::vector<uint8_t> pkt;
    write_u8(pkt, 0x01);
    write_u64_le(pkt, ts);
    pkt.resize(33, 0);
    write_f32_le(pkt, 1.0f);  // rot_w — a zero quat would be dropped
    pkt.resize(41, 0);  // POSE_PACKET_SIZE
    return pkt;
}

// Multi-chunk frame delivered IN ORDER reassembles + decodes to the right size.
static void test_frame_chunk_inorder() {
    assert(TEST_JPEG_LEN > 2 * FX_CHUNK_PAYLOAD && "fixture must span >=3 chunks");
    auto chunks = fixture_chunks(1800000000000000000ULL);
    sb_frame_t f{};
    bool got = replay_and_get_frame(chunks, &f);
    assert(got && "expected a reassembled+decoded frame");
    assert(f.width == TEST_JPEG_W && "decoded width");
    assert(f.height == TEST_JPEG_H && "decoded height");
    sb_free_frame(&f);
    std::printf("PASS test_frame_chunk_inorder\n");
}

// Same chunks delivered REVERSED still reassemble (offset-addressed, not
// order-dependent).
static void test_frame_chunk_out_of_order() {
    auto chunks = fixture_chunks(1800000000000000001ULL);
    std::reverse(chunks.begin(), chunks.end());
    sb_frame_t f{};
    bool got = replay_and_get_frame(chunks, &f);
    assert(got && "out-of-order chunks should still reassemble");
    assert(f.width == TEST_JPEG_W && f.height == TEST_JPEG_H);
    sb_free_frame(&f);
    std::printf("PASS test_frame_chunk_out_of_order\n");
}

// A frame MISSING a chunk must NOT produce a frame (no partial/garbage decode).
static void test_frame_chunk_incomplete() {
    auto chunks = fixture_chunks(1800000000000000002ULL);
    assert(chunks.size() >= 3);
    chunks.erase(chunks.begin() + 1);  // drop the middle chunk
    sb_frame_t f{};
    bool got = replay_and_get_frame(chunks, &f);
    assert(!got && "incomplete frame must not be emitted");
    std::printf("PASS test_frame_chunk_incomplete\n");
}

// A duplicate chunk must not corrupt reassembly or double-count toward
// completion (dedup by chunk index).
static void test_frame_chunk_duplicate() {
    auto chunks = fixture_chunks(1800000000000000003ULL);
    chunks.insert(chunks.begin() + 1, chunks[0]);  // resend chunk 0
    sb_frame_t f{};
    bool got = replay_and_get_frame(chunks, &f);
    assert(got && "duplicate chunk should not prevent completion");
    assert(f.width == TEST_JPEG_W && f.height == TEST_JPEG_H);
    sb_free_frame(&f);
    std::printf("PASS test_frame_chunk_duplicate\n");
}

// A chunk whose offset is NOT a multiple of FRAME_CHUNK_PAYLOAD must be
// rejected: the dedup bitset is indexed by offset/payload but the payload is
// memcpy'd to the raw offset, so a misaligned offset would alias indices and
// corrupt the buffer (it would skip the real chunk 0 as a "duplicate" and leave
// the JPEG's first bytes zeroed → decode fails).  Same timestamp = same frame.
static void test_frame_chunk_misaligned_offset() {
    uint64_t ts = 1800000000000000004ULL;
    auto chunks = fixture_chunks(ts);
    // Misaligned chunk: offset 600 (not a multiple of 1200), valid total/range.
    auto bad = make_frame_chunk(ts, TEST_JPEG_LEN, 600, TEST_JPEG + 600, 1200);
    chunks.insert(chunks.begin(), bad);
    sb_frame_t f{};
    bool got = replay_and_get_frame(chunks, &f);
    assert(got && "misaligned chunk must not block the real frame");
    assert(f.width == TEST_JPEG_W && f.height == TEST_JPEG_H &&
           "misaligned chunk must not corrupt the reassembled JPEG");
    sb_free_frame(&f);
    std::printf("PASS test_frame_chunk_misaligned_offset\n");
}

// Chunks with total==0 or total>MAX_FRAME_BYTES must be rejected before they
// touch reassembly state, so a valid frame interleaved with them still decodes.
static void test_frame_chunk_bad_total() {
    uint64_t ts     = 1800000000000000005ULL;
    auto chunks     = fixture_chunks(ts);
    auto zero_total = make_frame_chunk(ts, 0, 0, TEST_JPEG, 10);
    auto huge_total = make_frame_chunk(ts, (1u << 20) + 1, 0, TEST_JPEG, 10);
    chunks.insert(chunks.begin(), huge_total);
    chunks.insert(chunks.begin(), zero_total);
    sb_frame_t f{};
    bool got = replay_and_get_frame(chunks, &f);
    assert(got && "bad-total chunks must not corrupt a valid frame");
    assert(f.width == TEST_JPEG_W && f.height == TEST_JPEG_H);
    sb_free_frame(&f);
    std::printf("PASS test_frame_chunk_bad_total\n");
}

// A chunk whose offset+len exceeds total is out of bounds and must be rejected
// (else it would OOB-write the reassembly buffer).  A valid frame still decodes.
static void test_frame_chunk_range_oob() {
    uint64_t ts = 1800000000000000006ULL;
    auto chunks = fixture_chunks(ts);
    // offset near the end, len that overruns total.
    auto oob = make_frame_chunk(ts, TEST_JPEG_LEN, TEST_JPEG_LEN - 5, TEST_JPEG, 100);
    chunks.insert(chunks.begin(), oob);
    sb_frame_t f{};
    bool got = replay_and_get_frame(chunks, &f);
    assert(got && "out-of-range chunk must not corrupt a valid frame");
    assert(f.width == TEST_JPEG_W && f.height == TEST_JPEG_H);
    sb_free_frame(&f);
    std::printf("PASS test_frame_chunk_range_oob\n");
}

// REGRESSION (the most serious defect the defense surfaced): a corrupt chunk
// bearing a far-future timestamp must NOT be able to wedge reassembly forever.
// We anchor replay timing with a low-timestamp pose, then deliver a far-future
// (+2.05 s, just past FRAME_TS_MAX_SKEW) lone chunk FIRST, then the real frame.
// Old behaviour: the far-future chunk is adopted as the frame id, every real
// chunk is then ts < frame_id → "stale" → dropped → passthrough frozen forever.
// Fixed behaviour: the >2 s backward gap tells the receiver the in-progress id
// is bogus, it abandons it and reassembles the real frame.
static void test_frame_chunk_far_future_no_wedge() {
    uint64_t base       = 1800000000000000007ULL;
    uint64_t real_ts    = base + 100000000ULL;   // +0.10 s
    uint64_t corrupt_ts = base + 2150000000ULL;  // +2.15 s  (gap to real = 2.05 s > 2 s)
    std::vector<std::vector<uint8_t>> records;
    records.push_back(make_pose_anchor(base));                        // anchors first_pkt_ts low
    records.push_back(make_frame_chunk(corrupt_ts, TEST_JPEG_LEN, 0,  // corrupt, parsed first
                                       TEST_JPEG, 10));
    for (auto &c : fixture_chunks(real_ts)) records.push_back(c);  // real frame, must recover
    sb_frame_t f{};
    bool got = replay_and_get_frame_ms(records, &f, 2600);  // > corrupt relative time
    assert(got && "real frame must reassemble despite a prior far-future chunk (no wedge)");
    assert(f.width == TEST_JPEG_W && f.height == TEST_JPEG_H);
    sb_free_frame(&f);
    std::printf("PASS test_frame_chunk_far_future_no_wedge\n");
}

// REGRESSION (NEW-1 from the defense round 2): a partial frame stranded at
// T_old (a chunk was lost, so reasm never reset) followed by a LEGITIMATE
// resume more than FRAME_TS_MAX_SKEW in the future — exactly what happens when
// the pipeline stalls >2 s (the freeze, backgrounding, ARKit relocalisation)
// and then resumes — must reassemble.  An earlier fix REJECTED forward gaps
// >skew as "corrupt", which permanently re-wedged passthrough in this case:
// every resumed chunk looked too-far-future and was dropped forever.  The real
// frame must win.
static void test_frame_chunk_forward_resume() {
    uint64_t base    = 1800000000000000009ULL;
    uint64_t real_ts = base + 2100000000ULL;  // +2.1 s forward (> 2 s skew)
    std::vector<std::vector<uint8_t>> records;
    // Strand a partial frame at base: only chunk 0 of a 3-chunk frame.
    records.push_back(make_frame_chunk(base, TEST_JPEG_LEN, 0, TEST_JPEG, 1200));
    // Then a full real frame 2.1 s later — a legitimate post-stall resume.
    for (auto &c : fixture_chunks(real_ts)) records.push_back(c);
    sb_frame_t f{};
    bool got = replay_and_get_frame_ms(records, &f, 2600);  // > real_ts relative time
    assert(got && "legitimate >skew-forward resume must reassemble (no wedge)");
    assert(f.width == TEST_JPEG_W && f.height == TEST_JPEG_H);
    sb_free_frame(&f);
    std::printf("PASS test_frame_chunk_forward_resume\n");
}

// The retained whole-frame 0x03 path (kept for recorded-session replay) still
// decodes to the right dimensions — guards it from silent rot now that the
// live sender uses 0x09.
static void test_frame_0x03_roundtrip() {
    auto pkt =
        make_frame_0x03(1800000000000000008ULL, TEST_JPEG_W, TEST_JPEG_H, TEST_JPEG, TEST_JPEG_LEN);
    std::vector<std::vector<uint8_t>> records{pkt};
    sb_frame_t f{};
    bool got = replay_and_get_frame(records, &f);
    assert(got && "0x03 whole-frame path must still decode");
    assert(f.width == TEST_JPEG_W && f.height == TEST_JPEG_H);
    sb_free_frame(&f);
    std::printf("PASS test_frame_0x03_roundtrip\n");
}

static void test_concurrent_receiver_frames() {
    constexpr size_t count = 8;
    const uint64_t t0 = 1800000000000000000ULL;
    auto packet = make_frame_0x03(t0, TEST_JPEG_W, TEST_JPEG_H, TEST_JPEG, TEST_JPEG_LEN);
    sb_frame_t reference{};
    assert(replay_and_get_frame({packet}, &reference));
    std::vector<uint8_t> recording;
    for (uint64_t i = 0; i < 300; ++i)
        write_bin_record(recording, make_frame_0x03(t0 + i * 1000000ULL,
                         TEST_JPEG_W, TEST_JPEG_H, TEST_JPEG, TEST_JPEG_LEN));
    auto path = write_temp_file(recording);
    sb_receiver_t *receivers[count]{};
    unsigned observed[count]{};
    for (auto &rx : receivers) {
        rx = sb_receiver_from_file(path.c_str(), true);
        assert(rx);
        sb_start(rx);
    }
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline) {
        for (size_t i = 0; i < count; ++i) {
            sb_frame_t frame{};
            if (!sb_get_latest_frame(receivers[i], &frame)) continue;
            assert(frame.width == reference.width && frame.height == reference.height);
            assert(std::memcmp(frame.rgba, reference.rgba,
                               TEST_JPEG_W * TEST_JPEG_H * 4) == 0);
            ++observed[i];
            sb_free_frame(&frame);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    for (size_t i = 0; i < count; ++i) {
        sb_stop(receivers[i]);
        sb_receiver_destroy(receivers[i]);
        assert(observed[i] >= 10 && "each receiver must independently decode frames");
    }
    sb_free_frame(&reference);
    std::remove(path.c_str());
    std::printf("PASS test_concurrent_receiver_frames\n");
}

// Copy of the fixture whose SOF0 header claims 65500x16400 (65500 is libjpeg's
// own JPEG_MAX_DIMENSION, so the header parses): width*height*4 overflows
// uint32, so an unchecked decoder would malloc a tiny buffer and let libjpeg
// scribble far past it.  Must be rejected, never decoded.
static std::vector<uint8_t> make_oversize_sof_jpeg() {
    std::vector<uint8_t> j(TEST_JPEG, TEST_JPEG + TEST_JPEG_LEN);
    size_t sof = 0;
    for (size_t i = 0; i + 8 < j.size(); ++i) {
        if (j[i] == 0xFF && j[i + 1] == 0xC0) {
            sof = i;
            break;
        }
    }
    assert(sof != 0 && "fixture must contain an SOF0 marker");
    // SOF0: FF C0 | len(2) | precision(1) | height(2) | width(2) ...  (big-endian)
    j[sof + 5] = 0x40;
    j[sof + 6] = 0x10;  // height = 16400
    j[sof + 7] = 0xFF;
    j[sof + 8] = 0xDC;  // width = 65500
    return j;
}

static void test_frame_oversize_sof_rejected() {
    auto big = make_oversize_sof_jpeg();
    const uint64_t t0 = 1800000000000000000ULL;
    // Bad frame first, then a good one 50 ms later: the decoder must survive
    // the rejection and still decode the good frame afterwards.
    std::vector<std::vector<uint8_t>> records{
        make_frame_0x03(t0, 65500, 16400, big.data(), static_cast<uint32_t>(big.size())),
        make_frame_0x03(t0 + 50000000ULL, TEST_JPEG_W, TEST_JPEG_H, TEST_JPEG, TEST_JPEG_LEN),
    };
    sb_frame_t f{};
    bool got = replay_and_get_frame_ms(records, &f, 300);
    assert(got && "valid frame after an oversize one must still decode");
    assert(f.width == TEST_JPEG_W && f.height == TEST_JPEG_H);
    sb_free_frame(&f);

    // Oversize frame alone via the 0x09 chunk path: nothing must come out.
    std::vector<std::vector<uint8_t>> chunks;
    for (uint32_t off = 0; off < big.size(); off += FX_CHUNK_PAYLOAD) {
        uint32_t n = std::min<uint32_t>(FX_CHUNK_PAYLOAD, static_cast<uint32_t>(big.size()) - off);
        chunks.push_back(make_frame_chunk(t0, static_cast<uint32_t>(big.size()), off,
                                          big.data() + off, n));
    }
    sb_frame_t g{};
    assert(!replay_and_get_frame(chunks, &g) && "oversize SOF must never produce a frame");
    std::printf("PASS test_frame_oversize_sof_rejected\n");
}

// ---------------------------------------------------------------------------
// Non-finite float rejection
// ---------------------------------------------------------------------------

static std::vector<uint8_t> make_plane_packet(uint64_t ts, float nx, float ny, float nz) {
    std::vector<uint8_t> pkt;
    write_u8(pkt, 0x02);
    write_u64_le(pkt, ts);
    for (int i = 0; i < 16; ++i) write_u8(pkt, static_cast<uint8_t>(i));
    write_f32_le(pkt, 0.0f);
    write_f32_le(pkt, 0.0f);
    write_f32_le(pkt, 0.0f);
    write_f32_le(pkt, nx);
    write_f32_le(pkt, ny);
    write_f32_le(pkt, nz);
    write_f32_le(pkt, 1.0f);
    write_f32_le(pkt, 1.0f);
    write_u8(pkt, 0);
    write_u8(pkt, 0);
    assert(pkt.size() == 59);
    return pkt;
}

static void test_plane_nan_normal_ignored() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::vector<std::vector<uint8_t>> records{
        make_plane_packet(1800000000000000000ULL, nan, 1.0f, 0.0f),
        make_plane_packet(1800000000000000000ULL, 0.0f, 0.0f, 0.0f),  // zero-length normal
    };
    std::vector<uint8_t> file_buf;
    for (const auto &r : records) write_bin_record(file_buf, r);
    std::string path  = write_temp_file(file_buf);
    sb_receiver_t *rx = sb_receiver_from_file(path.c_str(), false);
    assert(rx);
    sb_start(rx);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sb_stop(rx);
    sb_plane_t planes[8];
    assert(sb_get_planes(rx, planes, 8) == 0 && "invalid plane must not enter the world model");
    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    std::printf("PASS test_plane_nan_normal_ignored\n");
}

static void test_hand_nan_joint_ignored() {
    std::vector<uint8_t> pkt;
    write_u8(pkt, 0x05);
    write_u64_le(pkt, 1800000000000000000ULL);
    write_u8(pkt, 1);
    write_u8(pkt, 21);
    for (int j = 0; j < 21; ++j) {
        write_f32_le(pkt, j == 7 ? std::numeric_limits<float>::infinity() : 0.1f);
        write_f32_le(pkt, j == 12 ? std::numeric_limits<float>::quiet_NaN() : 0.2f);
        write_f32_le(pkt, 0.3f);
        write_f32_le(pkt, 0.9f);
        write_f32_le(pkt, 0.0f);
    }
    assert(pkt.size() == 431);
    std::vector<uint8_t> file_buf;
    write_bin_record(file_buf, pkt);
    std::string path  = write_temp_file(file_buf);
    sb_receiver_t *rx = sb_receiver_from_file(path.c_str(), false);
    assert(rx);
    sb_start(rx);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sb_stop(rx);
    sb_hand_t h{};
    assert(!sb_get_hand(rx, 1, &h) && "hand with non-finite joint must be dropped");
    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    std::printf("PASS test_hand_nan_joint_ignored\n");
}

static void test_pose_nan_ignored() {
    std::vector<uint8_t> pkt;
    write_u8(pkt, 0x01);
    write_u64_le(pkt, 1800000000000000000ULL);
    for (int i = 0; i < 3; ++i) write_f32_le(pkt, 1.0f);
    write_f32_le(pkt, 0.0f);
    write_f32_le(pkt, 0.0f);
    write_f32_le(pkt, std::numeric_limits<float>::quiet_NaN());
    write_f32_le(pkt, 1.0f);
    write_f32_le(pkt, 1.0f);
    assert(pkt.size() == 41);
    std::vector<uint8_t> file_buf;
    write_bin_record(file_buf, pkt);
    std::string path  = write_temp_file(file_buf);
    sb_receiver_t *rx = sb_receiver_from_file(path.c_str(), false);
    assert(rx);
    sb_start(rx);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sb_stop(rx);
    sb_pose_t p{};
    assert(!sb_get_latest_pose(rx, &p) && "pose with NaN quaternion must be dropped");
    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    std::printf("PASS test_pose_nan_ignored\n");
}

// ---------------------------------------------------------------------------
// Camera-intrinsics (0x0A) tests
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Quaternion hygiene: parse_pose must publish unit quats, and drop degenerate
// ones instead of substituting identity (see the WHY in receiver.cpp).
// ---------------------------------------------------------------------------

static std::vector<uint8_t> make_pose_packet_rot(uint64_t ts, float x, float y,
                                                 float z, float w) {
    std::vector<uint8_t> pkt;
    write_u8(pkt, 0x01);
    write_u64_le(pkt, ts);
    for (int i = 0; i < 3; ++i) write_f32_le(pkt, 0.0f);
    write_f32_le(pkt, x);
    write_f32_le(pkt, y);
    write_f32_le(pkt, z);
    write_f32_le(pkt, w);
    write_f32_le(pkt, 1.0f);
    assert(pkt.size() == 41);
    return pkt;
}

static void test_pose_quat_normalized() {
    // (1.8, 0, 0, 2.4) has length 3 → unit form (0.6, 0, 0, 0.8).
    std::vector<uint8_t> file_buf;
    write_bin_record(file_buf, make_pose_packet_rot(1900000000000000000ULL,
                                                    1.8f, 0.0f, 0.0f, 2.4f));
    std::string path  = write_temp_file(file_buf);
    sb_receiver_t *rx = sb_receiver_from_file(path.c_str(), false);
    assert(rx);
    sb_start(rx);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sb_stop(rx);
    sb_pose_t p{};
    assert(sb_get_latest_pose(rx, &p) && "non-unit quat must still be accepted");
    float len = std::sqrt(p.rot[0] * p.rot[0] + p.rot[1] * p.rot[1] +
                          p.rot[2] * p.rot[2] + p.rot[3] * p.rot[3]);
    assert(std::abs(len - 1.0f) < 1e-6f && "quat must be unit length");
    assert(std::abs(p.rot[0] - 0.6f) < 1e-6f);
    assert(std::abs(p.rot[3] - 0.8f) < 1e-6f);
    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    std::printf("PASS test_pose_quat_normalized\n");
}

static void test_pose_zero_quat_dropped() {
    std::vector<uint8_t> file_buf;
    write_bin_record(file_buf, make_pose_packet_rot(1900000000000000001ULL,
                                                    0.0f, 0.0f, 0.0f, 0.0f));
    std::string path  = write_temp_file(file_buf);
    sb_receiver_t *rx = sb_receiver_from_file(path.c_str(), false);
    assert(rx);
    sb_start(rx);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sb_stop(rx);
    sb_pose_t p{};
    assert(!sb_get_latest_pose(rx, &p) && "zero quaternion must be dropped");
    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    std::printf("PASS test_pose_zero_quat_dropped\n");
}

static void test_intrinsics_packet_size() {
    // 1 (type) + 8 (ts) + 6*4 (fx,fy,cx,cy,img_w,img_h) = 33
    static_assert(1 + 8 + 4 * 6 == 33, "intrinsics packet size must be 33");
    std::printf("PASS test_intrinsics_packet_size\n");
}

static std::vector<uint8_t> make_intrinsics_packet(uint64_t ts, float fx, float fy, float cx,
                                                   float cy, float iw, float ih) {
    std::vector<uint8_t> pkt;
    write_u8(pkt, 0x0A);
    write_u64_le(pkt, ts);
    write_f32_le(pkt, fx);
    write_f32_le(pkt, fy);
    write_f32_le(pkt, cx);
    write_f32_le(pkt, cy);
    write_f32_le(pkt, iw);
    write_f32_le(pkt, ih);
    return pkt;
}

// Roundtrip through file replay; also asserts the getter is STICKY (latest-wins),
// not edge-triggered — the passthrough/driver read it every frame.
static void test_intrinsics_roundtrip() {
    const uint64_t ts = 1900000000000000000ULL;
    const float fx = 1361.5f, fy = 1362.0f, cx = 956.2f, cy = 727.2f;
    const float iw = 1920.0f, ih = 1440.0f;

    auto pkt = make_intrinsics_packet(ts, fx, fy, cx, cy, iw, ih);
    assert(pkt.size() == 33);

    std::vector<uint8_t> file_buf;
    write_bin_record(file_buf, pkt);
    std::string path = write_temp_file(file_buf);

    sb_receiver_t *rx_h = sb_receiver_from_file(path.c_str(), false);
    assert(rx_h && "sb_receiver_from_file failed");
    sb_start(rx_h);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sb_stop(rx_h);

    sb_intrinsics_t in{};
    bool got = sb_get_latest_intrinsics(rx_h, &in);
    assert(got && "expected intrinsics");
    assert(in.timestamp_ns == ts);
    assert(std::abs(in.fx - fx) < 1e-3f);
    assert(std::abs(in.fy - fy) < 1e-3f);
    assert(std::abs(in.cx - cx) < 1e-3f);
    assert(std::abs(in.cy - cy) < 1e-3f);
    assert(std::abs(in.image_width - iw) < 1e-3f);
    assert(std::abs(in.image_height - ih) < 1e-3f);

    // Sticky: a second call (no new packet) still returns the latest value.
    sb_intrinsics_t in2{};
    bool got2 = sb_get_latest_intrinsics(rx_h, &in2);
    assert(got2 && "intrinsics getter must be sticky (latest-wins), not edge-triggered");
    assert(in2.timestamp_ns == ts);

    sb_receiver_destroy(rx_h);
    std::remove(path.c_str());
    std::printf("PASS test_intrinsics_roundtrip\n");
}

// A session with no 0x0A packets → getter returns false (never written).
static void test_intrinsics_absent() {
    std::vector<uint8_t> file_buf;
    write_bin_record(file_buf, make_pose_packet(7));
    std::string path = write_temp_file(file_buf);

    sb_receiver_t *rx_h = sb_receiver_from_file(path.c_str(), false);
    assert(rx_h);
    sb_start(rx_h);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sb_stop(rx_h);

    sb_intrinsics_t in{};
    assert(!sb_get_latest_intrinsics(rx_h, &in) && "no 0x0A → intrinsics must be absent");

    sb_receiver_destroy(rx_h);
    std::remove(path.c_str());
    std::printf("PASS test_intrinsics_absent\n");
}

// Cross-process shm relay: writer → reader roundtrip, and the intrinsics seqlock
// is INDEPENDENT of the pose seqlock (writing one must not mark the other ready).
static void test_shm_intrinsics_roundtrip() {
    const char *name = "/sb_test_intrinsics_shm";
    sb_shm_writer_t *w = sb_shm_writer_create(name);
    assert(w && "shm writer create");
    sb_shm_reader_t *r = sb_shm_reader_open(name);
    assert(r && "shm reader open");

    // Nothing written yet → both absent.
    sb_intrinsics_t out{};
    assert(!sb_shm_reader_get_intrinsics(r, &out) && "intrinsics absent before first write");
    sb_pose_t p{};
    assert(!sb_shm_reader_get_pose(r, &p) && "pose absent before first write");

    sb_intrinsics_t in{};
    in.timestamp_ns = 4242ULL;
    in.fx = 1360.0f; in.fy = 1361.0f; in.cx = 956.0f; in.cy = 727.0f;
    in.image_width = 1920.0f; in.image_height = 1440.0f;
    sb_shm_writer_update_intrinsics(w, &in);

    assert(sb_shm_reader_get_intrinsics(r, &out) && "intrinsics present after write");
    assert(out.timestamp_ns == in.timestamp_ns);
    assert(std::abs(out.fx - in.fx) < 1e-3f);
    assert(std::abs(out.cy - in.cy) < 1e-3f);
    assert(std::abs(out.image_width - in.image_width) < 1e-3f);

    // Independent seqlocks: writing intrinsics must NOT make the pose "ready".
    assert(!sb_shm_reader_get_pose(r, &p) && "pose seqlock must be independent of intrinsics");

    sb_shm_reader_destroy(r);
    sb_shm_writer_destroy(w);
    std::printf("PASS test_shm_intrinsics_roundtrip\n");
}

// A writer that crashes mid-update leaves the seqlock odd forever.  The reader
// must give up and return false rather than spin the compositor thread.  The
// segment is poked directly (seq is the first u32; intr_seq follows the
// 8-byte-aligned pose) since there is no library hook for a torn writer.
static void test_shm_reader_stuck_seqlock_gives_up() {
    const char *name    = "/sb_test_stuck_shm";
    sb_shm_writer_t *w  = sb_shm_writer_create(name);
    assert(w && "shm writer create");
    sb_shm_reader_t *r  = sb_shm_reader_open(name);
    assert(r && "shm reader open");

    int fd = shm_open(name, O_RDWR, 0);
    assert(fd >= 0);
    const size_t intr_seq_off = 8 + sizeof(sb_pose_t);
    const size_t map_len      = intr_seq_off + 4;
    void *mem = mmap(nullptr, map_len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    assert(mem != MAP_FAILED);
    auto *seq      = reinterpret_cast<std::atomic<uint32_t> *>(mem);
    auto *intr_seq = reinterpret_cast<std::atomic<uint32_t> *>(static_cast<uint8_t *>(mem) +
                                                               intr_seq_off);

    sb_pose_t pose{};
    pose.timestamp_ns = 42;
    sb_shm_writer_update(w, &pose);
    sb_pose_t out{};
    assert(sb_shm_reader_get_pose(r, &out) && out.timestamp_ns == 42);

    seq->fetch_add(1, std::memory_order_release);  // simulate writer death mid-update
    assert((seq->load() & 1u) && "seq must be odd");
    auto t0 = std::chrono::steady_clock::now();
    assert(!sb_shm_reader_get_pose(r, &out) && "stuck seqlock must not yield a pose");
    intr_seq->fetch_add(1, std::memory_order_release);
    sb_intrinsics_t in{};
    assert(!sb_shm_reader_get_intrinsics(r, &in) && "stuck intrinsics seqlock must give up");
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now() - t0)
                  .count();
    assert(ms < 1000 && "reader must give up promptly");

    seq->fetch_add(1, std::memory_order_release);  // writer recovers: seq even again
    assert(sb_shm_reader_get_pose(r, &out) && out.timestamp_ns == 42);

    munmap(mem, map_len);
    close(fd);
    sb_shm_reader_destroy(r);
    sb_shm_writer_destroy(w);
    std::printf("PASS test_shm_reader_stuck_seqlock_gives_up\n");
}

// ---------------------------------------------------------------------------
// Depth-map (0x0B) tests
// ---------------------------------------------------------------------------

static void test_depth_subheader_size() {
    // sub-header: u16 width + u16 height + u8 encoding + f32 z_min + f32 z_max = 13
    static_assert(2 + 2 + 1 + 4 + 4 == 13, "depth sub-header must be 13 bytes");
    std::printf("PASS test_depth_subheader_size\n");
}

static std::vector<uint8_t> make_depth_chunk(uint64_t ts, uint32_t total, uint32_t offset,
                                             const uint8_t *payload, uint16_t len) {
    std::vector<uint8_t> pkt;
    write_u8(pkt, 0x0B);
    write_u64_le(pkt, ts);
    write_u32_le(pkt, total);
    write_u32_le(pkt, offset);
    write_u16_le(pkt, len);
    pkt.insert(pkt.end(), payload, payload + len);
    return pkt;
}

// Build a 0x0B reassembled payload (13-byte sub-header + u16 plane) from Z values
// in metres. NaN / out-of-[z_min,z_max] encodes the 0 sentinel (invalid).
static std::vector<uint8_t> make_depth_payload(uint16_t w, uint16_t h, float z_min, float z_max,
                                               const std::vector<float> &zs) {
    std::vector<uint8_t> p;
    write_u16_le(p, w);
    write_u16_le(p, h);
    write_u8(p, 1);  // encoding = u16 linear
    write_f32_le(p, z_min);
    write_f32_le(p, z_max);
    size_t count = (size_t)w * h;
    for (size_t i = 0; i < count; ++i) {
        float Z = (i < zs.size()) ? zs[i] : z_min;
        uint16_t q;
        if (!(Z > z_min) || !(Z < z_max)) {
            q = 0;  // invalid sentinel
        } else {
            long v = std::lround((Z - z_min) / (z_max - z_min) * 65535.0f);
            if (v < 1) v = 1;
            if (v > 65535) v = 65535;
            q = (uint16_t)v;
        }
        write_u16_le(p, q);
    }
    return p;
}

static std::vector<std::vector<uint8_t>> depth_chunks(uint64_t ts,
                                                      const std::vector<uint8_t> &payload) {
    std::vector<std::vector<uint8_t>> chunks;
    uint32_t total = (uint32_t)payload.size();
    for (uint32_t off = 0; off < total; off += FX_CHUNK_PAYLOAD) {
        uint16_t len = (uint16_t)std::min<uint32_t>(FX_CHUNK_PAYLOAD, total - off);
        chunks.push_back(make_depth_chunk(ts, total, off, payload.data() + off, len));
    }
    return chunks;
}

static bool replay_and_get_depth(const std::vector<std::vector<uint8_t>> &records,
                                 sb_depth_t *out) {
    std::vector<uint8_t> file_buf;
    for (const auto &r : records) write_bin_record(file_buf, r);
    std::string path  = write_temp_file(file_buf);
    sb_receiver_t *rx = sb_receiver_from_file(path.c_str(), false);
    assert(rx && "sb_receiver_from_file failed");
    sb_start(rx);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    sb_stop(rx);
    bool got = sb_get_latest_depth(rx, out);
    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    return got;
}

// Single-chunk 2x2: exact dequant within 1 LSB + the 0-sentinel (invalid) path.
static void test_depth_roundtrip_dequant() {
    const float zmn = 0.05f, zmx = 5.0f;
    std::vector<float> zs = { 0.5f, 1.0f, std::nanf(""), 4.0f };  // 3rd invalid
    auto payload = make_depth_payload(2, 2, zmn, zmx, zs);
    auto chunks  = depth_chunks(1900000000000000020ULL, payload);
    assert(chunks.size() == 1 && "2x2 payload fits one chunk");

    sb_depth_t d{};
    bool got = replay_and_get_depth(chunks, &d);
    assert(got && "expected a depth map");
    assert(d.width == 2 && d.height == 2 && d.depth_count == 4);
    assert(std::abs(d.z_min - zmn) < 1e-6f && std::abs(d.z_max - zmx) < 1e-6f);
    const float lsb = (zmx - zmn) / 65535.0f;
    assert(std::abs(d.depth[0] - 0.5f) < 2 * lsb);
    assert(std::abs(d.depth[1] - 1.0f) < 2 * lsb);
    assert(d.depth[2] == 0.0f && "NaN must decode to the 0 invalid sentinel");
    assert(std::abs(d.depth[3] - 4.0f) < 2 * lsb);
    sb_free_depth(&d);
    std::printf("PASS test_depth_roundtrip_dequant\n");
}

// Native 256x192 spans many chunks: in-order reassembly + sample values intact.
static void test_depth_chunk_reassembly() {
    const uint16_t w = 256, h = 192;
    const float zmn = 0.05f, zmx = 5.0f;
    std::vector<float> zs((size_t)w * h);
    for (size_t i = 0; i < zs.size(); ++i)
        zs[i] = 0.1f + (float)i / (float)zs.size() * 4.8f;  // 0.1..4.9, all valid
    auto payload = make_depth_payload(w, h, zmn, zmx, zs);
    auto chunks  = depth_chunks(1900000000000000021ULL, payload);
    assert(chunks.size() > 3 && "256x192 must span several chunks");

    sb_depth_t d{};
    bool got = replay_and_get_depth(chunks, &d);
    assert(got && "expected a reassembled depth map");
    assert(d.width == w && d.height == h && d.depth_count == (uint32_t)w * h);
    const float lsb = (zmx - zmn) / 65535.0f;
    assert(std::abs(d.depth[0] - zs[0]) < 2 * lsb);
    assert(std::abs(d.depth[zs.size() - 1] - zs[zs.size() - 1]) < 2 * lsb);
    sb_free_depth(&d);
    std::printf("PASS test_depth_chunk_reassembly\n");
}

// Reversed chunks still reassemble (offset-addressed, not order-dependent).
static void test_depth_out_of_order() {
    const uint16_t w = 256, h = 8;
    std::vector<float> zs((size_t)w * h, 1.234f);
    auto payload = make_depth_payload(w, h, 0.05f, 5.0f, zs);
    auto chunks  = depth_chunks(1900000000000000022ULL, payload);
    std::reverse(chunks.begin(), chunks.end());
    sb_depth_t d{};
    bool got = replay_and_get_depth(chunks, &d);
    assert(got && "out-of-order depth chunks must still reassemble");
    assert(d.width == w && d.height == h);
    sb_free_depth(&d);
    std::printf("PASS test_depth_out_of_order\n");
}

// A missing chunk → no depth published (no partial/garbage map).
static void test_depth_incomplete() {
    const uint16_t w = 256, h = 8;
    std::vector<float> zs((size_t)w * h, 1.0f);
    auto payload = make_depth_payload(w, h, 0.05f, 5.0f, zs);
    auto chunks  = depth_chunks(1900000000000000023ULL, payload);
    assert(chunks.size() >= 3);
    chunks.erase(chunks.begin() + 1);  // drop a middle chunk
    sb_depth_t d{};
    bool got = replay_and_get_depth(chunks, &d);
    assert(!got && "incomplete depth map must not be published");
    std::printf("PASS test_depth_incomplete\n");
}

// A corrupt sub-header (encoding != 1) must be rejected, not published.
static void test_depth_bad_subheader() {
    const uint16_t w = 4, h = 2;
    std::vector<float> zs((size_t)w * h, 1.0f);
    auto payload = make_depth_payload(w, h, 0.05f, 5.0f, zs);
    payload[4] = 9;  // corrupt the encoding byte (valid is 1)
    auto chunks = depth_chunks(1900000000000000024ULL, payload);
    sb_depth_t d{};
    bool got = replay_and_get_depth(chunks, &d);
    assert(!got && "depth with bad encoding must be rejected");
    std::printf("PASS test_depth_bad_subheader\n");
}

// ---------------------------------------------------------------------------
// 0x0C heartbeat — the only Mac->iPhone packet.  Bind a socket, send one pose
// from it, and expect the receiver to beacon back to that same source port.
// ---------------------------------------------------------------------------

static void test_heartbeat_returns_to_sender() {
    const uint16_t port = 19898;
    sb_receiver_t *rx_h = sb_receiver_create(port);
    assert(rx_h && "sb_receiver_create failed");
    sb_start(rx_h);

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    assert(fd >= 0 && "test socket");
    // Bind to an ephemeral port so the heartbeat has somewhere to land, and
    // time out the read rather than hanging the suite if it never arrives.
    struct sockaddr_in me = {};
    me.sin_family        = AF_INET;
    me.sin_addr.s_addr   = htonl(INADDR_LOOPBACK);
    me.sin_port          = 0;
    assert(bind(fd, reinterpret_cast<struct sockaddr *>(&me), sizeof(me)) == 0);
    struct timeval tv = {2, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in dst = {};
    dst.sin_family      = AF_INET;
    dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    dst.sin_port        = htons(port);

    std::vector<uint8_t> pose;
    write_u8(pose, 0x01);
    write_u64_le(pose, 1700000000000000000ULL);
    for (int i = 0; i < 3; i++) write_f32_le(pose, 0.0f);   // pos
    for (int i = 0; i < 3; i++) write_f32_le(pose, 0.0f);   // rot xyz
    write_f32_le(pose, 1.0f);                               // rot w
    write_f32_le(pose, 1.0f);                               // quality
    assert(pose.size() == 41);
    ssize_t sent = sendto(fd, pose.data(), pose.size(), 0,
                          reinterpret_cast<struct sockaddr *>(&dst), sizeof(dst));
    assert(sent == static_cast<ssize_t>(pose.size()) && "pose sendto failed");

    uint8_t hb[64] = {};
    ssize_t n      = recv(fd, hb, sizeof(hb), 0);
    assert(n == 13 && "expected a 13-byte 0x0C heartbeat within 2 s");
    assert(hb[0] == 0x0C && "heartbeat type byte");
    uint64_t ts = 0;
    std::memcpy(&ts, hb + 1, 8);
    assert(ts > 0 && "heartbeat timestamp must be set");
    float rate = 0;
    std::memcpy(&rate, hb + 9, 4);
    assert(std::isfinite(rate) && rate >= 0.0f && "heartbeat packet_rate");

    close(fd);
    sb_stop(rx_h);
    sb_receiver_destroy(rx_h);
    std::printf("PASS test_heartbeat_returns_to_sender\n");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

static void test_replay_stop_interrupts_packet_wait() {
    std::vector<uint8_t> recording;
    for (uint64_t ts : {1000000000ULL, 6000000000ULL}) {
        auto packet = make_pose_packet(1);
        for (unsigned i = 0; i < 8; ++i) packet[1 + i] = uint8_t(ts >> (8 * i));
        write_bin_record(recording, packet);
    }
    auto path = write_temp_file(recording);
    auto *rx = sb_receiver_from_file(path.c_str(), false);
    assert(rx);
    sb_start(rx);
    sb_pose_t pose{};
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!sb_get_latest_pose(rx, &pose) && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    assert(pose.timestamp_ns == 1000000000ULL);
    auto start = std::chrono::steady_clock::now();
    sb_stop(rx);
    auto elapsed = std::chrono::steady_clock::now() - start;
    assert(!sb_get_latest_pose(rx, &pose) && "stop must not dispatch the pending packet");
    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    assert(elapsed < std::chrono::milliseconds(500) && "stop must interrupt replay wait");
    std::printf("PASS test_replay_stop_interrupts_packet_wait\n");
}

int main() {
    test_replay_stop_interrupts_packet_wait();
    std::printf("=== bridge-receiver packet tests ===\n");
    test_pose_packet_size();
    test_plane_packet_size();
    test_hand_packet_size();
    test_little_endian_encoding();
    test_pose_roundtrip();
    test_plane_roundtrip();
    test_hand_roundtrip();
    test_known_u64_byte_order();
    test_seqlock_stress();
    test_pose_edge_triggered();
    test_pose_session_epoch();
    test_frame_chunk_inorder();
    test_frame_chunk_out_of_order();
    test_frame_chunk_incomplete();
    test_frame_chunk_duplicate();
    test_frame_chunk_misaligned_offset();
    test_frame_chunk_bad_total();
    test_frame_chunk_range_oob();
    test_frame_chunk_far_future_no_wedge();
    test_frame_chunk_forward_resume();
    test_frame_0x03_roundtrip();
    test_frame_oversize_sof_rejected();
    test_concurrent_receiver_frames();
    test_plane_nan_normal_ignored();
    test_hand_nan_joint_ignored();
    test_pose_nan_ignored();
    test_pose_quat_normalized();
    test_pose_zero_quat_dropped();
    test_intrinsics_packet_size();
    test_intrinsics_roundtrip();
    test_intrinsics_absent();
    test_shm_intrinsics_roundtrip();
    test_shm_reader_stuck_seqlock_gives_up();
    test_depth_subheader_size();
    test_depth_roundtrip_dequant();
    test_depth_chunk_reassembly();
    test_depth_out_of_order();
    test_depth_incomplete();
    test_depth_bad_subheader();
    test_heartbeat_returns_to_sender();
    std::printf("=== ALL TESTS PASSED ===\n");
    return 0;
}
