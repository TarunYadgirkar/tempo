// test_world_model.cpp — Tests for plane add / update / remove world model behaviour.
//
// Constructs .bin files with sequences of plane packets and verifies that
// sb_get_planes() returns the correct set at each step.

#include "spatial_bridge.h"

#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

// ---------------------------------------------------------------------------
// Helpers (duplicated from test_packets.cpp to keep files independent)
// ---------------------------------------------------------------------------

static void write_u8(std::vector<uint8_t> &b, uint8_t v) { b.push_back(v); }

static void write_u32_le(std::vector<uint8_t> &b, uint32_t v) {
    b.push_back(static_cast<uint8_t>(v >> 0));
    b.push_back(static_cast<uint8_t>(v >> 8));
    b.push_back(static_cast<uint8_t>(v >> 16));
    b.push_back(static_cast<uint8_t>(v >> 24));
}

static void write_u64_le(std::vector<uint8_t> &b, uint64_t v) {
    for (int i = 0; i < 8; ++i) b.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

static void write_f32_le(std::vector<uint8_t> &b, float f) {
    uint32_t bits;
    std::memcpy(&bits, &f, sizeof(bits));
    write_u32_le(b, bits);
}

static void bin_record(std::vector<uint8_t> &file_buf, const std::vector<uint8_t> &pkt) {
    write_u32_le(file_buf, static_cast<uint32_t>(pkt.size()));
    file_buf.insert(file_buf.end(), pkt.begin(), pkt.end());
}

static std::string write_temp(const std::vector<uint8_t> &data) {
    char tmpl[] = "/tmp/sb_wm_test_XXXXXX";
    int fd      = mkstemp(tmpl);
    assert(fd >= 0);
    std::string path(tmpl);
    FILE *fp = fdopen(fd, "wb");
    std::fwrite(data.data(), 1, data.size(), fp);
    std::fclose(fp);
    return path;
}

// Build a 0x02 plane packet.
static std::vector<uint8_t> make_plane_pkt(uint64_t ts, const uint8_t uuid[16], float cx, float cy,
                                           float cz, float nx, float ny, float nz, float ew,
                                           float eh, uint8_t alignment, uint8_t is_removed) {
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
    return pkt;
}

// Run a .bin file through the receiver and return it after replay completes.
static sb_receiver_t *replay_and_stop(const std::string &path) {
    sb_receiver_t *rx = sb_receiver_from_file(path.c_str(), false);
    assert(rx);
    sb_start(rx);
    // Give the replay thread time to finish (all records are nearly instant).
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    sb_stop(rx);
    return rx;
}

// ---------------------------------------------------------------------------
// UUID helpers
// ---------------------------------------------------------------------------

static void uuid_fill(uint8_t uuid[16], uint8_t seed) {
    for (int i = 0; i < 16; ++i) uuid[i] = static_cast<uint8_t>(seed + i);
}

static bool uuid_match(const uint8_t a[16], const uint8_t b[16]) {
    return std::memcmp(a, b, 16) == 0;
}

// ---------------------------------------------------------------------------
// Test 1: Adding multiple planes
// ---------------------------------------------------------------------------

static void test_add_planes() {
    uint8_t uuid_a[16], uuid_b[16];
    uuid_fill(uuid_a, 0x10);
    uuid_fill(uuid_b, 0x20);

    std::vector<uint8_t> file_buf;
    bin_record(file_buf, make_plane_pkt(1000, uuid_a, 0, 0, 0, 0, 1, 0, 2, 1, 0, 0));
    bin_record(file_buf, make_plane_pkt(2000, uuid_b, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0));
    std::string path = write_temp(file_buf);

    sb_receiver_t *rx = replay_and_stop(path);

    sb_plane_t planes[8];
    int n = sb_get_planes(rx, planes, 8);
    assert(n == 2 && "expected 2 planes after add");

    // Verify both UUIDs are present.
    bool found_a = false, found_b = false;
    for (int i = 0; i < n; ++i) {
        if (uuid_match(planes[i].uuid, uuid_a)) found_a = true;
        if (uuid_match(planes[i].uuid, uuid_b)) found_b = true;
    }
    assert(found_a && "uuid_a not found");
    assert(found_b && "uuid_b not found");

    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    std::printf("PASS test_add_planes\n");
}

// ---------------------------------------------------------------------------
// Test 2: Updating an existing plane
// ---------------------------------------------------------------------------

static void test_update_plane() {
    uint8_t uuid[16];
    uuid_fill(uuid, 0x30);

    std::vector<uint8_t> file_buf;
    // Add with center (0,0,0), extent (1,1).
    bin_record(file_buf, make_plane_pkt(1000, uuid, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0));
    // Update same UUID with center (5,5,5), extent (3,2).
    bin_record(file_buf, make_plane_pkt(2000, uuid, 5, 5, 5, 0, 1, 0, 3, 2, 0, 0));
    std::string path = write_temp(file_buf);

    sb_receiver_t *rx = replay_and_stop(path);

    sb_plane_t planes[8];
    int n = sb_get_planes(rx, planes, 8);
    assert(n == 1 && "expected 1 plane after update (no duplicates)");
    assert(uuid_match(planes[0].uuid, uuid));
    assert(std::abs(planes[0].center[0] - 5.0f) < 1e-6f && "center not updated");
    assert(std::abs(planes[0].extent[0] - 3.0f) < 1e-6f && "extent not updated");

    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    std::printf("PASS test_update_plane\n");
}

// ---------------------------------------------------------------------------
// Test 3: Removing a plane
// ---------------------------------------------------------------------------

static void test_remove_plane() {
    uint8_t uuid_keep[16], uuid_remove[16];
    uuid_fill(uuid_keep, 0x40);
    uuid_fill(uuid_remove, 0x50);

    std::vector<uint8_t> file_buf;
    bin_record(file_buf, make_plane_pkt(1000, uuid_keep, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0));
    bin_record(file_buf, make_plane_pkt(2000, uuid_remove, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0));
    // Remove uuid_remove.
    bin_record(file_buf, make_plane_pkt(3000, uuid_remove, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1));
    std::string path = write_temp(file_buf);

    sb_receiver_t *rx = replay_and_stop(path);

    sb_plane_t planes[8];
    int n = sb_get_planes(rx, planes, 8);
    assert(n == 1 && "expected 1 plane after remove");
    assert(uuid_match(planes[0].uuid, uuid_keep) && "wrong plane kept");

    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    std::printf("PASS test_remove_plane\n");
}

// ---------------------------------------------------------------------------
// Test 4: Remove of unknown UUID is a no-op
// ---------------------------------------------------------------------------

static void test_remove_unknown_plane() {
    uint8_t uuid_known[16], uuid_ghost[16];
    uuid_fill(uuid_known, 0x60);
    uuid_fill(uuid_ghost, 0x70);

    std::vector<uint8_t> file_buf;
    bin_record(file_buf, make_plane_pkt(1000, uuid_known, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0));
    // Remove an UUID we never added.
    bin_record(file_buf, make_plane_pkt(2000, uuid_ghost, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1));
    std::string path = write_temp(file_buf);

    sb_receiver_t *rx = replay_and_stop(path);

    sb_plane_t planes[8];
    int n = sb_get_planes(rx, planes, 8);
    assert(n == 1 && "known plane should still be present");

    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    std::printf("PASS test_remove_unknown_plane\n");
}

// ---------------------------------------------------------------------------
// Test 5: Add / remove / re-add the same UUID
// ---------------------------------------------------------------------------

static void test_readd_plane() {
    uint8_t uuid[16];
    uuid_fill(uuid, 0x80);

    std::vector<uint8_t> file_buf;
    bin_record(file_buf, make_plane_pkt(1000, uuid, 1, 0, 0, 0, 1, 0, 2, 2, 0, 0));
    bin_record(file_buf, make_plane_pkt(2000, uuid, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1));  // remove
    bin_record(file_buf, make_plane_pkt(3000, uuid, 9, 9, 9, 0, 1, 0, 5, 5, 0, 0));  // re-add
    std::string path = write_temp(file_buf);

    sb_receiver_t *rx = replay_and_stop(path);

    sb_plane_t planes[8];
    int n = sb_get_planes(rx, planes, 8);
    assert(n == 1);
    assert(std::abs(planes[0].center[0] - 9.0f) < 1e-6f && "re-added center wrong");
    assert(std::abs(planes[0].extent[0] - 5.0f) < 1e-6f && "re-added extent wrong");

    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    std::printf("PASS test_readd_plane\n");
}

// ---------------------------------------------------------------------------
// Test 6: sb_get_planes respects max_planes limit
// ---------------------------------------------------------------------------

static void test_get_planes_limit() {
    std::vector<uint8_t> file_buf;
    uint8_t uuid[16];
    for (int i = 0; i < 5; ++i) {
        uuid_fill(uuid, static_cast<uint8_t>(0x90 + i));
        bin_record(file_buf, make_plane_pkt(static_cast<uint64_t>(1000 + i * 100), uuid, 0, 0, 0, 0,
                                            1, 0, 1, 1, 0, 0));
    }
    std::string path = write_temp(file_buf);

    sb_receiver_t *rx = replay_and_stop(path);

    sb_plane_t planes[8];
    int n_full = sb_get_planes(rx, planes, 8);
    int n_lim  = sb_get_planes(rx, planes, 2);
    assert(n_full == 5 && "expected 5 planes");
    assert(n_lim == 2 && "expected limit of 2 to be respected");

    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    std::printf("PASS test_get_planes_limit\n");
}

// ---------------------------------------------------------------------------
// Test 7: packet rate survives repeated readers
//
// Regression: the rate used to be sampled destructively — every call rolled
// the window forward and returned only what had arrived since the previous
// call. mac-shell polls this once per rendered frame while the control
// socket's `stats` verb reads it from another thread, so the second caller
// saw a ~16 ms window and reported 0.0 pkt/s on a live link (observed
// 2026-08-29 against a phone streaming ~58 pkt/s). Reads must be pure.
// ---------------------------------------------------------------------------

static void test_packet_rate_repeated_reads() {
    constexpr int kPackets = 40;
    std::vector<uint8_t> file_buf;
    uint8_t uuid[16];
    for (int i = 0; i < kPackets; ++i) {
        uuid_fill(uuid, static_cast<uint8_t>(i));
        bin_record(file_buf, make_plane_pkt(static_cast<uint64_t>(1000 + i), uuid, 0, 0, 0, 0, 1, 0,
                                            1, 1, 0, 0));
    }
    std::string path = write_temp(file_buf);

    sb_receiver_t *rx = replay_and_stop(path);

    // Hammer it the way two consumer threads would.
    float first = sb_get_packet_rate(rx);
    assert(first > 0.0f && "expected a non-zero rate right after replay");
    for (int i = 0; i < 10; ++i) {
        float again = sb_get_packet_rate(rx);
        assert(again > first * 0.5f &&
               "repeated reads must not drain the window (see regression note)");
    }

    // ...and it must still fall back to silence once the window empties.
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    assert(sb_get_packet_rate(rx) == 0.0f && "rate should decay to 0 after >1s of silence");

    sb_receiver_destroy(rx);
    std::remove(path.c_str());
    std::printf("PASS test_packet_rate_repeated_reads\n");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
    std::printf("=== bridge-receiver world model tests ===\n");
    test_add_planes();
    test_update_plane();
    test_remove_plane();
    test_remove_unknown_plane();
    test_readd_plane();
    test_get_planes_limit();
    test_packet_rate_repeated_reads();
    std::printf("=== ALL TESTS PASSED ===\n");
    return 0;
}
