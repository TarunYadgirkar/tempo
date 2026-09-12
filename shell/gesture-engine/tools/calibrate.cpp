// calibrate.cpp — Per-user gesture calibration CLI.
//
// Replays a recorded session through the calibration library and emits a
// tuned gestures.toml to stdout.  Phase boundaries default to 10s/10s/10s
// of the recording (idle / pinch / grab); override via --phases <a,b,c>.
//
// Usage:
//   calibrate <recording.bin> [--phases <idle_s,pinch_s,grab_s>]
//
// The recording format is the same .bin that wxrd's SPATIAL_RECORD_HANDS
// produces (header SPHD + per-frame ts + dt + 2 hands).

#include "gesture_engine.h"
#include "gesture_calibrate.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

struct frame_t {
    uint64_t ts_ns;
    float    dt;
    ge_hand_t hands[2];
};

static int load_recording(const char *path, std::vector<frame_t> &out) {
    FILE *f = std::fopen(path, "rb");
    if (!f) {
        std::fprintf(stderr, "cannot open %s\n", path);
        return -1;
    }
    uint8_t hdr[8];
    if (std::fread(hdr, 1, 8, f) != 8 || std::memcmp(hdr, "SPHD", 4) != 0) {
        std::fprintf(stderr, "%s: invalid header (expected SPHD magic)\n", path);
        std::fclose(f);
        return -1;
    }
    uint8_t joint_count = hdr[6];
    if (joint_count != GE_JOINT_COUNT) {
        std::fprintf(stderr, "%s: joint count %d != GE_JOINT_COUNT %d\n",
                     path, joint_count, GE_JOINT_COUNT);
        std::fclose(f);
        return -1;
    }
    while (true) {
        frame_t fr{};
        if (std::fread(&fr.ts_ns, 8, 1, f) != 1) break;
        if (std::fread(&fr.dt, 4, 1, f) != 1) break;
        bool ok = true;
        for (int h = 0; h < 2 && ok; ++h) {
            if (std::fread(fr.hands[h].joints, sizeof(fr.hands[h].joints), 1, f) != 1) {
                ok = false;
                break;
            }
            uint32_t present;
            if (std::fread(&present, 4, 1, f) != 1) {
                ok = false;
                break;
            }
            fr.hands[h].present = (present != 0);
        }
        if (!ok) break;
        out.push_back(fr);
    }
    std::fclose(f);
    return 0;
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: %s <recording.bin> [--phases idle_s,pinch_s,grab_s]\n",
                     argv[0]);
        return 1;
    }
    const char *path = argv[1];
    double idle_s = 10.0, pinch_s = 10.0, grab_s = 10.0;
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--phases") == 0 && i + 1 < argc) {
            if (std::sscanf(argv[i + 1], "%lf,%lf,%lf",
                            &idle_s, &pinch_s, &grab_s) != 3) {
                std::fprintf(stderr, "--phases: expected idle_s,pinch_s,grab_s\n");
                return 1;
            }
            ++i;
        }
    }

    std::vector<frame_t> frames;
    if (load_recording(path, frames) != 0) return 2;
    if (frames.empty()) {
        std::fprintf(stderr, "%s: no frames\n", path);
        return 2;
    }

    ge_calib_t *gc = gc_create();
    if (!gc) return 3;

    uint64_t t0 = frames.front().ts_ns;
    double idle_end_s  = idle_s;
    double pinch_end_s = idle_s + pinch_s;
    double grab_end_s  = idle_s + pinch_s + grab_s;

    for (const auto &fr : frames) {
        double t_s = (fr.ts_ns - t0) / 1e9;
        ge_calib_phase_t phase;
        if (t_s < idle_end_s)       phase = GE_CALIB_PHASE_IDLE;
        else if (t_s < pinch_end_s) phase = GE_CALIB_PHASE_PINCH;
        else if (t_s < grab_end_s)  phase = GE_CALIB_PHASE_GRAB;
        else                        break;
        gc_set_phase(gc, phase);
        for (int h = 0; h < 2; ++h) {
            if (fr.hands[h].present) gc_observe(gc, &fr.hands[h]);
        }
    }

    gc_emit_toml(gc, stdout);
    gc_destroy(gc);
    return 0;
}
