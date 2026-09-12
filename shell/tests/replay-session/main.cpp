// main.cpp — Replay a recorded .bin UDP session file at original timing.
//
// Usage: replay-session <path-to-session.bin>
//
// Opens the .bin file with sb_receiver_from_file (loop=false), starts the
// background replay thread, and prints packet statistics every second until
// the replay is complete.

#include "spatial_bridge.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

// ---------------------------------------------------------------------------
// Forward-declarations for internal "done" detection.
// We can't directly query "is the thread done?" through the public API, so we
// poll sb_get_packet_rate(). When the rate has been zero for two consecutive
// 1-second intervals and we've seen at least one packet, we assume replay ended.
// A more robust approach would expose a "done" flag, but the public API doesn't
// have one — this heuristic is good enough for a CLI tool.
// ---------------------------------------------------------------------------

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <session.bin>\n", argv[0]);
        return 1;
    }

    const char *path = argv[1];
    std::printf("spatial_bridge replay: opening %s\n", path);

    sb_receiver_t *rx = sb_receiver_from_file(path, /*loop=*/false);
    if (!rx) {
        std::fprintf(stderr, "replay-session: failed to open %s\n", path);
        return 1;
    }

    sb_start(rx);

    // Print packet rate every second; detect end-of-replay by watching for
    // two consecutive zero-rate readings after we've seen at least one packet.
    bool seen_any_packets = false;
    int  zero_rate_streak = 0;

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        float rate = sb_get_packet_rate(rx);
        std::printf("  packet rate: %.1f pkt/s\n", static_cast<double>(rate));

        if (rate > 0.5f) {
            seen_any_packets = true;
            zero_rate_streak = 0;
        } else if (seen_any_packets) {
            ++zero_rate_streak;
            if (zero_rate_streak >= 2) {
                std::printf("  replay appears complete (rate dropped to zero)\n");
                break;
            }
        }

        // Safety: also check for current world state.
        sb_pose_t pose{};
        if (sb_get_latest_pose(rx, &pose)) {
            std::printf("  latest pose: pos=(%.3f, %.3f, %.3f) quality=%.2f\n",
                        static_cast<double>(pose.pos[0]),
                        static_cast<double>(pose.pos[1]),
                        static_cast<double>(pose.pos[2]),
                        static_cast<double>(pose.tracking_quality));
        }

        sb_plane_t planes[64];
        int n_planes = sb_get_planes(rx, planes, 64);
        if (n_planes > 0) {
            std::printf("  planes tracked: %d\n", n_planes);
        }

        // Release any consumed frame immediately.
        sb_frame_t frame{};
        if (sb_get_latest_frame(rx, &frame)) {
            std::printf("  camera frame: %ux%u RGBA (%u bytes)\n",
                        frame.width, frame.height, frame.rgba_size);
            sb_free_frame(&frame);
        }
    }

    sb_stop(rx);
    sb_receiver_destroy(rx);

    std::printf("replay-session: done\n");
    return 0;
}
