// port_retry.h — re-attempt the UDP bind while the port is busy.
//
// EADDRINUSE used to be permanent: the shell logged it once, showed a "Port
// didn't open" card, and stayed deaf even after the other process quit —
// only a relaunch fixed it. This is polled from whichever loop is already
// running (the render tick with a window, the headless tick loop without),
// so no extra thread or timer source is involved.

#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>

#include "spatial_bridge.h"

#include "core/scene.h"

namespace mac_shell {

class port_retry {
   public:
    static constexpr double INTERVAL_S = 2.0;

    port_retry(scene &world, std::atomic<sb_receiver_t *> &receiver, int port)
        : world_(world), receiver_(receiver), port_(port) {}

    port_retry(const port_retry &) = delete;
    port_retry &operator=(const port_retry &) = delete;

    void arm(double now_s) {
        pending_ = true;
        last_try_ = now_s;
    }
    bool pending() const { return pending_; }
    sb_receiver_t *receiver() const { return receiver_.load(); }

    // True on the one tick the port finally opened.
    bool poll(double now_s) {
        if (!pending_ || now_s - last_try_ < INTERVAL_S)
            return false;
        last_try_ = now_s;
        sb_receiver_t *r = sb_receiver_create((uint16_t)port_);
        if (!r)
            return false;
        sb_start(r);
        receiver_.store(r);
        world_.set_receiver(r);
        pending_ = false;
        std::fprintf(stderr, "mac-shell: listening on UDP %d (port freed)\n",
                     port_);
        return true;
    }

   private:
    scene &world_;
    std::atomic<sb_receiver_t *> &receiver_;
    int port_;
    bool pending_ = false;
    double last_try_ = 0;
};

}  // namespace mac_shell
