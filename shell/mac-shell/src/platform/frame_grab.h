// frame_grab.h — hand-off between the control thread (which wants a PNG of
// the composited scene) and the render thread (which owns the drawable).
//
// The `screenshot` verb has to answer with a path that already exists, so the
// control thread arms a request here and blocks until the render thread's
// command-buffer completion handler has written the file. Requests are
// re-armable: every frame drains whatever is pending, so the verb works as
// often as it is called rather than only at startup like the
// SPATULA_MAC_SCREENSHOT env grab (which now rides the same queue).
//
// Pure C++ (no Metal/AppKit) so main.cpp and control_server.cpp can hold one
// in builds without the renderer.

#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace mac_shell {

// A frame that never arrives must not wedge the control connection; the
// renderer runs at 60 fps, so a second is ~60 missed chances.
constexpr int SHOT_TIMEOUT_MS = 1000;
// grab() runs on the single control thread, so a renderer that has stopped
// presenting (window minimised / occluded — MTKView pauses) must fail fast
// instead of stalling every other client for the full timeout.
constexpr int RENDERER_IDLE_MS = 250;

class frame_grabber {
   public:
    struct request {
        std::string path;
        uint64_t id = 0;
    };

    // Control thread: arm a grab and block until it is written. False on
    // timeout or write failure, with `err` set to a one-word reason.
    bool grab(const std::string &path, std::chrono::milliseconds wait,
              std::string &err) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (ever_took_ && std::chrono::steady_clock::now() - last_take_ >
                              std::chrono::milliseconds(RENDERER_IDLE_MS)) {
            err = "renderer_idle";
            return false;
        }
        uint64_t id = ++next_id_;
        entries_[id] = entry{path, false, true, false, false, {}};
        bool done = cv_.wait_for(lock, wait, [&] {
            auto it = entries_.find(id);
            return it != entries_.end() && it->second.done;
        });
        auto it = entries_.find(id);
        if (!done) {
            // Taken but not finished: leave it for complete() to reap so the
            // render thread never signals a destroyed entry.
            if (it->second.taken)
                it->second.awaited = false;
            else
                entries_.erase(it);
            err = "timeout";
            return false;
        }
        bool ok = it->second.ok;
        err = it->second.err;
        entries_.erase(it);
        return ok;
    }

    // Env-var grab: write it, nobody is waiting for the answer.
    void submit_detached(const std::string &path) {
        std::lock_guard<std::mutex> lock(mutex_);
        entries_[++next_id_] = entry{path, false, false, false, false, {}};
    }

    // Render thread: claim every request armed since the last call.
    std::vector<request> take_pending() {
        std::lock_guard<std::mutex> lock(mutex_);
        last_take_ = std::chrono::steady_clock::now();
        ever_took_ = true;
        std::vector<request> out;
        for (auto &kv : entries_) {
            if (kv.second.taken)
                continue;
            kv.second.taken = true;
            out.push_back({kv.second.path, kv.first});
        }
        return out;
    }

    // Render thread: report one taken request's outcome.
    void complete(uint64_t id, bool ok, const std::string &err) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = entries_.find(id);
        if (it == entries_.end())
            return;
        if (!it->second.awaited) {
            entries_.erase(it);
            return;
        }
        it->second.done = true;
        it->second.ok = ok;
        it->second.err = err;
        cv_.notify_all();
    }

   private:
    struct entry {
        std::string path;
        bool taken;
        bool awaited;
        bool done;
        bool ok;
        std::string err;
    };

    std::mutex mutex_;
    std::condition_variable cv_;
    std::map<uint64_t, entry> entries_;
    uint64_t next_id_ = 0;
    std::chrono::steady_clock::time_point last_take_{};
    bool ever_took_ = false;
};

}  // namespace mac_shell
