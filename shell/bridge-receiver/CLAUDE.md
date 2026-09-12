# bridge-receiver — Subsystem Context

## What This Library Does

`bridge-receiver` is a C++17 static library (`spatial_bridge`) that:

1. **Receives** iPhone ARKit UDP packets on port 9898 (or replays a recorded .bin file).
2. **Parses** the live wire-protocol packet types: pose `0x01`, plane `0x02`,
   frame `0x03` (legacy/replay only — the sender now streams `0x09`), mesh
   `0x04`, hand joints `0x05`, chunked camera frame `0x09`, camera intrinsics
   `0x0A`, and depth map `0x0B`. Retired types `0x06`–`0x08` are recognised and
   silently skipped. See `docs/wire-protocol.md` for byte layouts and
   `receiver.cpp` for the dispatch switch.
2b. **Answers** with a 1 Hz `0x0C` heartbeat (13 B: type, `timestamp_ns`,
   `packet_rate`) to the source address of the most recent inbound datagram —
   the only Mac→iPhone packet, and the only way the phone can tell a live link
   from a black hole. Sent `MSG_DONTWAIT` from `run_udp_loop`; send errors are
   logged and dropped so the receive loop never stalls. UDP mode only (file
   replay has no peer).
   verified: 2026-09-04 -- bridge-receiver/src/receiver.cpp
3. **Stores** decoded world state in a thread-safe data structure used by the Monado driver
   and the wxrd compositor.

The library exposes a pure C API (`spatial_bridge.h`) so it can be linked from both C++
(the compositor) and C (the Monado XRT driver).

---

## File Layout

```
bridge-receiver/
├── CLAUDE.md               ← you are here
├── CMakeLists.txt          ← builds spatial_bridge static lib + tests
├── include/
│   └── spatial_bridge.h   ← public C API (extern "C" guarded)
├── src/
│   ├── receiver.cpp        ← UDP socket, file replay, packet dispatch, JPEG decode
│   └── world_model.cpp     ← seqlock pose, mutex planes, mutex frame, packet rate
│   └── world_model.h       ← internal C++ API (not installed)
└── tests/
    ├── CMakeLists.txt
    ├── test_packets.cpp    ← wire-protocol roundtrip + seqlock stress
    ├── test_world_model.cpp← plane add/update/remove correctness
    └── recorded/           ← committed .bin session files (see README.md)
```

---

## Building

Standard CMake out-of-tree build:

```bash
cmake -G Ninja -S bridge-receiver -B build/bridge-receiver
ninja -C build/bridge-receiver
```

Requires on Ubuntu 24.04:
- `libjpeg-turbo8-dev` (or `libturbojpeg0-dev`) — `apt install libjpeg-turbo8-dev`
- C++17 compiler — `apt install build-essential`
- CMake >= 3.16, Ninja — `apt install cmake ninja-build`

All of these are handled by `scripts/setup-ubuntu.sh`.

---

## Running Tests

```bash
cd build/bridge-receiver && ctest --output-on-failure -j$(nproc)
```

Tests require **no hardware**. The seqlock stress test runs for ~3 seconds.

---

## Architecture Notes

### Thread Safety Model

| Data         | Mechanism       | Rationale                                              |
|--------------|-----------------|--------------------------------------------------------|
| Pose (0x01)  | Seqlock         | Written at 60 Hz; read on compositor hot path.         |
|              |                 | Mutex would add latency; seqlock is lock-free on reads.|
| Planes (0x02)| `std::mutex`    | Updated at 1-5 Hz — mutex overhead is negligible.     |
| Frames (0x03/0x09)| `std::mutex` | ~20 Hz; JPEG decode is the bottleneck, not the mutex. |
| Hands (0x05) | `std::mutex`    | Per-frame joint set; reconstructed in-place on update. |
| Intrinsics (0x0A)| `std::mutex`| Latest-wins; consumed by driver FOV + wxrd quad.      |
| Depth (0x0B) | `std::mutex`    | Chunked depth map; edge-triggered read.                |
| Packet rate  | Lock-free ring  | 100 ms buckets bumped on ingress, summed on read.     |
|              |                 | Reads are pure — several consumers poll concurrently.  |

### Seqlock Pattern (exactly as specified in root CLAUDE.md)

```
// Writer (UDP thread):
atomic_fetch_add(&seq, 1, release);   // seq → odd (write in progress)
state.pose = new_pose;
atomic_fetch_add(&seq, 1, release);   // seq → even (write complete)

// Reader (Monado driver / compositor):
do {
    seq1 = atomic_load(&seq, acquire);
    if (seq1 & 1) continue;           // spin if write in progress
    snapshot = state.pose;
    seq2 = atomic_load(&seq, acquire);
} while (seq1 != seq2);               // retry if write happened during read
```

### .bin File Format

Recorded sessions are stored as length-prefixed datagrams:

```
[uint32_t len (LE)][len bytes: raw packet payload]
... repeated for each datagram ...
```

The receiver reconstructs inter-packet timing from the `timestamp_ns` fields
embedded in each packet (relative to the first packet's timestamp).

`sb_receiver_from_file()` with `loop=true` replays indefinitely — useful for
offline development when no iPhone is available.
Replay timestamp waits are interruptible: `sb_stop()` wakes a pending wait
and joins both workers before returning.

### JPEG Decoding

Frames are JPEG-compressed on the iPhone and decoded here using the libjpeg C API.
Output is always RGBA (4 bytes/pixel), allocated with `malloc`. The caller must
release frames with `sb_free_frame()` — NOT `free()` directly — so the allocator
can be changed without breaking the API contract.

If libjpeg-turbo's `JCS_EXT_RGBA` colour space is available (it is on Ubuntu with
libjpeg-turbo), decoding is a single pass. Otherwise we decode to RGB and expand
to RGBA with alpha=0xFF.

JPEG decoder, error recovery and scanline buffers belong to each worker, are
reused across frames, and are released when that worker exits. Concurrent
receiver instances must never share libjpeg state.

verified: 2026-09-06 -- bridge-receiver/src/receiver.cpp bridge-receiver/tests/test_packets.cpp

### Plane UUID Eviction

Planes are stored in a flat array of up to 64 entries. On remove, the target
entry is swapped with the last entry (O(1)). If a remove arrives for an unknown
UUID it is silently ignored.

---

## Gotchas

- **Strict aliasing**: ALL multi-byte reads from packet buffers use `memcpy` into
  local typed variables. Never cast `uint8_t*` to `uint32_t*` and dereference.

- **No exceptions**: library functions return `false`/`nullptr` and log to `stderr`.
  Never use `try`/`catch` in library code.

- **Frame ownership**: `world_model_update_frame()` takes ownership of the `rgba`
  pointer. Do not `free()` it after passing it in.

- **sb_get_latest_pose() is edge-triggered**: returns `false` if called twice
  without a new pose arriving in between. The Monado driver must handle this and
  return the last known pose rather than failing.

- **Packet rate**: `sb_get_packet_rate()` averages over a 1-second sliding window
  (all packet types combined). It resets to 0 when no packets arrive for > 1s.
  Reading it does **not** disturb the window, so any number of threads may poll
  it at any rate. Keep it that way: the sampler used to roll the window on
  every call, and once mac-shell polled it per-frame *and* over the control
  socket, the second caller read ~16 ms of traffic and reported 0.0 pkt/s on a
  live 58 pkt/s link. Covered by `test_packet_rate_repeated_reads`.
  verified: 2026-09-04 -- bridge-receiver/src/world_model.cpp
