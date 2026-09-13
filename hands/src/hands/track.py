"""`hands track` — the live loop.

Read the newest exported frame, find the landmarks, put them in the scene
frame in metres, send them to the shell, repeat. Prints one line a second:

  12.8 fps | detect 0.94 | depth 0.71 | landmark 41.2 ms | e2e 88.4 ms

  detect     fraction of frames MediaPipe found at least one hand in
  depth      fraction of landmarks that got a real LiDAR reading rather than
             an estimated range
  landmark   how long the model itself took, per frame
  e2e        the phone's capture timestamp to the moment the shell acked the
             injection — the number that decides whether a pinch feels live
"""

from __future__ import annotations

import signal
import time

from .control import ShellControl, ShellError
from .export_reader import ExportReader
from .tracker import HandTracker

# Past this, the frame's timestamp is on a different clock (a replayed
# fixture's invented epoch), not a pipeline that is genuinely this far behind.
MAX_PLAUSIBLE_LAG_S = 5.0


class _Stats:
    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.frames = 0
        self.with_hand = 0
        self.depth_valid = 0.0
        self.landmark_s = 0.0
        self.e2e_s = 0.0
        self.e2e_n = 0
        self.since = time.monotonic()

    def line(self) -> str:
        n = max(1, self.frames)
        dt = max(1e-6, time.monotonic() - self.since)
        return (
            f"{self.frames / dt:5.1f} fps | "
            f"detect {self.with_hand / n:.2f} | "
            f"depth {self.depth_valid / n:.2f} | "
            f"landmark {1000 * self.landmark_s / n:5.1f} ms | "
            + (
                f"e2e {1000 * self.e2e_s / self.e2e_n:6.1f} ms"
                if self.e2e_n
                else "e2e     n/a"
            )
        )


def run_track(args) -> int:
    stop = False

    def on_signal(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        shell = ShellControl(args.sock)
    except ShellError as exc:
        print(f"hands: {exc}", flush=True)
        return 1

    started_export = False
    try:
        if args.enable_export:
            print(shell.frame_export_on(args.dir), flush=True)
            started_export = True

        reader = ExportReader(args.dir)
        stats = _Stats()
        deadline = time.monotonic() + args.seconds if args.seconds > 0 else None

        with HandTracker(
            model_path=args.model,
            flip_handedness=args.flip_handedness,
            depth_window=args.depth_window,
            min_cutoff=args.min_cutoff,
            beta=args.beta,
            fallback_depth_m=args.fallback_depth_m,
        ) as tracker:
            print(f"tracking {args.dir} -> {shell.path}", flush=True)
            while not stop and (deadline is None or time.monotonic() < deadline):
                frame = reader.wait(timeout_s=1.0)
                if frame is None:
                    print("waiting for frames (is frame-export on?)", flush=True)
                    continue

                t_landmark = time.monotonic()
                result = tracker.track(frame)
                landmark_s = time.monotonic() - t_landmark

                payload = [
                    {
                        "chirality": h.chirality,
                        "confidence": h.confidence,
                        "joints": h.joints,
                    }
                    for h in result.hands
                ]
                now_ms = int(time.time() * 1000)
                try:
                    shell.inject_hands(now_ms, payload)
                except ShellError as exc:
                    print(f"hands: inject refused: {exc}", flush=True)
                    break

                stats.frames += 1
                stats.with_hand += 1 if result.hands else 0
                stats.depth_valid += result.depth_valid_fraction
                stats.landmark_s += landmark_s
                # The frame's own timestamp is the phone's wall clock, the
                # same clock time.time() reads here, so the difference is the
                # real capture-to-injection latency and not a clock offset.
                # A replayed fixture carries a made-up epoch, which would
                # otherwise be reported as a 25-year latency; anything past a
                # few seconds is a different clock, not a slow pipeline.
                lag = time.time() - frame.t_ns / 1e9
                if 0.0 <= lag <= MAX_PLAUSIBLE_LAG_S:
                    stats.e2e_s += lag
                    stats.e2e_n += 1

                if time.monotonic() - stats.since >= 1.0:
                    print(stats.line(), flush=True)
                    stats.reset()
    finally:
        if started_export:
            shell.frame_export_off()
        shell.close()
    return 0
