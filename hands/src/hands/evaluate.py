"""`hands eval` — is the Mac's tracking actually better than the phone's?

The numbers that decide whether a pinch works:

  jitter_mm       mean frame-to-frame displacement of a joint WHILE THE HAND
                  IS HELD STILL. Motion the user did not make. This is what
                  makes a pinch threshold chatter and a pointer crawl.
  detect_rate     fraction of frames the source had a hand at all. A tracker
                  that drops the hand for 200 ms mid-gesture cancels it.
  pinch_mm        the thumb-tip to index-tip distance while still. A pinch
                  fires under 25 mm, so a source whose resting distance
                  WANDERS across 25 mm fires pinches nobody made.
  depth_valid     fraction of landmarks with a real LiDAR reading. Mac-side
                  only, and reporting-only under `--depth rigid`, where
                  nothing is placed with it.

Two problems with the first version of this, both of which made the earlier
run unreadable:

**The phone column was the Mac.** `hands dump` returns whatever the gesture
engine is seeing, so with injection on it returns the Mac's joints and the
comparison was the Mac against itself. Turning injection off fixed the phone
column and removed the Mac's live path from the run entirely. So the run now
ALTERNATES: a window with injection off, where the dump is genuinely the
phone, then a window with injection on, where the Mac's joints are the ones
the shell is acting on. Each column is recorded only in the windows where it
is the real thing, and the injection is explicitly retracted (an empty
`hands` array) on the way into an off window so the phone comes back at once
rather than after the shell's 150 ms timeout.

**"Still" was measured on the source being scored.** A stretch counted as
still when that source's own wrist stayed inside a 2 cm box — so a source
noisy enough to jump 2 cm was excused from being measured exactly where it
was worst, and the two columns were scored over different moments. Stillness
is now a property of the USER, measured once, on the phone track, as a wrist
speed under 15 mm/s over half a second; both columns are then scored over the
same wall-clock intervals. The phone track exists only in the off windows, so
a still interval is bridged across the on window between two still off
windows — the user holding a hand up is still for seconds at a time, and the
bridge is what lets the mac column be scored at all.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .control import ShellControl, ShellError
from .export_reader import ExportReader
from .geometry import FINGERTIPS, INDEX_TIP, THUMB_TIP, WRIST
from .tracker import tracker_from_args

RESULTS_DIR = Path(__file__).resolve().parents[2] / "results"

# One alternating window. Long enough that the settle below is a small tax on
# it, short enough that the user's hand is in about the same state in both
# halves of any given second of the run.
DEFAULT_WINDOW_S = 5.0
# Ignored after flipping the injection. The shell retires a stale injection
# after 150 ms, so the dump is somebody else's hands until then.
SETTLE_S = 0.3

# The user is still when their wrist moves slower than this, measured over
# STILL_WINDOW_S of the reference track. 15 mm/s is about what a person
# holding a hand up manages; a deliberate reach is ten times it.
STILL_SPEED_MM_S = 15.0
STILL_WINDOW_S = 0.5
# A still interval is carried across a gap this long, which is how the
# reference track (phone, off windows only) covers the on windows between.
BRIDGE_SLACK_S = 1.0
# Two samples further apart than this are not consecutive frames — they
# straddle a window boundary or a dropout, and the distance between them is
# not jitter.
MAX_PAIR_DT_S = 0.35
# A pinch is registered under this in the gesture engine.
PINCH_MM = 25.0


@dataclass
class Sample:
    t_s: float
    joints: np.ndarray | None  # (21, 3) scene metres, MediaPipe order
    depth_valid: np.ndarray | None = None  # (21,) bool, mac only
    held: bool = False  # re-sent from an earlier frame, so not new evidence


@dataclass
class SourceLog:
    name: str
    samples: list[Sample] = field(default_factory=list)

    def add(self, t_s, joints, depth_valid=None, held=False) -> None:
        self.samples.append(Sample(t_s, joints, depth_valid, held))

    def tracked(self) -> list[Sample]:
        return [s for s in self.samples if s.joints is not None]


# ---------------------------------------------------------------------------
# stillness, measured once, on the reference track
# ---------------------------------------------------------------------------


def still_intervals(
    samples: list[Sample],
    speed_mm_s: float = STILL_SPEED_MM_S,
    window_s: float = STILL_WINDOW_S,
    bridge_s: float = 0.0,
) -> list[tuple[float, float]]:
    """Wall-clock stretches where the reference wrist was barely moving.

    Speed rather than a bounding box: a box says nothing about how long the
    hand took to cross it, so a slow drift and a fast twitch score the same.
    Each sample is compared against the oldest tracked sample still inside
    `window_s`, which keeps the baseline long enough that a single noisy frame
    cannot make a still hand look like a moving one.

    `bridge_s` joins two still stretches separated by no more than that, which
    is how a reference that only exists every other window can vouch for the
    windows in between.
    """
    tracked = [s for s in samples if s.joints is not None]
    marks: list[tuple[float, bool]] = []
    for i, s in enumerate(tracked):
        base = None
        for k in range(i - 1, -1, -1):
            if s.t_s - tracked[k].t_s > window_s:
                break
            base = tracked[k]
        if base is None or s.t_s - base.t_s < window_s / 2.0:
            continue
        speed = (
            float(np.linalg.norm(s.joints[WRIST] - base.joints[WRIST]))
            * 1000.0
            / (s.t_s - base.t_s)
        )
        marks.append((s.t_s, speed < speed_mm_s))

    runs: list[tuple[float, float]] = []
    start = None
    last = None
    for t, still in marks:
        # A gap longer than the stillness window is time nobody observed —
        # the reference track is absent, which is exactly what happens across
        # an injection-on window. Close the run there and let `bridge_s`
        # decide whether it may be carried across, rather than letting an
        # unobserved stretch inherit stillness silently.
        broke = last is not None and t - last > window_s
        if still and not broke:
            if start is None:
                start = t
            last = t
            continue
        if start is not None:
            runs.append((start, last))
        start = t if still else None
        last = t
    if start is not None:
        runs.append((start, last))

    return _bridge(runs, bridge_s)


def _bridge(runs: list[tuple[float, float]], gap_s: float) -> list[tuple[float, float]]:
    merged: list[tuple[float, float]] = []
    for run in runs:
        if merged and run[0] - merged[-1][1] <= gap_s:
            merged[-1] = (merged[-1][0], run[1])
        else:
            merged.append(run)
    return merged


def _inside(t_s: float, intervals: list[tuple[float, float]]) -> bool:
    return any(a <= t_s <= b for a, b in intervals)


# ---------------------------------------------------------------------------
# analysis
# ---------------------------------------------------------------------------


def _scored(samples: list[Sample], intervals) -> list[Sample]:
    """Samples that are real evidence inside a still stretch.

    Held samples are excluded rather than counted: a held frame repeats the
    previous frame's joints exactly, so counting it would score a zero
    displacement the tracker did not earn.
    """
    return [
        s
        for s in samples
        if s.joints is not None and not s.held and _inside(s.t_s, intervals)
    ]


def _jitter_mm(samples: list[Sample], intervals) -> tuple[list, int]:
    """Per-joint mean frame-to-frame displacement over the still stretches."""
    scored = _scored(samples, intervals)
    totals = np.zeros(21)
    counts = np.zeros(21)
    used = 0
    for prev, cur in zip(scored, scored[1:]):
        if cur.t_s - prev.t_s > MAX_PAIR_DT_S:
            continue
        totals += np.linalg.norm(cur.joints - prev.joints, axis=1) * 1000.0
        counts += 1
        used += 1
    with np.errstate(invalid="ignore", divide="ignore"):
        per_joint = np.where(counts > 0, totals / np.maximum(counts, 1), np.nan)
    return [None if np.isnan(v) else round(float(v), 3) for v in per_joint], used


def _pinch(samples: list[Sample], intervals) -> dict:
    """Thumb-tip to index-tip while still, in millimetres.

    The user is not asked to pinch on cue, so this is the DISTRIBUTION rather
    than a hit rate against known pinch windows: p50 is where the hand rests,
    p10 is how close the source claims the fingers came, and `frac_under_25mm`
    is how often a still hand would have fired a pinch nobody made. A source
    whose p10 sits under 25 mm while the user is holding a hand up is a source
    that pinches by itself.
    """
    d = [
        float(np.linalg.norm(s.joints[THUMB_TIP] - s.joints[INDEX_TIP])) * 1000.0
        for s in _scored(samples, intervals)
    ]
    if not d:
        return {"samples": 0}
    arr = np.array(d)
    p10, p50, p90 = (round(float(v), 1) for v in np.percentile(arr, [10, 50, 90]))
    return {
        "samples": int(arr.size),
        "p10_mm": p10,
        "p50_mm": p50,
        "p90_mm": p90,
        "frac_under_25mm": round(float((arr < PINCH_MM).mean()), 4),
    }


def analyse(log: SourceLog, intervals: list[tuple[float, float]]) -> dict:
    n = len(log.samples)
    tracked = log.tracked()
    live = [s for s in tracked if not s.held]
    per_joint, still_pairs = _jitter_mm(log.samples, intervals)

    finite = [v for v in per_joint if v is not None]
    fingertip = [per_joint[i] for i in FINGERTIPS if per_joint[i] is not None]

    out = {
        "source": log.name,
        "frames": n,
        "frames_with_hand": len(live),
        "frames_held": len(tracked) - len(live),
        "detect_rate": round(len(live) / n, 4) if n else 0.0,
        "delivered_rate": round(len(tracked) / n, 4) if n else 0.0,
        "still_frame_pairs": still_pairs,
        "jitter_mm_per_joint": per_joint,
        "jitter_mm_mean": round(float(np.mean(finite)), 3) if finite else None,
        "jitter_mm_fingertips": round(float(np.mean(fingertip)), 3)
        if fingertip
        else None,
        "pinch": _pinch(log.samples, intervals),
    }

    valid = [s.depth_valid for s in tracked if s.depth_valid is not None]
    if valid:
        stack = np.array(valid)
        out["depth_valid_rate_all"] = round(float(stack.mean()), 4)
        out["depth_valid_rate_fingertips"] = round(
            float(stack[:, list(FINGERTIPS)].mean()), 4
        )
    return out


def frame_interval_s(log: SourceLog) -> float | None:
    """Median gap between samples — the rate the pipeline actually ran at."""
    t = [s.t_s for s in log.samples]
    if len(t) < 2:
        return None
    return float(np.median(np.diff(np.array(t))))


# ---------------------------------------------------------------------------
# recording
# ---------------------------------------------------------------------------


def _phone_joints(shell: ShellControl):
    """The joints the shell is currently feeding the gesture engine.

    Returns (joints, source). `source` is the shell's own word for where they
    came from, which is what lets an off window be rejected if the injection
    had not actually retired yet.
    """
    try:
        dump = shell.hands_dump()
    except (ShellError, ValueError):
        return None, "unknown"
    hands = dump.get("hands", [])
    if not hands:
        return None, dump.get("source", "unknown")
    joints = np.array([j[:3] for j in hands[0]["joints"]], dtype=np.float64)
    return joints, dump.get("source", "unknown")


def _encode(hands) -> list[dict]:
    return [
        {"chirality": h.chirality, "confidence": h.confidence, "joints": h.joints}
        for h in hands
    ]


@dataclass
class Recording:
    phone: SourceLog
    mac: SourceLog
    dump_sources: set[str]
    depth_shape: tuple[int, int] | None
    image_shape: tuple[int, int] | None


def record(args, shell: ShellControl) -> Recording:
    """Alternate injection off / on, recording each source where it is real."""
    reader = ExportReader(args.dir)
    phone = SourceLog("phone")
    mac = SourceLog("mac")
    dump_sources: set[str] = set()
    depth_shape = None
    image_shape = None
    window = max(0.5, float(args.window_s))
    always_inject = bool(args.inject)

    phase = -1
    phase_started = 0.0
    injecting = always_inject
    t0 = time.monotonic()
    next_print = t0 + 1.0

    with tracker_from_args(args) as tracker:
        while True:
            t = time.monotonic() - t0
            if t >= args.seconds:
                break

            current = int(t // window)
            if current != phase:
                phase = current
                phase_started = t
                injecting = always_inject or (phase % 2 == 1)
                if not injecting:
                    # Retract explicitly: waiting out the shell's 150 ms
                    # staleness timeout would put two frames of the Mac's own
                    # joints into the phone column.
                    try:
                        shell.inject_hands(int(time.time() * 1000), [])
                    except ShellError:
                        pass

            frame = reader.wait(timeout_s=1.0)
            if frame is None:
                continue
            t = time.monotonic() - t0
            if frame.depth is not None:
                depth_shape = (int(frame.depth.shape[0]), int(frame.depth.shape[1]))
            image_shape = (frame.height, frame.width)

            result = tracker.track(frame)
            if injecting and result.hands:
                try:
                    shell.inject_hands(
                        int(time.time() * 1000),
                        _encode(result.hands),
                        frame_t_ns=frame.export_ns,
                    )
                except ShellError:
                    pass

            settled = t - phase_started >= SETTLE_S
            if injecting and settled:
                hand = result.hands[0] if result.hands else None
                mac.add(
                    t,
                    hand.joints if hand else None,
                    hand.depth_valid if hand else None,
                    held=bool(hand and hand.held),
                )

            joints, source = _phone_joints(shell)
            dump_sources.add(source)
            if not injecting and settled and source != "mac":
                phone.add(t, joints)

            if time.monotonic() >= next_print:
                next_print = time.monotonic() + 1.0
                print(
                    f"{t:5.1f}s  {'inject' if injecting else 'phone '}  "
                    f"mac {len(mac.tracked())}/{len(mac.samples)}  "
                    f"phone {len(phone.tracked())}/{len(phone.samples)}",
                    end="\r",
                    flush=True,
                )
    try:
        shell.inject_hands(int(time.time() * 1000), [])
    except ShellError:
        pass
    print()
    return Recording(phone, mac, dump_sources, depth_shape, image_shape)


# ---------------------------------------------------------------------------
# reporting
# ---------------------------------------------------------------------------


def _improvement(phone: dict, mac: dict, key: str) -> str:
    a, b = phone.get(key), mac.get(key)
    if a is None or b is None or not a:
        return "n/a"
    return f"{(a - b) / a * 100:+.0f}%"


def _fmt(v) -> str:
    return "n/a" if v is None else f"{v:.2f} mm"


def _pct(v) -> str:
    return "n/a" if v is None else f"{v:.0%}"


def _pinch_row(label: str, phone: dict, mac: dict, key: str, suffix="") -> str:
    def cell(d):
        v = d.get("pinch", {}).get(key)
        return "n/a" if v is None else (f"{v:.0%}" if suffix == "%" else f"{v:.0f} mm")

    return f"| {label} | {cell(phone)} | {cell(mac)} | |"


def write_report(results: dict, out_json: Path) -> Path:
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(results, indent=2) + "\n")

    phone, mac = results["phone"], results["mac"]
    depth = results.get("depth_map")
    rate = results.get("frame_interval_s")
    md = out_json.with_suffix(".md")
    lines = [
        "# Mac-side hand tracking vs the phone's",
        "",
        f"Recorded {results['seconds']:.0f} s on {results['recorded_at']} in "
        f"{results['window_s']:.0f} s alternating windows: injection off, so "
        "the shell's hands are the phone's, then injection on, so they are "
        "this tracker's. Each column is recorded only in its own windows, so "
        "neither is measuring the other.",
        "",
        f"| | phone (Apple Vision) | mac ({results['backend']}, "
        f"--depth {results['depth_mode']}) | change |",
        "| --- | --- | --- | --- |",
        f"| Frames with a hand | {phone['detect_rate']:.0%} | "
        f"{mac['detect_rate']:.0%} | "
        f"{(mac['detect_rate'] - phone['detect_rate']) * 100:+.0f} pts |",
        f"| Jitter, all joints | {_fmt(phone['jitter_mm_mean'])} | "
        f"{_fmt(mac['jitter_mm_mean'])} | "
        f"{_improvement(phone, mac, 'jitter_mm_mean')} |",
        f"| Jitter, fingertips | {_fmt(phone['jitter_mm_fingertips'])} | "
        f"{_fmt(mac['jitter_mm_fingertips'])} | "
        f"{_improvement(phone, mac, 'jitter_mm_fingertips')} |",
        _pinch_row("Thumb-index p10", phone, mac, "p10_mm"),
        _pinch_row("Thumb-index p50", phone, mac, "p50_mm"),
        _pinch_row("Thumb-index p90", phone, mac, "p90_mm"),
        _pinch_row("Still frames under 25 mm", phone, mac, "frac_under_25mm", "%"),
        "",
        "Jitter is the mean distance a joint moves between consecutive frames "
        "while the user is holding still, so it is motion they did not make. "
        "Stillness is one judgement about the user, taken from the phone "
        "track at a wrist speed under "
        f"{STILL_SPEED_MM_S:.0f} mm/s over {STILL_WINDOW_S:.1f} s, and both "
        "columns are scored over the same wall-clock intervals.",
        "",
        "The thumb-index rows are the pinch threshold's own margin: the "
        "gesture engine fires a pinch under 25 mm, so a source whose still "
        "hand reaches under that is firing pinches the user did not make.",
        "",
        "## The measurement itself",
        "",
        f"- LiDAR depth map: "
        + (f"{depth[1]}x{depth[0]}" if depth else "none seen"),
        f"- Camera frame: "
        + (
            f"{results['image'][1]}x{results['image'][0]}"
            if results.get("image")
            else "none seen"
        ),
        f"- Frame interval: "
        + (f"{rate * 1000:.0f} ms ({1 / rate:.1f} Hz)" if rate else "n/a"),
        f"- Still intervals: {results['still_intervals']} covering "
        f"{results['still_seconds']:.1f} s",
        f"- Still frame pairs scored: phone {phone['still_frame_pairs']}, "
        f"mac {mac['still_frame_pairs']}",
        f"- Frames the mac held after a dropped detection: {mac['frames_held']}",
        "",
        "## How much of the Mac's 3D is measured",
        "",
        f"- Landmarks with a real LiDAR reading: "
        f"{_pct(mac.get('depth_valid_rate_all'))}",
        f"- Fingertips with a real LiDAR reading: "
        f"{_pct(mac.get('depth_valid_rate_fingertips'))}",
        "",
    ]
    if results["depth_mode"] == "rigid":
        lines += [
            "Under `--depth rigid` those two are reporting only: nothing is "
            "placed with a per-landmark reading. The hand takes ONE range "
            "from the pooled patches under the wrist and the four finger "
            "MCPs, and the other twenty joints follow their own pixel rays "
            "out to ranges from a scaled adult hand-shape model.",
            "",
        ]
    if results.get("dump_sources") and "unknown" in results["dump_sources"]:
        lines += [
            "Caveat: the shell answered `hands dump` with no source for some "
            "frames of this run.",
            "",
        ]
    md.write_text("\n".join(lines).rstrip() + "\n")
    return md


def run_eval(args) -> int:
    try:
        shell = ShellControl(args.sock)
    except ShellError as exc:
        print(f"hands: {exc}", flush=True)
        return 1

    started_export = False
    try:
        if args.enable_export:
            print(shell.frame_export_on(args.dir, raw=args.raw), flush=True)
            started_export = True
        print(
            f"recording {args.seconds:.0f}s from {args.dir} "
            f"[{args.backend}, --depth {args.depth}], "
            f"{args.window_s:.0f}s windows",
            flush=True,
        )
        rec = record(args, shell)
    finally:
        if started_export:
            shell.frame_export_off()
        shell.close()

    if not rec.mac.samples:
        print("hands: no frames arrived — is `frame-export on <dir>` set?")
        return 1

    intervals = still_intervals(
        rec.phone.samples, bridge_s=float(args.window_s) + BRIDGE_SLACK_S
    )
    results = {
        "recorded_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "seconds": args.seconds,
        "window_s": float(args.window_s),
        "export_dir": args.dir,
        "backend": args.backend,
        "depth_mode": args.depth,
        "injected_whole_run": bool(args.inject),
        "dump_sources": sorted(rec.dump_sources),
        "depth_map": list(rec.depth_shape) if rec.depth_shape else None,
        "image": list(rec.image_shape) if rec.image_shape else None,
        "frame_interval_s": frame_interval_s(rec.mac),
        "still_intervals": len(intervals),
        "still_seconds": round(sum(b - a for a, b in intervals), 2),
        "phone": analyse(rec.phone, intervals),
        "mac": analyse(rec.mac, intervals),
    }

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_json = Path(args.out) if args.out else RESULTS_DIR / f"eval-{stamp}.json"
    md = write_report(results, out_json)
    print(f"wrote {out_json}\nwrote {md}")
    print(md.read_text())
    return 0
