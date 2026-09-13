"""`hands eval` — is the Mac's tracking actually better than the phone's?

The claim being tested is that MediaPipe on the Mac, with LiDAR depth, gives
the gesture engine steadier and more available joints than Apple Vision on the
phone. Three numbers say whether that is true, and they are the three that
decide whether a pinch works:

  jitter_mm       mean frame-to-frame displacement of a joint WHILE THE HAND
                  IS HELD STILL. Motion the user did not make. This is what
                  makes a pinch threshold chatter and a pointer crawl. Only
                  still stretches count, because a moving hand should move.
  detect_rate     fraction of frames the source had a hand at all. A tracker
                  that drops the hand for 200 ms mid-gesture cancels it.
  depth_valid     fraction of FINGERTIP landmarks that got a real LiDAR
                  reading rather than an estimated range. Mac-side only —
                  it is the honest account of how much of the metric 3D is
                  measured and how much is inferred from the hand model.

Both sources are sampled from the same loop iteration, against the same
exported frame, so a difference cannot be an artefact of sampling one of them
more often. By default the phone stays the live source (`--inject` opts in),
because injecting would make `hands dump` return the Mac's own joints and the
comparison would be the Mac against itself.
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
from .geometry import FINGERTIPS
from .tracker import tracker_from_args

RESULTS_DIR = Path(__file__).resolve().parents[2] / "results"

# A stretch counts as "still" when the wrist stays inside this radius over the
# stillness window. Big enough to admit a hand a person is holding up (nobody
# is steadier than a centimetre), small enough to exclude a reach or a wave.
STILL_RADIUS_M = 0.02
STILL_WINDOW_S = 0.5
# A still stretch shorter than this is not enough samples to average over.
MIN_STILL_SAMPLES = 8


@dataclass
class Sample:
    t_s: float
    joints: np.ndarray | None  # (21, 3) scene metres, MediaPipe order
    depth_valid: np.ndarray | None = None  # (21,) bool, mac only


@dataclass
class SourceLog:
    name: str
    samples: list[Sample] = field(default_factory=list)

    def add(self, t_s, joints, depth_valid=None) -> None:
        self.samples.append(Sample(t_s, joints, depth_valid))


# ---------------------------------------------------------------------------
# analysis
# ---------------------------------------------------------------------------


def _still_runs(samples: list[Sample]) -> list[tuple[int, int]]:
    """Index ranges [start, end) where the wrist barely moved.

    Stillness is judged on the WRIST rather than on every joint, so a hand
    held in place while the fingers pinch still counts as still — which is
    exactly the moment the jitter matters most.
    """
    runs: list[tuple[int, int]] = []
    start = None
    for i, s in enumerate(samples):
        if s.joints is None:
            if start is not None and i - start >= MIN_STILL_SAMPLES:
                runs.append((start, i))
            start = None
            continue
        if start is None:
            start = i
            continue
        window = [
            samples[k].joints[0]
            for k in range(start, i + 1)
            if samples[k].joints is not None
            and s.t_s - samples[k].t_s <= STILL_WINDOW_S
        ]
        spread = np.ptp(np.array(window), axis=0).max() if len(window) > 1 else 0.0
        if spread > STILL_RADIUS_M:
            if i - start >= MIN_STILL_SAMPLES:
                runs.append((start, i))
            start = i
    if start is not None and len(samples) - start >= MIN_STILL_SAMPLES:
        runs.append((start, len(samples)))
    return runs


def _jitter_mm(samples: list[Sample]) -> tuple[list[float], int]:
    """Per-joint mean frame-to-frame displacement over the still stretches."""
    runs = _still_runs(samples)
    totals = np.zeros(21)
    counts = np.zeros(21)
    used = 0
    for start, end in runs:
        prev = None
        for i in range(start, end):
            joints = samples[i].joints
            if joints is None:
                prev = None
                continue
            if prev is not None:
                totals += np.linalg.norm(joints - prev, axis=1) * 1000.0
                counts += 1
                used += 1
            prev = joints
    with np.errstate(invalid="ignore", divide="ignore"):
        per_joint = np.where(counts > 0, totals / np.maximum(counts, 1), np.nan)
    return [None if np.isnan(v) else round(float(v), 3) for v in per_joint], used


def analyse(log: SourceLog) -> dict:
    n = len(log.samples)
    tracked = [s for s in log.samples if s.joints is not None]
    per_joint, still_pairs = _jitter_mm(log.samples)

    finite = [v for v in per_joint if v is not None]
    fingertip = [
        per_joint[i] for i in FINGERTIPS if per_joint[i] is not None
    ]

    out = {
        "source": log.name,
        "frames": n,
        "frames_with_hand": len(tracked),
        "detect_rate": round(len(tracked) / n, 4) if n else 0.0,
        "still_frame_pairs": still_pairs,
        "jitter_mm_per_joint": per_joint,
        "jitter_mm_mean": round(float(np.mean(finite)), 3) if finite else None,
        "jitter_mm_fingertips": round(float(np.mean(fingertip)), 3)
        if fingertip
        else None,
    }

    valid = [s.depth_valid for s in tracked if s.depth_valid is not None]
    if valid:
        stack = np.array(valid)
        out["depth_valid_rate_all"] = round(float(stack.mean()), 4)
        out["depth_valid_rate_fingertips"] = round(
            float(stack[:, list(FINGERTIPS)].mean()), 4
        )
    return out


# ---------------------------------------------------------------------------
# recording
# ---------------------------------------------------------------------------


def _tracked(log: SourceLog) -> int:
    return sum(1 for s in log.samples if s.joints is not None)


def _phone_joints(shell: ShellControl):
    """The joints the shell is currently feeding the gesture engine.

    Returns (joints, source). `source` is the shell's own word for where they
    came from, so the report can flag a run that accidentally measured the Mac
    against itself.
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


def record(args, shell: ShellControl) -> tuple[SourceLog, SourceLog, set[str]]:
    reader = ExportReader(args.dir)
    phone = SourceLog("phone")
    mac = SourceLog("mac")
    dump_sources: set[str] = set()
    t0 = time.monotonic()
    next_print = t0 + 1.0

    with tracker_from_args(args) as tracker:
        while time.monotonic() - t0 < args.seconds:
            frame = reader.wait(timeout_s=1.0)
            if frame is None:
                continue
            t = time.monotonic() - t0

            result = tracker.track(frame)
            if result.hands:
                hand = result.hands[0]
                mac.add(t, hand.joints, hand.depth_valid)
                if args.inject:
                    shell.inject_hands(
                        int(time.time() * 1000),
                        [
                            {
                                "chirality": h.chirality,
                                "confidence": h.confidence,
                                "joints": h.joints,
                            }
                            for h in result.hands
                        ],
                        frame_t_ns=frame.export_ns,
                    )
            else:
                mac.add(t, None)

            joints, source = _phone_joints(shell)
            dump_sources.add(source)
            phone.add(t, joints)

            if time.monotonic() >= next_print:
                next_print = time.monotonic() + 1.0
                print(
                    f"{time.monotonic() - t0:5.1f}s  "
                    f"mac {_tracked(mac)}/{len(mac.samples)}  "
                    f"phone {_tracked(phone)}/{len(phone.samples)}",
                    end="\r",
                    flush=True,
                )
    print()
    return phone, mac, dump_sources


# ---------------------------------------------------------------------------
# reporting
# ---------------------------------------------------------------------------


def _improvement(phone: dict, mac: dict, key: str) -> str:
    a, b = phone.get(key), mac.get(key)
    if a is None or b is None or not a:
        return "n/a"
    return f"{(a - b) / a * 100:+.0f}%"


def write_report(results: dict, out_json: Path) -> Path:
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(results, indent=2) + "\n")

    phone, mac = results["phone"], results["mac"]
    md = out_json.with_suffix(".md")
    lines = [
        "# Mac-side hand tracking vs the phone's",
        "",
        f"Recorded {results['seconds']:.0f} s on {results['recorded_at']}, "
        f"{phone['frames']} frames, both sources sampled against the same "
        "exported frame.",
        "",
        f"| | phone (Apple Vision) | mac ({results['backend']} + LiDAR) | "
        "change |",
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
        "",
        "Jitter is the mean distance a joint moves between consecutive frames "
        "while the wrist is held still, so it is motion the user did not make. "
        "Lower is better.",
        "",
        "## How much of the Mac's 3D is measured",
        "",
        f"- Landmarks with a real LiDAR reading: "
        f"{_pct(mac.get('depth_valid_rate_all'))}",
        f"- Fingertips with a real LiDAR reading: "
        f"{_pct(mac.get('depth_valid_rate_fingertips'))}",
        "",
        "The rest take their range from MediaPipe's hand model, relative to a "
        "landmark that did get a reading, and keep their own measured "
        "direction. Fingertips score lower than the hand as a whole because a "
        "fingertip at arm's length covers less than one depth pixel.",
        "",
        f"Still stretches used: phone {phone['still_frame_pairs']} frame pairs, "
        f"mac {mac['still_frame_pairs']}.",
    ]
    if results.get("dump_sources") and results["dump_sources"] != ["phone"]:
        lines += [
            "",
            f"Caveat: the shell reported hand sources {results['dump_sources']} "
            "during this run, so the \"phone\" column may include frames where "
            "the Mac's own joints were live. Re-run without `--inject` for a "
            "clean comparison.",
        ]
    md.write_text("\n".join(lines) + "\n")
    return md


def _fmt(v) -> str:
    return "n/a" if v is None else f"{v:.2f} mm"


def _pct(v) -> str:
    return "n/a" if v is None else f"{v:.0%}"


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
            f"[{args.backend}]",
            flush=True,
        )
        phone, mac, dump_sources = record(args, shell)
    finally:
        if started_export:
            shell.frame_export_off()
        shell.close()

    if not mac.samples:
        print("hands: no frames arrived — is `frame-export on <dir>` set?")
        return 1

    results = {
        "recorded_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "seconds": args.seconds,
        "export_dir": args.dir,
        "backend": args.backend,
        "injected_while_recording": bool(args.inject),
        "dump_sources": sorted(dump_sources),
        "phone": analyse(phone),
        "mac": analyse(mac),
    }

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_json = Path(args.out) if args.out else RESULTS_DIR / f"eval-{stamp}.json"
    md = write_report(results, out_json)
    print(f"wrote {out_json}\nwrote {md}")
    print(md.read_text())
    return 0
