"""Tune pinch and fist thresholds to the wearer's own hand.

Samples the joints the shell is actually using (`hands dump`) through three
prompted phases, computes the same features the gesture engine keys on, and
writes a gestures.toml the engine reads at start-up.
"""
from __future__ import annotations

import json
import math
import os
import statistics as st
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from .control import ShellControl

WRIST, THUMB_TIP = 0, 4
FINGERS = {"index": (5, 6, 8), "middle": (9, 10, 12), "ring": (13, 14, 16), "pinky": (17, 18, 20)}
PHASES = (
    ("open", "hold your hand open, fingers relaxed, palm toward the phone"),
    ("pinch", "hold a pinch: thumb and index tip touching"),
    ("fist", "make a fist and hold it"),
)
PHASE_S = 6.0
SETTLE_S = 1.5
SAMPLE_HZ = 20.0
# Where between the two clusters the trigger sits (0 = at the gesture, 1 = at rest).
PINCH_GAP = 0.35
FIST_GAP = 0.4
PINCH_HYSTERESIS_M = 0.013
FIST_HYSTERESIS = 0.10
CONFIG_PATH = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config") / "spatial-os" / "gestures.toml"


def _dist(a, b) -> float:
    return math.dist(a[:3], b[:3])


def _curl(j, mcp: int, pip: int, tip: int) -> float:
    """Angle between MCP->PIP and MCP->TIP over pi, exactly as the engine computes it."""
    base = [j[pip][i] - j[mcp][i] for i in range(3)]
    to_tip = [j[tip][i] - j[mcp][i] for i in range(3)]
    lb, lt = math.sqrt(sum(v * v for v in base)), math.sqrt(sum(v * v for v in to_tip))
    if lb < 1e-8 or lt < 1e-8:
        return 0.0
    c = max(-1.0, min(1.0, sum(a * b for a, b in zip(base, to_tip)) / (lb * lt)))
    return math.acos(c) / math.pi


def features(joints) -> dict[str, float]:
    curls = {name: _curl(joints, *ids) for name, ids in FINGERS.items()}
    return {
        "thumb_index_distance": _dist(joints[THUMB_TIP], joints[FINGERS["index"][2]]),
        "all_fingers_curl": sum(curls.values()) / 4,
        **{f"{name}_curl": v for name, v in curls.items()},
    }


def _pct(values: list[float], p: float) -> float:
    if not values:
        return float("nan")
    s = sorted(values)
    return s[min(len(s) - 1, int(round(p * (len(s) - 1))))]


@dataclass
class Thresholds:
    pinch_trigger_m: float
    pinch_release_m: float
    pinch_curl_max: float
    fist_all_curl: float
    fist_finger_curl: dict[str, float]
    fist_release: float

    def toml(self) -> str:
        fingers = "\n".join(f"trigger.{n}_curl = {v:.3f}" for n, v in self.fist_finger_curl.items())
        windowed = "\n".join(
            f"trigger.{n}_curl_max_over_200ms = {v:.3f}" for n, v in self.fist_finger_curl.items() if n != "pinky"
        )
        return f"""# Written by `hands calibrate` on {time.strftime('%Y-%m-%d %H:%M')}. Loaded when the shell starts.

[pinch_select.standard]
trigger.thumb_index_distance = {self.pinch_trigger_m:.4f}
trigger.thumb_to_index_line_distance = {self.pinch_trigger_m:.4f}
trigger.all_fingers_curl = {self.pinch_curl_max:.3f}
release.thumb_index_distance = {self.pinch_release_m:.4f}

[fist_launcher.per_frame]
trigger.all_fingers_curl = {self.fist_all_curl:.3f}
{fingers}
release.all_fingers_curl = {self.fist_release:.3f}

[fist_launcher.windowed]
trigger.all_fingers_curl = {self.fist_all_curl:.3f}
trigger.pinky_curl = {self.fist_finger_curl['pinky']:.3f}
{windowed}
release.all_fingers_curl = {self.fist_release:.3f}
"""


def derive(samples: dict[str, list[dict[str, float]]]) -> Thresholds:
    """Thresholds from per-phase feature samples. Pure, so the eval can test it."""
    opn, pinch, fist = samples["open"], samples["pinch"], samples["fist"]
    ti = lambda rows, p: _pct([r["thumb_index_distance"] for r in rows], p)
    curl = lambda rows, key, p: _pct([r[key] for r in rows], p)

    pinch_hi, open_lo = ti(pinch, 0.9), ti(opn, 0.1)
    trigger = pinch_hi + PINCH_GAP * max(open_lo - pinch_hi, 0.0)
    trigger = max(0.02, min(trigger, 0.06))
    release = trigger + PINCH_HYSTERESIS_M
    pinch_curl_max = max(0.28, curl(pinch, "all_fingers_curl", 0.9) + 0.05)

    open_hi, fist_lo = curl(opn, "all_fingers_curl", 0.9), curl(fist, "all_fingers_curl", 0.1)
    fist_all = open_hi + FIST_GAP * max(fist_lo - open_hi, 0.0)
    fist_all = max(0.25, min(fist_all, 0.6))
    finger = {}
    for name in FINGERS:
        o, f = curl(opn, f"{name}_curl", 0.9), curl(fist, f"{name}_curl", 0.1)
        finger[name] = max(0.15, min(o + FIST_GAP * max(f - o, 0.0), 0.6))
    return Thresholds(trigger, release, pinch_curl_max, fist_all, finger, max(0.1, fist_all - FIST_HYSTERESIS))


def jitter_mm(rows: list[dict[str, float]]) -> float:
    d = [r["thumb_index_distance"] for r in rows]
    return st.pstdev(d) * 1000 if len(d) > 1 else float("nan")


def sample_phase(shell: ShellControl, seconds: float, log: Callable[[str], None]) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    deadline = time.monotonic() + seconds
    misses = 0
    while time.monotonic() < deadline:
        reply = shell.request("hands dump")
        hands = json.loads(reply[3:]).get("hands", [])
        if hands:
            rows.append(features(hands[0]["joints"]))
        else:
            misses += 1
        time.sleep(1 / SAMPLE_HZ)
    if misses:
        log(f"  ({misses} samples without a hand)")
    return rows


def run(write: bool, results_dir: Path, log: Callable[[str], None] = print, phase_s: float = PHASE_S) -> Thresholds:
    samples: dict[str, list[dict[str, float]]] = {}
    with ShellControl() as shell:
        for name, prompt in PHASES:
            log(f"\n{name.upper()}: {prompt}")
            for n in range(3, 0, -1):
                log(f"  {n}…")
                time.sleep(1.0)
            log("  sampling")
            time.sleep(SETTLE_S)
            samples[name] = sample_phase(shell, phase_s, log)
            log(f"  {len(samples[name])} samples, thumb-index median {_pct([r['thumb_index_distance'] for r in samples[name]], 0.5) * 1000:.0f} mm, jitter {jitter_mm(samples[name]):.1f} mm")
    if any(len(v) < 10 for v in samples.values()):
        raise SystemExit("calibrate: too few samples in a phase; is the hand in view?")
    th = derive(samples)
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    results_dir.mkdir(parents=True, exist_ok=True)
    (results_dir / f"calibrate-{stamp}.json").write_text(json.dumps({"samples": samples, "thresholds": th.__dict__}, indent=1))
    log("\n" + th.toml())
    if write:
        CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
        CONFIG_PATH.write_text(th.toml())
        log(f"wrote {CONFIG_PATH}; restart the shell (scripts/run-mac.sh --restart) to load it")
    return th
