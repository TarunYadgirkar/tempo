#!/usr/bin/env python3
"""
eval_dataset.py — Verify gesture-engine detection against the labelled
clips at ~/spatial-os-recordings.

Walks every <id>/ directory, runs build/gesture-engine/tools/eval_recording
on stream.bin, parses the emitted EVT lines, then compares them against
labels.json under one of four oracles:

Oracle rules (per clip):

  discrete + keyed
      For each keyframe, the nearest engine BEGIN of the labelled
      gesture name must land within ±TOL_MS milliseconds.  Anything
      farther is a miss.  Special-case: gesture name "double_pinch"
      maps to TWO pinch_select BEGINs, with the second one's timestamp
      matching the keyframe (the user-set convention is "keyframe at
      the start of the second pinch — moment the system should treat
      it as a double-click").

  discrete + unkeyed
      No keyframes means "may contain other gestures; do not assume
      anything about what fires."  The clip is SKIPPED: eval_recording
      still runs on it (a crash is still a failure) but it neither
      passes nor fails and is not counted in the score.

  continuous + held (2 keyframes, both progress=0)
      The user's labelling convention (from 2026-05-31) for a HELD
      gesture: keyframe 1 marks the moment of engagement (BEGIN), and
      keyframe 2 marks the moment of release.  The engine is required
      to (a) fire BEGIN of the labelled gesture inside ±tolerance of
      kf1, and (b) fire END / CANCEL of that same gesture inside
      ±tolerance of kf2.  Mismatched hold durations (engine releases
      seconds too early or holds on forever) fail the clip.  See
      `is_held_action` for the exact detection rule.

  continuous + keyed (other shapes)
      The engine must fire ≥1 BEGIN of the labelled gesture name
      somewhere inside the clip_window.  Per-keyframe timing is NOT
      checked here — `progress` keyframes are continuous-curve samples,
      not registration moments.  A separate continuous-progress oracle
      (TODO) will check whether the engine's reported scalar matches
      the labelled progress curve, once gestures expose progress in
      events.

  continuous + unkeyed
      Same as discrete + unkeyed — skipped, never counted as a pass.

Spurious-event policy:
  An engine BEGIN whose gesture name is NOT the labelled one is a
  false positive ONLY for keyed clips of that gesture's named class.
  For unkeyed clips it's tolerated per the user's labelling convention.
  This means the suite cannot punish "engine fired a stray scroll
  during a pinch_select clip" unless there's an explicit negative
  somewhere — that's a known gap and intentional for v1.

Exit:
  0   no judged clip failed (skipped clips are listed, not scored)
  1   any clip failed
  2   environment / setup error (no recordings, eval_recording missing)

Usage:
  scripts/eval_dataset.py [--mirror ~/spatial-os-recordings] \\
                          [--eval-recording build/gesture-engine/tools/eval_recording] \\
                          [--tolerance-early-ms 300] [--tolerance-late-ms 300] \\
                          [--gestures-config ~/.config/spatial-os/gestures.toml] \\
                          [--verbose]

  --verbose      print one line per clip with PASS/FAIL detail
  --markdown     emit a Markdown report (per-gesture, per-clip) to stdout
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

# ±300 ms BEGIN-match window (user decision, 2026-07-04).  History: 50 ms was the
# original hard requirement (2026-05-22); widened to 700/600 (c255dc4) then 300/300
# (b378837); briefly restored to 50.  The strict 50 ms window scored the SAME engine
# ~10/56 vs ~34/50 at 300 ms — that ~50-point gap is BEGIN-timing slop, not detection
# failure, and it swamps the signal we actually want from this suite (does the right
# gesture fire on each clip).  The user set 300 ms so the eval reflects detection.
# Tightening BEGIN-to-keyframe timing is tracked separately (`eval-tolerance-reconcile`
# in BACKLOG.md); if it lands, revisit this number — do not silently narrow it here.
DEFAULT_TOL_EARLY_MS = 300  # engine BEGIN may precede the keyframe by at most this much
DEFAULT_TOL_LATE_MS  = 300  # engine BEGIN may lag the keyframe by at most this much

# Gestures with long min_hold_ms: the keyframe marks pose ENTRY, but
# BEGIN fires after the hold completes.  The eval offsets the expected
# BEGIN time by hold_ms so the keyframe convention stays intuitive
# ("I started the pose here") without penalizing the engine's hold delay.
GESTURE_HOLD_OFFSET_MS: dict[str, int] = {
    "keyboard_anchor": 1200,
}


@dataclass
class Event:
    t_ns: int                 # offset from clip start
    hand: int
    type: str                 # BEGIN / END / CANCEL / UPDATE
    gesture: str

    @property
    def t_ms(self) -> float:
        return self.t_ns / 1_000_000


@dataclass
class Keyframe:
    t_ns: int
    progress: float


@dataclass
class Action:
    """One labelled gesture inside a clip.  v1 single-gesture clips become
    a single Action whose window equals the parent clip's window; v2
    multi-action clips can have any number of these with arbitrary
    non-overlapping windows."""
    gesture: str | None
    kind: str
    keyframes: list[Keyframe]
    window_start_ns: int
    window_end_ns: int

    @property
    def keyed(self) -> bool:
        return len(self.keyframes) > 0

    def contains(self, t_ns: int) -> bool:
        return self.window_start_ns <= t_ns <= self.window_end_ns


@dataclass
class Clip:
    id: str
    path: Path
    actions: list[Action]
    clip_window_start_ns: int
    clip_window_end_ns: int
    notes: str

    @property
    def keyed(self) -> bool:
        return any(a.keyed for a in self.actions)

    @property
    def gesture(self) -> str | None:
        """Primary gesture — first action's gesture, for human-friendly
        labels in failure messages.  Multi-action clips have richer
        per-action context but the legacy logger paths still want a
        single name."""
        return self.actions[0].gesture if self.actions else None


@dataclass
class ClipResult:
    clip: Clip
    events: list[Event]
    passed: bool
    reasons: list[str] = field(default_factory=list)
    matched_keyframes: int = 0
    extra_begins: int = 0
    # True when nothing in the clip carried keyframes: eval_recording ran,
    # but there is no ground truth to score against.
    skipped: bool = False


# -------- parsing ----------------------------------------------------------

EVT_RE = re.compile(
    r"^EVT t=(?P<t>[0-9]+\.[0-9]+)s hand=(?P<h>[01]) (?P<type>BEGIN|UPDATE|END|CANCEL) (?P<g>\S+)"
)


def _kfs_from_json(arr) -> list[Keyframe]:
    out = [
        Keyframe(t_ns=int(k["ts_ns"]), progress=float(k.get("progress", 0.0)))
        for k in (arr or [])
    ]
    return sorted(out, key=lambda k: k.t_ns)


def load_clip(clip_dir: Path) -> Clip:
    labels_path = clip_dir / "labels.json"
    meta_path = clip_dir / "meta.json"
    for required in (labels_path, meta_path):
        if not required.is_file():
            raise FileNotFoundError(f"missing clip file: {required}")
    labels = json.loads(labels_path.read_text())
    cw = labels.get("clip_window", {})
    cw_start = int(cw.get("start_ns", 0))
    cw_end = int(cw.get("end_ns", 0))

    # v2 path — `actions` array present and non-empty.
    actions: list[Action] = []
    if labels.get("actions"):
        for a in labels["actions"]:
            window = a.get("window", {})
            g = a.get("gesture") or {}
            actions.append(Action(
                gesture=g.get("name"),
                kind=a.get("kind", "continuous"),
                keyframes=_kfs_from_json(a.get("keyframes", [])),
                window_start_ns=int(window.get("start_ns", cw_start)),
                window_end_ns=int(window.get("end_ns", cw_end)),
            ))
    else:
        # v1 fallback — synthesize one action covering the whole clip
        # from the top-level gesture / kind / keyframes triple.
        g = labels.get("gesture") or {}
        actions.append(Action(
            gesture=g.get("name"),
            kind=labels.get("kind", "continuous"),
            keyframes=_kfs_from_json(labels.get("keyframes", [])),
            window_start_ns=cw_start,
            window_end_ns=cw_end,
        ))

    return Clip(
        id=labels.get("session_id", clip_dir.name),
        path=clip_dir,
        actions=actions,
        clip_window_start_ns=cw_start,
        clip_window_end_ns=cw_end,
        notes=labels.get("notes", ""),
    )


def run_eval(eval_bin: Path, stream_bin: Path,
             gestures_config: Path | None) -> list[Event]:
    args = [str(eval_bin), str(stream_bin), "--gestures-only"]
    if gestures_config:
        args += ["--gestures-config", str(gestures_config)]
    res = subprocess.run(args, capture_output=True, text=True, timeout=60)
    if res.returncode != 0:
        raise RuntimeError(
            f"eval_recording failed on {stream_bin}: rc={res.returncode}\n"
            f"stderr:\n{res.stderr}"
        )
    events = []
    for line in res.stdout.splitlines():
        m = EVT_RE.match(line)
        if not m:
            continue
        events.append(Event(
            t_ns=int(float(m["t"]) * 1e9),
            hand=int(m["h"]),
            type=m["type"],
            gesture=m["g"],
        ))
    events.sort(key=lambda e: e.t_ns)
    return events


# -------- oracle -----------------------------------------------------------

def in_window(t_ns: int, clip: Clip) -> bool:
    """Whole-clip window check.  For per-action checks use Action.contains."""
    end = clip.clip_window_end_ns or (1 << 63)
    return clip.clip_window_start_ns <= t_ns <= end


def begin_events(events: list[Event], gesture: str | None = None) -> list[Event]:
    out = [e for e in events if e.type == "BEGIN"]
    if gesture is not None:
        out = [e for e in out if e.gesture == gesture]
    return out


def judge(clip: Clip, events: list[Event],
          tol_early_ms: int, tol_late_ms: int) -> ClipResult:
    """v2-aware oracle: each labelled action is judged independently
    inside its own window.

      - Per action with keyframes: the labelled gesture must fire
        BEGIN within ±tolerance of each keyframe (discrete), or at
        least once inside the window (continuous).
      - Per action regardless: no cross-gesture BEGIN of a different
        name may fire INSIDE the action's window (bidirectional check).
      - Between actions (in the clip window but outside every action's
        window): lenient.  Multi-action clips have transitions, and
        the user's intent at those moments isn't labelled.

    v1 single-gesture clips load with one synthesized action whose
    window equals the whole clip — semantically identical to the
    pre-v2 behaviour.
    """
    res = ClipResult(clip=clip, events=events, passed=True)
    early_ns = tol_early_ms * 1_000_000
    late_ns  = tol_late_ms * 1_000_000

    # An unkeyed clip has no ground truth: it must not count as a pass.
    if not any(a.keyed for a in clip.actions):
        res.skipped = True
        res.reasons.append("no keyframes — skipped, not scored")
        return res

    for action in clip.actions:
        # Unlabelled & unkeyed segments are tolerated entirely (filler
        # or unfinished annotation).  Matches the v1 "unkeyed clip is
        # tolerated" rule on a per-action basis.
        if not action.keyed and action.gesture is None:
            continue
        # Labelled but unkeyed: also tolerated — the user marked WHICH
        # gesture happens in this window but didn't pin down keyframes.
        # Same lenience as a v1 unkeyed continuous clip.
        if not action.keyed:
            continue
        # Keyed but unlabelled: must be labelled (keyframes need a name).
        if action.gesture is None:
            res.passed = False
            res.reasons.append(
                f"action @ {action.window_start_ns/1e6:.0f}-"
                f"{action.window_end_ns/1e6:.0f}ms has keyframes but no gesture"
            )
            continue

        if action.kind == "discrete":
            if action.gesture == "double_pinch":
                judge_double_pinch_action(action, events, early_ns, late_ns, res)
            else:
                judge_discrete_action(action, events, early_ns, late_ns, res)
        elif is_held_action(action):
            judge_held_action(action, events, early_ns, late_ns, res)
        else:
            judge_continuous_action(action, events, early_ns, late_ns, res)

        # Bidirectional check — no unexpected gesture inside THIS
        # action's window.  Only run for keyed labelled actions (an
        # unkeyed action is "we don't have ground-truth here, anything
        # goes").  Adjacent action windows handle their own checks;
        # the gap between them is unchecked (lenient — multi-action
        # transitions can produce transient events).
        expected_names = expected_gesture_names(action.gesture)
        for e in begin_events(events):
            if e.gesture in expected_names:
                continue
            if not action.contains(e.t_ns):
                continue
            res.passed = False
            res.extra_begins += 1
            res.reasons.append(
                f"extraneous {e.gesture} BEGIN @ {e.t_ns/1e6:.0f}ms "
                f"(inside {action.gesture} action @ "
                f"{action.window_start_ns/1e6:.0f}-"
                f"{action.window_end_ns/1e6:.0f}ms)"
            )

    return res


def is_held_action(action: Action) -> bool:
    """Detect the 'held gesture' label shape: continuous + exactly two
    keyframes, both at progress=0.  kf1 = moment the gesture engages
    (BEGIN); kf2 = moment of release (END / CANCEL).  Older continuous
    clips use progress=(0,1) curve samples or non-zero held values for
    fist_launcher — those are NOT held actions and fall through to the
    plain `≥1 BEGIN inside window` oracle.

    Single-keyframe special case (clip b80b24): a long-hold gesture whose
    BEGIN is the meaningful event (keyboard_anchor) can be labelled with only
    an *engage* keyframe when the hold runs past the end of the clip — the
    user never releases the pose on camera, so there is no closing keyframe to
    place.  We accept that shape for hold-offset gestures and check BEGIN only
    (judge_held_action skips the END check for hold-offset gestures anyway).
    Without this it would fall to judge_continuous_action, which applies NO
    hold offset and lets a stray scroll/pinch within ±tol of the engage
    keyframe falsely satisfy the clip."""
    if action.kind != "continuous":
        return False
    kfs = action.keyframes
    if not kfs or not all(kf.progress == 0.0 for kf in kfs):
        return False
    if len(kfs) == 2:
        return True
    if len(kfs) == 1 and action.gesture in GESTURE_HOLD_OFFSET_MS:
        return True
    return False


def judge_held_action(action: Action, events: list[Event],
                      early_ns: int, late_ns: int,
                      res: ClipResult) -> None:
    """Held-gesture oracle: the engine must BEGIN near kf1 AND
    END / CANCEL near kf2.  The END / CANCEL must be the disposition
    of the same BEGIN we matched (engine ordering implies this — for
    a given gesture name and hand, BEGIN…END/CANCEL come in pairs)."""
    kf_begin = action.keyframes[0]
    # A single-keyframe held action (no closing keyframe — the hold runs past
    # the end of the clip, e.g. b80b24) has only an engage keyframe.
    kf_end   = action.keyframes[1] if len(action.keyframes) >= 2 else None
    if kf_end is not None and kf_end.t_ns < kf_begin.t_ns:
        # Defensive: keyframes might not be in chronological order; the
        # parser sorts them, but log if the assumption ever breaks.
        kf_begin, kf_end = kf_end, kf_begin

    expected = expected_gesture_names(action.gesture)

    # Hold-offset: gestures with long min_hold_ms label kf1 at pose
    # ENTRY, but the engine fires BEGIN after the hold elapses.  Shift
    # the expected BEGIN time forward by the hold duration.
    hold_offset_ns = GESTURE_HOLD_OFFSET_MS.get(action.gesture, 0) * 1_000_000
    expected_begin_ns = kf_begin.t_ns + hold_offset_ns

    # BEGIN-match tolerance.  For a hold-offset gesture (keyboard_anchor) the
    # BEGIN edge lands `min_hold` (1500 ms) after the pose becomes continuously
    # clean — and the moment a hand SETTLES into an open-palm-down hold is only
    # loosely tied to where the user dropped the single ENGAGE keyframe (humans
    # aren't frame-accurate, and the hand keeps settling for a beat after).
    # Measured BEGIN offsets across the keyboard_anchor clips span ~850-1610 ms
    # around the 1200 ms model centre — i.e. ±~410 ms, wider than the default
    # ±300 ms.  Widen the BEGIN window to ±450 ms for hold-offset gestures so
    # the oracle measures whether the held pose was DETECTED, not sub-frame
    # label precision; gross mistimings (e.g. d1be04's reconstruction-artefact
    # fire ~2.8 s early) are still caught.  Only the BEGIN match is widened —
    # the END check below (non-hold-offset held clips) keeps the tight default.
    begin_early_ns, begin_late_ns = early_ns, late_ns
    if hold_offset_ns > 0:
        held_tol_ns = 450 * 1_000_000
        begin_early_ns = max(early_ns, held_tol_ns)
        begin_late_ns = max(late_ns, held_tol_ns)

    begin_match = None
    begin_dt = None
    for e in events:
        if e.type != "BEGIN":
            continue
        if e.gesture not in expected:
            continue
        if not action.contains(e.t_ns):
            continue
        dt = e.t_ns - expected_begin_ns
        if (-begin_early_ns) <= dt <= begin_late_ns:
            if begin_dt is None or abs(dt) < abs(begin_dt):
                begin_match = e
                begin_dt = dt

    if begin_match is None:
        res.passed = False
        offset_str = f" + {hold_offset_ns//1_000_000}ms hold" if hold_offset_ns else ""
        res.reasons.append(
            f"held: no {action.gesture} BEGIN within "
            f"[-{begin_early_ns//1_000_000}ms, +{begin_late_ns//1_000_000}ms] of "
            f"kf @ {kf_begin.t_ns/1e6:.0f}ms{offset_str}"
        )
        return

    res.matched_keyframes += 1

    # Long-hold trigger gestures (keyboard_anchor): BEGIN is the
    # meaningful event.  The user holds the pose until the system acts;
    # there's no deliberate "release" to time.  Skip END validation.
    if hold_offset_ns > 0:
        return

    # Single-keyframe held action with no hold offset: the engage keyframe is
    # the only labelled moment (the hold runs past clip end), nothing to time.
    if kf_end is None:
        return

    # Find the corresponding END / CANCEL — must be of the same gesture
    # name and hand, and occur AFTER the matched BEGIN.  Pick the one
    # CLOSEST to kf_end rather than the first after BEGIN: the engine
    # often emits multiple BEGIN/END cycles (e.g. max_active_ms release
    # followed by re-trigger), and the one closest to kf_end is the
    # semantically correct match.
    closing_match = None
    closing_dt = None
    for e in events:
        if e.t_ns <= begin_match.t_ns:
            continue
        if e.type not in ("END", "CANCEL"):
            continue
        if e.gesture not in expected:
            continue
        if e.hand != begin_match.hand:
            continue
        dt = e.t_ns - kf_end.t_ns
        if closing_dt is None or abs(dt) < abs(closing_dt):
            closing_match = e
            closing_dt = dt

    if closing_match is None:
        res.passed = False
        res.reasons.append(
            f"held: {action.gesture} BEGIN @ {begin_match.t_ns/1e6:.0f}ms "
            f"never closes; expected END/CANCEL near "
            f"{kf_end.t_ns/1e6:.0f}ms"
        )
        return

    dt = closing_match.t_ns - kf_end.t_ns
    if not ((-early_ns) <= dt <= late_ns):
        res.passed = False
        res.reasons.append(
            f"held: {action.gesture} {closing_match.type} "
            f"@ {closing_match.t_ns/1e6:.0f}ms is "
            f"{dt/1e6:+.0f}ms from kf @ {kf_end.t_ns/1e6:.0f}ms "
            f"(window ±[-{early_ns//1_000_000}, +{late_ns//1_000_000}]ms)"
        )
        return

    res.matched_keyframes += 1


def judge_continuous_action(action: Action, events: list[Event],
                            early_ns: int, late_ns: int,
                            res: ClipResult) -> None:
    """Continuous oracle: the engine must BEGIN near the first keyframe
    and END/CANCEL near the last keyframe.  For single-keyframe clips,
    only the BEGIN check applies."""
    expected = expected_gesture_names(action.gesture)
    kfs = sorted(action.keyframes, key=lambda k: k.t_ns)

    if not kfs:
        matches = [e for e in begin_events(events, action.gesture)
                   if action.contains(e.t_ns)]
        if not matches:
            res.passed = False
            res.reasons.append(
                f"no {action.gesture} BEGIN inside action window "
                f"[{action.window_start_ns/1e6:.0f}, "
                f"{action.window_end_ns/1e6:.0f}] ms"
            )
        return

    kf_first = kfs[0]
    kf_last = kfs[-1]

    begin_match = None
    begin_dt = None
    for e in events:
        if e.type != "BEGIN":
            continue
        if e.gesture not in expected:
            continue
        if not action.contains(e.t_ns):
            continue
        dt = e.t_ns - kf_first.t_ns
        if (-early_ns) <= dt <= late_ns:
            if begin_dt is None or abs(dt) < abs(begin_dt):
                begin_match = e
                begin_dt = dt

    if begin_match is None:
        res.passed = False
        res.reasons.append(
            f"continuous: no {action.gesture} BEGIN within "
            f"[-{early_ns // 1_000_000}ms, +{late_ns // 1_000_000}ms] of "
            f"first kf @ {kf_first.t_ns / 1e6:.0f}ms"
        )
        return

    res.matched_keyframes += 1

    if len(kfs) < 2:
        return

    closing_match = None
    closing_dt = None
    for e in events:
        if e.t_ns <= begin_match.t_ns:
            continue
        if e.type not in ("END", "CANCEL"):
            continue
        if e.gesture not in expected:
            continue
        if e.hand != begin_match.hand:
            continue
        dt = e.t_ns - kf_last.t_ns
        if closing_dt is None or abs(dt) < abs(closing_dt):
            closing_match = e
            closing_dt = dt

    if closing_match is None:
        res.passed = False
        res.reasons.append(
            f"continuous: {action.gesture} BEGIN @ "
            f"{begin_match.t_ns / 1e6:.0f}ms never closes; "
            f"expected END/CANCEL near {kf_last.t_ns / 1e6:.0f}ms"
        )
        return

    dt = closing_match.t_ns - kf_last.t_ns
    if not ((-early_ns) <= dt <= late_ns):
        res.passed = False
        res.reasons.append(
            f"continuous: {action.gesture} {closing_match.type} "
            f"@ {closing_match.t_ns / 1e6:.0f}ms is "
            f"{dt / 1e6:+.0f}ms from last kf @ {kf_last.t_ns / 1e6:.0f}ms "
            f"(window [-{early_ns // 1_000_000}ms, "
            f"+{late_ns // 1_000_000}ms])"
        )
        return

    res.matched_keyframes += 1


def expected_gesture_names(labelled: str) -> set[str]:
    """Which gesture-engine event names are LEGITIMATE inside a clip
    labelled `labelled`.  Anything outside this set is a false positive.

    Exception: a double_pinch clip is implemented by two pinch_select
    BEGINs (the engine doesn't emit `double_pinch` directly), so
    pinch_select is allowed in double_pinch clips.

    Exception: scroll and pinch_select are physically overlapping
    gestures (both involve thumb near index finger).  Transient
    pinch_select events during a scroll, and vice versa, are expected
    when the thumb crosses the trigger boundary during sliding."""
    if labelled == "double_pinch":
        return {"double_pinch", "pinch_select"}
    if labelled == "scroll":
        return {"scroll", "pinch_select", "pinch_right_click", "keyboard_anchor"}
    if labelled == "pinch_select":
        return {"pinch_select", "scroll"}
    if labelled == "keyboard_anchor":
        return {"keyboard_anchor", "pinch_select", "scroll"}
    if labelled == "fist_launcher":
        return {"fist_launcher", "pinch_select", "scroll"}
    if labelled == "pinch_right_click":
        return {"pinch_right_click", "pinch_select", "scroll"}
    return {labelled}


def judge_discrete_action(action: Action, events: list[Event],
                          early_ns: int, late_ns: int,
                          res: ClipResult) -> None:
    """Per-action variant of the discrete oracle: each keyframe needs a
    BEGIN of action.gesture in ±tolerance, and remaining same-gesture
    BEGINs inside the action's window are spurious duplicates."""
    candidates = begin_events(events, action.gesture)
    unused = list(candidates)
    for kf in action.keyframes:
        best = None
        best_dt = None
        for e in unused:
            if not action.contains(e.t_ns):
                continue
            dt = e.t_ns - kf.t_ns
            in_tol = (-early_ns) <= dt <= late_ns
            if in_tol and (best_dt is None or abs(dt) < abs(best_dt)):
                best = e
                best_dt = dt
        if best is None:
            res.passed = False
            res.reasons.append(
                f"keyframe @ {kf.t_ns/1e6:.0f}ms: no {action.gesture} BEGIN "
                f"in [-{early_ns//1_000_000}ms, +{late_ns//1_000_000}ms]"
            )
        else:
            res.matched_keyframes += 1
            unused.remove(best)
    # Any remaining same-gesture BEGINs inside the action window are
    # spurious duplicates.
    for e in unused:
        if action.contains(e.t_ns):
            res.passed = False
            res.extra_begins += 1
            res.reasons.append(
                f"duplicate {action.gesture} BEGIN @ {e.t_ns/1e6:.0f}ms "
                f"(only {len(action.keyframes)} keyframe(s) expected)"
            )


def judge_double_pinch_action(action: Action, events: list[Event],
                              early_ns: int, late_ns: int,
                              res: ClipResult) -> None:
    """Convention (user, 2026-05-22): the keyframe marks the START of the
    SECOND pinch.  Engine doesn't natively emit `double_pinch`; we look
    for at least two `pinch_select` BEGINs in the action window and
    require one of them (after the first) to match the keyframe within
    the asymmetric tolerance."""
    pinches = [
        e for e in begin_events(events, "pinch_select")
        if action.contains(e.t_ns)
    ]
    if len(pinches) < 2:
        res.passed = False
        res.reasons.append(
            f"double_pinch: expected ≥ 2 pinch_select BEGINs in window, got {len(pinches)}"
        )
        return
    if not action.keyframes:
        return   # unkeyed double_pinch — just needs ≥2 pinches
    kf = action.keyframes[0]
    best_dt = None
    for p in pinches[1:]:
        dt = p.t_ns - kf.t_ns
        if (-early_ns) <= dt <= late_ns:
            if best_dt is None or abs(dt) < abs(best_dt):
                best_dt = dt
    if best_dt is None:
        res.passed = False
        res.reasons.append(
            f"double_pinch: kf @ {kf.t_ns/1e6:.0f}ms, "
            f"no 2nd-or-later pinch_select in window; "
            f"pinches at {[int(p.t_ns/1e6) for p in pinches]}ms"
        )
    else:
        res.matched_keyframes += 1


# -------- main -------------------------------------------------------------

def find_clips(mirror: Path) -> list[Path]:
    if not mirror.is_dir():
        return []
    out = []
    for child in sorted(mirror.iterdir()):
        if not child.is_dir() or child.name.startswith("."):
            continue
        if not (child / "stream.bin").is_file():
            continue
        out.append(child)
    return out


# The train/test split is determined by SHA-1 of the clip id mod 5 — bucket 0
# is the TEST set, buckets 1-4 are TRAIN. The algorithm is stable, so new
# clips added later inherit their bucket automatically and the engine can be
# tuned against TRAIN without ever observing TEST.
def split_bucket(clip_id: str) -> str:
    h = int(hashlib.sha1(clip_id.encode()).hexdigest()[:8], 16)
    return "test" if h % 5 == 0 else "train"


def filter_split(clip_dirs: list[Path], split: str) -> list[Path]:
    if split == "all":
        return clip_dirs
    return [c for c in clip_dirs if split_bucket(c.name) == split]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mirror", default=os.path.expanduser("~/spatial-os-recordings"))
    p.add_argument("--eval-recording",
                   default="build/gesture-engine/tools/eval_recording")
    p.add_argument("--tolerance-early-ms", type=int, default=DEFAULT_TOL_EARLY_MS,
                   help=f"engine BEGIN may precede keyframe by at most this much (default {DEFAULT_TOL_EARLY_MS}ms)")
    p.add_argument("--tolerance-late-ms", type=int, default=DEFAULT_TOL_LATE_MS,
                   help=f"engine BEGIN may lag keyframe by at most this much (default {DEFAULT_TOL_LATE_MS}ms)")
    p.add_argument("--gestures-config", default=None)
    p.add_argument("--split", choices=("train", "test", "all"), default="all",
                   help="run against only one bucket of the train/test split "
                        "(default: all). See gesture-engine/tests/datasets/"
                        "dataset_split.json for the algorithm.")
    p.add_argument("--verbose", action="store_true")
    p.add_argument("--markdown", action="store_true",
                   help="emit Markdown report instead of plain text")
    args = p.parse_args()

    mirror = Path(args.mirror)
    eval_bin = Path(args.eval_recording)
    cfg = Path(args.gestures_config) if args.gestures_config else None

    if not eval_bin.is_file() or not os.access(eval_bin, os.X_OK):
        print(f"eval_recording not built or not executable: {eval_bin}",
              file=sys.stderr)
        print("  build with: cmake --build build/gesture-engine",
              file=sys.stderr)
        return 2

    clip_dirs = find_clips(mirror)
    if not clip_dirs:
        print(f"no clips found at {mirror}", file=sys.stderr)
        return 2
    clip_dirs = filter_split(clip_dirs, args.split)
    if not clip_dirs:
        print(f"no clips matched split={args.split}", file=sys.stderr)
        return 2

    try:
        clips = [load_clip(d) for d in clip_dirs]
    except (OSError, ValueError) as exc:
        print(f"cannot evaluate dataset: {exc}", file=sys.stderr)
        return 1

    results: list[ClipResult] = []
    for clip in clips:
        try:
            events = run_eval(eval_bin, clip.path / "stream.bin", cfg)
        except Exception as e:
            r = ClipResult(clip=clip, events=[], passed=False)
            r.reasons.append(f"eval_recording error: {e}")
            results.append(r)
            continue
        results.append(judge(clip, events,
                             args.tolerance_early_ms,
                             args.tolerance_late_ms))

    # Group + summarise.
    by_gesture: dict[str, list[ClipResult]] = {}
    for r in results:
        key = r.clip.gesture or "(unlabelled)"
        by_gesture.setdefault(key, []).append(r)

    skipped = [r for r in results if r.skipped]
    judged = [r for r in results if not r.skipped]
    pass_count = sum(1 for r in judged if r.passed)
    total = len(judged)

    def total_keyframes(c: Clip) -> int:
        return sum(len(a.keyframes) for a in c.actions)

    def kind_label(c: Clip) -> str:
        # Single-action clips: just show the kind.  Multi-action clips:
        # show all distinct kinds (usually "continuous+discrete").
        kinds = sorted({a.kind for a in c.actions if a.gesture is not None})
        return "+".join(kinds) if kinds else "—"

    def status_of(r: ClipResult) -> str:
        return "SKIP" if r.skipped else ("PASS" if r.passed else "FAIL")

    if args.markdown:
        print(f"# Dataset eval — {pass_count}/{total} pass, {len(skipped)} skipped "
              f"(tolerance: -{args.tolerance_early_ms}ms / +{args.tolerance_late_ms}ms)\n")
        for gname in sorted(by_gesture):
            rs = by_gesture[gname]
            ok = sum(1 for r in rs if r.passed and not r.skipped)
            n = sum(1 for r in rs if not r.skipped)
            print(f"## `{gname}` — {ok}/{n}\n")
            print("| clip | kind | kf | result | detail |")
            print("|---|---|--:|---|---|")
            for r in rs:
                detail = "; ".join(r.reasons) if r.reasons else "—"
                print(f"| `{r.clip.id}` | {kind_label(r.clip)} | "
                      f"{total_keyframes(r.clip)} | "
                      f"{status_of(r)} | {detail} |")
            print()
    else:
        for gname in sorted(by_gesture):
            rs = by_gesture[gname]
            ok = sum(1 for r in rs if r.passed and not r.skipped)
            n = sum(1 for r in rs if not r.skipped)
            print(f"\n=== {gname}  {ok}/{n} ===")
            for r in rs:
                kf = total_keyframes(r.clip)
                line = (f"  [{status_of(r)}] {r.clip.id} "
                        f"kind={kind_label(r.clip)} kf={kf}")
                print(line)
                if args.verbose or not r.passed or r.skipped:
                    for reason in r.reasons:
                        print(f"          {reason}")
        print(f"\n=== TOTAL  {pass_count}/{total} judged, {len(skipped)} skipped ===")
        if skipped:
            print("    skipped (no keyframes, not scored): "
                  + ", ".join(r.clip.id for r in skipped))
        if total == 0:
            print("    no keyed clips — nothing was scored (smoke run only)")
        if args.gestures_config:
            print(f"    (gestures-config: {args.gestures_config})")
        print(f"    (tolerance: -{args.tolerance_early_ms}ms / +{args.tolerance_late_ms}ms)")

    return 0 if pass_count == total else 1


if __name__ == "__main__":
    sys.exit(main())
