"""Geometry vs pixels-only: run the fixed request set in both conditions and measure where panels land."""
from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path

from . import perception, spatial
from .actions import Executor
from .brain import Brain
from .shell import Shell

REQUESTS = Path(__file__).resolve().parents[2] / "eval" / "requests.json"
RESULTS_DIR = Path(__file__).resolve().parents[2] / "eval" / "results"


@dataclass
class Trial:
    id: int
    condition: str
    text: str
    expect: str
    landed_kind: str | None
    surface_error_m: float | None
    correct_surface: bool
    latency_s: float
    actions: list[str]
    # Object-anchored scoring (ids 26-30, expect: "object" with an `anchor` label).
    # None for every legacy request, so existing result JSON rows are unchanged apart
    # from these four trailing keys being added.
    anchor: str | None = None
    anchor_found: bool | None = None
    anchor_distance_m: float | None = None
    reason: str | None = None


SURFACE_SLACK_M = 0.25
# How close a placed panel must sit to the named anchor object's 3D position to count
# as "next to" it. 0.30 m matches the people-bubble offset and a reasonable arm's reach.
ANCHOR_NEAR_M = 0.30


def _nearest_surface(pos: spatial.Vec3, planes: list[dict], head: dict) -> tuple[str | None, float | None]:
    """Kind and off-surface distance of the plane the point sits over (projection inside its extent)."""
    best: tuple[str, float] | None = None
    for p in planes:
        n = p["normal"]
        c = p["center"]
        rel = spatial.sub(pos, c)
        d = abs(spatial.dot(n, rel))
        in_plane = spatial.sub(rel, spatial.scale(n, spatial.dot(n, rel)))
        if spatial.dist(in_plane, (0, 0, 0)) > max(p["extent"]) / 2 + SURFACE_SLACK_M:
            continue
        if best is None or d < best[1]:
            best = (spatial.plane_kind(p, head), d)
    return best if best else (None, None)


def _expected_kind(expect: str, head: dict, planes: list[dict]) -> str | None:
    if expect in ("table", "wall"):
        return expect
    if expect == "gaze":
        hit = spatial.gaze_hit(head, planes, {"wall", "table", "floor"})
        if hit is None:
            return None
        return _nearest_surface(hit, planes, head)[0]
    return None


def _anchor_match(anchor: str, objects_snapshot: list[dict]) -> list[dict]:
    """Objects in the snapshot whose label matches `anchor`, mirroring objects.nearest's substring rule."""
    want = anchor.lower().strip()
    return [
        o for o in objects_snapshot
        if want in o["label"].lower() or o["label"].lower() in want
    ]


def _score_object_anchor(
    anchor: str,
    panel_pos: spatial.Vec3,
    objects_snapshot: list[dict],
) -> tuple[bool, bool, float | None, str | None]:
    """Score an object-anchored placement by Euclidean distance to the anchor's 3D position.

    Returns (correct_surface, anchor_found, anchor_distance_m, reason). When the anchor is
    absent from the snapshot this is the honest image / remembered-place fallback the README
    promises to report: a miss with reason "anchor_not_found".
    """
    hits = _anchor_match(anchor, objects_snapshot)
    if not hits:
        return False, False, None, "anchor_not_found"
    obj = min(hits, key=lambda o: spatial.dist(o["position_m"], panel_pos))
    d = spatial.dist(obj["position_m"], panel_pos)
    return d <= ANCHOR_NEAR_M, True, round(d, 3), None


def _score(
    req: dict,
    windows: list[dict],
    planes: list[dict],
    head: dict,
    objects_snapshot: list[dict],
) -> dict:
    """Pure scoring for one request. Legacy entries (table/wall/gaze/front) score exactly as
    before; object-anchored entries with an `anchor` field are scored by distance to the anchor.

    Returns the mutable Trial fields so run() can build a Trial without re-deriving them.
    """
    landed_kind, err = (None, None)
    if windows:
        landed_kind, err = _nearest_surface(tuple(windows[-1]["pos"]), planes, head)
    surface_error_m = None if err is None else round(err, 3)

    anchor = req.get("anchor") if req.get("expect") == "object" else None
    anchor_found: bool | None = None
    anchor_distance_m: float | None = None
    reason: str | None = None

    if req.get("expect") == "object" and anchor:
        if windows:
            panel_pos = tuple(windows[-1]["pos"])
            ok, anchor_found, anchor_distance_m, reason = _score_object_anchor(
                anchor, panel_pos, objects_snapshot
            )
            correct_surface = ok
        else:
            correct_surface = False
            anchor_found = False
            reason = "no_panel"
    else:
        expected = _expected_kind(req["expect"], head, planes)
        correct_surface = (expected is None and bool(windows)) or landed_kind == expected

    return {
        "landed_kind": landed_kind,
        "surface_error_m": surface_error_m,
        "correct_surface": correct_surface,
        "anchor": anchor,
        "anchor_found": anchor_found,
        "anchor_distance_m": anchor_distance_m,
        "reason": reason,
    }


def run(condition: str, limit: int | None = None) -> list[Trial]:
    shell = Shell()
    brain = Brain(geometry=(condition == "geometry"))
    requests = json.loads(REQUESTS.read_text())[:limit]
    trials: list[Trial] = []
    for req in requests:
        for w in shell.windows():
            shell.close_window(w["handle"])
        t0 = time.time()
        snap = perception.take(shell)
        calls, _ = brain.decide(req["text"], snap)
        outcome = Executor(shell, snap).run(calls)
        time.sleep(0.4)
        latency = time.time() - t0
        windows = shell.windows()
        scored = _score(req, windows, snap.planes, snap.head, snap.objects())
        trials.append(
            Trial(
                id=req["id"],
                condition=condition,
                text=req["text"],
                expect=req["expect"],
                landed_kind=scored["landed_kind"],
                surface_error_m=scored["surface_error_m"],
                correct_surface=scored["correct_surface"],
                latency_s=round(latency, 2),
                actions=outcome.log,
                anchor=scored["anchor"],
                anchor_found=scored["anchor_found"],
                anchor_distance_m=scored["anchor_distance_m"],
                reason=scored["reason"],
            )
        )
        print(f"[{condition}] #{req['id']:2d} {req['expect']:5s} -> {scored['landed_kind']} err={scored['surface_error_m']} {latency:.1f}s")
    RESULTS_DIR.mkdir(exist_ok=True)
    out = RESULTS_DIR / f"{condition}-{int(time.time())}.json"
    out.write_text(json.dumps([asdict(t) for t in trials], indent=1))
    print("wrote", out)
    return trials


def summarize(trials: list[Trial]) -> dict:
    n = len(trials)
    errs = [t.surface_error_m for t in trials if t.surface_error_m is not None]
    return {
        "n": n,
        "surface_accuracy": round(sum(t.correct_surface for t in trials) / n, 2) if n else None,
        "median_surface_error_m": round(sorted(errs)[len(errs) // 2], 3) if errs else None,
        "mean_latency_s": round(sum(t.latency_s for t in trials) / n, 2) if n else None,
    }
