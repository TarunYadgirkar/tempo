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


SURFACE_SLACK_M = 0.25


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
        landed_kind, err = (None, None)
        if windows:
            landed_kind, err = _nearest_surface(tuple(windows[-1]["pos"]), snap.planes, snap.head)
        expected = _expected_kind(req["expect"], snap.head, snap.planes)
        trials.append(
            Trial(
                id=req["id"],
                condition=condition,
                text=req["text"],
                expect=req["expect"],
                landed_kind=landed_kind,
                surface_error_m=None if err is None else round(err, 3),
                correct_surface=(expected is None and bool(windows)) or landed_kind == expected,
                latency_s=round(latency, 2),
                actions=outcome.log,
            )
        )
        print(f"[{condition}] #{req['id']:2d} {req['expect']:5s} -> {landed_kind} err={err} {latency:.1f}s")
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
