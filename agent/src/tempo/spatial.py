"""Pure math on scene-frame poses. Scene frame: +Y up, metres, ARKit camera looks down -Z."""
from __future__ import annotations

import math
from typing import Sequence

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # xyzw


def rotate(q: Sequence[float], v: Sequence[float]) -> Vec3:
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def normalize(v: Sequence[float]) -> Vec3:
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return (v[0] / n, v[1] / n, v[2] / n)


def add(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(v: Sequence[float], s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)


def dist(a: Sequence[float], b: Sequence[float]) -> float:
    return math.dist(a, b)


def head_forward(head: dict) -> Vec3:
    fwd = rotate(head["scene_rot"], (0.0, 0.0, -1.0))
    return normalize((fwd[0], 0.0, fwd[2])) if abs(fwd[1]) > 0.95 else normalize(fwd)


def head_right(head: dict) -> Vec3:
    return normalize(rotate(head["scene_rot"], (1.0, 0.0, 0.0)))


def in_front(head: dict, distance: float = 0.9, right: float = 0.0, up: float = 0.0) -> Vec3:
    p = add(head["scene_pos"], scale(head_forward(head), distance))
    p = add(p, scale(head_right(head), right))
    return (p[0], p[1] + up, p[2])


def plane_kind(plane: dict, head: dict) -> str:
    ny = plane["normal"][1]
    if ny > 0.8:
        return "floor" if plane["center"][1] < head["scene_pos"][1] - 0.9 else "table"
    if ny < -0.8:
        return "ceiling"
    return "wall"


def describe_planes(planes: list[dict], head: dict) -> list[dict]:
    hp = head["scene_pos"]
    out = []
    for p in planes:
        c = p["center"]
        rel = (c[0] - hp[0], c[1] - hp[1], c[2] - hp[2])
        out.append(
            {
                "uuid": p["uuid"],
                "kind": plane_kind(p, head),
                "center": [round(v, 2) for v in c],
                "size_m": [round(v, 2) for v in p["extent"]],
                "distance_m": round(math.sqrt(sum(r * r for r in rel)), 2),
            }
        )
    return sorted(out, key=lambda d: d["distance_m"])


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def ray_plane_hit(origin: Sequence[float], direction: Sequence[float], plane: dict, slack_m: float = 0.3) -> tuple[float, Vec3] | None:
    """Distance along the ray and the hit point, if the ray meets this plane within its extent (plus slack)."""
    n = plane["normal"]
    denom = dot(n, direction)
    if abs(denom) < 1e-4:
        return None
    c = plane["center"]
    t = dot(n, (c[0] - origin[0], c[1] - origin[1], c[2] - origin[2])) / denom
    if t <= 0.05:
        return None
    hit = add(origin, scale(direction, t))
    reach = max(plane["extent"]) / 2 + slack_m
    if dist(hit, c) > reach:
        return None
    return t, hit


def gaze_hit(head: dict, planes: list[dict], kinds: set[str]) -> Vec3 | None:
    """Where the wearer's forward ray meets the nearest plane of the given kinds, pulled 5 cm off the surface."""
    origin = head["scene_pos"]
    direction = normalize(rotate(head["scene_rot"], (0.0, 0.0, -1.0)))
    best: tuple[float, Vec3, dict] | None = None
    for p in planes:
        if plane_kind(p, head) not in kinds:
            continue
        hit = ray_plane_hit(origin, direction, p)
        if hit and (best is None or hit[0] < best[0]):
            best = (hit[0], hit[1], p)
    if best is None:
        return None
    n = best[2]["normal"]
    toward_viewer = 1.0 if dot(n, (origin[0] - best[1][0], origin[1] - best[1][1], origin[2] - best[1][2])) > 0 else -1.0
    return add(best[1], scale(n, 0.05 * toward_viewer))
