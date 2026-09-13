"""Spatial memory: named places in the room that survive restarts."""
from __future__ import annotations

import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence

from . import spatial

STORE = Path.home() / ".config" / "tempo" / "memory.json"


@dataclass
class Place:
    label: str
    pos: list[float]
    kind: str
    saved_at: float


class Memory:
    def __init__(self, path: Path = STORE) -> None:
        self.path = path
        self.places: list[Place] = []
        if path.exists():
            self.places = [Place(**p) for p in json.loads(path.read_text())]

    def _save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps([asdict(p) for p in self.places], indent=1))

    def remember(self, label: str, pos: Sequence[float], kind: str = "spot") -> Place:
        self.places = [p for p in self.places if p.label.lower() != label.lower()]
        place = Place(label=label, pos=[round(v, 3) for v in pos], kind=kind, saved_at=time.time())
        self.places.append(place)
        self._save()
        return place

    def find(self, label: str) -> Place | None:
        needle = label.lower().strip()
        for p in self.places:
            if p.label.lower() == needle:
                return p
        for p in self.places:
            if needle in p.label.lower() or p.label.lower() in needle:
                return p
        return None

    def forget(self, label: str) -> bool:
        before = len(self.places)
        self.places = [p for p in self.places if p.label.lower() != label.lower()]
        self._save()
        return len(self.places) < before

    def describe_for(self, head: dict) -> list[dict]:
        return [
            {"label": p.label, "pos": p.pos, "from_you": relative_direction(head, p.pos)}
            for p in self.places
        ]


def relative_direction(head: dict, pos: Sequence[float]) -> str:
    """'1.2 m ahead and to your left, slightly below' style phrase from the wearer's point of view."""
    fwd = spatial.head_forward(head)
    right = spatial.head_right(head)
    rel = spatial.sub(pos, head["scene_pos"])
    ahead = spatial.dot(rel, fwd)
    side = spatial.dot(rel, right)
    up = rel[1]
    d = math.sqrt(sum(c * c for c in rel))
    angle = math.degrees(math.atan2(side, ahead))
    if abs(angle) < 25:
        where = "ahead"
    elif abs(angle) > 155:
        where = "behind you"
    elif angle > 0:
        where = "to your right" if abs(angle) > 60 else "ahead and to your right"
    else:
        where = "to your left" if abs(angle) > 60 else "ahead and to your left"
    vert = ", above you" if up > 0.5 else (", down low" if up < -0.5 else "")
    return f"{d:.1f} m {where}{vert}"
