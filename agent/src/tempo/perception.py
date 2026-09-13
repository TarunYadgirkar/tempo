"""One snapshot of what the wearer sees and where things are."""
from __future__ import annotations

import io
import json
import time
from dataclasses import dataclass
from typing import Any

from PIL import Image

from . import spatial
from .memory import Memory
from .shell import Shell

VIEW_WIDTH_PX = 1024


@dataclass
class Snapshot:
    png: bytes
    head: dict[str, Any]
    planes: list[dict[str, Any]]
    windows: list[dict[str, Any]]
    aim: dict[str, Any]
    taken_at: float

    def scene_text(self) -> str:
        head_pos = [round(v, 2) for v in self.head["scene_pos"]]
        fwd = [round(v, 2) for v in spatial.head_forward(self.head)]
        panels = [
            {
                "handle": w["handle"],
                "title": w["title"],
                "app": w["app_id"],
                "pos": [round(v, 2) for v in w["pos"]],
                "anchored": bool(w["anchor"]),
                "focused": w["focused"],
            }
            for w in self.windows
        ]
        return json.dumps(
            {
                "head_position_m": head_pos,
                "head_forward": fwd,
                "tracking": self.head["tracking"],
                "surfaces": spatial.describe_planes(self.planes, self.head),
                "panels": panels,
                "remembered_places": Memory().describe_for(self.head),
                "hand": {
                    "visible": self.aim["hands"] > 0,
                    "pointing_at_m": [round(v, 2) for v in self.aim["hit"]] if self.aim.get("hit") else None,
                    "pointing_at_panel": self.aim.get("aimed_handle"),
                    "pinching": self.aim["pinching"],
                },
            },
            indent=1,
        )


def _shrink(path: str) -> bytes:
    img = Image.open(path).convert("RGB")
    bbox = img.getbbox()
    if bbox:
        img = img.crop(bbox)
    if img.width > VIEW_WIDTH_PX:
        img = img.resize((VIEW_WIDTH_PX, round(img.height * VIEW_WIDTH_PX / img.width)))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def take(shell: Shell) -> Snapshot:
    path = shell.screenshot()
    return Snapshot(
        png=_shrink(path),
        head=shell.head_pose(),
        planes=shell.planes(),
        windows=shell.windows(),
        aim=shell.aim(),
        taken_at=time.time(),
    )
