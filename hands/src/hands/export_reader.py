"""Reading the shell's `frame-export` directory.

The shell renames latest.json into place LAST, after latest.jpg and
latest.depth, so the sidecar is the commit record: poll it, and when its `seq`
changes the image and depth map it describes are already whole on disk. That
is why this watches the JSON rather than the JPEG — watching the image would
race the metadata that gives it meaning.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image

from .geometry import Intrinsics


@dataclass
class ExportedFrame:
    seq: int
    t_ns: int
    rgb: np.ndarray  # (h, w, 3) uint8
    intrinsics: Intrinsics | None
    head_pos: np.ndarray | None  # scene frame, metres
    head_quat: np.ndarray | None  # scene frame, xyzw
    depth: np.ndarray | None  # (h, w) float32 metres, 0 = no reading

    @property
    def width(self) -> int:
        return int(self.rgb.shape[1])

    @property
    def height(self) -> int:
        return int(self.rgb.shape[0])

    def ready_for_metric_3d(self) -> bool:
        """Everything needed to put a landmark in the scene frame in metres."""
        return (
            self.intrinsics is not None
            and self.intrinsics.valid()
            and self.head_pos is not None
            and self.depth is not None
        )


class ExportReader:
    """Polls one `frame-export` directory and yields each new frame once."""

    def __init__(self, directory: str | Path):
        self.dir = Path(directory)
        self.last_seq = -1

    def read(self) -> ExportedFrame | None:
        """The newest frame, or None when nothing new (or nothing yet)."""
        sidecar = self.dir / "latest.json"
        try:
            meta = json.loads(sidecar.read_text())
        except (OSError, ValueError):
            return None

        seq = int(meta.get("seq", -1))
        if seq == self.last_seq:
            return None

        try:
            with Image.open(self.dir / meta["image"]["file"]) as im:
                rgb = np.asarray(im.convert("RGB"))
        except (OSError, KeyError, ValueError):
            return None

        depth = None
        dmeta = meta.get("depth")
        if dmeta:
            try:
                raw = (self.dir / dmeta["file"]).read_bytes()
                want = int(dmeta["width"]) * int(dmeta["height"])
                flat = np.frombuffer(raw, dtype=np.float32)
                if flat.size == want:
                    depth = flat.reshape(int(dmeta["height"]), int(dmeta["width"]))
            except (OSError, KeyError, ValueError):
                depth = None

        head = meta.get("head")
        head_pos = np.array(head["pos"], dtype=np.float64) if head else None
        head_quat = np.array(head["quat"], dtype=np.float64) if head else None

        intr = meta.get("intrinsics")
        self.last_seq = seq
        return ExportedFrame(
            seq=seq,
            t_ns=int(meta.get("t_ns", 0)),
            rgb=rgb,
            intrinsics=Intrinsics.from_json(intr) if intr else None,
            head_pos=head_pos,
            head_quat=head_quat,
            depth=depth,
        )

    def wait(self, timeout_s: float = 5.0, poll_s: float = 0.005):
        """Block for the next new frame. None on timeout."""
        deadline = time.monotonic() + timeout_s
        while True:
            frame = self.read()
            if frame is not None:
                return frame
            if time.monotonic() >= deadline:
                return None
            time.sleep(poll_s)
