"""Reading the shell's `frame-export` directory.

The shell renames latest.json into place LAST, after the image and the depth
map, so the sidecar is the commit record: poll it, and when its `seq` changes
the image and depth map it describes are already whole on disk. That is why
this watches the JSON rather than the image — watching the image would race
the metadata that gives it meaning.

Polling is on `os.stat`, not on the file's contents: at 30 Hz a re-read and
re-parse of the sidecar every couple of milliseconds is most of a core spent
learning nothing. The mtime changes when the rename lands, and only then is
the file opened. The sleep between stats is short on purpose — it is pure
added latency on every gesture, and a stat of a file in the page cache costs
a few microseconds.

Two image formats. `format: "jpeg"` is the original `latest.jpg`, which has to
be decoded. `format: "rgb8"` is `latest.rgb`: width x height x 3 uint8, no
header, already downscaled by the shell, which is the point — the shell was
encoding a JPEG out of pixels it already had so that this process could decode
them again.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image

from .geometry import Intrinsics

# Between stats of the sidecar. Small enough to be a rounding error against a
# 33 ms frame interval, large enough not to spin a core.
POLL_S = 0.002


@dataclass
class ExportedFrame:
    seq: int
    t_ns: int  # the phone's capture clock
    export_ns: int  # this Mac's realtime clock when the shell published it
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


def read_raw_rgb(path: Path | str, width: int, height: int) -> np.ndarray | None:
    """`latest.rgb` -> an (h, w, 3) uint8 array, or None if it is the wrong size.

    A short read is a half-written file, which the rename is supposed to make
    impossible; refusing it is cheaper than handing the detector a frame whose
    bottom rows are last frame's.
    """
    try:
        raw = np.fromfile(path, dtype=np.uint8)
    except OSError:
        return None
    if raw.size != width * height * 3:
        return None
    return raw.reshape(height, width, 3)


class ExportReader:
    """Polls one `frame-export` directory and yields each new frame once."""

    def __init__(self, directory: str | Path):
        self.dir = Path(directory)
        self.sidecar = self.dir / "latest.json"
        self.last_seq = -1
        self._last_mtime_ns = -1

    def read(self) -> ExportedFrame | None:
        """The newest frame, or None when nothing new (or nothing yet)."""
        try:
            mtime_ns = os.stat(self.sidecar).st_mtime_ns
        except OSError:
            return None
        if mtime_ns == self._last_mtime_ns:
            return None
        self._last_mtime_ns = mtime_ns

        try:
            meta = json.loads(self.sidecar.read_text())
        except (OSError, ValueError):
            return None

        seq = int(meta.get("seq", -1))
        if seq == self.last_seq:
            return None

        rgb = self._read_image(meta.get("image") or {})
        if rgb is None:
            return None

        head = meta.get("head")
        intr = meta.get("intrinsics")
        self.last_seq = seq
        return ExportedFrame(
            seq=seq,
            t_ns=int(meta.get("t_ns", 0)),
            export_ns=int(meta.get("export_ns", 0)),
            rgb=rgb,
            intrinsics=Intrinsics.from_json(intr) if intr else None,
            head_pos=np.array(head["pos"], dtype=np.float64) if head else None,
            head_quat=np.array(head["quat"], dtype=np.float64) if head else None,
            depth=self._read_depth(meta.get("depth")),
        )

    def _read_image(self, image: dict) -> np.ndarray | None:
        try:
            path = self.dir / image["file"]
            # A sidecar written before raw mode existed carries no `format`,
            # and the only thing it could have been is the JPEG.
            if image.get("format", "jpeg") == "rgb8":
                return read_raw_rgb(path, int(image["width"]), int(image["height"]))
            with Image.open(path) as im:
                return np.asarray(im.convert("RGB"))
        except (OSError, KeyError, ValueError):
            return None

    def _read_depth(self, dmeta: dict | None) -> np.ndarray | None:
        if not dmeta:
            return None
        try:
            raw = (self.dir / dmeta["file"]).read_bytes()
            want = int(dmeta["width"]) * int(dmeta["height"])
            flat = np.frombuffer(raw, dtype=np.float32)
            if flat.size != want:
                return None
            return flat.reshape(int(dmeta["height"]), int(dmeta["width"]))
        except (OSError, KeyError, ValueError):
            return None

    def wait(self, timeout_s: float = 5.0, poll_s: float = POLL_S):
        """Block for the next new frame. None on timeout."""
        deadline = time.monotonic() + timeout_s
        while True:
            frame = self.read()
            if frame is not None:
                return frame
            if time.monotonic() >= deadline:
                return None
            time.sleep(poll_s)
