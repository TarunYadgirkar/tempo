"""Open-vocabulary object detection, lifted into the scene frame and remembered across frames.

Why OWL-ViT / OWLv2 over YOLO-World: YOLO-World's open vocabulary needs the OpenAI CLIP text
encoder, which ultralytics pulls at runtime from a git URL (`git+.../ultralytics/CLIP.git`) and
which is not installable in this uv venv, so a custom prompt list cannot be set at all. The OWL
family ships its own text tower in the checkpoint, so any prompt list works offline once the
weights are cached. Measured on this M5 Pro over MPS, 24 prompts, 2560x1600 frame:

    google/owlv2-base-patch16-ensemble   ~350 ms/frame   better labels, misses less
    google/owlvit-base-patch32            ~50 ms/frame   hits the <300 ms budget with room spare

OWLv2 is the default because placement accuracy is the point; `--backend owlvit` is the fast path
for the live per-frame loop. First call downloads weights to the HF cache under ~/.cache.
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Sequence

from PIL import Image

from . import spatial

DEFAULT_PROMPTS = [
    "lamp", "monitor", "laptop", "keyboard", "mouse", "phone", "bottle", "cup", "mug", "book",
    "chair", "desk", "bed", "pillow", "plant", "poster", "whiteboard", "door", "window",
    "backpack", "headphones", "speaker", "clock", "shelf",
]

BACKENDS = {
    "owlv2": "google/owlv2-base-patch16-ensemble",
    "owlvit": "google/owlvit-base-patch32",
}
DEFAULT_BACKEND = os.environ.get("TEMPO_DETECT_BACKEND", "owlv2")
DEFAULT_CONF = 0.3

STORE = Path(os.environ.get("TEMPO_OBJECT_STORE", Path.home() / ".config" / "tempo" / "objects.json"))

MATCH_RADIUS_M = 0.4
SMOOTHING = 0.4          # weight on the new observation
UNSEEN_DROP_S = 5.0      # an unconfirmed track is dropped this long after its last sighting
CONFIRM_HITS = 2         # seen this many times and it becomes room memory, kept and persisted
DEPTH_PATCH = 7          # median over a 7x7 patch at the box centre
NEAR_OBJECT_M = 0.25     # how far toward the viewer a panel sits from the object it labels

# Fallback only, used when the frame carries no depth map: rough real-world heights in metres.
# distance = fy * height_m / box_height_px. Good to maybe 30% on a well-framed object, worthless on
# a cropped one, so every position derived this way is tagged source="prior".
HEIGHT_PRIOR_M = {
    "lamp": 0.45, "monitor": 0.35, "laptop": 0.24, "keyboard": 0.02, "mouse": 0.04, "phone": 0.15,
    "bottle": 0.25, "cup": 0.10, "mug": 0.10, "book": 0.24, "chair": 0.95, "desk": 0.74,
    "bed": 0.55, "pillow": 0.35, "plant": 0.45, "poster": 0.60, "whiteboard": 1.20, "door": 2.00,
    "window": 1.20, "backpack": 0.45, "headphones": 0.20, "speaker": 0.25, "clock": 0.25,
    "shelf": 1.00,
}
DEFAULT_HEIGHT_M = 0.30


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    box: tuple[float, float, float, float]  # x0, y0, x1, y1 in pixels of the frame as given


@dataclass
class Track:
    label: str
    confidence: float
    position_m: list[float]
    size_m_estimate: float
    source: str
    first_seen: float
    last_seen: float
    hits: int = 1

    @property
    def confirmed(self) -> bool:
        return self.hits >= CONFIRM_HITS

    def public(self, now: float) -> dict[str, Any]:
        return {
            "label": self.label,
            "confidence": round(self.confidence, 3),
            "position_m": [round(v, 3) for v in self.position_m],
            "size_m_estimate": round(self.size_m_estimate, 2),
            "source": self.source,
            "unseen_s": round(now - self.last_seen, 1),
        }


class Detector:
    """One cached OWL model. Text embeddings are recomputed per call, which is cheap at 24 prompts."""

    _cache: dict[str, "Detector"] = {}

    def __init__(self, backend: str = DEFAULT_BACKEND, prompts: Sequence[str] | None = None) -> None:
        import torch
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

        self.backend = backend
        self.model_id = BACKENDS.get(backend, backend)
        self.prompts = list(prompts or DEFAULT_PROMPTS)
        self.device = "mps" if torch.backends.mps.is_available() else "cpu"
        self.processor = AutoProcessor.from_pretrained(self.model_id)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(self.model_id).to(self.device).eval()
        self._torch = torch

    @classmethod
    def shared(cls, backend: str = DEFAULT_BACKEND, prompts: Sequence[str] | None = None) -> "Detector":
        key = f"{backend}:{','.join(prompts or DEFAULT_PROMPTS)}"
        if key not in cls._cache:
            cls._cache[key] = cls(backend, prompts)
        return cls._cache[key]

    def detect(self, image: Image.Image, conf: float = DEFAULT_CONF) -> tuple[list[Detection], float]:
        torch = self._torch
        queries = [[f"a photo of a {p}" for p in self.prompts]]
        t0 = time.perf_counter()
        inputs = self.processor(text=queries, images=image, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)
        if self.device == "mps":
            torch.mps.synchronize()
        result = self.processor.post_process_grounded_object_detection(
            outputs, threshold=conf, target_sizes=[(image.height, image.width)]
        )[0]
        ms = (time.perf_counter() - t0) * 1000
        dets = [
            Detection(self.prompts[int(label)], float(score), tuple(float(v) for v in box))
            for score, label, box in zip(result["scores"].tolist(), result["labels"].tolist(), result["boxes"].tolist())
        ]
        return sorted(dets, key=lambda d: -d.confidence), ms


class DepthMap:
    """Raw float32 metre depths the compositor exports alongside the frame."""

    def __init__(self, path: str | Path, width: int, height: int) -> None:
        import numpy as np

        self.width = int(width)
        self.height = int(height)
        self.data = np.fromfile(str(path), dtype=np.float32).reshape(self.height, self.width)

    @classmethod
    def load(cls, meta: dict[str, Any] | None, base: Path) -> "DepthMap | None":
        if not meta:
            return None
        path = Path(meta["path"])
        if not path.is_absolute():
            path = base / path
        if not path.exists():
            return None
        return cls(path, meta["width"], meta["height"])

    def sample(self, u: float, v: float, frame_w: int, frame_h: int) -> float | None:
        """Median of a DEPTH_PATCH square around (u, v), invalid samples skipped. Metres, or None."""
        import numpy as np

        cu = int(round(u * self.width / frame_w))
        cv = int(round(v * self.height / frame_h))
        r = DEPTH_PATCH // 2
        patch = self.data[
            max(cv - r, 0) : min(cv + r + 1, self.height),
            max(cu - r, 0) : min(cu + r + 1, self.width),
        ]
        good = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 12.0)]
        return float(np.median(good)) if good.size else None


def unproject(u: float, v: float, depth_m: float, intr: dict[str, Any]) -> spatial.Vec3:
    """Pixel plus depth to camera space: +x right, +y up, -z forward, matching ARKit."""
    x = (u - intr["cx"]) / intr["fx"] * depth_m
    y = -(v - intr["cy"]) / intr["fy"] * depth_m
    return (x, y, -depth_m)


def to_scene(point: Sequence[float], head: dict[str, Any]) -> spatial.Vec3:
    rot = head.get("quat") or head.get("scene_rot")
    pos = head.get("pos") or head.get("scene_pos")
    return spatial.add(pos, spatial.rotate(rot, point))


def locate(det: Detection, head: dict, intr: dict, depth: DepthMap | None, frame: tuple[int, int]) -> dict[str, Any]:
    """One detection to a scene-frame position, from depth when there is depth and a prior when not."""
    x0, y0, x1, y1 = det.box
    u, v = (x0 + x1) / 2, (y0 + y1) / 2
    box_h = max(y1 - y0, 1.0)
    scale = intr["width"] / frame[0]
    d = depth.sample(u, v, *frame) if depth else None
    source = "depth"
    if d is None:
        d = intr["fy"] * HEIGHT_PRIOR_M.get(det.label, DEFAULT_HEIGHT_M) / (box_h * scale)
        source = "prior"
    cam = unproject(u * scale, v * scale, d, intr)
    size = max(x1 - x0, box_h) * scale / intr["fx"] * d
    return {
        "label": det.label,
        "confidence": det.confidence,
        "position_m": list(to_scene(cam, head)),
        "size_m_estimate": size,
        "source": source,
    }


class ObjectMap:
    """Label-matched tracks with exponential smoothing, persisted so the room survives a restart.

    An unconfirmed track (seen once) is dropped UNSEEN_DROP_S after its last sighting. A track seen
    CONFIRM_HITS times is kept as room memory and written to disk: the lamp does not stop existing
    because the wearer turned their head. `unseen_s` in the snapshot says how stale each one is.
    """

    def __init__(self, store: Path = STORE) -> None:
        self.store = store
        self.tracks: list[Track] = []
        self._load()

    def _load(self) -> None:
        if not self.store.exists():
            return
        raw = json.loads(self.store.read_text())
        self.tracks = [Track(**t) for t in raw.get("tracks", [])]

    def save(self) -> None:
        self.store.parent.mkdir(parents=True, exist_ok=True)
        payload = {"saved_at": time.time(), "tracks": [vars(t) for t in self.tracks if t.confirmed]}
        tmp = self.store.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(payload, indent=1))
        tmp.replace(self.store)

    def update(self, located: Iterable[dict[str, Any]], now: float | None = None) -> list[Track]:
        now = now or time.time()
        fresh: list[Track] = []
        for obs in located:
            match = self._match(obs)
            if match is None:
                match = Track(
                    label=obs["label"],
                    confidence=obs["confidence"],
                    position_m=list(obs["position_m"]),
                    size_m_estimate=obs["size_m_estimate"],
                    source=obs["source"],
                    first_seen=now,
                    last_seen=now,
                )
                self.tracks.append(match)
            else:
                self._blend(match, obs, now)
            fresh.append(match)
        self._expire(now)
        self.save()
        return fresh

    def _match(self, obs: dict[str, Any]) -> Track | None:
        best, best_d = None, MATCH_RADIUS_M
        for t in self.tracks:
            if t.label != obs["label"]:
                continue
            d = spatial.dist(t.position_m, obs["position_m"])
            if d < best_d:
                best, best_d = t, d
        return best

    @staticmethod
    def _blend(track: Track, obs: dict[str, Any], now: float) -> None:
        a = SMOOTHING
        track.position_m = [(1 - a) * old + a * new for old, new in zip(track.position_m, obs["position_m"])]
        track.size_m_estimate = (1 - a) * track.size_m_estimate + a * obs["size_m_estimate"]
        track.confidence = max(track.confidence, obs["confidence"])
        track.source = obs["source"]
        track.last_seen = now
        track.hits += 1

    def _expire(self, now: float) -> None:
        self.tracks = [t for t in self.tracks if t.confirmed or now - t.last_seen < UNSEEN_DROP_S]

    def snapshot(self, now: float | None = None) -> list[dict[str, Any]]:
        now = now or time.time()
        return [t.public(now) for t in sorted(self.tracks, key=lambda t: t.last_seen, reverse=True)]

    def nearest(self, label: str, to: Sequence[float] | None = None) -> dict[str, Any] | None:
        want = label.lower().strip()
        hits = [t for t in self.tracks if want in t.label or t.label in want]
        if not hits:
            return None
        if to is None:
            return max(hits, key=lambda t: (t.last_seen, t.confidence)).public(time.time())
        return min(hits, key=lambda t: spatial.dist(t.position_m, to)).public(time.time())


_MAP: ObjectMap | None = None


def store() -> ObjectMap:
    global _MAP
    if _MAP is None:
        _MAP = ObjectMap()
    return _MAP


def snapshot() -> list[dict[str, Any]]:
    """Every object the room currently knows about, freshest first."""
    return store().snapshot()


def nearest(label: str, to: Sequence[float] | None = None) -> dict[str, Any] | None:
    """The tracked object best matching `label`, closest to `to` when a point is given."""
    return store().nearest(label, to)


# --- frame sources -------------------------------------------------------------------

FALLBACK_INTRINSICS_FOV_DEG = 60.0


def _intrinsics_for(image: Image.Image, meta: dict[str, Any] | None) -> dict[str, Any]:
    if meta and meta.get("intrinsics"):
        return meta["intrinsics"]
    import math

    f = image.width / (2 * math.tan(math.radians(FALLBACK_INTRINSICS_FOV_DEG) / 2))
    return {"fx": f, "fy": f, "cx": image.width / 2, "cy": image.height / 2,
            "width": image.width, "height": image.height}


IDENTITY_HEAD = {"pos": [0.0, 0.0, 0.0], "quat": [0.0, 0.0, 0.0, 1.0]}


def process_frame(
    image_path: str | Path,
    meta: dict[str, Any] | None = None,
    base: Path | None = None,
    backend: str = DEFAULT_BACKEND,
    prompts: Sequence[str] | None = None,
    conf: float = DEFAULT_CONF,
) -> tuple[list[dict[str, Any]], float]:
    """Detect, place in the scene frame, fold into the map. Returns what was seen and the inference ms."""
    image_path = Path(image_path)
    base = base or image_path.parent
    image = Image.open(image_path).convert("RGB")
    dets, ms = Detector.shared(backend, prompts).detect(image, conf)
    head = (meta or {}).get("head") or IDENTITY_HEAD
    intr = _intrinsics_for(image, meta)
    depth = DepthMap.load((meta or {}).get("depth"), base)
    located = [locate(d, head, intr, depth, (image.width, image.height)) for d in dets]
    store().update(located)
    return located, ms


def read_export(frames_dir: str | Path) -> tuple[Path, dict[str, Any]] | None:
    """The shell's frame export: latest.jpg plus latest.json. None until the shell has written one."""
    d = Path(frames_dir)
    img, meta = d / "latest.jpg", d / "latest.json"
    if not img.exists():
        return None
    return img, (json.loads(meta.read_text()) if meta.exists() else {})


def read_pair(image_path: str | Path, meta_path: str | Path | None = None) -> tuple[Path, dict[str, Any]]:
    """A plain image with an optional sidecar json, for testing before the export exists."""
    image_path = Path(image_path)
    if meta_path is None:
        guess = image_path.with_suffix(".json")
        meta_path = guess if guess.exists() else None
    return image_path, (json.loads(Path(meta_path).read_text()) if meta_path else {})
