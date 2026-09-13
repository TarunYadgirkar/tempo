"""Faces on the exported frames: YuNet detector + ArcFace embeddings.

Lifted from FaceLock (Tarun's webcam presence lock): the SFace file is loaded
only for its 5-point aligner, which produces the exact 112 px template the
ArcFace MobileFaceNet (w600k_mbf) expects.
"""
from __future__ import annotations

import io
import os
import shutil
import urllib.request
import zipfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

MODELS_DIR = Path(os.environ.get("TEMPO_MODELS_DIR") or Path(__file__).resolve().parents[2] / "models")
FACELOCK_MODELS = Path.home() / "TarunsCode" / "facelock" / "models"
OPENCV_ZOO = "https://github.com/opencv/opencv_zoo/raw/main/models"
INSIGHTFACE_RELEASE = "https://github.com/deepinsight/insightface/releases/download/v0.7"
MODELS = {
    "face_detection_yunet_2023mar.onnx": (f"{OPENCV_ZOO}/face_detection_yunet/face_detection_yunet_2023mar.onnx", None),
    "face_recognition_sface_2021dec.onnx": (f"{OPENCV_ZOO}/face_recognition_sface/face_recognition_sface_2021dec.onnx", None),
    "w600k_mbf.onnx": (f"{INSIGHTFACE_RELEASE}/buffalo_sc.zip", "w600k_mbf.onnx"),
}
DETECTOR = MODELS_DIR / "face_detection_yunet_2023mar.onnx"
ALIGNER = MODELS_DIR / "face_recognition_sface_2021dec.onnx"
ARCFACE = MODELS_DIR / "w600k_mbf.onnx"

DETECT_WIDTH = 640
DETECT_SCORE = 0.7
ARCFACE_INPUT = 112
ARCFACE_SCALE = 1 / 127.5
ARCFACE_MEAN = (127.5, 127.5, 127.5)
# FaceLock's calibrated ArcFace threshold: strangers top out near 0.30 on a 4000-face bank.
FACE_MATCH = 0.42


def ensure_models(log=print) -> None:
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    for name, (url, member) in MODELS.items():
        target = MODELS_DIR / name
        if target.exists():
            continue
        local = FACELOCK_MODELS / name
        if local.exists():
            shutil.copy(local, target)
            continue
        log(f"faces: downloading {name}")
        with urllib.request.urlopen(url) as r:
            payload = r.read()
        if member:
            with zipfile.ZipFile(io.BytesIO(payload)) as z:
                payload = z.read(member)
        part = target.with_suffix(".part")
        part.write_bytes(payload)
        part.rename(target)


@dataclass(frozen=True)
class Face:
    box: tuple[float, float, float, float]  # x, y, w, h in frame pixels
    score: float
    embedding: np.ndarray  # (512,) L2-normalized

    @property
    def center(self) -> tuple[float, float]:
        x, y, w, h = self.box
        return (x + w / 2, y + h / 2)

    @property
    def area(self) -> float:
        return self.box[2] * self.box[3]


class FaceEngine:
    def __init__(self) -> None:
        import cv2

        ensure_models()
        self._cv2 = cv2
        self._detector = cv2.FaceDetectorYN.create(str(DETECTOR), "", (DETECT_WIDTH, DETECT_WIDTH), DETECT_SCORE, 0.3, 5000)
        self._aligner = cv2.FaceRecognizerSF.create(str(ALIGNER), "")
        self._arcface = cv2.dnn.readNetFromONNX(str(ARCFACE))
        self._input_size: tuple[int, int] | None = None

    def _detect(self, bgr: np.ndarray) -> np.ndarray:
        cv2 = self._cv2
        height, width = bgr.shape[:2]
        scale = min(1.0, DETECT_WIDTH / width)
        small = cv2.resize(bgr, (int(width * scale), int(height * scale))) if scale < 1.0 else bgr
        size = (small.shape[1], small.shape[0])
        if size != self._input_size:
            self._detector.setInputSize(size)
            self._input_size = size
        _, faces = self._detector.detect(small)
        if faces is None:
            return np.empty((0, 15), dtype=np.float32)
        faces = np.asarray(faces, dtype=np.float32).copy()
        if scale < 1.0:
            faces[:, :14] /= scale
        return faces

    def _embed(self, bgr: np.ndarray, face_row: np.ndarray) -> np.ndarray | None:
        cv2 = self._cv2
        aligned = self._aligner.alignCrop(bgr, face_row)
        blob = cv2.dnn.blobFromImage(aligned, ARCFACE_SCALE, (ARCFACE_INPUT, ARCFACE_INPUT), ARCFACE_MEAN, swapRB=True)
        self._arcface.setInput(blob)
        feature = self._arcface.forward().flatten().astype(np.float32)
        norm = float(np.linalg.norm(feature))
        return feature / norm if norm > 0 else None

    def faces(self, rgb: np.ndarray) -> list[Face]:
        """Every face in an RGB frame, largest first."""
        bgr = self._cv2.cvtColor(np.ascontiguousarray(rgb), self._cv2.COLOR_RGB2BGR)
        out: list[Face] = []
        for row in self._detect(bgr):
            emb = self._embed(bgr, row)
            if emb is None:
                continue
            out.append(Face(tuple(float(v) for v in row[:4]), float(row[14]), emb))
        return sorted(out, key=lambda f: -f.area)


def similarity(embedding: np.ndarray, bank: np.ndarray, top_k: int = 3) -> float:
    """Mean of the top-k cosine similarities against a person's stored embeddings."""
    if bank.size == 0:
        return -1.0
    scores = np.sort(bank @ embedding)[::-1]
    return float(scores[: min(top_k, len(scores))].mean())
