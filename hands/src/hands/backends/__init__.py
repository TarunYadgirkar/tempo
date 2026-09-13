"""The two hand models, behind one shape.

A backend's whole job is to turn one exported RGB frame into pixel landmarks.
Everything after that — the LiDAR depth sample, the unprojection, the scene
transform, One Euro — is the same code for both and lives in `tracker.py`,
which is the point of the seam: swapping the model must not be able to change
the metric pipeline underneath it.

  mediapipe  Hand Landmarker, the 2020 model. Gives handedness and a metric
             `world` skeleton, which the depth stage uses to estimate the
             range of a landmark that missed the LiDAR.
  rtmpose    RTMDet-nano hand detector + RTMPose-m hand, through rtmlib on
             ONNX Runtime. Sharper landmarks and several times faster, but
             2D only: no world skeleton and no chirality, so the range of a
             missed landmark falls back to the reference landmark's and the
             chirality is inferred from the thumb's side of the palm.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

BACKENDS = ("rtmpose", "mediapipe")
DEFAULT_BACKEND = "rtmpose"

# Named here rather than in mediapipe_backend so `--model` can be parsed and
# defaulted without importing MediaPipe, which is most of a second of import
# time for a run that may never touch it.
DEFAULT_MODEL = Path(__file__).resolve().parents[3] / "models" / "hand_landmarker.task"


@dataclass
class Detection:
    """One hand, as the model saw it — pixels, not metres.

    `landmarks_px` is (21, 2) in the exported image's own pixels, MediaPipe
    landmark order. `world` is (21, 3) metres centred on the hand when the
    model offers one, else None. `chirality` is "left"/"right" when the model
    is sure and None when the caller has to infer it.
    """

    landmarks_px: np.ndarray
    confidence: float
    world: np.ndarray | None = None
    chirality: str | None = None


def make_backend(name: str, **kwargs):
    """Construct a backend by name. Imports lazily, so a run on one backend
    never pays for the other's model runtime (rtmlib pulls in onnxruntime and
    OpenCV; mediapipe pulls in its own graph runtime)."""
    if name == "mediapipe":
        from .mediapipe_backend import MediaPipeBackend

        return MediaPipeBackend(**kwargs)
    if name == "rtmpose":
        from .rtmpose_backend import RTMPoseBackend

        return RTMPoseBackend(**kwargs)
    raise ValueError(f"unknown backend {name!r}: expected one of {BACKENDS}")
