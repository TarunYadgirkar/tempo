"""MediaPipe Hand Landmarker, behind the backend seam.

This is the original path, moved out of `tracker.py` unchanged in behaviour:
VIDEO mode, two hands, its own handedness labels and its own metric `world`
skeleton. It is kept as `--backend mediapipe` so the rtmpose numbers have
something to be compared against on the same frames.
"""

from __future__ import annotations

from pathlib import Path

import mediapipe as mp
import numpy as np
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

from . import DEFAULT_MODEL, Detection


class MediaPipeBackend:
    name = "mediapipe"

    def __init__(
        self,
        model_path: str | Path = DEFAULT_MODEL,
        num_hands: int = 2,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
        flip_handedness: bool = True,
    ):
        model_path = Path(model_path)
        if not model_path.is_file():
            raise FileNotFoundError(
                f"{model_path} is missing — run hands/scripts/fetch-model.sh"
            )
        options = mp_vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(model_path)),
            running_mode=mp_vision.RunningMode.VIDEO,
            num_hands=num_hands,
            min_hand_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._landmarker = mp_vision.HandLandmarker.create_from_options(options)
        self._flip = flip_handedness
        self._last_ts_ms = -1

    def close(self) -> None:
        self._landmarker.close()

    def detect(self, rgb: np.ndarray, t_ns: int) -> list[Detection]:
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        # VIDEO mode insists on a strictly increasing millisecond clock, and
        # the shell's frame timestamps come off the phone's wall clock, so a
        # replay loop or a stalled camera can hand us the same stamp twice.
        ts_ms = max(t_ns // 1_000_000, self._last_ts_ms + 1)
        self._last_ts_ms = ts_ms
        result = self._landmarker.detect_for_video(image, ts_ms)

        h, w = rgb.shape[0], rgb.shape[1]
        out: list[Detection] = []
        for i, landmarks in enumerate(result.hand_landmarks):
            px = np.array(
                [[lm.x * w, lm.y * h] for lm in landmarks], dtype=np.float64
            )
            world = None
            if result.hand_world_landmarks:
                world = np.array(
                    [[lm.x, lm.y, lm.z] for lm in result.hand_world_landmarks[i]],
                    dtype=np.float64,
                )
            out.append(
                Detection(
                    landmarks_px=px,
                    confidence=self._confidence(result.handedness, i),
                    world=world,
                    chirality=self._chirality(result.handedness, i),
                )
            )
        return out

    def _chirality(self, handedness, i: int) -> str:
        label = "right"
        if handedness and i < len(handedness) and handedness[i]:
            label = handedness[i][0].category_name.lower()
        # MediaPipe labels handedness as if it were looking at a mirror, which
        # is right for a selfie camera and backwards for the phone's rear
        # camera streaming this rig.
        if self._flip:
            label = "left" if label == "right" else "right"
        return label

    @staticmethod
    def _confidence(handedness, i: int) -> float:
        if handedness and i < len(handedness) and handedness[i]:
            return float(handedness[i][0].score)
        return 1.0
