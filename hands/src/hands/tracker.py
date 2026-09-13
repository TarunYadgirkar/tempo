"""MediaPipe Hand Landmarker over the exported camera frames.

One frame in, scene-frame metric joints out:

  1. HandLandmarker (VIDEO mode, 2 hands) finds the 21 landmarks as normalised
     image coordinates, plus a `world` skeleton in metres whose origin is the
     hand's own centre.
  2. Each landmark samples the LiDAR depth map at its pixel — the median of a
     5x5 patch, skipping the 0 "no reading" holes.
  3. A landmark with depth unprojects through the intrinsics into camera
     space. A landmark without takes its RANGE from the world skeleton,
     relative to a landmark that did get depth, and keeps its own (accurate)
     pixel direction. This matters constantly in practice: the LiDAR map is
     32x24-ish, so a fingertip at arm's length covers well under a pixel and
     lands in a hole whenever the hand is not filling the frame.
  4. The head pose the shell stamped on the frame carries the point into the
     scene frame, where the panels are.
  5. One Euro per axis per landmark takes out the residual jitter.

The output is deliberately the same 21 joints in the same order the phone's
0x05 packet carries, so nothing downstream of the scene has to know which
tracker produced them.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mediapipe as mp
import numpy as np
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

from .export_reader import ExportedFrame
from .geometry import INDEX_MCP, WRIST, camera_to_scene, sample_depth, unproject
from .onefilter import JointFilter

DEFAULT_MODEL = Path(__file__).resolve().parents[2] / "models" / "hand_landmarker.task"


@dataclass
class TrackedHand:
    chirality: str  # "left" | "right"
    confidence: float
    joints: np.ndarray  # (21, 3) scene-frame metres, MediaPipe landmark order
    depth_valid: np.ndarray  # (21,) bool — which landmarks got a real reading


@dataclass
class TrackResult:
    hands: list[TrackedHand]
    detected: int  # hands MediaPipe found, before the metric stage could drop any
    depth_valid_fraction: float  # over the fingertips of the hands that survived


class HandTracker:
    def __init__(
        self,
        model_path: str | Path = DEFAULT_MODEL,
        num_hands: int = 2,
        min_detection_confidence: float = 0.5,
        min_tracking_confidence: float = 0.5,
        flip_handedness: bool = True,
        depth_window: int = 5,
        min_cutoff: float = 1.0,
        beta: float = 0.5,
        fallback_depth_m: float = 0.0,
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
        self._window = depth_window
        self._fallback_depth = fallback_depth_m
        self._filters: dict[str, JointFilter] = {}
        self._filter_params = (min_cutoff, beta)
        self._last_ts_ms = -1

    def close(self) -> None:
        self._landmarker.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    # -- the loop body -----------------------------------------------------

    def track(self, frame: ExportedFrame) -> TrackResult:
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame.rgb)
        # VIDEO mode insists on a strictly increasing millisecond clock, and
        # the shell's frame timestamps come off the phone's wall clock, so a
        # replay loop or a stalled camera can hand us the same stamp twice.
        ts_ms = max(frame.t_ns // 1_000_000, self._last_ts_ms + 1)
        self._last_ts_ms = ts_ms
        result = self._landmarker.detect_for_video(image, ts_ms)

        detected = len(result.hand_landmarks)
        hands: list[TrackedHand] = []
        valid_total = 0
        valid_count = 0
        seen: set[str] = set()

        for i, landmarks in enumerate(result.hand_landmarks):
            chirality = self._chirality(result.handedness, i)
            if chirality in seen:
                # Two detections labelled the same hand: the shell keys its
                # slots on chirality, so the second would overwrite the first.
                continue
            world = result.hand_world_landmarks[i] if result.hand_world_landmarks else None
            hand = self._to_scene(frame, landmarks, world, chirality,
                                  self._confidence(result.handedness, i))
            if hand is None:
                continue
            seen.add(chirality)
            hands.append(hand)
            valid_total += int(hand.depth_valid.size)
            valid_count += int(hand.depth_valid.sum())

        self._reset_missing(seen)

        return TrackResult(
            hands=hands,
            detected=detected,
            depth_valid_fraction=(valid_count / valid_total) if valid_total else 0.0,
        )

    # -- internals ---------------------------------------------------------

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

    def _to_scene(
        self, frame: ExportedFrame, landmarks, world, chirality: str, confidence: float
    ) -> TrackedHand | None:
        if not frame.ready_for_metric_3d():
            return None
        depth_map = frame.depth
        intr = frame.intrinsics

        n = len(landmarks)
        depths = np.zeros(n, dtype=np.float64)
        for j, lm in enumerate(landmarks):
            depths[j] = sample_depth(depth_map, lm.x, lm.y, self._window)
        depth_valid = depths > 0.0

        ref = self._reference(depth_valid)
        if ref is None:
            if self._fallback_depth <= 0.0:
                return None
            depths[:] = self._fallback_depth
        elif world is not None:
            # MediaPipe's world z grows AWAY from the camera in the same sense
            # as the depth map, so a difference in world z is a difference in
            # range. The landmark keeps its own pixel, so only the range is
            # being guessed, never the direction.
            for j in range(n):
                if not depth_valid[j]:
                    depths[j] = max(
                        0.05, depths[ref] + (world[j].z - world[ref].z)
                    )
        else:
            depths[~depth_valid] = depths[ref]

        joints = np.zeros((n, 3), dtype=np.float64)
        for j, lm in enumerate(landmarks):
            p_cam = unproject(
                intr,
                frame.width,
                frame.height,
                lm.x * frame.width,
                lm.y * frame.height,
                depths[j],
            )
            if p_cam is None:
                return None
            joints[j] = camera_to_scene(frame.head_pos, frame.head_quat, p_cam)

        filt = self._filters.get(chirality)
        if filt is None:
            filt = JointFilter(*self._filter_params, joints=n)
            self._filters[chirality] = filt
        joints = filt.apply(joints, frame.t_ns / 1e9)

        return TrackedHand(
            chirality=chirality,
            confidence=confidence,
            joints=joints,
            depth_valid=depth_valid,
        )

    @staticmethod
    def _reference(depth_valid: np.ndarray) -> int | None:
        """The landmark whose real depth anchors the estimated ones.

        Wrist first, then index MCP: both sit on the broad part of the hand,
        which is many depth pixels wide and therefore almost always has a
        reading, unlike a fingertip. Any valid landmark beats none.
        """
        for candidate in (WRIST, INDEX_MCP):
            if depth_valid[candidate]:
                return candidate
        where = np.flatnonzero(depth_valid)
        return int(where[0]) if where.size else None

    def _reset_missing(self, seen: set[str]) -> None:
        """Drop filter state for a hand that left, so its return is not lerped
        in from wherever it was last seen."""
        for chirality, filt in self._filters.items():
            if chirality not in seen:
                filt.reset()
