"""Pixel landmarks -> scene-frame metric joints.

The model that found the landmarks lives behind `backends/`; everything here
is the same for both of them, which is the point of the split — swapping the
model must not be able to change the metric pipeline underneath it.

  1. A backend finds the 21 landmarks in the exported image's own pixels,
     already in MediaPipe landmark order.
  2. Each landmark samples the LiDAR depth map at its pixel — the median of a
     5x5 patch, skipping the 0 "no reading" holes.
  3. A landmark with depth unprojects through the intrinsics into camera
     space. A landmark without takes its RANGE from a landmark that did get
     depth, and keeps its own (accurate) pixel direction. This matters
     constantly in practice: the LiDAR map is 32x24-ish, so a fingertip at
     arm's length covers well under a pixel and lands in a hole whenever the
     hand is not filling the frame. MediaPipe offers a metric hand skeleton
     that makes the estimate a shaped one; RTMPose does not, and the estimate
     flattens to the reference landmark's own range.
  4. The head pose the shell stamped on the frame carries the point into the
     scene frame, where the panels are.
  5. One Euro per axis per landmark takes out the residual jitter.

The output is deliberately the same 21 joints in the same order the phone's
0x05 packet carries, so nothing downstream of the scene has to know which
tracker produced them.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .backends import (  # noqa: F401 (DEFAULT_MODEL is re-exported for the CLI)
    DEFAULT_BACKEND,
    DEFAULT_MODEL,
    Detection,
    make_backend,
)
from .export_reader import ExportedFrame
from .geometry import (
    INDEX_MCP,
    WRIST,
    camera_to_scene,
    chirality_from_joints,
    chirality_from_pixels,
    sample_depth,
    unproject,
)
from .onefilter import JointFilter


@dataclass
class TrackedHand:
    chirality: str  # "left" | "right"
    confidence: float
    joints: np.ndarray  # (21, 3) scene-frame metres, MediaPipe landmark order
    depth_valid: np.ndarray  # (21,) bool — which landmarks got a real reading


@dataclass
class TrackResult:
    hands: list[TrackedHand]
    detected: int  # hands the model found, before the metric stage could drop any
    depth_valid_fraction: float  # over the fingertips of the hands that survived


class HandTracker:
    def __init__(
        self,
        backend: str = DEFAULT_BACKEND,
        depth_window: int = 5,
        min_cutoff: float = 1.0,
        beta: float = 0.5,
        fallback_depth_m: float = 0.0,
        dorsal_view: bool = True,
        **backend_kwargs,
    ):
        self.backend_name = backend
        self._backend = make_backend(backend, **backend_kwargs)
        self._window = depth_window
        self._fallback_depth = fallback_depth_m
        self._dorsal = dorsal_view
        self._filters: dict[str, JointFilter] = {}
        self._filter_params = (min_cutoff, beta)

    def close(self) -> None:
        self._backend.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    # -- the loop body -----------------------------------------------------

    def track(self, frame: ExportedFrame) -> TrackResult:
        detections = self._backend.detect(frame.rgb, frame.t_ns)

        hands: list[TrackedHand] = []
        valid_total = 0
        valid_count = 0
        seen: set[str] = set()

        for detection in detections:
            hand = self._to_scene(frame, detection)
            if hand is None:
                continue
            if hand.chirality in seen:
                # Two detections of the same hand: the shell keys its slots on
                # chirality, so the second would overwrite the first.
                continue
            seen.add(hand.chirality)
            hands.append(hand)
            valid_total += int(hand.depth_valid.size)
            valid_count += int(hand.depth_valid.sum())

        self._reset_missing(seen)

        return TrackResult(
            hands=hands,
            detected=len(detections),
            depth_valid_fraction=(valid_count / valid_total) if valid_total else 0.0,
        )

    # -- internals ---------------------------------------------------------

    def _to_scene(
        self, frame: ExportedFrame, detection: Detection
    ) -> TrackedHand | None:
        if not frame.ready_for_metric_3d():
            return None

        landmarks = detection.landmarks_px
        depths = self._sample_depths(frame, landmarks)
        depth_valid = depths > 0.0

        ref = self._reference(depth_valid)
        if ref is None:
            if self._fallback_depth <= 0.0:
                return None
            depths[:] = self._fallback_depth
        else:
            self._fill_missing_ranges(depths, depth_valid, ref, detection.world)

        joints = self._unproject_all(frame, landmarks, depths)
        if joints is None:
            return None

        chirality = self._chirality(detection, joints, landmarks)
        if chirality is None:
            # Edge-on in both the depth and the picture: nothing left to tell
            # this hand from its mirror image, and guessing would put it in
            # the other hand's slot, which reads downstream as the hand
            # teleporting across the body. Dropping the frame lets the last
            # good hand age out on its own 150 ms clock instead.
            return None

        filt = self._filters.get(chirality)
        if filt is None:
            filt = JointFilter(*self._filter_params, joints=joints.shape[0])
            self._filters[chirality] = filt

        return TrackedHand(
            chirality=chirality,
            confidence=detection.confidence,
            joints=filt.apply(joints, frame.t_ns / 1e9),
            depth_valid=depth_valid,
        )

    def _chirality(
        self, detection: Detection, joints: np.ndarray, landmarks: np.ndarray
    ) -> str | None:
        """Which hand this is, from the model, the depth, or the picture.

        The 3D test is the one that needs no assumption, so it goes first —
        but it needs the thumb to stand off the palm plane in MEASURED depth,
        and most of a hand's landmarks miss the 32x24 LiDAR map and take the
        reference landmark's range. A hand whose fingers all came back at one
        depth is flat by construction and says nothing about chirality, which
        is the common case rather than the corner one. The picture then
        decides it, under the stated assumption about which face is turned to
        the camera.
        """
        if detection.chirality is not None:
            return detection.chirality
        return chirality_from_joints(joints) or chirality_from_pixels(
            landmarks, dorsal=self._dorsal
        )

    def _sample_depths(self, frame: ExportedFrame, landmarks: np.ndarray) -> np.ndarray:
        depths = np.zeros(landmarks.shape[0], dtype=np.float64)
        for j, (u, v) in enumerate(landmarks):
            depths[j] = sample_depth(
                frame.depth, u / frame.width, v / frame.height, self._window
            )
        return depths

    @staticmethod
    def _fill_missing_ranges(
        depths: np.ndarray, depth_valid: np.ndarray, ref: int, world: np.ndarray | None
    ) -> None:
        if world is None:
            # No metric hand model: the best available guess is that the
            # landmark is as far away as the one anchoring it.
            depths[~depth_valid] = depths[ref]
            return
        # MediaPipe's world z grows AWAY from the camera in the same sense as
        # the depth map, so a difference in world z is a difference in range.
        # The landmark keeps its own pixel, so only the range is being
        # guessed, never the direction.
        missing = np.flatnonzero(~depth_valid)
        depths[missing] = np.maximum(
            0.05, depths[ref] + (world[missing, 2] - world[ref, 2])
        )

    @staticmethod
    def _unproject_all(
        frame: ExportedFrame, landmarks: np.ndarray, depths: np.ndarray
    ) -> np.ndarray | None:
        joints = np.zeros((landmarks.shape[0], 3), dtype=np.float64)
        for j, (u, v) in enumerate(landmarks):
            p_cam = unproject(
                frame.intrinsics, frame.width, frame.height, u, v, depths[j]
            )
            if p_cam is None:
                return None
            joints[j] = camera_to_scene(frame.head_pos, frame.head_quat, p_cam)
        return joints

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


def tracker_from_args(args) -> HandTracker:
    """One place where the CLI's flags become a tracker, so `track` and `eval`
    cannot drift into configuring the same pipeline differently."""
    shared = dict(
        backend=args.backend,
        depth_window=args.depth_window,
        min_cutoff=args.min_cutoff,
        beta=args.beta,
        fallback_depth_m=args.fallback_depth_m,
        dorsal_view=args.dorsal_view,
    )
    if args.backend == "mediapipe":
        return HandTracker(
            **shared, model_path=args.model, flip_handedness=args.flip_handedness
        )
    return HandTracker(**shared, device=args.device, det_interval=args.det_interval)
