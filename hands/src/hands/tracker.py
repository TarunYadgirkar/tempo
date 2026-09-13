"""Pixel landmarks -> scene-frame metric joints.

The model that found the landmarks lives behind `backends/`; everything here
is the same for both of them, which is the point of the split — swapping the
model must not be able to change the metric pipeline underneath it.

  1. A backend finds the 21 landmarks in the exported image's own pixels,
     already in MediaPipe landmark order.
  2. The hand gets ONE range from the LiDAR map, pooled over the patches under
     the wrist and the four finger MCPs, and that range is smoothed harder
     than anything else in the pipeline.
  3. The 21 ranges come from a hand-shape model at that range (`handshape.py`)
     and each landmark unprojects along its own pixel ray. This is the
     `--depth rigid` default. `--depth per-joint` is the older path, where
     every landmark reads the depth map under itself; it is kept because it is
     the thing the default has to beat, and it loses badly — the map is
     256x192 over a 1920x1440 capture, so a 5x5 patch on a fingertip is wider
     than the finger and the median falls through to the wall behind it.
  4. The head pose the shell stamped on the frame carries the point into the
     scene frame, where the panels are.
  5. One Euro per axis per landmark takes out the residual jitter, and a
     2-frame hold covers a dropped detection so the gesture engine does not
     see the hand blink out of existence for 30 ms.

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
    chirality_from_pixels,
    sample_depth,
    unproject,
)
from .handshape import palm_range_m, rigid_ranges
from .onefilter import JointFilter, OneEuro

DEPTH_MODES = ("rigid", "per-joint")
DEFAULT_DEPTH_MODE = "rigid"

# Frames a hand is re-sent after the model stopped finding it. The shell
# retires an injection after 150 ms on its own clock, so this is not about
# keeping a dead tracker alive — it is about a single dropped detection in the
# middle of a pinch, which at 12 Hz is 80 ms of the gesture engine being told
# the hand left the room. Two frames, then the truth.
DEFAULT_HOLD_FRAMES = 2
# A head-mounted camera sees the wearer's own hand within arm's reach.
DEFAULT_MAX_RANGE_M = 1.0
DEFAULT_MIN_CONFIDENCE = 0.5



@dataclass
class TrackedHand:
    chirality: str  # "left" | "right"
    confidence: float
    joints: np.ndarray  # (21, 3) scene-frame metres, MediaPipe landmark order
    depth_valid: np.ndarray  # (21,) bool — which landmarks got a real reading
    range_m: float = 0.0  # the hand's smoothed LiDAR range, 0 in per-joint mode
    range_samples: int = 0  # LiDAR readings that voted for it
    held: bool = False  # re-sent from the last frame the model found it


@dataclass
class TrackResult:
    hands: list[TrackedHand]
    detected: int  # hands the model found, before the metric stage could drop any
    depth_valid_fraction: float  # over the fingertips of the hands that survived

    @property
    def live(self) -> list[TrackedHand]:
        """The hands this frame actually saw, without the held ones."""
        return [h for h in self.hands if not h.held]


class HandTracker:
    def __init__(
        self,
        backend: str = DEFAULT_BACKEND,
        depth_window: int = 5,
        depth_mode: str = DEFAULT_DEPTH_MODE,
        min_cutoff: float = 2.0,
        beta: float = 10.0,
        range_min_cutoff: float = 1.5,
        range_beta: float = 2.0,
        hold_frames: int = DEFAULT_HOLD_FRAMES,
        fallback_depth_m: float = 0.0,
        dorsal_view: bool = True,
        max_range_m: float = DEFAULT_MAX_RANGE_M,
        min_confidence: float = DEFAULT_MIN_CONFIDENCE,
        **backend_kwargs,
    ):
        if depth_mode not in DEPTH_MODES:
            raise ValueError(f"unknown depth mode {depth_mode!r}: expected one of {DEPTH_MODES}")
        self.backend_name = backend
        self.depth_mode = depth_mode
        self._backend = make_backend(backend, **backend_kwargs)
        self._window = depth_window
        self._fallback_depth = fallback_depth_m
        self._dorsal = dorsal_view
        self._max_range = max_range_m
        self._min_confidence = min_confidence
        self._hold_frames = max(0, hold_frames)
        self._filters: dict[str, JointFilter] = {}
        self._filter_params = (min_cutoff, beta)
        self._range_filters: dict[str, OneEuro] = {}
        self._range_params = (range_min_cutoff, range_beta)
        self._held: dict[str, tuple[TrackedHand, int]] = {}

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

        hands += self._hold(seen)
        self._reset_missing({h.chirality for h in hands})

        return TrackResult(
            hands=hands,
            detected=len(detections),
            depth_valid_fraction=(valid_count / valid_total) if valid_total else 0.0,
        )

    # -- internals ---------------------------------------------------------

    def _hold(self, seen: set[str]) -> list[TrackedHand]:
        """Re-send a hand the model lost, for a couple of frames.

        The joints are re-sent unchanged rather than extrapolated. A held
        frame is one the tracker has no news about, and inventing motion for
        it would make a stalled hand drift off on its last velocity, which is
        worse to look at than a hand that pauses.
        """
        out: list[TrackedHand] = []
        for chirality in list(self._held):
            hand, age = self._held[chirality]
            if chirality in seen:
                # Already refreshed by _to_scene this frame.
                continue
            if age >= self._hold_frames:
                del self._held[chirality]
                continue
            self._held[chirality] = (hand, age + 1)
            out.append(
                TrackedHand(
                    chirality=hand.chirality,
                    confidence=hand.confidence,
                    joints=hand.joints,
                    depth_valid=hand.depth_valid,
                    range_m=hand.range_m,
                    range_samples=hand.range_samples,
                    held=True,
                )
            )
        return out

    def _to_scene(
        self, frame: ExportedFrame, detection: Detection
    ) -> TrackedHand | None:
        if not frame.ready_for_metric_3d():
            return None

        landmarks = detection.landmarks_px
        rigid = self.depth_mode == "rigid"
        built = (
            self._measure_range(frame, detection)
            if rigid
            else self._per_joint_depths(frame, detection)
        )
        if built is None:
            return None
        depths, depth_valid, range_m, range_samples = built
        if not self._plausible(detection, range_m):
            return None

        chirality = self._chirality(detection, landmarks)
        if chirality is None:
            # Edge-on in the picture, and nothing else can tell this hand from
            # its mirror image. Guessing would put it in the other hand's
            # slot, which reads downstream as the hand teleporting across the
            # body; dropping the frame lets the last good hand age out on the
            # shell's own 150 ms clock instead.
            return None

        range_m = self._smooth_range(chirality, range_m, frame.t_ns / 1e9)
        if rigid:
            depths = rigid_ranges(
                frame.intrinsics,
                frame.width,
                frame.height,
                landmarks,
                range_m,
                world=detection.world,
                dorsal=self._dorsal,
            )

        joints = self._unproject_all(frame, landmarks, depths)
        if joints is None:
            return None

        filt = self._filters.get(chirality)
        if filt is None:
            filt = JointFilter(*self._filter_params, joints=joints.shape[0])
            self._filters[chirality] = filt

        hand = TrackedHand(
            chirality=chirality,
            confidence=detection.confidence,
            joints=filt.apply(joints, frame.t_ns / 1e9),
            depth_valid=depth_valid,
            range_m=range_m,
            range_samples=range_samples,
        )
        self._held[chirality] = (hand, 0)
        return hand

    def _plausible(self, detection: Detection, range_m: float) -> bool:
        """Is this a wearer's hand, or something hand-shaped across the room?

        The camera is on the head, so a hand the wearer can gesture with is
        within arm's reach; anything further is another person or a false
        positive. Gated on the RAW range, before smoothing, so a phantom never
        primes the range filter that the real hand then inherits.
        """
        if detection.confidence < self._min_confidence:
            return False
        if self._max_range > 0.0 and range_m > self._max_range:
            return False
        return True

    def _measure_range(self, frame: ExportedFrame, detection: Detection):
        """The one LiDAR question the rigid path asks: how far is the hand?

        The ranges themselves are built afterwards, from the SMOOTHED answer,
        so no joint is ever placed with an unfiltered range.

        `depth_valid` is still the per-landmark LiDAR validity even though
        nothing places a joint with it any more: it is what the eval reports
        as "how much of this is measured", and answering that with the rigid
        model's own coverage would be answering a different question.
        """
        landmarks = detection.landmarks_px
        measured, samples = palm_range_m(
            frame.depth, landmarks, frame.width, frame.height, self._window
        )
        if measured <= 0.0:
            measured = self._fallback_depth
            if measured <= 0.0:
                return None
            samples = 0
        depth_valid = self._sample_depths(frame, landmarks) > 0.0
        return None, depth_valid, measured, samples

    def _per_joint_depths(self, frame: ExportedFrame, detection: Detection):
        """The original path: every landmark reads the map under itself."""
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
        return depths, depth_valid, 0.0, int(depth_valid.sum())

    def _smooth_range(self, chirality: str, range_m: float, t_s: float) -> float:
        """One Euro on the hand's range, tuned far harder than the joints.

        Range is the one number every joint in the frame depends on, so its
        noise moves the whole hand together — and unlike a fingertip's
        position it is not something a person changes quickly: an arm extends
        at tens of centimetres a second, not metres. A low cutoff here costs
        almost no responsiveness and takes out the LiDAR's own ranging noise
        before it is multiplied across 21 joints.
        """
        if range_m <= 0.0:
            return range_m
        filt = self._range_filters.get(chirality)
        if filt is None:
            filt = OneEuro(*self._range_params)
            self._range_filters[chirality] = filt
        return float(filt(range_m, t_s))

    def _chirality(self, detection: Detection, landmarks: np.ndarray) -> str | None:
        """Which hand this is, from the model or from the picture.

        The model answers when it can (MediaPipe does, RTMPose does not). The
        picture answers otherwise, under the stated assumption about which
        face is turned to the camera.

        There used to be a 3D test here, on the sign of the thumb's offset
        from the palm plane, tried before the picture. It cannot run any more
        and should not: under the rigid shape model the thumb's side of the
        palm plane is set by the SAME `dorsal` assumption the pixel test uses,
        so asking the joints would be asking the assumption to confirm itself.
        `chirality_from_joints` stays in geometry.py because the shell's
        gesture engine computes the same quantity and the two are pinned
        against each other.
        """
        if detection.chirality is not None:
            return detection.chirality
        return chirality_from_pixels(landmarks, dorsal=self._dorsal)

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
        for chirality, filt in self._range_filters.items():
            if chirality not in seen:
                filt.reset()


def tracker_from_args(args) -> HandTracker:
    """One place where the CLI's flags become a tracker, so `track` and `eval`
    cannot drift into configuring the same pipeline differently."""
    shared = dict(
        backend=args.backend,
        depth_window=args.depth_window,
        depth_mode=args.depth,
        min_cutoff=args.min_cutoff,
        beta=args.beta,
        range_min_cutoff=args.range_min_cutoff,
        range_beta=args.range_beta,
        hold_frames=args.hold_frames,
        fallback_depth_m=args.fallback_depth_m,
        dorsal_view=args.dorsal_view,
        max_range_m=args.max_range_m,
        min_confidence=args.min_confidence,
    )
    if args.backend == "mediapipe":
        return HandTracker(
            **shared, model_path=args.model, flip_handedness=args.flip_handedness
        )
    return HandTracker(**shared, device=args.device, det_interval=args.det_interval)
