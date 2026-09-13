"""RTMPose hand through rtmlib, on ONNX Runtime.

Two models, both from OpenMMLab's RTMPose release and downloaded by rtmlib on
first use into `~/.cache/rtmlib`:

  RTMDet-nano hand  320x320, finds the hand boxes. ~6 ms on this Mac's CPU.
  RTMPose-m hand5   256x256, 21 keypoints inside one box. ~13 ms on the CPU,
                    ~4 ms on the CoreML execution provider.

The detector stays on the CPU on purpose. Its ONNX export carries the
grid-decode ops whose static shapes CoreML infers at a different rank, and
ONNX Runtime raises mid-inference rather than falling back — so CoreML is
asked for the pose model only, which is where the time is anyway. If the
CoreML provider is missing or refuses the pose model, the constructor drops to
the CPU and says so once.

Detection is not run every frame. A hand that was found last frame is looked
for inside its own grown box, and the detector is called again only every
`det_interval` frames, when the pose score falls below `redetect_score`, or
when nothing was tracked at all. That is what takes a frame from ~10 ms to
~4 ms: the detector is the more expensive half and the hand rarely leaves the
box it was in 30 ms ago.

rtmlib is fed BGR because that is what every rtmlib example feeds it — its
preprocessing does no colour conversion of its own, and the normalisation
constants were chosen against that input.
"""

from __future__ import annotations

import numpy as np

from ..keypoints import rtmpose_to_mediapipe
from . import Detection

DET_URL = "https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/rtmdet_nano_8xb32-300e_hand-267f9c8f.zip"
POSE_URL = "https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/rtmpose-m_simcc-hand5_pt-aic-coco_210e-256x256-74fb594_20230320.zip"

# How far past the previous frame's landmarks the tracking box is grown, as a
# fraction of that hand's own span. The hand has one frame (~33 ms) to move,
# and RTMPose wants the whole hand inside the box or the landmarks outside it
# get pulled to the edge.
TRACK_BOX_MARGIN = 0.35


class RTMPoseBackend:
    name = "rtmpose"

    def __init__(
        self,
        num_hands: int = 2,
        det_interval: int = 10,
        redetect_score: float = 0.4,
        min_detection_confidence: float = 0.3,
        device: str = "auto",
        **_ignored,
    ):
        from rtmlib import RTMDet

        self._det = RTMDet(
            DET_URL,
            model_input_size=(320, 320),
            backend="onnxruntime",
            device="cpu",
            score_thr=min_detection_confidence,
        )
        self._pose, self.device = self._make_pose(device)
        self._num_hands = num_hands
        self._det_interval = max(1, det_interval)
        self._redetect_score = redetect_score
        self._boxes: list[list[float]] = []
        self._since_detect = 0

    @staticmethod
    def _make_pose(device: str):
        from rtmlib import RTMPose

        def build(dev):
            return RTMPose(
                POSE_URL,
                model_input_size=(256, 256),
                backend="onnxruntime",
                device=dev,
            )

        if device in ("auto", "mps", "coreml"):
            try:
                pose = build("mps")
                # CoreML defers its real failures to the first inference, so a
                # session that constructs is not yet a session that runs.
                pose(np.zeros((64, 64, 3), dtype=np.uint8), bboxes=[[0, 0, 64, 64]])
                return pose, "coreml"
            except Exception as exc:
                if device != "auto":
                    raise
                print(f"hands: CoreML refused the pose model ({exc}); using CPU")
        return build("cpu"), "cpu"

    def close(self) -> None:
        pass

    # -- the loop body -----------------------------------------------------

    def detect(self, rgb: np.ndarray, t_ns: int) -> list[Detection]:
        bgr = np.ascontiguousarray(rgb[:, :, ::-1])

        if self._needs_detection():
            self._boxes = self._detect_boxes(bgr)
            self._since_detect = 0
        else:
            self._since_detect += 1

        if not self._boxes:
            return []

        keypoints, scores = self._pose(bgr, bboxes=self._boxes)
        keypoints = rtmpose_to_mediapipe(np.asarray(keypoints, dtype=np.float64))
        scores = rtmpose_to_mediapipe(np.asarray(scores, dtype=np.float64))

        out: list[Detection] = []
        boxes: list[list[float]] = []
        for i in range(keypoints.shape[0]):
            confidence = float(np.mean(scores[i]))
            if confidence < self._redetect_score:
                continue
            out.append(
                Detection(
                    landmarks_px=keypoints[i],
                    confidence=confidence,
                    world=None,
                    chirality=None,
                )
            )
            boxes.append(_box_around(keypoints[i], rgb.shape[1], rgb.shape[0]))
        # Track from where the hand actually is, not from where the detector
        # last guessed. A hand that failed the score gate drops its box too,
        # which is what forces a detection on the next frame.
        self._boxes = boxes
        return out

    def _needs_detection(self) -> bool:
        return not self._boxes or self._since_detect >= self._det_interval

    def _detect_boxes(self, bgr: np.ndarray) -> list[list[float]]:
        boxes = np.asarray(self._det(bgr), dtype=np.float64)
        if boxes.size == 0:
            return []
        # Biggest first: when more hands are found than the shell has slots,
        # the near one is the one being gestured with.
        areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        order = np.argsort(-areas)[: self._num_hands]
        return [list(boxes[i]) for i in order]


def _box_around(landmarks_px: np.ndarray, width: int, height: int) -> list[float]:
    """The next frame's search box: this frame's landmarks, grown and clamped."""
    x0, y0 = landmarks_px.min(axis=0)
    x1, y1 = landmarks_px.max(axis=0)
    mx = (x1 - x0) * TRACK_BOX_MARGIN
    my = (y1 - y0) * TRACK_BOX_MARGIN
    return [
        float(max(0.0, x0 - mx)),
        float(max(0.0, y0 - my)),
        float(min(width, x1 + mx)),
        float(min(height, y1 + my)),
    ]
