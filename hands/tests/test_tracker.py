"""The metric stage, with the model replaced by a fixture.

The point of the backend seam is that everything from the landmarks onward is
the same code whichever model found them, so it can be tested without either
model being installed.
"""

import numpy as np
import pytest

from hands.backends import Detection
from hands.export_reader import ExportedFrame
from hands.geometry import INDEX_TIP, WRIST, Intrinsics
from hands.tracker import HandTracker
from test_handshape import IMG_H, IMG_W, INTR, flat_hand_camera, project

RANGE = 0.5


class FakeBackend:
    """Replays a scripted list of per-frame detections."""

    name = "fake"

    def __init__(self, script, **_):
        self._script = list(script)
        self.closed = False

    def detect(self, rgb, t_ns):
        return self._script.pop(0) if self._script else []

    def close(self):
        self.closed = True


def _tracker(script, monkeypatch, **kwargs):
    monkeypatch.setattr(
        "hands.tracker.make_backend", lambda name, **kw: FakeBackend(script)
    )
    return HandTracker(backend="fake", **kwargs)


def _frame(depth_m, seq=0, t_ns=0, hole=False):
    depth = np.full((192, 256), depth_m, dtype=np.float32)
    if hole:
        # The silhouette problem: most of the map is the wall behind the hand.
        depth[:, :] = 3.0
        depth[80:110, 110:150] = depth_m
    return ExportedFrame(
        seq=seq,
        t_ns=t_ns,
        export_ns=t_ns,
        rgb=np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8),
        intrinsics=INTR,
        head_pos=np.zeros(3),
        head_quat=np.array([0.0, 0.0, 0.0, 1.0]),
        depth=depth,
    )


def _detection(landmarks=None):
    px = project(flat_hand_camera()) if landmarks is None else landmarks
    return Detection(landmarks_px=px, confidence=0.9, world=None, chirality="right")


def test_rigid_mode_places_the_whole_hand_from_one_range(monkeypatch):
    tracker = _tracker([[_detection()]], monkeypatch, depth_mode="rigid")
    result = tracker.track(_frame(RANGE))
    hand = result.hands[0]
    assert hand.range_m == pytest.approx(RANGE, abs=1e-6)
    assert hand.range_samples > 0
    assert np.allclose(hand.joints[:, 2], -RANGE, atol=1e-4)


def test_rigid_mode_ignores_a_fingertip_that_read_the_wall_behind_it(monkeypatch):
    """The whole point: a landmark whose depth patch fell through the hand."""
    detection = _detection()
    frames = [_frame(RANGE, hole=True)]
    # Put the index tip somewhere the depth map only knows the wall.
    detection.landmarks_px[INDEX_TIP] = [10.0, 10.0]
    rigid = _tracker([[detection]], monkeypatch, depth_mode="rigid")
    per_joint = _tracker([[detection]], monkeypatch, depth_mode="per-joint")
    a = rigid.track(frames[0]).hands[0].joints[INDEX_TIP]
    b = per_joint.track(_frame(RANGE, hole=True)).hands[0].joints[INDEX_TIP]
    assert abs(a[2]) < 1.0  # still near the hand
    assert abs(b[2]) > 2.0  # out at the wall


def test_a_hand_with_no_lidar_reading_at_all_is_dropped(monkeypatch):
    tracker = _tracker([[_detection()]], monkeypatch, depth_mode="rigid")
    frame = _frame(RANGE)
    frame.depth[:, :] = 0.0
    assert tracker.track(frame).hands == []


def test_the_range_filter_damps_a_jumpy_measurement(monkeypatch):
    """One frame of bad ranging must not move the whole hand a decimetre."""
    script = [[_detection()] for _ in range(6)]
    tracker = _tracker(script, monkeypatch, depth_mode="rigid", range_beta=0.0)
    ranges = []
    for i, depth in enumerate([0.5, 0.5, 0.5, 0.9, 0.5, 0.5]):
        ranges.append(
            tracker.track(_frame(depth, seq=i, t_ns=i * 83_000_000)).hands[0].range_m
        )
    assert ranges[3] < 0.7  # the 0.9 m spike arrives heavily damped
    assert ranges[0] == pytest.approx(0.5, abs=1e-6)


def test_a_dropped_detection_is_held_for_two_frames_then_released(monkeypatch):
    script = [[_detection()], [], [], [], []]
    tracker = _tracker(script, monkeypatch, depth_mode="rigid", hold_frames=2)
    frames = [_frame(RANGE, seq=i, t_ns=i * 83_000_000) for i in range(5)]

    first = tracker.track(frames[0])
    assert not first.hands[0].held
    kept = first.hands[0].joints.copy()

    for i in (1, 2):
        result = tracker.track(frames[i])
        assert len(result.hands) == 1
        assert result.hands[0].held
        assert result.live == []
        assert np.allclose(result.hands[0].joints, kept)

    assert tracker.track(frames[3]).hands == []
    assert tracker.track(frames[4]).hands == []


def test_the_hold_can_be_turned_off(monkeypatch):
    tracker = _tracker([[_detection()], []], monkeypatch, hold_frames=0)
    tracker.track(_frame(RANGE))
    assert tracker.track(_frame(RANGE, seq=1, t_ns=83_000_000)).hands == []


def test_an_unknown_depth_mode_is_refused_at_construction(monkeypatch):
    with pytest.raises(ValueError, match="depth mode"):
        _tracker([], monkeypatch, depth_mode="whatever")


def test_a_hand_beyond_arms_reach_is_dropped_before_it_primes_the_range_filter(
    monkeypatch,
):
    # A hand-shaped thing across the room at 1.5 m, then the real hand at 0.5 m.
    tr = _tracker([[_detection()], [_detection()]], monkeypatch, max_range_m=1.0)
    assert tr.track(_frame(1.5, t_ns=0)).hands == []
    hands = tr.track(_frame(RANGE, seq=1, t_ns=int(0.05e9))).hands
    assert len(hands) == 1
    # Had the phantom primed the filter, the real hand would start far too deep.
    assert hands[0].range_m == pytest.approx(RANGE, abs=0.02)


def test_a_low_confidence_hand_is_dropped(monkeypatch):
    weak = Detection(**{**_detection().__dict__, "confidence": 0.2})
    tr = _tracker([[weak]], monkeypatch, min_confidence=0.5)
    assert tr.track(_frame(RANGE)).hands == []
