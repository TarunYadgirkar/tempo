"""Parity with the C++ half of the pipeline.

Every number here is also asserted in shell/mac-shell/tests/test_hand_inject.cpp
(ctest `hand_inject`). The two implementations are separate on purpose — the
shell needs the math to answer `hands-inject`, this package needs it to produce
the payload — so they need a shared set of numbers to keep agreeing on.
"""

import numpy as np
import pytest

from hands.geometry import (
    MEDIAPIPE_TO_SB,
    Intrinsics,
    camera_to_scene,
    quat_rotate,
    sample_depth,
    unproject,
)

INTR = Intrinsics(fx=1000, fy=1000, cx=960, cy=540, image_width=1920, image_height=1080)


def test_landmark_order_is_a_bijection():
    assert len(MEDIAPIPE_TO_SB) == 21
    assert sorted(MEDIAPIPE_TO_SB) == list(range(21))


def test_principal_point_lies_on_the_optical_axis():
    p = unproject(INTR, 1920, 1080, 960, 540, 2.0)
    assert np.allclose(p, [0, 0, -2.0])


def test_one_focal_length_off_centre_is_one_metre_at_one_metre():
    assert np.allclose(unproject(INTR, 1920, 1080, 1960, 540, 1.0), [1.0, 0.0, -1.0])
    # Image v grows downward and camera +Y is up, so below centre is -Y.
    assert np.allclose(unproject(INTR, 1920, 1080, 960, 1540, 1.0), [0.0, -1.0, -1.0])


def test_a_downscaled_image_unprojects_to_the_same_point():
    """The streamed JPEG is smaller than the capture the intrinsics describe."""
    full = unproject(INTR, 1920, 1080, 1960, 540, 1.0)
    half = unproject(INTR, 960, 540, 980, 270, 1.0)
    assert np.allclose(full, half)


@pytest.mark.parametrize("depth", [0.0, -1.0, float("nan")])
def test_no_reading_is_a_refusal_not_a_point(depth):
    assert unproject(INTR, 1920, 1080, 960, 540, depth) is None


def test_degenerate_intrinsics_are_refused():
    bad = Intrinsics(fx=0, fy=0, cx=0, cy=0, image_width=0, image_height=0)
    assert unproject(bad, 1920, 1080, 10, 10, 1.0) is None


def test_yaw_takes_the_view_direction_onto_minus_x():
    s = np.sin(np.pi / 4)
    yaw90 = np.array([0, s, 0, s])
    assert np.allclose(quat_rotate(yaw90, np.array([0, 0, -1.0])), [-1, 0, 0], atol=1e-6)


def test_full_pipeline_puts_the_axis_point_in_front_of_the_head():
    p_cam = unproject(INTR, 1920, 1080, INTR.cx, INTR.cy, 2.0)
    scene = camera_to_scene(np.array([0, 1.0, 0]), np.array([0, 0, 0, 1.0]), p_cam)
    assert np.allclose(scene, [0.0, 1.0, -2.0])


def test_depth_sampling_medians_the_patch_and_skips_the_holes():
    depth = np.zeros((9, 9), dtype=np.float32)
    depth[3:6, 3:6] = 1.5
    depth[4, 4] = 0.0  # a hole right where the landmark lands
    assert sample_depth(depth, 0.5, 0.5, window=5) == pytest.approx(1.5)

    # A patch of nothing but holes reports "no reading" rather than zero metres.
    assert sample_depth(np.zeros((9, 9), dtype=np.float32), 0.5, 0.5) == 0.0

    # The median rejects a minority of background readings behind a finger.
    straddle = np.full((9, 9), 3.0, dtype=np.float32)
    straddle[3:6, 3:6] = 0.6
    straddle[3, 3] = 3.0
    assert sample_depth(straddle, 0.5, 0.5, window=3) == pytest.approx(0.6)
