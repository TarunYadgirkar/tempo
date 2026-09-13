"""The reprojection round trip, and the smoothing lag it is paired with.

The round trip is the cheap half: it catches a camera model that disagrees
with itself. The lag numbers are the half that found the bug this file exists
for — a One Euro whose beta was small enough, against joint speeds measured in
metres per second, that the cutoff never left its floor and the filter was a
fixed low-pass drawing the hand a hand's width behind itself.
"""

import numpy as np
import pytest

from hands.check import LAG_BUDGET_PX, filter_lag_s
from hands.cli import build_parser
from hands.geometry import (
    Intrinsics,
    camera_to_scene,
    project,
    scene_to_camera,
    unproject,
)
from hands.onefilter import OneEuro

INTR = Intrinsics(
    fx=1430.02, fy=1430.02, cx=957.497, cy=718.977, image_width=1920, image_height=1440
)
CAM_POS = np.array([0.121871, -0.0614285, -0.152949])
CAM_QUAT = np.array([-0.288326, -0.357226, -0.604342, 0.651174])
CAM_QUAT = CAM_QUAT / np.linalg.norm(CAM_QUAT)
FPS = 30.0


@pytest.mark.parametrize("u,v,d", [(0.0, 0.0, 0.4), (320.0, 240.0, 0.5), (639.0, 479.0, 1.2)])
def test_scene_round_trip_is_subpixel(u, v, d):
    p_cam = unproject(INTR, 640, 480, u, v, d)
    scene = camera_to_scene(CAM_POS, CAM_QUAT, p_cam)
    back = project(INTR, 640, 480, scene_to_camera(CAM_POS, CAM_QUAT, scene))
    assert np.hypot(back[0] - u, back[1] - v) < 0.5


def test_range_does_not_move_the_pixel():
    """Why a wrong LiDAR range cannot be the source of a static overlay offset:
    the joint slides along its own ray and reprojects to the same pixel."""
    near = project(INTR, 640, 480, unproject(INTR, 640, 480, 200.0, 300.0, 0.3))
    far = project(INTR, 640, 480, unproject(INTR, 640, 480, 200.0, 300.0, 0.9))
    assert np.allclose(near, far, atol=1e-9)


def _measured_lag_s(min_cutoff: float, beta: float, speed: float) -> float:
    """Steady-state lag of the real filter on a ramp, in seconds."""
    filt = OneEuro(min_cutoff, beta)
    out = 0.0
    for i in range(600):
        t = i / FPS
        out = filt(speed * t, t)
    truth = speed * (599 / FPS)
    return (truth - out) / speed


@pytest.mark.parametrize("speed", [0.3, 1.0])
def test_lag_model_is_a_tight_upper_bound(speed):
    # One Euro estimates speed from the FILTERED history, so its own estimate
    # runs ahead of the true ramp speed and the real lag comes in a little
    # under the closed form. The reported number is therefore conservative,
    # which is the right direction for a budget.
    predicted = filter_lag_s(2.0, 10.0, speed)
    measured = _measured_lag_s(2.0, 10.0, speed)
    assert measured <= predicted
    assert measured > 0.6 * predicted


def test_shipped_defaults_stay_inside_the_pixel_budget():
    args = build_parser().parse_args(["check", "--dir", "/tmp"])
    px_per_m = INTR.fx * (640 / INTR.image_width) / 0.5
    for speed in (0.3, 1.0):
        lag = filter_lag_s(args.min_cutoff, args.beta, speed)
        assert px_per_m * lag * speed <= LAG_BUDGET_PX


def test_a_beta_that_never_lifts_the_cutoff_is_over_budget():
    px_per_m = INTR.fx * (640 / INTR.image_width) / 0.5
    lag = filter_lag_s(1.0, 0.02, 0.3)
    assert px_per_m * lag * 0.3 > 3 * LAG_BUDGET_PX


def test_range_filter_keeps_up_with_a_reaching_arm():
    args = build_parser().parse_args(["check", "--dir", "/tmp"])
    lag = filter_lag_s(args.range_min_cutoff, args.range_beta, 0.5)
    assert 0.5 * lag < 0.05
