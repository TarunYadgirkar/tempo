"""The rigid hand-shape model.

The load-bearing claim is that a hand placed from ONE range still projects
back to the pixels the model found it at, and still has the bone lengths it
was built with. Everything else the model does is a choice about which of two
roots to take; this pins that it is solving the right equation.
"""

import numpy as np
import pytest

from hands.geometry import (
    INDEX_MCP,
    INDEX_TIP,
    PINKY_MCP,
    THUMB_TIP,
    WRIST,
    Intrinsics,
    unproject,
)
from hands.handshape import (
    BONE_M,
    CHAINS,
    MIN_RANGE_M,
    PALM_IDS,
    PALM_WIDTH_M,
    RANGE_IDS,
    SIZE_SCALE_MAX,
    SIZE_SCALE_MIN,
    palm_range_m,
    ray_from_pixel,
    rigid_ranges,
    size_scale,
    solve_segment,
)

INTR = Intrinsics(
    fx=1347.0, fy=1347.0, cx=960.0, cy=720.0, image_width=1920, image_height=1440
)
IMG_W, IMG_H = 640, 480
RANGE = 0.5


# --- a synthetic flat hand -------------------------------------------------


def flat_hand_camera(range_m=RANGE, scale=1.0):
    """21 joints of a flat hand, palm-on to the camera at `range_m`.

    Laid out in the plane z = -range_m: the wrist at the optical axis, the
    knuckles on a row PALM_WIDTH_M across, and each finger extended straight
    up its own column with the model's own bone lengths. So the answer the
    shape model should reproduce is known exactly.
    """
    j = np.zeros((21, 3))
    palm = PALM_WIDTH_M * scale
    # Knuckle row: index at -palm/2 through pinky at +palm/2, on x.
    knuckles = {5: -0.5, 9: -0.5 / 3.0, 13: 0.5 / 3.0, 17: 0.5}
    j[WRIST] = [0.0, -0.09 * scale, 0.0]
    j[1] = [-0.55 * palm, -0.045 * scale, 0.0]  # thumb CMC, off the wrist
    for idx, frac in knuckles.items():
        j[idx] = [frac * palm, 0.0, 0.0]
    for chain in CHAINS:
        for proximal, distal in zip(chain, chain[1:]):
            length = BONE_M[(proximal, distal)] * scale
            # Fingers straight up the image; the thumb out to its own side.
            step = (
                np.array([-length, 0.0, 0.0])
                if chain[0] == 1
                else np.array([0.0, length, 0.0])
            )
            j[distal] = j[proximal] + step
    j[:, 2] = -range_m
    return j


def project(joints):
    """Camera-frame points -> pixels of the IMG_W x IMG_H exported image."""
    px = []
    for x, y, z in joints:
        d = -z
        u = (x * INTR.fx / d + INTR.cx) * (IMG_W / INTR.image_width)
        v = (-y * INTR.fy / d + INTR.cy) * (IMG_H / INTR.image_height)
        px.append([u, v])
    return np.array(px)


def rebuild(landmarks_px, range_m=RANGE, world=None, dorsal=True):
    depths = rigid_ranges(
        INTR, IMG_W, IMG_H, landmarks_px, range_m, world=world, dorsal=dorsal
    )
    return np.array(
        [
            unproject(INTR, IMG_W, IMG_H, u, v, depths[i])
            for i, (u, v) in enumerate(landmarks_px)
        ]
    )


def test_a_flat_hand_at_half_a_metre_comes_back_where_it_was():
    truth = flat_hand_camera()
    built = rebuild(project(truth))
    assert np.allclose(built, truth, atol=1e-6)


def test_the_rebuilt_hand_reprojects_to_the_pixels_it_came_from():
    """The model only ever answers "how far"; it must not move a pixel."""
    landmarks = project(flat_hand_camera())
    assert np.allclose(project(rebuild(landmarks)), landmarks, atol=1e-6)


def test_the_rebuilt_hand_keeps_its_bone_lengths():
    built = rebuild(project(flat_hand_camera()))
    for (proximal, distal), length in BONE_M.items():
        got = float(np.linalg.norm(built[distal] - built[proximal]))
        assert got == pytest.approx(length, abs=1e-4)


def test_a_smaller_hand_is_not_forced_to_the_adult_skeleton():
    truth = flat_hand_camera(scale=0.8)
    built = rebuild(project(truth))
    for (proximal, distal), length in BONE_M.items():
        got = float(np.linalg.norm(built[distal] - built[proximal]))
        assert got == pytest.approx(length * 0.8, abs=5e-4)


def test_every_joint_lands_at_the_measured_range_when_the_hand_is_flat():
    landmarks = project(flat_hand_camera())
    depths = rigid_ranges(INTR, IMG_W, IMG_H, landmarks, RANGE)
    assert np.allclose(depths, RANGE, atol=1e-6)


def test_the_palm_is_one_plate_at_the_measured_range():
    """Whatever the fingers do, the palm is placed, not solved."""
    landmarks = project(flat_hand_camera())
    landmarks[INDEX_TIP] += [30.0, -20.0]  # a badly placed fingertip
    depths = rigid_ranges(INTR, IMG_W, IMG_H, landmarks, RANGE)
    assert np.allclose(depths[list(PALM_IDS)], RANGE, atol=1e-9)


def test_a_curled_finger_goes_away_from_a_dorsal_camera_and_toward_a_palmar_one():
    landmarks = project(flat_hand_camera())
    # Pull the index tip back toward its knuckle: the finger is curling, so
    # its apparent length is now shorter than the bone.
    landmarks[INDEX_TIP] = landmarks[7] + (landmarks[INDEX_TIP] - landmarks[7]) * 0.3
    dorsal = rigid_ranges(INTR, IMG_W, IMG_H, landmarks, RANGE, dorsal=True)
    palmar = rigid_ranges(INTR, IMG_W, IMG_H, landmarks, RANGE, dorsal=False)
    assert dorsal[INDEX_TIP] > RANGE
    assert palmar[INDEX_TIP] < RANGE
    # Both roots are on the bone's sphere, so the distal phalanx comes out at
    # its model length either way — only its side of the palm differs.
    for depths in (dorsal, palmar):
        built = np.array(
            [
                unproject(INTR, IMG_W, IMG_H, u, v, depths[i])
                for i, (u, v) in enumerate(landmarks)
            ]
        )
        assert float(np.linalg.norm(built[INDEX_TIP] - built[7])) == pytest.approx(
            BONE_M[(7, 8)], abs=1e-6
        )


def test_an_impossible_segment_is_placed_rather_than_dropped():
    """A finger longer in the picture than the model says it can be."""
    landmarks = project(flat_hand_camera())
    landmarks[INDEX_TIP] = landmarks[7] + (landmarks[INDEX_TIP] - landmarks[7]) * 6.0
    depths = rigid_ranges(INTR, IMG_W, IMG_H, landmarks, RANGE)
    assert np.all(np.isfinite(depths))
    assert depths[INDEX_TIP] >= MIN_RANGE_M


def test_no_joint_is_placed_inside_the_camera():
    r0 = np.array([0.0, 0.0, 1.0])
    assert solve_segment(r0, 0.06, r0 * 1.0, 0.5, dorsal=False) == MIN_RANGE_M


# --- the measured range ----------------------------------------------------


def test_the_range_pools_the_palm_patches_and_outvotes_one_bad_one():
    depth = np.zeros((192, 256), dtype=np.float32)
    landmarks = project(flat_hand_camera())
    for i, j in enumerate(RANGE_IDS):
        u, v = landmarks[j]
        cx = int(round(u / IMG_W * 255))
        cy = int(round(v / IMG_H * 191))
        # One knuckle reads the wall behind the hand; the other four read skin.
        value = 3.0 if i == 0 else 0.5
        depth[cy - 2 : cy + 3, cx - 2 : cx + 3] = value
    got, samples = palm_range_m(depth, landmarks, IMG_W, IMG_H)
    assert got == pytest.approx(0.5)
    assert samples == 5 * 25


def test_a_hand_with_no_readings_reports_no_range():
    landmarks = project(flat_hand_camera())
    got, samples = palm_range_m(
        np.zeros((192, 256), dtype=np.float32), landmarks, IMG_W, IMG_H
    )
    assert (got, samples) == (0.0, 0)


def test_the_size_scale_is_clamped_against_a_foreshortened_palm():
    assert size_scale(0.0, PALM_WIDTH_M) == 1.0
    assert size_scale(0.001, PALM_WIDTH_M) == SIZE_SCALE_MIN
    assert size_scale(10.0, PALM_WIDTH_M) == SIZE_SCALE_MAX


def test_the_ray_scales_a_downscaled_pixel_into_intrinsics_pixels():
    full = ray_from_pixel(INTR, INTR.image_width, INTR.image_height, 1920.0, 1440.0)
    half = ray_from_pixel(INTR, IMG_W, IMG_H, IMG_W, IMG_H)
    assert np.allclose(full, half)


# --- the MediaPipe path ----------------------------------------------------


def test_a_world_skeleton_is_slid_until_its_palm_sits_at_the_measured_range():
    truth = flat_hand_camera()
    landmarks = project(truth)
    # MediaPipe's world frame: centred on the hand, z growing AWAY from the
    # camera. Give it a real fist-like curl so the offsets are not all zero.
    world = truth.copy()
    world[:, 2] = -world[:, 2] + RANGE  # away-positive, palm at 0
    world[INDEX_TIP, 2] += 0.03
    world[THUMB_TIP, 2] -= 0.02
    depths = rigid_ranges(INTR, IMG_W, IMG_H, landmarks, RANGE, world=world)
    assert depths[WRIST] == pytest.approx(RANGE, abs=1e-6)
    assert depths[INDEX_TIP] == pytest.approx(RANGE + 0.03, abs=2e-3)
    assert depths[THUMB_TIP] == pytest.approx(RANGE - 0.02, abs=2e-3)


def test_the_world_skeleton_is_sized_against_its_own_palm_not_the_adult_mean():
    """A model that thinks the hand is small must not shrink it twice."""
    truth = flat_hand_camera()
    landmarks = project(truth)
    world = truth.copy()
    world[:, 2] = -world[:, 2] + RANGE
    world[INDEX_TIP, 2] += 0.03
    world *= 0.8  # the whole skeleton, palm width included, at four fifths
    depths = rigid_ranges(INTR, IMG_W, IMG_H, landmarks, RANGE, world=world)
    assert depths[INDEX_TIP] == pytest.approx(RANGE + 0.03, abs=3e-3)


def test_the_knuckle_span_is_what_sets_the_size():
    landmarks = project(flat_hand_camera())
    rays = np.array([ray_from_pixel(INTR, IMG_W, IMG_H, u, v) for u, v in landmarks])
    span = float(np.linalg.norm(rays[INDEX_MCP] - rays[PINKY_MCP])) * RANGE
    assert span == pytest.approx(PALM_WIDTH_M, abs=1e-6)
