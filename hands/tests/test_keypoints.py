"""The RTMPose -> MediaPipe landmark mapping, and the chirality it needs.

RTMPose gives 21 points in COCO-WholeBody hand order and no handedness. Both
facts are load-bearing: a wrong mapping swaps the user's fingers, and a wrong
chirality puts the hand in the other hand's slot. Neither fails loudly, so
both are pinned here.
"""

import numpy as np
import pytest

from hands.geometry import chirality_from_joints, chirality_from_pixels
from hands.keypoints import (
    MEDIAPIPE_FROM_RTMPOSE,
    MEDIAPIPE_NAMES,
    RTMPOSE_HAND21_NAMES,
    RTMPOSE_TO_MEDIAPIPE,
    rtmpose_to_mediapipe,
)

# RTMPose's own name for each landmark, beside the MediaPipe name it maps to.
# Written out rather than derived, so a renumbering on either side has to be
# argued with rather than absorbed.
EXPECTED_PAIRS = (
    ("wrist", "WRIST"),
    ("thumb1", "THUMB_CMC"),
    ("thumb2", "THUMB_MCP"),
    ("thumb3", "THUMB_IP"),
    ("thumb4", "THUMB_TIP"),
    ("forefinger1", "INDEX_MCP"),
    ("forefinger2", "INDEX_PIP"),
    ("forefinger3", "INDEX_DIP"),
    ("forefinger4", "INDEX_TIP"),
    ("middle_finger1", "MIDDLE_MCP"),
    ("middle_finger2", "MIDDLE_PIP"),
    ("middle_finger3", "MIDDLE_DIP"),
    ("middle_finger4", "MIDDLE_TIP"),
    ("ring_finger1", "RING_MCP"),
    ("ring_finger2", "RING_PIP"),
    ("ring_finger3", "RING_DIP"),
    ("ring_finger4", "RING_TIP"),
    ("pinky_finger1", "PINKY_MCP"),
    ("pinky_finger2", "PINKY_PIP"),
    ("pinky_finger3", "PINKY_DIP"),
    ("pinky_finger4", "PINKY_TIP"),
)


def test_the_mapping_is_a_bijection():
    assert len(RTMPOSE_TO_MEDIAPIPE) == 21
    assert sorted(RTMPOSE_TO_MEDIAPIPE) == list(range(21))
    for m, r in enumerate(MEDIAPIPE_FROM_RTMPOSE):
        assert RTMPOSE_TO_MEDIAPIPE[r] == m


def test_each_rtmpose_landmark_maps_to_the_joint_of_the_same_name():
    for r, (rtm_name, mp_name) in enumerate(EXPECTED_PAIRS):
        assert RTMPOSE_HAND21_NAMES[r] == rtm_name
        assert MEDIAPIPE_NAMES[RTMPOSE_TO_MEDIAPIPE[r]] == mp_name


def test_rtmlib_still_numbers_its_hand_the_way_the_table_says():
    """The table above is a claim about the installed rtmlib, not about the
    paper. A version bump that renumbers must fail here rather than quietly
    move the user's fingertips."""
    hand21 = pytest.importorskip(
        "rtmlib.visualization.skeleton.hand21", reason="rtmlib not installed"
    ).hand21
    info = hand21["keypoint_info"]
    assert tuple(info[i]["name"] for i in range(21)) == RTMPOSE_HAND21_NAMES
    # thumb1 links to the wrist, which is what makes it the CMC rather than
    # the MCP — the off-by-one that would shift the whole thumb.
    links = {tuple(link["link"]) for link in hand21["skeleton_info"].values()}
    assert ("wrist", "thumb1") in links


def test_reordering_moves_landmarks_where_the_table_says():
    """The mapping is the identity today, so a test on real data proves
    nothing about the direction the table is applied in. Feed the reorder a
    known permutation instead, by checking it against the table itself."""
    keypoints = np.arange(21 * 2, dtype=np.float64).reshape(21, 2)
    out = rtmpose_to_mediapipe(keypoints)
    for r, m in enumerate(RTMPOSE_TO_MEDIAPIPE):
        assert np.array_equal(out[m], keypoints[r])


def test_reordering_handles_scores_and_batches():
    scores = np.arange(2 * 21, dtype=np.float64).reshape(2, 21)
    out = rtmpose_to_mediapipe(scores)
    assert out.shape == scores.shape
    for r, m in enumerate(RTMPOSE_TO_MEDIAPIPE):
        assert out[1][m] == scores[1][r]

    batched = np.arange(2 * 21 * 2, dtype=np.float64).reshape(2, 21, 2)
    out = rtmpose_to_mediapipe(batched)
    assert out.shape == batched.shape
    for r, m in enumerate(RTMPOSE_TO_MEDIAPIPE):
        assert np.array_equal(out[1][m], batched[1][r])


# ---------------------------------------------------------------------------
# chirality
# ---------------------------------------------------------------------------


def _flat_hand(thumb_side: float) -> np.ndarray:
    """A hand in the z = 0 plane: wrist at the origin, index MCP out along +x,
    pinky MCP along +y, and the thumb pushed `thumb_side` metres off the plane.

    cross(+x, +y) is +z, so a thumb at +z is on the palmar side and the hand
    is a right one.
    """
    joints = np.zeros((21, 3))
    joints[5] = [0.08, 0.0, 0.0]  # INDEX_MCP
    joints[17] = [0.0, 0.08, 0.0]  # PINKY_MCP
    for j in (2, 3, 4):  # THUMB_MCP, IP, TIP
        joints[j] = [0.04, -0.02, thumb_side]
    return joints


def test_a_thumb_on_the_palm_side_reads_right_and_its_mirror_reads_left():
    assert chirality_from_joints(_flat_hand(0.03)) == "right"
    assert chirality_from_joints(_flat_hand(-0.03)) == "left"


def test_a_thumb_in_the_palm_plane_is_undecidable_rather_than_guessed():
    assert chirality_from_joints(_flat_hand(0.0)) is None


def test_a_degenerate_hand_is_undecidable():
    assert chirality_from_joints(np.zeros((21, 3))) is None


def test_chirality_does_not_depend_on_hand_size():
    """The threshold is in palm widths, so a child's hand and a large one give
    the same answer at the same pose."""
    for scale in (0.5, 1.0, 2.0):
        assert chirality_from_joints(_flat_hand(0.03) * scale) == "right"


def _fingers_up(mirrored: bool) -> np.ndarray:
    """A hand in image pixels (u right, v DOWN), fingers up, wrist at the
    bottom. Unmirrored is a right hand seen palm-on: the index knuckle left of
    the pinky knuckle."""
    px = np.zeros((21, 2))
    px[0] = [100.0, 200.0]  # WRIST
    px[5] = [70.0, 120.0]  # INDEX_MCP
    px[17] = [130.0, 120.0]  # PINKY_MCP
    if mirrored:
        px[:, 0] = 200.0 - px[:, 0]
    return px


def test_the_picture_decides_chirality_once_the_face_is_known():
    assert chirality_from_pixels(_fingers_up(False), dorsal=False) == "right"
    assert chirality_from_pixels(_fingers_up(True), dorsal=False) == "left"
    # The same two pictures, read as backs of hands, are the other two hands.
    assert chirality_from_pixels(_fingers_up(False), dorsal=True) == "left"
    assert chirality_from_pixels(_fingers_up(True), dorsal=True) == "right"


def test_rotating_the_hand_in_the_image_does_not_change_the_answer():
    """The knuckles' order around the wrist survives a rotation; only a mirror
    changes it, which is the whole reason the signed area is the test."""
    px = _fingers_up(False)
    for degrees in (0, 37, 90, 175, 270):
        a = np.deg2rad(degrees)
        rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        turned = (px - px[0]) @ rot.T + px[0]
        assert chirality_from_pixels(turned, dorsal=False) == "right"


def test_an_edge_on_palm_is_undecidable_in_the_picture_too():
    px = np.zeros((21, 2))
    px[0] = [100.0, 200.0]
    px[5] = [100.0, 120.0]
    px[17] = [100.0, 130.0]  # knuckles in a line with the wrist
    assert chirality_from_pixels(px) is None
    assert chirality_from_pixels(np.zeros((21, 2))) is None


def test_a_flat_hand_falls_through_from_the_depth_to_the_picture():
    """Most landmarks miss the LiDAR map and inherit one range, so the 3D test
    goes quiet exactly when it is needed. The picture has to answer then."""
    flat = _flat_hand(0.0)
    assert chirality_from_joints(flat) is None
    assert chirality_from_pixels(_fingers_up(False), dorsal=True) is not None
