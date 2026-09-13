"""The 21 hand keypoints, and the two orders that have to agree.

Three enumerations of the same skeleton meet in this package:

  MediaPipe Hand Landmarker  wrist, thumb CMC/MCP/IP/TIP, then index, middle,
                             ring and pinky MCP/PIP/DIP/TIP.
  RTMPose hand21             wrist, thumb1..4, forefinger1..4, then middle,
                             ring and pinky 1..4 (rtmlib's own name for the
                             COCO-WholeBody left/right-hand block).
  SB_JOINT_*                 the shell's order, which the 0x05 packet packs
                             and GE_JOINT_* mirrors.

All three run wrist first, then the thumb, then the fingers radial to ulnar,
each finger proximal to distal. So every mapping between them is the identity.
That is a fact about today's three enumerations, not a law, and it is the kind
of fact that stops being true silently when one side renumbers — so each
mapping is written out as a table and asserted, here and in
`shell/mac-shell/src/core/hand_inject.cpp`.

`thumb1` is the CMC (the joint at the wrist), not the MCP: rtmlib's hand21
numbers the thumb from the same carpometacarpal joint MediaPipe calls CMC, so
`thumb1..4` and `THUMB_CMC/MCP/IP/TIP` name the same four joints in the same
order rather than being offset by one. Checked against rtmlib's own
`visualization/skeleton/hand21.py`, whose skeleton links thumb1 to the wrist.
"""

from __future__ import annotations

MEDIAPIPE_NAMES = (
    "WRIST",
    "THUMB_CMC",
    "THUMB_MCP",
    "THUMB_IP",
    "THUMB_TIP",
    "INDEX_MCP",
    "INDEX_PIP",
    "INDEX_DIP",
    "INDEX_TIP",
    "MIDDLE_MCP",
    "MIDDLE_PIP",
    "MIDDLE_DIP",
    "MIDDLE_TIP",
    "RING_MCP",
    "RING_PIP",
    "RING_DIP",
    "RING_TIP",
    "PINKY_MCP",
    "PINKY_PIP",
    "PINKY_DIP",
    "PINKY_TIP",
)

# rtmlib's hand21 keypoint names, in its own index order. Lifted from
# rtmlib/visualization/skeleton/hand21.py so a version bump that renumbers the
# model fails the test in tests/test_keypoints.py rather than swapping the
# user's fingers in silence.
RTMPOSE_HAND21_NAMES = (
    "wrist",
    "thumb1",
    "thumb2",
    "thumb3",
    "thumb4",
    "forefinger1",
    "forefinger2",
    "forefinger3",
    "forefinger4",
    "middle_finger1",
    "middle_finger2",
    "middle_finger3",
    "middle_finger4",
    "ring_finger1",
    "ring_finger2",
    "ring_finger3",
    "ring_finger4",
    "pinky_finger1",
    "pinky_finger2",
    "pinky_finger3",
    "pinky_finger4",
)

# RTMPose hand21 index -> MediaPipe landmark index. Written out rather than
# generated, so the diff of a renumbering is readable.
RTMPOSE_TO_MEDIAPIPE = (
    0,  # wrist          -> WRIST
    1,  # thumb1         -> THUMB_CMC
    2,  # thumb2         -> THUMB_MCP
    3,  # thumb3         -> THUMB_IP
    4,  # thumb4         -> THUMB_TIP
    5,  # forefinger1    -> INDEX_MCP
    6,  # forefinger2    -> INDEX_PIP
    7,  # forefinger3    -> INDEX_DIP
    8,  # forefinger4    -> INDEX_TIP
    9,  # middle_finger1 -> MIDDLE_MCP
    10,  # middle_finger2 -> MIDDLE_PIP
    11,  # middle_finger3 -> MIDDLE_DIP
    12,  # middle_finger4 -> MIDDLE_TIP
    13,  # ring_finger1   -> RING_MCP
    14,  # ring_finger2   -> RING_PIP
    15,  # ring_finger3   -> RING_DIP
    16,  # ring_finger4   -> RING_TIP
    17,  # pinky_finger1  -> PINKY_MCP
    18,  # pinky_finger2  -> PINKY_PIP
    19,  # pinky_finger3  -> PINKY_DIP
    20,  # pinky_finger4  -> PINKY_TIP
)


# The inverse: MediaPipe landmark index -> the RTMPose index it comes from.
# Reordering is a gather along the landmark axis, so this is the table the
# code actually indexes with; RTMPOSE_TO_MEDIAPIPE above is the one a person
# reads. Deriving one from the other rather than writing both out is what
# keeps them from disagreeing.
MEDIAPIPE_FROM_RTMPOSE = tuple(
    RTMPOSE_TO_MEDIAPIPE.index(m) for m in range(len(RTMPOSE_TO_MEDIAPIPE))
)


def rtmpose_to_mediapipe(array):
    """Reorder RTMPose's landmark axis into MediaPipe order.

    Takes (..., 21, 2) keypoints or (..., 21) scores — the landmark axis is
    the last one of length 21 either way, and both come out of rtmlib's hand
    pipeline needing the same permutation.
    """
    axis = -1 if array.shape[-1] == len(MEDIAPIPE_FROM_RTMPOSE) else -2
    return array.take(MEDIAPIPE_FROM_RTMPOSE, axis=axis)
