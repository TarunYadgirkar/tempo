"""Pixel + depth -> scene-frame metres.

This is the Python mirror of shell/mac-shell/src/core/hand_inject.{h,cpp}. The
two must agree: the shell's unit test (ctest `hand_inject`) pins the C++ half,
and test_geometry.py pins this half against the same numbers.

Frames, once, so the rest of the package can stop restating them:

  image      pixels, u right and v DOWN, origin top-left.
  camera     ARKit: +X right, +Y up, -Z the view direction. Depth is measured
             along the optical axis, so a point d metres out sits at z = -d.
  scene      what the shell's panels live in: the ARKit world with the
             captured origin subtracted. `head` in the export sidecar is the
             camera pose in this frame at the instant the frame was exposed.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# MediaPipe landmark index -> the shell's SB_JOINT_* index. Identity: both
# enumerate wrist, thumb CMC/MCP/IP/TIP, then index/middle/ring/pinky
# MCP/PIP/DIP/TIP. See hand_inject.h for the side-by-side table. The wire
# format carries MediaPipe order and the shell reorders on the way in, so this
# package never needs to apply the mapping — it is here to be asserted.
MEDIAPIPE_TO_SB = tuple(range(21))

WRIST = 0
THUMB_MCP = 2
THUMB_IP = 3
THUMB_TIP = 4
INDEX_MCP = 5
INDEX_TIP = 8
MIDDLE_TIP = 12
RING_TIP = 16
PINKY_MCP = 17
PINKY_TIP = 20
FINGERTIPS = (THUMB_TIP, INDEX_TIP, MIDDLE_TIP, RING_TIP, PINKY_TIP)

# Below this, the thumb is close enough to the palm plane that its side of it
# is noise. Measured in palm widths, so it does not care about hand size.
CHIRALITY_MIN_OFFSET = 0.05
# Below this, the palm is edge-on in the image and the knuckles do not order
# left to right at all. Measured in palm widths squared, for the same reason.
CHIRALITY_MIN_AREA = 0.05


@dataclass(frozen=True)
class Intrinsics:
    """ARKit pinhole parameters, in pixels of image_width x image_height."""

    fx: float
    fy: float
    cx: float
    cy: float
    image_width: float
    image_height: float

    @classmethod
    def from_json(cls, d: dict) -> "Intrinsics":
        return cls(
            fx=float(d["fx"]),
            fy=float(d["fy"]),
            cx=float(d["cx"]),
            cy=float(d["cy"]),
            image_width=float(d["image_width"]),
            image_height=float(d["image_height"]),
        )

    def valid(self) -> bool:
        return (
            self.fx > 1.0
            and self.fy > 1.0
            and self.image_width > 1.0
            and self.image_height > 1.0
        )


def unproject(
    intr: Intrinsics,
    img_w: float,
    img_h: float,
    u: float,
    v: float,
    depth_m: float,
) -> np.ndarray | None:
    """One image pixel at `depth_m` metres -> a camera-space point.

    `u`, `v` are in an image of img_w x img_h, which is usually NOT the
    resolution the intrinsics are expressed in: the phone streams a downscaled
    JPEG and a smaller depth map of the same view, so a per-axis scale carries
    the pixel into intrinsics pixels. Skipping that scale is the mistake that
    puts every landmark at several times its true angle off-axis.

    None for the depth map's 0 "no reading" sentinel or degenerate intrinsics.
    """
    if not depth_m > 0.0 or not np.isfinite(depth_m):
        return None
    if not intr.valid() or img_w <= 0.0 or img_h <= 0.0:
        return None

    px = u * (intr.image_width / img_w)
    py = v * (intr.image_height / img_h)
    return np.array(
        [
            (px - intr.cx) * depth_m / intr.fx,
            -(py - intr.cy) * depth_m / intr.fy,
            -depth_m,
        ],
        dtype=np.float64,
    )


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate `v` by the unit quaternion `q` = (x, y, z, w).

    Rodrigues form, matching vec_math.h's quat_rotate_vec so the Python and
    C++ halves of the pipeline produce the same numbers.
    """
    qx, qy, qz, qw = (float(c) for c in q)
    t = np.array(
        [
            qy * v[2] - qz * v[1] + qw * v[0],
            qz * v[0] - qx * v[2] + qw * v[1],
            qx * v[1] - qy * v[0] + qw * v[2],
        ]
    )
    return np.array(
        [
            v[0] + 2.0 * (qy * t[2] - qz * t[1]),
            v[1] + 2.0 * (qz * t[0] - qx * t[2]),
            v[2] + 2.0 * (qx * t[1] - qy * t[0]),
        ]
    )


def camera_to_scene(
    cam_pos: np.ndarray, cam_quat: np.ndarray, p_cam: np.ndarray
) -> np.ndarray:
    """Camera-space point -> scene frame, given the camera's scene-frame pose."""
    return np.asarray(cam_pos, dtype=np.float64) + quat_rotate(cam_quat, p_cam)


def chirality_from_joints(joints: np.ndarray) -> str | None:
    """"left" / "right" from the geometry, or None when it is too close to call.

    RTMPose gives 21 points and no handedness, so the hand has to say which it
    is. The palm plane is spanned by (INDEX_MCP - WRIST) and
    (PINKY_MCP - WRIST); in a right-handed frame their cross product points
    out of the PALM for a right hand and out of the BACK for a left one, and
    the thumb is anatomically always on the palmar side. So the sign of the
    thumb's offset along that normal is the chirality, and its magnitude is
    how sure the answer is.

    This is the Python mirror of `palm_thumb_signed_offset` in
    gesture-engine/src/ge_features.cpp, down to averaging the thumb MCP, IP and
    TIP rather than using one of them: the CMC sits in the palm plane and only
    dilutes the signal, and the average rides out the per-joint tracking noise.

    The joints must be in a RIGHT-HANDED frame — the scene frame is one. Hand
    it image pixels or a mirrored frame and every answer flips.
    """
    wrist = joints[WRIST]
    normal = np.cross(joints[INDEX_MCP] - wrist, joints[PINKY_MCP] - wrist)
    length = float(np.linalg.norm(normal))
    palm_width = float(np.linalg.norm(joints[INDEX_MCP] - joints[PINKY_MCP]))
    if length < 1e-9 or palm_width < 1e-6:
        return None
    thumb = (joints[THUMB_MCP] + joints[THUMB_IP] + joints[THUMB_TIP]) / 3.0
    offset = float(np.dot(normal / length, thumb - wrist)) / palm_width
    if abs(offset) < CHIRALITY_MIN_OFFSET:
        return None
    return "right" if offset > 0.0 else "left"


def chirality_from_pixels(landmarks_px: np.ndarray, dorsal: bool = True) -> str | None:
    """"left" / "right" from the image alone, given which face is turned to us.

    Two dimensions cannot tell a left hand from a right one on their own: a
    right hand seen palm-on and a left hand seen back-on project to the same
    picture. What the image does decide is the ORDER of the knuckles around
    the wrist, and one bit of outside knowledge settles the rest.

    The bit is `dorsal`. The phone sits on the user's head and looks where
    they look, so a hand raised to point at a panel is seen from the back —
    dorsal, the default. `dorsal=False` is the selfie-style palm-on view.

    Derivation, in image pixels (u right, v DOWN):
      s = cross_z(INDEX_MCP - WRIST, PINKY_MCP - WRIST)
    For a RIGHT hand held fingers-up with the palm toward the camera, the
    index knuckle is left of the pinky knuckle and both are above the wrist,
    so a = (-, -) and b = (+, -) and s = ax*by - ay*bx > 0. Rotating the hand
    in the image plane cannot change that sign, and mirroring it — the other
    hand, or the other face — is exactly what does. So s > 0 means a right
    hand seen palmar or a left hand seen dorsal, and s < 0 the reverse.

    None when the palm is edge-on, where the knuckles no longer order.
    """
    wrist = landmarks_px[WRIST]
    a = landmarks_px[INDEX_MCP] - wrist
    b = landmarks_px[PINKY_MCP] - wrist
    palm_width = float(np.linalg.norm(landmarks_px[INDEX_MCP] - landmarks_px[PINKY_MCP]))
    if palm_width < 1e-6:
        return None
    area = float(a[0] * b[1] - a[1] * b[0]) / (palm_width * palm_width)
    if abs(area) < CHIRALITY_MIN_AREA:
        return None
    palmar_answer = "right" if area > 0.0 else "left"
    if not dorsal:
        return palmar_answer
    return "left" if palmar_answer == "right" else "right"


def sample_depth(
    depth: np.ndarray, u_norm: float, v_norm: float, window: int = 5
) -> float:
    """Median of the valid readings in a `window` x `window` patch, in metres.

    `u_norm`/`v_norm` are MediaPipe's normalised landmark coordinates. A single
    pixel is the wrong thing to read: the LiDAR map is low resolution and
    speckled with 0 (no reading), and a landmark that lands on one hole would
    otherwise drop out entirely. The median also rejects the depth of whatever
    is BEHIND a finger when the patch straddles its silhouette, which a mean
    would average into a point floating in mid-air.

    0.0 when every sample in the patch is invalid.
    """
    h, w = depth.shape
    cx = int(round(u_norm * (w - 1)))
    cy = int(round(v_norm * (h - 1)))
    r = window // 2
    x0, x1 = max(0, cx - r), min(w, cx + r + 1)
    y0, y1 = max(0, cy - r), min(h, cy + r + 1)
    if x0 >= x1 or y0 >= y1:
        return 0.0
    patch = depth[y0:y1, x0:x1]
    valid = patch[(patch > 0.0) & np.isfinite(patch)]
    if valid.size == 0:
        return 0.0
    return float(np.median(valid))
