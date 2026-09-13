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
THUMB_TIP = 4
INDEX_MCP = 5
INDEX_TIP = 8
MIDDLE_TIP = 12
RING_TIP = 16
PINKY_TIP = 20
FINGERTIPS = (THUMB_TIP, INDEX_TIP, MIDDLE_TIP, RING_TIP, PINKY_TIP)


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
