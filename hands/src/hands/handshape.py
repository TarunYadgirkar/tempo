"""One range for the hand, a rigid model for the other twenty joints.

Sampling the LiDAR map under every landmark separately is what made the Mac
path jitter. The exported map is 256x192 over a 1920x1440 capture, so one
depth pixel covers 7.5 capture pixels: a 5x5 patch centred on a fingertip
spans roughly 14 mm of the scene at half a metre, which is wider than the
finger. Whenever the patch crosses the silhouette the median jumps from the
finger to whatever is behind it, and a joint teleports by the depth of the
room. That is a metre of error arriving and leaving at the frame rate, and no
amount of smoothing on the output fixes it, because the error is not noise
around the truth — it is a different surface.

So the depth map is asked ONE question instead of twenty-one: how far away is
the hand? The palm is the part of the hand that is many depth pixels across
and never straddles a silhouette in the middle, so the range comes from the
pooled valid samples under the wrist and the four finger MCPs. Every joint
then keeps its own (sharp, model-supplied) pixel ray, and its range comes from
a hand whose bones have known lengths.

Two ways to get those ranges, because the backends offer different things:

  world is None (RTMPose)  The palm joints sit on one frontoparallel plane at
      the measured range, and each finger segment is solved outward: the ray
      through the distal landmark, intersected with the sphere of the bone's
      length around the proximal joint. Two roots, one nearer and one farther;
      fingers curl toward the palm, and the head-mounted camera sees the BACK
      of a raised hand, so curling carries a fingertip AWAY and the farther
      root is the anatomical one. `dorsal=False` takes the nearer.
  world given (MediaPipe)  Its metric skeleton already knows the shape, so it
      only needs placing: the world z offsets are scaled by the hand's
      apparent size and added to the measured range, which slides the whole
      skeleton until its palm centroid sits where the LiDAR says it is.

Bone lengths are adult means in metres, scaled per-frame by the hand's
apparent palm width, so a smaller hand or a child's is not forced to an adult
skeleton. They are lengths, never directions: the picture supplies every
direction, and this file only answers "how far".
"""

from __future__ import annotations

import numpy as np

from .geometry import (
    INDEX_MCP,
    MIDDLE_MCP,
    PINKY_MCP,
    RING_MCP,
    WRIST,
    Intrinsics,
    depth_patch_values,
)

# Landmarks treated as one rigid frontoparallel plate at the measured range.
# The wrist, the thumb's CMC and the four finger knuckles: the part of the
# hand that does not articulate against itself.
PALM_IDS = (WRIST, 1, INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP)
# Sampled for the range itself. The thumb CMC is left out — it sits at the
# edge of the hand, where the patch is as likely to read the background.
RANGE_IDS = (WRIST, INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP)

# Adult mean segment lengths, metres, keyed (proximal landmark, distal
# landmark) in MediaPipe order. Phalanx values follow the published adult
# means (Buryanov & Kotiuk 2010, mixed-sex means rounded to the millimetre);
# the thumb metacarpal is the first-ray length from the same source. They are
# a SHAPE, not a size: PALM_WIDTH_M below is the reference this shape is drawn
# at, and every length is multiplied by the hand's own apparent palm width
# divided by that reference.
BONE_M = {
    (1, 2): 0.046,   # thumb metacarpal, CMC -> MCP
    (2, 3): 0.032,   # thumb proximal phalanx
    (3, 4): 0.024,   # thumb distal phalanx
    (5, 6): 0.040,   # index proximal
    (6, 7): 0.025,   # index middle
    (7, 8): 0.020,   # index distal
    (9, 10): 0.045,  # middle proximal
    (10, 11): 0.028,
    (11, 12): 0.021,
    (13, 14): 0.041,  # ring proximal
    (14, 15): 0.027,
    (15, 16): 0.020,
    (17, 18): 0.033,  # pinky proximal
    (18, 19): 0.019,
    (19, 20): 0.018,
}

CHAINS = ((1, 2, 3, 4), (5, 6, 7, 8), (9, 10, 11, 12), (13, 14, 15, 16), (17, 18, 19, 20))

# Index MCP to pinky MCP on the hand BONE_M is drawn for. The measured span
# between those two landmarks divided by this is the per-frame size scale.
PALM_WIDTH_M = 0.080
# The scale is clamped, because the span it comes from shrinks under
# foreshortening as well as under a smaller hand, and a palm turned edge-on
# would otherwise collapse the skeleton to nothing.
SIZE_SCALE_MIN = 0.70
SIZE_SCALE_MAX = 1.40
# No joint is placed closer than this, whatever the algebra says.
MIN_RANGE_M = 0.05


def palm_range_m(
    depth: np.ndarray,
    landmarks_px: np.ndarray,
    img_w: float,
    img_h: float,
    window: int = 5,
) -> tuple[float, int]:
    """The hand's range in metres, plus how many LiDAR samples voted for it.

    The five patches are pooled BEFORE the median rather than medianed one at
    a time and averaged: pooling lets a knuckle that landed in a hole
    contribute nothing instead of contributing a guess, and lets one patch
    that straddled the silhouette be outvoted by the other four.

    (0.0, 0) when the hand got no reading at all.
    """
    pooled: list[np.ndarray] = []
    for j in RANGE_IDS:
        u, v = landmarks_px[j]
        pooled.append(depth_patch_values(depth, u / img_w, v / img_h, window))
    values = np.concatenate(pooled) if pooled else np.empty(0)
    if values.size == 0:
        return 0.0, 0
    return float(np.median(values)), int(values.size)


def ray_from_pixel(
    intr: Intrinsics, img_w: float, img_h: float, u: float, v: float
) -> np.ndarray:
    """The unit-depth ray through one image pixel: (x, y_down, 1) per metre.

    Same rescale into intrinsics pixels that `unproject` does, and the same
    frame up to the sign flips on y and z — which are a rotation, so distances
    between two points on two of these rays are camera-frame distances.
    """
    px = float(u) * (intr.image_width / img_w)
    py = float(v) * (intr.image_height / img_h)
    return np.array(
        [(px - intr.cx) / intr.fx, (py - intr.cy) / intr.fy, 1.0], dtype=np.float64
    )


def observed_palm_m(rays: np.ndarray, range_m: float) -> float:
    """Index MCP to pinky MCP in metres, for a palm at `range_m`."""
    return float(np.linalg.norm(rays[INDEX_MCP] - rays[PINKY_MCP])) * range_m


def size_scale(observed_m: float, reference_m: float) -> float:
    """How big this hand is against the one the shape model is drawn for."""
    if not observed_m > 0.0 or not reference_m > 0.0:
        return 1.0
    return float(np.clip(observed_m / reference_m, SIZE_SCALE_MIN, SIZE_SCALE_MAX))


def solve_segment(
    r0: np.ndarray, d0: float, r1: np.ndarray, bone_m: float, dorsal: bool
) -> float:
    """Range of the distal joint: its ray, meeting the bone's sphere.

    |d1*r1 - d0*r0| = bone gives a quadratic in d1. Both roots are on the
    ray, one in front of the proximal joint's depth and one behind it, and
    `dorsal` picks which — see the module docstring. A segment whose apparent
    length in the picture already exceeds the bone has no root at all (the
    hand is bigger than the model says, or a landmark is wrong); the vertex is
    the closest the ray ever comes to the sphere, so it is the least-wrong
    answer rather than a dropped joint.
    """
    a = float(np.dot(r1, r1))
    b = -2.0 * d0 * float(np.dot(r0, r1))
    c = d0 * d0 * float(np.dot(r0, r0)) - bone_m * bone_m
    vertex = -b / (2.0 * a)
    disc = b * b - 4.0 * a * c
    if disc <= 0.0:
        return max(MIN_RANGE_M, vertex)
    half = float(np.sqrt(disc)) / (2.0 * a)
    return max(MIN_RANGE_M, vertex + half if dorsal else vertex - half)


def rigid_ranges(
    intr: Intrinsics,
    img_w: float,
    img_h: float,
    landmarks_px: np.ndarray,
    range_m: float,
    world: np.ndarray | None = None,
    dorsal: bool = True,
) -> np.ndarray:
    """(21,) ranges in metres for one hand, from ONE measured range.

    The pixels are untouched: this only answers how far along each landmark's
    own ray its joint sits.
    """
    rays = np.array(
        [ray_from_pixel(intr, img_w, img_h, u, v) for u, v in landmarks_px]
    )
    observed = observed_palm_m(rays, range_m)
    if world is not None:
        return _world_ranges(world, range_m, observed)

    scale = size_scale(observed, PALM_WIDTH_M)
    depths = np.full(landmarks_px.shape[0], float(range_m), dtype=np.float64)
    for chain in CHAINS:
        for proximal, distal in zip(chain, chain[1:]):
            depths[distal] = solve_segment(
                rays[proximal],
                depths[proximal],
                rays[distal],
                BONE_M[(proximal, distal)] * scale,
                dorsal,
            )
    return depths


def _world_ranges(
    world: np.ndarray, range_m: float, observed_m: float
) -> np.ndarray:
    """MediaPipe's metric skeleton, slid until its palm sits at `range_m`.

    Its world z grows away from the camera in the same sense the depth map
    measures, so a world z difference is a range difference. Only the offset
    and the size are being supplied here; the shape is the model's — and the
    size is measured against the model's OWN palm width rather than the adult
    mean, since this skeleton already has one.
    """
    world_palm = float(
        np.linalg.norm(world[INDEX_MCP] - world[PINKY_MCP])
    )
    scale = size_scale(observed_m, world_palm)
    centroid_z = float(np.mean(world[list(PALM_IDS), 2]))
    return np.maximum(
        MIN_RANGE_M, range_m + scale * (world[:, 2] - centroid_z)
    )
