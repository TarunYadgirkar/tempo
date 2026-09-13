"""`hands check` — is the 3D the tracker sends back where the picture says?

Two independent questions, because they fail differently and a single number
would hide one behind the other.

  Reprojection. Take a pixel, give it a range, unproject it into the scene
  frame, then put it back through the SAME intrinsics and the SAME head pose.
  A joint that survives that round trip to well under a pixel proves the
  camera model — the capture-pixel rescale, the principal point, the ARKit
  y/z sign flips, the quaternion — agrees with itself. It says nothing about
  whether the range was right, and that is the point: under the rigid shape
  model every joint stays on its own pixel ray whatever range it is given, so
  a range error CANNOT move the overlay in the camera that took the frame. It
  can only move it in a camera that has since moved.

  Registration lag. What actually separates the drawn skeleton from the real
  hand is time. The joints are smoothed in the scene frame before they are
  sent, the shell holds them until the next injection, and the renderer draws
  them over a NEWER passthrough frame. Every millisecond of that is the hand
  drawn where it used to be. This reports it as the pixel offset it becomes,
  at the hand's own measured range, so it can be compared against what the
  user sees rather than against a millisecond budget nobody can picture.

The phone's own skeleton is the reference for the 3D itself, and it can only
be read when a hand is actually in view of a connected phone.
"""

from __future__ import annotations

import math

import numpy as np

from .control import ShellControl, ShellError
from .export_reader import ExportReader, ExportedFrame
from .geometry import project, scene_to_camera
from .tracker import tracker_from_args

# A round trip is pure float error; anything above this is a real disagreement
# between the forward and inverse camera models.
REPROJECT_TOL_PX = 0.5
# Hand speeds the lag figure is reported at. A pinch travels at roughly the
# first, a point-and-sweep at the second.
REPORT_SPEEDS_MS = (0.3, 1.0)
# What the smoothing stage alone may cost, at the slower of those speeds and
# half a metre. A fingertip reticle is about 12 px across at that range, so
# past this the mark no longer touches the finger it is marking.
LAG_BUDGET_PX = 15.0
GRID = 8


def _grid_points(frame: ExportedFrame) -> np.ndarray:
    """Pixels spread over the frame that have a LiDAR reading behind them."""
    dh, dw = frame.depth.shape
    out = []
    for gy in range(1, GRID):
        for gx in range(1, GRID):
            u = frame.width * gx / GRID
            v = frame.height * gy / GRID
            d = float(frame.depth[int(v / frame.height * dh), int(u / frame.width * dw)])
            if d > 0.0 and math.isfinite(d):
                out.append((u, v, d))
    return np.array(out, dtype=np.float64).reshape(-1, 3)


def reprojection_error_px(
    frame: ExportedFrame, pixels: np.ndarray, depths: np.ndarray
) -> np.ndarray:
    """Per-point |reprojected - original| in frame pixels."""
    from .geometry import camera_to_scene, unproject

    errors = []
    for (u, v), d in zip(pixels, depths):
        p_cam = unproject(frame.intrinsics, frame.width, frame.height, u, v, d)
        if p_cam is None:
            continue
        scene = camera_to_scene(frame.head_pos, frame.head_quat, p_cam)
        back = project(
            frame.intrinsics,
            frame.width,
            frame.height,
            scene_to_camera(frame.head_pos, frame.head_quat, scene),
        )
        if back is None:
            continue
        errors.append(float(np.hypot(back[0] - u, back[1] - v)))
    return np.array(errors, dtype=np.float64)


def filter_lag_s(min_cutoff: float, beta: float, speed_m_s: float) -> float:
    """Steady-state lag of the One Euro stage on a hand moving at `speed`.

    A first-order low-pass tracking a ramp settles one time constant behind
    it, and One Euro's cutoff at a given speed is min_cutoff + beta * speed.
    """
    cutoff = min_cutoff + beta * speed_m_s
    return 1.0 / (2.0 * math.pi * cutoff)


def px_per_metre(frame: ExportedFrame, range_m: float) -> float:
    """How many frame pixels a metre of lateral motion covers at `range_m`."""
    f_px = frame.intrinsics.fx * (frame.width / frame.intrinsics.image_width)
    return f_px / max(range_m, 1e-3)


def _phone_hands(shell: ShellControl) -> tuple[str, dict[int, np.ndarray]]:
    dump = shell.hands_dump()
    hands = {
        int(h["slot"]): np.array(h["joints"], dtype=np.float64)[:, :3]
        for h in dump.get("hands", [])
    }
    return str(dump.get("source", "?")), hands


def run_check(args) -> int:
    frame = ExportReader(args.dir).read()
    if frame is None:
        # The reader yields each frame once; a directory nobody is writing to
        # still has a whole frame set sitting in it, and re-reading it is the
        # only way to check a saved fixture.
        reader = ExportReader(args.dir)
        reader._last_mtime_ns = -1
        frame = reader.read()
    if frame is None:
        print(f"hands check: no frame in {args.dir}")
        return 1
    if not frame.ready_for_metric_3d():
        print("hands check: frame has no intrinsics, head pose or depth map")
        return 1

    print(
        f"frame seq={frame.seq} {frame.width}x{frame.height} "
        f"depth {frame.depth.shape[1]}x{frame.depth.shape[0]} "
        f"intrinsics {frame.intrinsics.image_width:.0f}x"
        f"{frame.intrinsics.image_height:.0f}"
    )

    grid = _grid_points(frame)
    ok = True
    if grid.size:
        err = reprojection_error_px(frame, grid[:, :2], grid[:, 2])
        print(
            f"depth-grid round trip: n={err.size} "
            f"mean={err.mean():.4f} px max={err.max():.4f} px"
        )
        ok = ok and bool(err.max() < REPROJECT_TOL_PX)
    else:
        print("depth-grid round trip: no valid depth samples")

    range_m = 0.5
    with tracker_from_args(args) as tracker:
        result = tracker.track(frame)
    if result.live:
        for hand in result.live:
            if hand.range_m > 0.0:
                range_m = hand.range_m
            pixels = np.array(
                [
                    project(
                        frame.intrinsics,
                        frame.width,
                        frame.height,
                        scene_to_camera(frame.head_pos, frame.head_quat, j),
                    )
                    for j in hand.joints
                ],
                dtype=np.float64,
            )
            print(
                f"{hand.chirality}: range {hand.range_m:.3f} m "
                f"({hand.range_samples} LiDAR samples), "
                f"landmark span {np.ptp(pixels[:, 0]):.0f}x"
                f"{np.ptp(pixels[:, 1]):.0f} px"
            )
    else:
        print("no hand in this frame: landmark reprojection not checked")

    ppm = px_per_metre(frame, range_m)
    print(f"at {range_m:.2f} m, 1 cm of hand motion = {ppm / 100.0:.1f} px")
    for speed in REPORT_SPEEDS_MS:
        lag = filter_lag_s(args.min_cutoff, args.beta, speed)
        offset_px = ppm * lag * speed
        flag = ""
        if speed == REPORT_SPEEDS_MS[0]:
            ok = ok and offset_px <= LAG_BUDGET_PX
            flag = "" if offset_px <= LAG_BUDGET_PX else "  OVER BUDGET"
        print(
            f"  smoothing lag at {speed:.1f} m/s: {1000 * lag:5.1f} ms "
            f"= {100 * lag * speed:4.1f} cm = {ppm * lag * speed:4.0f} px{flag}"
        )

    try:
        with ShellControl(args.sock) as shell:
            source, hands = _phone_hands(shell)
            status = shell.hands_status()
    except ShellError as exc:
        print(f"phone comparison: no shell ({exc})")
        return 0 if ok else 1

    print(f"shell: source={source} e2e_ms={status.get('e2e_ms')}")
    if source != "phone":
        print("phone comparison: shell is serving injected hands, not the phone")
    elif not hands:
        print("phone comparison: no hand in view of the phone")
    elif not result.live:
        print("phone comparison: no hand found by this tracker")
    else:
        for hand in result.live:
            slot = 0 if hand.chirality == "left" else 1
            if slot not in hands:
                continue
            d = np.linalg.norm(hands[slot] - hand.joints, axis=1)
            print(
                f"phone vs mac ({hand.chirality}): mean {1000 * d.mean():.0f} mm "
                f"max {1000 * d.max():.0f} mm"
            )
    return 0 if ok else 1
