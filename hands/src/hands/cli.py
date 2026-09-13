"""`uv run hands <command>` — the mac-side hand tracker and its evaluation."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

from .backends import BACKENDS, DEFAULT_BACKEND, DEFAULT_MODEL
from .check import run_check
from .evaluate import DEFAULT_WINDOW_S, run_eval
from .track import run_track
from .tracker import (
    DEFAULT_DEPTH_MODE,
    DEFAULT_HOLD_FRAMES,
    DEFAULT_MAX_RANGE_M,
    DEFAULT_MIN_CONFIDENCE,
    DEPTH_MODES,
)


def _common(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--dir",
        required=True,
        help="the directory `frame-export on <dir>` is writing into",
    )
    p.add_argument(
        "--sock",
        default=None,
        help="shell control socket (default: $SPATIAL_OS_SOCK)",
    )
    p.add_argument(
        "--backend",
        choices=BACKENDS,
        default=DEFAULT_BACKEND,
        help=(
            "which model finds the landmarks. rtmpose is RTMDet-nano + "
            "RTMPose-m through rtmlib, several times faster and sharper than "
            "mediapipe's 2020 model; mediapipe is the original path, kept so "
            "the two can be compared on the same frames"
        ),
    )
    p.add_argument(
        "--device",
        default="auto",
        help=(
            "rtmpose only: auto (CoreML for the pose model, CPU if it is "
            "refused), mps, or cpu"
        ),
    )
    p.add_argument(
        "--det-interval",
        type=int,
        default=10,
        help=(
            "rtmpose only: frames between hand DETECTIONS. In between, the "
            "hand is looked for inside the box it was in last frame, which is "
            "most of the speed-up"
        ),
    )
    p.add_argument("--model", default=str(DEFAULT_MODEL), help="hand_landmarker.task")
    p.add_argument(
        "--raw",
        action="store_true",
        help=(
            "ask the shell for raw decoded pixels (latest.rgb) instead of a "
            "JPEG, which drops an encode on its side and a decode on ours. "
            "Only meaningful with --enable-export"
        ),
    )
    p.add_argument(
        "--no-flip-handedness",
        dest="flip_handedness",
        action="store_false",
        help=(
            "keep MediaPipe's handedness labels as-is. They assume a mirrored "
            "selfie image, so they are flipped by default for the phone's rear "
            "camera"
        ),
    )
    p.add_argument(
        "--palmar-view",
        dest="dorsal_view",
        action="store_false",
        help=(
            "the camera sees the PALMS of the hands rather than their backs. "
            "Only used when the model gives no handedness of its own (rtmpose "
            "does not) and the hand is too flat in depth to tell: two "
            "dimensions cannot separate a left hand from a right one without "
            "knowing which face is turned to the camera. The default suits "
            "the phone's head-mounted rear camera, which sees a raised hand "
            "from the back"
        ),
    )
    p.add_argument(
        "--depth",
        choices=DEPTH_MODES,
        default=DEFAULT_DEPTH_MODE,
        help=(
            "how a joint gets its range. rigid takes ONE range for the whole "
            "hand from the palm and places the other twenty joints with a "
            "hand-shape model; per-joint reads the depth map under every "
            "landmark, which is the older path and the one rigid has to beat"
        ),
    )
    p.add_argument(
        "--min-cutoff",
        type=float,
        default=2.0,
        help="One Euro cutoff floor in Hz — lower is smoother and laggier",
    )
    p.add_argument(
        "--beta",
        type=float,
        default=10.0,
        help=(
            "One Euro speed coupling on the joints, in Hz per metre/second. "
            "The joint speeds this sees are in m/s, so a beta under ~1 never "
            "lifts the cutoff off its floor at any speed a hand moves at and "
            "the filter degenerates into a fixed low-pass: at the old 0.02 a "
            "hand moving 0.3 m/s was drawn 4.7 cm — about 45 px at half a "
            "metre — behind itself, whether it was moving or not. `hands "
            "check` reports that offset in pixels"
        ),
    )
    p.add_argument(
        "--range-min-cutoff",
        type=float,
        default=1.5,
        help=(
            "One Euro cutoff floor on the hand's RANGE, which every joint "
            "depends on. Lower than the joints' because an arm changes its "
            "distance slowly and the LiDAR's ranging noise does not"
        ),
    )
    p.add_argument(
        "--range-beta",
        type=float,
        default=2.0,
        help=(
            "One Euro speed coupling on the hand's range, in Hz per "
            "metre/second. Same failure as --beta: at the old 0.005 the "
            "range never left its 0.4 Hz floor, so an arm reaching out at "
            "0.5 m/s had its hand placed 20 cm short until it stopped"
        ),
    )
    p.add_argument(
        "--hold-frames",
        type=int,
        default=DEFAULT_HOLD_FRAMES,
        help=(
            "frames a hand keeps being sent after the model stops finding it, "
            "so one dropped detection mid-pinch does not read downstream as "
            "the hand leaving. 0 disables the hold"
        ),
    )
    p.add_argument(
        "--depth-window",
        type=int,
        default=5,
        help="side of the square patch each landmark medians its depth over",
    )
    p.add_argument(
        "--fallback-depth-m",
        type=float,
        default=0.0,
        help=(
            "range to assume when no landmark got a LiDAR reading at all. 0 "
            "(the default) drops the hand instead of inventing a distance"
        ),
    )


def _gates(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--max-range-m",
        type=float,
        default=DEFAULT_MAX_RANGE_M,
        help=(
            "drop a detected hand further than this from the camera, before "
            "it touches the range filter. The wearer's hand is within arm's "
            "reach; further is someone else or a false positive. 0 disables"
        ),
    )
    p.add_argument(
        "--min-confidence",
        type=float,
        default=DEFAULT_MIN_CONFIDENCE,
        help="drop a detected hand whose model confidence is below this",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="hands",
        description=(
            "Track hands on the Mac with MediaPipe over the camera frames the "
            "shell exports, and inject the joints back into the gesture path."
        ),
    )
    sub = parser.add_subparsers(dest="command", required=True)

    track = sub.add_parser(
        "track", help="run the tracker and feed the shell (the live path)"
    )
    _common(track)
    _gates(track)
    track.add_argument(
        "--enable-export",
        action="store_true",
        help="turn `frame-export` on for --dir before starting, and off after",
    )
    track.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="stop after this long (0 = until interrupted)",
    )

    ev = sub.add_parser(
        "eval",
        help="record the phone's hands and the Mac's side by side and compare",
    )
    _common(ev)
    _gates(ev)
    ev.add_argument("--seconds", type=float, default=60.0)
    ev.add_argument(
        "--window-s",
        type=float,
        default=DEFAULT_WINDOW_S,
        help=(
            "length of one alternating window. The run flips between "
            "injection OFF (the shell's hands are the phone's, so the phone "
            "column is clean) and injection ON (the shell's hands are this "
            "tracker's, which is the mac column measured on the live path)"
        ),
    )
    ev.add_argument(
        "--inject",
        action="store_true",
        help=(
            "legacy: inject for the WHOLE run instead of alternating. Makes "
            "the phone column meaningless, since `hands dump` then returns "
            "the Mac's own joints — kept only to reproduce the old numbers"
        ),
    )
    ev.add_argument(
        "--out",
        default=None,
        help="results JSON path (default: results/eval-<timestamp>.json)",
    )
    ev.add_argument(
        "--enable-export",
        action="store_true",
        help="turn `frame-export` on for --dir before starting, and off after",
    )

    check = sub.add_parser(
        "check",
        help=(
            "one frame, end to end: reproject the joints through the same "
            "camera that made them, report the registration lag in pixels, "
            "and compare against the phone's own skeleton"
        ),
    )
    _common(check)
    _gates(check)
    cal = sub.add_parser("calibrate", help="tune pinch/fist thresholds to your hand and write gestures.toml")
    cal.add_argument("--sock", default=None)
    cal.add_argument("--write", action="store_true", help="write ~/.config/spatial-os/gestures.toml")
    cal.add_argument("--phase-seconds", type=float, default=6.0)
    cal.add_argument("--results", default=str(Path(__file__).resolve().parents[2] / "results"))
    return parser


def run_calibrate(args) -> int:
    from .calibrate import run

    if args.sock:
        os.environ["SPATIAL_OS_SOCK"] = args.sock
    run(args.write, Path(args.results), print, args.phase_seconds)
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "track":
        return run_track(args)
    if args.command == "check":
        return run_check(args)
    if args.command == "calibrate":
        return run_calibrate(args)
    return run_eval(args)


if __name__ == "__main__":
    sys.exit(main())
