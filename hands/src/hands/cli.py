"""`uv run hands <command>` — the mac-side hand tracker and its evaluation."""

from __future__ import annotations

import argparse
import sys

from .backends import BACKENDS, DEFAULT_BACKEND, DEFAULT_MODEL
from .evaluate import run_eval
from .track import run_track


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
        "--min-cutoff",
        type=float,
        default=1.0,
        help="One Euro cutoff floor in Hz — lower is smoother and laggier",
    )
    p.add_argument(
        "--beta",
        type=float,
        default=0.5,
        help="One Euro speed coupling — higher tracks fast motion more closely",
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
    ev.add_argument("--seconds", type=float, default=60.0)
    ev.add_argument(
        "--inject",
        action="store_true",
        help=(
            "also inject while recording. Off by default so the phone's hands "
            "stay the live source and both sources can be sampled independently"
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
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "track":
        return run_track(args)
    return run_eval(args)


if __name__ == "__main__":
    sys.exit(main())
