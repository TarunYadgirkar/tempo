"""Mac-side hand tracking for the Spatula shell.

MediaPipe Hand Landmarker runs here, over the camera frames the shell exports
(`frame-export`), with fingertip range from the streamed LiDAR depth map. The
joints go back in through `hands-inject` in the same 21-landmark order and the
same scene frame the phone's 0x05 hand packet uses, so every gesture already
built on those joints benefits without knowing the source changed.

Entry point: `uv run hands track --dir <export dir>`.
"""

from .cli import main

__all__ = ["main"]
