# hands — Mac-side hand tracking for the shell

The iPhone streams the camera, the LiDAR depth map and, until now, its own
Apple Vision hand skeleton. That skeleton is the weak link: it drops the hand
on rotation and its fingertips wander by centimetres, so pinch, point and fist
fire late or not at all.

This package moves the tracking to the Mac. MediaPipe Hand Landmarker runs
over the camera frames the shell exports, each landmark takes its range from
the LiDAR depth map, and the resulting metric joints go back into the shell
through `hands-inject` — the same 21 landmarks, the same order, the same scene
frame the phone's `0x05` packet uses. Nothing downstream changes: every
gesture already built on those joints gets the better ones for free.

## Setup

```sh
cd hands
uv sync
./scripts/fetch-model.sh    # hand_landmarker.task, ~7 MB, gitignored
```

## Run it against a live shell

The shell publishes frames only while you ask it to, so the tracker turns the
export on for itself and off again when it exits.

```sh
export SPATIAL_OS_SOCK="$TMPDIR/spatial-os.sock"     # the running Spatula
uv run hands track --dir "$TMPDIR/spatula-frames" --enable-export
```

The export directory must sit inside `$TMPDIR` or `$HOME` — the control socket
is reachable by anything running as you, and the verb writes files.

One line a second says how it is doing:

```
 12.8 fps | detect 0.94 | depth 0.71 | landmark 41.2 ms | e2e 88.4 ms
```

`detect` is the fraction of frames with a hand, `depth` the fraction of
landmarks that got a real LiDAR reading rather than an estimated range, and
`e2e` the phone's capture instant to the shell's ack — the number that decides
whether a pinch feels live.

Check who is driving from the other side:

```sh
nc -U "$SPATIAL_OS_SOCK" <<< "hands status"
# ok overlay=on source=mac age_ms=31
```

`source=mac` for as long as injections keep arriving; 150 ms without one and
the phone's hands take back over, so a tracker that dies degrades instead of
freezing your hands in mid-air.

## Measure it

```sh
uv run hands eval --dir "$TMPDIR/spatula-frames" --enable-export --seconds 60
```

Hold a hand up in view and keep it still for stretches of a second or two.
The run records both sources against the same frames — the Mac's own output
and, through `hands dump`, whatever the shell is currently feeding the gesture
engine — and writes `results/eval-<timestamp>.json` plus a markdown summary
comparing:

- **jitter**, the mean distance a joint moves between frames while the wrist is
  held still, which is motion you did not make and what makes a pinch
  threshold chatter;
- **detect rate**, the fraction of frames each source had a hand at all;
- **depth validity**, how much of the Mac's metric 3D is measured by the LiDAR
  and how much is inferred from MediaPipe's hand model.

Injection stays off during an eval by default, so the phone's hands remain the
live source and the two columns are genuinely two trackers. `--inject` turns
it on, and the report flags the run if the shell reported a mac source while
recording.

## How a landmark becomes a point in the room

1. `HandLandmarker` (VIDEO mode, 2 hands) gives 21 normalised image
   coordinates and a `world` skeleton in metres centred on the hand.
2. Each landmark medians a 5x5 patch of the depth map at its pixel, skipping
   the `0` holes.
3. A landmark with a reading unprojects through the intrinsics into camera
   space. One without takes its **range** from the world skeleton relative to
   a landmark that did get one (wrist, else index MCP), and keeps its own
   measured direction. This is the common case, not the exception: the depth
   map is a few dozen pixels across, so a fingertip at arm's length is
   sub-pixel.
4. The scene-frame head pose the shell stamped on the frame carries the point
   into the frame the panels live in.
5. One Euro per axis per landmark removes the residual jitter, smoothing hard
   while the hand is still and getting out of the way when it moves.

Frames, so nothing has to be guessed: image pixels have `v` pointing **down**;
ARKit camera space is `+X` right, `+Y` up, `-Z` the view direction, with depth
measured along the optical axis; the scene frame is the ARKit world with the
shell's captured origin subtracted. The intrinsics are in ARKit's capture
pixels, which are larger than the streamed JPEG — `geometry.unproject` rescales,
and forgetting that step is what puts every landmark at several times its true
angle off-axis.

## Joint order

MediaPipe's 21 landmarks and the shell's `SB_JOINT_*` enumerate the same
skeleton in the same sequence, so the mapping is the identity:

| index | MediaPipe | `SB_JOINT_*` |
| --- | --- | --- |
| 0 | wrist | `WRIST` |
| 1–4 | thumb CMC, MCP, IP, TIP | `THUMB_CMC`…`THUMB_TIP` |
| 5–8 | index MCP, PIP, DIP, TIP | `INDEX_MCP`…`INDEX_TIP` |
| 9–12 | middle MCP, PIP, DIP, TIP | `MIDDLE_MCP`…`MIDDLE_TIP` |
| 13–16 | ring MCP, PIP, DIP, TIP | `RING_MCP`…`RING_TIP` |
| 17–20 | pinky MCP, PIP, DIP, TIP | `PINKY_MCP`…`PINKY_TIP` |

It is written out as a table on both sides anyway (here, and
`shell/mac-shell/src/core/hand_inject.cpp`) and pinned by a test on both
sides, because "they happen to agree today" is exactly the fact that stops
being true silently.

## Layout

| file | what it owns |
| --- | --- |
| `geometry.py` | pixel + depth → camera → scene, and depth patch sampling |
| `export_reader.py` | polling the shell's export directory |
| `tracker.py` | MediaPipe, the depth fusion, the filter |
| `onefilter.py` | One Euro |
| `control.py` | the shell's line protocol, including the message split |
| `track.py` | the live loop |
| `evaluate.py` | recording both sources and the jitter metric |

## Tests

```sh
uv run pytest
```

`geometry.py`'s numbers are also asserted in the shell's own suite
(`ctest -R hand_inject`), so the C++ and Python halves of the same math keep
agreeing. The verbs themselves are covered end to end over a real socket by
`ctest -R hands_e2e`.

## Handedness

MediaPipe labels handedness as if it were looking at a mirror, which is right
for a selfie camera and backwards for the phone's rear camera. The label is
flipped by default; `--no-flip-handedness` keeps it as MediaPipe gave it. Left
lands in the shell's hand slot 0 and right in slot 1 — a fixed assignment,
which is already an improvement on the phone's first-seen-order slots.
