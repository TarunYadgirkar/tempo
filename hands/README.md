# hands — Mac-side hand tracking for the shell

The iPhone streams the camera, the LiDAR depth map and, until now, its own
Apple Vision hand skeleton. That skeleton is the weak link: it drops the hand
on rotation and its fingertips wander by centimetres, so pinch, point and fist
fire late or not at all.

This package moves the tracking to the Mac. A hand model runs over the camera
frames the shell exports, each landmark takes its range from the LiDAR depth
map, and the resulting metric joints go back into the shell through
`hands-inject` — the same 21 landmarks, the same order, the same scene frame
the phone's `0x05` packet uses. Nothing downstream changes: every gesture
already built on those joints gets the better ones for free.

## Two models

`--backend rtmpose` (the default) runs OpenMMLab's RTMDet-nano hand detector
and RTMPose-m hand through rtmlib on ONNX Runtime. `--backend mediapipe` runs
Google's 2020 Hand Landmarker, which is what this package started on and is
kept so the two can be compared over the same frames.

On this Mac, per frame of a 640x480 export:

| | model | whole loop | frames/s |
| --- | --- | --- | --- |
| rtmpose | 5.7 ms | 7.3 ms | 30.0 |
| mediapipe | 11.3 ms | 12.6 ms | 30.0 |

Both saturate the 30 Hz export, so the number that moved is the latency, not
the frame rate. RTMPose gets there by running its detector only every tenth
frame (`--det-interval`) and looking for the hand inside last frame's box in
between, and by running the pose model on CoreML — which the detector cannot
use, because its ONNX export carries grid-decode ops CoreML infers at the
wrong rank and ONNX Runtime raises on rather than falling back.

RTMPose gives 21 points and nothing else: no handedness, and no metric hand
model for the landmarks that miss the LiDAR. See **Handedness** and **How a
landmark becomes a point in the room** below for what fills both gaps.

## Setup

```sh
cd hands
uv sync
./scripts/fetch-model.sh    # hand_landmarker.task, ~7 MB, gitignored
```

`fetch-model.sh` is only needed for `--backend mediapipe`. rtmlib downloads
its own two ONNX models (4 MB + 55 MB) into `~/.cache/rtmlib` the first time
it runs.

## Run it against a live shell

The shell publishes frames only while you ask it to, so the tracker turns the
export on for itself and off again when it exits.

```sh
export SPATIAL_OS_SOCK="$TMPDIR/spatial-os.sock"     # the running Spatula
uv run hands track --dir "$TMPDIR/spatula-frames" --enable-export --raw
```

`--raw` asks the shell for `latest.rgb`, the decoded pixels downscaled to 640
wide, instead of a JPEG it would have to encode and this process would have to
decode again. Without it the export is the original JPEG, which still works.

The export directory must sit inside `$TMPDIR` or `$HOME` — the control socket
is reachable by anything running as you, and the verb writes files.

One line a second says how it is doing:

```
 30.0 fps | detect 1.00 | depth 0.78 | landmark   5.7 ms | e2e    7.3 ms
```

`detect` is the fraction of frames with a hand, `depth` the fraction of
landmarks that got a real LiDAR reading rather than an estimated range, and
`e2e` the instant the shell published the frame to the moment it acked the
injection made from it — the number that decides whether a pinch feels live.

Both ends of `e2e` are this Mac's realtime clock: the shell stamps `export_ns`
into the sidecar and the tracker echoes it back in `hands-inject`, so the
shell can report the same number from its side. The frame's own `t_ns` is the
phone's capture clock, and subtracting one from the other would measure a
clock offset rather than a latency.

Check who is driving from the other side:

```sh
nc -U "$SPATIAL_OS_SOCK" <<< "hands status"
# ok overlay=on source=mac age_ms=31 e2e_ms=38
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

1. The backend gives 21 landmarks in the exported image's own pixels, already
   in MediaPipe order. MediaPipe also gives a `world` skeleton in metres
   centred on the hand; RTMPose does not.
2. Each landmark medians a 5x5 patch of the depth map at its pixel, skipping
   the `0` holes.
3. A landmark with a reading unprojects through the intrinsics into camera
   space. One without takes its **range** from a landmark that did get one
   (wrist, else index MCP) and keeps its own measured direction. With
   MediaPipe the world skeleton shapes that estimate, so the fingers keep
   their relative depth; with RTMPose there is no hand model to shape it and
   the estimate flattens to the reference landmark's own range. This is the
   common case, not the exception: the depth map is a few dozen pixels across,
   so a fingertip at arm's length is sub-pixel.
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
| `geometry.py` | pixel + depth → camera → scene, depth sampling, chirality |
| `keypoints.py` | the RTMPose ↔ MediaPipe landmark tables |
| `export_reader.py` | polling the shell's export directory, both formats |
| `backends/` | the two models, behind one `detect(rgb, t_ns)` |
| `tracker.py` | the depth fusion, the unprojection, the filter |
| `onefilter.py` | One Euro |
| `control.py` | the shell's line protocol, including the message split |
| `track.py` | the live loop |
| `evaluate.py` | recording both sources and the jitter metric |

## Tests

```sh
uv run pytest
```

`test_keypoints.py` pins the RTMPose → MediaPipe mapping against rtmlib's own
installed keypoint table, so a version bump that renumbers fails there rather
than swapping the user's fingers in silence; `test_export_reader.py` pins the
raw frame format, including that a short file is refused rather than reshaped.
`geometry.py`'s numbers are also asserted in the shell's own suite
(`ctest -R hand_inject`), so the C++ and Python halves of the same math keep
agreeing. The verbs themselves are covered end to end over a real socket by
`ctest -R hands_e2e`.

## Handedness

Left lands in the shell's hand slot 0 and right in slot 1 — a fixed
assignment, which is already an improvement on the phone's first-seen-order
slots. Which hand is which is decided three ways, in order.

**The model, when it says.** MediaPipe labels handedness as if it were looking
at a mirror, which is right for a selfie camera and backwards for the phone's
rear camera. The label is flipped by default; `--no-flip-handedness` keeps it
as MediaPipe gave it.

**The depth, when the hand is not flat.** RTMPose says nothing, so the
geometry has to. The palm plane is spanned by (index MCP − wrist) and
(pinky MCP − wrist); in a right-handed frame their cross product points out of
the palm for a right hand and out of the back for a left one, and the thumb is
anatomically always on the palmar side. So the sign of the thumb's offset
along that normal is the answer. This is the same test the gesture engine's
`ge_hand_chirality` runs (`gesture-engine/src/ge_features.cpp`), down to
averaging the thumb MCP, IP and TIP rather than trusting one of them.

**The picture, when it is.** The test above needs the thumb to stand off the
palm in MEASURED depth, and most landmarks miss the LiDAR map and inherit one
range — so a hand that came back flat says nothing, which is the common case
rather than the corner one. The image then decides it: the signed area of
(index MCP − wrist) × (pinky MCP − wrist) in pixels fixes the order of the
knuckles around the wrist, which is rotation-invariant and flips under a
mirror. That one number cannot separate a right hand seen palm-on from a left
hand seen back-on, so it takes one bit of outside knowledge: the phone is on
the user's head, so a hand raised to point at a panel is seen from the BACK.
`--palmar-view` is the other assumption. A hand that is edge-on in both the
depth and the picture is dropped rather than guessed, and the shell's last
good hand ages out on its own 150 ms clock.
