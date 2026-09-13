# hands — Mac-side hand tracking for the shell

The iPhone streams the camera, the LiDAR depth map and, until now, its own
Apple Vision hand skeleton. That skeleton is the weak link: it drops the hand
on rotation and its fingertips wander by centimetres, so pinch, point and fist
fire late or not at all.

This package moves the tracking to the Mac. A hand model runs over the camera
frames the shell exports, the LiDAR depth map says how far away the hand is, a
hand-shape model turns that one range into 21, and the resulting metric joints
go back into the shell through
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
model. See **Handedness** and **How a landmark becomes a point in the room**
below for what fills both gaps.

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
 14.7 fps | detect 1.00 | depth 0.98 | held  0 | range 0.52 m | landmark   5.7 ms | e2e    7.3 ms
```

`detect` is the fraction of frames with a hand, `depth` the fraction of
landmarks that got a real LiDAR reading (reporting only under the default
`--depth rigid`, where no joint is placed with one), `held` the frames a
dropped detection was covered by re-sending the last hand, `range` the hand's
smoothed distance, and `e2e` the instant the shell published the frame to the moment it acked the
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
uv run hands eval --dir "$TMPDIR/spatula-frames" --enable-export \
  --seconds 60 --window-s 5
```

Hold a hand up in view, keep it still for stretches of several seconds, and
pinch a few times. The run writes `results/eval-<timestamp>.json` plus a
markdown summary.

It alternates 5 s windows. With injection **off**, `hands dump` returns the
phone's own hands, so that window feeds the phone column. With injection
**on**, the shell is acting on this tracker's joints, so that window feeds the
mac column. Recording both columns in one window is what made the first
version of this report unreadable: `hands dump` returns whatever the gesture
engine is seeing, so with injection on the "phone" column was the Mac.

The three numbers:

- **jitter**, the mean distance a joint moves between frames while you are
  holding still, which is motion you did not make and what makes a pinch
  threshold chatter;
- **detect rate**, the fraction of frames each source had a hand at all;
- **thumb-index distance** at p10/p50/p90 while still. The gesture engine
  fires a pinch under 25 mm, so a source whose resting hand reaches under that
  is pinching by itself.

"Still" is one judgement about **you**, not about the source being scored: the
phone track's wrist speed under 15 mm/s over half a second, bridged across the
injection-on windows it is absent for. Both columns are then scored over the
same wall-clock intervals. Judging each source on its own stillness excused a
noisy source from being measured exactly where it was worst.

`--inject` reverts to injecting for the whole run, which reproduces the old
(meaningless) phone column and nothing else.

## How a landmark becomes a point in the room

1. The backend gives 21 landmarks in the exported image's own pixels, already
   in MediaPipe order. MediaPipe also gives a `world` skeleton in metres
   centred on the hand; RTMPose does not.
2. The depth map answers **one** question: how far away is the hand? The
   patches under the wrist and the four finger MCPs are pooled and medianed
   together, and the result goes through a One Euro filter of its own, tuned
   far harder than the joints (`--range-min-cutoff 0.4`, `--range-beta
   0.005`). A hand's distance changes slowly; the LiDAR's ranging noise does
   not.
3. `handshape.py` turns that one range into 21. Each landmark keeps its own
   pixel ray and gets a range from a hand whose bones have known lengths,
   scaled per frame by the hand's apparent palm width. With RTMPose the palm
   is one frontoparallel plate at the measured range and each finger segment
   is solved outward — the distal landmark's ray meeting the sphere of the
   bone's length around the proximal joint, taking the farther of the two
   roots because a head-mounted camera sees the back of a raised hand and
   curling carries a fingertip away. With MediaPipe the world skeleton already
   knows the shape, so it is only scaled and slid until its palm centroid sits
   at the measured range.
4. The scene-frame head pose the shell stamped on the frame carries the point
   into the frame the panels live in.
5. One Euro per axis per landmark removes the residual jitter, and a two-frame
   hold covers a dropped detection so the gesture engine never sees the hand
   blink out for one frame.

`--depth per-joint` is the older path, where every landmark medians a 5x5
patch under itself and a landmark that missed takes the wrist's range. It is
kept because it is what the default has to beat, and it loses badly. The
exported map is 256x192 over a 1920x1440 capture, so one depth pixel covers
7.5 capture pixels and a 5x5 patch spans about 14 mm of the scene at half a
metre — wider than a finger. Every time the patch crosses the silhouette the
median falls through to the wall behind the hand and the joint teleports by
the depth of the room. That is not noise around the truth to be smoothed away,
it is a different surface, which is why the fix is upstream of the filter.

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
| `handshape.py` | one measured range → 21, and the bone lengths it uses |
| `tracker.py` | the range measurement, the unprojection, the filters, the hold |
| `onefilter.py` | One Euro |
| `control.py` | the shell's line protocol, including the message split |
| `track.py` | the live loop |
| `evaluate.py` | recording both sources and the jitter metric |

## Tests

```sh
uv run pytest
```

`test_handshape.py` builds a flat hand at half a metre, projects it, and
insists the shape model puts every joint back where it was — same pixels, same
bone lengths — from the one range; `test_tracker.py` drives the metric stage
with a fixture backend, covering the hold and the silhouette case the rigid
path exists for; `test_evaluate.py` pins the stillness windowing, including
that a held frame is not credited with being steady. `test_keypoints.py` pins
the RTMPose → MediaPipe mapping against rtmlib's own
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

**The picture, otherwise.** The signed area of
(index MCP − wrist) × (pinky MCP − wrist) in pixels fixes the order of the
knuckles around the wrist, which is rotation-invariant and flips under a
mirror. That one number cannot separate a right hand seen palm-on from a left
hand seen back-on, so it takes one bit of outside knowledge: the phone is on
the user's head, so a hand raised to point at a panel is seen from the BACK.
`--palmar-view` is the other assumption. A hand that is edge-on in the picture
is dropped rather than guessed, and the shell's last good hand ages out on its
own 150 ms clock.

There used to be a third test here, tried before the picture: the sign of the
thumb's offset from the palm plane in measured depth, the same quantity the
gesture engine's `ge_hand_chirality` computes
(`gesture-engine/src/ge_features.cpp`). It is gone. Under the shape model the
thumb's side of the palm plane is set by the *same* dorsal assumption the
pixel test uses, so asking the joints would be asking the assumption to
confirm itself. `chirality_from_joints` stays in `geometry.py` because the
shell computes the same quantity and the two are pinned against each other.
