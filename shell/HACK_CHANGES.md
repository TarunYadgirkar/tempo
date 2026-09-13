# Hackathon changes inside shell/

Three control-socket verbs on top of the wxrd wire layer. Replies keep the
line protocol's `ok ...` / `err <code> <detail>` shape.

- **`note <json>`** — `{"title":str,"body":str,"accent":optional bool}` creates
  a `panel_kind::note` panel: 512x320, title over word-wrapped body (six lines,
  then an ellipsis), Vantage surface/ink tokens, one thin accent rule in the
  brand orange when `accent` is true. Replies `ok handle=<n>`; the panel shows
  up in `list-windows` with `app_id` `note` and the note title. Errors:
  `err bad_json <why>`, `err resource_exhausted`.
  Files: `mac-shell/src/ui/note_card.{h,cpp}` (new),
  `mac-shell/src/core/json_lite.{h,cpp}` (new),
  `mac-shell/src/core/scene.{h,cpp}`,
  `mac-shell/src/platform/control_server.cpp`, `mac-shell/CMakeLists.txt`.
- **`note-update <handle> <json>`** — changes title, body and/or accent in
  place; omitted keys keep their value. Replies `ok`; `err no_such_window` for
  an unknown handle or a panel that is not a note, `err bad_json <why>`,
  `err parse_error <line>`.
  Files: `mac-shell/src/core/scene.{h,cpp}`,
  `mac-shell/src/platform/control_server.cpp`.
- **`aim`** — replies
  `ok {"hands":<0|1|2>,"aim":[x,y,z]|null,"ray_origin":[x,y,z]|null,"ray_dir":[x,y,z]|null,"pinching":bool,"aimed_handle":<n>|null,"hit":[x,y,z]|null}`
  in the scene frame. Reuses `scene::aim_point` + `pick_aim_panel`; `hit` is the
  ray/quad intersection on the aimed panel, else the ray's intersection with
  the nearest detected plane. Answers with nulls (and `hands:0`) when no hand
  is visible.
  Files: `mac-shell/src/core/scene.{h,cpp}`,
  `mac-shell/src/platform/control_server.cpp`.
- **`layout save|load|list <name>`** — `~/.config/spatial-os/layouts/<name>.json`
  (override the root with `SPATIAL_OS_CONFIG_DIR`) holds every panel's kind,
  app_id/title/note text, pos, yaw, size, anchor uuid and anchor offset. Save
  replies `ok saved=<n>`; load closes nothing and re-creates note and internal
  panels at their saved poses, re-running `launch-app` for captured windows and
  counting the ones that do not come back, replying
  `ok restored=<n> skipped=<m>`; list replies `ok {"layouts":[...]}`. A name
  outside `[A-Za-z0-9_-]{1,40}` gets `err bad_name`, a missing layout
  `err not_found <name>`.
  Files: `mac-shell/src/core/layout_store.{h,cpp}` (new),
  `mac-shell/src/core/scene.{h,cpp}`,
  `mac-shell/src/platform/control_server.{h,cpp}`.
- **`event pinch phase=begin|end`** on the `subscribe` stream. The existing
  `event gesture name=... phase=...` line carries the winning pinch_select
  VARIANT (`pinch_select.loose`, `pinch_select.double_tap`, …), so a subscriber
  matching on the plain name misses most pinches; the new line is the one to
  watch. The same `pinch_held_` flag feeds `aim`'s `pinching` field.
  Files: `mac-shell/src/core/scene.{h,cpp}`.

Supporting changes: `uuid_to_hex` moved next to `parse_uuid_hex` in
`mac-shell/src/core/anchor_math.{h,cpp}` and the duplicate `json_escape` in
`scene.cpp` now comes from `json_lite.h`.

Tests: `mac-shell/tests/test_note_layout.cpp` (ctest `note_layout`) covers the
word wrap, the accent rule, the JSON reader, layout name validation and the
save/load roundtrip, the scene-side note and layout operations, and the pinch
events; `mac-shell/tests/test_hack_verbs_e2e.cpp` (ctest `hack_verbs_e2e`)
drives every new verb and its error paths over a real control socket.

- **`cast [<ox> <oy> <oz> <dx> <dy> <dz>]`** — where a ray meets the room, in
  the scene frame. Bare `cast` uses the head's forward ray. Replies
  `ok {"hit":[x,y,z],"distance_m":d,"normal":[nx,ny,nz],"kind":"horizontal|vertical|slanted","source":"depth|plane|none"}`
  (every value null and `source":"none"` when nothing is hit). `source=depth`
  marches the LiDAR depth map — the first sample the ray passes within 3 cm
  of, else the point where it crosses behind the depth surface — and takes the
  normal from a cross product of the neighbouring samples' finite differences,
  oriented back toward the ray origin. `source=plane` is the old
  `ray_plane_hit` against the ARKit plane list, used when no depth map is
  available or the ray finds no valid samples along it; `kind` buckets on
  `|ny|` (> 0.8 horizontal, < 0.3 vertical). This replaces "intersect the head
  ray with the plane list and fall back to a fixed offset" for "put it on the
  desk": the plane list is sparse and stale (a desk can be missing, a bed can
  come back vertical), the depth map is neither. `err parse_error <line>` for
  anything but six floats.
  Files: `mac-shell/src/core/depth_cast.{h,cpp}` (new),
  `mac-shell/src/core/scene.{h,cpp}`,
  `mac-shell/src/platform/control_server.cpp`,
  `mac-shell/src/platform/renderer{.mm,_internal.h}`, `mac-shell/CMakeLists.txt`.
- **`pose <handle> <x> <y> <z> <qx> <qy> <qz> <qw>`** — a panel's full
  scene-frame transform. Position plus an xyzw orientation quaternion (an
  un-normalised one is normalised); the panel's own +Z is the face that looks
  at the viewer, so identity faces +Z. Replies `ok`. Until now only a yaw was
  settable on an unanchored panel, and `refresh_anchor_transforms` rebuilt the
  matrix from that yaw every tick; a panel now carries `has_pose_quat` and
  keeps its full matrix instead, which `list-windows` reports as the real
  `quat`. Setting a pose drops any anchor (and any pending re-snap) — a
  gather or a `layout load` re-derives a facing yaw and retires it. Errors:
  `err no_such_window`, `err bad_quat <why>`, `err parse_error <line>`.
  Files: `mac-shell/src/core/scene.{h,cpp}`,
  `mac-shell/src/platform/control_server.cpp`.
- **`floor`** — the lowest horizontal surface currently observed. Replies
  `ok {"height":y|null,"source":"depth|plane|none"}`, `height` being a
  scene-frame Y. The depth answer bins every upward-facing sample (normal
  `ny > 0.8`) by height in 2 cm bins and takes the lowest bin holding a real
  patch of surface, so a desk slab above the floor cannot raise the answer and
  one stray reading below it cannot lower it; the fallback is the lowest
  horizontal plane's scene-frame centre.
  Files: `mac-shell/src/core/depth_cast.{h,cpp}`,
  `mac-shell/src/core/scene.{h,cpp}`,
  `mac-shell/src/platform/control_server.cpp`.
- **`head-pose` gains `"roll_deg"`** — the camera's roll about its own forward
  axis measured against gravity: 0 held level (landscape-native), ±90
  portrait, 180 upside-down, and `null` when the optical axis is near-parallel
  to gravity, where roll is undefined rather than zero. Same `atan2(ux, uy)`
  projection the renderer's orientation buckets snap
  (`depth_math.h orientation_bucket_from_gravity`), reported unsnapped.
  Files: `mac-shell/src/core/scene.cpp`.

Supporting change: the SCENE is now the single consumer of
`sb_get_latest_depth` (the call transfers the buffer's ownership, so there can
only be one) — it keeps the latest map with its intrinsics and the SCENE-frame
camera pose at the frame's timestamp, and the renderer pulls its occlusion
texture from `scene::snapshot_depth` instead of the receiver.

Tests: `mac-shell/tests/test_depth_cast.cpp` (ctest `depth_cast`) casts
against a synthetic floor at y = -0.8 and wall at z = -2, pinning the hit
point, normal, distance and kind for rays at the wall, at the floor, from an
offset origin, plus the no-hit paths and the floor-height search; the `pose`,
`floor` and `roll_deg` scene operations are in
`mac-shell/tests/test_scene.cpp`; `mac-shell/tests/test_cast_e2e.cpp` (ctest
`cast_e2e`) drives all four over a real control socket against two synthesised
replay sessions — planes only (`source=plane`) and planes plus 0x0A/0x0B LiDAR
(`source=depth`).

## Hand tracking moved to the Mac

The phone's Apple Vision hand skeleton is the weakest link in the gesture
stack: it drops the hand on rotation and its fingertips wander by centimetres,
so pinch, point and fist fire late or not at all. These two verbs let a process
on the Mac do the tracking instead (MediaPipe Hand Landmarker over the streamed
camera frames, fingertip range from the LiDAR depth map — `hands/`) and push
the result back into the SAME joints the `0x05` packet fills, so every gesture
already built on them benefits without knowing the source changed.

- **`frame-export on <dir> | off | status`** — publishes the live camera frame
  and everything needed to interpret it into `<dir>`, at up to 15 Hz:
  `latest.jpg` (the decoded RGBA re-encoded), `latest.depth` (raw float32
  metres, row-major, `0` = no reading) and `latest.json`. Each is written to a
  temp file in the same directory and `rename()`d into place; the sidecar is
  renamed LAST and carries the frame timestamp, so a reader that polls it never
  sees a half-written image. Sidecar shape:
  `{"t_ns":..,"seq":..,"image":{"width":..,"height":..,"file":"latest.jpg"},`
  `"head":{"frame":"scene","pos":[x,y,z],"quat":[x,y,z,w]}|null,`
  `"intrinsics":{"fx","fy","cx","cy","image_width","image_height"}|null,`
  `"depth":{"width":..,"height":..,"file":"latest.depth","format":"float32","t_ns":..}|null}`.
  `head` is `scene::head_pose_at(frame.timestamp)`, not the newest pose — the
  landmarks have to land in the frame the camera saw, not the one the head has
  since turned to. The intrinsics are in ARKit CAPTURE pixels, which are larger
  than the JPEG's; a consumer rescales by `image_width/width`. All three of
  `head`, `intrinsics` and `depth` can be null, and a run can legitimately have
  colour and nothing else. Replies `ok on dir=<path> frames=<n> hz=15` /
  `ok off frames=<n>`. Like `screenshot`, the target must resolve inside
  `$TMPDIR` or `$HOME` (`err outside_allowed_roots`, `err not_absolute`,
  `err mkdir_failed`) — it is a file-write primitive on an agent-reachable
  socket. The renderer feeds the exporter the frame it already decoded;
  headless, `main.cpp`'s tick loop pulls one itself, but only while export is
  on (`sb_get_latest_frame` transfers ownership, so draining frames nobody
  looks at would starve the renderer).
  Files: `mac-shell/src/platform/frame_export.{h,mm}` (new),
  `mac-shell/src/platform/main.cpp`, `mac-shell/src/platform/renderer.mm`,
  `mac-shell/src/platform/control_server.{h,cpp}` (one dispatch block + a
  handler seam, so the verb links without the renderer sources).
- **`hands-inject <json>`** —
  `{"t":ms,"hands":[{"chirality":"left|right","confidence":0..1,"joints":[[x,y,z] x21]}]}`,
  joints in SCENE-frame metres in MediaPipe landmark order. Replaces the
  phone's hands for the gesture engine and the skeleton overlay while the
  injection is fresher than 150 ms; past that the phone's hands come straight
  back, so a tracker that dies degrades instead of freezing the user's hands in
  mid-air. `err bad_json <why>` otherwise.
  Storage is per chirality rather than per message, because the vendored wire
  layer caps a request line at `CTL_CONN_INPUT_MAX` (~1.1 kB) and two hands of
  21 joints do not fit in one: each message REPLACES the chiralities it
  carries and leaves the other to age out on its own clock, and an empty
  `hands` array is the explicit "no hands in view" that retracts both. Left
  takes scene slot 0 and right slot 1 — fixed, unlike the phone's
  first-seen-order slots.
- **`hands status`** now replies `ok overlay=on|off source=phone|mac age_ms=<n>`
  (`age_ms` is the age of the last injection). **`hands dump`** replies
  `ok {"source":"phone|mac","age_ms":..,"frame":"scene","hands":[{"slot":n,`
  `"age_s":..,"joints":[[x,y,z,confidence] x21]}]}` — the joints the gesture
  engine is seeing right now, in MediaPipe order so an evaluation harness can
  line the two sources up landmark by landmark without a second table.

**Joint order.** MediaPipe's 21 landmarks and `SB_JOINT_*` (what the `0x05`
packet packs, and what `GE_JOINT_*` mirrors) enumerate the same skeleton in the
same sequence — wrist, thumb CMC/MCP/IP/TIP, then index/middle/ring/pinky
MCP/PIP/DIP/TIP — so the mapping is the identity. It is still written out as a
table in `core/hand_inject.cpp` and asserted as a bijection by a test, because
an agreement that holds by coincidence stops holding silently.

Supporting changes: `scene::ingest_hand` takes an `already_scene` flag (mac
joints arrive in the scene frame and must skip the ARKit-world rewrite), and
`scene::tick_locked` consults the injection store before draining
`sb_get_hand` — never both in one tick, since alternating sources would make
every gesture threshold chatter.
Files: `mac-shell/src/core/hand_inject.{h,cpp}` (new),
`mac-shell/src/core/scene.{h,cpp}`, `mac-shell/CMakeLists.txt`.

Tests: `mac-shell/tests/test_hand_inject.cpp` (ctest `hand_inject`) pins the
landmark bijection, the depth unprojection (including that a downscaled image
unprojects to the same point as the full-resolution one, which is the mistake
that would put every landmark at several times its true angle off-axis), the
camera→scene transform, the payload parse and its refusals, the freshness
window, and the scene-level takeover and fallback;
`mac-shell/tests/test_hands_e2e.cpp` (ctest `hands_e2e`) drives both verbs over
a real control socket against a synthesised session carrying pose, intrinsics,
a chunked JPEG frame and a chunked LiDAR depth map — checking the sidecar's
contents, that `latest.depth` reads back as exactly the metres the wire
carried, the containment refusals, that the export keeps publishing rather than
firing once, and that the mac takeover expires back to the phone.

The tracker itself lives in `hands/` (its own uv project, Python 3.12,
mediapipe 0.10.21) with `uv run hands track` and `uv run hands eval`; see
`hands/README.md`. `hands/tests/` re-asserts the same unprojection numbers in
Python, so the two halves of that math keep agreeing.

---

## RTMPose hand tracking, and cutting the transport latency

The Mac path worked but felt bad: 8-10 fps end to end with about 250 ms of
lag, and MediaPipe's 2020 hand model wandering under it. Two separate causes,
fixed separately.

### The model: `--backend rtmpose|mediapipe`

`hands track` and `hands eval` now take `--backend`, defaulting to `rtmpose`.
It runs OpenMMLab's RTMDet-nano hand detector and RTMPose-m hand through
`rtmlib` on ONNX Runtime — no mmcv, no PyTorch, and the two ONNX models
download themselves into `~/.cache/rtmlib` on first use. `mediapipe` is the
original path, kept so the two can be measured over the same frames.

Per frame of a 640x480 export on this Mac: rtmpose 5.7 ms against mediapipe's
11.3 ms, whole loop 7.3 ms against 12.6 ms. Two things buy that. The pose
model runs on the CoreML execution provider (3.7 ms against 12.8 ms on the
CPU); the DETECTOR cannot, because its ONNX export carries grid-decode ops
whose static shapes CoreML infers at a different rank and ONNX Runtime raises
mid-inference rather than falling back, so it stays on the CPU at 6.1 ms and a
CoreML session that fails to construct or to run once drops to the CPU with a
printed line. And the detector is not run every frame: a hand found last frame
is looked for inside its own grown box, with a detection forced every
`--det-interval` frames (10), when the pose score drops, or when nothing was
tracked.

**Landmark order.** rtmlib's hand21 is wrist, thumb1..4, forefinger1..4, then
middle, ring and pinky 1..4 — the COCO-WholeBody hand block, which enumerates
the same skeleton in the same sequence as MediaPipe and as `SB_JOINT_*`. So
that mapping is the identity too. `hands/src/hands/keypoints.py` writes it out
as a table beside both sets of names, and `hands/tests/test_keypoints.py`
checks it against rtmlib's own installed `hand21` keypoint table — including
that `thumb1` links to the wrist, which is what makes it the CMC rather than
the MCP and would otherwise shift the whole thumb by one joint.

**Handedness.** RTMPose gives no chirality, so the geometry supplies it.
First the same test the gesture engine runs: the palm plane from
(index MCP − wrist) × (pinky MCP − wrist), and the sign of the thumb's offset
along that normal, since the thumb is anatomically always palmar
(`palm_thumb_signed_offset`, `gesture-engine/src/ge_features.cpp`). That needs
the thumb to stand off the palm in MEASURED depth, and most landmarks miss the
32x24 LiDAR map and inherit the reference landmark's range — so a hand that
came back flat is undecidable, which is the common case rather than the corner
one. The picture then decides it: the signed area of the same two palm vectors
in pixels fixes the order of the knuckles around the wrist, which survives
rotation and flips under a mirror. That cannot separate a right hand seen
palm-on from a left hand seen back-on, so it takes one assumption — the phone
is head-mounted, so a raised hand is seen from the BACK; `--palmar-view` is the
other. Edge-on in both is dropped rather than guessed.

The LiDAR unprojection, the scene transform and the One Euro stage are shared:
the backends live behind `hands/src/hands/backends/`, whose whole contract is
one frame in and 21 pixel landmarks out, so swapping the model cannot change
the metric pipeline under it. What does differ is the depth fallback —
MediaPipe's metric `world` skeleton shapes the estimated range of a landmark
that missed the LiDAR, and RTMPose has no hand model to shape it with, so the
estimate flattens to the reference landmark's own range.

### The transport: 30 Hz, raw frames, and a latency that is finally reported

- **`FRAME_EXPORT_MAX_HZ` 15 -> 30.** The phone streams at 30 and the gate was
  holding half of them, which is up to 66 ms of pure waiting on every gesture.
- **`frame-export on <dir> --raw`** (or `raw=1`) publishes `latest.rgb`:
  uint8 RGB, row-major, no header, box-averaged down to at most 640 wide,
  instead of `latest.jpg`. The shell was encoding a JPEG out of pixels it
  already had so that the tracker could decode them again; the sidecar's
  `image` object grows a `"format":"jpeg"|"rgb8"` and the status line grows
  `format=`. The intrinsics are untouched — they are in ARKit capture pixels
  and every consumer already rescales by `image_width/width`, so the downscale
  costs only pixels nobody was going to use. The flag is a trailing token, so
  a directory whose last component is literally `--raw` cannot be named, which
  beats a quoting grammar on a line-oriented socket.
- **Polling is on `os.stat`,** not on the sidecar's contents, with a 2 ms
  sleep rather than 5 ms. Re-reading and re-parsing the JSON every few
  milliseconds at 30 Hz was most of a core spent learning nothing.
- **`e2e` is a number now, not `n/a`.** The sidecar carries `export_ns`, this
  Mac's `CLOCK_REALTIME` at the instant the frame was published;
  `hands-inject` takes an optional `frame_t_ns` echoing it back; and
  `hands status` reports `e2e_ms=<n>` alongside `age_ms`, or `-1` when the
  held injection carries no stamp. Both ends are one clock on one machine.
  The frame's own `t_ns` is the PHONE's capture clock, and the old line
  subtracted the two, so it was measuring a clock offset and discarding the
  result as implausible.

Measured on a synthetic 30 Hz export (a drawn hand, a LiDAR-shaped depth map
with readings on the palm only) against a headless shell on
`$TMPDIR/tempo-rtm.sock`: **30.0 fps, detect 1.00, depth 0.78, landmark 5.7 ms,
e2e 7.3 ms**, against mediapipe's 11.3 ms / 12.6 ms over the same frames.
Reading the raw file rather than decoding the JPEG is 0.09 ms against 0.52 ms
per frame on the Python side; the encode it saves on the shell's side is the
larger half and is not measured here, because it needs a live phone.

Files: `hands/src/hands/backends/{__init__,mediapipe_backend,rtmpose_backend}.py`
(new), `hands/src/hands/keypoints.py` (new), `hands/src/hands/{tracker,
export_reader,geometry,control,cli,track,evaluate}.py`,
`mac-shell/src/platform/frame_export.{h,mm}`,
`mac-shell/src/core/hand_inject.{h,cpp}`, `mac-shell/src/core/scene.{h,cpp}`.

Tests: `hands/tests/test_keypoints.py` pins the RTMPose landmark mapping
against rtmlib's own installed table and both chirality tests, including that
an edge-on hand is refused rather than guessed;
`hands/tests/test_export_reader.py` pins the raw frame format, that a short
file is refused rather than reshaped, that a pre-raw sidecar still reads as a
JPEG with no export clock, and that the mtime poll yields each frame once.
`mac-shell/tests/test_hands_e2e.cpp` grows a raw-mode section (`format=rgb8`,
`latest.rgb` exactly width x height x 3, no `latest.jpg`) and pins `e2e_ms`
over the socket; `test_hand_inject.cpp` pins the `frame_t_ns` parse, its
refusals, that "no hands in view" retracts the stamp with the hands, and that
the nanosecond stamp survives its round trip through a JSON double to within
~256 ns. 45 pytest and 31 ctest, all green.

---

## The overlay was drawn behind the hand: `hands check`

With `hands-inject` live the skeleton sat 30-60 px off the real hand in the
passthrough, shape intact. Every geometric suspect was wrong. `uv run hands
check --dir <export dir>` takes one exported frame, unprojects pixels to the
scene frame and puts them back through the SAME intrinsics and head pose: the
round trip closes to **0.0006 px** over a grid of 49 LiDAR-backed pixels, so
the capture-pixel rescale, the principal point, the ARKit y/z flips and the
quaternion all agree with each other and with `hand_inject.cpp`. The renderer
agrees too, by construction: `passthrough_uv_mapping` composed with
`perspective_rh` and the orientation-bucket roll reduces algebraically to the
pinhole `unproject_pixel` inverts, offsets and portrait buckets included. And
under the rigid shape model a wrong LiDAR range slides a joint along its OWN
pixel ray, which cannot move it in the picture at all.

What was left was time. The One Euro stage smooths the joints in the scene
frame, and its speed coupling was `beta=0.02` against joint speeds measured in
**metres per second** — 0.3 m/s lifts a 1.0 Hz cutoff to 1.006 Hz, so the
filter was a fixed 1 Hz low-pass and never the adaptive one it is named after.
A first-order low-pass tracks a ramp one time constant behind it: 158 ms, or
4.7 cm, or **45 px at half a metre**, for a hand moving gently, and 149 px for
one moving at 1 m/s. The hand's RANGE had the same defect twice as badly
(`0.4 Hz`, `beta=0.005`): an arm reaching out at 0.5 m/s had its hand placed
20 cm short until it stopped.

Defaults now `--min-cutoff 2.0 --beta 10.0` and `--range-min-cutoff 1.5
--range-beta 2.0`, which puts the same hand **9 px** behind itself at 0.3 m/s
and 13 px at 1 m/s, and the range within 3 cm of a reaching arm. `hands check`
prints the offset in pixels at the frame's own measured range and fails past
15 px, which is the width of a fingertip reticle at that range.

`hands check` also compares the mac joints against the phone's own
(`hands dump`, source=phone) landmark by landmark when both see a hand. That
comparison has NOT been run: the only saved frame set
(`hands/eval/fixtures/hand-frame/`, seq 14190, kept as the fixture) is the
phone face-down on a desk with no hand in it, and the phone had no hand in
view at the time of writing. The fixture still exercises the round trip, the
depth read and the lag arithmetic.

Files: `hands/src/hands/check.py` (new), `hands/src/hands/{cli,tracker,
geometry}.py`, `hands/tests/test_check.py` (new),
`hands/eval/fixtures/hand-frame/` (new). 86 pytest, no shell sources touched.

### Sun Sep 13, morning: launched windows that "never showed"
- `scene::place_new_panel` used three fixed sideways slots keyed on the handle, so the fourth launch sat exactly on the first, and a new spawn never took focus. Spawn now takes the nearest free slot (`SPAWN_MIN_GAP_M`), pulls in ahead of the depth-cast surface along the head forward, and the new panel is focused.
- `scene::recall_panel` + capture.mm: launching an app whose window is already streamed recalls that panel in front of the head instead of opening a duplicate stream. Emits `event recall handle=N`.
- hands daemon: `--max-range-m` (1.0) and `--min-confidence` (0.5) drop hand-shaped detections across the room before they reach `hands-inject`; a 1.5 m phantom had been outranking the phone's own hands.
