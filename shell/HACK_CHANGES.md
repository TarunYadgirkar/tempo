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
