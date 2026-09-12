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
