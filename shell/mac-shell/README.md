# mac-shell

Native macOS spatial compositor for the Spatula stack — the wxrd replacement.
Reuses bridge-receiver, gesture-engine, and the wxrd control-plane wire layer
unchanged; ports the anchors.c pose math; adds a Metal renderer and
ScreenCaptureKit app panels.

This file is the **developer quickstart**. The other mac docs each own one job:

- [`docs/mac-shell-design.md`](../docs/mac-shell-design.md) — architecture
  rationale + the UI/visual design of every surface (panels, keyboard,
  launcher, welcome card, dock, toasts, depth occlusion).
- [`docs/macos-port.md`](../docs/macos-port.md) — port status and
  feature-parity reference vs the Linux build.
- [`docs/live-mac-rig.md`](../docs/live-mac-rig.md) — user how-to: run live
  against the iPhone, permission prompts, troubleshooting.

Ships as one installed app: **Spatula** (`/Applications/Spatula.app`,
bundle id `com.spatialos.spatula`, icon from `resources/Spatula.icns` —
regenerate with `swift scripts/gen-app-icon.swift`). The executable inside
stays `mac-shell`; the bundle id and app name are FINAL — changing either
resets the user's TCC grants (see the warning block in `Info.plist`).

## Install / update + run (the normal flow)

```sh
./scripts/install-mac.sh   # build signed bundle → rsync to /Applications/Spatula.app
./scripts/run-mac.sh       # auto-installs if missing/stale, then launches it
```

Install once, grant Screen Recording once to "Spatula", done — every later
install rsyncs over the same path with the same signature, so the grant
persists. `run-mac.sh --dev` runs from `build/mac-shell/` instead (TCC
grants may not stick there). Full rig walkthrough: `docs/live-mac-rig.md`.

## Build + test

```sh
cmake -G Ninja -S mac-shell -B build/mac-shell && ninja -C build/mac-shell && ctest --test-dir build/mac-shell --output-on-failure
```

(or `./scripts/build-mac.sh` + `./scripts/test-mac.sh`, which cover the
sibling suites too.)

Source layout (`src/`): `core/` — pure C++ world state and math (scene,
anchor/placement/depth/vec math, HUD state; headless-testable, no
AppKit/Metal); `ui/` — CPU-drawn surfaces and overlays (panel surface, font
atlas, keyboard, launcher, theme tokens); `platform/` — the macOS seam
(main, control server, Metal renderer, ScreenCaptureKit capture).

## Run

Headless, replaying a recorded session:

```sh
SPATULA_MAC_HEADLESS=1 build/mac-shell/mac-shell --replay tests/sessions/<session>.bin
```

Run live (UDP 9898 from the iPhone app): `build/mac-shell/mac-shell` —
non-headless opens the Metal window (fly-cam via WASD/QE + mouse drag when no
phone is connected). `launch-app <bundle-id-or-title>` captures a real mac
window as a panel (needs Screen Recording permission; degrades to a test card
without it, and says so with an in-scene toast). CGEvent input injection into
captured windows is on by default once Accessibility is granted
(`SPATULA_MAC_INJECT=0` disables; headless runs never inject). Focusing a
captured panel also mirrors that focus onto the real window — main + focused +
raised over the Accessibility API, which does not disturb the user's frontmost
app; only if AX refuses does it fall back to activating the owning app.
Without it `CGEventPostToPid` keystrokes go to whatever that app last treated
as its key window. Clicks and scrolls re-read the source window's frame from
`CGWindowListCopyWindowInfo` at injection time rather than trusting the 1 s
geometry poll, so a window moved since the last tick still takes the click.
A pinch landing inside a panel's quad posts a real click at that pixel
(right-click pinch → button 1), and the panel a pinch would hit is rimmed in
the accent while you aim.

Verification helpers: `SPATULA_MAC_EXIT_AFTER=<s>`,
`SPATULA_MAC_SCREENSHOT=<png>` (`SPATULA_MAC_SCREENSHOT_AT=<s>` retimes it —
a startup one-shot of the same grab the `screenshot` control verb takes on
demand),
`SPATULA_MAC_FIRST_RUN=0|1|cheat` (suppress / force the first-run aids —
verification runs suppress them by default and never touch NSUserDefaults),
`SPATULA_MAC_HAND_DEBUG=0` (start without the 21-joint skeleton, on by default; toggle it
live with `hands overlay on|off`),
`SPATULA_MAC_DEPTH_OCCLUSION=0|1|2` (LiDAR occlusion off/hard/soft).
UI states render to PNGs via `scripts/mac-ui-screenshots.sh`.

## Control socket

`$SPATIAL_OS_SOCK`, else `$TMPDIR/spatial-os.sock` — drive it with
`vendor/wxrd/tools/spatialctl.c` semantics (`version`, `list-windows`,
`move`, `anchor`, `type`, `dump-state`, `subscribe`, …). mac-shell
extensions: `launch-app <target>`, `permissions`, `stats`, `gather-panels`,
`keyboard show|hide|status`, `launcher show|hide|select <n>|commit|status`,
`note <json>` / `note-update <handle> <json>`, `aim`,
`layout save|load|list <name>`, `depth-occlusion on|off|soft|status`, `hands overlay on|off|status` (the
21-joint skeleton with thin translucent bones and smaller joints. Cyan/yellow
identify stream slots, not guaranteed handedness. Fingertips use rings with
a small center in both modes; invalid points are hidden, low confidence fades,
and contact with the keyboard adds a subtle warm highlight).

`stats` reports `packet_rate`, `panels`, `frame_age_ms`, `have_intrinsics`,
and `view_lag_ms`. `frame_age_ms` is milliseconds since the last camera frame
was *ingested* (`-1` = none ever), independent of whether the renderer
consumed it — poses and frames arrive separately, so a live `packet_rate` next
to a climbing `frame_age_ms` is exactly the "camera died, ARKit didn't" case.
Past ~1 s the renderer fades the passthrough quad out over 0.3 s (a new frame
snaps it straight back) and posts a one-shot toast. `have_intrinsics` is 0
until the first `0x0A` packet lands; while it is 0 and frames are arriving the
passthrough is stretched across the hardcoded 60° frustum, and the dock shows
a `CALIBRATING` chip to say so.

`view_lag_ms` is the pose/frame time offset the renderer actually drew with:
the newest head pose's timestamp minus the timestamp of the passthrough frame
on screen. Poses arrive in one datagram while frames are chunk-reassembled and
JPEG-decoded, so the image is always older than the newest pose; building the
view from the newest pose is what used to make world-locked panels swim across
the passthrough on head turns. The renderer instead asks the scene for
`head_pose_at(frame.timestamp_ns)` — a fixed 30-slot ring of recent
SCENE-frame poses, position lerp'd and rotation slerp'd between the two
bracketing samples — and the head-locked passthrough quad is drawn against
that same view. Hands,
gestures, and the orientation-bucket roll compensation keep using the newest
pose. The ring is dropped whenever the SCENE frame it was expressed in dies (a
tracking reset or a post-gap origin recapture). Read it as: `0.0` means the
view is drawn from the newest pose — no frame on screen yet, a frame stamped
exactly on a pose, or one older than the ring, which all fall back to it;
`-1` means no renderer has published one at all (any headless run). Typical
live values track the camera pipeline's decode lag, in the tens of ms.

Stage-1 notes: `launch` spawns an internal test-card panel; `key` accepts any
name and logs it verbatim (no xkb on macOS), but only the named keys
(`Return`, `Tab`, `space`, `Escape`, `BackSpace`/`Delete`, the four arrows) and
a plain decimal macOS virtual keycode in 0–127 are injected into a captured
window — anything else is logged and dropped rather than posted as a raw
keycode; `dump-state` windows carry an extra `input_log` tail so agents/tests
can observe routed input.

### `screenshot`

Two tiers, both replying `ok path=<abs path>` once the PNG is on disk (the
path, not the bytes — the control stream stays cheap and the agent reads the
file):

```
screenshot [<path>]      # the whole composited scene, from the NEXT frame
screenshot <handle>      # one panel's own pixels (wxrd's form)
```

Bare `screenshot` writes `$TMPDIR/spatula-shot-<unix ms>.png`. The reply waits
for the frame the grab lands on, bounded at 1 s — past that it is
`err timeout no frame presented within 1s`. Headless there is no drawable, so
the whole-scene form is `err unimplemented renderer not running`; the
per-window form is pull-based and works in both modes (a captured window's
latest ScreenCaptureKit frame, or an internal panel's CPU surface — never the
composited scene, so passthrough camera frames can't leak into a per-window
shot). wxrd's `region=`/`scale=` options are `err unsupported`.

The verb is a file-write primitive on an agent-reachable socket, so the target
must resolve inside `$TMPDIR` or `$HOME` (`err bad_path <reason>` otherwise —
containment is checked against the realpath of the parent, so a symlinked
directory can't escape) and the file is opened `O_NOFOLLOW`, so a symlink
planted at the path is refused rather than followed.

**Coordinate frames in replies.** Every reply that carries a position tags the
frame it is in, because the two are not interchangeable and used to be mixed
silently. `list-windows` / `dump-state` windows are `"frame":"scene"` — the
origin-subtracted SCENE frame (`docs/coordinate-systems.md` section 9), which
is also what `move` takes and what `list-planes` centres are in. `head-pose` is
`"frame":"raw"`: its `pos`/`rot` are the untouched ARKit values (wire-compatible
with wxrd's reply), and it additionally carries `scene_pos`/`scene_rot` — the
same pose in the SCENE frame, which is the pair to compare against window
positions.
