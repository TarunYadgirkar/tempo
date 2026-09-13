# Next session, start here

Written Sat Sep 12 2026, 11:50 PM PT, end of the checkpoint-1 session. Read this, then `README.md` (checkpoint log at the bottom), then `shell/HACK_CHANGES.md`.

## What Tempo is, in one paragraph

Tarun's solo entry to the East v. West 72 Hour Hackathon, Deep Tech / Physical AI track, freestyle. It is Vantage (his AR-glasses startup with Boris, Project Ithaca, currently interviewing with a16z) rebuilt around a room agent: iPhone streams ARKit pose + LiDAR + camera + hands to a Mac; the Mac compositor (`shell/`, carried over from Spatula) renders real Mac windows in 3D; a Python agent (`agent/`) sends Gemini the view plus a scene graph (head pose, depth casts, objects, remembered places, hand aim, panels) and executes spatial actions. Hand tracking was moved onto the Mac (`hands/`, RTMPose + LiDAR). Judges score every 12 h on innovation / technical / business / presentation.

## Deadlines and submission mechanics

- Checkpoint 1 was due Sat 11:59 PM PT (Tarun's correction; the briefing said 7 PM). Confirm the remaining times with Tarun before assuming: briefing says Sun 7 AM, Sun 7 PM, Mon 7 AM PT, final Mon 9 AM PT, but the checkpoint-1 correction suggests they may be midnight/noon.
- Each checkpoint: 60 s video + a doc with the repo link and a what-changed paragraph, dropped in Google Drive → Team Submissions (12 hour cycle) → deep tech → `Tempo`. Only Tarun can upload the video.
- Code lives ONLY at https://github.com/TarunYadgirkar/tempo (main). Do NOT push project code to the official hackathon fork or open upstream PRs until Tarun says so (he reversed that on Sat). The fork `BTC-2026/teams/Tempo` holds a pointer README.
- Final also emailed to hackofthrones@gmail.com. Google Cloud env is hard-deleted Mon Sep 14 noon ET.
- `docs/checkpoint-1-submission.md` is the text Tarun pasted into the checkpoint-1 doc; reuse its structure for checkpoint 2 (Why me, what we're building, question, what shipped, next 12 h, feedback wanted).

## How to bring the rig up

```bash
cd "/Users/tarunyadgirkar/TarunsCode/east west hackathon/tempo"
shell/scripts/build-mac.sh && shell/scripts/test-mac.sh        # 31 ctest
shell/scripts/run-mac.sh --restart                              # installs /Applications/Spatula.app, launches, phone auto-connects
export SPATIAL_OS_SOCK="$TMPDIR/spatial-os.sock"
cd hands && uv run hands track --dir "$TMPDIR/spatula-frames" --enable-export --raw &   # Mac hand tracking daemon
cd ../agent && uv run tempo objects --frames "$TMPDIR/spatula-frames" &                   # object map daemon
uv run tempo scene            # what the agent sees
uv run tempo --speak ask put a note on the wall that says hello
uv run tempo live             # pinch-and-talk (local whisper)
uv run tempo eval geometry pixels     # 20-request placement eval, closes all panels first
cd ../hands && uv run hands eval --dir "$TMPDIR/spatula-frames" --seconds 60 --window-s 5   # phone vs mac hand jitter
uv run hands check --dir "$TMPDIR/spatula-frames"                                          # reprojection check
```

Checks before assuming anything is broken: `pgrep -fl mac-shell`; `printf 'stats\n' | nc -U $TMPDIR/spatial-os.sock` (packet_rate=0 means the phone app is closed; ask Tarun to reopen SpatialBridge). `frame-export status` must say `on` for the hands/objects daemons to get frames; run-mac restart turns it off, re-enable with `frame-export on <dir> --raw`.

Gemini key is in `agent/.env` (`GEMINI_API_KEY`, model `gemini-3.6-flash`); the hooks block Claude from writing env files, Tarun pastes via `open -e`. Whisper model is `mlx-community/whisper-large-v3-turbo`, cached.

## State of each piece

| Piece | State | Evidence |
|---|---|---|
| Agent loop (voice/typed → Gemini → actions) | live, 3 to 4 s/request | README checkpoint log |
| Surface placement via depth cast + full pose | live; wall error 0.05 m vs 0.32 m image-only | `agent/eval/results/`, README table |
| Object map (OWL-ViT, LiDAR-unprojected, persisted) | live at conf 0.25; "next to the whiteboard" worked once | `~/.config/tempo/objects.json` |
| Spatial memory, layouts | implemented, memory demoed once; layouts untested on hardware | `agent/src/tempo/memory.py`, shell `layout` verbs |
| Pinch-and-talk (`tempo live`) | implemented, never tested with a real pinch | `agent/src/tempo/live.py` |
| Mac hand tracking (RTMPose + LiDAR) | live, 10-14 fps, 55 ms e2e; detection 97%; jitter was 36 mm/joint before the palm-range + filter fixes, which are in but UNMEASURED on a real hand | `hands/results/eval-20260913T011508Z.md` |
| Gesture thresholds (pinch/point/fist) | untuned; Tarun says gestures barely work | gesture-engine calibrate tool exists in shell/gesture-engine/tools |

## Next 12 hours, as promised to the judges in the checkpoint-1 doc

1. Hands: with Tarun's hand up, run `hands eval` (alternating phone/mac windows) and `hands check`; publish before/after jitter. Then calibrate pinch/point/fist on his hand with the gesture engine's calibrate tool. Then try WiLoR on a GCE GPU (sponsored credits, project eastwest72hack26bos-505) for the pointing path and report round-trip latency vs local.
2. Hardware: Tarun said the next 12 h is "sourcing glasses and hardware". Help him shortlist see-through display dev kits with camera + depth that can ship by Monday, and a wearable phone mount as fallback. The software already treats the phone as one sensor.
3. Placement: rerun `tempo eval` with the desk in view (ask Tarun to point the phone at his desk), add object-anchored requests to `agent/eval/requests.json`, update the README table.
4. Checkpoint-2 video + doc: same structure as checkpoint 1; must say what changed since.

## Things Tarun has said that constrain the work

- "This has to be insane and amazing" and "use great open source stuff": be ambitious, prefer open models (RTMPose, OWL-ViT, Whisper are in; WiLoR/HaMeR next).
- Hand tracking quality is his number one complaint. Every skeleton offset so far has been a pipeline bug (filter lag), not model quality; check `hands check` before swapping models.
- UI mode: polished. Design rules in `~/.claude/CLAUDE.md` apply to note cards and any UI.
- Do not restart the shell while he might be recording; ask first. Restarting reconnects the phone automatically but kills frame export.
- He wants to be told when a hardware session (his hand, his desk in view) is needed, in one line.
- Never `git add -A` across the repo while background agents are mid-edit; it swept up half-finished files once. Stage by path.

## Known gaps and bugs

- `agent/eval` scorer counts a desk request as wrong when no horizontal surface is in view (both conditions fail); that's the sensor's limit, documented in README.
- Object detector at conf 0.25 still emits some false positives (headphones, clock); raise to 0.3 if it pollutes the scene JSON.
- `hands status` reply format changed to `overlay=on source=phone|mac age_ms=N e2e_ms=N`.
- `shell/HACK_CHANGES.md` still says the LiDAR map is 32x24; it is 256x192.
- Left/right placement was once swapped because the portrait camera's axes leaked into "right"; fixed with forward x world-up. Watch for regressions if head math changes.
- The `chirality` guess for RTMPose assumes a dorsal view (head-mounted camera); if left/right hands flip on the real rig, pass `--palmar-view`.
