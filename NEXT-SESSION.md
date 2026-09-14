# Next session, start here

Written Sun Sep 13 2026, 7:15 PM PT; rig state and the credits section added 6:40 PM PT by a parallel session. Read this, then `README.md` (checkpoint log near the bottom), then `shell/HACK_CHANGES.md`. Everything below is committed and pushed to https://github.com/TarunYadgirkar/tempo (main).

## What Tempo is, in one paragraph

Tarun's solo entry to the East v. West 72 Hour Hackathon, Deep Tech / Physical AI track. It is Vantage (his AR-glasses startup with Boris, Project Ithaca) rebuilt around a room agent: iPhone (SpatialBridge app) streams ARKit pose + LiDAR + camera to a Mac; the Mac compositor (`shell/`, carried over from Spatula) renders real Mac windows in 3D; a Python agent (`agent/`) sends Gemini the view plus a scene graph and executes spatial actions; hand tracking runs on the Mac (`hands/`, RTMPose + LiDAR). New this cycle: a people layer (faces, voices, a bubble beside each head with name and what you talked about). Judges score every 12 h on innovation / technical / business / presentation, and the placement score is the final plus the average of the five check-ins, so every check-in needs a demoable build.

## Scores so far

| Check-in | Innovation | Technical | Business | Presentation | Judge feedback |
|---|---|---|---|---|---|
| 1 (hour 12) | 8 | 4 | 6 | 8 | Position as "memory for AR"; why won't Apple/Meta build it natively; name 2-3 killer use cases ("where is my charger" is a good one). |
| 2 (hour 24) | 7 | 6 | 7 | 7 | The differentiator is that it is *directional* (points you to where something is) plus persistent spatial memory. |

Innovation dropped a point at check-in 2. Check-in 3 must show a visibly new capability on camera (people bubbles, hand-launch), not fixes. The "directional memory" framing is for this hackathon's pitch only; do not carry it into Ithaca/Vantage material.

## Rig state, measured Sun 6:35 PM PT

- `mac-shell` is running (pid 5721) and the control socket answers.
- `stats` returns `packet_rate=0.0 panels=0 frame_age_ms=8806523`. **The phone has been disconnected about 2.4 hours**; the log repeats `spatial_bridge: heartbeat sendto: Host is down` and the newest exported frame is 4:45 PM.
- `hands track`, `tempo objects`, and `tempo people` are all still running against that dead stream. Harmless, but their output is stale.
- `hands status` reports `source=phone`, not `mac`, so the Mac tracker is not feeding the shell.
- `~/.config/spatial-os/gestures.toml` **does not exist**: `hands calibrate --write` has never actually been run, so the rig is still on default thresholds. That is the cheapest available fix for Tarun's standing "gestures barely work" complaint.
- 7 people are enrolled in the people store and 5 conversation lines are logged, all from the earlier session.

Reopen SpatialBridge on the iPhone before attempting any verification below. Nothing here can be checked while `packet_rate` is 0.

## Deadlines and submission mechanics

- Check-ins are every 12 h; the check-in-2 card was stamped hour 24. Confirm the exact next deadline with Tarun before assuming (the briefing said Sun 7 AM / Sun 7 PM / Mon 7 AM PT, final Mon 9 AM PT; his checkpoint-1 correction was midnight).
- Each check-in: 60 s video + a doc (repo link, what changed) in Google Drive → Team Submissions (12 hour cycle) → deep tech → `Tempo`. Only Tarun uploads. `docs/checkpoint-1-submission.md` and `docs/checkpoint-2-submission.md` are the texts he used; reuse the structure.
- Code lives ONLY in this repo. Do NOT push to the official hackathon fork or open upstream PRs until Tarun says so.
- Final also emailed to hackofthrones@gmail.com. Google Cloud env is hard-deleted Mon Sep 14 noon ET.

## How to bring the rig up

```bash
cd "/Users/tarunyadgirkar/TarunsCode/east west hackathon/tempo"
shell/scripts/build-mac.sh && shell/scripts/test-mac.sh        # 21 mac-shell + 8 gesture-engine + 2 bridge tests
shell/scripts/run-mac.sh --restart                              # installs /Applications/Spatula.app and launches it
export SPATIAL_OS_SOCK="$TMPDIR/spatial-os.sock"
cd hands  && uv run hands track --dir "$TMPDIR/spatula-frames" --enable-export --raw &   # Mac hand tracking (~18 fps)
cd ../agent && uv run tempo people --frames "$TMPDIR/spatula-frames" &                    # faces + voices + bubbles, mic open
uv run tempo objects --frames "$TMPDIR/spatula-frames" &                                   # object map (optional)
uv run tempo scene                          # what the agent sees (now includes `people`)
uv run tempo --speak ask who is in the room
uv run tempo live                           # pinch-and-talk
uv run tempo people list | forget NAME | summarize NAME | me Tarun   # store admin; `me` records 6 s of the wearer's voice
cd ../hands && uv run hands calibrate --write                          # open / pinch / fist, 6 s each -> ~/.config/spatial-os/gestures.toml (shell restart to load)
```

Checks: `pgrep -fl mac-shell`; `printf 'stats\n' | nc -U $TMPDIR/spatial-os.sock` (use the Python client in `hands/src/hands/control.py` if nc prints nothing). `packet_rate=0` means the phone is not streaming. `frame-export status` must be `on` for the hands/people/objects daemons; a shell restart turns it off, `hands track --enable-export` turns it back on. Restarting the shell needs Tarun's OK when he might be recording.

Gemini key: `GEMINI_API_KEY` in the agent's env file, model `gemini-3.6-flash`. Hooks block Claude from touching that file, and the bash-guard hook rejects any Bash command whose text even mentions its filename: test env-dependent code through the CLI (`tempo ...`), never name the file in a command.

### Phone (SpatialBridge)

- Xcode project `~/TarunsCode/ithaca/Spatula/iphone-bridge/SpatialBridge.xcodeproj`, scheme `SpatialBridge`, bundle `com.spatialos.spatialbridge`, Tarun's iPhone 17 Pro device id `02685A41-CDBC-50E8-8883-8EEE10889194`. Reinstall over USB: `xcodebuild -project ... -scheme SpatialBridge -destination "id=<id>" -allowProvisioningUpdates build`, then `xcrun devicectl device install app --device <id> <.app>` and `... process launch --device <id> com.spatialos.spatialbridge`. Done once on Sun Sep 13.
- A fresh install must accept iOS's Local Network prompt before it sends anything. The app browses Bonjour `_spatialbridge._udp` and also has manual host entry (default port 9898); on campus Wi-Fi Bonjour may be blocked, so enter the Mac's IP. Phone and Mac must share a network (or the phone's hotspot).

## State of each piece

| Piece | State | Where |
|---|---|---|
| Agent loop (voice/typed → Gemini → actions) | live, 3 to 4 s/request | `agent/src/tempo/{brain,actions,live}.py` |
| Placement via depth cast + full pose | live; wall error 0.05 m | README results table, `agent/eval/results/` |
| Launched windows | FIXED Sun: free-slot spawn, depth pull-in, new panel takes focus, relaunch recalls instead of duplicating (`scene::recall_panel`) | `shell/mac-shell/src/core/scene.cpp`, `platform/capture.mm` |
| Mac hand tracking | live ~18 fps; phantom hands past 1 m / conf < 0.5 gated (`--max-range-m`, `--min-confidence`) | `hands/src/hands/tracker.py` |
| Gesture thresholds | `hands calibrate` written and unit-tested; NOT yet run on Tarun's hand | `hands/src/hands/calibrate.py` |
| People layer | written, verified offline (3 faces on a real frame; TTS audio: names parsed, ECAPA 0.89 same / 0.0 cross speaker; Gemini summary works). NOT yet run with a face in front of the live rig | `agent/src/tempo/{people,faces,voices,siyi}.py`, `agent/tests/test_people.py` |
| Object map | live at conf 0.25 | `agent/src/tempo/objects.py` |
| siyi (CRM) notes in bubbles | code done, needs `TEMPO_SIYI_URL` + `TEMPO_SIYI_KEY` in the agent env (Supabase URL + service key from `~/TarunsCode/shared/siyi.app`) | `agent/src/tempo/siyi.py` |

Stores: `~/.config/tempo/people.json`, `~/.config/tempo/conversations.jsonl`, `~/.config/tempo/objects.json`, `~/.config/tempo/memory.json`. Face models in `agent/models/` (gitignored; copied from `~/TarunsCode/facelock/models` or downloaded). ECAPA weights cache at `~/.cache/tempo/ecapa`.

## How the demo gestures work

- Hold a fist ~300 ms → launcher opens; rotate the wrist to pick an entry (Test Card, Safari, Terminal, Finder); release → launches. Pinch = click on the aimed panel. Double-pinch on empty space → `gather-panels` fans every panel into an arc in front of you.
- People: a face in view gets a "Someone new" bubble to the wearer's right of the head. Someone saying "I'm Alice" names their face and voice; the wearer saying "this is Bob" names the largest face in view. After 3 lines, Gemini writes a one-sentence summary into the bubble. `tempo ask who is this` answers from the scene's `people` block.

## Next: the hardware session that was never done (needs Tarun, ~10 min)

1. Face: point the phone at a face (mirror, person, or a photo on a screen). Bubble should appear within a second. Say "I'm Tarun", then two sentences; the title should flip and a summary should appear. Check `/tmp/claude-people.log`-style output (`tempo people` prints `heard:` and `people:` lines).
2. `uv run hands calibrate --write` (three prompted poses), then restart the shell to load `gestures.toml`. Record per-phase jitter in mm; put it in the README hands section.
3. Fist → launcher → pinch on the window, on camera. That plus the people bubble is the check-in-3 video.
4. `tempo people me Tarun` so the wearer's voice is never filed under a guest.

## After that, in priority order

1. Check-in 3 doc + video (structure from `docs/checkpoint-2-submission.md`; must say what changed since check-in 2).
2. Placement eval with the desk in view + object-anchored requests (`agent/eval/requests.json`), update README table.
3. Bubble UX: the note panel is full note size; a compact person card would read better. `note` JSON only takes title/body/accent today (`control_server.cpp` `parse_note_json`).
4. Pico bring-up (pose/depth streaming from a microcontroller-class board) was promised in the check-in-2 doc as "start", keep it honest.
5. WiLoR on a GCE GPU for the pointing path, only if hands still feel bad after calibration. See the credits section below.

## Google Cloud and AI Studio credits, how to actually use them

Each team was assigned a dedicated Google Cloud project, claimed one row per team, first come first served, off the sheet linked from Caleb Moulema's Sep 12 email "Passwords + log in for ai studio for Google". The row carries a username, a password, and a project id. Ours is **`eastwest72hack26bos-505`**. The same credentials log into both `console.cloud.google.com` and `aistudio.google.com`.

**The whole project is hard-deleted Monday Sep 14 at noon ET, which is 9 AM Pacific**, the same hour as the final submission. Nothing built on those credits can sit on the demo's critical path.

Used today: only the AI Studio half, via the Gemini key in the agent's env file. The Cloud Console half is untouched, and `gcloud` is not installed on this machine. Two uses are worth the remaining hours.

1. **Move agent inference onto Vertex AI.** `agent/src/tempo/brain.py` builds `genai.Client(api_key=key)` from a raw AI Studio key. Pointing it at Vertex on project `eastwest72hack26bos-505` puts the calls on sponsored quota instead of a free tier that can throttle mid-demo with judges watching. About fifteen minutes plus a `gcloud` install and auth. Keep the API-key path behind an env switch so a deleted project on Monday morning cannot take the demo down.
2. **Run WiLoR on a GPU VM as an offline batch.** The checkpoint-1 doc promised the judges a WiLoR trial on sponsored GPU credits for the pointing path. One L4 instance, a few hundred recorded frames from the export dir, publish accuracy and round-trip latency against local RTMPose. That closes the promise with numbers.

Do **not** wire a live cloud inference hop into the demo loop. Campus Wi-Fi plus a project that disappears Monday at 9 AM Pacific is the kind of fragility that earned the "buggy" Technical score.

Tarun has to pull the username and password from his claimed row; Claude cannot handle those credentials. Once `gcloud` is authorized locally the rest is scriptable.

## Gotchas

- Never `git add -A` while background agents edit; stage by path.
- Shell tests: a new spawn now takes focus (`test_scene.cpp` was updated for this).
- `hands` daemon `--enable-export` turns export off when it exits; killing it kills the frame feed for the other daemons.
- Google Docs editing through the Chrome extension: keystrokes race the find dialog and cmd+a can replace the whole doc. Verify with the Drive `read_file_content` tool, click fields explicitly, triple-click to select field text, never cmd+a.
- A temporary `genai.Client` gets closed mid-call; keep one module-level client (see `people.summarize`).
- Camera TCC: processes launched from Claude sessions have no camera grant (mic is fine). Faces come from the shell's exported frames, not a webcam, so this does not affect the people daemon.
