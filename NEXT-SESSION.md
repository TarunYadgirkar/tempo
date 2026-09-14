# Next session, start here

Updated: 2026-09-13T18:35-07:00 by claude session. Read this, then `README.md` (checkpoint log at the bottom), then `shell/HACK_CHANGES.md`.

## What Tempo is, in one paragraph

Tarun's solo entry to the East v. West 72 Hour Hackathon, Deep Tech / Physical AI track. It is Vantage (his AR-glasses startup with Boris, Project Ithaca) rebuilt around a room agent: iPhone streams ARKit pose + LiDAR + camera to a Mac; the Mac compositor (`shell/`, carried over from Spatula) renders real Mac windows in 3D; a Python agent (`agent/`) sends Gemini the view plus a scene graph (head pose, depth casts, objects, people, remembered places, hand aim, panels) and executes spatial actions. Hand tracking runs on the Mac (`hands/`, RTMPose + LiDAR). Judges score every 12 h on innovation / technical / business / presentation.

The pitch, as of checkpoint 2: **memory for AR, and it is directional** — it points you to where a thing is, not just text floating in view, and the memory persists across sessions. That framing is for this hackathon only; do not carry it into Ithaca or Vantage docs.

## Clock

Now: Sun Sep 13, ~6:35 PM PT. Observed cadence has been midnight and noon PT, so **the next checkpoint is Sun Sep 13 midnight PT, roughly 5 h out**. Confirm with Tarun before trusting it; the briefing's stated times and the real ones have disagreed twice.

Placement = final score + the average of the five check-in scores, so check-ins are half the grade and a missed one is a zero. Judges score what they see at that hour, not its potential.

Scorecards so far:

| Checkpoint | Innovation | Technical | Business | Presentation | Weighted |
|---|---|---|---|---|---|
| 1 (hour 12) | 8 | 4 | 6 | 8 | 6.50 |
| 2 (hour 24) | 7 | 6 | 7 | 7 | 6.75 |

Weights: Innovation 30, Technical 25, Business 25, Presentation 20. Technical is the historical weak spot ("basic, buggy"), and Innovation fell 8 to 7 at checkpoint 2 because the work was fixes rather than a new capability. **Checkpoint 3 needs one visible new thing that demonstrably works on camera.** Track was mislabeled "AI Apps" on the checkpoint-1 card and has since been corrected to Deeptech.

## Rig state right now

Measured at 6:33 PM PT:

- `mac-shell` is running (pid 5721) and the control socket answers.
- `stats` returns `packet_rate=0.0 panels=0 frame_age_ms=8806523`. The phone has been disconnected for about 2.4 hours; the log is repeating `spatial_bridge: heartbeat sendto: Host is down`. Newest frame in the export dir is 4:45 PM.
- The `hands track`, `tempo objects`, and `tempo people` daemons are all still running against that dead stream. They are harmless but their output is stale.
- `hands status` says `source=phone`, not `mac`, so the Mac tracker is not currently feeding the shell.

**Hardware session needed:** reopen SpatialBridge on the iPhone. If Bonjour is blocked on campus Wi-Fi, use the app's manual host entry on port 9898. Nothing below can be verified until frames flow again.

## Done since checkpoint 2

All pushed to `main`, remote is level with local at `f5ec107`.

- `4371ce9` hands range and confidence gate, drops phantom detections past arm's reach.
- `72e98ae` shell spawns panels into a free slot with depth pull-in, focuses new panels, recalls instead of duplicating a launch. 21 mac-shell tests green.
- `ec1b960`, `6ea7356`, `cd0d5d6` the people layer: YuNet + ArcFace faces, ECAPA voiceprints, mlx-whisper open mic, a bubble beside each head, names learned from "I'm X", stored in the tempo config dir, people exposed in the scene JSON. 7 people are currently enrolled.
- `94c6cca` `hands calibrate` with three prompted phases (open, pinch, fist).
- `2441ea3` one-line Gemini summary per person in the bubble plus a conversation log. 5 lines logged so far.
- `f5ec107` docs.

Only uncommitted file is `agent/last-view.jpg`, a scratch artifact.

## Blocked

- **Phone offline.** Blocks every hardware verification below. Only Tarun can fix it.
- **Gesture thresholds were never written.** `hands calibrate --write` has not been run: `~/.config/spatial-os/gestures.toml` does not exist. The calibration code shipped but the rig is still on defaults, and "gestures barely work" is Tarun's standing complaint. The shell must be restarted after the file is written for it to load.
- **One-Euro lag fix and the palm-range depth model are still unmeasured on a real hand.** They are in the build; the last published jitter number, 36 mm per joint, predates them.
- **Google Cloud credentials are not in hand.** See below.

## Google Cloud and AI Studio credits, how to actually use them

The organizers assigned each team a dedicated Google Cloud project, claimed one row per team, first come first served, off a shared sheet linked from Caleb Moulema's Sep 12 email "Passwords + log in for ai studio for Google". The row carries a username, a password, and a project id. Ours is **`eastwest72hack26bos-505`**. The same credentials log into both `console.cloud.google.com` and `aistudio.google.com`.

**Everything in that project is hard-deleted Monday Sep 14 at noon ET, which is 9 AM Pacific.** That is the same hour as the final submission, so nothing built on those credits can sit on the demo's critical path.

What is used today: only the AI Studio half, via the Gemini key in the agent env file, model `gemini-3.6-flash`. The Cloud Console half is completely untouched. `gcloud` is not installed on this machine.

Two uses are worth the remaining hours, in priority order.

1. **Move agent inference onto Vertex AI.** [agent/src/tempo/brain.py:32](agent/src/tempo/brain.py:32) builds `genai.Client(api_key=key)` from a raw AI Studio key. Swapping it for a Vertex client on project `eastwest72hack26bos-505` puts the calls on sponsored quota instead of a free tier that can throttle mid-demo with judges watching. Roughly a fifteen-minute change plus a `gcloud` install and auth. Keep the API-key path as a fallback behind an env switch so a dead project on Monday morning cannot take the demo down.
2. **Run WiLoR on a GPU VM as an offline batch.** The checkpoint-1 doc promised the judges a WiLoR trial on sponsored GPU credits for the pointing path. Spin up a single L4 instance, push a few hundred recorded frames from the export dir, and publish accuracy and round-trip latency against local RTMPose. That closes the promise with real numbers.

**Do not** wire a live cloud inference hop into the demo loop. Campus Wi-Fi plus a project that disappears Monday at 9 AM Pacific is exactly the kind of fragility that earned the "buggy" Technical score.

Tarun has to pull the username and password from his claimed row; Claude cannot handle those credentials. Once `gcloud` is authorized locally, the rest is scriptable.

## How to bring the rig up

```bash
cd "/Users/tarunyadgirkar/TarunsCode/east west hackathon/tempo"
shell/scripts/build-mac.sh && shell/scripts/test-mac.sh        # 31 ctest
shell/scripts/run-mac.sh --restart                              # installs /Applications/Spatula.app, launches, phone auto-connects
export SPATIAL_OS_SOCK="$TMPDIR/spatial-os.sock"
cd hands && uv run hands track --dir "$TMPDIR/spatula-frames" --enable-export --raw &   # Mac hand tracking daemon
cd ../agent && uv run tempo objects --frames "$TMPDIR/spatula-frames" &                 # object map daemon
uv run tempo people --frames "$TMPDIR/spatula-frames" &                                 # faces and voices
uv run tempo scene            # what the agent sees
uv run tempo --speak ask put a note on the wall that says hello
uv run tempo live             # pinch-and-talk (local whisper)
uv run tempo eval geometry pixels     # 20-request placement eval, closes all panels first
cd ../hands && uv run hands eval --dir "$TMPDIR/spatula-frames" --seconds 60 --window-s 5   # phone vs mac hand jitter
uv run hands check --dir "$TMPDIR/spatula-frames"                                          # reprojection check
uv run hands calibrate --write                                                             # then restart the shell
```

Checks before assuming anything is broken: `pgrep -fl mac-shell`; `printf 'stats\n' | nc -U $TMPDIR/spatial-os.sock` (`packet_rate=0` means the phone app is closed). `frame-export status` must say `on` for the hands, objects, and people daemons to get frames; a `run-mac --restart` turns it off, re-enable with `frame-export on <dir> --raw`.

The Gemini key lives in the agent env file; hooks block Claude from writing env files, Tarun pastes via `open -e`. Whisper model is `mlx-community/whisper-large-v3-turbo`, cached. Owner voice enrollment is `uv run tempo people me Tarun`.

## Next, in order

1. Get the phone streaming again. Confirm `packet_rate` is non-zero before anything else.
2. Run `hands calibrate --write` with Tarun's hand, restart the shell, and confirm the thresholds file exists and loaded. This is the cheapest fix for his number one complaint.
3. With his hand up, run `hands eval` and `hands check`. Publish before-and-after jitter against the 36 mm baseline. This is the Technical-score evidence.
4. Pick the one visible new capability for checkpoint 3 and make it work on camera. The standing candidate is the hand launcher: hold a fist 300 ms to open the launcher, rotate the wrist to pick, release to launch, pinch to click, double-pinch on empty space to gather panels. Innovation will not recover without something new.
5. Vertex AI swap in `brain.py` once credentials are in hand, with the API-key fallback preserved.
6. WiLoR batch on a GPU VM, numbers only, no live path.
7. Checkpoint-3 video and doc, same structure as `docs/checkpoint-1-submission.md`: why me, what we're building, the question, what shipped, next 12 h, feedback wanted. Must say what changed since checkpoint 2. Drop in Drive, Team Submissions (12 hour cycle), deep tech, `Tempo`. Only Tarun can upload the video.

Judge questions still owed an answer in the doc: why won't AV and headset companies build this natively, and two or three killer use cases. "Where is my charger" tested well as a hook.

## Constraints and lessons

- Code lives only at https://github.com/TarunYadgirkar/tempo on `main`. Do not push project code to the official hackathon fork or open upstream PRs until Tarun says so. The fork's `teams/Tempo` holds a pointer README only. Final is also emailed to hackofthrones@gmail.com.
- "This has to be insane and amazing" and "use great open source stuff". RTMPose, OWL-ViT, Whisper, ArcFace, ECAPA are in; WiLoR, HaMeR, HaWoR are the next candidates.
- Every hand-skeleton offset so far has been a pipeline bug, not model quality. Run `hands check` before swapping models.
- UI mode is polished. The design rules in `~/.claude/CLAUDE.md` apply to note cards and any UI.
- Never restart the shell while Tarun might be recording. Ask first. A restart reconnects the phone but kills frame export.
- Tell Tarun in one line whenever a hardware session is needed, meaning his hand or his desk in view.
- Never `git add -A` while background agents are mid-edit. Stage by path.
- The bash-guard hook blocks any Bash command whose text mentions the agent's dot-env filename, even inside a heredoc or a comment. Test env-dependent code through the CLI.
- Google Docs plus the Chrome extension: keystrokes race the find dialog and cmd+a can replace the whole document. Click fields explicitly, triple-click to select field text, never cmd+a, and verify with the Drive reader afterwards.

## Known gaps and bugs

- `agent/eval` scores a desk request as wrong when no horizontal surface is in view, so both conditions fail. That is the sensor's limit, documented in the README.
- The object detector at confidence 0.25 still emits false positives such as headphones and clocks. Raise to 0.3 if it pollutes the scene JSON.
- `shell/HACK_CHANGES.md` still says the LiDAR map is 32x24; it is 256x192.
- Left and right placement was once swapped because the portrait camera's axes leaked into "right", fixed with forward cross world-up. Watch for regressions if the head math changes.
- The RTMPose `chirality` guess assumes a dorsal, head-mounted view. If hands flip on the real rig, pass `--palmar-view`.
- `tempo people summarize NAME` needs a module-level genai client; a temporary client gets closed mid-call.
- The placement eval has still never been run with a desk in view. Ask Tarun to point the phone at his desk and add object-anchored requests to `agent/eval/requests.json`.
