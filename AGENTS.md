# Tempo

East v. West 72 Hour Hackathon entry. Solo (Tarun Yadgirkar, UC Berkeley), Deep Tech / Physical AI track. AR room agent: iPhone streams ARKit pose+LiDAR+camera to a Mac compositor (`shell/`); a Python agent (`agent/`) sends Gemini the view + scene graph and executes spatial actions; hand tracking on the Mac (`hands/`, RTMPose + LiDAR). See `NEXT-SESSION.md` for the full start-here brief, `README.md` for the checkpoint log, `shell/HACK_CHANGES.md` for every compositor change.

## Ongoing

Updated: 2026-09-13T19:57:00-07:00 by claude session

Done:
- People layer (faces YuNet+ArcFace, voices mlx-whisper+ECAPA, siyi CRM), people in scene JSON, one-line Gemini summary per person bubble. (2441ea3, 6ea7356)
- Hands calibrate code written + unit-tested; `gestures.toml` NOT yet run on Tarun's hand. (94c6cca, f5ec107)
- Launched-window fixes: free-slot spawn, depth pull-in, focus on spawn, `recall_panel` on relaunch. (Sun morning; see `shell/HACK_CHANGES.md`)
- Depth-cast placement + full pose: wall error 0.05 m. (README results table)
- Check-in 1 (hour 12) and check-in 2 (hour 24) submitted; `docs/checkpoint-{1,2}-submission.md`.
- Handoff docs: `NEXT-SESSION.md`, README checkpoint log, `shell/HACK_CHANGES.md`. (f055a7f, 7af47ff, 7a5dea3)

In flight:
- Check-in 3 (Sun 7 PM PT) — CONFIRM WITH TARUN whether the 7 PM card was stamped. Doc structure from `docs/checkpoint-2-submission.md`; must say what changed since check-in 2 (people bubbles live, hand-launch on camera). NOT yet written.
- Hardware session (needs Tarun, ~10 min): (1) point phone at a face, say "I'm Tarun" + two sentences, verify bubble + summary; (2) `uv run hands calibrate --write` then restart shell, record per-phase jitter; (3) fist -> launcher -> pinch on camera = check-in-3 video; (4) `tempo people me Tarun`.

Blocked:
- mac-shell (Spatula.app) is NOT running; control socket `$TMPDIR/spatial-os.sock` is gone. Last exported frame is 4:45 PM PT (~3h stale). `hands track` (pid 9829), `tempo people` (pid 11106), `tempo objects` (pid 38976) daemons are running but feeding on that stale frame — zombies. Reopen SpatialBridge on the iPhone AND relaunch mac-shell (`shell/scripts/run-mac.sh --restart`) before any verification.
- `gestures.toml` still does not exist; `hands calibrate --write` has never been run on Tarun's hand. Cheapest fix for the standing "gestures barely work" complaint.
- siyi CRM notes need `TEMPO_SIYI_URL` + `TEMPO_SIYI_KEY` in the agent env (Supabase URL + service key from `~/TarunsCode/shared/siyi.app`).

Next (in priority order):
1. Confirm check-in-3 deadline/submission status with Tarun (briefing said Sun 7 PM PT; his checkpoint-1 correction was midnight — confirm before assuming).
2. Bring the rig up: reopen SpatialBridge on the iPhone, `shell/scripts/run-mac.sh --restart`, restart the three daemons against a live export dir. Verify `packet_rate > 0`.
3. Hardware session (above, ~10 min with Tarun) -> check-in-3 video.
4. Check-in-3 doc + upload to Drive (Team Submissions -> deep tech -> Tempo). Only Tarun uploads.
5. Placement eval with the desk in view + object-anchored requests (`agent/eval/requests.json`); update the README results table.
6. Bubble UX: a compact person card would read better than the full-size note panel (`control_server.cpp` `parse_note_json` takes title/body/accent only today).
7. Pico bring-up (pose/depth streaming from a microcontroller-class board) — promised in the check-in-2 doc as "start"; keep it honest.
8. Google Cloud credits (OFF the critical path; see `NEXT-SESSION.md` "Google Cloud" section): (a) move agent inference onto Vertex AI on sponsored quota, behind an env switch so a project deleted Mon 9 AM PT cannot take the demo down; (b) run WiLoR as an offline batch on one L4 GCE VM to publish the numbers promised to judges. Do NOT wire a live cloud hop into the demo loop. Tarun must pull username/password from his row; `gcloud` is not installed yet.

Notes:
- Never `git add -A` while background agents edit; stage by path.
- The Gemini key lives in the agent env file; hooks block Claude from touching it and the bash-guard rejects any Bash command that names the file. Test env-dependent code through the CLI (`tempo ...`), never name the file in a command.
- Git push from a normal terminal on the Mac (not device_bash).
- Google Cloud project `eastwest72hack26bos-505` is hard-deleted Mon Sep 14 noon ET = 9 AM PT = the final deadline. Nothing built on those credits can sit on the demo's critical path.
- Scores so far: check-in 1 (8/4/6/8), check-in 2 (7/6/7/7) across Innovation/Technical/Business/Presentation. Innovation dropped a point at check-in 2; check-in 3 must show a visibly new capability on camera (people bubbles, hand-launch), not fixes.
