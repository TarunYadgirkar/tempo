# Tempo

East v. West 72 Hour Hackathon entry. Solo (Tarun Yadgirkar, UC Berkeley), Deep Tech / Physical AI track. AR room agent: iPhone streams ARKit pose+LiDAR+camera to a Mac compositor (`shell/`); a Python agent (`agent/`) sends Gemini the view + scene graph and executes spatial actions; hand tracking on the Mac (`hands/`, RTMPose + LiDAR). See `NEXT-SESSION.md` for the full start-here brief, `README.md` for the checkpoint log, `shell/HACK_CHANGES.md` for every compositor change.

## Ongoing

Updated: 2026-09-14T22:35:00-07:00 by claude session

Done (pushed, main at dd2693d):
- Check-in 4 and final submission text: docs/checkpoint-4-submission.md, docs/final-submission.md; README checkpoint log rows 3 and 4. Google Docs created in Drive: "Check in 4 doc" (Check in 4 folder) and "Final doc" (Final folder); the earlier [fill] template was renamed "Check in 4 template (superseded)". One bracket left for Tarun in the check-in-4 doc: the hardware (Pico/display) sentence.
- People store: 15 duplicate identities merged into one "Tarun Yadgirkar" record (owner, 12 face vectors, clean summary). Finding: ArcFace cross-frame similarity 0.30 to 0.57 vs FACE_MATCH 0.42 spawned 32 records for one face; per-session clustering is the real fix.
- people.py bubble text uses straight quotes (shell font has no curly-quote glyphs; they rendered as ???). 22 agent tests pass via `uv run --with pytest python -m pytest -q tests/` (pytest is not in the agent venv).
- Rig staged for the final video: layout "final" saved = ring of 6 panels at 1.6 m facing the head with the centre open for hands (name note top-left, Remembered/charger note top-right, Safari left, Terminal right, Notes bottom-left, Finder bottom-right). Real Safari/Notes/Terminal windows captured via `launch-app com.apple.<Bundle>`.

In flight:
- Tarun is recording the final demo on this layout (Mon ~10:35 PM PT). Do not restart the shell or touch panels until he says done.
- Daemons in screen sessions: tempo-hands (13-16 fps), tempo-objects, tempo-people (started with --no-mic: the mic path throws PortAudio -50 and hangs the daemon before it logs "watching"). Logs at $TMPDIR/tempo-logs/*.log; stdout is block-buffered so the people log looks empty while it is running.

Blocked (on Tarun):
- Final submission upload (video + docs are his), and the final email to hackofthrones@gmail.com.
- gestures.toml still absent; `hands calibrate --write` never run cleanly on his hand.
- 30-request placement eval never run; WiLoR never run (MANO gated); Vertex path never flipped on. All three are described honestly as not-done in the docs.

Next:
1. After the recording: staging mode that freezes gesture injection (pinch/fist fired mid-shot: launcher opened Terminal/Finder, gather-panels re-fanned the layout).
2. `layout load` spawns duplicate notes and fans apps in front of the head instead of restoring saved poses; fix or document.
3. Spawn pull-in uses the nearest depth surface, which is the wearer's own face when the phone is held selfie-style; clamp the minimum spawn distance.
4. Bubble sizing for near faces (0.30 m offset fills the view at arm's length).
5. Per-session face clustering instead of the constant FACE_MATCH.

Notes:
- The installed Spatula build predates 1eb0e88: `close-all` returns parse_error on the live shell. Close windows one by one.
- `launch-app` wants a bundle id (com.apple.Safari); `app:Safari` yields a "no-window" placeholder card.
- Screenshots right after `pose` catch panels mid-animation; wait ~4 s.
- Never git add -A while background agents edit; stage by path.
- Google Cloud project eastwest72hack26bos-505 (gcloud authed as devstar5053@gcplab.me) is hard-deleted at the deadline; nothing in the demo depends on it.
- Scores: check-in 1 (8/4/6/8), check-in 2 (7/6/7/7); check-in 3 score not yet seen (no scorecard comment on the doc as of Mon 10:20 PM).
