# Tempo

East v. West 72 Hour Hackathon entry. Solo (Tarun Yadgirkar, UC Berkeley), Deep Tech / Physical AI track. AR room agent: iPhone streams ARKit pose+LiDAR+camera to a Mac compositor (`shell/`); a Python agent (`agent/`) sends Gemini the view + scene graph and executes spatial actions; hand tracking on the Mac (`hands/`, RTMPose + LiDAR). See `NEXT-SESSION.md` for the full start-here brief, `README.md` for the checkpoint log, `shell/HACK_CHANGES.md` for every compositor change.

## Ongoing

Updated: 2026-09-13T20:40:00-07:00 by claude session

Done (this session, all pushed to main):
- Handoff files created: AGENTS.md + CLAUDE.md (@AGENTS.md). (3259f4c)
- Check-in-3 submission doc drafted: docs/checkpoint-3-submission.md (structure from check-in-2; what changed since check-in 2 = people layer + launched-window fixes + hands gate; honest limits; 60s video TBD). (98d1dbd)
- Placement eval extended: +10 requests (ids 21-30) in agent/eval/requests.json — 5 desk-in-view (expect:"table"), 5 object-anchored (expect:"object" + anchor); agent/eval/RUNBOOK.md. (75da639)
- Scorer closed the object-anchor gap: agent/src/tempo/eval.py scores expect:"object" by Euclidean distance to the anchor object's 3D pos (threshold ANCHOR_NEAR_M=0.30), missing anchor = anchor_not_found; additive, legacy unchanged; 12 tests green. (d26218b)
- Vertex AI opt-in inference path: agent/src/tempo/brain.py, TEMPO_USE_VERTEX=1 + TEMPO_VERTEX_PROJECT/TEMPO_VERTEX_LOCATION (defaults eastwest72hack26bos-505/us-central1), ADC, try/except fallback to API-key client; 16 tests green; no creds in code. (074833c)
- WiLoR offline batch runner: hands/wilor_batch.py (record/run/compare; reuses hands geometry pipeline), hands/WILOR_DEPLOY.md (GCE L4 runbook), hands/wilor_requirements.txt; targets rolpotamias/WiLoR CVPR 2025; one gated step = MANO_RIGHT.pkl from mano.is.tue.mpg.de. (7f07b73)
- Rig brought up: mac-shell pid 41795, packet_rate=3540, 3 panels, frame-export ON 30 Hz rgb8; daemons in detached screen sessions (hands 49318 / people 49316 / objects 49317); tempo scene sees 398 objects, 1 remembered place (charger), 1 person present.

In flight:
- gcloud CLI installed (584.0.0). Awaiting Tarun's `gcloud auth login` + `gcloud auth application-default login` + `gcloud config set project eastwest72hack26bos-505` to enable Vertex AI + Compute APIs and run the WiLoR GCE batch.

Blocked (on Tarun):
- Hardware session (~10 min): (1) point phone at a face, say "I'm Tarun" + two sentences, verify bubble + summary; (2) `uv run hands calibrate --write` then restart shell, record per-phase jitter; (3) fist -> launcher -> pinch on camera = check-in-3 video; (4) `tempo people me Tarun`.
- Placement eval run needs the desk + target objects in the camera view (Tarun positions the rig); then `uv run tempo eval` per agent/eval/RUNBOOK.md.
- WiLoR frame recording needs a hand in view — record during the hardware session (`hands/wilor_batch.py record --live "$TMPDIR/spatula-frames" --out <batchset> --seconds 30`).
- MANO_RIGHT.pkl gated download (free registration at mano.is.tue.mpg.de) — the one non-scriptable WiLoR step.
- gestures.toml still does not exist; hands calibrate --write never run on Tarun's hand.
- siyi CRM notes need TEMPO_SIYI_URL + TEMPO_SIYI_KEY in agent env (Supabase from ~/TarunsCode/shared/siyi.app).

Next (in priority order):
1. Hardware session (above) -> check-in-3 video (OVERDUE; check-in 3 was due Sun 7 PM PT). Confirm with Tarun whether the 7 PM card was stamped.
2. Run the placement eval with desk in view + object-anchored requests; update the README results table.
3. Record the WiLoR frame set (hand in view), then run hands/wilor_batch.py on a GCE L4 (after gcloud auth) to publish WiLoR-vs-RTMPose accuracy + latency; tear the VM down before Mon 9 AM PT.
4. Optionally flip TEMPO_USE_VERTEX=1 once ADC is authed (off critical path; falls back to API key if Vertex fails).

Notes:
- Daemons run in screen sessions tempo-hands/tempo-people/tempo-objects (reattach `screen -r`); logs at $TMPDIR/tempo-logs/{hands,people,objects}.log.
- Never git add -A while background agents edit; stage by path.
- The Gemini key lives in the agent env file; hooks block Claude from touching it and bash-guard rejects any Bash command naming it. Test env-dependent code via the CLI (tempo ...), never name the file in a command.
- Git push from a normal terminal on the Mac (not device_bash).
- Google Cloud project eastwest72hack26bos-505 is hard-deleted Mon Sep 14 noon ET = 9 AM PT = the final deadline. Nothing built on those credits can sit on the demo's critical path.
- Scores so far: check-in 1 (8/4/6/8), check-in 2 (7/6/7/7) across Innovation/Technical/Business/Presentation. Innovation dropped a point at check-in 2; check-in 3 must show a visibly new capability on camera (people bubbles, hand-launch), not fixes.
