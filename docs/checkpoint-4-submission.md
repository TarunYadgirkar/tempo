Tempo, checkpoint 4 (text of the Google Doc submitted Mon Sep 14 night)

Team: Tarun Yadgirkar (UC Berkeley)
Track: Deep Tech / Physical AI (Freestyle)
Resources: https://github.com/TarunYadgirkar/tempo | Video demo attached in the submission folder

Vantage: Outside this hackathon, I'm building lightweight AR glasses and the spatial OS behind them with my co-founder, where we recently interviewed with A16Z.

What changed since checkpoint 3

Checkpoint 3 was bring-up: the people layer, the real-app launcher, and the hardware path each worked on their own. This cycle was about running them together on the live rig and being honest about what the measurements say.

People memory, live on the rig

The people layer ran against the live camera tonight instead of exported frames. My own face was detected on the first frame it appeared in, unprojected through the LiDAR map, and given a bubble beside the head. The name attached to it is now persistent: point the rig at me again and the bubble reads "Tarun Yadgirkar" with the one-line summary, without re-enrolling.

The interesting finding was in the store. Over the weekend the recognizer had accumulated 32 separate person records for what was essentially one face, because ArcFace cross-frame similarity in this lighting sits between 0.30 and 0.57 and the match threshold was 0.42, tuned on FaceLock's 4000-face bank. Every frame that scored 0.40 became a "new person". I merged the duplicates into one identity (15 records folded in, embeddings kept, 12 face vectors retained) and the daemon now resolves my face against that bank. The threshold itself is the open question: lowering it globally would start merging strangers, so the right fix is per-session clustering, not a constant.

The bubble's own text also had a rendering bug: the shell font has no glyph for curly quotes, so the "Say 'this is Alice'" hint printed as question marks. Fixed to straight quotes.

Two honest gaps. The open-mic path errored on the audio device tonight (PortAudio error -50), so naming from speech did not run live; the name was attached through the store instead. And the bubble sizing assumes the person is across the room: at arm's length a 0.30 m offset puts a full-size note across the wearer's own face, which is how the first staged shot looked.

Launcher, windows, and staging

Real Safari and Notes windows are captured and placed in the room now, not placeholder cards, and a note panel sits on the wall with the wearer's name. Two behaviours bit during staging and are worth stating because they are the next fixes: new panels pull in toward the nearest depth surface, which is correct for a wall and wrong when the nearest surface is your own face at 30 cm; and the pinch and fist gestures keep firing while you hold the phone, so the launcher opened Terminal and Finder and gather-panels re-fanned the layout mid-shot. A staging mode that freezes gestures is a five-line change and is on the list.

Hands

The Mac tracker held 13 to 16 fps tonight with 9 to 24 ms end to end and 0.43 detection with a hand at 0.5 m. Pinch, point, and fist are still on default thresholds: the calibration flow exists and is unit tested, but I did not get a clean calibration run on my hand this cycle, so I am not claiming calibrated gestures. The 36 mm per-joint jitter figure from checkpoint 1 stands as the last published number.

Placement eval

The eval set grew from 20 to 30 requests: five desk requests to be run with the desk in view, and five object-anchored requests ("next to the lamp") scored by 3D distance to the anchor, within 0.30 m counting as correct. The scorer and the runbook are in the repo with 12 tests. The 30-request run itself has not happened yet; the checkpoint-1 numbers (wall error 0.05 m with depth geometry, 0.65 surface accuracy) remain the published result.

Sponsored cloud

gcloud is authenticated on the hackathon project. The agent has an opt-in Vertex AI inference path with the API-key client as fallback, and a WiLoR offline batch runner plus a GCE L4 runbook for the hand-mesh comparison. Neither ran on the cloud this cycle: WiLoR's MANO weights are behind a registration gate, and the project is deleted at the final deadline, so nothing in the demo depends on it.

Hardware

The glasses camera, display path, and Pico controller from checkpoint 3 are unchanged in the repo this cycle; that work stayed on the bench. [Tarun: one or two sentences on what the Pico and display did this cycle, if anything, with any number you actually measured.]

Answering the feedback

Checkpoint 3 asked whether the bubble should show the summary automatically or wait until asked. Tonight's answer is automatic but short: the bubble carries the name and one line, the transcript stays in the store, and "who is this" pulls the rest on request. What made it feel wrong was not the summary, it was a wrong summary from a merged stranger's transcript, which is why the identity dedup mattered more than the wording.

The framing holds: Tempo is memory for AR and it is directional. "Where is my charger" and "who is this" run over the same map. This round showed the map has to be right before the memory is worth anything.

What's working on the rig now

Live iPhone stream at 3600 packets per second and 47 ms frame age; Mac hand tracking; face recognition with a persistent named identity and a bubble; real Safari and Notes panels placed in the room; note panels on real surfaces; the placement, hands, and people test suites (31 shell, 90 hands, 16 agent).

Still not done

Gesture calibration on my own hand. The 30-request placement run. Naming from speech on the live mic. Bubble sizing for near faces. A gesture freeze for staging. The Mac stays in the loop for every model and for the compositor.

Plan for the final

One uncut take: walk into the room, the bubble names me, ask where the charger is, place a note on the wall, open Safari with a gesture. Publish the eval JSON and the table in the README. Write up the Pico versus Mac split.

Feedback I'd like

- For the final, one long uncut take or short clips per capability?
- With the people layer running live, does it read as useful or invasive from the outside?
- Is per-session face clustering the right fix for the duplicate-identity problem, or should the threshold adapt to lighting?
