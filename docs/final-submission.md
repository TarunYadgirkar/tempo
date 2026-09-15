Tempo, final submission (text of the Google Doc in the Final folder, Mon Sep 14 night)

Team: Tarun Yadgirkar (UC Berkeley), solo
Track: Deep Tech / Physical AI (Freestyle)
Repo: https://github.com/TarunYadgirkar/tempo
Video: attached in the submission folder

What Tempo is

Tempo is a room agent for AR. An iPhone on the head streams ARKit pose, LiDAR depth, and camera to a Mac. The Mac runs a spatial compositor that renders real Mac windows in 3D, and a Python agent that sends Gemini the live view plus an explicit scene graph: head pose, depth casts, detected objects, remembered places, people, hand aim, and open panels. Gemini answers with actions, and the agent executes them in the room: put a note on that wall, open Notes next to the monitor, tell me where my charger is, tell me who this is.

The one-line pitch the judges gave me at checkpoint 2, and which I kept: Tempo is memory for AR, and it is directional. It does not just remember that a thing exists. It points back to where the thing is, places information next to it, and brings the context back when the object or the person shows up again.

Outside the hackathon this is the agent layer of Vantage, the AR glasses and spatial OS I am building with my co-founder (Project Ithaca; we recently interviewed with A16Z). The compositor base was carried over from that project and is marked as prior art in the repo; everything in the agent, the hands tracker, the people layer, the eval, and every compositor change listed in shell/HACK_CHANGES.md was built in these 72 hours.

What was built, in order

Hours 0 to 12. Compositor ported (31 tests). The agent and its tool set: note panels, depth ray-cast placement with full surface pose, floor and roll, an OWL-ViT object map with LiDAR depth, spatial memory of places, saved layouts, local Whisper pinch-and-talk, and Mac-side hand tracking with RTMPose fused with LiDAR. A placement eval and a hand-tracking eval.

Hours 12 to 24. Fist opens the launcher, pinch clicks. Launched windows land in a free slot, take focus, and re-launching recalls instead of duplicating. Phantom hands past arm's reach gated out. The people layer: YuNet and ArcFace faces, ECAPA voiceprints, an open mic through mlx-whisper, a bubble beside each head, names learned from "I'm Alice".

Hours 24 to 36. Live people flow completed: speech gate so room noise does not become transcript, one stable identity across nearby detections, bubble recovery when the shell retires its window, Gemini one-line summaries. Real app entries in the launcher (Safari, Terminal, Finder, Notes) and a close-all verb. Hardware: a better camera mounted on the glasses, a display path, and a Raspberry Pi Pico as onboard controller, all in bring-up.

Hours 36 to 72. The eval grew to 30 requests with object-anchored scoring. An opt-in Vertex AI inference path and a WiLoR batch runner for the sponsored cloud project. The people layer ran on the live rig; the duplicate-identity problem was found and fixed in the store; real Safari and Notes panels were placed in the room.

What the numbers say

Placement, 20 requests, one room, same model and prompt (checkpoint 1):

| Condition | Surface accuracy | Median off-surface error | Wall requests only | Mean latency |
|---|---|---|---|---|
| Image + depth geometry | 0.65 | 0.29 m | 0.05 m | 3.9 s |
| Image only | 0.65 | 0.39 m | 0.32 m | 4.8 s |

Every wall request lands on the wall within the deliberate 5 cm standoff once the sparse plane list was replaced with a ray march through the dense LiDAR depth. The remaining misses are desk requests made with no horizontal surface in view. Geometry only helps when the geometry is there, which is the honest limit of a single head-mounted sensor.

Hands: the Mac tracker matches the phone's own tracking on detection (0.97) and ran at 13 to 16 fps with 9 to 24 ms end to end on the final night. Jitter was 36 mm per joint before the One-Euro retune; the retune is in the build and the after number is not yet published.

People: 32 person records had accumulated for one face because ArcFace cross-frame similarity in this room sits between 0.30 and 0.57 against a 0.42 threshold. Merged into one persistent identity; the fix that generalises is per-session clustering rather than a constant.

Rig: 3600 packets per second from the phone, 47 ms frame age, 30 Hz raw frame export feeding the hands, objects, and people daemons.

Tests: 31 compositor, 90 hands, 16 agent.

Why the big companies won't build this natively

Their OS is the window manager and the agent is a feature wrapped around it. Tempo is the inverse: the agent is the OS, and windows, notes, and people bubbles are all outputs of one room model. People memory is the clearest case. The bubble is not an app you open. It is something the room does when someone walks in.

Use cases

- "Where is my charger" (and keys, and the thing I just put down): spatial memory of objects that points.
- "Who is this, and what did we talk about": people memory beside each head.
- "Put a note on the wall behind the monitor": information on real surfaces.
- "Open Notes here": apps in physical slots, not a flat grid.
- "Put my work where I am": walking back to a desk recalls the panels left there.

What is not finished

Gestures are on default thresholds; the calibration flow exists but was not run cleanly on my own hand. The 30-request placement run has not been executed. Naming from speech did not run on the live mic on the final night because of an audio device error. Bubble sizing assumes a person across the room and fails at arm's length. New panels pull toward the nearest depth surface, which is wrong when that surface is your face. The Mac is still in the loop for every model and for rendering; the Pico and display path are bring-up, not a standalone wearable. The WiLoR cloud comparison did not run: the model weights are behind a registration gate and the sponsored project is deleted at the deadline.

What the sponsored credits were used for

Gemini through AI Studio for every agent request all weekend. gcloud authenticated on the hackathon project with an opt-in Vertex path in the agent and a GCE runbook for the hand-mesh batch, neither on the demo's critical path by design.

What comes next

Per-session face clustering. A staging mode that freezes gestures. Calibration and the published jitter number. The desk-in-view and object-anchored eval run. Moving sensor coordination onto the Pico and measuring what the head-mounted board can own.
