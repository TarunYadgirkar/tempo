Tempo, checkpoint 1
Team Tempo (Tarun Yadgirkar, UC Berkeley), Deep Tech / Physical AI track, freestyle

Repo: https://github.com/TarunYadgirkar/tempo
Video: attached in this folder

What we are building

Every computer people use today was designed before AI. Phones, laptops, and even the current headsets take an app grid and put it somewhere, then try to wrap an assistant around it. For the Deep Tech track we are building the opposite: a pair of spatial glasses and the operating system for them, designed from the start around an agent that lives in the room with you. The long-term product is Vantage: lightweight AR glasses and the spatial OS together, meant to replace the phone and the laptop for daily work, with compute split between the glasses, a pocket unit, and the cloud.

Tempo is the piece of that we can prove in 72 hours. An iPhone strapped in front of a Mac stands in for the glasses: the phone streams head pose, LiDAR depth, the camera, and detected surfaces; the Mac renders real Mac application windows as panels in the room and runs the agent. The agent sees what the wearer sees, knows where the walls, desk, floor, and objects are, and answers spoken requests with spatial actions instead of text. "Put a note on the wall behind the monitor" produces a note on that wall, facing out from it, five centimetres off the surface. "Remember this is where my charger lives" saves a spot in the room, and "where is my charger" is answered relative to where the wearer is standing now.

Why this matters

The room is the only interface that does not have to be learned. If an agent can place information where things are, the glasses stop being a screen you look at and become a way to use the room you are already in. That is the wedge for Vantage: an AI-native device that is useful before it is a general computer, starting with the workflows where knowing where things are is the whole job.

The question we are measuring

How much does explicit room geometry buy an embodied agent over pixels alone? The physical operation is placing information at a location. The baseline is the same model, same prompt, same tools, given only the image, so it has to guess distances and surfaces. The treatment adds the scene graph the glasses already have: head pose, a LiDAR depth ray cast, detected objects with 3D positions, and the hand's pointing ray. We score placement error in metres against the real surface, surface class accuracy, and latency, on a fixed set of twenty spoken requests in one room.

01 Prove the agent  →  02 Ground it in the room  →  03 Make the hands work  →  04 Put it on glasses
Gemini agent with spatial tools, live on the rig  →  LiDAR depth casts, object map, spatial memory  →  Mac-side hand tracking fused with depth, calibrated gestures  →  Glasses hardware sourced, same OS, same agent

What we have done in the first cycle

The whole loop is live on real hardware. Spoken or typed request, local Whisper transcription, Gemini with nine spatial tools, actions executed against the compositor, result visible in the room in three to four seconds. New this cycle: note panels with a real layout, a depth ray cast that hits whatever surface the wearer is looking at, full six degree of freedom panel poses so notes take the surface's orientation, floor height and camera roll, an open-vocabulary object detector (OWL-ViT) running on the Mac GPU whose detections are unprojected through LiDAR depth into a persistent 3D object map, named places the wearer can save and recall, saved layouts per room, pinch-and-talk, and hand tracking moved off the phone onto the Mac (RTMPose over the streamed frames, fingertip depth from LiDAR) so every gesture in the system can be measured and tuned.

Two eval runs so far. In the first, geometry lost to pixels-only (0.25 vs 0.45 surface accuracy) because ARKit's plane list was sparse and stale, so the agent fell back to fixed offsets. That was the right failure to find on day one. After replacing the plane list with a dense depth ray march, every wall request lands within five centimetres of the wall; the image-only baseline sits at thirty. The remaining misses are desk requests made while no desk was in view, which both conditions get wrong, and which we report as the limit of what the sensor can see rather than hide.

Hand tracking is the honest weak point. The Mac path now matches the phone on detection (97 percent of frames) but jitter was 36 mm per joint, which makes any pinch threshold unusable. The cause was per-fingertip depth sampling on a 256 by 192 LiDAR map. The fix, one robust range per hand from the palm plus a hand shape model, is in the repo and waiting on a measured run.

Next 12 hours

Hardware. We start sourcing the glasses: a see-through display module with a forward camera and depth, or the closest developer-kit equivalent we can get shipped, plus the mount that turns the current phone rig into something wearable for the Monday demo. The software is built so the phone is just one sensor; the same compositor and agent run unchanged when the sensor is a pair of glasses.

Hands. Measure the jitter fix against the phone's own tracking, calibrate pinch, point, and fist thresholds on the wearer's actual hand with the gesture engine's calibration tool, and run the strongest open-source 3D hand model (WiLoR) on a cloud GPU from the sponsored credits to see whether a 120 ms round trip beats local tracking for the pointing path.

Placement. Rerun the twenty-request eval with the desk in view, add object-anchored requests ("next to the whiteboard") now that the object map is live, and publish the numbers in the README.

Where feedback would help

Whether surface accuracy and metre error are the right primary metrics, or whether we should score against human-placed targets. And whether the cloud-versus-local split for hand tracking is worth the latency for an agent that mostly needs "where is the hand pointing" rather than a fast click.
