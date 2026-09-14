Tempo, checkpoint 3 (text of the Google Doc to be submitted Sun Sep 13 evening; scores pending)

Added at the top, above the checkpoint-2 body:

What changed since checkpoint 2

The new capability on camera this round is people. The room now remembers who is in it. Faces on the Mac frames are recognized with YuNet plus ArcFace and unprojected through the LiDAR map into the scene, so each visible head gets a floating bubble 0.30 m to the wearer's right with the person's name and a one-line summary of what they last said. Voices are tracked in parallel: an open mic runs an energy VAD into mlx-whisper into an ECAPA voiceprint, so "I'm Alice" names the speaker's face and voice, and "this is Bob" names the largest face in view. After three lines from a person, Gemini writes a single sentence into the bubble. People are now part of the scene JSON the agent sees, so "who is this" and "who is in the room" are answered over the same map as "where is my charger". The same memory store holds places, objects, and now people.

The launched-window bugs that made the fist launcher feel broken are fixed. Every new window used to spawn on the exact spot of the last one and never take focus, so the fourth launch sat on top of the first and looked like nothing happened. Windows now take the nearest free slot in front of the wearer, pull in ahead of whatever the depth map says is there, take focus on spawn, and relaunching an app that is already open recalls its panel instead of opening a duplicate stream.

On the hands: a range and confidence gate on the Mac tracker drops hand-shaped detections past 1 m and below 0.5 confidence. A phantom hand 1.5 m across the room had been outranking the phone's own hands and hijacking the pointer; that is gone.

Answering the feedback

The check-in-2 feedback was that the differentiator is that Tempo is directional — it points you to where something is — plus persistent spatial memory. People are the half of memory that was missing: a room that remembers things but not faces is forgetting the part that matters most. "Who is this, and what did we talk about" is now the same query as "where is my charger", over the same map, and the bubble is the same surface as a note on the wall.

On the check-in-1 question of why Apple or Meta do not build this natively: they still won't, for the same reason. Their OS is the window manager and the agent is a feature wrapped around it. Tempo is the inverse — the agent is the OS and windows, notes, and people bubbles are all outputs of it. People memory is the clearest version of that: the bubble is not an app you open, it is something the room does when someone walks in.

Use cases, updated:
- "Where is my charger" (and my keys, and the thing I just put down): spatial memory of objects.
- "Who is this, and what did we talk about": people memory, the bubbles beside each head.
- "Put a note on the wall behind the monitor": information placed on real surfaces.
- "Put my work where I am": walk to the desk and the panels you left there come back.

Honest limits, and what is next not done

The people layer is verified offline, not yet on the live rig. Three faces were recognized on a real exported frame; the voice path was tested on TTS audio, with ECAPA scoring 0.89 same-speaker and 0.0 cross-speaker, and the Gemini one-line summary works on three lines of dialogue. It has not yet been run with a live face and a live voice in front of the camera and mic, so the end-to-end bubble-on-a-real-person demo is the first thing on camera once the rig is back up.

Gestures are still on default thresholds. The `hands calibrate` flow is written and unit-tested, but it has never been run on my own hand, so `gestures.toml` does not exist yet and pinch, point, and fist still fire on the out-of-the-box cutoffs. That is the cheapest remaining fix for the "gestures barely work" complaint and the second thing on camera.

The 60-second video for this check-in is TBD; I record it once the rig is live and the two items above are shot.

Next 12 hours

1. Bring the rig up: reopen SpatialBridge on the iPhone, restart the shell, restart the hands/people/objects daemons against a live export dir. Verify packet_rate is above zero before anything else.
2. Run the people layer on a live face: point the phone at a face, say "I'm Tarun" and two sentences, verify the bubble appears, the name flips, and the summary lands. That plus the fist-to-launch hand clip is the check-in-3 video.
3. Calibrate gestures on my own hand: `hands calibrate --write`, restart the shell, record per-phase jitter in mm, publish the numbers.
4. Repeat the placement test with the desk in view and add object-anchored requests such as "next to the whiteboard"; update the README results table.
5. Start the Pico bring-up: pose and depth streaming from a microcontroller-class board, to find out what the phone is doing that a $4 chip cannot.

Feedback I'd like

- For the people bubbles, is a one-line Gemini summary the right density, or should the bubble stay name-only until the wearer asks?
- The people layer is verified offline but not yet live on the rig. Is "verified offline, live demo next" the right way to frame that to judges, or does it read as unfinished?
- For placement quality, is raw distance from the target enough, or should I also compare against human judgments of where a panel feels natural?
