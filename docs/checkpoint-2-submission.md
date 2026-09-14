Tempo, checkpoint 2 (text of the Google Doc submitted Sun Sep 13; scored 7/6/7/7)

Added at the top, above the checkpoint-1 body:

What changed since checkpoint 1

The judge's note that this should be positioned as memory for AR is the framing I'm taking. Everything Tempo does is remembering where things and people are, and handing that back at the moment it matters.

Most of this cycle went into the interaction layer, the part that makes the room feel conversational instead of command-driven. Pinch-and-talk is live: you pinch, speak, and the agent answers with an action in the room. The hand tracking behind it moved onto the Mac so it can be measured and tuned, and this morning I added a gate that drops hand-shaped detections beyond arm's reach, which had been hijacking the pointer.

People are the next thing the memory covers. I'm building floating bubbles that sit next to a person's head with their name and context: who they are, what we last talked about, what they asked me to do. Face recognition feeds the same memory store that already holds places and objects, so "where is my charger" and "who is this and what did we discuss" are the same query over the same map.

On the hands: the fist launcher and pinch click now open and select real Mac windows in the room. I found and fixed why launched windows seemed to vanish: every new window spawned on the exact spot of the last one and never took focus. Windows now take the nearest free slot in front of you, pull in ahead of whatever the depth map says is there, and relaunching an open app recalls its window instead of duplicating it.

Hardware direction: I want the sensor side of this OS to run on a microcontroller-class board (Raspberry Pi Pico) so the glasses don't depend on a phone. The compositor and agent stay on the Mac or a pocket unit; the bridge that streams pose, depth, and frames is the part that moves down.

Answering the feedback

Why won't Apple or Meta build this natively? They are building an app grid with an assistant wrapped around it. Their OS is the window manager and the agent is a feature. Tempo is the inverse: the agent is the OS and windows are one of its outputs. That is a different product, not a feature they can ship in an update.

Use cases I'm building the consumer hook around:
- "Where is my charger" (and my keys, and the thing I just put down): spatial memory of objects.
- "Who is this, and what did we talk about": people memory, the bubbles above.
- "Put a note on the wall behind the monitor": information placed on real surfaces.
- "Put my work where I am": walk to the desk and the panels you left there come back.

Next 12 hours

1. Ship the people layer. Face recognition on the Mac frames, a bubble next to each recognized head with their name and the last thing we talked about, backed by the same memory store as places.
2. Calibrate gestures on my own hand. Measure fingertip jitter after the palm-anchor fix, then set pinch, point, and fist thresholds from live samples and publish the numbers.
3. Install the window fixes on the rig and record fist-to-launch and pinch-to-click on video.
4. Repeat the placement test with the desk in view and add object-anchored requests such as "next to the whiteboard."
5. Start the Pico bring-up: pose and depth streaming from a microcontroller-class board, to find out what the phone is doing that a $4 chip cannot.

Feedback I'd like

- For placement quality, is raw distance from the target enough, or should I also compare against human judgments of where a panel feels natural?
- Is "memory for AR" the right one-line pitch, or does it read as a feature rather than a product?
- For the people bubbles, what context is worth showing next to a face without it feeling invasive?
