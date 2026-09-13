# Checkpoint 1 video (60 s)

Record with QuickTime screen recording, mic on. Spatula window on the left two thirds, a terminal on the right. Phone pointed at the wall behind the bed, then pan down to the bed for the third command.

Terminal, before recording:

```
cd "/Users/tarunyadgirkar/TarunsCode/east west hackathon/tempo/agent"
```

**0 to 10 s, the problem.** Say: "Spatial computers put an app grid in front of your face and treat the room as scenery. Tempo makes the room the interface. iPhone is the sensor, Mac is the compute, and a Gemini agent sees what I see and knows where every surface is."

**10 to 45 s, three commands.** Run each, let it land, keep talking over the latency.

```
uv run tempo --speak ask pin a note on the wall in front of me that says checkpoint one shipped
```
Say: "The agent gets my frame plus the scene: head pose, a LiDAR depth ray cast, the panels already in the room. It answers with actions, not text. That note is on the wall to five centimetres, taking the wall's orientation."

```
uv run tempo --speak ask open safari on the wall and the terminal to my left
```
Say: "Real Mac windows, placed where I asked."

```
uv run tempo --speak ask remember that this spot is where my charger lives
```
then turn the phone away and run
```
uv run tempo --speak ask where is my charger
```
Say: "It remembers places in the room and answers relative to where I'm standing now."

**45 to 60 s, the question and the next 12 hours.** Say: "The precise question: how much does geometry buy an embodied agent over pixels alone? Same model, same prompt, twenty requests. With depth geometry, wall placements land within five centimetres. Image only, thirty. The failures are honest: no desk in view, no desk placement. Next twelve hours: hand tracking moved to the Mac with MediaPipe fused with LiDAR, an open-vocabulary object detector so 'next to the lamp' resolves, and pinch-to-talk."
