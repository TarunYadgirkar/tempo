# Checkpoint 1 video (60 s)

Record with QuickTime screen recording, mic on, Spatula window on the left half and a terminal on the right. Phone pointed at the desk and the wall behind it.

**0 to 12 s, the problem.** Say: "Spatial computers put an app grid in front of your face and treat the room as scenery. Tempo asks what changes when the room is the interface. This is my glasses prototype: iPhone as the sensor, Mac as the compute, and a Gemini agent that sees what I see and knows where the surfaces are."

**12 to 45 s, the demo.** Run two commands, let each land on screen:

```
uv run tempo --speak ask put a note on the desk that says finish the eval harness
uv run tempo --speak ask open safari on the wall in front of me
```

While they run, say: "The agent gets the same frame I see plus the scene graph: head pose, planes classified as table, wall, floor with distances, and the panels already in the room. It answers with spatial actions, not text. That note landed on the actual desk plane, the ray from my head hit the detected surface at 0.7 metres."

**45 to 60 s, the question and the next 12 hours.** Say: "The precise question: how much does geometry buy an embodied agent over pixels alone? Baseline is the same model with the image only. Next 12 hours: real note panels instead of test cards, a pointing ray from the hand so 'that one' works, and the first placement-error numbers on a fixed command set."
