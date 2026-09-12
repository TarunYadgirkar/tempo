# tests/sessions/

Workflow-level regression tests. Each session is a directory containing:

- `session.yaml` — the executable specification (input + expected output).
- `transcript.jsonl` — captured actual output during the last replay
  (gitignored).
- `input/` — optional, any large input fixtures (e.g., `.bin` UDP recordings
  to replay).

## What is a "session"?

A session is an **executable specification** of a user-facing behavior. It
has two parts:

1. **Input stimulus.** Either:
   - A recorded UDP `.bin` to replay (existing infrastructure in
     `bridge-receiver/tests/recorded/`).
   - A scripted sequence of synthesized packets (poses, hands, frames,
     planes) authored by hand.
   - User-action events (e.g., "fist gesture begins at t=5s, ends at t=7s").

2. **Expected output transcript.** A JSONL list of events the system should
   emit, with tolerance bands. Categories:
   - Gesture events fired (action, type, timestamp ± 33 ms).
   - Window mappings and positions (window name, world coords ± 5 mm).
   - Marker positions per frame (joint index, world coords ± 5 mm).
   - Click deliveries (surface, sx/sy ± 2 px, button).
   - Visibility flags (e.g., `view_map: Launcher arc` must appear within
     500 ms of a fist BEGIN).

## How is this different from "recording"?

A session can be **authored by hand for unbuilt or broken features.** This is
the answer to "how do you replay something that doesn't work yet?"

### Case 1 — Feature not built yet (e.g., virtual keyboard)

Write `session.yaml` with the *desired* input and *desired* output. The
session will fail until the feature is implemented. Implementing the feature
turns the session green. The session is the spec.

This is TDD at the workflow level. Sessions written before implementation
serve as acceptance criteria.

### Case 2 — Feature currently broken (e.g., pinch gesture misclassifies)

Author TWO sessions:

- `pinch-current-buggy.session/` — input: known scenario. Expected output:
  what the system actually does today. Tagged `expected-fail: true` so CI
  doesn't fail on it but it serves as a frozen reproduction.
- `pinch-correct.session/` — input: same scenario. Expected output: what the
  system *should* do (hand-authored by you, not recorded). Tagged
  `expected-pass: true`. Fails today. Fixing the bug turns it green.

Once the fix lands, the `-current-buggy` session is retired (or kept as
"this used to be broken" archive). The `-correct` session becomes the
regression test.

### Case 3 — Feature works and you want to keep it that way

Record a real session via `scripts/record-session.sh`. The expected output
transcript is auto-captured. Save as
`tests/sessions/<descriptive-name>/session.yaml`.

## session.yaml schema

```yaml
name: launcher-fist-opens-radial
description: |
  Fist gesture causes the launcher's radial arc to map within 500ms.
  Regression for the stale /tmp/spatial-launcher-visible bug
  (see plans/2026-05-26-launcher-stale-flag/ for the full story).

tags:
  - expected-pass: true     # this session SHOULD pass on a healthy main
  - subsystem: launcher
  - subsystem: gesture-engine
  - feature: radial-launcher

input:
  type: recorded-bin
  path: input/launcher-fist.bin
  # alternatively:
  # type: synthetic
  # script: |
  #   wait 1s
  #   pose pos=(0,0,0) rot=(0,0,0,1) at_ts=0
  #   hand_packet hand=1 wrist=(0.1,-0.1,-0.3) ... at_ts=1s
  #   plane uuid=aaaaaa... center=(x,y,z) normal=(x,y,z) extent=(w,h) at_ts=50ms
  #   setup_env KEY=VALUE                            # shell env inherited by spawn_client
  #   spawn_client name=test_panel at_ts=200ms       # uses currently-set env
  #   ...

setup:
  # commands to run before replay (e.g., wipe /tmp state)
  - rm -f /tmp/spatial-launcher-visible

expectations:
  - kind: gesture-event
    action: TOGGLE_LAUNCHER
    type: BEGIN
    timestamp_after_t0: 1s
    tolerance: 200ms
  - kind: view-mapped
    name: Launcher arc
    timestamp_after_t0: 1.5s
    tolerance: 500ms
  - kind: world-position
    object: Launcher arc
    pos: [0.10, -0.05, -0.55]
    tolerance: 0.05  # meters
  - kind: anchored-window-screen-position
    # screen-space probe for anchored panels; expected_sx_sy supports
    # the literal pair `[sx, sy]` or the shorthand `from_view_at_pose`
    # — the latter asks the harness to compute the canonical projection
    # of the named panel's anchor through the view matrix at this ts.
    name: test_panel
    at_ts: 750ms
    expected_sx_sy: from_view_at_pose
    tolerance_px: 5
  # ... etc

teardown:
  - pkill -9 wxrd || true
  - pkill -9 monado-service || true
```

## Replay harness (not yet built — see BACKLOG)

The replay harness:
1. Runs `setup`.
2. Starts wxrd in `WXRD_HEADLESS=1` mode against the bin (or synthetic
   script) feeding port 9898.
3. Captures every relevant event into `transcript.jsonl`.
4. Diffs the transcript against `expectations:` with tolerance bands.
5. Runs `teardown`.
6. Reports pass/fail per expectation.

The first sessions to author by hand (before the harness exists) should be:

1. `launcher-fist-opens-radial` — guards the recent stale-flag fix.
2. `pinch-misclassify-current` (expected-fail) + `pinch-correct`
   (expected-pass) — the misclassification bug you reported.
3. `markers-stay-aligned-during-rotation` — guards the drift fix.
4. `passthrough-no-black-strip` — guards the principal-point offset fix.

Each of these is a frozen statement of "this behavior must hold."
