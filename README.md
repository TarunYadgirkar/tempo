# Tempo

East v. West 72 Hour Hackathon, Sep 12 to 14 2026. Team Tempo: Tarun Yadgirkar (UC Berkeley), solo. Track: Deep Tech / Physical AI. Freestyle entry.

## The question

Spatial computers today are a phone-style app grid floating in front of your face. The room is scenery. Tempo asks what changes when the room itself is the interface: when an agent sees what you see, knows where the table, the walls, and your open windows are, and can put information *into the room* instead of talking at you.

The precise version: **how much does explicit room geometry buy an embodied agent over pixels alone, for the physical operation of placing information at a location?**

- **Operation.** Given a spoken request and the wearer's view, put a panel at the right physical spot (the desk, the wall behind the monitor, next to the thing being pointed at).
- **Baseline.** The same model, same prompt, same tools, but with the image only. It has to guess distances and surfaces from pixels.
- **Treatment.** Image plus the scene graph the compositor already has: head pose, LiDAR planes classified as table / wall / floor with distances, existing panels, and (from checkpoint 2) the hand pointing ray.
- **Measured.** On a fixed set of 20 requests in one room: placement error in metres from the intended surface (ground truth from the detected planes), surface-class accuracy (landed on a table when asked for the table), end-to-end latency from request to panel visible, and cost per request. Reported per checkpoint as the set grows.
- **Failure case we expect.** Requests that reference an object the planes don't describe ("next to the lamp"). Geometry gives no surface; the agent has to fall back to the image. We will report how often that fallback lands.

## What it is

An AR shell for glasses, running today on an iPhone strapped in front of a Mac.

- **iPhone** streams ARKit head pose, LiDAR depth, detected planes, camera frames, and 21-joint hand skeletons over UDP.
- **Mac compositor** (`shell/`) renders passthrough plus real Mac app windows as panels in 3D, with pinch-to-click, a spatial keyboard, and plane anchoring. Everything is driven through a line-protocol control socket.
- **Room agent** (`agent/`) is new this weekend. It takes a snapshot of what the wearer sees, the scene geometry (head pose, surfaces classified as table / wall / floor with distances, existing panels), and a spoken or typed request, sends them to Gemini with a small set of spatial tools, and executes the returned actions against the compositor: place a note near the thing you pointed at, open an app where you're looking, move a panel onto the table, gather lost windows.

The agent never gets pixels-only or geometry-only. It gets both, which is what lets it answer "put that on the wall behind the monitor" with a world coordinate.

## Architecture

```
iPhone (SpatialBridge)  --UDP 9898-->  mac-shell (Metal, ScreenCaptureKit)
                                           |  control socket ($TMPDIR/spatial-os.sock)
                                           |  screenshot · head-pose · list-planes · list-windows
                                           |  launch · launch-app · move · anchor · note · aim · layout
                                           v
                                     agent (Python)  <--tools-->  Gemini
```

## Setup

Compositor (macOS, Apple silicon, Xcode CLT, cmake + ninja):

```bash
cd shell
./scripts/build-mac.sh
./scripts/test-mac.sh
./scripts/run-mac.sh          # installs /Applications/Spatula.app and launches it
```

Grant Screen Recording once. Open SpatialBridge on the iPhone; it finds the Mac over Bonjour.

Agent:

```bash
cd agent
uv sync
echo 'GEMINI_API_KEY=...' > .env
uv run tempo scene                          # what the agent sees right now
uv run tempo ask put a note on the desk that says buy milk
uv run tempo --speak listen                 # push-to-talk loop
```

Without a phone: `SPATULA_MAC_HEADLESS=1 shell/build/mac-shell/mac-shell --replay shell/tests/recorded-sessions/initial.bin` replays a recorded session so `head-pose` and `list-planes` work (screenshot needs the windowed renderer).

## Prior work

`shell/` is carried over from our prior private project (Spatula / Vantage, Project Ithaca) as the compositor base; see `shell/PRIOR_ART.md`. Everything in `agent/`, this README, and every change listed in `shell/HACK_CHANGES.md` was built during the hackathon.

## Checkpoint log

| # | When (PT) | State | What changed | Where feedback helps |
|---|---|---|---|---|
| 1 | Sat Sep 12, 7 PM | Live end to end: spoken or typed request → Gemini → note or app panel placed in the room, 3 to 4 s per request. First eval run below. | Compositor base ported and building (27 tests). New this weekend: the agent, note panels, hand aim ray, layout save/load, gaze-ray surface placement, the 20-request eval in two conditions. | Is surface accuracy the right primary metric, or should we score against a human-placed target? |

### Checkpoint 1 result, and why the geometry condition lost

| Condition | Surface accuracy | Median off-surface error | Mean latency |
|---|---|---|---|
| Image + geometry | 0.25 | 0.55 m | 3.7 s |
| Image only (model guesses metre offsets) | 0.45 | 0.36 m | 4.4 s |

Twenty requests each, one room, same model and prompt. The geometry condition lost, and the identical 0.55 m error on every "desk" request says why: the LiDAR *plane list* the compositor exposes is sparse and stale. At run time it held eight vertical planes and a ceiling; the bed the wearer was looking at had been classified vertical and the desk was absent, so the gaze ray hit no horizontal surface and the executor fell back to a fixed offset ahead of the head. The image-only model, guessing "about 0.7 m ahead and 0.3 m down", landed closer.

So the baseline is doing its job: it shows that geometry only helps when the geometry is actually there. Next 12 hours: replace the plane-list ray cast with a ray march through the dense LiDAR depth map (the compositor already has it for occlusion), which gives a hit on any surface in view regardless of ARKit's plane detection, then rerun both conditions.
