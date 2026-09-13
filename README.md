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

### What the agent knows about the room

| Signal | Where it comes from | What it enables |
|---|---|---|
| Head pose, gravity up, camera roll | ARKit on the phone | "in front", "left", "above", level panels |
| Surface under the gaze or the pointing finger | Ray march through the LiDAR depth map (`cast`) | notes that sit on the wall or lie on the desk with the surface's orientation |
| Floor height | Lowest horizontal depth cluster (`floor`) | "on the floor", height-aware placement |
| Objects with 3D positions | OWL-ViT open-vocabulary detector on the Mac GPU, box centre unprojected through LiDAR depth, tracked across frames, persisted (`agent/src/tempo/objects.py`) | "next to the whiteboard", "by the lamp" |
| Remembered places | Named 3D spots saved by the wearer (`memory.py`) | "remember this is my charger" / "where's my charger" answered relative to where they stand now |
| Hands | Mac-side RTMPose (rtmlib, ONNX) over the streamed frames, fused with LiDAR depth, injected into the compositor's gesture engine (`hands/`) | pinch to click, pinch-and-talk, "put it there" while pointing |
| Panels already in the room | compositor (`list-windows`) | move, close, gather, save and restore layouts |

### Talking to it

`uv run tempo live` is the daily driver: hold a pinch, speak, release. Speech goes through a local Whisper (mlx-whisper on the Mac GPU), the request plus the room snapshot goes to Gemini with the spatial tools, and the reply is spoken back. `tempo ask` is the typed equivalent.

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
uv run tempo people --frames "$TMPDIR/spatula-frames"   # faces + voices, a bubble beside each head
uv run tempo people me Tarun                # 6 s of your own voice so your speech is never filed under a guest
```

Without a phone: `SPATULA_MAC_HEADLESS=1 shell/build/mac-shell/mac-shell --replay shell/tests/recorded-sessions/initial.bin` replays a recorded session so `head-pose` and `list-planes` work (screenshot needs the windowed renderer).

## Prior work

`shell/` is carried over from our prior private project (Spatula / Vantage, Project Ithaca) as the compositor base; see `shell/PRIOR_ART.md`. Everything in `agent/`, this README, and every change listed in `shell/HACK_CHANGES.md` was built during the hackathon.

## Checkpoint log

| # | When (PT) | State | What changed | Where feedback helps |
|---|---|---|---|---|
| 1 | Sat Sep 12, midnight | Live end to end: spoken or typed request → Gemini → note or app panel placed on the real surface, 3 to 4 s per request. Objects, remembered places, layouts, pinch-and-talk, Mac-side hand tracking all running on the rig. Two eval runs below. | Compositor base ported (31 shell tests). New today: the agent and its tools, note panels, depth ray-cast + full pose placement, floor and roll, OWL-ViT object map with LiDAR depth, spatial memory, layouts, local Whisper pinch-and-talk, RTMPose hands fused with LiDAR (77 tests), the placement eval, the hand-tracking eval. | Hand tracking: the Mac path matches the phone on detection but not yet on jitter; is the LiDAR-anchored shape model the right next step, or should we push frames to a cloud GPU running a mesh model (WiLoR) and eat the round trip? |
| 2 | Sun Sep 13, noon | Fist opens the launcher, pinch clicks; launched windows now land in a free slot in front of the wearer, take focus, and re-launching recalls instead of duplicating. Phantom hands past arm's reach are gated out of the Mac tracker. People layer: faces and voices in the room, a bubble beside each head with name and what they last said, names taken from speech. | `hands` range/confidence gate; shell spawn placement with depth pull-in, focus on spawn, `recall_panel`; `tempo people` (YuNet + ArcFace, mlx-whisper + ECAPA, siyi lookup), people in the scene JSON. | Is a bubble beside the head the right surface for people context, or should it live in the wearer's periphery until asked? What context is fair to show next to a face? |

### Checkpoint 1 result, and why the geometry condition lost

| Condition | Surface accuracy | Median off-surface error | Mean latency |
|---|---|---|---|
| Image + geometry | 0.25 | 0.55 m | 3.7 s |
| Image only (model guesses metre offsets) | 0.45 | 0.36 m | 4.4 s |

Twenty requests each, one room, same model and prompt. The geometry condition lost, and the identical 0.55 m error on every "desk" request says why: the LiDAR *plane list* the compositor exposes is sparse and stale. At run time it held eight vertical planes and a ceiling; the bed the wearer was looking at had been classified vertical and the desk was absent, so the gaze ray hit no horizontal surface and the executor fell back to a fixed offset ahead of the head. The image-only model, guessing "about 0.7 m ahead and 0.3 m down", landed closer.

So the baseline is doing its job: it shows that geometry only helps when the geometry is actually there.

**Second run, same evening, after replacing the plane list with a ray march through the dense LiDAR depth map** (`cast` verb, plus a full `pose` so panels take the surface's orientation):

| Condition | Surface accuracy | Median off-surface error | Wall requests only, median error | Mean latency |
|---|---|---|---|---|
| Image + depth geometry | 0.65 | 0.29 m | **0.05 m** (the 5 cm lift, i.e. on the wall) | 3.9 s |
| Image only | 0.65 | 0.39 m | 0.32 m | 4.8 s |

Every wall request now lands on the wall to within the deliberate 5 cm standoff. The remaining misses are all "desk" requests made while no horizontal surface was in the camera's view (the wearer was facing a wall and a bed), so both conditions put the note on the nearest wall; the scorer counts that as wrong for both. That's the honest limit of the setup, not of the method: the agent can only place on what the sensor sees. Next: the object detector so "next to the lamp" resolves, and a run with the desk in view.
