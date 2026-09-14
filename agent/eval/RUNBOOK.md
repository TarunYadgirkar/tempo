# Placement eval runbook

How to run the fixed placement eval against a **live rig** (iPhone + Mac compositor + agent). The eval is defined in `agent/src/tempo/eval.py` and the request set lives in `agent/eval/requests.json`. Results land in `agent/eval/results/<condition>-<unix-ts>.json` and the headline numbers go into the README results table.

This doc is for the operator (Tarun). The eval is **not** run headless: it places real panels in the compositor from spoken/typed-equivalent requests, so the room, the surfaces in view, and the tracked objects all matter.

## What the eval measures

For each request the agent places one panel; the scorer (`eval.py`) records:

- `landed_kind` — the plane kind (table / wall / floor) the placed panel sits over, from a ray march through the live plane list.
- `surface_error_m` — distance from the placed panel to the nearest plane it sits over.
- `correct_surface` — true when `landed_kind` matches the request's `expect` (see schema below).
- `latency_s` — request to panel visible.
- `actions` — the executor log (e.g. `note #5 'buy milk' -> on_table` or `note #5 'bedtime' -> near_object (lamp)`).

`summarize()` rolls a run up into surface accuracy, median off-surface error, and mean latency.

## Request schema

`agent/eval/requests.json` is a flat array. Each entry:

```json
{"id": 1, "text": "put a note on the desk that says buy milk", "expect": "table"}
```

- `id` — stable int, used in result rows.
- `text` — the request fed verbatim to the agent.
- `expect` — the ground-truth surface the panel should land on. The scorer (`_expected_kind`) understands:
  - `table` — must land on a horizontal surface classified `table`.
  - `wall` — must land on a vertical surface classified `wall`.
  - `gaze` — must land on whatever surface the gaze ray hits (table / wall / floor).
  - `front` — any panel placed counts as correct (no surface claim).
  - `object` — **new.** Object-anchored request; the panel should land beside the named object. Carries an extra `anchor` field (see below). The current scorer counts `object` as "a panel was placed" (pass) because measuring the distance from the placed panel to the named anchor's 3D position is a scorer extension, not yet wired in — see "Scoring object-anchored requests" below.
- Optional `group` — `desk-in-view` or `object-anchored`, so a run can be sliced by group in the README.
- Optional `anchor` — for `expect: "object"` entries, the label the agent should resolve via the object map (`lamp`, `whiteboard`, `plant`, `monitor`, …) or fall back to the image for.
- Optional `note` — operator-facing hint, not read by the scorer.

### The two new groups (ids 21–30)

- **desk-in-view** (21–25, `expect: "table"`): the same "desk" / "table" requests as the legacy set, but meant to be run while a horizontal surface is **actually in the camera view**. The README's check-in-1/2 misses were all desk requests made while the wearer faced a wall and a bed — both conditions put the note on the nearest wall and the scorer counted it wrong for both. This group isolates the case the geometry condition is supposed to win: desk in view, LiDAR depth ray-march finds the horizontal, panel lies on it.
- **object-anchored** (26–30, `expect: "object"`, `anchor` set): "next to the lamp", "by the whiteboard", "next to my charger", "by the plant", "next to the monitor". Tests the OWL-ViT object map + `near_object` placement path the README calls the honest limit. `charger` is deliberately **not** in the OWL-ViT default vocabulary (`agent/src/tempo/objects.py DEFAULT_PROMPTS`), so request 28 exercises the pure image / remembered-place fallback — the agent has no object-map hit and must guess from pixels or a saved `remember_place charger`.

## Before you run: bring the rig up

The eval reads live state from the compositor control socket (`$TMPDIR/spatial-os.sock`) and the frame export dir. If the shell is down or the frame is stale, every trial lands on a stale snapshot and the numbers are garbage. From `AGENTS.md` the rig has been down since ~4:45 PM PT; bring it up fresh.

1. **iPhone.** Open the **SpatialBridge** app on the iPhone. It advertises over Bonjour; tap the Mac when it appears. The Mac IP + UDP port (9898) are printed by `run-mac.sh`.
2. **Mac compositor.** From the repo root:
   ```bash
   shell/scripts/run-mac.sh --restart
   ```
   `--restart` quits any stale Spatula first. Grant Screen Recording in System Settings → Privacy & Security if prompted (once, for `/Applications/Spatula.app`).
3. **Verify the iPhone link is live.**
   ```bash
   shell/scripts/run-mac.sh --check
   ```
   Look for `iPhone link: LIVE (<n> packets/s)`. `packet_rate > 0` is the gate — do not proceed if it is 0; the phone is not streaming (check same WiFi, restart SpatialBridge).
4. **Turn on frame export** so the agent's object detector and the people layer can read frames. The shell writes to a per-user dir; the convention across the repo is `$TMPDIR/spatula-frames`. The `hands track --enable-export` flag turns export on for its `--dir` before tracking; if you are not running hands, enable it from the control socket:
   ```bash
   # from the agent dir, one-shot:
   uv run python -c "from tempo.shell import Shell; Shell().send('frame-export on ' + __import__('os').environ['TMPDIR'] + '/spatula-frames')"
   ```
   Confirm `$TMPDIR/spatula-frames/latest.json` is being written (mtime within the last second).
5. **Restart the three daemons** against the live export dir (they were zombies feeding on a stale frame):
   ```bash
   # kill the stale ones first
   pkill -f 'tempo people' ; pkill -f 'tempo objects' ; pkill -f 'hands track'

   cd agent
   uv run tempo objects --frames "$TMPDIR/spatula-frames" &          # OWL-ViT object map
   uv run tempo people --frames "$TMPDIR/spatula-frames" &          # faces + voices (optional for placement, but harmless)
   cd ../hands
   uv run hands track --dir "$TMPDIR/spatula-frames" --enable-export &  # Mac-side hands -> gesture engine
   ```
   `hands track --enable-export` will itself turn `frame-export` on for `--dir`, so if you start hands you can skip step 4.
6. **Confirm the agent can see the room.**
   ```bash
   cd agent
   uv run tempo scene > /tmp/scene.json && head -40 /tmp/scene.json
   ```
   You should see `surfaces` with at least one `table` and one `wall` (when you are facing them), `objects` with the lamp / monitor / whiteboard / plant tracked and `unseen_s` small, and `hand.visible: true` if your hand is in frame. `last-view.jpg` is the frame the agent will get.

## Position the wearer for the new groups

The eval runs the request set as-is; there is no per-request repositioning. So before you start, stand / sit so that **both** the desk and the target objects are in the camera view at once:

- A **horizontal surface** (the desk / table) is in view and the LiDAR plane list shows a `table` plane. This makes the desk-in-view group (21–25) winnable.
- The **anchor objects** are in view and the object map has confirmed tracks for them: lamp, whiteboard, plant, monitor. Run `uv run tempo objects --frames "$TMPDIR/spatula-frames" --once` a couple of times and check the printed detections; `charger` will never appear (it is not in the vocabulary) — that is intentional, request 28 is the fallback case.
- A **wall** is in view (the legacy wall requests and the `wall` expect still need it).

If you cannot get the desk and the lamp/monitor in the same frame, run the two groups in separate passes with `--limit` (see below) and note it in the README.

## Run the eval

From the `agent/` dir, with `GEMINI_API_KEY` in `agent/.env`:

```bash
cd agent
uv run tempo eval                       # both conditions, all 30 requests
uv run tempo eval geometry              # geometry condition only
uv run tempo eval geometry pixels       # explicit order
uv run tempo eval geometry --limit 20   # legacy set only (ids 1–20)
uv run tempo eval geometry --limit 30   # full set including the two new groups
```

The runner:
1. Closes all existing panels before each request.
2. Takes a fresh snapshot (`perception.take`).
3. Calls `Brain.decide` with the request text + snapshot (geometry condition gets the scene JSON; pixels condition is told to estimate from the image).
4. Executes the returned actions (`Executor.run`).
5. Records where the last-placed panel landed.
6. Writes `agent/eval/results/<condition>-<unix-ts>.json` and prints one line per trial + a summary.

A run of all 30 in both conditions is ~30 × 2 × 4 s ≈ 4 min. Stand still while it runs.

## Where results land

`agent/eval/results/<condition>-<unix-ts>.json` — one array of `Trial` rows (see `eval.py` `Trial`). The two newest files per condition are the latest run.

## Update the README results table

After a run, take the printed `summarize()` line for each condition and add a row to the README's checkpoint results table (`README.md`, the "Checkpoint N result" tables). For a group-level breakdown (desk-in-view vs object-anchored), slice the written JSON by `group`:

```bash
cd agent
python3 - <<'PY'
import json, glob, os
latest = max(glob.glob("eval/results/geometry-*.json"), key=os.path.getmtime)
trials = json.load(open(latest))
for g in ("legacy", "desk-in-view", "object-anchored"):
    rows = [t for t in trials if t.get("group", "legacy") == g]
    errs = [t["surface_error_m"] for t in rows if t["surface_error_m"] is not None]
    acc = sum(t["correct_surface"] for t in rows) / len(rows) if rows else 0
    med = sorted(errs)[len(errs)//2] if errs else None
    lat = sum(t["latency_s"] for t in rows) / len(rows) if rows else 0
    print(f"{g:16s} n={len(rows):2d}  acc={acc:.2f}  median_err={med}  mean_lat={lat:.1f}s")
PY
```

Note that the legacy entries (1–20) carry no `group` field; the slicer above treats a missing `group` as `legacy`. Add the new rows to the README table and say in the "What changed" column that the desk-in-view and object-anchored groups were added.

## Scoring object-anchored requests (honest limit)

The current scorer (`eval.py _expected_kind`) returns `None` for `expect: "object"`, so `correct_surface` becomes "a panel was placed" — it does **not** verify the panel is near the named anchor. That is a known gap, not a bug: the README promises to "report how often that fallback lands," and the executor log already records whether the `near_object` path was taken (`actions` row, e.g. `note #26 'bedtime' -> near_object (lamp)` vs a fallback `-> where_looking`).

For check-in 3, report object-anchored results as: of the 5 object-anchored requests, how many took the `near_object` path (object map hit) vs fell back to `where_looking` (image guess), and of the `near_object` ones, whether the panel visually landed beside the named object. To get a numeric distance-to-anchor score, extend `_expected_kind` to read the `anchor` field, look up the anchor's position in `objects.snapshot()`, and compare to the placed panel's `pos` — a small change, but out of scope for this prep commit (no live rig to validate against). Track it as the next scorer task.

## Failure modes to watch

- **`packet_rate = 0`** → phone not streaming; do not run, numbers will be stale.
- **No `table` plane in `tempo scene` while running the desk-in-view group** → the same miss the README already reports; reposition so the desk is in view and re-run just that group with `--limit` slicing (edit `requests.json` temporarily, or filter the result JSON by `group`).
- **`objects` list empty in `tempo scene`** → the object detector daemon is not running or the frame export is stale; the object-anchored group will fall back to image for every request.
- **`actions` row shows `near_object (lamp)` but the panel is nowhere near the lamp** → the object map's 3D position is off (often `source: "prior"` height estimate); this is the honest limit the README calls out.
