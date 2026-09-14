"""Unit tests for the placement eval scorer (agent/src/tempo/eval.py).

These cover the object-anchored scoring gap closed in this commit: for
`expect: "object"` with an `anchor` field, `correct_surface` is now decided by
the Euclidean distance from the placed panel to the anchor object's 3D position
in the objects snapshot, with an honest `anchor_not_found` fallback. Legacy
table/wall/gaze/front entries must score exactly as before.

No live rig is needed: the scorer helpers are pure and take synthetic snapshots.
"""
from __future__ import annotations

from tempo import eval as E


def _obj(label: str, pos: tuple[float, float, float]) -> dict:
    """One objects.snapshot() entry — the shape objects.Track.public() returns."""
    return {
        "label": label,
        "confidence": 0.9,
        "position_m": list(pos),
        "size_m_estimate": 0.2,
        "source": "depth",
        "unseen_s": 0.0,
    }


def _window(pos: tuple[float, float, float]) -> dict:
    """A placed panel, as shell.windows() exposes it (only `pos` is read by the scorer)."""
    return {"handle": 1, "title": "t", "app_id": "a", "pos": list(pos), "anchor": None, "focused": True}


# --- object-anchored scoring --------------------------------------------------------


def test_object_anchor_within_threshold_scores_correct():
    snap = [_obj("lamp", (1.0, 0.5, -1.0))]
    req = {"id": 26, "text": "put a note next to the lamp that says bedtime",
           "expect": "object", "anchor": "lamp"}
    scored = E._score(req, [_window((1.05, 0.5, -0.85))], [], {"scene_pos": [0, 0, 0]}, snap)
    assert scored["correct_surface"] is True
    assert scored["anchor_found"] is True
    assert scored["anchor"] == "lamp"
    assert scored["reason"] is None
    # distance ~0.18 m, recorded and within the 0.30 m threshold
    assert scored["anchor_distance_m"] is not None
    assert scored["anchor_distance_m"] <= E.ANCHOR_NEAR_M


def test_object_anchor_far_away_scores_incorrect_with_distance():
    snap = [_obj("whiteboard", (0.0, 1.2, -2.0))]
    req = {"id": 27, "text": "open spotify by the whiteboard",
           "expect": "object", "anchor": "whiteboard"}
    scored = E._score(req, [_window((2.0, 1.2, -2.0))], [], {"scene_pos": [0, 0, 0]}, snap)
    assert scored["correct_surface"] is False
    assert scored["anchor_found"] is True
    assert scored["reason"] is None
    assert scored["anchor_distance_m"] == round(2.0, 3)  # 2.0 m off


def test_object_anchor_missing_from_snapshot_scores_incorrect_with_reason():
    # charger is deliberately not in the OWL-ViT default vocabulary -> the honest
    # image / remembered-place fallback the README promises to report.
    snap = [_obj("lamp", (1.0, 0.5, -1.0)), _obj("monitor", (0.5, 1.0, -1.2))]
    req = {"id": 28, "text": "put a reminder next to my charger: unplug at 6",
           "expect": "object", "anchor": "charger"}
    scored = E._score(req, [_window((0.5, 1.0, -1.0))], [], {"scene_pos": [0, 0, 0]}, snap)
    assert scored["correct_surface"] is False
    assert scored["anchor_found"] is False
    assert scored["anchor_distance_m"] is None
    assert scored["reason"] == "anchor_not_found"


def test_object_anchor_no_panel_placed_is_incorrect():
    snap = [_obj("plant", (0.0, 0.4, -1.5))]
    req = {"id": 29, "text": "leave a note by the plant that says water me",
           "expect": "object", "anchor": "plant"}
    scored = E._score(req, [], [], {"scene_pos": [0, 0, 0]}, snap)
    assert scored["correct_surface"] is False
    assert scored["anchor_found"] is False
    assert scored["reason"] == "no_panel"


def test_object_anchor_picks_nearest_of_multiple_same_label():
    snap = [_obj("monitor", (0.5, 1.0, -1.2)), _obj("monitor", (3.0, 1.0, -1.2))]
    req = {"id": 30, "text": "open messages next to the monitor",
           "expect": "object", "anchor": "monitor"}
    scored = E._score(req, [_window((0.55, 1.0, -1.05))], [], {"scene_pos": [0, 0, 0]}, snap)
    assert scored["correct_surface"] is True
    assert scored["anchor_distance_m"] is not None
    assert scored["anchor_distance_m"] < E.ANCHOR_NEAR_M


# --- legacy entries score exactly as before ----------------------------------------


def test_legacy_table_request_still_scores_as_before():
    """A table request with a panel placed over a table plane is correct, no anchor keys set."""
    head = {"scene_pos": [0.0, 1.5, 0.0], "scene_rot": [0.0, 0.0, 0.0, 1.0]}
    table_plane = {
        "uuid": "t1", "normal": [0.0, 1.0, 0.0], "center": [0.0, 0.7, -1.0],
        "extent": [1.0, 1.0],  # square extent; max/2 + slack = 0.5 + 0.25 = 0.75 m reach
    }
    req = {"id": 1, "text": "put a note on the desk that says buy milk", "expect": "table"}
    # panel sits on the table
    scored = E._score(req, [_window((0.0, 0.75, -1.0))], [table_plane], head, [])
    assert scored["correct_surface"] is True
    assert scored["landed_kind"] == "table"
    assert scored["anchor"] is None
    assert scored["anchor_found"] is None
    assert scored["anchor_distance_m"] is None
    assert scored["reason"] is None


def test_legacy_front_request_any_panel_counts_as_correct():
    head = {"scene_pos": [0.0, 1.5, 0.0], "scene_rot": [0.0, 0.0, 0.0, 1.0]}
    req = {"id": 10, "text": "put a note in front of me that says 7pm checkpoint", "expect": "front"}
    scored = E._score(req, [_window((0.0, 1.4, -0.9))], [], head, [])
    assert scored["correct_surface"] is True
    assert scored["anchor"] is None
