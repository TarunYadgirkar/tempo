import math

import numpy as np

from tempo import people as pp


def unit(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(512).astype(np.float32)
    return v / np.linalg.norm(v)


def test_parse_name_keeps_case_and_drops_filler():
    assert pp.parse_name("Hi, I am Tarun and this is great") == ("self", "Tarun")
    assert pp.parse_name("my name is Alice Chen") == ("self", "Alice Chen")
    assert pp.parse_name("This is Bob") == ("other", "Bob")
    assert pp.parse_name("This is the best") is None
    assert pp.parse_name("I'm going home") is None


def test_match_then_name_merges_duplicates(tmp_path):
    store = pp.People(tmp_path / "people.json")
    a = store.new("face", unit(1))
    assert store.match("face", unit(1))[0] is a
    assert store.match("face", unit(2))[0] is None
    b = store.new("face", unit(3))
    store.name(a, "Alice")
    merged = store.name(b, "alice")
    assert merged is a and len(store.people) == 1
    assert len(a.faces) == 2
    store.save()
    again = pp.People(tmp_path / "people.json")
    assert again.by_name("Alice") is not None


def test_bubble_sits_to_the_wearers_right_and_faces_them():
    head = (0.0, 0.0, 0.0)
    face = (0.0, 0.0, -1.0)  # straight ahead
    pos, quat = pp.bubble_pose(face, head)
    assert pos[0] > 0.2 and abs(pos[2] + 1.0) < 1e-6
    # front = (sin yaw, 0, cos yaw) must point back at the head
    yaw = 2 * math.atan2(quat[1], quat[3])
    front = (math.sin(yaw), math.cos(yaw))
    to_head = (head[0] - pos[0], head[2] - pos[2])
    n = math.hypot(*to_head)
    assert front[0] * to_head[0] / n + front[1] * to_head[1] / n > 0.99


def test_bubble_text_asks_for_a_name_until_it_has_one():
    p = pp.Person(id="x")
    title, body = pp.bubble_text(p)
    assert title == "Someone new" and "this is Alice" in body
    p.name = "Alice"
    p.say("we should ship on Monday", 0.0)
    title, body = pp.bubble_text(p)
    assert title == "Alice" and "ship on Monday" in body
