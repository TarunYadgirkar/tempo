import math

import numpy as np

from tempo import people as pp
from tempo import voices as vv


def unit(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    v = rng.standard_normal(512).astype(np.float32)
    return v / np.linalg.norm(v)


def test_is_garbage_drops_whisper_hallucinations():
    # The exact strings the live mic produced before the fix.
    assert vv._is_garbage("subscribe dot com")
    assert vv._is_garbage("code 218 00 00 00 00 00 00 00")
    assert vv._is_garbage("shoot fold fold fold fold")
    assert vv._is_garbage("Ok Ok")
    assert vv._is_garbage("Uh Uh")
    assert vv._is_garbage("...")
    assert vv._is_garbage("thanks for watching.")
    assert vv._is_garbage("Please subscribe.")
    assert vv._is_garbage("ok")
    assert vv._is_garbage("")
    # Real speech survives the gate.
    assert not vv._is_garbage("I'm Tarun and this is my apartment.")
    assert not vv._is_garbage("Can you place the mug on the desk?")
    assert not vv._is_garbage("Maroon red or I'm thinking about maroon.")


def test_ear_finish_requires_minimum_speech_duration():
    """A segment that is long enough overall but has <MIN_SPEECH_S of above-threshold
    speech (a noise burst padded with silence) is not queued for transcription."""
    ear = vv.Ear(lambda seg: None, log=lambda *a: None)
    # Simulate: 1.0 s of silence (33 blocks) then a 0.1 s noise burst (3 blocks) then
    # enough silence to end. Total > MIN_SEGMENT_S but speech < MIN_SPEECH_S.
    ear._speaking = True
    silence = np.zeros(vv.BLOCK, dtype="int16")
    for _ in range(33):
        ear._chunks.append(silence)
        ear._silence_blocks += 1
    burst = np.full(vv.BLOCK, 40, dtype="int16")  # below SPEECH_FLOOR_RMS
    for _ in range(3):
        ear._chunks.append(burst)
        # _speech_blocks only counts blocks that cleared the threshold in _on_audio;
        # emulate a burst that did not.
    ear._speech_blocks = 3  # 3 * 30ms = 90ms < 0.4s
    queued = []
    ear._queue.put = queued.append  # type: ignore[assignment]
    ear._finish()
    assert queued == []
    # Now with enough real speech it is queued.
    ear._speaking = True
    ear._chunks = [silence] * 10 + [np.full(vv.BLOCK, 2000, dtype="int16")] * 20
    ear._speech_blocks = 20  # 600ms >= 0.4s
    ear._silence_blocks = 0
    ear._finish()
    assert len(queued) == 1


def test_parse_name_keeps_case_and_drops_filler():
    assert pp.parse_name("Hi, I am Tarun and this is great") == ("self", "Tarun")
    assert pp.parse_name("my name is Alice Chen") == ("self", "Alice Chen")
    assert pp.parse_name("This is Bob") == ("other", "Bob")
    assert pp.parse_name("This is the best") is None
    assert pp.parse_name("I'm going home") is None


def test_parse_name_rejects_lowercase_filler_after_im():
    # Whisper hallucinations that previously became false names: "I'm gonna", "I'm sick",
    # "I'm obviously". The name must start with a capital letter, so these are not names.
    assert pp.parse_name("I'm gonna have my little teleporting thing") is None
    assert pp.parse_name("I'm sick of this") is None
    assert pp.parse_name("I'm obviously going to do it") is None
    assert pp.parse_name("I'm Tarun") == ("self", "Tarun")


def test_parse_name_rejects_nonalphabetic_and_overlong_names():
    # A name must be alphabetic and 2-20 chars; garbage that Whisper once capitalized
    # ("I'm Gonna" with a stray capital, numeric junk, a 25-char run) never enrolls.
    assert pp.parse_name("I'm Gonna") is None            # "gonna" is a stopword
    assert pp.parse_name("I'm 21800") is None            # numeric
    assert pp.parse_name("this is A") is None            # too short (<2)
    assert pp.parse_name("I'm Tarun") == ("self", "Tarun")
    assert pp.parse_name("this is Alice Chen") == ("other", "Alice Chen")
    # A 21-char single word is too long to be a plausible first name.
    assert pp.parse_name("I'm Abcdefghijklmnopqrstu") is None  # 21 chars


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


def test_summary_cadence(tmp_path, monkeypatch):
    monkeypatch.setattr(pp, "CONVERSATION_LOG", tmp_path / "c.jsonl")
    p = pp.Person(id="y")
    for i in range(2):
        p.say(f"line {i}", float(i))
    assert not p.wants_summary()
    p.say("line 2", 2.0)
    assert p.wants_summary()
    p.summary, p.summarized_count = "talked about lines", 3
    p.say("line 3", 3.0)
    assert not p.wants_summary()
    p.say("line 4", 4.0)
    assert p.wants_summary()
    assert (tmp_path / "c.jsonl").read_text().count("\n") == 5
    assert "talked about lines" in pp.bubble_text(p)[1]


def test_spatial_track_reuses_nearby_face(tmp_path):
    """A face seen within TRACK_RADIUS_M of a recent sighting is the same person,
    so one person in view keeps one bubble instead of fragmenting per frame."""
    store = pp.People(tmp_path / "people.json")
    person = store.new("face", unit(1))
    daemon = pp.PeopleDaemon.__new__(pp.PeopleDaemon)
    daemon.recent = {}
    daemon.people = store
    # First sighting at z=-1.0; a moment later a face reappears 0.05 m away (same person).
    face = pp.Face(box=(0, 0, 10, 10), score=1.0, embedding=unit(2))
    daemon.recent[person.id] = pp.Sighting(person, face, (0.0, 0.0, -1.0), 0.0)
    assert daemon._track((0.05, 0.0, -1.0), 0.5) is person
    # A face 0.5 m away is too far -> no spatial track.
    assert daemon._track((0.5, 0.0, -1.0), 0.5) is None
    # After TRACK_WINDOW_S the track expires.
    assert daemon._track((0.05, 0.0, -1.0), pp.TRACK_WINDOW_S + 1.0) is None


class _FakeShell:
    """Minimal Shell stand-in for bubble lifecycle tests."""

    def __init__(self, update_raises=False):
        self.update_raises = update_raises
        self.note_calls = 0
        self.update_calls = 0
        self.pose_calls = 0
        self.close_calls = 0
        self._next_handle = 100

    def note(self, title, body, accent=False):
        self.note_calls += 1
        self._next_handle += 1
        return self._next_handle

    def send(self, line):
        self.update_calls += 1
        if self.update_raises:
            raise pp.ShellError(f"{line!r} -> err no_such_window")
        from .shell import Reply  # noqa: F401  (kept for parity with Shell)
        return type("R", (), {"ok": True})()

    def pose(self, handle, pos, quat):
        self.pose_calls += 1

    def close_window(self, handle):
        self.close_calls += 1


def test_bubble_recreates_when_shell_closed_its_window():
    """A note-update on a window the compositor already retired (no_such_window)
    must not crash the daemon: drop the stale handle and create a fresh bubble."""
    shell = _FakeShell(update_raises=True)
    bubbles = pp.Bubbles(shell, log=lambda *a: None)
    person = pp.Person(id="p1")
    person.name = "Alice"
    # First show creates the bubble.
    bubbles.show(person, (0.0, 0.0, -1.0), (0.0, 0.0, 0.0), 0.0)
    assert shell.note_calls == 1
    first_handle = bubbles.by_person[person.id].handle
    # A new utterance changes the bubble text -> note-update is attempted, fails,
    # and the bubble is recreated instead of raising.
    person.say("now I have a new line", 1.0)
    bubbles.show(person, (0.0, 0.0, -1.0), (0.0, 0.0, 0.0), 1.0)
    assert shell.update_calls == 1
    assert shell.note_calls == 2  # recreated
    assert bubbles.by_person[person.id].handle != first_handle
