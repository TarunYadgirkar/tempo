"""People in the room: who they are, where their head is, what they said.

A face seen through the head camera becomes a person the moment it is seen;
a name arrives when they say "I'm Alice" or the wearer says "this is Alice".
Each visible person gets a note bubble pinned beside their head with their
name and context. Faces (ArcFace) and voices (ECAPA) share one store, so
"where is my charger" and "who is this" are the same memory.
"""
from __future__ import annotations

import json
import math
import os
import re
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from . import objects, siyi, spatial
from .faces import FACE_MATCH, Face, FaceEngine, similarity
from .shell import Shell, ShellError
from .voices import VOICE_MATCH, Ear, Segment

STORE = Path(os.environ.get("TEMPO_PEOPLE_STORE") or Path.home() / ".config" / "tempo" / "people.json")
CONVERSATION_LOG = STORE.with_name("conversations.jsonl")
SUMMARY_MIN_UTTERANCES = 3
SUMMARY_EVERY = 2
MAX_EMBEDDINGS = 12
MAX_UTTERANCES = 30
# A new embedding is only banked when it adds something the bank lacks.
BANK_NOVELTY = 0.75
FOCUS_WINDOW_S = 2.5   # a spoken name goes to the face seen within this long
BUBBLE_GONE_S = 3.0     # close the bubble this long after the face leaves
BUBBLE_RIGHT_M = 0.30   # bubble sits this far to the wearer's right of the head
BUBBLE_UP_M = 0.04
BUBBLE_MOVE_M = 0.06    # re-pose only when the head moved this much
BUBBLE_MIN_REPOSE_S = 0.25
FRAME_POLL_S = 0.12
# Spatial re-identification: a face seen within this radius+time of a recent sighting is the same
# person, so one person in view keeps one bubble instead of fragmenting into a new face each frame
# (ArcFace cross-frame cosine hovers ~0.40 < FACE_MATCH 0.42 and would otherwise spawn duplicates).
TRACK_WINDOW_S = 4.0
TRACK_RADIUS_M = 0.35
# Words that can follow "I'm"/"this is" but are not a name. Whisper capitalizes proper nouns,
# so the NAME regex below requires a capital first letter; this set is the second line of defense
# against sentence-initial capitals ("This is The ...") and rare capitalized filler.
STOPWORDS = {"the", "a", "an", "my", "your", "it", "what", "how", "not", "so", "just", "going", "really",
             "here", "there", "me", "him", "her", "good", "great", "fine", "okay", "ok", "done", "back",
             "gonna", "wanna", "gotta", "kinda", "sick", "obviously", "actually", "basically", "literally",
             "very", "sorry", "sure", "yeah", "yes", "no", "like", "doing", "feeling", "looking", "trying",
             "getting", "ready", "tired", "hungry", "late", "busy", "awesome", "cool", "glad", "happy",
             "sad", "bored", "excited", "confused", "worried", "look", "listen", "man", "dude", "guys",
             "everyone", "everybody", "someone", "somebody", "nothing", "something", "everything",
             "anything", "finished", "stuffed", "full", "drunk", "sleepy", "scared", "afraid", "nervous",
             "anxious", "proud", "honored", "blessed", "alright", "right", "still", "now", "again", "home"}
# The cue is case-insensitive; the NAME keeps its case so "and" never becomes a surname.
# The first letter must be uppercase: Whisper capitalizes proper nouns, so "I'm gonna" (lowercase g)
# never becomes a name, while "I'm Tarun" does.
NAME = r"([A-Z][a-z]+(?:\s+[A-Z][a-z]+)?)"
FIRST_PERSON = re.compile(r"\b(?i:i am|i'm|im|my name is|my name's|call me)\s+" + NAME)
THIRD_PERSON = re.compile(r"\b(?i:this is|that's|that is|meet|say hi to|say hello to|her name is|his name is|their name is)\s+" + NAME)


@dataclass
class Person:
    id: str
    name: str | None = None
    faces: list[list[float]] = field(default_factory=list)
    voices: list[list[float]] = field(default_factory=list)
    utterances: list[dict[str, Any]] = field(default_factory=list)
    first_seen: float = field(default_factory=time.time)
    last_seen: float = field(default_factory=time.time)
    seen_count: int = 0
    is_owner: bool = False
    siyi: dict[str, Any] | None = None
    siyi_checked_for: str | None = None
    summary: str | None = None        # one line from Gemini: what we talked about
    summarized_count: int = 0         # utterances covered by `summary`

    @property
    def label(self) -> str:
        return self.name or "Someone new"

    def bank(self, kind: str) -> np.ndarray:
        rows = self.faces if kind == "face" else self.voices
        return np.asarray(rows, dtype=np.float32) if rows else np.empty((0, 0), dtype=np.float32)

    def add_embedding(self, kind: str, emb: np.ndarray) -> None:
        rows = self.faces if kind == "face" else self.voices
        bank = self.bank(kind)
        if bank.size and float(np.max(bank @ emb)) > BANK_NOVELTY and len(rows) >= 3:
            return
        rows.append([round(float(v), 5) for v in emb])
        del rows[:-MAX_EMBEDDINGS]

    def say(self, text: str, at: float) -> None:
        self.utterances.append({"t": at, "text": text})
        del self.utterances[:-MAX_UTTERANCES]
        try:
            CONVERSATION_LOG.parent.mkdir(parents=True, exist_ok=True)
            with CONVERSATION_LOG.open("a") as f:
                f.write(json.dumps({"t": at, "person": self.id, "name": self.name, "text": text}) + "\n")
        except OSError:
            pass

    def wants_summary(self) -> bool:
        n = len(self.utterances)
        return n >= SUMMARY_MIN_UTTERANCES and n - self.summarized_count >= SUMMARY_EVERY

    def last_said(self) -> str | None:
        return self.utterances[-1]["text"] if self.utterances else None


class People:
    def __init__(self, store: Path = STORE) -> None:
        self.store = store
        self.lock = threading.RLock()
        self.people: dict[str, Person] = {}
        self._load()

    def _load(self) -> None:
        if not self.store.exists():
            return
        raw = json.loads(self.store.read_text())
        for row in raw.get("people", []):
            self.people[row["id"]] = Person(**row)

    def save(self) -> None:
        with self.lock:
            self.store.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.store.with_suffix(".json.tmp")
            tmp.write_text(json.dumps({"people": [p.__dict__ for p in self.people.values()]}, indent=1))
            tmp.replace(self.store)

    def match(self, kind: str, emb: np.ndarray) -> tuple[Person | None, float]:
        threshold = FACE_MATCH if kind == "face" else VOICE_MATCH
        best, best_score = None, -1.0
        with self.lock:
            for p in self.people.values():
                s = similarity(emb, p.bank(kind))
                if s > best_score:
                    best, best_score = p, s
        return (best, best_score) if best is not None and best_score >= threshold else (None, best_score)

    def new(self, kind: str, emb: np.ndarray) -> Person:
        p = Person(id=uuid.uuid4().hex[:8])
        p.add_embedding(kind, emb)
        with self.lock:
            self.people[p.id] = p
        return p

    def by_name(self, name: str) -> Person | None:
        name = name.strip().lower()
        for p in self.people.values():
            if p.name and p.name.lower() == name:
                return p
        return None

    def name(self, person: Person, name: str) -> Person:
        """Attach a name; if that name already exists, fold this person into it."""
        with self.lock:
            existing = self.by_name(name)
            if existing and existing.id != person.id:
                for kind in ("face", "voice"):
                    for row in (person.faces if kind == "face" else person.voices):
                        existing.add_embedding(kind, np.asarray(row, dtype=np.float32))
                existing.utterances = (existing.utterances + person.utterances)[-MAX_UTTERANCES:]
                existing.seen_count += person.seen_count
                del self.people[person.id]
                return existing
            person.name = name.strip()
            return person

    def owner(self) -> Person | None:
        return next((p for p in self.people.values() if p.is_owner), None)

    def forget(self, name: str) -> bool:
        with self.lock:
            p = self.by_name(name)
            if not p:
                return False
            del self.people[p.id]
            return True


RECENT_S = 15.0


def describe(head: dict[str, Any] | None = None) -> dict[str, Any]:
    """What the agent should know about people: who is here now, who it knows."""
    try:
        store = People()
    except Exception:
        return {"present": [], "known": []}
    now = time.time()
    present, known = [], []
    for p in store.people.values():
        if p.is_owner:
            continue
        known.append(p.label)
        if now - p.last_seen <= RECENT_S:
            entry: dict[str, Any] = {"name": p.name, "seen_s_ago": round(now - p.last_seen, 1)}
            if p.utterances:
                entry["recent"] = [u["text"] for u in p.utterances[-5:]]
            if p.summary:
                entry["summary"] = p.summary
            if p.siyi:
                entry["notes"] = p.siyi.get("note") or None
            present.append(entry)
    return {"present": present, "known": sorted(n for n in known if n != "Someone new")}


SUMMARY_PROMPT = (
    "You are the memory of a pair of AR glasses. Below are things one person said to the wearer, in order. "
    "Write ONE plain sentence (max 18 words) the wearer would want beside this person's face next time: "
    "what they talked about, anything they asked for or promised. No preamble, no quotes."
)


_gemini = None


def summarize(person: Person) -> str | None:
    """One line from Gemini about what this person and the wearer discussed."""
    key = os.environ.get("GEMINI_API_KEY")
    if not key or not person.utterances:
        return None
    from google import genai

    global _gemini
    if _gemini is None:
        _gemini = genai.Client(api_key=key)  # the SDK closes a client that goes out of scope mid-call
    lines = "\n".join(f"- {u['text']}" for u in person.utterances[-MAX_UTTERANCES:])
    who = person.name or "an unnamed person"
    try:
        resp = _gemini.models.generate_content(
            model=os.environ.get("TEMPO_MODEL", "gemini-3.6-flash"),
            contents=f"{SUMMARY_PROMPT}\n\nPerson: {who}\n{lines}",
        )
        text = (resp.text or "").strip().splitlines()
        return text[0].strip()[:160] if text else None
    except Exception as exc:
        print(f"people: summary failed: {exc}", file=sys.stderr)
        return None


def parse_name(text: str) -> tuple[str, str] | None:
    """('self'|'other', Name) when the sentence introduces someone.

    A name is only mined from a transcript that already passed the voices-layer
    gates, and even then must look like a name: alphabetic, 2-20 chars, capital
    first letter (Whisper capitalizes proper nouns), not a sentence-initial
    filler word. So "I'm gonna" / "I'm obviously" never enroll a person.
    """
    for kind, pattern in (("self", FIRST_PERSON), ("other", THIRD_PERSON)):
        m = pattern.search(text)
        if not m:
            continue
        candidate = m.group(1).strip()
        first = candidate.split()[0]
        if first.lower() in STOPWORDS or len(first) < 2:
            continue
        # Plausible name: letters only, 2-20 chars total ("Alice", "Alice Chen").
        if not (2 <= len(candidate) <= 20):
            continue
        if not candidate.replace(" ", "").isalpha():
            continue
        return kind, " ".join(w.capitalize() for w in candidate.split())
    return None


def bubble_text(person: Person) -> tuple[str, str]:
    lines: list[str] = []
    if person.siyi:
        if person.siyi.get("note"):
            lines.append(person.siyi["note"])
        if person.siyi.get("last_at"):
            tail = f": {person.siyi['last_note']}" if person.siyi.get("last_note") else ""
            lines.append(f"Last logged {person.siyi['last_at']}{tail}")
    if person.summary:
        lines.append(person.summary)
    said = person.last_said()
    if said:
        lines.append(f"Said: “{said[:90]}”")
    if not person.name:
        lines.append("Not in memory yet. Say “this is Alice”, or let them say “I’m Alice”.")
    elif not lines:
        lines.append(f"Seen {person.seen_count} times since {time.strftime('%b %d', time.localtime(person.first_seen))}.")
    return person.label, "\n".join(lines)


def bubble_pose(face_pos: spatial.Vec3, head_pos: spatial.Vec3) -> tuple[spatial.Vec3, tuple[float, float, float, float]]:
    """Beside the head, on the wearer's right, facing the wearer."""
    fwd = spatial.normalize((face_pos[0] - head_pos[0], 0.0, face_pos[2] - head_pos[2]))
    right = (-fwd[2], 0.0, fwd[0])
    pos = (face_pos[0] + right[0] * BUBBLE_RIGHT_M, face_pos[1] + BUBBLE_UP_M, face_pos[2] + right[2] * BUBBLE_RIGHT_M)
    to_head = (head_pos[0] - pos[0], head_pos[2] - pos[2])
    yaw = math.atan2(to_head[0], to_head[1])
    return pos, (0.0, math.sin(yaw / 2), 0.0, math.cos(yaw / 2))


@dataclass
class Bubble:
    handle: int
    pos: spatial.Vec3
    text: tuple[str, str]
    last_seen: float
    last_posed: float = 0.0


@dataclass
class Sighting:
    person: Person
    face: Face
    pos: spatial.Vec3
    at: float


class Bubbles:
    def __init__(self, shell: Shell, log=print) -> None:
        self.shell = shell
        self.log = log
        self.by_person: dict[str, Bubble] = {}

    def show(self, person: Person, pos: spatial.Vec3, head_pos: spatial.Vec3, now: float) -> None:
        text = bubble_text(person)
        b = self.by_person.get(person.id)
        if b is None:
            handle = self.shell.note(text[0], text[1], accent=person.name is None)
            b = Bubble(handle, (math.inf, 0, 0), text, now)
            self.by_person[person.id] = b
            self.log(f"people: bubble {handle} for {person.label}")
        elif text != b.text:
            self.shell.send(f"note-update {b.handle} " + json.dumps({"title": text[0], "body": text[1], "accent": person.name is None}))
            b.text = text
        b.last_seen = now
        moved = math.dist(pos, b.pos) if math.isfinite(b.pos[0]) else math.inf
        if moved > BUBBLE_MOVE_M and now - b.last_posed >= BUBBLE_MIN_REPOSE_S:
            bpos, quat = bubble_pose(pos, head_pos)
            self.shell.pose(b.handle, bpos, quat)
            b.pos, b.last_posed = pos, now

    def retire(self, person: Person) -> None:
        b = self.by_person.pop(person.id, None)
        if b:
            try:
                self.shell.close_window(b.handle)
            except ShellError:
                pass

    def sweep(self, people: People, now: float) -> None:
        for pid, b in list(self.by_person.items()):
            if now - b.last_seen > BUBBLE_GONE_S:
                p = people.people.get(pid)
                if p:
                    self.retire(p)
                else:
                    self.by_person.pop(pid, None)

    def close_all(self) -> None:
        for pid in list(self.by_person):
            b = self.by_person.pop(pid)
            try:
                self.shell.close_window(b.handle)
            except ShellError:
                pass


class PeopleDaemon:
    def __init__(self, frames_dir: str | Path, shell: Shell, mic: bool = True, log=print) -> None:
        self.frames = Path(frames_dir)
        self.shell = shell
        self.log = log
        self.people = People()
        self.engine = FaceEngine()
        self.bubbles = Bubbles(shell, log)
        self.ear = Ear(self.on_segment, log) if mic else None
        self.recent: dict[str, Sighting] = {}
        self.last_stamp: tuple[Any, Any] | None = None
        self.head_pos: spatial.Vec3 = (0.0, 0.0, 0.0)

    # -- frames -------------------------------------------------------------

    def step_frame(self) -> int:
        pair = objects.read_export(self.frames)
        if pair is None:
            return 0
        image_path, meta = pair
        stamp = (meta.get("seq"), meta.get("t_ns"))
        if stamp == self.last_stamp:
            return 0
        self.last_stamp = stamp
        image = objects._open_frame(image_path, meta)
        rgb = np.asarray(image)
        head = meta.get("head") or objects.IDENTITY_HEAD
        intr = objects._intrinsics_for(image, meta)
        depth = objects.DepthMap.load(meta.get("depth"), image_path.parent)
        now = time.time()
        self.head_pos = tuple(head.get("pos") or head.get("scene_pos") or (0.0, 0.0, 0.0))
        faces = self.engine.faces(rgb)
        with self.people.lock:
            for face in faces:
                pos = self.locate(face, head, intr, depth, (image.width, image.height))
                tracked = self._track(pos, now)
                if tracked is not None:
                    person = tracked
                    person.add_embedding("face", face.embedding)
                else:
                    person, score = self.people.match("face", face.embedding)
                    if person is None:
                        person = self.people.new("face", face.embedding)
                        self.log(f"people: new face {person.id} (best {score:.2f})")
                    elif score < BANK_NOVELTY:
                        person.add_embedding("face", face.embedding)
                person.last_seen, person.seen_count = now, person.seen_count + 1
                self.recent[person.id] = Sighting(person, face, pos, now)
                self.enrich(person)
                self.bubbles.show(person, pos, self.head_pos, now)
            self._prune_recent(now)
            self.bubbles.sweep(self.people, now)
        if faces:
            self.people.save()
        return len(faces)

    @staticmethod
    def locate(face: Face, head: dict, intr: dict, depth: objects.DepthMap | None, frame: tuple[int, int]) -> spatial.Vec3:
        u, v = face.center
        scale = intr["width"] / frame[0]
        d = depth.sample(u, v, *frame) if depth else None
        if d is None:
            # A face is ~0.16 m tall; use its pixel height as the range.
            d = intr["fy"] * 0.16 / max(face.box[3] * scale, 1.0)
        cam = objects.unproject(u * scale, v * scale, d, intr)
        return objects.to_scene(cam, head)

    def _track(self, pos: spatial.Vec3, now: float) -> Person | None:
        """A face within TRACK_RADIUS_M of a sighting in the last TRACK_WINDOW_S is the same person."""
        best, best_d = None, TRACK_RADIUS_M
        for s in self.recent.values():
            if now - s.at > TRACK_WINDOW_S:
                continue
            d = math.dist(pos, s.pos)
            if d <= best_d:
                best, best_d = s.person, d
        return best

    def _prune_recent(self, now: float) -> None:
        """Drop sightings old enough that no track or focus can still reference them."""
        cutoff = max(TRACK_WINDOW_S, FOCUS_WINDOW_S) + BUBBLE_GONE_S
        for pid in [pid for pid, s in self.recent.items() if now - s.at > cutoff]:
            self.recent.pop(pid, None)

    def enrich(self, person: Person) -> None:
        if person.name and person.siyi_checked_for != person.name and siyi.configured():
            person.siyi_checked_for = person.name
            threading.Thread(target=self._siyi_lookup, args=(person,), daemon=True).start()

    def _summarize(self, person: Person) -> None:
        count = len(person.utterances)
        line = summarize(person)
        if not line:
            return
        with self.people.lock:
            person.summary, person.summarized_count = line, count
            sight = self.recent.get(person.id)
            if sight and time.time() - sight.at <= FOCUS_WINDOW_S:
                self.bubbles.show(person, sight.pos, self.head_pos, time.time())
        self.people.save()
        self.log(f"people: {person.label}: {line}")

    def _siyi_lookup(self, person: Person) -> None:
        info = siyi.lookup(person.name or "")
        if info:
            with self.people.lock:
                person.siyi = info
            self.people.save()

    # -- voice --------------------------------------------------------------

    def focus_person(self, now: float) -> Person | None:
        live = [s for s in self.recent.values() if now - s.at <= FOCUS_WINDOW_S]
        if not live:
            return None
        return max(live, key=lambda s: s.face.area).person

    def on_segment(self, seg: Segment) -> None:
        now = seg.ended_at
        self.log(f"heard: {seg.text}")
        with self.people.lock:
            owner = self.people.owner()
            speaker: Person | None = None
            if seg.voiceprint is not None:
                speaker, _ = self.people.match("voice", seg.voiceprint)
            introduced = parse_name(seg.text)
            focus = self.focus_person(now)
            if speaker is None and (owner is None or seg.voiceprint is None):
                speaker = focus
            if speaker is not None and speaker.is_owner:
                if introduced and introduced[0] == "other" and focus is not None:
                    self.rename(focus, introduced[1])
                return
            if speaker is None:
                speaker = focus
            if speaker is None:
                return
            speaker.say(seg.text, now)
            if seg.voiceprint is not None:
                speaker.add_embedding("voice", seg.voiceprint)
            if introduced:
                target = speaker if introduced[0] == "self" else focus
                if target is not None:
                    self.rename(target, introduced[1])
            self.enrich(speaker)
            if speaker.wants_summary():
                threading.Thread(target=self._summarize, args=(speaker,), daemon=True).start()
            sight = self.recent.get(speaker.id)
            if sight and now - sight.at <= FOCUS_WINDOW_S:
                self.bubbles.show(speaker, sight.pos, self.head_pos, now)
        self.people.save()

    def rename(self, person: Person, name: str) -> None:
        merged = self.people.name(person, name)
        if merged.id != person.id:
            self.bubbles.retire(person)
            sight = self.recent.pop(person.id, None)
            if sight:
                self.recent[merged.id] = Sighting(merged, sight.face, sight.pos, sight.at)
        self.log(f"people: {merged.id} is {name}")
        self.enrich(merged)

    # -- loop ---------------------------------------------------------------

    def run(self, once: bool = False) -> None:
        if self.ear:
            self.ear.start()
            self.log("people: listening")
        self.log(f"people: watching {self.frames} ({len(self.people.people)} known)")
        try:
            while True:
                n = self.step_frame()
                if once:
                    self.log(f"people: {n} face(s)")
                    return
                time.sleep(FRAME_POLL_S)
        finally:
            if self.ear:
                self.ear.stop()
            self.bubbles.close_all()


def enroll_owner(name: str, seconds: float = 6.0, log=print) -> Person:
    """Record the wearer for a few seconds so their own speech is never filed under a guest."""
    from .voice import Recorder
    from .voices import voiceprint

    rec = Recorder()
    log(f"people: recording you for {seconds:.0f} s, keep talking")
    rec.start()
    time.sleep(seconds)
    pcm = rec.stop()
    emb = voiceprint(pcm)
    if emb is None:
        raise SystemExit("people: not enough audio")
    people = People()
    with people.lock:
        p = people.owner() or people.by_name(name) or Person(id=uuid.uuid4().hex[:8])
        p.name, p.is_owner = name, True
        p.add_embedding("voice", emb)
        people.people[p.id] = p
    people.save()
    return p
