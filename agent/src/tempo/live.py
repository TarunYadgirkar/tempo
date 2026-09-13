"""Pinch-and-talk: hold a pinch to record, release to run the request. Enter works as a fallback."""
from __future__ import annotations

import subprocess
import threading
import time

from . import perception, voice
from .actions import Executor
from .brain import Brain
from .memory import Memory
from .shell import Shell

HOLD_TO_TALK_S = 0.35


class Live:
    def __init__(self, model: str | None, speak: bool) -> None:
        self.cmd = Shell()
        self.brain = Brain(model)
        self.memory = Memory()
        self.speak = speak
        self.rec = voice.Recorder()
        self.recording = False
        self.pinch_started: float | None = None
        self.lock = threading.Lock()

    def say(self, text: str) -> None:
        print("tempo:", text)
        if self.speak:
            subprocess.Popen(["say", "-r", "190", text])

    def start_recording(self) -> None:
        with self.lock:
            if self.recording:
                return
            self.recording = True
        self.rec.start()
        print("● listening")

    def stop_and_handle(self) -> None:
        with self.lock:
            if not self.recording:
                return
            self.recording = False
        pcm = self.rec.stop()
        t0 = time.time()
        text = voice.transcribe(pcm)
        t1 = time.time()
        if not text:
            print("  (heard nothing)")
            return
        print(f"you: {text}   [whisper {t1 - t0:.2f}s]")
        self.handle(text)

    def handle(self, text: str) -> None:
        t0 = time.time()
        snap = perception.take(self.cmd)
        calls, reply = self.brain.decide(text, snap)
        t1 = time.time()
        outcome = Executor(self.cmd, snap, self.memory).run(calls)
        for line in outcome.log:
            print("  ·", line)
        self.say(outcome.spoken or reply.strip() or "Done.")
        print(f"  [gemini {t1 - t0:.2f}s · {len(calls)} actions]")

    def watch_pinch(self) -> None:
        events = Shell()
        for ev in events.events():
            if ev.startswith("pinch phase=begin"):
                self.pinch_started = time.time()
                threading.Timer(HOLD_TO_TALK_S, self._maybe_start).start()
            elif ev.startswith("pinch phase=end"):
                held = time.time() - (self.pinch_started or time.time())
                self.pinch_started = None
                if held >= HOLD_TO_TALK_S:
                    threading.Thread(target=self.stop_and_handle, daemon=True).start()

    def _maybe_start(self) -> None:
        if self.pinch_started is not None:
            self.start_recording()

    def run(self) -> None:
        threading.Thread(target=self.watch_pinch, daemon=True).start()
        self.say("Tempo is listening. Hold a pinch to talk.")
        while True:
            typed = input("(Enter = talk, or type a request) ").strip()
            if typed:
                self.handle(typed)
                continue
            if self.recording:
                self.stop_and_handle()
            else:
                self.start_recording()
                input("recording… Enter to stop ")
                self.stop_and_handle()
