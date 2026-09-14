"""Open mic: voice-activity segments become Whisper text plus an ECAPA voiceprint.

The voiceprint model is the one Amelia's sidecar uses (SpeechBrain ECAPA-TDNN,
192-d, cosine), so the same enrol/match thresholds carry over.
"""
from __future__ import annotations

import os
import queue
import re
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from .voice import RATE, transcribe

BLOCK = 480  # 30 ms at 16 kHz
MIN_SEGMENT_S = 0.8        # whole segment (speech + trailing silence) must be this long
MIN_SPEECH_S = 0.4        # ...and contain at least this much above-threshold speech,
                          # so a sub-second noise burst never reaches Whisper
MAX_SEGMENT_S = 12.0
SILENCE_END_S = 0.7
# Raised from 320: the MacBook's built-in mic sits at ~150-300 RMS on room tone
# and keyboard/fan noise, which used to clear the old floor and feed Whisper
# near-silence (its favourite hallucination regime). Real speech is well above.
SPEECH_FLOOR_RMS = 520.0
SPEECH_OVER_NOISE = 3.0
VOICEPRINT_MIN_S = 1.0
# Amelia's attribution threshold for ECAPA cosine.
VOICE_MATCH = 0.6
ECAPA_DIR = Path(os.environ.get("TEMPO_ECAPA_DIR") or Path.home() / ".cache" / "tempo" / "ecapa")
# Whisper's favourite hallucinations on near-silence / short bursts.
JUNK = {"", "you", "thank you.", "thanks for watching.", "bye.", "thank you", ".",
        "yeah.", "yes.", "no.", "ok.", "okay.", "sure.", "sorry.", "right.", "wow.",
        "hmm.", "mm-hmm.", "uh-huh.", "laughs", "music", "[music]", "(music)", "?"}
# Substrings Whisper emits when it invents content on noise (YouTube-style burn-in,
# numeric timer readback, subtitle credits). Drop these before they can be filed
# or enrolled as a name.
HALLUCINATION_PHRASES = (
    "subscribe", "dot com", ".com", "thanks for watching", "thank you for watching",
    "please subscribe", "subtitles by", "subtitled by", "amara", "caption",
)


@dataclass(frozen=True)
class Segment:
    text: str
    voiceprint: np.ndarray | None
    seconds: float
    ended_at: float


_classifier = None
_classifier_lock = threading.Lock()


def _is_garbage(text: str) -> bool:
    """True if a transcript is a Whisper hallucination on noise, not real speech.

    These must be dropped before they reach the people layer, so a garbage line
    is never filed as an utterance and never mined for an "I'm X" name.
    """
    t = text.strip()
    if len(t) < 3:
        return True
    low = t.lower()
    if low in JUNK:
        return True
    for phrase in HALLUCINATION_PHRASES:
        if phrase in low:
            return True
    # All-numeric / all-punctuation: "218 00 00 00", "...", "code 218 00".
    letters = re.sub(r"[^A-Za-z]", "", t)
    if len(letters) < 3:
        return True
    # Mostly numeric (timer / coordinate readback): "code 218 00 00 00 00".
    digits = sum(c.isdigit() for c in t)
    if digits and digits >= len(letters):
        return True
    # One word repeated: "fold fold fold", "ok ok ok", "uh uh", or one word
    # dominating a short transcript ("shoot fold fold fold fold").
    words = re.findall(r"[A-Za-z']+", low)
    if len(words) >= 2:
        from collections import Counter

        counts = Counter(words)
        top, n = counts.most_common(1)[0]
        if len(counts) == 1 or (n >= 3 and n >= len(words) * 0.6):
            return True
    return False


def voiceprint(pcm: np.ndarray) -> np.ndarray | None:
    """192-d L2-normalized ECAPA embedding of 16 kHz int16 audio, or None when too short."""
    global _classifier
    if len(pcm) < RATE * VOICEPRINT_MIN_S:
        return None
    import torch

    with _classifier_lock:
        if _classifier is None:
            from speechbrain.inference.speaker import EncoderClassifier

            _classifier = EncoderClassifier.from_hparams(
                source="speechbrain/spkrec-ecapa-voxceleb", savedir=str(ECAPA_DIR), run_opts={"device": "cpu"}
            )
        wav = torch.from_numpy(pcm.astype("float32") / 32768.0)[None, :]
        with torch.no_grad():
            emb = _classifier.encode_batch(wav).squeeze().cpu().numpy().astype(np.float32)
    norm = float(np.linalg.norm(emb))
    return emb / norm if norm > 0 else None


class Ear:
    """Listens continuously; calls `on_segment` from a worker thread per utterance."""

    def __init__(self, on_segment: Callable[[Segment], None], log=print) -> None:
        self._on_segment = on_segment
        self._log = log
        self._queue: queue.Queue[np.ndarray] = queue.Queue()
        self._noise = SPEECH_FLOOR_RMS
        self._chunks: list[np.ndarray] = []
        self._speaking = False
        self._silence_blocks = 0
        self._speech_blocks = 0  # blocks actually above the speech threshold
        self._stream = None

    def start(self) -> None:
        import sounddevice as sd

        threading.Thread(target=self._worker, daemon=True).start()
        self._stream = sd.InputStream(samplerate=RATE, channels=1, dtype="int16", blocksize=BLOCK, callback=self._on_audio)
        self._stream.start()

    def stop(self) -> None:
        if self._stream:
            self._stream.stop()
            self._stream.close()

    def _on_audio(self, indata, frames, t, status) -> None:
        block = indata[:, 0].copy()
        rms = float(np.sqrt(np.mean(block.astype("float32") ** 2)))
        threshold = max(SPEECH_FLOOR_RMS, self._noise * SPEECH_OVER_NOISE)
        if rms < threshold:
            self._noise = 0.98 * self._noise + 0.02 * rms
        if rms >= threshold:
            self._speaking = True
            self._silence_blocks = 0
            self._chunks.append(block)
            self._speech_blocks += 1
        elif self._speaking:
            self._chunks.append(block)
            self._silence_blocks += 1
            if self._silence_blocks * BLOCK / RATE >= SILENCE_END_S:
                self._finish()
        if self._speaking and len(self._chunks) * BLOCK / RATE >= MAX_SEGMENT_S:
            self._finish()

    def _finish(self) -> None:
        pcm = np.concatenate(self._chunks) if self._chunks else np.zeros(0, dtype="int16")
        speech_s = self._speech_blocks * BLOCK / RATE
        self._chunks = []
        self._speaking = False
        self._silence_blocks = 0
        self._speech_blocks = 0
        # Need both a long-enough segment and enough real speech; a noise burst that
        # happens to span MIN_SEGMENT_S but is mostly silence is rejected here.
        if len(pcm) / RATE >= MIN_SEGMENT_S and speech_s >= MIN_SPEECH_S:
            self._queue.put(pcm)

    def _worker(self) -> None:
        while True:
            pcm = self._queue.get()
            try:
                text = transcribe(pcm).strip()
                if _is_garbage(text):
                    continue
                self._on_segment(Segment(text, voiceprint(pcm), len(pcm) / RATE, time.time()))
            except Exception as exc:  # a bad segment must not kill the ear
                self._log(f"voices: {exc}")
