"""Open mic: voice-activity segments become Whisper text plus an ECAPA voiceprint.

The voiceprint model is the one Amelia's sidecar uses (SpeechBrain ECAPA-TDNN,
192-d, cosine), so the same enrol/match thresholds carry over.
"""
from __future__ import annotations

import os
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from .voice import RATE, transcribe

BLOCK = 480  # 30 ms at 16 kHz
MIN_SEGMENT_S = 0.8
MAX_SEGMENT_S = 12.0
SILENCE_END_S = 0.7
SPEECH_FLOOR_RMS = 250.0
SPEECH_OVER_NOISE = 3.0
VOICEPRINT_MIN_S = 1.0
# Amelia's attribution threshold for ECAPA cosine.
VOICE_MATCH = 0.6
ECAPA_DIR = Path(os.environ.get("TEMPO_ECAPA_DIR") or Path.home() / ".cache" / "tempo" / "ecapa")
# Whisper's favourite hallucinations on near-silence.
JUNK = {"", "you", "thank you.", "thanks for watching.", "bye.", "thank you", "."}


@dataclass(frozen=True)
class Segment:
    text: str
    voiceprint: np.ndarray | None
    seconds: float
    ended_at: float


_classifier = None
_classifier_lock = threading.Lock()


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
        elif self._speaking:
            self._chunks.append(block)
            self._silence_blocks += 1
            if self._silence_blocks * BLOCK / RATE >= SILENCE_END_S:
                self._finish()
        if self._speaking and len(self._chunks) * BLOCK / RATE >= MAX_SEGMENT_S:
            self._finish()

    def _finish(self) -> None:
        pcm = np.concatenate(self._chunks) if self._chunks else np.zeros(0, dtype="int16")
        self._chunks = []
        self._speaking = False
        self._silence_blocks = 0
        if len(pcm) / RATE >= MIN_SEGMENT_S:
            self._queue.put(pcm)

    def _worker(self) -> None:
        while True:
            pcm = self._queue.get()
            try:
                text = transcribe(pcm).strip()
                if text.lower() in JUNK or len(text) < 2:
                    continue
                self._on_segment(Segment(text, voiceprint(pcm), len(pcm) / RATE, time.time()))
            except Exception as exc:  # a bad segment must not kill the ear
                self._log(f"voices: {exc}")
