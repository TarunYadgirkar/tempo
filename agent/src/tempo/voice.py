"""Microphone capture and local speech to text (mlx-whisper, runs on the Mac's GPU)."""
from __future__ import annotations

import io
import threading
import wave

import numpy as np
import sounddevice as sd

RATE = 16000
WHISPER_MODEL = "mlx-community/whisper-large-v3-turbo"


class Recorder:
    """Start/stop capture from any thread; returns 16 kHz mono int16 samples."""

    def __init__(self) -> None:
        self._chunks: list[np.ndarray] = []
        self._stream: sd.InputStream | None = None

    def start(self) -> None:
        self._chunks = []
        self._stream = sd.InputStream(samplerate=RATE, channels=1, dtype="int16", callback=self._on_audio)
        self._stream.start()

    def _on_audio(self, indata, frames, t, status) -> None:
        self._chunks.append(indata.copy())

    def stop(self) -> np.ndarray:
        if self._stream:
            self._stream.stop()
            self._stream.close()
            self._stream = None
        return np.concatenate(self._chunks)[:, 0] if self._chunks else np.zeros(0, dtype="int16")


def record_until_enter() -> np.ndarray:
    rec = Recorder()
    rec.start()
    done = threading.Event()
    threading.Thread(target=lambda: (input("recording… Enter to stop "), done.set()), daemon=True).start()
    done.wait()
    return rec.stop()


def to_wav(pcm: np.ndarray) -> bytes:
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(pcm.astype("int16").tobytes())
    return buf.getvalue()


def transcribe(pcm: np.ndarray) -> str:
    import mlx_whisper

    if len(pcm) < RATE // 4:
        return ""
    audio = pcm.astype("float32") / 32768.0
    result = mlx_whisper.transcribe(audio, path_or_hf_repo=WHISPER_MODEL, language="en", fp16=True)
    return result["text"].strip()
