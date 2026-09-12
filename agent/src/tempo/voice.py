"""Push-to-talk capture from the Mac microphone as 16 kHz mono WAV bytes."""
from __future__ import annotations

import io
import threading
import wave

import numpy as np
import sounddevice as sd

RATE = 16000


def record_until_enter() -> bytes:
    chunks: list[np.ndarray] = []
    done = threading.Event()

    def on_audio(indata, frames, t, status):
        chunks.append(indata.copy())

    with sd.InputStream(samplerate=RATE, channels=1, dtype="int16", callback=on_audio):
        threading.Thread(target=lambda: (input("recording… Enter to stop "), done.set()), daemon=True).start()
        done.wait()
    pcm = np.concatenate(chunks) if chunks else np.zeros((0, 1), dtype="int16")
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(pcm.tobytes())
    return buf.getvalue()
