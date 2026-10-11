"""Gemini model used by every agent call; TEMPO_MODEL overrides it."""
import os

GEMINI_MODEL = "gemini-3.8-flash"


def gemini_model() -> str:
    return os.environ.get("TEMPO_MODEL", GEMINI_MODEL)
