"""Gemini turns a request plus a room snapshot into spatial actions."""
from __future__ import annotations

import os

from google import genai
from google.genai import types

from .actions import DECLARATIONS, PIXELS_DECLARATIONS
from .perception import Snapshot

DEFAULT_MODEL = "gemini-3.6-flash"

# Vertex AI opt-in. The API-key path stays the default; Vertex is used only when
# TEMPO_USE_VERTEX=1. On any Vertex construction failure we log one line and fall
# back to the API-key client — never crash. The Google Cloud project below is
# not secret (it is already published in context/EVENT.md) and is hard-deleted
# Mon Sep 14 9 AM PT, which is exactly why Vertex must stay opt-in.
DEFAULT_VERTEX_PROJECT = "eastwest72hack26bos-505"
DEFAULT_VERTEX_LOCATION = "us-central1"

# One module-level client. A temporary `genai.Client` gets closed mid-call by
# the SDK (see NEXT-SESSION.md `people.summarize` gotcha), so we cache a single
# instance for the process.
_client: genai.Client | None = None


def _build_client() -> genai.Client:
    """Return the process-wide Gemini client, building it on first use.

    Order:
      1. Require GEMINI_API_KEY (needed for the default path and the Vertex fallback).
      2. If TEMPO_USE_VERTEX=1, try a Vertex AI client (ADC, no API key) with
         project/location overrides; on any failure, log one line and fall back.
      3. Otherwise (or after a Vertex fallback) build the API-key client.
    """
    global _client
    if _client is not None:
        return _client

    key = os.environ.get("GEMINI_API_KEY")
    if not key:
        raise SystemExit("GEMINI_API_KEY is not set (put it in the agent env file)")

    if os.environ.get("TEMPO_USE_VERTEX") == "1":
        project = os.environ.get("TEMPO_VERTEX_PROJECT", DEFAULT_VERTEX_PROJECT)
        location = os.environ.get("TEMPO_VERTEX_LOCATION", DEFAULT_VERTEX_LOCATION)
        try:
            _client = genai.Client(vertexai=True, project=project, location=location)
            return _client
        except Exception as exc:  # noqa: BLE001 - any Vertex failure must fall back
            print(
                f"[tempo] Vertex AI unavailable ({project}/{location}), "
                f"falling back to API-key client: {exc}",
                flush=True,
            )

    _client = genai.Client(api_key=key)
    return _client


def _reset_client() -> None:
    """Drop the cached client (tests only: lets each test start clean)."""
    global _client
    _client = None


SYSTEM = """You are Tempo, the agent inside a pair of spatial computing glasses.
You see what the wearer sees (the image) and you know the room's geometry (the scene JSON:
head position, detected surfaces with kinds and distances, and the panels already floating in the room).
The wearer talks to you. Answer by taking spatial actions with the tools, then call `say` once with a
short reply. Prefer putting information into the room (a note near the relevant object, an app panel
where they are looking) over just talking. Use the image to understand what objects are around and
what the wearer is pointing at or looking at. Keep notes short. Never invent panel handles; only use
handles listed in the scene. The scene lists people: who is in view right now (name if known, what they last said, notes from the wearer's contacts) and who the system knows. Answer "who is this" and "what did we talk about" from that; if someone is unnamed, say so and suggest the wearer introduce them. The scene also lists remembered_places: named spots the wearer saved earlier,
with their direction from where the wearer stands now; use recall_place when they ask where something is
and remember_place when they tell you where something lives. If the request needs no action, just call `say`."""


class Brain:
    def __init__(self, model: str | None = None, geometry: bool = True) -> None:
        self.geometry = geometry
        self.client = _build_client()
        self.model = model or os.environ.get("TEMPO_MODEL", DEFAULT_MODEL)

    def decide(self, request: str, snap: Snapshot, audio: bytes | None = None) -> tuple[list[types.FunctionCall], str]:
        parts: list[types.Part] = [
            types.Part.from_bytes(data=snap.png, mime_type="image/jpeg"),
        ]
        if self.geometry:
            parts.append(types.Part.from_text(text="Scene:\n" + snap.scene_text()))
        else:
            parts.append(types.Part.from_text(text="No geometry is available. Estimate distances and surfaces from the image and place things by metre offsets."))
        if audio:
            parts.append(types.Part.from_bytes(data=audio, mime_type="audio/wav"))
            parts.append(types.Part.from_text(text="The wearer said the audio above."))
        else:
            parts.append(types.Part.from_text(text="Wearer: " + request))
        resp = self.client.models.generate_content(
            model=self.model,
            contents=[types.Content(role="user", parts=parts)],
            config=types.GenerateContentConfig(
                system_instruction=SYSTEM,
                tools=[types.Tool(function_declarations=DECLARATIONS if self.geometry else PIXELS_DECLARATIONS)],
                temperature=0.2,
            ),
        )
        parts = resp.candidates[0].content.parts if resp.candidates and resp.candidates[0].content else []
        calls = [p.function_call for p in parts if p.function_call]
        text = " ".join(p.text for p in parts if p.text)
        return calls, text
