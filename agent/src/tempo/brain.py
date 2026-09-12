"""Gemini turns a request plus a room snapshot into spatial actions."""
from __future__ import annotations

import os

from google import genai
from google.genai import types

from .actions import DECLARATIONS, PIXELS_DECLARATIONS
from .perception import Snapshot

DEFAULT_MODEL = "gemini-3.6-flash"

SYSTEM = """You are Tempo, the agent inside a pair of spatial computing glasses.
You see what the wearer sees (the image) and you know the room's geometry (the scene JSON:
head position, detected surfaces with kinds and distances, and the panels already floating in the room).
The wearer talks to you. Answer by taking spatial actions with the tools, then call `say` once with a
short reply. Prefer putting information into the room (a note near the relevant object, an app panel
where they are looking) over just talking. Use the image to understand what objects are around and
what the wearer is pointing at or looking at. Keep notes short. Never invent panel handles; only use
handles listed in the scene. If the request needs no action, just call `say`."""


class Brain:
    def __init__(self, model: str | None = None, geometry: bool = True) -> None:
        self.geometry = geometry
        key = os.environ.get("GEMINI_API_KEY")
        if not key:
            raise SystemExit("GEMINI_API_KEY is not set (put it in the agent env file)")
        self.client = genai.Client(api_key=key)
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
