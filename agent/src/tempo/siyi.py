"""Optional context from siyi, Tarun's personal CRM (Supabase REST).

Set TEMPO_SIYI_URL (the Supabase project URL) and TEMPO_SIYI_KEY (a service or
secret key) in agent/.env. Without them every lookup returns None.
"""
from __future__ import annotations

import json
import os
import urllib.parse
import urllib.request
from typing import Any

TIMEOUT_S = 3.0
NOTE_CHARS = 140


def configured() -> bool:
    return bool(os.environ.get("TEMPO_SIYI_URL") and os.environ.get("TEMPO_SIYI_KEY"))


def lookup(name: str) -> dict[str, Any] | None:
    """{"name", "note", "last_at", "last_note"} for the best name match, or None."""
    if not configured() or not name.strip():
        return None
    base = os.environ["TEMPO_SIYI_URL"].rstrip("/")
    key = os.environ["TEMPO_SIYI_KEY"]
    pattern = f"*{name.strip()}*"
    params = {
        "select": "full_name,preferred_name,general_notes,interactions(occurred_at,type,note)",
        "or": f"(full_name.ilike.{pattern},preferred_name.ilike.{pattern})",
        "interactions.order": "occurred_at.desc",
        "interactions.limit": "1",
        "limit": "1",
    }
    url = f"{base}/rest/v1/people?" + urllib.parse.urlencode(params)
    req = urllib.request.Request(url, headers={"apikey": key, "Authorization": f"Bearer {key}"})
    try:
        with urllib.request.urlopen(req, timeout=TIMEOUT_S) as r:
            rows = json.loads(r.read().decode())
    except Exception:
        return None
    if not rows:
        return None
    row = rows[0]
    last = (row.get("interactions") or [None])[0] or {}
    note = (row.get("general_notes") or "").strip().replace("\n", " ")
    return {
        "name": row.get("preferred_name") or row.get("full_name") or name,
        "note": note[:NOTE_CHARS],
        "last_at": (last.get("occurred_at") or "")[:10],
        "last_note": (last.get("note") or "").strip()[:NOTE_CHARS],
    }
