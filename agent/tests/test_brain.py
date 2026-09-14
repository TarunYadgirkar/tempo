"""Tests for the opt-in Vertex AI inference path in brain.py.

These tests mock `genai.Client` so no network is used. They cover:
  (a) TEMPO_USE_VERTEX unset  -> API-key client used.
  (b) TEMPO_USE_VERTEX=1 + successful Vertex construction -> Vertex client used.
  (c) TEMPO_USE_VERTEX=1 + Vertex construction raises -> falls back to API-key
      client, no crash.
"""
from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from tempo import brain


@pytest.fixture(autouse=True)
def _clean_client(monkeypatch):
    """Start each test with no cached client and a fresh call log."""
    brain._reset_client()
    yield
    brain._reset_client()


def _install_fake_genai(monkeypatch, call_log: list[dict]) -> MagicMock:
    """Replace `brain.genai` with a mock whose `Client` records its kwargs."""
    fake_genai = MagicMock()

    def _client(**kwargs):
        call_log.append(kwargs)
        # Vertex failures are simulated by raising from the side_effect set below.
        return MagicMock(name="genai.Client")

    fake_genai.Client.side_effect = _client
    monkeypatch.setattr(brain, "genai", fake_genai)
    return fake_genai


def test_default_uses_api_key_client(monkeypatch):
    monkeypatch.setenv("GEMINI_API_KEY", "fake-key")
    monkeypatch.delenv("TEMPO_USE_VERTEX", raising=False)

    call_log: list[dict] = []
    _install_fake_genai(monkeypatch, call_log)

    brain.Brain()

    assert len(call_log) == 1
    assert call_log[0] == {"api_key": "fake-key"}
    assert "vertexai" not in call_log[0]


def test_vertex_opt_in_uses_vertex_client(monkeypatch):
    monkeypatch.setenv("GEMINI_API_KEY", "fake-key")
    monkeypatch.setenv("TEMPO_USE_VERTEX", "1")
    monkeypatch.setenv("TEMPO_VERTEX_PROJECT", "my-project")
    monkeypatch.setenv("TEMPO_VERTEX_LOCATION", "my-location")

    call_log: list[dict] = []
    _install_fake_genai(monkeypatch, call_log)

    brain.Brain()

    assert len(call_log) == 1
    assert call_log[0] == {
        "vertexai": True,
        "project": "my-project",
        "location": "my-location",
    }


def test_vertex_failure_falls_back_to_api_key(monkeypatch, capsys):
    monkeypatch.setenv("GEMINI_API_KEY", "fake-key")
    monkeypatch.setenv("TEMPO_USE_VERTEX", "1")
    # No project/location overrides -> defaults used.
    monkeypatch.delenv("TEMPO_VERTEX_PROJECT", raising=False)
    monkeypatch.delenv("TEMPO_VERTEX_LOCATION", raising=False)

    call_log: list[dict] = []
    fake_genai = _install_fake_genai(monkeypatch, call_log)

    # First call (Vertex) raises; second call (API key) succeeds.
    def _client(**kwargs):
        call_log.append(kwargs)
        if kwargs.get("vertexai"):
            raise RuntimeError("vertex down")
        return MagicMock(name="genai.Client")

    fake_genai.Client.side_effect = _client

    # Must not raise.
    brain.Brain()

    # Two construction attempts: Vertex first, then API key.
    assert len(call_log) == 2
    assert call_log[0] == {
        "vertexai": True,
        "project": brain.DEFAULT_VERTEX_PROJECT,
        "location": brain.DEFAULT_VERTEX_LOCATION,
    }
    assert call_log[1] == {"api_key": "fake-key"}

    # Exactly one fallback log line, no crash.
    out = capsys.readouterr().out
    assert out.count("Vertex AI unavailable") == 1
    assert "vertex down" in out


def test_missing_api_key_raises_systemexit(monkeypatch):
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    monkeypatch.delenv("TEMPO_USE_VERTEX", raising=False)

    _install_fake_genai(monkeypatch, [])

    with pytest.raises(SystemExit):
        brain.Brain()
