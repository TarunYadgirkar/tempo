"""The wire shape `hands-inject` goes out in."""

import json

import numpy as np
import pytest

from hands.control import MAX_LINE, ShellControl


def _hand(chirality, x):
    joints = np.zeros((21, 3))
    joints[:, 0] = x + np.arange(21) * 0.001
    joints[:, 1] = -0.1234567
    joints[:, 2] = -0.5
    return {"chirality": chirality, "confidence": 0.87, "joints": joints}


def test_one_hand_is_one_message_and_parses_back():
    payloads = ShellControl._payloads(1234, [_hand("left", 0.2)])
    assert len(payloads) == 1
    parsed = json.loads(payloads[0])
    assert parsed["t"] == 1234
    assert parsed["hands"][0]["chirality"] == "left"
    assert len(parsed["hands"][0]["joints"]) == 21
    assert parsed["hands"][0]["joints"][0] == [0.2, -0.1235, -0.5]


def test_two_hands_split_so_neither_message_overruns_the_line_cap():
    """The vendored wire layer drops a request line past ~1.1 kB, silently."""
    payloads = ShellControl._payloads(1, [_hand("left", 0.2), _hand("right", -0.2)])
    assert len(payloads) == 2
    for p in payloads:
        assert len(p) + len("hands-inject ") <= MAX_LINE
    assert {json.loads(p)["hands"][0]["chirality"] for p in payloads} == {
        "left",
        "right",
    }


def test_no_hands_is_an_explicit_empty_frame():
    payloads = ShellControl._payloads(9, [])
    assert payloads == ['{"t":9,"hands":[]}']


@pytest.mark.parametrize("n", [1, 2])
def test_every_message_stays_inside_the_cap(n):
    hands = [_hand("left", 0.2), _hand("right", -0.2)][:n]
    for p in ShellControl._payloads(1_700_000_000_000, hands):
        assert len(p) + len("hands-inject ") <= MAX_LINE
