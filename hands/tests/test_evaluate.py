"""The jitter metric, which is the number the whole project is judged on."""

import numpy as np

from hands.evaluate import STILL_RADIUS_M, SourceLog, analyse


def _hand(x=0.2, y=-0.1, z=-0.5):
    j = np.zeros((21, 3))
    j[:, 0] = x + np.arange(21) * 0.001
    j[:, 1] = y
    j[:, 2] = z
    return j


def _log(name, n, noise_m, rng, drop_every=0):
    log = SourceLog(name)
    for i in range(n):
        if drop_every and i % drop_every == 0:
            log.add(i / 30.0, None)
            continue
        log.add(i / 30.0, _hand() + rng.normal(0.0, noise_m, (21, 3)))
    return log


def test_a_noisier_source_reports_more_jitter():
    rng = np.random.default_rng(1)
    steady = analyse(_log("mac", 120, 0.0005, rng))
    shaky = analyse(_log("phone", 120, 0.005, rng))
    assert steady["jitter_mm_mean"] < shaky["jitter_mm_mean"]
    # 0.5 mm of per-axis noise is under 2 mm of frame-to-frame displacement.
    assert steady["jitter_mm_mean"] < 2.0
    assert shaky["jitter_mm_mean"] > 5.0


def test_a_perfectly_still_hand_has_no_jitter():
    log = SourceLog("mac")
    for i in range(60):
        log.add(i / 30.0, _hand())
    out = analyse(log)
    assert out["jitter_mm_mean"] == 0.0
    assert out["detect_rate"] == 1.0
    assert out["still_frame_pairs"] > 0


def test_detect_rate_counts_the_frames_the_hand_was_missing():
    rng = np.random.default_rng(2)
    out = analyse(_log("phone", 100, 0.001, rng, drop_every=4))
    assert 0.7 < out["detect_rate"] < 0.8


def test_gross_motion_is_excluded_from_jitter():
    """A hand being moved should not be charged for moving."""
    log = SourceLog("mac")
    for i in range(90):
        log.add(i / 30.0, _hand(x=0.2 + i * 0.01))  # 30 cm/s sweep
    out = analyse(log)
    # Either no still stretch qualified, or the ones that did are short.
    assert out["still_frame_pairs"] * 0.01 < 90 * STILL_RADIUS_M


def test_depth_validity_is_reported_per_fingertip():
    log = SourceLog("mac")
    valid = np.ones(21, dtype=bool)
    valid[[4, 8, 12, 16, 20]] = False  # every fingertip missed
    for i in range(30):
        log.add(i / 30.0, _hand(), valid)
    out = analyse(log)
    assert out["depth_valid_rate_fingertips"] == 0.0
    assert out["depth_valid_rate_all"] == round(16 / 21, 4)
