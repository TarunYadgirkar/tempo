"""The jitter metric and the windowing it is measured over.

Two things are being pinned. The metric itself: a noisier source must score
worse, and a source must not be charged for motion the user made. And the
windowing, which is what the first version of this got wrong — the phone
column was contaminated by the Mac's own joints, and "still" was judged on the
same track being scored, so each source was measured over its own moments.
"""

import numpy as np
import pytest

from hands.evaluate import (
    BRIDGE_SLACK_S,
    PINCH_MM,
    SourceLog,
    analyse,
    frame_interval_s,
    still_intervals,
)

HZ = 12.0


def _hand(x=0.2, y=-0.1, z=-0.5, pinch_m=0.06):
    j = np.zeros((21, 3))
    j[:, 0] = x + np.arange(21) * 0.001
    j[:, 1] = y
    j[:, 2] = z
    j[4] = [x, y, z]  # thumb tip
    j[8] = [x + pinch_m, y, z]  # index tip, pinch_m away
    return j


def _log(name, n, noise_m, rng, drop_every=0, hz=HZ, **hand):
    log = SourceLog(name)
    for i in range(n):
        if drop_every and i % drop_every == 0:
            log.add(i / hz, None)
            continue
        log.add(i / hz, _hand(**hand) + rng.normal(0.0, noise_m, (21, 3)))
    return log


def _all_still(log):
    return still_intervals(log.samples)


# --- the metric ------------------------------------------------------------


def test_a_noisier_source_reports_more_jitter():
    rng = np.random.default_rng(1)
    steady = _log("mac", 120, 0.0005, rng)
    shaky = _log("phone", 120, 0.005, rng)
    reference = _log("ref", 120, 0.0, rng)
    intervals = _all_still(reference)
    a = analyse(steady, intervals)
    b = analyse(shaky, intervals)
    assert a["jitter_mm_mean"] < b["jitter_mm_mean"]
    assert a["jitter_mm_mean"] < 2.0
    assert b["jitter_mm_mean"] > 5.0


def test_a_perfectly_still_hand_has_no_jitter():
    log = SourceLog("mac")
    for i in range(60):
        log.add(i / HZ, _hand())
    out = analyse(log, _all_still(log))
    assert out["jitter_mm_mean"] == 0.0
    assert out["detect_rate"] == 1.0
    assert out["still_frame_pairs"] > 0


def test_detect_rate_counts_the_frames_the_hand_was_missing():
    rng = np.random.default_rng(2)
    log = _log("phone", 100, 0.001, rng, drop_every=4)
    assert 0.7 < analyse(log, _all_still(log))["detect_rate"] < 0.8


def test_gross_motion_is_excluded_from_jitter():
    """A hand being moved should not be charged for moving."""
    log = SourceLog("mac")
    for i in range(90):
        log.add(i / HZ, _hand(x=0.2 + i * 0.025))  # 30 cm/s sweep
    assert still_intervals(log.samples) == []
    assert analyse(log, [])["still_frame_pairs"] == 0


def test_depth_validity_is_reported_per_fingertip():
    log = SourceLog("mac")
    valid = np.ones(21, dtype=bool)
    valid[[4, 8, 12, 16, 20]] = False  # every fingertip missed
    for i in range(30):
        log.add(i / HZ, _hand(), valid)
    out = analyse(log, _all_still(log))
    assert out["depth_valid_rate_fingertips"] == 0.0
    assert out["depth_valid_rate_all"] == round(16 / 21, 4)


def test_a_held_frame_is_not_credited_with_being_steady():
    """A repeated frame moves zero millimetres without tracking anything."""
    rng = np.random.default_rng(3)
    log = SourceLog("mac")
    for i in range(60):
        log.add(i / HZ, _hand() + rng.normal(0.0, 0.004, (21, 3)))
    for i in range(60, 120):  # the same joints, over and over
        log.add(i / HZ, log.samples[59].joints, held=True)
    out = analyse(log, _all_still(log))
    assert out["frames_held"] == 60
    assert out["detect_rate"] == 0.5
    assert out["delivered_rate"] == 1.0
    assert out["jitter_mm_mean"] > 3.0  # not diluted toward zero by the holds


# --- stillness, measured on a track that is not the one being scored -------


def test_stillness_is_a_speed_not_a_box():
    """A slow drift across 2 cm is not still; a fast twitch inside it is."""
    drift = SourceLog("ref")
    twitch = SourceLog("ref")
    rng = np.random.default_rng(4)
    for i in range(120):
        drift.add(i / HZ, _hand(x=0.2 + i * 0.0004))  # 4.8 mm/s... still
        twitch.add(i / HZ, _hand() + rng.normal(0.0, 0.004, (21, 3)))
    assert still_intervals(drift.samples)
    fast = SourceLog("ref")
    for i in range(120):
        fast.add(i / HZ, _hand(x=0.2 + i * 0.005))  # 60 mm/s
    assert still_intervals(fast.samples) == []
    # Per-frame noise inside a 2 cm box still reads as a still hand, because
    # the speed is measured over half a second rather than one frame pair.
    assert still_intervals(twitch.samples)


def test_the_reference_track_scores_both_columns_over_the_same_moments():
    reference = SourceLog("phone")
    noisy = SourceLog("mac")
    rng = np.random.default_rng(5)
    for i in range(120):
        t = i / HZ
        moving = i >= 60
        x = 0.2 + (i - 60) * 0.02 if moving else 0.2
        reference.add(t, _hand(x=x))
        noisy.add(t, _hand(x=x) + rng.normal(0.0, 0.004, (21, 3)))
    intervals = still_intervals(reference.samples)
    assert intervals and intervals[0][1] < 60 / HZ + 0.5
    # The noisy column is scored only where the REFERENCE says the user was
    # still, so its own noise cannot buy or lose it scoring time.
    scored = analyse(noisy, intervals)["still_frame_pairs"]
    assert 0 < scored < 60


def test_a_still_stretch_is_bridged_across_the_window_the_reference_is_absent_for():
    """The phone only exists in the off windows; the mac is scored in the on
    windows between them, so the mask has to carry across."""
    reference = SourceLog("phone")
    window = 5.0
    for i in range(int(20 * HZ)):
        t = i / HZ
        if int(t // window) % 2 == 1:
            continue  # injection on: the dump is the mac, not the phone
        reference.add(t, _hand())
    bridged = still_intervals(reference.samples, bridge_s=window + BRIDGE_SLACK_S)
    assert len(bridged) == 1
    assert bridged[0][1] - bridged[0][0] > 14.0
    # Without the bridge the same track is one stretch per off window, and
    # the on windows between them would be scored over nothing at all.
    assert len(still_intervals(reference.samples)) == 2


def test_an_absent_reference_scores_nothing_rather_than_everything():
    log = SourceLog("mac")
    for i in range(60):
        log.add(i / HZ, _hand())
    assert analyse(log, [])["jitter_mm_mean"] is None


# --- the pinch margin ------------------------------------------------------


def test_the_pinch_distribution_reports_where_a_resting_hand_sits():
    rng = np.random.default_rng(6)
    log = SourceLog("mac")
    for i in range(200):
        log.add(i / HZ, _hand(pinch_m=0.06) + rng.normal(0.0, 0.001, (21, 3)))
    pinch = analyse(log, _all_still(log))["pinch"]
    assert pinch["p10_mm"] < pinch["p50_mm"] < pinch["p90_mm"]
    assert pinch["p50_mm"] == pytest.approx(60.0, abs=3.0)
    assert pinch["frac_under_25mm"] == 0.0


def test_a_source_that_pinches_by_itself_is_caught():
    """The user holds their fingers 3 cm apart; the source often says 2."""
    rng = np.random.default_rng(7)
    log = SourceLog("mac")
    for i in range(200):
        log.add(i / HZ, _hand(pinch_m=0.03) + rng.normal(0.0, 0.012, (21, 3)))
    pinch = analyse(log, _all_still(log))["pinch"]
    assert pinch["p10_mm"] < PINCH_MM
    assert pinch["frac_under_25mm"] > 0.05


def test_no_still_frames_means_no_pinch_numbers_rather_than_zeros():
    log = SourceLog("mac")
    for i in range(30):
        log.add(i / HZ, _hand())
    assert analyse(log, [])["pinch"] == {"samples": 0}


# --- the rate the pipeline actually ran at ---------------------------------


def test_the_frame_interval_is_measured_rather_than_assumed():
    log = SourceLog("mac")
    for i in range(50):
        log.add(i / HZ, _hand())
    assert frame_interval_s(log) == pytest.approx(1 / HZ, abs=1e-9)
    assert frame_interval_s(SourceLog("mac")) is None
