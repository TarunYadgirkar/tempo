import math

from hands.calibrate import derive, features


def hand(thumb_index_m: float, curl: float):
    # 21 joints; fingers laid along +x from their MCP, tip folded back by `curl` * pi
    j = [[0.0, 0.0, 0.0, 1.0] for _ in range(21)]
    for mcp, pip, dip, tip in ((5, 6, 7, 8), (9, 10, 11, 12), (13, 14, 15, 16), (17, 18, 19, 20)):
        base = [0.0, -0.02 * mcp, 0.0]
        j[mcp] = base + [1.0]
        j[pip] = [base[0] + 0.03, base[1], base[2], 1.0]
        ang = curl * math.pi
        j[tip] = [base[0] + 0.06 * math.cos(ang), base[1], base[2] + 0.06 * math.sin(ang), 1.0]
        j[dip] = j[tip]
    j[4] = [j[8][0], j[8][1] - thumb_index_m, j[8][2], 1.0]
    return j


def test_features_match_engine_definitions():
    f = features(hand(0.05, 0.0))
    assert abs(f["thumb_index_distance"] - 0.05) < 1e-6
    assert f["all_fingers_curl"] < 0.05
    assert features(hand(0.01, 0.9))["all_fingers_curl"] > 0.85


def test_derive_puts_triggers_between_the_clusters():
    samples = {
        "open": [features(hand(0.09 + 0.002 * (i % 5), 0.05)) for i in range(40)],
        "pinch": [features(hand(0.015 + 0.002 * (i % 5), 0.15)) for i in range(40)],
        "fist": [features(hand(0.03, 0.85 + 0.01 * (i % 3))) for i in range(40)],
    }
    th = derive(samples)
    assert 0.023 < th.pinch_trigger_m < 0.06
    assert th.pinch_release_m > th.pinch_trigger_m
    assert 0.25 <= th.fist_all_curl <= 0.6
    assert th.fist_release < th.fist_all_curl
    assert "[pinch_select.standard]" in th.toml() and "[fist_launcher.windowed]" in th.toml()
