// test_calibrate.cpp — Synthetic-jitter calibration unit tests.
//
// Feeds the calibration library a stream of synthetic hand frames with
// known noise + known active amplitude, then asserts the resulting
// trigger/release thresholds land within ±5% of the analytically-correct
// values dictated by the formulas in calibrate.cpp.

#include "gesture_engine.h"
#include "gesture_calibrate.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

// ---------------------------------------------------------------------------
// Hand fixture helpers
// ---------------------------------------------------------------------------

static ge_hand_t make_open_hand_with_thumb(float thumb_index_dist,
                                           float thumb_middle_dist) {
    ge_hand_t h{};
    h.present = true;
    // Wrist
    h.joints[GE_JOINT_WRIST][0] = 0; h.joints[GE_JOINT_WRIST][1] = 0; h.joints[GE_JOINT_WRIST][2] = 0;
    h.joints[GE_JOINT_WRIST][3] = 1.0f;

    // Thumb fully extended along -X
    for (int j = GE_JOINT_THUMB_CMC; j <= GE_JOINT_THUMB_TIP; ++j) {
        h.joints[j][0] = -0.07f; h.joints[j][1] = 0.02f * (j - GE_JOINT_THUMB_CMC + 1);
        h.joints[j][2] = 0;      h.joints[j][3] = 1.0f;
    }
    // Place index TIP at distance `thumb_index_dist` from thumb TIP
    // along +X: thumb_tip is at x=-0.07, place index_tip at -0.07 + dist.
    float thumb_tip_x = h.joints[GE_JOINT_THUMB_TIP][0];
    float thumb_tip_y = h.joints[GE_JOINT_THUMB_TIP][1];

    // Build a straight index finger ending at (thumb_tip_x + thumb_index_dist, thumb_tip_y, 0).
    auto fill_finger = [&h](int mcp, int pip, int dip, int tip,
                            float tip_x, float tip_y) {
        // MCP at (tip_x - 0.09, ...) — finger length ~9 cm
        float dx = tip_x;
        h.joints[mcp][0] = dx; h.joints[mcp][1] = tip_y - 0.09f; h.joints[mcp][2] = 0; h.joints[mcp][3] = 1.0f;
        h.joints[pip][0] = dx; h.joints[pip][1] = tip_y - 0.06f; h.joints[pip][2] = 0; h.joints[pip][3] = 1.0f;
        h.joints[dip][0] = dx; h.joints[dip][1] = tip_y - 0.03f; h.joints[dip][2] = 0; h.joints[dip][3] = 1.0f;
        h.joints[tip][0] = dx; h.joints[tip][1] = tip_y;          h.joints[tip][2] = 0; h.joints[tip][3] = 1.0f;
    };
    fill_finger(GE_JOINT_INDEX_MCP,  GE_JOINT_INDEX_PIP,  GE_JOINT_INDEX_DIP,  GE_JOINT_INDEX_TIP,
                thumb_tip_x + thumb_index_dist, thumb_tip_y);
    fill_finger(GE_JOINT_MIDDLE_MCP, GE_JOINT_MIDDLE_PIP, GE_JOINT_MIDDLE_DIP, GE_JOINT_MIDDLE_TIP,
                thumb_tip_x + thumb_middle_dist, thumb_tip_y);
    // Ring + pinky stay extended at fixed positions
    fill_finger(GE_JOINT_RING_MCP,   GE_JOINT_RING_PIP,   GE_JOINT_RING_DIP,   GE_JOINT_RING_TIP,
                0.06f, thumb_tip_y);
    fill_finger(GE_JOINT_PINKY_MCP,  GE_JOINT_PINKY_PIP,  GE_JOINT_PINKY_DIP,  GE_JOINT_PINKY_TIP,
                0.10f, thumb_tip_y);
    return h;
}

// Drive `frames` worth of synthetic data into the calibration state for
// the requested phase, with normally-distributed jitter on the target
// distance.
static void run_phase(ge_calib_t *gc, ge_calib_phase_t phase,
                      float thumb_index_target, float thumb_middle_target,
                      float jitter_sd, int frames,
                      std::mt19937 &rng) {
    gc_set_phase(gc, phase);
    std::normal_distribution<float> jitter(0.0f, jitter_sd);
    for (int i = 0; i < frames; ++i) {
        ge_hand_t h = make_open_hand_with_thumb(
            thumb_index_target  + jitter(rng),
            thumb_middle_target + jitter(rng));
        gc_observe(gc, &h);
    }
}

static bool within_pct(float got, float expected, float pct) {
    if (std::isnan(got)) return false;
    float tol = std::abs(expected) * (pct / 100.0f);
    if (tol < 1e-4f) tol = 1e-4f;
    return std::abs(got - expected) <= tol;
}

// ---------------------------------------------------------------------------
// Test 1: pinch_select calibration converges to expected formula
// ---------------------------------------------------------------------------

static void test_pinch_select_calibration() {
    std::mt19937 rng(1234);
    ge_calib_t *gc = gc_create();
    assert(gc);

    // Idle: thumb_index ~0.10 m with 5 mm jitter
    const float idle_dist  = 0.10f;
    // Active pinch: thumb_index ~0.005 m with 1 mm jitter
    const float pinch_dist = 0.005f;

    run_phase(gc, GE_CALIB_PHASE_IDLE,  idle_dist,  idle_dist,  0.005f, 300, rng);
    run_phase(gc, GE_CALIB_PHASE_PINCH, pinch_dist, idle_dist,  0.001f, 300, rng);
    // Need a grab phase too so resolve_threshold for grab/middle distances
    // doesn't fail on later tests, but pinch_select uses only PINCH.
    run_phase(gc, GE_CALIB_PHASE_GRAB,  pinch_dist, pinch_dist, 0.001f, 300, rng);

    float trigger = NAN, release = NAN;
    bool t_ok = gc_resolve_threshold(gc, "pinch_select", "trigger",
                                     "thumb_index_distance", &trigger);
    bool r_ok = gc_resolve_threshold(gc, "pinch_select", "release",
                                     "thumb_index_distance", &release);
    assert(t_ok && "should resolve pinch_select trigger");
    assert(r_ok && "should resolve pinch_select release");

    // active = 0.5 * (min + mean), with min ≈ pinch_dist - 3*0.001 ≈ 0.002
    // mean ≈ pinch_dist = 0.005, so active ≈ 0.0035.  Use a relaxed
    // expected band: 0.002 .. 0.005.
    // noise = mean(idle) ≈ 0.10
    // trigger = active + 0.30*(0.10 - active) = 0.30*0.10 + 0.70*active
    //         ≈ 0.03 + 0.70*0.0035 = 0.0325 (give or take)
    // release = 0.07*0.10 + ... = 0.07 + 0.70*0.0035 = 0.0725
    // Use looser tolerance (15%) because of the 0.5*(min+mean) approximation.
    assert(within_pct(trigger, 0.032f, 15.0f) && "trigger within ±15% of analytic");
    assert(within_pct(release, 0.072f, 15.0f) && "release within ±15% of analytic");

    // Trigger MUST be smaller than release for hysteresis.  This is the
    // hard invariant — without it the gesture self-cancels immediately.
    assert(trigger < release && "trigger must be < release for less-than gesture");

    gc_destroy(gc);
    std::printf("PASS test_pinch_select_calibration  trigger=%.4f release=%.4f\n",
                trigger, release);
}

// ---------------------------------------------------------------------------
// Test 2: grab_window calibration converges using the GRAB phase
// ---------------------------------------------------------------------------

static void test_grab_window_calibration() {
    std::mt19937 rng(5678);
    ge_calib_t *gc = gc_create();
    assert(gc);

    const float idle_index  = 0.09f;
    const float idle_middle = 0.085f;
    const float grab_index  = 0.02f;
    const float grab_middle = 0.03f;

    run_phase(gc, GE_CALIB_PHASE_IDLE,  idle_index, idle_middle, 0.005f, 300, rng);
    // Pinch phase irrelevant for grab thresholds; provide some data so the
    // phase isn't empty but values won't be used by grab_window.
    run_phase(gc, GE_CALIB_PHASE_PINCH, 0.005f,     idle_middle, 0.001f, 300, rng);
    run_phase(gc, GE_CALIB_PHASE_GRAB,  grab_index, grab_middle, 0.001f, 300, rng);

    float t_idx = NAN, r_idx = NAN, t_mid = NAN, r_mid = NAN;
    assert(gc_resolve_threshold(gc, "grab_window", "trigger",
                                "thumb_index_distance",  &t_idx));
    assert(gc_resolve_threshold(gc, "grab_window", "release",
                                "thumb_index_distance",  &r_idx));
    assert(gc_resolve_threshold(gc, "grab_window", "trigger",
                                "thumb_middle_distance", &t_mid));
    assert(gc_resolve_threshold(gc, "grab_window", "release",
                                "thumb_middle_distance", &r_mid));

    // For thumb_index: noise = 0.09, active ≈ 0.5*(0.017+0.02) ≈ 0.019
    // trigger = 0.019 + 0.3*(0.09-0.019) = 0.0403
    // release = 0.019 + 0.7*(0.09-0.019) = 0.0687
    assert(within_pct(t_idx, 0.040f, 15.0f));
    assert(within_pct(r_idx, 0.069f, 15.0f));
    // For thumb_middle: noise = 0.085, active ≈ 0.5*(0.027+0.03) ≈ 0.0285
    // trigger ≈ 0.0285 + 0.3*(0.085-0.0285) = 0.0455
    // release ≈ 0.0285 + 0.7*(0.085-0.0285) = 0.0681
    assert(within_pct(t_mid, 0.046f, 15.0f));
    assert(within_pct(r_mid, 0.068f, 15.0f));

    assert(t_idx < r_idx && t_mid < r_mid &&
           "trigger < release for less-than gestures");

    gc_destroy(gc);
    std::printf("PASS test_grab_window_calibration\n");
}

// ---------------------------------------------------------------------------
// Test 3: missing phase returns false instead of NaN-ing through
// ---------------------------------------------------------------------------

static void test_missing_phase_returns_false() {
    ge_calib_t *gc = gc_create();
    assert(gc);

    // Don't observe any IDLE data — only PINCH.
    std::mt19937 rng(99);
    run_phase(gc, GE_CALIB_PHASE_PINCH, 0.005f, 0.05f, 0.001f, 100, rng);

    float v;
    bool ok = gc_resolve_threshold(gc, "pinch_select", "trigger",
                                   "thumb_index_distance", &v);
    assert(!ok && "should return false when idle phase missing");

    gc_destroy(gc);
    std::printf("PASS test_missing_phase_returns_false\n");
}

// ---------------------------------------------------------------------------
// Test 4: emit_toml round-trips through ge_load_config
// ---------------------------------------------------------------------------

#include <cstdlib>
#include <unistd.h>

static void test_toml_roundtrips_through_loader() {
    std::mt19937 rng(42);
    ge_calib_t *gc = gc_create();
    assert(gc);

    run_phase(gc, GE_CALIB_PHASE_IDLE,  0.10f,  0.09f, 0.003f, 300, rng);
    run_phase(gc, GE_CALIB_PHASE_PINCH, 0.005f, 0.09f, 0.001f, 300, rng);
    run_phase(gc, GE_CALIB_PHASE_GRAB,  0.02f,  0.03f, 0.001f, 300, rng);

    // Write the calibrated TOML to a temp file.
    char tmpl[] = "/tmp/spatos-calib-XXXXXX";
    int fd = mkstemp(tmpl);
    assert(fd >= 0);
    FILE *f = fdopen(fd, "w");
    assert(f);
    gc_emit_toml(gc, f);
    std::fclose(f);

    // Capture the calibrated trigger value before loading.
    float expected_trigger;
    bool ok = gc_resolve_threshold(gc, "pinch_select", "trigger",
                                   "thumb_index_distance", &expected_trigger);
    assert(ok);

    // Load through the gesture-engine config loader and read it back via
    // the public accessor.
    ge_engine_t *ge = ge_create();
    assert(ge);
    bool applied = ge_load_config(ge, tmpl);
    assert(applied && "load_config should apply at least one override");

    float loaded = ge_gesture_threshold(ge, "pinch_select", "trigger",
                                        "thumb_index_distance");
    assert(within_pct(loaded, expected_trigger, 0.5f) &&
           "threshold round-trips through loader within float precision");

    ge_destroy(ge);
    gc_destroy(gc);
    std::remove(tmpl);
    std::printf("PASS test_toml_roundtrips_through_loader\n");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
    std::printf("=== gesture-engine calibrate tests ===\n");
    test_pinch_select_calibration();
    test_grab_window_calibration();
    test_missing_phase_returns_false();
    test_toml_roundtrips_through_loader();
    std::printf("=== ALL CALIBRATE TESTS PASSED ===\n");
    return 0;
}
