// test_config.cpp — Unit tests for ge_load_config() TOML loading.
//
// Verifies:
//   1. Basic key=value override of trigger thresholds
//   2. min_hold_ms override
//   3. release threshold override
//   4. Comment + blank-line tolerance
//   5. Unknown gestures / fields are skipped (don't abort load)
//   6. Missing file is a benign no-op
//   7. Per-line parse errors don't abort subsequent overrides
//   8. ge_gesture_threshold + ge_gesture_min_hold_ms accessors return NaN/-1
//      for unknown queries
//   9. The shipped sample mirrors print_default_config and loads as a no-op
//  10. [name] / [name.variant] addressing, duplicate headers, op selectors
//  11. [filter] range validation, indeterminate_release_ms field

#include "gesture_engine.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <unistd.h>

static std::string make_temp_path(const char *suffix) {
    const char *tmpdir = std::getenv("TMPDIR");
    if (!tmpdir || !*tmpdir) tmpdir = "/tmp";
    std::string base = std::string(tmpdir) + "/spatos-test-config-XXXXXX";
    std::vector<char> tmpl(base.begin(), base.end());
    tmpl.push_back('\0');
    int fd = mkstemp(tmpl.data());
    assert(fd >= 0 && "mkstemp failed");
    close(fd);
    std::string path(tmpl.data());
    path += suffix;
    return path;
}

static void write_file(const std::string &path, const std::string &content) {
    std::ofstream out(path);
    assert(out && "cannot open temp file for write");
    out << content;
}

// ---------------------------------------------------------------------------
// Test 1: simple trigger threshold override
// ---------------------------------------------------------------------------

static void test_basic_trigger_override() {
    ge_engine_t *ge = ge_create();
    assert(ge);

    // Default pinch_select trigger is 0.035
    float before = ge_gesture_threshold(ge, "pinch_select", "trigger",
                                        "thumb_index_distance");
    assert(std::abs(before - 0.035f) < 1e-6f && "default pinch trigger is 0.035");

    std::string path = make_temp_path(".toml");
    write_file(path,
               "[pinch_select]\n"
               "trigger.thumb_index_distance = 0.10\n");

    bool ok = ge_load_config(ge, path.c_str());
    assert(ok && "load_config should report success when an override applied");

    float after = ge_gesture_threshold(ge, "pinch_select", "trigger",
                                       "thumb_index_distance");
    assert(std::abs(after - 0.10f) < 1e-6f && "trigger should now be 0.10");

    std::remove(path.c_str());
    ge_destroy(ge);
    std::printf("PASS test_basic_trigger_override\n");
}

// ---------------------------------------------------------------------------
// Test 2: release threshold and min_hold_ms overrides
// ---------------------------------------------------------------------------

static void test_release_and_hold_override() {
    ge_engine_t *ge = ge_create();
    assert(ge);

    /* This test exercises release-threshold + min_hold_ms overrides
     * against scroll (was: grab_window).  grab is currently disabled
     * in engine.cpp pending a clearer activation gesture, so its
     * defaults are no longer registered.  scroll is a stable always-
     * on gesture that's appropriate for the config plumbing check. */
    // Defaults retuned 2026-05-23: scroll min_hold is 200 ms so it
    // matches pinch_select on overlap clips (39c1bf-style where the
    // thumb sits at the tip statically before sliding) — under the
    // strict bidirectional eval, scroll's higher priority claims
    // the hand on its hold elapse and pinch_select's Pending state
    // aborts silently.  See engine.cpp scroll block.
    int hold_before = ge_gesture_min_hold_ms(ge, "scroll");
    assert(hold_before == 120 && "default scroll hold is 120ms");

    float rel_before = ge_gesture_threshold(ge, "scroll", "release",
                                            "thumb_index_distance");
    assert(std::abs(rel_before - 0.055f) < 1e-6f && "default scroll TIGHT release is 0.055");

    std::string path = make_temp_path(".toml");
    write_file(path,
               "[scroll]\n"
               "min_hold_ms = 400\n"
               "release.thumb_index_distance = 0.09\n");

    bool ok = ge_load_config(ge, path.c_str());
    assert(ok);

    assert(ge_gesture_min_hold_ms(ge, "scroll") == 400);
    float rel_after = ge_gesture_threshold(ge, "scroll", "release",
                                           "thumb_index_distance");
    assert(std::abs(rel_after - 0.09f) < 1e-6f);

    std::remove(path.c_str());
    ge_destroy(ge);
    std::printf("PASS test_release_and_hold_override\n");
}

// ---------------------------------------------------------------------------
// Test 3: comments, blank lines, trailing whitespace
// ---------------------------------------------------------------------------

static void test_comments_and_whitespace() {
    ge_engine_t *ge = ge_create();
    assert(ge);

    std::string path = make_temp_path(".toml");
    write_file(path,
               "# This is a comment\n"
               "\n"
               "  [scroll]   # section header with comment\n"
               "  min_hold_ms = 200   # inline comment\n"
               "trigger.thumb_index_distance = 0.030  # tighter\n"
               "\n"
               "# trailing comment\n");

    bool ok = ge_load_config(ge, path.c_str());
    assert(ok);

    assert(ge_gesture_min_hold_ms(ge, "scroll") == 200);
    float t = ge_gesture_threshold(ge, "scroll", "trigger",
                                   "thumb_index_distance");
    assert(std::abs(t - 0.030f) < 1e-6f);

    std::remove(path.c_str());
    ge_destroy(ge);
    std::printf("PASS test_comments_and_whitespace\n");
}

// ---------------------------------------------------------------------------
// Test 4: unknown gesture / unknown field are skipped, not fatal
// ---------------------------------------------------------------------------

static void test_unknown_keys_skipped() {
    ge_engine_t *ge = ge_create();
    assert(ge);

    std::string path = make_temp_path(".toml");
    write_file(path,
               "[no_such_gesture]\n"
               "min_hold_ms = 999\n"
               "[pinch_select]\n"
               "release.no_such_feature = 1.0\n"
               "trigger.thumb_index_distance = 0.077\n");

    bool ok = ge_load_config(ge, path.c_str());
    assert(ok && "the one valid override should still apply");

    float t = ge_gesture_threshold(ge, "pinch_select", "trigger",
                                   "thumb_index_distance");
    assert(std::abs(t - 0.077f) < 1e-6f && "valid override applied");

    std::remove(path.c_str());
    ge_destroy(ge);
    std::printf("PASS test_unknown_keys_skipped\n");
}

// ---------------------------------------------------------------------------
// Test 5: missing file is a benign no-op
// ---------------------------------------------------------------------------

static void test_missing_file() {
    ge_engine_t *ge = ge_create();
    assert(ge);

    float before = ge_gesture_threshold(ge, "pinch_select", "trigger",
                                        "thumb_index_distance");

    bool ok = ge_load_config(ge,
                             "/nonexistent/path/spatial-os-test/gestures.toml");
    assert(!ok && "missing file should return false");

    float after = ge_gesture_threshold(ge, "pinch_select", "trigger",
                                       "thumb_index_distance");
    assert(before == after && "thresholds should be unchanged after missing-file load");

    ge_destroy(ge);
    std::printf("PASS test_missing_file\n");
}

// ---------------------------------------------------------------------------
// Test 6: malformed lines don't abort subsequent overrides
// ---------------------------------------------------------------------------

static void test_malformed_lines_tolerated() {
    ge_engine_t *ge = ge_create();
    assert(ge);

    std::string path = make_temp_path(".toml");
    write_file(path,
               "[pinch_select]\n"
               "this is not valid toml at all\n"
               "trigger.thumb_index_distance = not_a_number\n"
               "trigger.thumb_index_distance = 0.088\n");

    bool ok = ge_load_config(ge, path.c_str());
    assert(ok);

    // The duplicate key — TOML parsers usually error on duplicate keys, but our
    // simple map-based reader keeps the last value, so the final override wins.
    float t = ge_gesture_threshold(ge, "pinch_select", "trigger",
                                   "thumb_index_distance");
    assert(std::abs(t - 0.088f) < 1e-6f);

    std::remove(path.c_str());
    ge_destroy(ge);
    std::printf("PASS test_malformed_lines_tolerated\n");
}

// ---------------------------------------------------------------------------
// Test 7: accessors return NaN/-1 for unknown queries
// ---------------------------------------------------------------------------

static void test_accessor_unknown_returns_sentinel() {
    ge_engine_t *ge = ge_create();
    assert(ge);

    assert(ge_gesture_min_hold_ms(ge, "no_such_gesture") == -1);
    assert(ge_gesture_min_hold_ms(nullptr, "pinch_select") == -1);

    float v = ge_gesture_threshold(ge, "no_such_gesture", "trigger", "x");
    assert(std::isnan(v));
    v = ge_gesture_threshold(ge, "pinch_select", "trigger", "no_such_feature");
    assert(std::isnan(v));
    v = ge_gesture_threshold(ge, "pinch_select", "neither", "thumb_index_distance");
    assert(std::isnan(v));

    ge_destroy(ge);
    std::printf("PASS test_accessor_unknown_returns_sentinel\n");
}

// ---------------------------------------------------------------------------
// Test 8: the shipped sample is byte-identical to print_default_config, and
// loading it is a no-op (dump after load == default dump).  Guards the
// same-name-variant bug where four bare [pinch_select] sections collapsed
// into one and rewrote the first variant with the last section's values.
// ---------------------------------------------------------------------------

static std::string read_file(const std::string &path) {
    std::ifstream in(path);
    assert(in && "cannot open file");
    std::string s((std::istreambuf_iterator<char>(in)),
                  std::istreambuf_iterator<char>());
    return s;
}

static std::string capture_default_config() {
    std::string path = make_temp_path(".toml");
    FILE *f = std::fopen(path.c_str(), "w");
    assert(f);
    ge_print_default_config(f);
    std::fclose(f);
    std::string s = read_file(path);
    std::remove(path.c_str());
    return s;
}

static std::string capture_config(const ge_engine_t *ge) {
    std::string path = make_temp_path(".toml");
    FILE *f = std::fopen(path.c_str(), "w");
    assert(f);
    ge_print_config(ge, f);
    std::fclose(f);
    std::string s = read_file(path);
    std::remove(path.c_str());
    return s;
}

static void test_sample_config_roundtrip_is_noop() {
    std::string def = capture_default_config();
    std::string sample = read_file(GE_SAMPLE_CONFIG_PATH);
    if (sample != def) {
        std::printf("sample-config/gestures.toml is stale — regenerate with "
                    "tools/print_default_config\n");
    }
    assert(sample == def && "sample config must mirror the compiled defaults");

    ge_engine_t *ge = ge_create();
    assert(ge);
    ge_load_config(ge, GE_SAMPLE_CONFIG_PATH);
    std::string after = capture_config(ge);
    if (after != def) {
        std::printf("---- default ----\n%s\n---- after loading sample ----\n%s\n",
                    def.c_str(), after.c_str());
    }
    assert(after == def && "loading the sample must change nothing");
    ge_destroy(ge);
    std::printf("PASS test_sample_config_roundtrip_is_noop\n");
}

// ---------------------------------------------------------------------------
// Test 9: variant addressing — bare [name] hits every variant, [name.variant]
// one; unknown variants are skipped.
// ---------------------------------------------------------------------------

static void test_variant_sections() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    assert(ge_gesture_min_hold_ms(ge, "pinch_select.loose") == 100);
    assert(std::abs(ge_gesture_threshold(ge, "pinch_select.loose", "trigger",
                                         "thumb_index_distance") - 0.075f) < 1e-6f);

    std::string path = make_temp_path(".toml");
    write_file(path,
               "[pinch_select]\n"
               "min_hold_ms = 77\n"
               "[pinch_select.loose]\n"
               "trigger.thumb_index_distance = 0.09\n"
               "[pinch_select.nope]\n"
               "min_hold_ms = 5\n");
    assert(ge_load_config(ge, path.c_str()));

    const char *variants[] = {"pinch_select", "pinch_select.standard",
                              "pinch_select.loose", "pinch_select.double_tap",
                              "pinch_select.palm_up"};
    for (const char *v : variants)
        assert(ge_gesture_min_hold_ms(ge, v) == 77 && "bare section applies to every variant");
    assert(std::abs(ge_gesture_threshold(ge, "pinch_select.loose", "trigger",
                                         "thumb_index_distance") - 0.09f) < 1e-6f);
    assert(std::abs(ge_gesture_threshold(ge, "pinch_select.standard", "trigger",
                                         "thumb_index_distance") - 0.035f) < 1e-6f &&
           "variant section must not leak into other variants");
    assert(ge_gesture_min_hold_ms(ge, "pinch_select.nope") == -1);

    std::remove(path.c_str());
    ge_destroy(ge);
    std::printf("PASS test_variant_sections\n");
}

// ---------------------------------------------------------------------------
// Test 10: a repeated section header is rejected (first one wins).
// ---------------------------------------------------------------------------

static void test_duplicate_section_rejected() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    std::string path = make_temp_path(".toml");
    write_file(path,
               "[pinch_select]\n"
               "min_hold_ms = 11\n"
               "[scroll]\n"
               "min_hold_ms = 33\n"
               "[pinch_select]\n"
               "min_hold_ms = 22\n"
               "cooldown_ms = 999\n");
    assert(ge_load_config(ge, path.c_str()));
    assert(ge_gesture_min_hold_ms(ge, "pinch_select") == 11);
    assert(ge_gesture_min_hold_ms(ge, "pinch_select.palm_up") == 11);
    assert(ge_gesture_cooldown_ms(ge, "pinch_select") == 60 &&
           "keys under the duplicate header must be ignored");
    assert(ge_gesture_min_hold_ms(ge, "scroll") == 33);
    std::remove(path.c_str());
    ge_destroy(ge);
    std::printf("PASS test_duplicate_section_rejected\n");
}

// ---------------------------------------------------------------------------
// Test 11: comparison selector for a feature with two conditions in one list.
// ---------------------------------------------------------------------------

static int find_def(const ge_engine_t *ge, const char *name, const char *variant) {
    for (int i = 0; i < ge_gesture_count(ge); ++i) {
        if (std::string(ge_gesture_name_at(ge, i)) == name
            && std::string(ge_gesture_variant_at(ge, i)) == variant)
            return i;
    }
    return -1;
}

static float threshold_of(const ge_engine_t *ge, int def, const char *kind,
                          const char *feature, const char *op) {
    for (int c = 0; c < ge_gesture_condition_count(ge, def, kind); ++c) {
        const char *f, *o; float th;
        assert(ge_gesture_condition_at(ge, def, kind, c, &f, &o, &th));
        if (std::string(f) == feature && std::string(o) == op) return th;
    }
    return 0.0f / 0.0f;
}

static void test_op_selector() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    int wide = find_def(ge, "scroll", "wide");
    assert(wide >= 0);
    assert(std::abs(threshold_of(ge, wide, "trigger", "thumb_index_distance", "less") - 0.07f) < 1e-6f);
    assert(std::abs(threshold_of(ge, wide, "trigger", "thumb_index_distance", "greater") - 0.02f) < 1e-6f);

    std::string path = make_temp_path(".toml");
    write_file(path,
               "[scroll.wide]\n"
               "trigger.thumb_index_distance.greater = 0.025\n"
               "trigger.thumb_index_distance.bogus = 0.5\n");
    assert(ge_load_config(ge, path.c_str()));
    assert(std::abs(threshold_of(ge, wide, "trigger", "thumb_index_distance", "greater") - 0.025f) < 1e-6f);
    assert(std::abs(threshold_of(ge, wide, "trigger", "thumb_index_distance", "less") - 0.07f) < 1e-6f &&
           "selector must leave the other comparison alone");
    std::remove(path.c_str());
    ge_destroy(ge);
    std::printf("PASS test_op_selector\n");
}

// ---------------------------------------------------------------------------
// Test 12: [filter] validation — out-of-range values keep the defaults.
// ---------------------------------------------------------------------------

static void test_filter_validation() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    std::string path = make_temp_path(".toml");
    write_file(path,
               "[filter]\n"
               "one_euro_min_cutoff_hz = 0\n"
               "one_euro_beta = -1\n"
               "one_euro_d_cutoff_hz = -2\n"
               "min_joint_confidence = 1.5\n"
               "one_euro_enabled = 0\n");
    assert(ge_load_config(ge, path.c_str()) && "the one valid key still applies");
    assert(std::abs(ge_filter_param(ge, "one_euro_min_cutoff_hz") - 1.5f) < 1e-6f);
    assert(std::abs(ge_filter_param(ge, "one_euro_beta") - 30.0f) < 1e-6f);
    assert(std::abs(ge_filter_param(ge, "one_euro_d_cutoff_hz") - 1.0f) < 1e-6f);
    assert(std::abs(ge_filter_param(ge, "min_joint_confidence") - 0.15f) < 1e-6f);
    assert(ge_filter_param(ge, "one_euro_enabled") == 0.0f);

    write_file(path,
               "[filter]\n"
               "one_euro_min_cutoff_hz = 2.5\n"
               "one_euro_beta = 0\n"
               "one_euro_d_cutoff_hz = 0.5\n");
    assert(ge_load_config(ge, path.c_str()));
    assert(std::abs(ge_filter_param(ge, "one_euro_min_cutoff_hz") - 2.5f) < 1e-6f);
    assert(ge_filter_param(ge, "one_euro_beta") == 0.0f && "beta = 0 is valid");
    assert(std::abs(ge_filter_param(ge, "one_euro_d_cutoff_hz") - 0.5f) < 1e-6f);
    std::remove(path.c_str());
    ge_destroy(ge);
    std::printf("PASS test_filter_validation\n");
}

// ---------------------------------------------------------------------------
// Test 13: indeterminate_release_ms is a loadable per-gesture field.
// ---------------------------------------------------------------------------

static void test_indeterminate_release_ms_override() {
    ge_engine_t *ge = ge_create();
    assert(ge);
    assert(ge_gesture_indeterminate_release_ms(ge, "pinch_select") == 300);
    std::string path = make_temp_path(".toml");
    write_file(path,
               "[pinch_select.standard]\n"
               "indeterminate_release_ms = 900\n");
    assert(ge_load_config(ge, path.c_str()));
    assert(ge_gesture_indeterminate_release_ms(ge, "pinch_select.standard") == 900);
    assert(ge_gesture_indeterminate_release_ms(ge, "pinch_select.loose") == 300);
    std::remove(path.c_str());
    ge_destroy(ge);
    std::printf("PASS test_indeterminate_release_ms_override\n");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
    std::printf("=== gesture-engine config tests ===\n");
    test_basic_trigger_override();
    test_release_and_hold_override();
    test_comments_and_whitespace();
    test_unknown_keys_skipped();
    test_missing_file();
    test_malformed_lines_tolerated();
    test_accessor_unknown_returns_sentinel();
    test_sample_config_roundtrip_is_noop();
    test_variant_sections();
    test_duplicate_section_rejected();
    test_op_selector();
    test_filter_validation();
    test_indeterminate_release_ms_override();
    std::printf("=== ALL CONFIG TESTS PASSED ===\n");
    return 0;
}
