// calibrate.cpp — Per-user gesture-engine calibration implementation.

#include "gesture_calibrate.h"
#include "ge_features.h"
#include "gesture_engine.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace {

// Features sampled during calibration.  These are the inputs to every
// trigger/release condition currently in engine.cpp; if more conditions
// land later, extend this list.
const char *kCalibratedFeatures[] = {
    "thumb_index_distance",
    "thumb_middle_distance",
    "thumb_ring_distance",
    "thumb_pinky_distance",
    "thumb_to_index_line_distance",
    "thumb_on_index_projection",
    "thumb_on_middle_projection",
    "fingertip_gather_radius",
    "all_fingers_curl",
    "index_curl",
    "middle_curl",
    "ring_curl",
    "pinky_curl",
};

constexpr int kFeatureCount =
    sizeof(kCalibratedFeatures) / sizeof(kCalibratedFeatures[0]);

struct Stat {
    int   n     = 0;
    float min   = std::numeric_limits<float>::infinity();
    float max   = -std::numeric_limits<float>::infinity();
    double sum  = 0.0;

    void add(float v) {
        ++n;
        if (v < min) min = v;
        if (v > max) max = v;
        sum += v;
    }
    bool valid() const { return n > 0; }
    float mean() const { return n > 0 ? static_cast<float>(sum / n) : 0.0f; }
};

// For each feature, the "noise floor" is the band the value sits in when
// the user isn't doing the gesture, and "active" is what we measure when
// they are.  For distance-style features (thumb_index_distance etc.):
//   noise  is large (hand open)
//   active is small (fingers touching)
// For curl-style features:
//   noise  is small (fingers extended)
//   active is large (fingers curled)
// We take noise = mean during idle, active = the value most extreme on
// the active side observed during the active phase.

float noise_value(const Stat &idle) { return idle.mean(); }

float active_value(const Stat &idle, const Stat &active) {
    // Direction is set by which side of `idle` the active values fell on.
    if (active.mean() < idle.mean()) {
        // Active is below idle (e.g., a distance closing during a pinch).
        // Use the lowest values we saw: a robust "active" is the average
        // of the bottom 25% of samples; with min/max only we approximate
        // by 0.5*(min + mean), biased toward the low end.
        return 0.5f * (active.min + active.mean());
    } else {
        return 0.5f * (active.max + active.mean());
    }
}

float compute_trigger(float noise, float active) {
    // Lands at 30% of the gap between active and noise, on the active side.
    return active + 0.30f * (noise - active);
}

float compute_release(float noise, float active) {
    // 70% of the gap — further from active, closer to noise.  Hysteresis
    // direction is naturally correct: the gesture exits when the feature
    // moves further from active than the trigger threshold required.
    return active + 0.70f * (noise - active);
}

// Maps each (gesture, kind, feature) override the loader supports to the
// active phase we should pull statistics from.
struct OverrideKey {
    std::string gesture;
    std::string kind;     // "trigger" | "release"
    std::string feature;
    ge_calib_phase_t active_phase;
};

const std::vector<OverrideKey> &calibration_table() {
    static const std::vector<OverrideKey> table = {
        // pinch_select
        {"pinch_select", "trigger", "thumb_index_distance", GE_CALIB_PHASE_PINCH},
        {"pinch_select", "release", "thumb_index_distance", GE_CALIB_PHASE_PINCH},
        // thumb_slider
        {"thumb_slider", "trigger", "thumb_index_distance", GE_CALIB_PHASE_PINCH},
        {"thumb_slider", "release", "thumb_index_distance", GE_CALIB_PHASE_PINCH},
        // grab_window
        {"grab_window",  "trigger", "thumb_index_distance",  GE_CALIB_PHASE_GRAB},
        {"grab_window",  "trigger", "thumb_middle_distance", GE_CALIB_PHASE_GRAB},
        {"grab_window",  "release", "thumb_index_distance",  GE_CALIB_PHASE_GRAB},
        {"grab_window",  "release", "thumb_middle_distance", GE_CALIB_PHASE_GRAB},
        // pinch_right_click — middle-finger pinch is treated as a "grab"-class
        // active sample for the middle finger.  Without a dedicated phase,
        // we fall back to PINCH.
        {"pinch_right_click", "trigger", "thumb_middle_distance", GE_CALIB_PHASE_GRAB},
        {"pinch_right_click", "release", "thumb_middle_distance", GE_CALIB_PHASE_GRAB},
    };
    return table;
}

}  // anonymous namespace

struct ge_calib_t {
    // [phase][feature_index] -> stat
    Stat phases[GE_CALIB_PHASE_COUNT][kFeatureCount];
    ge_calib_phase_t current_phase = GE_CALIB_PHASE_IDLE;
};

ge_calib_t *gc_create(void) {
    return new (std::nothrow) ge_calib_t();
}

void gc_destroy(ge_calib_t *gc) {
    delete gc;
}

void gc_set_phase(ge_calib_t *gc, ge_calib_phase_t phase) {
    if (!gc || phase < 0 || phase >= GE_CALIB_PHASE_COUNT) return;
    gc->current_phase = phase;
}

void gc_observe(ge_calib_t *gc, const ge_hand_t *hand) {
    if (!gc || !hand || !hand->present) return;

    // Drop frames where any required joint has low confidence — same
    // policy as the engine's predicate evaluation.
    for (int j = 0; j < GE_JOINT_COUNT; ++j) {
        if (hand->joints[j][3] < 0.2f) return;
    }

    ge::HandFeatures f = ge::compute_features(*hand);
    for (int i = 0; i < kFeatureCount; ++i) {
        float v = ge::get_feature_by_name(f, std::string(kCalibratedFeatures[i]));
        gc->phases[gc->current_phase][i].add(v);
    }
}

static int feature_index(const std::string &name) {
    for (int i = 0; i < kFeatureCount; ++i) {
        if (name == kCalibratedFeatures[i]) return i;
    }
    return -1;
}

bool gc_resolve_threshold(const ge_calib_t *gc,
                          const char *gesture_name,
                          const char *kind,
                          const char *feature,
                          float *out_value) {
    if (!gc || !gesture_name || !kind || !feature || !out_value) return false;

    ge_calib_phase_t active_phase = GE_CALIB_PHASE_PINCH;
    bool found = false;
    for (const auto &row : calibration_table()) {
        if (row.gesture == gesture_name && row.kind == kind &&
            row.feature == feature) {
            active_phase = row.active_phase;
            found = true;
            break;
        }
    }
    if (!found) return false;

    int fi = feature_index(feature);
    if (fi < 0) return false;

    const Stat &idle   = gc->phases[GE_CALIB_PHASE_IDLE][fi];
    const Stat &active = gc->phases[active_phase][fi];
    if (!idle.valid() || !active.valid()) return false;

    float noise = noise_value(idle);
    float act   = active_value(idle, active);
    *out_value = (std::strcmp(kind, "trigger") == 0)
                     ? compute_trigger(noise, act)
                     : compute_release(noise, act);
    return true;
}

void gc_emit_toml(const ge_calib_t *gc, FILE *f) {
    if (!f) f = stdout;
    if (!gc) return;

    std::fprintf(f,
        "# AUTO-GENERATED by gesture-engine calibration.\n"
        "# Idle / active samples per feature are folded into trigger/release\n"
        "# thresholds via:\n"
        "#   trigger = active + 0.30 * (noise - active)\n"
        "#   release = active + 0.70 * (noise - active)\n"
        "# Features that didn't get enough data fall back to compiled defaults.\n");

    // Only gestures the engine actually compiles: the loader rejects a
    // section for an unknown name (grab_window / thumb_slider rows stay
    // in the table for when those gestures return).
    std::set<std::string> compiled;
    if (ge_engine_t *ge = ge_create()) {
        for (int i = 0; i < ge_gesture_count(ge); ++i)
            compiled.insert(ge_gesture_name_at(ge, i));
        ge_destroy(ge);
    }

    // Group rows by gesture for nicer output.
    std::map<std::string, std::vector<const OverrideKey *>> by_gesture;
    for (const auto &row : calibration_table()) {
        if (compiled.count(row.gesture) == 0) continue;
        by_gesture[row.gesture].push_back(&row);
    }
    for (const auto &[gesture, rows] : by_gesture) {
        std::fprintf(f, "\n[%s]\n", gesture.c_str());
        for (const auto *row : rows) {
            float v;
            if (!gc_resolve_threshold(gc, row->gesture.c_str(),
                                      row->kind.c_str(),
                                      row->feature.c_str(), &v)) {
                continue;
            }
            std::fprintf(f, "%s.%s = %g\n",
                         row->kind.c_str(), row->feature.c_str(),
                         static_cast<double>(v));
        }
    }
}
