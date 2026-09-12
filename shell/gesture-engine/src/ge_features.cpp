// features.cpp — Hand feature extraction implementation.

#include "ge_features.h"
#include <algorithm>
#include <cmath>

namespace ge {

float compute_finger_curl(const ge_hand_t &hand, int mcp, int pip, int dip, int tip) {
    // Curl measures how much the fingertip has folded back relative to the
    // finger's base direction (MCP→PIP).  Angle between MCP→PIP and MCP→TIP:
    //   0   = straight (tip extends from MCP in same direction as PIP)
    //   PI  = fully folded back (tip behind MCP relative to base direction)
    //
    // Previous approach averaged consecutive segment angles, but in a real
    // fist the PIP joint bends ~180° while DIP stays straight.  Average of
    // ~180° and ~0° = ~90° = curl 0.50, so a real fist could never exceed
    // ~0.55.  This formula directly captures tip fold-back and gives ~0.97
    // for real fist data.
    (void)dip;  // not used — retained in signature for API compatibility
    Vec3 base_dir = joint_pos(hand, pip) - joint_pos(hand, mcp);
    Vec3 to_tip   = joint_pos(hand, tip) - joint_pos(hand, mcp);

    float lb = base_dir.length(), lt = to_tip.length();
    if (lb < 1e-8f || lt < 1e-8f) return 0.0f;

    float cos_angle = base_dir.dot(to_tip) / (lb * lt);
    cos_angle = std::clamp(cos_angle, -1.0f, 1.0f);

    return std::clamp(std::acos(cos_angle) / static_cast<float>(M_PI), 0.0f, 1.0f);
}

float project_thumb_on_index(const ge_hand_t &hand) {
    // Project thumb tip onto the polyline: INDEX_MCP -> INDEX_PIP -> INDEX_DIP -> INDEX_TIP
    // Returns 0..1 where 0 = at MCP, 1 = at TIP.
    Vec3 thumb = joint_pos(hand, GE_JOINT_THUMB_TIP);

    const int segments[] = {
        GE_JOINT_INDEX_MCP, GE_JOINT_INDEX_PIP,
        GE_JOINT_INDEX_PIP, GE_JOINT_INDEX_DIP,
        GE_JOINT_INDEX_DIP, GE_JOINT_INDEX_TIP,
    };

    // Compute total polyline length for normalization
    float total_length = 0;
    float seg_lengths[3];
    for (int i = 0; i < 3; ++i) {
        seg_lengths[i] = distance(joint_pos(hand, segments[i * 2]),
                                  joint_pos(hand, segments[i * 2 + 1]));
        total_length += seg_lengths[i];
    }
    if (total_length < 1e-8f) return 0.0f;

    // Find the closest point on each segment and compute the projection parameter
    float best_dist = 1e10f;
    float best_t = 0.0f;
    float cumulative_length = 0.0f;

    for (int i = 0; i < 3; ++i) {
        Vec3 a = joint_pos(hand, segments[i * 2]);
        Vec3 b = joint_pos(hand, segments[i * 2 + 1]);
        Vec3 ab = b - a;
        float seg_len = seg_lengths[i];
        if (seg_len < 1e-8f) {
            cumulative_length += seg_len;
            continue;
        }

        // Project thumb onto the line segment [a, b]
        float t = (thumb - a).dot(ab) / (seg_len * seg_len);
        t = std::clamp(t, 0.0f, 1.0f);

        Vec3 closest = a + ab * t;
        float d = distance(thumb, closest);

        if (d < best_dist) {
            best_dist = d;
            best_t = (cumulative_length + t * seg_len) / total_length;
        }
        cumulative_length += seg_len;
    }

    return std::clamp(best_t, 0.0f, 1.0f);
}

float project_thumb_on_middle(const ge_hand_t &hand) {
    // Project thumb tip onto the polyline: MIDDLE_MCP -> MIDDLE_PIP -> MIDDLE_DIP -> MIDDLE_TIP
    Vec3 thumb = joint_pos(hand, GE_JOINT_THUMB_TIP);

    const int segments[] = {
        GE_JOINT_MIDDLE_MCP, GE_JOINT_MIDDLE_PIP,
        GE_JOINT_MIDDLE_PIP, GE_JOINT_MIDDLE_DIP,
        GE_JOINT_MIDDLE_DIP, GE_JOINT_MIDDLE_TIP,
    };

    float total_length = 0;
    float seg_lengths[3];
    for (int i = 0; i < 3; ++i) {
        seg_lengths[i] = distance(joint_pos(hand, segments[i * 2]),
                                  joint_pos(hand, segments[i * 2 + 1]));
        total_length += seg_lengths[i];
    }
    if (total_length < 1e-8f) return 0.0f;

    float best_dist = 1e10f;
    float best_t = 0.0f;
    float cumulative_length = 0.0f;

    for (int i = 0; i < 3; ++i) {
        Vec3 a = joint_pos(hand, segments[i * 2]);
        Vec3 b = joint_pos(hand, segments[i * 2 + 1]);
        Vec3 ab = b - a;
        float seg_len = seg_lengths[i];
        if (seg_len < 1e-8f) {
            cumulative_length += seg_len;
            continue;
        }

        float t = (thumb - a).dot(ab) / (seg_len * seg_len);
        t = std::clamp(t, 0.0f, 1.0f);

        Vec3 closest = a + ab * t;
        float d = distance(thumb, closest);

        if (d < best_dist) {
            best_dist = d;
            best_t = (cumulative_length + t * seg_len) / total_length;
        }
        cumulative_length += seg_len;
    }

    return std::clamp(best_t, 0.0f, 1.0f);
}

static HandFeatures compute_features_raw(const ge_hand_t &hand) {
    HandFeatures f{};

    // Default: assume palm has been stable for "a long time" so a
    // fresh hand isn't treated as having just flipped.  The engine
    // overwrites this each tick with its own temporal tracker.
    f.palm_y_flip_recency_ms = 1e6f;
    // Default: assume thumb was just distant from index so a fresh
    // hand with a distant thumb behaves as if it had just converged.
    // Engine overwrites with real temporal tracking each tick.
    f.thumb_index_recency_above_30mm_ms = 0.0f;
    f.thumb_middle_recency_above_60mm_ms = 0.0f;
    f.pinch_select_end_recency_ms = 1e6f;
    f.palm_y_below_neg_half_recency_ms = 1e6f;
    f.fist_launcher_end_recency_ms = 1e6f;
    // Shape features default to "no shape evidence" — engine overwrites
    // each tick with values derived from a per-hand ring buffer.
    f.thumb_index_distance_min_over_50ms = 1e6f;
    f.thumb_index_distance_min_over_200ms = 1e6f;
    f.thumb_index_convergence_velocity_mps = 0.0f;
    f.thumb_index_convergence_ratio = 0.0f;
    f.thumb_middle_distance_min_over_200ms = 1e6f;
    f.thumb_middle_convergence_velocity_mps = 0.0f;
    f.thumb_middle_convergence_ratio = 0.0f;
    f.thumb_on_index_projection_velocity = 0.0f;
    f.thumb_on_index_projection_range_500ms = 0.0f;
    f.wrist_speed_mps = 0.0f;
    f.thumb_to_index_line_distance_max_over_200ms = 0.0f;
    f.index_curl_max_over_200ms = 0.0f;
    f.middle_curl_max_over_200ms = 0.0f;
    f.ring_curl_max_over_200ms = 0.0f;
    f.middle_tip_confidence_min_over_200ms = 1.0f;

    // Minimum confidence across all joints
    f.min_confidence = 1.0f;
    for (int j = 0; j < GE_JOINT_COUNT; ++j) {
        f.min_confidence = std::min(f.min_confidence, joint_conf(hand, j));
    }

    // Per-fingertip confidence — exposed so triggers can react to
    // tracker occlusion (e.g. middle finger drops to ~0.20 confidence
    // when the user's thumb-index pinch hides it from the camera).
    f.thumb_tip_confidence  = joint_conf(hand, GE_JOINT_THUMB_TIP);
    f.index_tip_confidence  = joint_conf(hand, GE_JOINT_INDEX_TIP);
    f.middle_tip_confidence = joint_conf(hand, GE_JOINT_MIDDLE_TIP);
    f.ring_tip_confidence   = joint_conf(hand, GE_JOINT_RING_TIP);
    f.pinky_tip_confidence  = joint_conf(hand, GE_JOINT_PINKY_TIP);

    // Finger-tip to thumb-tip distances
    Vec3 thumb_tip = joint_pos(hand, GE_JOINT_THUMB_TIP);
    f.thumb_index_distance  = distance(thumb_tip, joint_pos(hand, GE_JOINT_INDEX_TIP));
    f.thumb_middle_distance = distance(thumb_tip, joint_pos(hand, GE_JOINT_MIDDLE_TIP));
    f.thumb_ring_distance   = distance(thumb_tip, joint_pos(hand, GE_JOINT_RING_TIP));
    f.thumb_pinky_distance  = distance(thumb_tip, joint_pos(hand, GE_JOINT_PINKY_TIP));

    // Finger curl angles
    f.index_curl  = compute_finger_curl(hand,
        GE_JOINT_INDEX_MCP,  GE_JOINT_INDEX_PIP,  GE_JOINT_INDEX_DIP,  GE_JOINT_INDEX_TIP);
    f.middle_curl = compute_finger_curl(hand,
        GE_JOINT_MIDDLE_MCP, GE_JOINT_MIDDLE_PIP, GE_JOINT_MIDDLE_DIP, GE_JOINT_MIDDLE_TIP);
    f.ring_curl   = compute_finger_curl(hand,
        GE_JOINT_RING_MCP,   GE_JOINT_RING_PIP,   GE_JOINT_RING_DIP,   GE_JOINT_RING_TIP);
    f.pinky_curl  = compute_finger_curl(hand,
        GE_JOINT_PINKY_MCP,  GE_JOINT_PINKY_PIP,  GE_JOINT_PINKY_DIP,  GE_JOINT_PINKY_TIP);
    f.thumb_curl  = compute_finger_curl(hand,
        GE_JOINT_THUMB_CMC,  GE_JOINT_THUMB_MCP,  GE_JOINT_THUMB_IP,   GE_JOINT_THUMB_TIP);

    f.all_fingers_curl = (f.index_curl + f.middle_curl + f.ring_curl + f.pinky_curl) * 0.25f;

    // Palm openness = 1 - average curl
    f.palm_openness = 1.0f - f.all_fingers_curl;

    // Thumb-on-index projection
    f.thumb_on_index_projection = project_thumb_on_index(hand);

    // Thumb-on-middle projection (for middle-finger pinch detection)
    f.thumb_on_middle_projection = project_thumb_on_middle(hand);

    // Perpendicular distance from thumb tip to the index finger axis
    {
        Vec3 mcp = joint_pos(hand, GE_JOINT_INDEX_MCP);
        Vec3 tip = joint_pos(hand, GE_JOINT_INDEX_TIP);
        Vec3 line_dir = tip - mcp;
        float line_len = line_dir.length();
        if (line_len < 1e-8f) {
            f.thumb_to_index_line_distance = distance(thumb_tip, mcp);
        } else {
            Vec3 to_thumb = thumb_tip - mcp;
            f.thumb_to_index_line_distance = to_thumb.cross(line_dir).length() / line_len;
        }
    }

    // Signed projection of thumb (relative to index TIP) onto the
    // finger's pointing direction.  See header for the why.
    {
        Vec3 dip = joint_pos(hand, GE_JOINT_INDEX_DIP);
        Vec3 tip = joint_pos(hand, GE_JOINT_INDEX_TIP);
        Vec3 fdir = tip - dip;
        float flen = fdir.length();
        if (flen < 1e-8f) {
            f.thumb_along_index_tip_direction = 0.0f;
        } else {
            Vec3 fdir_n{fdir.x / flen, fdir.y / flen, fdir.z / flen};
            Vec3 to_thumb = thumb_tip - tip;
            f.thumb_along_index_tip_direction
                = to_thumb.x * fdir_n.x
                + to_thumb.y * fdir_n.y
                + to_thumb.z * fdir_n.z;
        }
    }

    // Cosine of middle-finger-tip direction toward thumb.  See header.
    {
        Vec3 mdip = joint_pos(hand, GE_JOINT_MIDDLE_DIP);
        Vec3 mtip = joint_pos(hand, GE_JOINT_MIDDLE_TIP);
        Vec3 mdir = mtip - mdip;
        float mlen = mdir.length();
        Vec3 to_thumb = thumb_tip - mtip;
        float tlen = to_thumb.length();
        if (mlen < 1e-8f || tlen < 1e-8f) {
            f.middle_tip_direction_to_thumb_cos = 0.0f;
        } else {
            f.middle_tip_direction_to_thumb_cos
                = (mdir.x * to_thumb.x
                 + mdir.y * to_thumb.y
                 + mdir.z * to_thumb.z) / (mlen * tlen);
        }
    }

    // Fingertip gather radius: compute centroid of all 5 fingertips, then
    // find the max distance from any tip to the centroid.
    {
        Vec3 tips[5] = {
            joint_pos(hand, GE_JOINT_THUMB_TIP),
            joint_pos(hand, GE_JOINT_INDEX_TIP),
            joint_pos(hand, GE_JOINT_MIDDLE_TIP),
            joint_pos(hand, GE_JOINT_RING_TIP),
            joint_pos(hand, GE_JOINT_PINKY_TIP),
        };
        Vec3 centroid = (tips[0] + tips[1] + tips[2] + tips[3] + tips[4]) * 0.2f;
        float max_r = 0.0f;
        for (int i = 0; i < 5; ++i) {
            float r = distance(tips[i], centroid);
            if (r > max_r) max_r = r;
        }
        f.fingertip_gather_radius = max_r;
    }

    // Key positions
    f.wrist_pos = joint_pos(hand, GE_JOINT_WRIST);

    Vec3 index_tip = joint_pos(hand, GE_JOINT_INDEX_TIP);
    f.pinch_midpoint = (thumb_tip + index_tip) * 0.5f;

    // Palm center: average of MCP joints
    Vec3 sum{};
    Vec3 index_mcp = joint_pos(hand, GE_JOINT_INDEX_MCP);
    Vec3 pinky_mcp = joint_pos(hand, GE_JOINT_PINKY_MCP);
    sum = sum + index_mcp;
    sum = sum + joint_pos(hand, GE_JOINT_MIDDLE_MCP);
    sum = sum + joint_pos(hand, GE_JOINT_RING_MCP);
    sum = sum + pinky_mcp;
    f.palm_center = sum * 0.25f;

    // Palm normal: cross product of (INDEX_MCP-WRIST) x (PINKY_MCP-WRIST).
    // Points in the palm-facing direction (verified from recording data).
    Vec3 wrist = f.wrist_pos;
    Vec3 to_index = index_mcp - wrist;
    Vec3 to_pinky = pinky_mcp - wrist;
    f.palm_normal = to_index.cross(to_pinky).normalized();

    return f;
}

// Robust per-hand metric scale: mean of the index/middle/ring proximal phalanx
// (MCP->PIP) lengths — rigid bones, invariant to pinch/curl.
float compute_hand_scale(const ge_hand_t &hand) {
    const int pairs[3][2] = {
        {GE_JOINT_INDEX_MCP,  GE_JOINT_INDEX_PIP},
        {GE_JOINT_MIDDLE_MCP, GE_JOINT_MIDDLE_PIP},
        {GE_JOINT_RING_MCP,   GE_JOINT_RING_PIP},
    };
    float s = 0.0f; int n = 0;
    for (auto &p : pairs) {
        float d = distance(joint_pos(hand, p[0]), joint_pos(hand, p[1]));
        if (d > 1e-5f) { s += d; ++n; }
    }
    return n ? s / (float)n : 0.0f;
}

float palm_thumb_signed_offset(const ge_hand_t &hand) {
    Vec3 wrist = joint_pos(hand, GE_JOINT_WRIST);
    Vec3 n = (joint_pos(hand, GE_JOINT_INDEX_MCP) - wrist)
                 .cross(joint_pos(hand, GE_JOINT_PINKY_MCP) - wrist);
    float nlen = n.length();
    float scale = compute_hand_scale(hand);
    if (nlen < 1e-9f || scale < 1e-5f) return 0.0f;
    // MCP/IP/TIP, not CMC: the CMC sits essentially in the palm plane and
    // only dilutes the signal.  Averaging the other three rides out the
    // ~+/-10 mm per-joint tracking noise.
    Vec3 thumb = (joint_pos(hand, GE_JOINT_THUMB_MCP)
                  + joint_pos(hand, GE_JOINT_THUMB_IP)
                  + joint_pos(hand, GE_JOINT_THUMB_TIP)) * (1.0f / 3.0f);
    return (n * (1.0f / nlen)).dot(thumb - wrist) / scale;
}

// Canonical hand scale (m): the reference proximal-phalanx length every hand is
// normalized to (about the wrist) before shape-feature extraction, so all
// distance-based features become scale-invariant (independent of hand size /
// the capture pipeline's metric scale). Set to the synthetic test fixture's
// scale (make_open_hand: MCP@y=0.04 -> PIP@y=0.07 = 0.03 m) so the fixtures
// normalize to identity (k=1) and the existing thresholds + synthetic tests are
// unaffected; only differently-scaled real hands get rescaled to this unit.
static constexpr float GE_REF_HAND_SCALE_M = 0.03f;

// iPhone Vision emits joint position (0,0,0) for a few consecutive ticks when
// it loses lock on a finger occluded by the thumb.  Any feature derived from
// such a joint is garbage (e.g. thumb-"middle" distance becomes the thumb's
// distance from the WORLD ORIGIN — ~0-30 mm during a pinch held near the
// body), so those features are marked indeterminate (NaN) instead.  Checked
// on the RAW hand: wrist-normalization would smear the exact zero.
static bool joint_is_zero_artifact(const ge_hand_t &hand, int j) {
    constexpr float kZeroEps = 1e-5f;
    Vec3 p = joint_pos(hand, j);
    return std::fabs(p.x) < kZeroEps && std::fabs(p.y) < kZeroEps
        && std::fabs(p.z) < kZeroEps;
}

HandFeatures compute_features(const ge_hand_t &hand,
                              float min_tip_confidence) {
    float scale = compute_hand_scale(hand);
    float k = (scale > 1e-5f) ? (GE_REF_HAND_SCALE_M / scale) : 1.0f;
    // Normalize the joint cloud about the wrist: preserves directions/angles
    // (curls, projections, palm normal) while making inter-joint distances
    // canonical. Confidence/reserved are carried over by the struct copy.
    ge_hand_t nh = hand;
    Vec3 wrist = joint_pos(hand, GE_JOINT_WRIST);
    for (int j = 0; j < GE_JOINT_COUNT; ++j) {
        Vec3 q = wrist + (joint_pos(hand, j) - wrist) * k;
        nh.joints[j][0] = q.x; nh.joints[j][1] = q.y; nh.joints[j][2] = q.z;
    }
    HandFeatures f = compute_features_raw(nh);
    f.hand_scale_m = scale;
    // World-position features must stay in real metres (gravity gates, window
    // move tracking): restore them from the ORIGINAL (un-normalized) hand.
    f.wrist_pos = wrist;
    f.pinch_midpoint = (joint_pos(hand, GE_JOINT_THUMB_TIP)
                        + joint_pos(hand, GE_JOINT_INDEX_TIP)) * 0.5f;
    Vec3 index_mcp = joint_pos(hand, GE_JOINT_INDEX_MCP);
    Vec3 pinky_mcp = joint_pos(hand, GE_JOINT_PINKY_MCP);
    f.palm_center = (index_mcp + joint_pos(hand, GE_JOINT_MIDDLE_MCP)
                     + joint_pos(hand, GE_JOINT_RING_MCP) + pinky_mcp) * 0.25f;
    f.palm_normal = (index_mcp - wrist).cross(pinky_mcp - wrist).normalized();

    // Zero-frame artifact filter (see joint_is_zero_artifact).  A trigger
    // condition reading NaN evaluates "not met" (the engine additionally
    // freezes — not resets — a Pending hold on indeterminate frames), and
    // NaN samples are skipped by the shape-window ring buffers.
    // A fingertip below the confidence gate is as untrustworthy as a
    // zeroed one — same indeterminate treatment, so low-confidence joints
    // can't drive triggers (see gesture-engine/CLAUDE.md "gotchas").
    auto tip_bad = [&](int j) {
        return joint_is_zero_artifact(hand, j)
            || joint_conf(hand, j) < min_tip_confidence;
    };
    const bool z_thumb  = tip_bad(GE_JOINT_THUMB_TIP);
    const bool z_index  = tip_bad(GE_JOINT_INDEX_TIP);
    const bool z_middle = tip_bad(GE_JOINT_MIDDLE_TIP);
    const bool z_ring   = tip_bad(GE_JOINT_RING_TIP);
    const bool z_pinky  = tip_bad(GE_JOINT_PINKY_TIP);
    if (z_thumb || z_index || z_middle || z_ring || z_pinky) {
        const float ind = std::nanf("");
        if (z_thumb || z_index) {
            f.thumb_index_distance           = ind;
            f.thumb_on_index_projection      = ind;
            f.thumb_to_index_line_distance   = ind;
            f.thumb_along_index_tip_direction = ind;
        }
        if (z_thumb || z_middle) {
            f.thumb_middle_distance             = ind;
            f.thumb_on_middle_projection        = ind;
            f.middle_tip_direction_to_thumb_cos = ind;
        }
        if (z_thumb || z_ring)  f.thumb_ring_distance  = ind;
        if (z_thumb || z_pinky) f.thumb_pinky_distance = ind;
        if (z_thumb)  f.thumb_curl  = ind;
        if (z_index)  f.index_curl  = ind;
        if (z_middle) f.middle_curl = ind;
        if (z_ring)   f.ring_curl   = ind;
        if (z_pinky)  f.pinky_curl  = ind;
        f.fingertip_gather_radius = ind;
        // NaN propagates through the aggregates arithmetically.
        f.all_fingers_curl = (f.index_curl + f.middle_curl
                              + f.ring_curl + f.pinky_curl) * 0.25f;
        f.palm_openness = 1.0f - f.all_fingers_curl;
    }
    return f;
}

float get_feature_by_name(const HandFeatures &f, const std::string &name) {
    if (name == "hand_scale_m")              return f.hand_scale_m;
    if (name == "thumb_index_distance")      return f.thumb_index_distance;
    if (name == "thumb_middle_distance")     return f.thumb_middle_distance;
    if (name == "thumb_ring_distance")       return f.thumb_ring_distance;
    if (name == "thumb_pinky_distance")      return f.thumb_pinky_distance;
    if (name == "index_curl")                return f.index_curl;
    if (name == "middle_curl")               return f.middle_curl;
    if (name == "ring_curl")                 return f.ring_curl;
    if (name == "pinky_curl")                return f.pinky_curl;
    if (name == "thumb_curl")                return f.thumb_curl;
    if (name == "all_fingers_curl")          return f.all_fingers_curl;
    if (name == "palm_openness")             return f.palm_openness;
    if (name == "thumb_on_index_projection")  return f.thumb_on_index_projection;
    if (name == "thumb_on_middle_projection") return f.thumb_on_middle_projection;
    if (name == "thumb_to_index_line_distance") return f.thumb_to_index_line_distance;
    if (name == "thumb_along_index_tip_direction") return f.thumb_along_index_tip_direction;
    if (name == "middle_tip_direction_to_thumb_cos") return f.middle_tip_direction_to_thumb_cos;
    if (name == "fingertip_gather_radius")   return f.fingertip_gather_radius;
    if (name == "min_confidence")            return f.min_confidence;
    if (name == "thumb_tip_confidence")      return f.thumb_tip_confidence;
    if (name == "index_tip_confidence")      return f.index_tip_confidence;
    if (name == "middle_tip_confidence")     return f.middle_tip_confidence;
    if (name == "ring_tip_confidence")       return f.ring_tip_confidence;
    if (name == "pinky_tip_confidence")      return f.pinky_tip_confidence;
    if (name == "palm_normal_x")             return f.palm_normal.x;
    if (name == "palm_normal_y")             return f.palm_normal.y;
    if (name == "palm_normal_z")             return f.palm_normal.z;
    if (name == "palm_y_flip_recency_ms")    return f.palm_y_flip_recency_ms;
    if (name == "thumb_index_recency_above_30mm_ms")
        return f.thumb_index_recency_above_30mm_ms;
    if (name == "thumb_middle_recency_above_60mm_ms")
        return f.thumb_middle_recency_above_60mm_ms;
    if (name == "pinch_select_end_recency_ms")
        return f.pinch_select_end_recency_ms;
    if (name == "palm_y_below_neg_half_recency_ms")
        return f.palm_y_below_neg_half_recency_ms;
    if (name == "fist_launcher_end_recency_ms")
        return f.fist_launcher_end_recency_ms;
    if (name == "thumb_index_distance_min_over_50ms")
        return f.thumb_index_distance_min_over_50ms;
    if (name == "thumb_index_distance_min_over_200ms")
        return f.thumb_index_distance_min_over_200ms;
    if (name == "thumb_index_convergence_velocity_mps")
        return f.thumb_index_convergence_velocity_mps;
    if (name == "thumb_index_convergence_ratio")
        return f.thumb_index_convergence_ratio;
    if (name == "thumb_middle_distance_min_over_200ms")
        return f.thumb_middle_distance_min_over_200ms;
    if (name == "thumb_middle_convergence_velocity_mps")
        return f.thumb_middle_convergence_velocity_mps;
    if (name == "thumb_middle_convergence_ratio")
        return f.thumb_middle_convergence_ratio;
    if (name == "thumb_to_index_line_distance_max_over_200ms")
        return f.thumb_to_index_line_distance_max_over_200ms;
    if (name == "index_curl_max_over_200ms")
        return f.index_curl_max_over_200ms;
    if (name == "middle_curl_max_over_200ms")
        return f.middle_curl_max_over_200ms;
    if (name == "ring_curl_max_over_200ms")
        return f.ring_curl_max_over_200ms;
    if (name == "middle_tip_confidence_min_over_200ms")
        return f.middle_tip_confidence_min_over_200ms;
    if (name == "thumb_on_index_projection_velocity")
        return f.thumb_on_index_projection_velocity;
    if (name == "thumb_on_index_projection_range_500ms")
        return f.thumb_on_index_projection_range_500ms;
    if (name == "wrist_speed_mps")           return f.wrist_speed_mps;
    // Tracked positions (return Y component for scroll-like tracking)
    if (name == "pinch_midpoint_y")          return f.pinch_midpoint.y;
    if (name == "palm_center_y")             return f.palm_center.y;
    if (name == "wrist_pos_x")               return f.wrist_pos.x;
    if (name == "wrist_pos_y")               return f.wrist_pos.y;
    if (name == "wrist_pos_z")               return f.wrist_pos.z;
    return 0.0f;
}

}  // namespace ge
