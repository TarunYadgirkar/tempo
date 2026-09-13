#include "core/hand_inject.h"

#include <cmath>
#include <cstring>
#include <ctime>

#include "core/json_lite.h"
#include "core/vec_math.h"

namespace mac_shell {
namespace {

// MediaPipe landmark -> SB_JOINT_*. Identity today (see hand_inject.h), but
// written out so a renumbering on either side shows up as a diff here and a
// failing test rather than as silently swapped fingers.
const int MP_TO_SB[SB_HAND_JOINT_COUNT] = {
    SB_JOINT_WRIST,      SB_JOINT_THUMB_CMC,  SB_JOINT_THUMB_MCP,
    SB_JOINT_THUMB_IP,   SB_JOINT_THUMB_TIP,  SB_JOINT_INDEX_MCP,
    SB_JOINT_INDEX_PIP,  SB_JOINT_INDEX_DIP,  SB_JOINT_INDEX_TIP,
    SB_JOINT_MIDDLE_MCP, SB_JOINT_MIDDLE_PIP, SB_JOINT_MIDDLE_DIP,
    SB_JOINT_MIDDLE_TIP, SB_JOINT_RING_MCP,   SB_JOINT_RING_PIP,
    SB_JOINT_RING_DIP,   SB_JOINT_RING_TIP,   SB_JOINT_PINKY_MCP,
    SB_JOINT_PINKY_PIP,  SB_JOINT_PINKY_DIP,  SB_JOINT_PINKY_TIP,
};

}  // namespace

int mediapipe_to_sb_joint(int mp_index) {
    if (mp_index < 0 || mp_index >= SB_HAND_JOINT_COUNT)
        return -1;
    return MP_TO_SB[mp_index];
}

int sb_to_mediapipe_joint(int sb_index) {
    for (int i = 0; i < SB_HAND_JOINT_COUNT; i++)
        if (MP_TO_SB[i] == sb_index)
            return i;
    return -1;
}

bool unproject_pixel(const sb_intrinsics_t &intr, float img_w, float img_h,
                     float u, float v, float depth_m, float out_cam[3]) {
    if (!(depth_m > 0.0f) || !std::isfinite(depth_m))
        return false;
    if (!(intr.fx > 1.0f) || !(intr.fy > 1.0f) || !(intr.image_width > 1.0f) ||
        !(intr.image_height > 1.0f))
        return false;
    if (!(img_w > 0.0f) || !(img_h > 0.0f))
        return false;
    if (!std::isfinite(u) || !std::isfinite(v))
        return false;

    // Intrinsics live in ARKit capture pixels; the streamed image is a scaled
    // copy of the same view, so one scale per axis carries the pixel across.
    const float sx = intr.image_width / img_w;
    const float sy = intr.image_height / img_h;
    const float px = u * sx;
    const float py = v * sy;

    // Pinhole, ARKit camera basis: image +v points down, camera +Y points up,
    // and the view direction is -Z, so depth along the axis is -z.
    out_cam[0] = (px - intr.cx) * depth_m / intr.fx;
    out_cam[1] = -(py - intr.cy) * depth_m / intr.fy;
    out_cam[2] = -depth_m;
    return v3finite(out_cam);
}

void camera_to_scene(const float cam_pos[3], const float cam_quat[4],
                     const float p_cam[3], float out[3]) {
    float rotated[3];
    quat_rotate_vec(cam_quat, p_cam, rotated);
    out[0] = cam_pos[0] + rotated[0];
    out[1] = cam_pos[1] + rotated[1];
    out[2] = cam_pos[2] + rotated[2];
}

uint64_t hand_inject_now_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)(ts.tv_nsec / 1000000);
}

namespace {

bool parse_one_hand(const json_value &h, sb_hand_t &out, bool &is_left,
                    std::string &err) {
    const json_value *chir = h.find("chirality");
    if (!chir || !chir->is_string()) {
        err = "hand.chirality must be \"left\" or \"right\"";
        return false;
    }
    if (chir->str == "left") {
        is_left = true;
    } else if (chir->str == "right") {
        is_left = false;
    } else {
        err = "hand.chirality must be \"left\" or \"right\"";
        return false;
    }

    float confidence = 1.0f;
    if (const json_value *c = h.find("confidence")) {
        if (!c->is_number()) {
            err = "hand.confidence must be a number";
            return false;
        }
        confidence = (float)c->number;
        if (!std::isfinite(confidence))
            confidence = 0.0f;
        confidence = confidence < 0.0f ? 0.0f
                                       : (confidence > 1.0f ? 1.0f : confidence);
    }

    const json_value *joints = h.find("joints");
    if (!joints || !joints->is_array() ||
        (int)joints->items.size() != SB_HAND_JOINT_COUNT) {
        err = "hand.joints must be 21 [x,y,z] triples";
        return false;
    }

    out = sb_hand_t{};
    out.joint_count = SB_HAND_JOINT_COUNT;
    for (int mp = 0; mp < SB_HAND_JOINT_COUNT; mp++) {
        float p[3];
        if (!joints->items[(size_t)mp].floats(p, 3) || !v3finite(p)) {
            err = "hand.joints entry is not a finite [x,y,z]";
            return false;
        }
        const int sb = mediapipe_to_sb_joint(mp);
        out.joints[sb][0] = p[0];
        out.joints[sb][1] = p[1];
        out.joints[sb][2] = p[2];
        out.joints[sb][3] = confidence;
        out.joints[sb][4] = 0.0f;
    }
    return true;
}

}  // namespace

bool parse_hands_inject(const std::string &json, injected_hands &out,
                        std::string &err) {
    json_value root;
    if (!json_parse(json, root, err))
        return false;
    if (!root.is_object()) {
        err = "payload must be an object";
        return false;
    }

    out = injected_hands{};

    const json_value *t = root.find("t");
    if (!t || !t->is_number() || !(t->number >= 0.0) ||
        !std::isfinite(t->number)) {
        err = "t must be a non-negative millisecond timestamp";
        return false;
    }
    out.t_ms = (uint64_t)t->number;

    const json_value *hands = root.find("hands");
    if (!hands || !hands->is_array()) {
        err = "hands must be an array";
        return false;
    }
    if (hands->items.size() > 2) {
        err = "at most two hands";
        return false;
    }

    for (const json_value &h : hands->items) {
        if (!h.is_object()) {
            err = "hands entry must be an object";
            return false;
        }
        if (!parse_one_hand(h, out.hands[out.count], out.is_left[out.count],
                            err))
            return false;
        out.hands[out.count].hand_index = (uint8_t)out.count;
        out.hands[out.count].timestamp_ns = out.t_ms * 1000000ull;
        out.count++;
    }
    return true;
}

void hand_inject_store::set(const injected_hands &h) {
    hands_ = h;
    received_ms_ = hand_inject_now_ms();
    ever_ = true;
}

uint64_t hand_inject_store::age_ms(uint64_t now_ms) const {
    if (!ever_ || now_ms <= received_ms_)
        return 0;
    return now_ms - received_ms_;
}

bool hand_inject_store::fresh(uint64_t now_ms) const {
    return ever_ && age_ms(now_ms) < HAND_INJECT_FRESH_MS;
}

}  // namespace mac_shell
