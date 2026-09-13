// scene.cpp — see scene.h.

#include "core/scene.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "core/json_lite.h"
#include "core/placement_math.h"
#include "ui/note_card.h"
#include "ui/theme_tokens.h"
#include "core/vec_math.h"

extern "C" {
#include "keyboard_geom.h"
}

namespace mac_shell {

namespace {

// A hand with no fresh packet for this long is treated as absent.
constexpr float HAND_STALE_S = 0.15f;
constexpr float AIM_MIN_CONFIDENCE = 0.15f;
// Pinch-to-focus targeting. A panel is only focusable when the pinch point
// itself is near it (direct touch), or when it sits near the head->pinch ray
// IN FRONT of the user — a pinch must never focus a panel behind you.
constexpr float FOCUS_TOUCH_DIST_M = 0.35f;   // direct-touch radius
constexpr float FOCUS_RAY_MAX_PERP_M = 0.45f; // ray corridor (panels ~0.6 m wide)
constexpr float FOCUS_MAX_DIST_M = 2.0f;      // beyond this, no focus at all
// Grab feel: hand must move this far before the panel starts following
// (kills micro-jitter of a held grab). Beyond the deadband the panel tracks
// the hand target directly; the renderer's move spring is the only
// smoothing stage (docs/mac-shell-design.md).
constexpr float GRAB_DEADBAND_M = 0.02f;
// Legacy spawn shelf — used only before the first head pose arrives (tests,
// pose-less e2e runs). With a pose, panels spawn in front of the CURRENT
// head via placement_math.h.
constexpr float SPAWN_POS[3] = {0.0f, 0.0f, -1.2f};
constexpr float SPAWN_STEP_X = 0.25f;
// A hole this long in the pose stream means the phone app most likely restarted
// its ARKit session, so its origin has moved (wxrd main.c uses the same 2 s).
constexpr double POSE_GAP_RECAPTURE_S = 2.0;

std::string fnum(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g", v);
    return buf;
}

std::string vec_json(const float *v, int n) {
    std::string s = "[";
    for (int i = 0; i < n; i++) {
        if (i) s += ",";
        s += fnum((double)v[i]);
    }
    s += "]";
    return s;
}

// A non-unit origin quaternion would make world_origin_rot_inv a bad inverse
// for every RAW->SCENE consumer, so the capture waits for a sane one.
bool quat_near_unit(const float q[4]) {
    float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    return n >= 0.9f && n <= 1.1f;
}

float panel_height_m(const panel &p) {
    return p.width_m * (float)p.height_px / (float)p.width_px;
}

// Panel placement matrix from a full orientation: rows are the panel's own
// basis (right / up / front) in the scene frame, translation last — the same
// row-major, row-vector storage plane_to_matrix writes, so every consumer
// (panel_hit_point, the renderer, window_json's quat) reads it unchanged.
// The panel's +Z is its face, which is why identity faces +Z.
void matrix_from_quat_pos(const float q[4], const float pos[3],
                          float out[16]) {
    const float basis[3][3] = {
        {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    for (int row = 0; row < 3; row++) {
        float v[3];
        quat_rotate_vec(q, basis[row], v);
        out[row * 4 + 0] = v[0];
        out[row * 4 + 1] = v[1];
        out[row * 4 + 2] = v[2];
        out[row * 4 + 3] = 0.0f;
    }
    out[12] = pos[0];
    out[13] = pos[1];
    out[14] = pos[2];
    out[15] = 1.0f;
}

// Where a pinch lands on a panel's quad, in surface pixels (origin top-left,
// Y down — the frame capture.mm's click injection and click_panel both take).
//
// `direct` projects the pinch point straight down the panel normal (the user
// is touching the quad); otherwise the head->point ray is intersected with
// the panel plane. False when the hit falls outside the quad, so the caller
// can fall back to focus-only.
bool panel_hit_point(const panel &p, const float head[3],
                     const float point[3], bool direct, float out_hit[3],
                     float *out_x, float *out_y, float *out_distance) {
    const float right[3] = {p.m[0], p.m[1], p.m[2]};
    const float up[3] = {p.m[4], p.m[5], p.m[6]};
    const float normal[3] = {p.m[8], p.m[9], p.m[10]};
    const float origin[3] = {p.m[12], p.m[13], p.m[14]};

    float hit[3];
    if (direct) {
        float rel[3];
        v3sub(point, origin, rel);
        float d = v3dot(rel, normal);
        for (int i = 0; i < 3; i++)
            hit[i] = point[i] - d * normal[i];
    } else {
        float dir[3];
        v3sub(point, head, dir);
        if (v3normalize(dir) <= 1e-5f)
            return false;
        float denom = v3dot(dir, normal);
        if (std::fabs(denom) < 1e-5f)
            return false;  // ray runs along the panel plane
        float rel[3];
        v3sub(origin, head, rel);
        float t = v3dot(rel, normal) / denom;
        if (t <= 0.0f)
            return false;
        if (out_distance) *out_distance = t;
        for (int i = 0; i < 3; i++)
            hit[i] = head[i] + t * dir[i];
    }

    float local[3];
    v3sub(hit, origin, local);
    float lx = v3dot(local, right), ly = v3dot(local, up);
    float hw = p.width_m * 0.5f, hh = panel_height_m(p) * 0.5f;
    if (!(hw > 0.0f) || !(hh > 0.0f))
        return false;
    if (lx < -hw || lx > hw || ly < -hh || ly > hh)
        return false;
    if (out_hit)
        v3copy(hit, out_hit);
    if (out_x && out_y) {
        float px = (lx / (hw * 2.0f) + 0.5f) * (float)p.width_px;
        float py = (0.5f - ly / (hh * 2.0f)) * (float)p.height_px;
        *out_x = std::fmax(0.0f, std::fmin(px, (float)p.width_px - 1.0f));
        *out_y = std::fmax(0.0f, std::fmin(py, (float)p.height_px - 1.0f));
    }
    return true;
}

bool panel_hit_px(const panel &p, const float head[3], const float point[3],
                  bool direct, float *out_x, float *out_y,
                  float *out_distance = nullptr) {
    return panel_hit_point(p, head, point, direct, nullptr, out_x, out_y,
                           out_distance);
}

// Where the head->aim ray first meets a detected plane, in the scene frame.
// A hit inside a plane's extent wins over one that only grazes its infinite
// plane, so a ray pointed at the desk lands on the desk and not on the wall
// behind it. False when the ray meets nothing in front of the user.
bool ray_plane_hit(const anchor_env &env, const sb_plane_t *planes,
                   int n_planes, const float origin[3], const float dir[3],
                   float out_hit[3], float *out_normal = nullptr) {
    float best_t = 0.0f, best_any_t = 0.0f;
    float best_n[3] = {0.0f, 1.0f, 0.0f}, best_any_n[3] = {0.0f, 1.0f, 0.0f};
    bool have = false, have_any = false;
    for (int i = 0; i < n_planes; i++) {
        if (planes[i].is_removed)
            continue;
        float center[3], normal[3], right[3], fwd[3];
        plane_to_scene_basis(env, planes[i], center, normal, right, fwd);
        float denom = v3dot(dir, normal);
        if (std::fabs(denom) < 1e-5f)
            continue;
        float rel[3];
        v3sub(center, origin, rel);
        float t = v3dot(rel, normal) / denom;
        if (!(t > 0.0f) || !std::isfinite(t))
            continue;
        if (!have_any || t < best_any_t) {
            best_any_t = t;
            v3copy(normal, best_any_n);
            have_any = true;
        }
        float hit[3] = {origin[0] + t * dir[0], origin[1] + t * dir[1],
                        origin[2] + t * dir[2]};
        float local[3];
        v3sub(hit, center, local);
        if (std::fabs(v3dot(local, right)) > planes[i].extent[0] * 0.5f ||
            std::fabs(v3dot(local, fwd)) > planes[i].extent[1] * 0.5f)
            continue;
        if (!have || t < best_t) {
            best_t = t;
            v3copy(normal, best_n);
            have = true;
        }
    }
    if (!have && !have_any)
        return false;
    float t = have ? best_t : best_any_t;
    for (int i = 0; i < 3; i++)
        out_hit[i] = origin[i] + t * dir[i];
    if (out_normal) {
        v3copy(have ? best_n : best_any_n, out_normal);
        // plane_to_scene_basis hands back ARKit's own normal, which may point
        // away from the caster; `cast` answers with the face the ray met.
        float back[3];
        v3sub(origin, out_hit, back);
        if (v3dot(out_normal, back) < 0.0f)
            for (int i = 0; i < 3; i++)
                out_normal[i] = -out_normal[i];
    }
    return true;
}

// Whether a scene-frame point sits within the key field's 2D extent, ignoring
// height above the plane (the summon pose is judged by where the palm hovers).
bool point_over_keyboard(const keyboard_plane &kp, const float point[3]) {
    float off[3];
    v3sub(point, kp.origin, off);
    float u = v3dot(off, kp.right), v = v3dot(off, kp.down);
    return u >= 0.0f && u <= KBD_GEOM_WIDTH_M && v >= 0.0f &&
           v <= KBD_GEOM_HEIGHT_M;
}

// "event <what> handle=<h>" — the shape shared by map/unmap/focus events.
std::string handle_event(const char *what, uint64_t h) {
    char line[64];
    std::snprintf(line, sizeof(line), "event %s handle=%llu", what,
                  (unsigned long long)h);
    return line;
}

}  // namespace

scene::scene() {
    engine_ = ge_create();
    if (engine_) {
        ge_load_config(engine_, nullptr);  // user overrides if present
        ge_set_callback(engine_, &scene::gesture_trampoline, this);
    }
    // Both callbacks fire with mutex_ already held (tick / control paths).
    keyboard_.set_emit_callback(
        [this](uint32_t keysym, const char *label, bool repeat) {
            on_key_emit(keysym, label, repeat);
        });
    launcher_.set_launch_callback(
        [this](const launcher_entry &e) { on_launcher_launch(e); });
}

scene::~scene() {
    if (engine_)
        ge_destroy(engine_);
}

void scene::set_receiver(sb_receiver_t *receiver) {
    std::lock_guard<std::mutex> lock(mutex_);
    receiver_ = receiver;
}

// ------------------------------------------------------------------
// per-tick update
// ------------------------------------------------------------------

void scene::world_frame_reset(uint32_t epoch) {
    env_.have_world_origin = false;
    clear_head_ring();
    for (auto &p : panels_) {
        if (!p->has_anchor)
            continue;
        p->has_anchor = false;
        p->reanchor_pending = true;
        p->anchor_offset[0] = p->anchor_offset[1] = p->anchor_offset[2] = 0.0f;
        std::memset(p->anchor_uuid, 0, sizeof(p->anchor_uuid));
        v3copy(p->pos, p->target_pos);
    }
    toasts_.push("Tracking reset — re-anchoring panels");
    char line[64];
    std::snprintf(line, sizeof(line), "event tracking-reset epoch=%u", epoch);
    push_event(line);
}

// A pose-stream gap means the iPhone app probably restarted its ARKit session
// (fresh origin, tens of cm away from ours), so drop the cached origin and let
// the next good pose re-capture it rather than teleporting the world. Mirrors
// vendor/wxrd/src/main.c:624-644 + 783-814. Only a NEW timestamp counts as a
// fresh sample: sb_get_latest_pose is sticky and re-serves the last pose every
// tick while the stream is down, which would otherwise mask every gap.
bool scene::maybe_recapture_origin(uint64_t pose_ts_ns) {
    if (last_fresh_pose_s_ >= 0.0) {
        if (pose_ts_ns == last_fresh_pose_ts_)
            return false;
        if (now_s_ - last_fresh_pose_s_ > POSE_GAP_RECAPTURE_S) {
            // No panel velocity to clear here any more: the only move spring
            // left is the renderer's, and it re-aims at the new target.
            env_.have_world_origin = false;
            clear_head_ring();
        }
    }
    last_fresh_pose_ts_ = pose_ts_ns;
    last_fresh_pose_s_ = now_s_;
    return true;
}

void scene::clear_head_ring() {
    head_ring_count_ = 0;
    head_ring_next_ = 0;
}

void scene::push_head_sample(uint64_t ts_ns, const float pos[3],
                             const float quat[4]) {
    head_sample &s = head_ring_[head_ring_next_];
    s.ts_ns = ts_ns;
    v3copy(pos, s.pos);
    for (int i = 0; i < 4; i++)
        s.quat[i] = quat[i];
    head_ring_next_ = (head_ring_next_ + 1) % HEAD_POSE_RING;
    if (head_ring_count_ < HEAD_POSE_RING)
        head_ring_count_++;
}

void scene::update_head(const sb_pose_t &pose) {
    const bool fresh_ts = maybe_recapture_origin(pose.timestamp_ns);
    // Epoch 0 means "unknown / pre-epoch sender", so it is inert in BOTH
    // directions: a legacy stream must not look like a reset on its first
    // packet, and a sender that stops stamping must not fake one either.
    if (pose.session_epoch != 0 && session_epoch_ != 0 &&
        pose.session_epoch != session_epoch_)
        world_frame_reset(pose.session_epoch);
    session_epoch_ = pose.session_epoch;

    last_pose_ = pose;
    have_pose_ = true;

    if (!env_.have_world_origin && pose.tracking_quality >= 0.5f &&
        quat_near_unit(pose.rot)) {
        v3copy(pose.pos, env_.world_origin_pos);
        // Yaw only: subtracting the full session-start rotation would tilt the
        // SCENE frame by whatever pitch/roll the phone happened to have, and
        // every {0,1,0} "up" in the anchor/placement/keyboard math would lean
        // with it. See docs/coordinate-systems.md section 9.
        float yaw_q[4];
        quat_yaw_only(pose.rot, yaw_q);
        env_.world_origin_rot_inv[0] = -yaw_q[0];
        env_.world_origin_rot_inv[1] = -yaw_q[1];
        env_.world_origin_rot_inv[2] = -yaw_q[2];
        env_.world_origin_rot_inv[3] = yaw_q[3];
        env_.have_world_origin = true;
    }

    world_to_scene(pose.pos, env_.head_pos);

    if (fresh_ts && env_.have_world_origin) {
        float scene_q[4];
        quat_mul(env_.world_origin_rot_inv, pose.rot, scene_q);
        push_head_sample(pose.timestamp_ns, env_.head_pos, scene_q);
    }
}

void scene::world_to_scene(const float in[3], float out[3]) const {
    float tmp[3] = {in[0], in[1], in[2]};
    if (env_.have_world_origin) {
        tmp[0] -= env_.world_origin_pos[0];
        tmp[1] -= env_.world_origin_pos[1];
        tmp[2] -= env_.world_origin_pos[2];
        quat_rotate_vec(env_.world_origin_rot_inv, tmp, tmp);
    }
    v3copy(tmp, out);
}

void scene::tick(float dt) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        tick_locked(dt);
    }
    process_pending_launches();
}

void scene::tick_locked(float dt) {
    now_s_ += dt;
    toasts_.tick(dt);
    // A fresh `hands-inject` outranks the phone: mac-side MediaPipe is the
    // better tracker, and mixing the two sources frame-to-frame would make
    // every gesture threshold chatter. Checked before the receiver drain so
    // it works with or without one.
    const bool injected = adopt_injected_hands();
    if (receiver_) {
        sb_pose_t pose;
        if (sb_get_latest_pose(receiver_, &pose))
            update_head(pose);

        sb_plane_t planes[SCENE_MAX_PLANES];
        int n = sb_get_planes(receiver_, planes, SCENE_MAX_PLANES);
        if (n >= 0)
            ingest_planes(planes, n);

        if (!injected) {
            for (int slot = 0; slot < 2; slot++) {
                sb_hand_t hand;
                if (sb_get_hand(receiver_, slot, &hand))
                    ingest_hand(slot, hand);
            }
        }

        // After update_head, so the pose ring already carries the sample the
        // depth frame's timestamp interpolates against.
        ingest_depth();
    }

    for (int slot = 0; slot < 2; slot++) {
        hand_age_s_[slot] += dt;
        hands_[slot].present = hand_age_s_[slot] <= HAND_STALE_S;
        if (hands_[slot].present)
            std::memcpy(hands_[slot].joints, hand_raw_[slot].joints,
                        sizeof(hands_[slot].joints));
    }

    // One physical hand streamed under both slots (chirality flip-flop on the
    // sender) shows up as two nearly coincident wrists; keep only the fresher.
    if (hands_[0].present && hands_[1].present) {
        const float *w0 = hand_raw_[0].joints[SB_JOINT_WRIST];
        const float *w1 = hand_raw_[1].joints[SB_JOINT_WRIST];
        float dx = w0[0] - w1[0], dy = w0[1] - w1[1], dz = w0[2] - w1[2];
        if (dx * dx + dy * dy + dz * dz < 0.10f * 0.10f) {
            int staler = hand_age_s_[0] >= hand_age_s_[1] ? 0 : 1;
            hands_[staler].present = false;
        }
    }

    if (engine_)
        ge_update(engine_, hands_, dt);  // gesture events fire into on_gesture

    bool present[2] = {hands_[0].present, hands_[1].present};
    keyboard_.update(hand_raw_, present, dt);

    if (launcher_.visible() && launcher_engine_hand_ >= 0 && engine_) {
        int slot = ge_hand_input_slot(engine_, launcher_engine_hand_);
        if (slot >= 0 && hands_[slot].present) {
            const auto &j = hand_raw_[slot].joints;
            // Wire slot is first-seen order, not handedness; a wrong guess
            // mirrors the scrub for the whole session.
            ge_chirality_t ch = ge_hand_chirality(&hands_[slot]);
            if (ch != GE_CHIRALITY_UNKNOWN)
                launcher_hand_left_ = ch == GE_CHIRALITY_LEFT;
            launcher_.update_rotation(j[SB_JOINT_WRIST], j[SB_JOINT_INDEX_MCP],
                                      j[SB_JOINT_MIDDLE_MCP],
                                      j[SB_JOINT_PINKY_MCP],
                                      launcher_hand_left_);
        }
    }

    // Launcher dismiss timers: tracking-loss grace fades out without
    // committing; a committed release lingers with the wedge highlighted.
    auto run_dismiss_timer = [&](float &left_s) {
        if (left_s < 0.0f)
            return;
        left_s -= dt;
        if (left_s >= 0.0f)
            return;
        left_s = -1.0f;
        if (launcher_.visible()) {
            launcher_.hide();
            push_event("event launcher hidden");
        }
    };
    run_dismiss_timer(launcher_grace_left_s_);
    run_dismiss_timer(launcher_linger_left_s_);

    update_aim();
    update_hud_arcs();

    refresh_anchor_transforms();

    for (auto &p : panels_) {
        if (!p->surface_dirty)
            continue;
        if (p->kind == panel_kind::note) {
            render_note_card(p->surface, p->title, p->note_body,
                             p->note_accent);
            p->surface_dirty = false;
            p->surface_version++;
            continue;
        }
        if (p->kind != panel_kind::internal_test_card)
            continue;
        std::vector<std::string> tail;
        size_t start = p->input_log.size() > PANEL_INPUT_LOG_TAIL
                           ? p->input_log.size() - PANEL_INPUT_LOG_TAIL
                           : 0;
        for (size_t i = start; i < p->input_log.size(); i++)
            tail.push_back(p->input_log[i]);
        render_test_card(p->surface, p->handle, p->title, tail,
                         p->handle == focused_);
        p->surface_dirty = false;
        p->surface_version++;
    }
}

void scene::refresh_anchor_transforms() {
    for (auto &p : panels_) {
        // Re-acquire an anchor in the new world frame once the origin is
        // re-captured and planes have been re-broadcast. Finding nothing is a
        // valid outcome (the panel floats); we keep looking because ARKit
        // restates planes at only 1–5 Hz after a reset.
        if (p->reanchor_pending && env_.have_world_origin && p->handle != grabbed_) {
            float offset[3];
            int idx = anchors_snap_core(env_, p->pos, planes_, n_planes_, -1, offset);
            if (idx >= 0) {
                std::memcpy(p->anchor_uuid, planes_[idx].uuid, 16);
                v3copy(offset, p->anchor_offset);
                p->has_anchor = true;
                p->reanchor_pending = false;
            }
        }
        if (!p->has_anchor && p->has_pose_quat) {
            matrix_from_quat_pos(p->pose_quat, p->pos, p->m);
            continue;
        }
        if (!p->has_anchor) {
            // Yaw about +Y (facing the head at spawn/gather; identity when
            // yaw == 0), translation = pos. Row-vector convention.
            float sy = std::sin(p->yaw), cy = std::cos(p->yaw);
            float m[16] = {cy,        0, -sy,       0,   // right
                           0,         1, 0,         0,   // up
                           sy,        0, cy,        0,   // front
                           p->pos[0], p->pos[1], p->pos[2], 1};
            std::memcpy(p->m, m, sizeof(m));
            continue;
        }
        const sb_plane_t *match = nullptr;
        for (int i = 0; i < n_planes_; i++) {
            if (planes_[i].is_removed)
                continue;
            if (std::memcmp(planes_[i].uuid, p->anchor_uuid, 16) == 0) {
                match = &planes_[i];
                break;
            }
        }
        if (!match)
            continue;  // plane not yet detected or removed — keep last pose
        float m[16];
        plane_to_matrix(env_, *match, p->anchor_offset, m);
        bool finite = true;
        for (float x : m)
            finite = finite && std::isfinite(x);
        if (!finite)
            continue;
        std::memcpy(p->m, m, sizeof(m));
        p->pos[0] = p->m[12];
        p->pos[1] = p->m[13];
        p->pos[2] = p->m[14];
        v3copy(p->pos, p->target_pos);
    }
}

// ------------------------------------------------------------------
// gestures
// ------------------------------------------------------------------

// Targeting shared by the pinch handler and the per-tick aim feedback: the
// pinch must actually aim at the panel. Direct touch (pinch point near the
// panel) wins outright; otherwise the panel must sit near the head->pinch ray
// and IN FRONT of the user — never target a panel behind you.
panel *scene::pick_aim_panel(const float point[3],
                             bool *out_direct) const {
    float ray[3];
    v3sub(point, env_.head_pos, ray);
    bool have_ray = v3normalize(ray) > 1e-5f;
    panel *best = nullptr;
    bool best_direct = false;
    int best_rank = 3;
    float best_score = FOCUS_RAY_MAX_PERP_M;
    // Real quad hits outrank proximity-only focus; direct touch wins over rays.
    auto consider = [&](panel *candidate, bool direct, int rank, float score) {
        if (rank < best_rank || (rank == best_rank && score < best_score)) {
            best = candidate;
            best_direct = direct;
            best_rank = rank;
            best_score = score;
        }
    };
    for (auto &p : panels_) {
        float to_p[3];
        v3sub(p->pos, point, to_p);
        float touch_d = v3length(to_p);
        float px, py;
        if (touch_d < FOCUS_TOUCH_DIST_M) {
            if (panel_hit_px(*p, env_.head_pos, point, true, &px, &py)) {
                consider(p.get(), true, 0, touch_d);
                continue;
            }
            consider(p.get(), true, 2, touch_d - FOCUS_TOUCH_DIST_M);
        }
        if (!have_ray)
            continue;
        float from_head[3];
        v3sub(p->pos, env_.head_pos, from_head);
        float t = v3dot(from_head, ray);
        float hit_distance = 0.0f;
        if (panel_hit_px(*p, env_.head_pos, point, false, &px, &py, &hit_distance)) {
            if (hit_distance <= FOCUS_MAX_DIST_M)
                consider(p.get(), false, 1, hit_distance);
            continue;
        }
        if (t <= 0.0f || t > FOCUS_MAX_DIST_M)
            continue;
        float perp[3] = {from_head[0] - t * ray[0], from_head[1] - t * ray[1],
                         from_head[2] - t * ray[2]};
        float pd = v3length(perp);
        if (pd < FOCUS_RAY_MAX_PERP_M)
            consider(p.get(), false, 2, pd);
    }
    if (out_direct)
        *out_direct = best_direct;
    return best;
}

bool scene::aim_point(float out[3]) const {
    int slot = -1;
    for (int i = 0; i < 2; i++) {
        if (!hands_[i].present || hand_age_s_[i] > HAND_STALE_S)
            continue;
        const auto &j = hand_raw_[i].joints;
        if (!(j[SB_JOINT_THUMB_TIP][3] >= AIM_MIN_CONFIDENCE) ||
            !(j[SB_JOINT_INDEX_TIP][3] >= AIM_MIN_CONFIDENCE))
            continue;
        if (slot < 0 || hand_age_s_[i] < hand_age_s_[slot])
            slot = i;
    }
    if (slot < 0)
        return false;
    const auto &j = hand_raw_[slot].joints;
    for (int i = 0; i < 3; i++)
        out[i] = 0.5f * (j[SB_JOINT_THUMB_TIP][i] + j[SB_JOINT_INDEX_TIP][i]);
    return v3finite(out);
}

void scene::update_aim() {
    float point[3];
    uint64_t next = 0;
    if (!keyboard_.visible() && !launcher_.visible() && aim_point(point)) {
        if (panel *p = pick_aim_panel(point, nullptr))
            next = p->handle;
    }
    aimed_ = next;
}

void scene::gesture_trampoline(const ge_event_t *ev, void *user) {
    static_cast<scene *>(user)->on_gesture(*ev);
}

void scene::on_gesture(const ge_event_t &ev) {
    // mutex_ is already held: ge_update is only called from tick().
    const char *phase = ev.type == GE_EVENT_BEGIN    ? "begin"
                        : ev.type == GE_EVENT_UPDATE ? "update"
                        : ev.type == GE_EVENT_END    ? "end"
                                                     : "cancel";
    if (ev.type != GE_EVENT_UPDATE) {
        char line[160];
        std::snprintf(line, sizeof(line), "event gesture name=%s phase=%s",
                      ev.gesture_name ? ev.gesture_name : "?", phase);
        push_event(line);
    }

    // While typing or scrubbing the launcher, hand poses overlap the pointer
    // vocabulary — suppress window manipulation so taps/rolls can't move or
    // close panels underneath.
    bool overlay_active = keyboard_.visible() || launcher_.visible();

    // Pinch state is physical: it holds even while an overlay owns the
    // pointer vocabulary. Broadcast separately from the gesture line because
    // that one carries the winning VARIANT ("pinch_select.loose"), so a
    // subscriber matching on the plain name misses most pinches.
    if (ev.action == GE_ACTION_POINTER_CLICK) {
        if (ev.type == GE_EVENT_BEGIN) {
            pinch_held_ = true;
            push_event("event pinch phase=begin");
        } else if (ev.type == GE_EVENT_END || ev.type == GE_EVENT_CANCEL) {
            pinch_held_ = false;
            push_event("event pinch phase=end");
        }
    }

    switch (ev.action) {
        case GE_ACTION_TOGGLE_LAUNCHER: {
            // The engine's 300 ms fist min-hold pre-arms the menu; here we
            // keep it from being twitchy on the way OUT: tracking loss
            // (CANCEL) starts a grace window instead of dismissing, and a
            // committed release lingers with the wedge highlighted.
            if (ev.type == GE_EVENT_BEGIN) {
                if (launcher_.visible() && launcher_grace_left_s_ >= 0.0f) {
                    // Fist re-acquired within the grace: resume, keep focus.
                    launcher_engine_hand_ = ev.hand_index;
                    launcher_.begin_rotation();
                    launcher_grace_left_s_ = -1.0f;
                } else if (launcher_.visible() && launcher_engine_hand_ < 0 &&
                           launcher_linger_left_s_ < 0.0f) {
                    // Menu up and unattended: a second fist puts it away.
                    launcher_grace_left_s_ = -1.0f;
                    launcher_.hide();
                    push_event("event launcher hidden");
                } else {
                    launcher_engine_hand_ = ev.hand_index;
                    launcher_.begin_rotation();
                    if (launcher_linger_left_s_ >= 0.0f) {
                        launcher_linger_left_s_ = -1.0f;
                        launcher_.hide();  // finish the previous linger
                    }
                    launcher_grace_left_s_ = -1.0f;
                    launcher_.show();
                    push_event("event launcher shown");
                }
            } else if (ev.type == GE_EVENT_END) {
                bool was_visible = launcher_.visible();
                bool committed = launcher_.end_rotation_commit_keep_visible();
                launcher_engine_hand_ = -1;
                launcher_grace_left_s_ = -1.0f;
                launcher_linger_left_s_ =
                    committed ? theme::LAUNCHER_COMMIT_LINGER_S : -1.0f;
                if (was_visible && !committed)
                    push_event("event launcher hidden");
            } else if (ev.type == GE_EVENT_CANCEL) {
                // Hand vanished — don't dismiss for a few dropped frames.
                launcher_engine_hand_ = -1;
                if (launcher_.visible() && launcher_linger_left_s_ < 0.0f)
                    launcher_grace_left_s_ = theme::LAUNCHER_GRACE_S;
            }
            break;
        }
        case GE_ACTION_KEYBOARD_ANCHOR: {
            if (ev.type != GE_EVENT_BEGIN)
                break;
            // Hands resting flat ON the board satisfy the palm-down summon
            // pose, so an unguarded toggle dismisses the keyboard mid-typing.
            // Dismissal still works — from a palm held off the key field.
            if (keyboard_.visible() &&
                point_over_keyboard(keyboard_.plane(), ev.position))
                break;
            keyboard_toggle_impl(!keyboard_.visible());
            break;
        }
        case GE_ACTION_POINTER_CLICK:
        case GE_ACTION_POINTER_RIGHT_CLICK: {
            if (ev.type != GE_EVENT_BEGIN || overlay_active)
                break;
            bool direct = false;
            panel *best = pick_aim_panel(ev.position, &direct);
            if (best) {
                focus_locked(best->handle);
                log_input(*best, "pinch-click");
                float px = 0.0f, py = 0.0f;
                if (panel_hit_px(*best, env_.head_pos, ev.position, direct,
                                 &px, &py)) {
                    int button =
                        ev.action == GE_ACTION_POINTER_RIGHT_CLICK ? 1 : 0;
                    click_panel_locked(best->handle, px, py, button);
                }
            } else {
                // Empty-space pinch: two in quick succession recall lost
                // panels ("where did my window go" rescue).
                if (now_s_ - last_empty_pinch_s_ <=
                    (double)theme::DOUBLE_PINCH_GATHER_S) {
                    last_empty_pinch_s_ = -1e9;
                    gather_panels_impl();
                } else {
                    last_empty_pinch_s_ = now_s_;
                }
            }
            break;
        }
        case GE_ACTION_WINDOW_MOVE: {
            if (overlay_active)
                break;
            if (ev.type == GE_EVENT_BEGIN) {
                grabbed_ = focused_;
                grab_accum_[0] = grab_accum_[1] = grab_accum_[2] = 0.0f;
                grab_engaged_ = false;
                if (panel *p = find_panel(grabbed_)) {
                    p->has_anchor = false;  // lift off while grabbed
                    p->reanchor_pending = false;
                    v3copy(p->pos, p->target_pos);
                }
            } else if (ev.type == GE_EVENT_UPDATE) {
                panel *p = find_panel(grabbed_);
                if (!p)
                    break;
                if (grab_engaged_) {
                    p->target_pos[0] += ev.delta[0];
                    p->target_pos[1] += ev.delta[1];
                    p->target_pos[2] += ev.delta[2];
                    v3copy(p->target_pos, p->pos);
                    break;
                }
                // Deadband: the panel target holds still until the hand has
                // moved GRAB_DEADBAND_M from the grab point.  On breakout,
                // carry the overshoot (accum minus the deadband along its
                // direction) so the panel doesn't jump.
                grab_accum_[0] += ev.delta[0];
                grab_accum_[1] += ev.delta[1];
                grab_accum_[2] += ev.delta[2];
                float alen = v3length(grab_accum_);
                if (alen >= GRAB_DEADBAND_M) {
                    grab_engaged_ = true;
                    float k = (alen - GRAB_DEADBAND_M) / alen;
                    p->target_pos[0] += grab_accum_[0] * k;
                    p->target_pos[1] += grab_accum_[1] * k;
                    p->target_pos[2] += grab_accum_[2] * k;
                    v3copy(p->target_pos, p->pos);
                }
            } else {  // END or CANCEL
                if (ev.type == GE_EVENT_END) {
                    if (panel *p = find_panel(grabbed_)) {
                        // Snap decision uses where the user dropped it.
                        float offset[3];
                        int idx = anchors_snap_core(env_, p->target_pos,
                                                    planes_, n_planes_, -1,
                                                    offset);
                        if (idx >= 0) {
                            std::memcpy(p->anchor_uuid, planes_[idx].uuid, 16);
                            v3copy(offset, p->anchor_offset);
                            p->has_anchor = true;
                        }
                    }
                }
                grabbed_ = 0;
                grab_engaged_ = false;
            }
            break;
        }
        case GE_ACTION_WINDOW_CLOSE: {
            if (ev.type != GE_EVENT_BEGIN || overlay_active)
                break;
            if (panel *p = find_panel(focused_)) {
                uint64_t h = p->handle;
                panels_.erase(
                    std::remove_if(panels_.begin(), panels_.end(),
                                   [h](const std::unique_ptr<panel> &q) {
                                       return q->handle == h;
                                   }),
                    panels_.end());
                focused_ = 0;
                push_event(handle_event("window-unmap", h));
            }
            break;
        }
        default:
            break;  // scroll/slider/launcher not routed in Stage 1
    }
}

// ------------------------------------------------------------------
// test injection
// ------------------------------------------------------------------

void scene::inject_pose(const sb_pose_t &pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    update_head(pose);
}

void scene::inject_planes(const sb_plane_t *planes, int n) {
    std::lock_guard<std::mutex> lock(mutex_);
    ingest_planes(planes, n);
}

void scene::ingest_planes(const sb_plane_t *planes, int n) {
    n = std::min(n, SCENE_MAX_PLANES);
    int kept = 0;
    for (int i = 0; i < n; i++) {
        const sb_plane_t &pl = planes[i];
        if (!v3finite(pl.center) || !v3finite(pl.normal) ||
            !std::isfinite(pl.extent[0]) || !std::isfinite(pl.extent[1]))
            continue;
        planes_[kept++] = pl;
    }
    n_planes_ = kept;
}

void scene::ingest_depth() {
    sb_intrinsics_t intr;
    if (sb_get_latest_intrinsics(receiver_, &intr)) {
        latest_depth_.intr = intr;
        latest_depth_.have_intrinsics = true;
    }
    sb_depth_t d;
    if (!sb_get_latest_depth(receiver_, &d))
        return;
    if (d.depth && d.width > 0 && d.height > 0 &&
        d.depth_count == d.width * d.height) {
        latest_depth_.depth.assign(d.depth, d.depth + d.depth_count);
        latest_depth_.width = d.width;
        latest_depth_.height = d.height;
        latest_depth_.ts_ns = d.timestamp_ns;
        latest_depth_.version++;
        // The map is a snapshot of the room from where the camera was when it
        // was exposed, not from where the head is now; casting against it from
        // the newest pose would shear the whole room on a head turn.
        latest_depth_.have_pose = head_pose_at_locked(
            d.timestamp_ns, latest_depth_.cam_pos, latest_depth_.cam_quat,
            nullptr);
    }
    sb_free_depth(&d);
}

void scene::ingest_hand(int slot, const sb_hand_t &hand, bool already_scene) {
    for (int j = 0; j < SB_HAND_JOINT_COUNT; j++)
        if (!v3finite(hand.joints[j]))
            return;
    hand_raw_[slot] = hand;
    if (!already_scene)
        hand_to_scene(hand_raw_[slot]);
    hand_age_s_[slot] = 0.0f;
}

bool scene::adopt_injected_hands() {
    if (!hand_inject_.fresh(hand_inject_now_ms()))
        return false;
    const injected_hands &inj = hand_inject_.hands();
    for (int i = 0; i < inj.count; i++)
        ingest_hand(i, inj.hands[i], true);
    return true;
}

bool scene::hands_inject(const std::string &json, std::string &err) {
    injected_hands parsed;
    if (!parse_hands_inject(json, parsed, err))
        return false;
    std::lock_guard<std::mutex> lock(mutex_);
    hand_inject_.set(parsed);
    return true;
}

std::string scene::hands_source_status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    const uint64_t now = hand_inject_now_ms();
    char buf[64];
    std::snprintf(buf, sizeof(buf), "source=%s age_ms=%llu",
                  hand_inject_.fresh(now) ? "mac" : "phone",
                  (unsigned long long)hand_inject_.age_ms(now));
    return buf;
}

std::string scene::hands_dump_json() const {
    std::lock_guard<std::mutex> lock(mutex_);
    const uint64_t now = hand_inject_now_ms();
    std::string out = "{\"source\":\"";
    out += hand_inject_.fresh(now) ? "mac" : "phone";
    out += "\",\"age_ms\":";
    out += std::to_string((unsigned long long)hand_inject_.age_ms(now));
    out += ",\"frame\":\"scene\",\"hands\":[";
    bool first = true;
    for (int slot = 0; slot < 2; slot++) {
        if (hand_age_s_[slot] > HAND_STALE_S)
            continue;
        if (!first)
            out += ',';
        first = false;
        char buf[96];
        std::snprintf(buf, sizeof(buf),
                      "{\"slot\":%d,\"age_s\":%.3f,\"joints\":[", slot,
                      (double)hand_age_s_[slot]);
        out += buf;
        // MediaPipe landmark order, so a consumer can line these up with the
        // mac tracker's own output without a second mapping table.
        for (int mp = 0; mp < SB_HAND_JOINT_COUNT; mp++) {
            const float *j = hand_raw_[slot].joints[mediapipe_to_sb_joint(mp)];
            std::snprintf(buf, sizeof(buf), "%s[%.5f,%.5f,%.5f,%.3f]",
                          mp ? "," : "", (double)j[0], (double)j[1],
                          (double)j[2], (double)j[3]);
            out += buf;
        }
        out += "]}";
    }
    out += "]}";
    return out;
}

void scene::inject_gesture(const ge_event_t &ev) {
    std::lock_guard<std::mutex> lock(mutex_);
    on_gesture(ev);
}

void scene::inject_hand(int slot, const sb_hand_t &hand) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (slot < 0 || slot > 1)
        return;
    ingest_hand(slot, hand);
}

// Rewrites joints[j][0..2] in place; confidence (joints[j][3]) is untouched.
void scene::hand_to_scene(sb_hand_t &hand) const {
    for (int j = 0; j < SB_HAND_JOINT_COUNT; j++) {
        float p[3] = {hand.joints[j][0], hand.joints[j][1], hand.joints[j][2]};
        world_to_scene(p, p);
        hand.joints[j][0] = p[0];
        hand.joints[j][1] = p[1];
        hand.joints[j][2] = p[2];
    }
}

// ------------------------------------------------------------------
// control operations
// ------------------------------------------------------------------

panel *scene::find_panel(uint64_t handle) {
    for (auto &p : panels_)
        if (p->handle == handle)
            return p.get();
    return nullptr;
}

const panel *scene::find_panel(uint64_t handle) const {
    for (const auto &p : panels_)
        if (p->handle == handle)
            return p.get();
    return nullptr;
}

void scene::push_event(const std::string &line) {
    events_.push_back(line);
    while (events_.size() > 256)
        events_.pop_front();
}

void scene::focus_locked(uint64_t handle) {
    if (focused_ == handle)
        return;
    focused_ = handle;
    for (auto &p : panels_)
        p->surface_dirty = true;  // focus ring changed
    // A captured window's app routes CGEventPostToPid keystrokes to its own
    // key window, so spatial focus has to be mirrored onto the real window or
    // typing lands wherever that app was last focused.
    panel *p = find_panel(handle);
    if (p && p->kind == panel_kind::captured_window && injector_) {
        inject_event ev;
        ev.type = inject_event::kind::focus;
        injector_(p->handle, p->owner_pid, ev);
    }
    push_event(handle_event("focus", handle));
}

void scene::log_input(panel &p, const std::string &line) {
    p.input_log.push_back(line);
    while (p.input_log.size() > PANEL_INPUT_LOG_MAX)
        p.input_log.pop_front();
    p.surface_dirty = true;
}

uint64_t scene::spawn_panel(const std::string &app_id,
                            const std::string &title) {
    std::lock_guard<std::mutex> lock(mutex_);
    return spawn_panel_impl(app_id, title);
}

bool scene::scene_head_quat(float out_quat[4]) const {
    if (!have_pose_)
        return false;
    quat_mul(env_.world_origin_rot_inv, last_pose_.rot, out_quat);
    return true;
}

void scene::place_new_panel(panel &p) {
    float head_quat[4];
    if (scene_head_quat(head_quat)) {
        // In front of the CURRENT head: 0.9 m along the horizontal forward,
        // ~10 cm below the eyes, above the floor, clear of planes, facing
        // the head. Stacked spawns nudge sideways so panels don't coincide.
        spawn_tuning tune;
        spawn_pose_in_front(env_, env_.head_pos, head_quat, planes_,
                            n_planes_, tune, p.pos, &p.yaw);
        float step = SPAWN_STEP_X * (float)((p.handle - 1) % 3);
        p.pos[0] += std::cos(p.yaw) * step;  // along the panel's right
        p.pos[2] += -std::sin(p.yaw) * step;
        p.yaw = yaw_facing(p.pos, env_.head_pos);
    } else {
        // No pose yet (tests / pose-less runs): legacy spawn shelf.
        p.pos[0] = SPAWN_POS[0] + SPAWN_STEP_X * (float)((p.handle - 1) % 5);
        p.pos[1] = SPAWN_POS[1];
        p.pos[2] = SPAWN_POS[2];
        p.yaw = 0.0f;
    }
    v3copy(p.pos, p.target_pos);
}

uint64_t scene::spawn_panel_impl(const std::string &app_id,
                                 const std::string &title) {
    if (panels_.size() >= MAX_PANELS)
        return 0;
    auto p = std::make_unique<panel>();
    p->handle = next_handle_++;
    p->app_id = app_id;
    p->title = title;
    place_new_panel(*p);
    p->surface.resize(p->width_px, p->height_px);
    uint64_t h = p->handle;
    panels_.push_back(std::move(p));
    if (focused_ == 0)
        focused_ = h;
    push_event(handle_event("window-map", h));
    return h;
}

uint64_t scene::spawn_captured_panel(const std::string &app_id,
                                     const std::string &title,
                                     int32_t owner_pid, int width_px,
                                     int height_px) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (panels_.size() >= MAX_PANELS)
        return 0;
    auto p = std::make_unique<panel>();
    p->handle = next_handle_++;
    p->kind = panel_kind::captured_window;
    p->owner_pid = owner_pid;
    p->app_id = app_id;
    p->title = title;
    if (width_px > 0 && height_px > 0) {
        p->width_px = width_px;
        p->height_px = height_px;
    }
    place_new_panel(*p);
    uint64_t h = p->handle;
    panels_.push_back(std::move(p));
    if (focused_ == 0)
        focused_ = h;
    push_event(handle_event("window-map", h));
    return h;
}

uint64_t scene::spawn_note_panel(const std::string &title,
                                 const std::string &body, bool accent) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (panels_.size() >= MAX_PANELS)
        return 0;
    auto p = std::make_unique<panel>();
    p->handle = next_handle_++;
    p->kind = panel_kind::note;
    p->app_id = "note";
    p->title = title;
    p->note_body = body;
    p->note_accent = accent;
    p->width_px = NOTE_DEFAULT_W_PX;
    p->height_px = NOTE_DEFAULT_H_PX;
    place_new_panel(*p);
    p->surface.resize(p->width_px, p->height_px);
    uint64_t h = p->handle;
    panels_.push_back(std::move(p));
    if (focused_ == 0)
        focused_ = h;
    push_event(handle_event("window-map", h));
    return h;
}

bool scene::update_note(uint64_t handle, const note_patch &patch) {
    std::lock_guard<std::mutex> lock(mutex_);
    panel *p = find_panel(handle);
    if (!p || p->kind != panel_kind::note)
        return false;
    if (patch.set_title)
        p->title = patch.title;
    if (patch.set_body)
        p->note_body = patch.body;
    if (patch.set_accent)
        p->note_accent = patch.accent;
    p->surface_dirty = true;
    return true;
}

void scene::set_input_injector(injector_fn fn) {
    std::lock_guard<std::mutex> lock(mutex_);
    injector_ = std::move(fn);
}

bool scene::move_panel(uint64_t handle, const float vec[3], bool relative) {
    std::lock_guard<std::mutex> lock(mutex_);
    panel *p = find_panel(handle);
    if (!p)
        return false;
    p->reanchor_pending = false;  // explicit placement outranks a pending re-snap
    // Control-plane moves are deterministic: position and animation target
    // move together, no spring transient.
    if (relative) {
        p->pos[0] += vec[0];
        p->pos[1] += vec[1];
        p->pos[2] += vec[2];
    } else {
        v3copy(vec, p->pos);
    }
    v3copy(p->pos, p->target_pos);
    return true;
}

bool scene::pose_panel(uint64_t handle, const float pos[3],
                       const float quat[4]) {
    std::lock_guard<std::mutex> lock(mutex_);
    panel *p = find_panel(handle);
    if (!p)
        return false;
    if (!v3finite(pos))
        return false;
    float q[4] = {quat[0], quat[1], quat[2], quat[3]};
    float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] +
                          q[3] * q[3]);
    if (!(len > 1e-6f) || !std::isfinite(len))
        return false;
    for (int i = 0; i < 4; i++)
        q[i] /= len;

    v3copy(pos, p->pos);
    v3copy(pos, p->target_pos);
    for (int i = 0; i < 4; i++)
        p->pose_quat[i] = q[i];
    p->has_pose_quat = true;
    // An explicit pose says where this panel goes, so it stops following a
    // plane and stops waiting to re-snap to one.
    p->has_anchor = false;
    p->reanchor_pending = false;
    // Keep yaw in step with the heading part of the new orientation, so a
    // later gather or layout load starts from something sane.
    const float fwd_local[3] = {0.0f, 0.0f, 1.0f};
    float fwd[3];
    quat_rotate_vec(q, fwd_local, fwd);
    if (std::fabs(fwd[0]) > 1e-6f || std::fabs(fwd[2]) > 1e-6f)
        p->yaw = std::atan2(fwd[0], fwd[2]);
    matrix_from_quat_pos(q, p->pos, p->m);
    return true;
}

int scene::anchor_panel(uint64_t handle, int mode, const uint8_t uuid[16],
                        uint8_t out_uuid[16]) {
    std::lock_guard<std::mutex> lock(mutex_);
    panel *p = find_panel(handle);
    if (!p)
        return -1;
    p->reanchor_pending = false;

    if (mode == 0) {
        std::memcpy(p->anchor_uuid, uuid, 16);
        p->has_anchor = true;
        p->anchor_offset[0] = p->anchor_offset[1] = p->anchor_offset[2] = 0.0f;
        std::memcpy(out_uuid, p->anchor_uuid, 16);
        return 1;
    }

    int alignment = (mode == 1) ? 1 : 0;  // wall = vertical, else horizontal
    float offset[3];
    int idx = anchors_snap_core(env_, p->pos, planes_, n_planes_, alignment,
                                offset);
    if (idx < 0) {
        // Mirror anchors.c "drop in mid-air": clear any previous anchor.
        p->has_anchor = false;
        p->anchor_offset[0] = p->anchor_offset[1] = p->anchor_offset[2] = 0.0f;
        return 0;
    }
    std::memcpy(p->anchor_uuid, planes_[idx].uuid, 16);
    v3copy(offset, p->anchor_offset);
    p->has_anchor = true;
    std::memcpy(out_uuid, p->anchor_uuid, 16);
    return 1;
}

bool scene::clear_anchor(uint64_t handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    panel *p = find_panel(handle);
    if (!p)
        return false;
    p->has_anchor = false;
    p->reanchor_pending = false;
    return true;
}

int scene::query_anchor(uint64_t handle, uint8_t out_uuid[16]) {
    std::lock_guard<std::mutex> lock(mutex_);
    const panel *p = find_panel(handle);
    if (!p)
        return -1;
    if (!p->has_anchor)
        return 0;
    std::memcpy(out_uuid, p->anchor_uuid, 16);
    return 1;
}

bool scene::focus_panel(uint64_t handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    panel *p = find_panel(handle);
    if (!p)
        return false;
    focus_locked(handle);
    return true;
}

bool scene::close_panel(uint64_t handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    panel *p = find_panel(handle);
    if (!p)
        return false;
    panels_.erase(std::remove_if(panels_.begin(), panels_.end(),
                                 [handle](const std::unique_ptr<panel> &q) {
                                     return q->handle == handle;
                                 }),
                  panels_.end());
    if (focused_ == handle)
        focused_ = panels_.empty() ? 0 : panels_.back()->handle;
    if (grabbed_ == handle)
        grabbed_ = 0;
    push_event(handle_event("window-unmap", handle));
    return true;
}

bool scene::resize_panel(uint64_t handle, int width, int height) {
    std::lock_guard<std::mutex> lock(mutex_);
    panel *p = find_panel(handle);
    if (!p || width <= 0 || height <= 0)
        return false;
    p->width_px = width;
    p->height_px = height;
    p->surface.resize(width, height);
    p->surface_dirty = true;
    return true;
}

bool scene::type_text(const std::string &text) {
    std::lock_guard<std::mutex> lock(mutex_);
    panel *p = find_panel(focused_);
    if (!p)
        return false;
    log_input(*p, "type:" + text);
    if (p->kind == panel_kind::captured_window && injector_) {
        inject_event ev;
        ev.type = inject_event::kind::text;
        ev.text = text;
        injector_(p->handle, p->owner_pid, ev);
    }
    return true;
}

bool scene::key_press(const std::string &name) {
    std::lock_guard<std::mutex> lock(mutex_);
    panel *p = find_panel(focused_);
    if (!p)
        return false;
    log_input(*p, "key:" + name);
    if (p->kind == panel_kind::captured_window && injector_) {
        inject_event ev;
        ev.type = inject_event::kind::key;
        ev.text = name;
        injector_(p->handle, p->owner_pid, ev);
    }
    return true;
}

bool scene::click_panel(uint64_t handle, float x, float y, int button) {
    std::lock_guard<std::mutex> lock(mutex_);
    return click_panel_locked(handle, x, y, button);
}

bool scene::click_panel_locked(uint64_t handle, float x, float y, int button) {
    panel *p = find_panel(handle);
    if (!p)
        return false;
    focus_locked(handle);
    char buf[96];
    std::snprintf(buf, sizeof(buf), "click:%g,%g btn=%d", (double)x, (double)y,
                  button);
    log_input(*p, buf);
    if (p->kind == panel_kind::captured_window && injector_) {
        inject_event ev;
        ev.type = inject_event::kind::click;
        ev.x = x;
        ev.y = y;
        ev.button = button;
        injector_(p->handle, p->owner_pid, ev);
    }
    return true;
}

bool scene::scroll(float dx, float dy) {
    std::lock_guard<std::mutex> lock(mutex_);
    panel *p = find_panel(focused_);
    if (p) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "scroll:%g,%g", (double)dx,
                      (double)dy);
        log_input(*p, buf);
        if (p->kind == panel_kind::captured_window && injector_) {
            inject_event ev;
            ev.type = inject_event::kind::scroll;
            ev.x = dx;
            ev.y = dy;
            injector_(p->handle, p->owner_pid, ev);
        }
    }
    return true;  // control.c replies ok regardless of focus
}

int scene::gather_panels() {
    std::lock_guard<std::mutex> lock(mutex_);
    return gather_panels_impl();
}

int scene::gather_panels_impl() {
    // mutex_ held. Move every unanchored (and ungrabbed) panel into an arc
    // in front of the head; the renderer's move spring animates the flight.
    float head_quat[4];
    if (!scene_head_quat(head_quat)) {
        head_quat[0] = head_quat[1] = head_quat[2] = 0.0f;
        head_quat[3] = 1.0f;
    }
    std::vector<panel *> loose;
    for (auto &p : panels_)
        if (!p->has_anchor && p->handle != grabbed_)
            loose.push_back(p.get());
    if (loose.empty())
        return 0;
    spawn_tuning tune;
    for (size_t i = 0; i < loose.size(); i++) {
        gather_arc_position(env_, env_.head_pos, head_quat, (int)i,
                            (int)loose.size(), planes_, n_planes_, tune,
                            loose[i]->target_pos, &loose[i]->yaw);
        v3copy(loose[i]->target_pos, loose[i]->pos);
        loose[i]->has_pose_quat = false;  // gather re-derives a facing yaw
    }
    char line[64];
    std::snprintf(line, sizeof(line), "event gather-panels count=%zu",
                  loose.size());
    push_event(line);
    return (int)loose.size();
}

void scene::set_depth_occlusion_mode(int mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    depth_occlusion_mode_ = mode < 0 ? 0 : (mode > 2 ? 2 : mode);
    char line[48];
    std::snprintf(line, sizeof(line), "event depth-occlusion mode=%d",
                  depth_occlusion_mode_);
    push_event(line);
}

int scene::depth_occlusion_mode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return depth_occlusion_mode_;
}

void scene::set_hand_overlay(bool on) {
    std::lock_guard<std::mutex> lock(mutex_);
    hand_overlay_ = on;
    char line[40];
    std::snprintf(line, sizeof(line), "event hands overlay=%d", on ? 1 : 0);
    push_event(line);
}

bool scene::hand_overlay() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return hand_overlay_;
}

// ------------------------------------------------------------------
// virtual keyboard + radial launcher
// ------------------------------------------------------------------

void scene::on_key_emit(uint32_t keysym, const char *label, bool repeat) {
    // mutex_ held (called from tick or a locked control path).
    (void)repeat;
    panel *p = find_panel(focused_);
    if (p) {
        inject_event iev;
        bool mapped = true;
        if (keysym == KBD_KEYSYM_BACKSPACE || keysym == KBD_KEYSYM_RETURN) {
            const char *name =
                keysym == KBD_KEYSYM_RETURN ? "Return" : "BackSpace";
            log_input(*p, std::string("key:") + name);
            iev.type = inject_event::kind::key;
            iev.text = name;
        } else if (keysym >= 0x20 && keysym <= 0x7e) {
            // Printable ASCII only. The old 0x7f mask folded every keysym into
            // that range, so a capital or a symbol keysym would have typed a
            // wrong character silently; anything outside the range now logs
            // and is dropped instead of mistyping.
            std::string text(1, (char)keysym);
            log_input(*p, "type:" + text);
            iev.type = inject_event::kind::text;
            iev.text = text;
        } else {
            mapped = false;
            log_input(*p, "unmapped");
        }
        if (mapped && p->kind == panel_kind::captured_window && injector_)
            injector_(p->handle, p->owner_pid, iev);
    }
    char line[96];
    std::snprintf(line, sizeof(line), "event keyboard-key sym=0x%x label=%s",
                  keysym, label ? label : "?");
    push_event(line);
}

void scene::keyboard_toggle_impl(bool want_visible) {
    if (want_visible) {
        float head_quat[4] = {0.0f, 0.0f, 0.0f, 1.0f};
        scene_head_quat(head_quat);  // identity until the first pose arrives
        keyboard_.show(env_, head_quat, planes_, n_planes_);
        push_event("event keyboard shown");
    } else {
        keyboard_.hide();
        push_event("event keyboard hidden");
    }
}

bool scene::keyboard_show() {
    std::lock_guard<std::mutex> lock(mutex_);
    keyboard_toggle_impl(true);
    return keyboard_.anchored();
}

bool scene::keyboard_hide() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!keyboard_.visible())
        return false;
    keyboard_toggle_impl(false);
    return true;
}

bool scene::keyboard_visible() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keyboard_.visible();
}

std::string scene::keyboard_status_json() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keyboard_.status_json();
}

void scene::on_launcher_launch(const launcher_entry &entry) {
    // mutex_ held. Internal targets spawn synchronously; app targets are
    // queued and dispatched after the lock drops (the app launcher may spawn
    // a fallback panel synchronously, which needs the lock).
    std::string line = "event launcher-launch label=" + entry.label;
    push_event(line);
    const std::string internal_prefix = "internal:";
    if (entry.target.rfind(internal_prefix, 0) == 0) {
        spawn_panel_impl("test-card",
                         entry.target.substr(internal_prefix.size()));
    } else {
        std::string target = entry.target;
        const std::string app_prefix = "app:";
        if (target.rfind(app_prefix, 0) == 0)
            target = target.substr(app_prefix.size());
        pending_app_launches_.push_back(std::move(target));
    }
}

void scene::process_pending_launches() {
    std::vector<std::string> targets;
    app_launch_fn fn;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        targets.swap(pending_app_launches_);
        fn = app_launcher_;
    }
    for (const auto &t : targets) {
        if (fn)
            fn(t);
        else
            spawn_panel("test-card", t);  // headless fallback
    }
}

void scene::launcher_show() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        launcher_grace_left_s_ = -1.0f;
        launcher_linger_left_s_ = -1.0f;
        launcher_.show();
        push_event("event launcher shown");
    }
}

bool scene::launcher_hide() {
    std::lock_guard<std::mutex> lock(mutex_);
    launcher_grace_left_s_ = -1.0f;
    launcher_linger_left_s_ = -1.0f;
    if (!launcher_.visible())
        return false;
    launcher_.hide();
    push_event("event launcher hidden");
    return true;
}

bool scene::launcher_visible() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return launcher_.visible();
}

bool scene::launcher_select(int idx_on_page) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!launcher_.select(idx_on_page))
        return false;
    char line[96];
    std::snprintf(line, sizeof(line), "event launcher-select index=%d",
                  idx_on_page);
    push_event(line);
    return true;
}

bool scene::launcher_commit() {
    bool ok;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        launcher_grace_left_s_ = -1.0f;
        launcher_linger_left_s_ = -1.0f;
        ok = launcher_.commit();
        if (ok)
            push_event("event launcher hidden");
    }
    process_pending_launches();
    return ok;
}

std::string scene::launcher_status_json() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return launcher_.status_json();
}

void scene::set_app_launcher(app_launch_fn fn) {
    std::lock_guard<std::mutex> lock(mutex_);
    app_launcher_ = std::move(fn);
}

void scene::update_hud_arcs() {
    for (int h = 0; h < 2; h++) {
        hud_arcs_[h] = {};
        if (!engine_)
            continue;
        // Hold-progress hint: keyboard summon or the launcher fist pre-arm
        // (300 ms engine min-hold), whichever is further along.
        float prog = ge_pending_progress(engine_, "keyboard_anchor", h);
        prog = std::max(prog,
                        ge_pending_progress(engine_, "fist_launcher", h));
        if (prog <= 0.0f)
            continue;
        int slot = ge_hand_input_slot(engine_, h);
        if (slot < 0 || !hands_[slot].present)
            continue;
        // Palm centre approximation: mean of wrist + the four finger MCPs.
        const int refs[5] = {SB_JOINT_WRIST, SB_JOINT_INDEX_MCP,
                             SB_JOINT_MIDDLE_MCP, SB_JOINT_RING_MCP,
                             SB_JOINT_PINKY_MCP};
        float acc[3] = {0, 0, 0};
        for (int r : refs) {
            acc[0] += hand_raw_[slot].joints[r][0];
            acc[1] += hand_raw_[slot].joints[r][1];
            acc[2] += hand_raw_[slot].joints[r][2];
        }
        hud_arcs_[h].active = true;
        hud_arcs_[h].progress = prog;
        for (int i = 0; i < 3; i++)
            hud_arcs_[h].pos[i] = acc[i] / 5.0f;
    }
}

void scene::hud_arcs(hud_arc out[2]) const {
    std::lock_guard<std::mutex> lock(mutex_);
    out[0] = hud_arcs_[0];
    out[1] = hud_arcs_[1];
}

void scene::post_toast(const std::string &text, toast_action action) {
    std::lock_guard<std::mutex> lock(mutex_);
    toasts_.push(text, action);
}

std::vector<toast> scene::snapshot_toasts() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<toast> out;
    if (const toast *t = toasts_.front())
        out.push_back(*t);
    return out;
}

void scene::dismiss_toast() {
    std::lock_guard<std::mutex> lock(mutex_);
    toasts_.dismiss_front();
}

keyboard_render_state scene::snapshot_keyboard(uint64_t have_version) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keyboard_.snapshot(have_version);
}

launcher_render_state scene::snapshot_launcher(uint64_t have_version) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return launcher_.snapshot(have_version);
}

// ------------------------------------------------------------------
// serialisation
// ------------------------------------------------------------------

std::string scene::window_json(const panel &p, bool include_input_log) const {
    std::string s = "{";
    s += "\"handle\":" + std::to_string(p.handle);
    s += ",\"app_id\":\"";
    json_escape(p.app_id, s);
    s += "\",\"title\":\"";
    json_escape(p.title, s);
    s += "\",\"pid\":" + std::to_string(p.owner_pid);
    s += std::string(",\"focused\":") +
         (p.handle == focused_ ? "true" : "false");
    if (p.has_anchor)
        s += ",\"anchor\":\"" + uuid_to_hex(p.anchor_uuid) + "\"";
    else
        s += ",\"anchor\":null";
    // Panel geometry is authored in the SCENE frame (origin-subtracted); the
    // head-pose reply carries both frames. Tagged so an agent reading one reply
    // never silently compares it against the other.
    s += ",\"frame\":\"scene\"";
    s += ",\"pos\":" + vec_json(p.pos, 3);
    // Orientation quaternion from the (orthonormal) row basis of m. Identity
    // unless anchored.
    float quat[4];
    quat_from_row_matrix(p.m, quat);
    s += ",\"quat\":" + vec_json(quat, 4);
    float wh[2] = {(float)p.width_px, (float)p.height_px};
    s += ",\"size\":" + vec_json(wh, 2);
    if (include_input_log) {
        s += ",\"input_log\":[";
        size_t start = p.input_log.size() > PANEL_INPUT_LOG_TAIL
                           ? p.input_log.size() - PANEL_INPUT_LOG_TAIL
                           : 0;
        bool first = true;
        for (size_t i = start; i < p.input_log.size(); i++) {
            if (!first) s += ",";
            first = false;
            s += "\"";
            json_escape(p.input_log[i], s);
            s += "\"";
        }
        s += "]";
    }
    s += "}";
    return s;
}

std::string scene::windows_json(bool include_input_log) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string s = "{\"windows\":[";
    bool first = true;
    for (const auto &p : panels_) {
        if (!first) s += ",";
        first = false;
        s += window_json(*p, include_input_log);
    }
    s += "]}";
    return s;
}

std::string scene::planes_json() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string s = "{\"planes\":[";
    bool first = true;
    for (int i = 0; i < n_planes_; i++) {
        if (planes_[i].is_removed)
            continue;
        if (!first) s += ",";
        first = false;
        s += "{\"uuid\":\"" + uuid_to_hex(planes_[i].uuid) + "\"";
        s += ",\"center\":" + vec_json(planes_[i].center, 3);
        s += ",\"normal\":" + vec_json(planes_[i].normal, 3);
        s += ",\"extent\":" + vec_json(planes_[i].extent, 2);
        s += ",\"alignment\":" + std::to_string(planes_[i].alignment);
        s += "}";
    }
    s += "]}";
    return s;
}

std::string scene::head_pose_json() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string s = "{";
    if (have_pose_) {
        s += std::string("\"tracking\":") +
             (last_pose_.tracking_quality > 0.0f ? "true" : "false");
        // "pos"/"rot" stay RAW ARKit for wire compatibility with wxrd's
        // head-pose reply; "scene_*" is the same pose in the frame every panel
        // position and plane centre is reported in.
        s += ",\"frame\":\"raw\"";
        s += ",\"pos\":" + vec_json(last_pose_.pos, 3);
        s += ",\"rot\":" + vec_json(last_pose_.rot, 4);
        float scene_rot[4];
        quat_mul(env_.world_origin_rot_inv, last_pose_.rot, scene_rot);
        s += ",\"scene_pos\":" + vec_json(env_.head_pos, 3);
        s += ",\"scene_rot\":" + vec_json(scene_rot, 4);
        // Camera roll about its own forward axis, measured against gravity:
        // 0 with the phone level (landscape-native, the sensor's own
        // orientation), +/-90 held portrait, 180 upside-down. Same ux/uy
        // projection the renderer's orientation buckets snap
        // (depth_math.h orientation_bucket_from_gravity), reported unsnapped.
        const float rx[3] = {1.0f, 0.0f, 0.0f};
        const float ry[3] = {0.0f, 1.0f, 0.0f};
        float cam_right[3], cam_up[3];
        quat_rotate_vec(last_pose_.rot, rx, cam_right);
        quat_rotate_vec(last_pose_.rot, ry, cam_up);
        float ux = cam_right[1], uy = cam_up[1];
        // Looking near-straight up or down leaves gravity almost parallel to
        // the optical axis, where roll is genuinely undefined rather than 0.
        if (ux * ux + uy * uy >= 0.05f * 0.05f)
            s += ",\"roll_deg\":" +
                 fnum((double)(std::atan2(ux, uy) * 57.29577951308232f));
        else
            s += ",\"roll_deg\":null";
        s += ",\"quality\":" + fnum((double)last_pose_.tracking_quality);
    } else {
        s += "\"tracking\":false,\"roll_deg\":null";
    }
    s += "}";
    return s;
}

std::string scene::aim_json() const {
    std::lock_guard<std::mutex> lock(mutex_);
    int n_hands = (hands_[0].present ? 1 : 0) + (hands_[1].present ? 1 : 0);
    std::string s = "{\"hands\":" + std::to_string(n_hands);

    float point[3], dir[3];
    bool have_aim = aim_point(point);
    bool have_ray = false;
    if (have_aim) {
        v3sub(point, env_.head_pos, dir);
        have_ray = v3normalize(dir) > 1e-5f;
    }
    s += ",\"aim\":" + (have_aim ? vec_json(point, 3) : std::string("null"));
    s += ",\"ray_origin\":" +
         (have_ray ? vec_json(env_.head_pos, 3) : std::string("null"));
    s += ",\"ray_dir\":" + (have_ray ? vec_json(dir, 3) : std::string("null"));
    s += std::string(",\"pinching\":") + (pinch_held_ ? "true" : "false");

    bool direct = false;
    const panel *aimed = have_aim ? pick_aim_panel(point, &direct) : nullptr;
    s += ",\"aimed_handle\":" +
         (aimed ? std::to_string(aimed->handle) : std::string("null"));

    // The quad the ray picks is the hit when it actually lands on it; a panel
    // chosen by proximity alone leaves the ray to fall through to the room.
    float hit[3];
    bool have_hit = false;
    if (aimed)
        have_hit = panel_hit_point(*aimed, env_.head_pos, point, direct, hit,
                                   nullptr, nullptr, nullptr);
    if (!have_hit && have_ray)
        have_hit = ray_plane_hit(env_, planes_, n_planes_, env_.head_pos, dir,
                                 hit);
    s += ",\"hit\":" + (have_hit ? vec_json(hit, 3) : std::string("null"));
    return s + "}";
}

bool scene::snapshot_depth(uint64_t have_version, depth_snapshot &out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_depth_.version == 0 || latest_depth_.version == have_version)
        return false;
    out = latest_depth_;
    return true;
}

std::string scene::cast_json(const float *origin_in, const float *dir_in) const {
    static const char *NONE =
        "{\"hit\":null,\"distance_m\":null,\"normal\":null,\"kind\":null,"
        "\"source\":\"none\"}";
    std::lock_guard<std::mutex> lock(mutex_);

    float origin[3], dir[3];
    if (origin_in && dir_in) {
        v3copy(origin_in, origin);
        v3copy(dir_in, dir);
    } else {
        float head_quat[4];
        if (!scene_head_quat(head_quat))
            return NONE;  // no pose yet: there is no "forward" to cast along
        v3copy(env_.head_pos, origin);
        const float fwd_local[3] = {0.0f, 0.0f, -1.0f};
        quat_rotate_vec(head_quat, fwd_local, dir);
    }
    if (!v3finite(origin) || !v3finite(dir) || v3normalize(dir) <= 1e-6f)
        return NONE;

    auto reply = [&](const float hit[3], float distance, const float normal[3],
                     const char *source) {
        std::string s = "{\"hit\":" + vec_json(hit, 3);
        s += ",\"distance_m\":" + fnum((double)distance);
        s += ",\"normal\":" + vec_json(normal, 3);
        s += std::string(",\"kind\":\"") + surface_kind(normal) + "\"";
        s += std::string(",\"source\":\"") + source + "\"}";
        return s;
    };

    if (latest_depth_.have_pose && latest_depth_.have_intrinsics &&
        !latest_depth_.depth.empty()) {
        depth_cast_input in;
        in.depth = latest_depth_.depth.data();
        in.width = (int)latest_depth_.width;
        in.height = (int)latest_depth_.height;
        in.intr = latest_depth_.intr;
        v3copy(latest_depth_.cam_pos, in.cam_pos);
        for (int i = 0; i < 4; i++)
            in.cam_quat[i] = latest_depth_.cam_quat[i];
        depth_cast_result r;
        if (depth_cast_ray(in, origin, dir, r))
            return reply(r.point, r.distance_m, r.normal, "depth");
    }

    float hit[3], normal[3];
    if (ray_plane_hit(env_, planes_, n_planes_, origin, dir, hit, normal)) {
        float rel[3];
        v3sub(hit, origin, rel);
        return reply(hit, v3length(rel), normal, "plane");
    }
    return NONE;
}

std::string scene::floor_json() const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (latest_depth_.have_pose && latest_depth_.have_intrinsics &&
        !latest_depth_.depth.empty()) {
        depth_cast_input in;
        in.depth = latest_depth_.depth.data();
        in.width = (int)latest_depth_.width;
        in.height = (int)latest_depth_.height;
        in.intr = latest_depth_.intr;
        v3copy(latest_depth_.cam_pos, in.cam_pos);
        for (int i = 0; i < 4; i++)
            in.cam_quat[i] = latest_depth_.cam_quat[i];
        float y;
        if (depth_floor_height(in, y))
            return "{\"height\":" + fnum((double)y) +
                   ",\"source\":\"depth\"}";
    }

    bool have = false;
    float lowest = 0.0f;
    for (int i = 0; i < n_planes_; i++) {
        if (planes_[i].is_removed)
            continue;
        float center[3], normal[3];
        plane_to_scene_frame(env_, planes_[i], center, normal);
        if (std::fabs(normal[1]) <= DEPTH_CAST_HORIZONTAL_NY)
            continue;
        if (!have || center[1] < lowest) {
            lowest = center[1];
            have = true;
        }
    }
    if (have)
        return "{\"height\":" + fnum((double)lowest) +
               ",\"source\":\"plane\"}";
    return "{\"height\":null,\"source\":\"none\"}";
}

// ------------------------------------------------------------------
// layouts
// ------------------------------------------------------------------

std::vector<layout_panel> scene::capture_layout() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<layout_panel> out;
    out.reserve(panels_.size());
    for (const auto &p : panels_) {
        layout_panel lp;
        lp.kind = p->kind == panel_kind::note              ? "note"
                  : p->kind == panel_kind::captured_window ? "captured"
                                                           : "internal";
        lp.app_id = p->app_id;
        lp.title = p->title;
        lp.body = p->note_body;
        lp.accent = p->note_accent;
        v3copy(p->pos, lp.pos);
        lp.yaw = p->yaw;
        lp.width_px = p->width_px;
        lp.height_px = p->height_px;
        lp.width_m = p->width_m;
        lp.has_anchor = p->has_anchor;
        std::memcpy(lp.anchor_uuid, p->anchor_uuid, 16);
        v3copy(p->anchor_offset, lp.anchor_offset);
        out.push_back(std::move(lp));
    }
    return out;
}

bool scene::apply_layout_pose(uint64_t handle, const layout_panel &saved) {
    std::lock_guard<std::mutex> lock(mutex_);
    panel *p = find_panel(handle);
    if (!p)
        return false;
    v3copy(saved.pos, p->pos);
    v3copy(saved.pos, p->target_pos);
    p->yaw = saved.yaw;
    // The layout file stores a yaw, not a full orientation, so a restored
    // panel is a yawed panel again.
    p->has_pose_quat = false;
    if (saved.width_m > 0.0f)
        p->width_m = saved.width_m;
    // A captured panel's pixel size belongs to the live window, not the file.
    if (p->kind != panel_kind::captured_window && saved.width_px > 0 &&
        saved.height_px > 0) {
        p->width_px = saved.width_px;
        p->height_px = saved.height_px;
        p->surface.resize(saved.width_px, saved.height_px);
    }
    p->has_anchor = saved.has_anchor;
    std::memcpy(p->anchor_uuid, saved.anchor_uuid, 16);
    v3copy(saved.anchor_offset, p->anchor_offset);
    p->reanchor_pending = false;
    p->surface_dirty = true;
    return true;
}

std::vector<uint64_t> scene::panel_handles() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<uint64_t> out;
    out.reserve(panels_.size());
    for (const auto &p : panels_)
        out.push_back(p->handle);
    return out;
}

std::string scene::dump_state_json() const {
    // Compose from the individually-locked pieces; a tick between them is
    // acceptable (each piece is internally consistent).
    std::string windows = windows_json(true);
    std::string planes = planes_json();
    std::string head = head_pose_json();
    std::string s = "{\"windows\":";
    s += windows.substr(windows.find(':') + 1,
                        windows.rfind('}') - windows.find(':') - 1);
    s += ",\"planes\":";
    s += planes.substr(planes.find(':') + 1,
                       planes.rfind('}') - planes.find(':') - 1);
    s += ",\"head\":" + head + "}";
    return s;
}

std::vector<std::string> scene::drain_events() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> out(events_.begin(), events_.end());
    events_.clear();
    return out;
}

// ------------------------------------------------------------------
// introspection
// ------------------------------------------------------------------

uint64_t scene::focused_handle() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return focused_;
}

uint64_t scene::aim_handle() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return aimed_;
}

size_t scene::panel_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return panels_.size();
}

bool scene::panel_pose(uint64_t handle, float out_pos[3]) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const panel *p = find_panel(handle);
    if (!p)
        return false;
    v3copy(p->pos, out_pos);
    return true;
}

bool scene::panel_matrix(uint64_t handle, float out_m[16]) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const panel *p = find_panel(handle);
    if (!p)
        return false;
    std::memcpy(out_m, p->m, sizeof(p->m));
    return true;
}

std::vector<std::string> scene::panel_input_log(uint64_t handle) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const panel *p = find_panel(handle);
    if (!p)
        return {};
    return std::vector<std::string>(p->input_log.begin(), p->input_log.end());
}

bool scene::head_pose(float out_pos[3], float out_quat[4]) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_pose_)
        return false;
    v3copy(env_.head_pos, out_pos);
    // Scene-frame orientation: inv(origin_rot) * pose_rot, mirroring the
    // origin-rotation subtraction applied to positions.
    quat_mul(env_.world_origin_rot_inv, last_pose_.rot, out_quat);
    return true;
}

bool scene::head_pose_at(uint64_t ts_ns, float out_pos[3], float out_quat[4],
                         float *out_lag_ms) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return head_pose_at_locked(ts_ns, out_pos, out_quat, out_lag_ms);
}

bool scene::head_pose_at_locked(uint64_t ts_ns, float out_pos[3],
                                float out_quat[4], float *out_lag_ms) const {
    if (out_lag_ms)
        *out_lag_ms = 0.0f;
    if (head_ring_count_ == 0)
        return false;

    // One linear pass over <= HEAD_POSE_RING slots: the tightest bracketing
    // pair around ts_ns, plus the newest sample for the fallback. Scanning
    // rather than assuming monotonic order keeps a reordered datagram from
    // producing a bracket that spans the wrong interval.
    const head_sample *lo = nullptr;   // greatest ts <= ts_ns
    const head_sample *hi = nullptr;   // least ts >= ts_ns
    const head_sample *newest = nullptr;
    for (size_t i = 0; i < head_ring_count_; i++) {
        const head_sample &s = head_ring_[i];
        if (!newest || s.ts_ns > newest->ts_ns)
            newest = &s;
        if (s.ts_ns <= ts_ns && (!lo || s.ts_ns > lo->ts_ns))
            lo = &s;
        if (s.ts_ns >= ts_ns && (!hi || s.ts_ns < hi->ts_ns))
            hi = &s;
    }

    if (!lo || !hi) {
        v3copy(newest->pos, out_pos);
        for (int i = 0; i < 4; i++)
            out_quat[i] = newest->quat[i];
        return true;
    }

    if (lo->ts_ns == hi->ts_ns) {
        v3copy(lo->pos, out_pos);
        for (int i = 0; i < 4; i++)
            out_quat[i] = lo->quat[i];
    } else {
        const float t = (float)(ts_ns - lo->ts_ns) /
                        (float)(hi->ts_ns - lo->ts_ns);
        for (int i = 0; i < 3; i++)
            out_pos[i] = lo->pos[i] + (hi->pos[i] - lo->pos[i]) * t;
        quat_slerp(lo->quat, hi->quat, t, out_quat);
    }
    if (out_lag_ms && newest->ts_ns > ts_ns)
        *out_lag_ms = (float)(newest->ts_ns - ts_ns) / 1e6f;
    return true;
}

void scene::set_view_lag_ms(float ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    view_lag_ms_ = ms;
}

float scene::view_lag_ms() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return view_lag_ms_;
}

float scene::tracking_quality() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return have_pose_ ? last_pose_.tracking_quality : -1.0f;
}

bool scene::head_up_in_camera(float out_uxuy[2]) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_pose_)
        return false;
    // Raw ARKit world up is gravity-up; project it into the camera image
    // plane: components along the camera's +X (right) and +Y (up) axes,
    // which are the y components of the rotated basis vectors.
    const float rx[3] = {1.0f, 0.0f, 0.0f};
    const float ry[3] = {0.0f, 1.0f, 0.0f};
    float cam_right[3], cam_up[3];
    quat_rotate_vec(last_pose_.rot, rx, cam_right);
    quat_rotate_vec(last_pose_.rot, ry, cam_up);
    out_uxuy[0] = cam_right[1];
    out_uxuy[1] = cam_up[1];
    return true;
}

bool scene::hand_joints(int slot, sb_hand_t &out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (slot < 0 || slot > 1 || hand_age_s_[slot] > HAND_STALE_S)
        return false;
    out = hand_raw_[slot];
    return true;
}

std::vector<scene::render_panel> scene::snapshot_render_panels(
    const have_version_fn &have_version) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<render_panel> out;
    out.reserve(panels_.size());
    for (const auto &p : panels_) {
        render_panel rp;
        rp.handle = p->handle;
        rp.kind = p->kind;
        rp.title = p->title;
        std::memcpy(rp.m, p->m, sizeof(p->m));
        rp.width_m = p->width_m;
        rp.height_m = panel_height_m(*p);
        rp.focused = p->handle == focused_;
        rp.aimed = p->handle == aimed_;
        rp.grabbed = p->handle == grabbed_;
        rp.width_px = p->surface.width();
        rp.height_px = p->surface.height();
        rp.surface_version = p->surface_version;
        // Multi-megabyte copy under the scene mutex on the render thread —
        // only pay it when the caller's cached texture is actually stale.
        if (p->kind != panel_kind::captured_window &&
            (!have_version || have_version(p->handle) != p->surface_version))
            rp.rgba.assign(p->surface.pixels(),
                           p->surface.pixels() + p->surface.size_bytes());
        out.push_back(std::move(rp));
    }
    return out;
}

}  // namespace mac_shell
