// scene.h — mac-shell world state: panels, anchors, head pose, hands.
//
// Pure C++ (no Metal, no AppKit). One tick() call advances everything at a
// caller-chosen dt so tests can drive it deterministically. Inputs come from
// a bridge-receiver handle (live/replay) or from the inject_* test hooks.
//
// Thread safety: every public method locks an internal mutex, so the control
// server thread and the tick thread can interleave freely.

#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "gesture_engine.h"
#include "spatial_bridge.h"

#include "core/anchor_math.h"
#include "core/hud_state.h"
#include "core/layout_store.h"
#include "ui/keyboard_overlay.h"
#include "ui/launcher_menu.h"
#include "ui/panel_surface.h"

namespace mac_shell {

constexpr int SCENE_MAX_PLANES = 64;  // matches anchors.c MAX_PLANES
constexpr size_t MAX_PANELS = 32;     // spawn_* return 0 past this
constexpr size_t PANEL_INPUT_LOG_MAX = 64;
constexpr size_t PANEL_INPUT_LOG_TAIL = 8;
// Recent SCENE-frame head poses kept for frame/pose time alignment. 30 slots
// is half a second at the 60 Hz pose rate — comfortably longer than the
// chunk-reassembly + JPEG-decode lag a passthrough frame carries behind the
// pose that was current when the camera exposed it.
constexpr size_t HEAD_POSE_RING = 30;

enum class panel_kind {
    internal_test_card,
    captured_window,  // Stage 2: ScreenCaptureKit
    note,             // CPU-drawn note card (control verb `note`)
};

// Input events forwarded to captured windows (Stage 2 CGEvent injection).
struct inject_event {
    enum class kind { text, key, click, scroll, focus } type = kind::text;
    std::string text;   // text payload / key name
    float x = 0, y = 0; // click surface-local px / scroll deltas
    int button = 0;
};

struct panel {
    uint64_t handle = 0;
    panel_kind kind = panel_kind::internal_test_card;
    int32_t owner_pid = 0;  // captured panels: pid of the source app
    std::string app_id;
    std::string title;
    std::string note_body;      // note panels: the wrapped body text
    bool note_accent = false;   // note panels: draw the accent rule

    // Scene-frame position. Unanchored panels carry a yaw (rotation about
    // +Y, facing the head at spawn/gather time); when anchored, m holds the
    // full plane transform.
    float pos[3] = {0.0f, 0.0f, 0.0f};
    float yaw = 0.0f;
    float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    // Hand-driven motion (grab) accumulates here behind the grab deadband;
    // pos follows it directly. Smoothing is the renderer's single move
    // spring (see docs/mac-shell-design.md), not a second stage here.
    float target_pos[3] = {0.0f, 0.0f, 0.0f};

    int width_px = 512;
    int height_px = 384;
    float width_m = 0.6f;  // rendered quad width; height follows aspect

    bool has_anchor = false;
    uint8_t anchor_uuid[16] = {0};
    float anchor_offset[3] = {0.0f, 0.0f, 0.0f};
    // Set when an ARKit world re-init invalidated this panel's anchor: the
    // uuid may survive the reset but its coordinates do not, so the panel
    // re-snaps to the nearest plane in the NEW frame instead of holding a
    // transform derived from the dead one. Cleared by a successful re-snap or
    // by any user/control action that expresses a new intent for the panel.
    bool reanchor_pending = false;

    std::deque<std::string> input_log;
    panel_surface surface;
    bool surface_dirty = true;
    // Bumped only when THIS panel's surface is redrawn, so one panel's
    // repaint never invalidates another panel's cached GPU texture.
    uint64_t surface_version = 1;
};

class scene {
   public:
    scene();
    ~scene();

    scene(const scene &) = delete;
    scene &operator=(const scene &) = delete;

    // Receiver is optional; tests may inject state directly instead.
    void set_receiver(sb_receiver_t *receiver);

    // Advance the world by dt seconds: drain receiver state, run gestures,
    // update anchored panel transforms, refresh dirty surfaces.
    void tick(float dt);

    // ---- test injection (used instead of a receiver) ----
    void inject_pose(const sb_pose_t &pose);
    void inject_planes(const sb_plane_t *planes, int n);
    void inject_hand(int slot, const sb_hand_t &hand);
    // Feed a synthetic gesture event straight into the gesture handler —
    // lets tests exercise grab/close/focus flows for gestures the engine
    // currently has disabled (e.g. grab_window / WINDOW_MOVE).
    void inject_gesture(const ge_event_t &ev);

    // ---- control-plane operations ----
    // Returns the new handle, or 0 when MAX_PANELS is reached.
    uint64_t spawn_panel(const std::string &app_id, const std::string &title);
    // Note card: title over word-wrapped body, drawn CPU-side. app_id is
    // "note" so `list-windows` distinguishes it from a test card.
    uint64_t spawn_note_panel(const std::string &title,
                              const std::string &body, bool accent);
    // Fields left unset keep their current value (`note-update` may carry
    // only the half that changed).
    struct note_patch {
        bool set_title = false;
        std::string title;
        bool set_body = false;
        std::string body;
        bool set_accent = false;
        bool accent = false;
    };
    bool update_note(uint64_t handle, const note_patch &patch);

    // Stage 2: panel backed by a ScreenCaptureKit stream (renderer pulls the
    // texture from the capture manager by handle).
    uint64_t spawn_captured_panel(const std::string &app_id,
                                  const std::string &title, int32_t owner_pid,
                                  int width_px, int height_px);
    // Called for input verbs on a captured panel. Set by the capture layer,
    // which injects by default once Accessibility is granted
    // (SPATULA_MAC_INJECT=0 opts out; headless runs never inject).
    using injector_fn =
        std::function<void(uint64_t handle, int32_t pid, const inject_event &)>;
    void set_input_injector(injector_fn fn);
    bool move_panel(uint64_t handle, const float vec[3], bool relative);
    // mode: 0 = explicit uuid, 1 = closest wall, 2 = closest horizontal.
    // Returns 1 anchored (out_uuid filled), 0 no_anchor, -1 no such window.
    int anchor_panel(uint64_t handle, int mode, const uint8_t uuid[16],
                     uint8_t out_uuid[16]);
    bool clear_anchor(uint64_t handle);
    // Returns 1 anchored (out_uuid filled), 0 no anchor, -1 no such window.
    int query_anchor(uint64_t handle, uint8_t out_uuid[16]);
    bool focus_panel(uint64_t handle);
    bool close_panel(uint64_t handle);
    bool resize_panel(uint64_t handle, int width, int height);
    // Input routed to the focused panel's input log. false → no focus.
    bool type_text(const std::string &text);
    bool key_press(const std::string &name);
    bool click_panel(uint64_t handle, float x, float y, int button);
    bool scroll(float dx, float dy);
    // Recall every unanchored panel into an arc in front of the head
    // (control verb `gather-panels`; also fired by a double-pinch on empty
    // space). Returns the number of panels moved.
    int gather_panels();
    // LiDAR occlusion mode consumed by the renderer: 0 off, 1 hard, 2 soft
    // (default). Set from SPATULA_MAC_DEPTH_OCCLUSION or the
    // `depth-occlusion on|off|soft` control verb.
    void set_depth_occlusion_mode(int mode);
    int depth_occlusion_mode() const;

    // Full 21-joint hand skeleton (cyan left / yellow right, matching the
    // iPhone Lab overlay) instead of the minimal fingertip orbs. This is the
    // instrument for judging tracking quality and the user's only feedback
    // that hands are seen at all, so it is on by default. Initial value from
    // SPATULA_MAC_HAND_DEBUG, flipped live by `hands overlay on|off`.
    void set_hand_overlay(bool on);
    bool hand_overlay() const;

    // ---- virtual keyboard ----
    // show anchors to the nearest horizontal plane (else floating default);
    // returns whether it ended up plane-anchored.
    bool keyboard_show();
    bool keyboard_hide();  // false if it was already hidden
    bool keyboard_visible() const;
    std::string keyboard_status_json() const;

    // ---- radial launcher ----
    void launcher_show();
    bool launcher_hide();  // false if it was already hidden
    bool launcher_visible() const;
    bool launcher_select(int idx_on_page);
    bool launcher_commit();
    std::string launcher_status_json() const;
    // Handler for non-internal launcher targets (renderer wires this to
    // capture_manager::launch_app). Unset → internal test-card fallback.
    using app_launch_fn = std::function<void(const std::string &target)>;
    void set_app_launcher(app_launch_fn fn);

    // ---- layouts (control verb `layout`) ----
    std::vector<layout_panel> capture_layout() const;
    // Re-poses a freshly spawned panel onto its saved placement (position,
    // yaw, pixel size, quad width, anchor). False for an unknown handle.
    bool apply_layout_pose(uint64_t handle, const layout_panel &saved);
    std::vector<uint64_t> panel_handles() const;

    // ---- state serialisation (reply payloads; shapes mirror control.c) ----
    std::string windows_json(bool include_input_log) const;
    std::string planes_json() const;
    std::string head_pose_json() const;
    std::string dump_state_json() const;
    // Where the user is pointing right now: hand count, the pinch midpoint,
    // the head->aim ray, whether a pinch is held, the panel that ray picks,
    // and where it lands (on that panel's quad, else on the nearest plane).
    // Every vector is null when no confident hand is visible.
    std::string aim_json() const;

    // Event lines queued for `subscribe` streams ("event ..." payloads).
    std::vector<std::string> drain_events();

    // ---- introspection (tests + renderer) ----
    uint64_t focused_handle() const;
    // Panel a pinch would hit right now, recomputed every tick from the live
    // hand. 0 when nothing is aimed at (no hand, no candidate, or an overlay
    // owns the pointer vocabulary). The renderer rims it in the accent so the
    // user sees the target before committing to the pinch.
    uint64_t aim_handle() const;
    size_t panel_count() const;
    bool panel_pose(uint64_t handle, float out_pos[3]) const;
    bool panel_matrix(uint64_t handle, float out_m[16]) const;
    std::vector<std::string> panel_input_log(uint64_t handle) const;
    bool head_pose(float out_pos[3], float out_quat[4]) const;
    // SCENE-frame head pose as it was at `ts_ns` — the timestamp of the
    // passthrough frame the renderer is about to draw. Position is linearly
    // interpolated and rotation slerp'd between the two bracketing samples of
    // the pose ring, so a frame stamped between two pose packets gets the pose
    // the camera actually had. A `ts_ns` outside the ring (older than its
    // oldest sample, or ahead of the newest pose) yields the NEWEST pose
    // rather than a ring end: a frame that stale is worse than no alignment.
    // `out_lag_ms`, when given, receives newest_pose_ts - ts_ns in ms, clamped
    // at 0, and 0 whenever the newest pose was used as-is.
    // False before the first pose, or while no world origin is captured (the
    // ring's samples are only meaningful inside a captured SCENE frame).
    bool head_pose_at(uint64_t ts_ns, float out_pos[3], float out_quat[4],
                      float *out_lag_ms = nullptr) const;
    // Offset the renderer actually drew with this frame, surfaced by `stats`
    // as `view_lag_ms`. -1 until a renderer publishes one.
    void set_view_lag_ms(float ms);
    float view_lag_ms() const;
    // World-up projected into the raw camera image plane: out[0] =
    // dot(up, cam_right), out[1] = dot(up, cam_up). Feeds the renderer's
    // phone-orientation compensation (depth_math.h
    // orientation_bucket_from_gravity). False before the first pose.
    bool head_up_in_camera(float out_uxuy[2]) const;
    // Latest tracking quality (0 lost, 0.5 limited, 1 normal); -1 if no pose
    // has ever arrived. HUD use.
    float tracking_quality() const;
    bool hand_joints(int slot, sb_hand_t &out) const;  // scene frame
    // Copies the panel list for rendering. Surface pixels are copied only
    // when the caller's cached version is stale — a full RGBA copy of every
    // internal panel per frame is multiple megabytes under the scene mutex.
    struct render_panel {
        uint64_t handle;
        panel_kind kind;
        std::string title;
        float m[16];
        float width_m, height_m;
        bool focused;
        bool aimed;    // pinch candidate: renderer rims it in the accent
        bool grabbed;  // mid-drag: renderer lifts/tilts it slightly
        int width_px, height_px;
        std::vector<uint8_t> rgba;  // internal panels only
        uint64_t surface_version;
    };
    // have_version(handle) -> the surface version the caller already holds
    // on the GPU; empty means "hold nothing, copy everything".
    using have_version_fn = std::function<uint64_t(uint64_t)>;
    std::vector<render_panel> snapshot_render_panels(
        const have_version_fn &have_version = {});
    keyboard_render_state snapshot_keyboard(uint64_t have_version = 0) const;
    launcher_render_state snapshot_launcher(uint64_t have_version = 0) const;

    // keyboard_anchor hold-progress (one per engine hand): drives the summon
    // progress arc while the palm-down hold runs.
    struct hud_arc {
        bool active = false;
        float pos[3] = {0, 0, 0};
        float progress = 0.0f;
    };
    void hud_arcs(hud_arc out[2]) const;

    // ---- in-scene toasts (plain-language notices) ----
    // Posted by the capture layer (permission degrades) and the renderer
    // (stream drops); ticked by tick(); drawn by the renderer's HUD pass.
    void post_toast(const std::string &text,
                    toast_action action = toast_action::none);
    std::vector<toast> snapshot_toasts() const;
    // Dismiss the currently shown toast (its action button was clicked or
    // it was tapped away).
    void dismiss_toast();

   private:
    panel *find_panel(uint64_t handle);
    const panel *find_panel(uint64_t handle) const;
    void push_event(const std::string &line);
    // mutex_ held: set focus, dirty every surface (focus ring), emit event.
    void focus_locked(uint64_t handle);
    // mutex_ held: body of click_panel, callable from the gesture path where
    // the mutex is already taken.
    bool click_panel_locked(uint64_t handle, float x, float y, int button);
    // mutex_ held: panel a pinch at `point` targets — direct touch first,
    // else the nearest panel to the head->point ray in front of the user.
    // `out_direct` reports which of the two won (the hit test differs).
    panel *pick_aim_panel(const float point[3], bool *out_direct) const;
    // mutex_ held: refresh aim_handle_ from the live hand's pinch midpoint.
    void update_aim();
    // mutex_ held: the pinch midpoint of the freshest tracked hand, which is
    // the point the gesture engine reports in its own pointer events.
    bool aim_point(float out[3]) const;
    // Rewrite a raw hand's joints from ARKit world frame to scene frame.
    void hand_to_scene(sb_hand_t &hand) const;
    // Copies planes in, dropping any with non-finite geometry. mutex_ held.
    void ingest_planes(const sb_plane_t *planes, int n);
    // Adopts a hand into slot if every joint is finite. mutex_ held.
    void ingest_hand(int slot, const sb_hand_t &hand);
    uint64_t spawn_panel_impl(const std::string &app_id,
                              const std::string &title);
    // Head-relative spawn placement (falls back to the legacy shelf when no
    // pose has arrived). mutex_ held.
    void place_new_panel(panel &p);
    int gather_panels_impl();
    bool scene_head_quat(float out_quat[4]) const;
    void on_key_emit(uint32_t keysym, const char *label, bool repeat);
    void on_launcher_launch(const launcher_entry &entry);
    void keyboard_toggle_impl(bool want_visible);
    void update_hud_arcs();
    void process_pending_launches();
    void tick_locked(float dt);
    void on_gesture(const ge_event_t &ev);
    static void gesture_trampoline(const ge_event_t *ev, void *user);
    void update_head(const sb_pose_t &pose);
    // mutex_ held: append one SCENE-frame sample to the fixed-size pose ring.
    void push_head_sample(uint64_t ts_ns, const float pos[3],
                          const float quat[4]);
    // mutex_ held: every sample was expressed in a SCENE frame that no longer
    // exists (origin dropped, recaptured, or reset), so none of them can be
    // interpolated against the poses that follow.
    void clear_head_ring();
    // mutex_ held: ARKit re-initialised its world origin, so every scene-frame
    // coordinate we hold is stale. Drop the captured origin and invalidate
    // anchors; emits the toast + control event.
    void world_frame_reset(uint32_t epoch);
    // Drops the cached world origin after a pose-stream gap. mutex_ held.
    // Returns whether this timestamp is a genuinely new sample.
    bool maybe_recapture_origin(uint64_t pose_ts_ns);
    void world_to_scene(const float in[3], float out[3]) const;
    void log_input(panel &p, const std::string &line);
    void refresh_anchor_transforms();
    std::string window_json(const panel &p, bool include_input_log) const;

    mutable std::mutex mutex_;

    sb_receiver_t *receiver_ = nullptr;
    ge_engine_t *engine_ = nullptr;

    anchor_env env_;
    bool have_pose_ = false;
    sb_pose_t last_pose_ = {};
    // Last ARKit world-frame generation seen (docs/wire-protocol.md 0x01).
    // 0 = none yet, or a pre-epoch sender — never a reset trigger.
    uint32_t session_epoch_ = 0;
    uint64_t last_fresh_pose_ts_ = 0;
    double last_fresh_pose_s_ = -1.0;  // < 0 → no pose seen yet

    // Fixed-size ring of recent SCENE-frame head poses (head_pose_at). Never
    // grows: the render path queries it every frame and must not allocate.
    struct head_sample {
        uint64_t ts_ns = 0;
        float pos[3] = {0.0f, 0.0f, 0.0f};
        float quat[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    };
    head_sample head_ring_[HEAD_POSE_RING];
    size_t head_ring_count_ = 0;  // filled slots, saturating at HEAD_POSE_RING
    size_t head_ring_next_ = 0;   // write cursor
    float view_lag_ms_ = -1.0f;

    sb_plane_t planes_[SCENE_MAX_PLANES] = {};
    int n_planes_ = 0;

    ge_hand_t hands_[2] = {};
    bool launcher_hand_left_ = false;
    sb_hand_t hand_raw_[2] = {};    // scene-frame joints for render/introspect
    float hand_age_s_[2] = {1e9f, 1e9f};

    keyboard_overlay keyboard_;
    launcher_menu launcher_;
    int launcher_engine_hand_ = -1;
    // Launcher dismiss state machine (theme LAUNCHER_* timings): tracking
    // loss keeps the menu up for a grace window; a release-commit keeps the
    // chosen wedge highlighted for a linger window. < 0 → inactive.
    float launcher_grace_left_s_ = -1.0f;
    float launcher_linger_left_s_ = -1.0f;
    app_launch_fn app_launcher_;
    std::vector<std::string> pending_app_launches_;
    hud_arc hud_arcs_[2];

    std::vector<std::unique_ptr<panel>> panels_;
    injector_fn injector_;
    uint64_t next_handle_ = 1;
    uint64_t focused_ = 0;
    uint64_t aimed_ = 0;
    uint64_t grabbed_ = 0;
    // pinch_select is held: set on the POINTER_CLICK BEGIN, cleared on its
    // END/CANCEL. The gesture stream is edge-triggered, so `aim` needs this
    // to answer "is the user pinching right now".
    bool pinch_held_ = false;
    // Grab deadband: accumulated hand displacement since WINDOW_MOVE BEGIN;
    // the panel target only starts following once it exceeds the deadband,
    // so a held (jittering) grab doesn't micro-shake the panel.
    float grab_accum_[3] = {0, 0, 0};
    bool grab_engaged_ = false;
    // Monotonic scene time (sum of tick dts) for gesture double-tap windows.
    double now_s_ = 0.0;
    double last_empty_pinch_s_ = -1e9;
    int depth_occlusion_mode_ = 2;  // 0 off, 1 hard, 2 soft
    bool hand_overlay_ = true;  // the skeleton is the user's tracking feedback
    toast_queue toasts_;

    std::deque<std::string> events_;
};

}  // namespace mac_shell
