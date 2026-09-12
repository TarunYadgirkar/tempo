// eval_recording.cpp — Replay an iPhone-recorded stream.bin through the
// gesture engine and emit one event per detected gesture.
//
// The iPhone Lab tab writes stream.bin in the live UDP wire-format with
// per-datagram length framing:
//
//   [u32 LE payload_length][payload bytes]   (repeated)
//
// Of the payload types this tool only consumes 0x05 (hand joints):
//
//   byte 0       : 0x05
//   bytes 1-8    : timestamp_ns      (u64 LE)
//   byte 9       : hand_index        (0 = left, 1 = right)
//   byte 10      : joint_count       (always 21)
//   bytes 11+    : joints[21][5] f32 (LE) — [x, y, z, confidence, reserved]
//                                              i.e. 420 bytes, total 431.
//
// 0x01 pose packets ARE consumed: the latest camera pose drives bone-length
// depth reconstruction (hr_reconstruct) of each hand before it reaches the
// engine — the same metric correction the runtime applies in world_model.
// Pass --no-reconstruct to feed the raw stream.bin joints instead (debugging).
// 0x02 / 0x03 / 0x04 packets are skipped silently — they don't drive detection.
//
// The engine wants per-tick ge_hand_t[2] with a dt seconds parameter.
// Hand packets arrive interleaved (a 60 Hz left frame, then a 60 Hz right
// frame, etc.) so we batch all hand packets that share a timestamp ±5 ms
// into one tick and use the timestamp delta as dt.
//
// Output is newline-delimited so it's trivial to diff against labels.json
// programmatically:
//
//   EVT t=1.234s hand=1 BEGIN  pinch_select  pos=(0.12,-0.04,-0.31)
//   EVT t=1.298s hand=1 END    pinch_select
//
// Stats summary at the end: total frames, gesture-events-per-gesture-name,
// and last error if the bin truncated mid-packet.
//
// SPDX-License-Identifier: MIT

#include "gesture_engine.h"
#include "hand_reconstruct.h"

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace {

constexpr uint8_t  PKT_POSE     = 0x01;
constexpr size_t   POSE_MIN_BYTES = 37;   // type + ts(8) + pos(12) + quat(16)
constexpr uint8_t  PKT_HAND     = 0x05;
constexpr size_t   HAND_BYTES   = 431;
constexpr uint64_t TICK_GROUP_NS = 5 * 1000 * 1000;  // group hand pkts within ±5 ms

struct HandPacket {
  uint64_t  ts_ns;
  uint8_t   hand_index;
  float     joints[GE_JOINT_COUNT][5];
};

bool read_u32_le (FILE *f, uint32_t *out, bool *incomplete) {
  uint8_t b[4];
  size_t n = fread (b, 1, 4, f);
  if (n != 4) {
    *incomplete = n != 0 || std::ferror (f);
    return false;
  }
  *out = (uint32_t)b[0]
       | ((uint32_t)b[1] << 8)
       | ((uint32_t)b[2] << 16)
       | ((uint32_t)b[3] << 24);
  return true;
}

uint64_t read_u64_le (const uint8_t *b) {
  return (uint64_t)b[0]
       | ((uint64_t)b[1] << 8)
       | ((uint64_t)b[2] << 16)
       | ((uint64_t)b[3] << 24)
       | ((uint64_t)b[4] << 32)
       | ((uint64_t)b[5] << 40)
       | ((uint64_t)b[6] << 48)
       | ((uint64_t)b[7] << 56);
}

float read_f32_le (const uint8_t *b) {
  uint32_t u = (uint32_t)b[0]
             | ((uint32_t)b[1] << 8)
             | ((uint32_t)b[2] << 16)
             | ((uint32_t)b[3] << 24);
  float f;
  std::memcpy (&f, &u, 4);
  return f;
}

const char *event_type_name (int t) {
  switch (t) {
  case GE_EVENT_BEGIN:  return "BEGIN";
  case GE_EVENT_UPDATE: return "UPDATE";
  case GE_EVENT_END:    return "END";
  case GE_EVENT_CANCEL: return "CANCEL";
  default:              return "?";
  }
}

struct EvalState {
  uint64_t                  t0_ns;
  uint64_t                  current_ts_ns;
  std::map<std::string,int> begin_counts;
  std::map<std::string,int> end_counts;
  bool                      gestures_only;
};

void on_event (const ge_event_t *ev, void *user_data) {
  auto *st = static_cast<EvalState *> (user_data);
  const char *name = ev->gesture_name ? ev->gesture_name : "?";
  double t_sec = (double)(st->current_ts_ns - st->t0_ns) / 1e9;
  if (st->gestures_only
      && ev->type != GE_EVENT_BEGIN && ev->type != GE_EVENT_END
      && ev->type != GE_EVENT_CANCEL)
    return;
  std::printf ("EVT t=%.3fs hand=%d %s %s",
               t_sec, ev->hand_index,
               event_type_name (ev->type), name);
  if (ev->type == GE_EVENT_UPDATE)
    std::printf (" pos=(%.3f,%.3f,%.3f) val=%.3f",
                 ev->delta[0], ev->delta[1], ev->delta[2], ev->value);
  std::printf ("\n");
  if (ev->type == GE_EVENT_BEGIN) ++st->begin_counts[name];
  else if (ev->type == GE_EVENT_END
           || ev->type == GE_EVENT_CANCEL) ++st->end_counts[name];
}

}  // namespace

// Camera-axis depth of a world point: rotate (world - cam) by the INVERSE
// camera quaternion and return -z_cam.  ARKit/OpenXR cameras look down -Z, so a
// larger return value = farther from the camera.
static float cam_depth (const float q[4], const float cam[3], const float w[3]) {
  const float v[3] = { w[0]-cam[0], w[1]-cam[1], w[2]-cam[2] };
  const float x=-q[0], y=-q[1], z=-q[2], s=q[3];   // inverse-quat vec + scalar
  const float tx=2*(y*v[2]-z*v[1]), ty=2*(z*v[0]-x*v[2]), tz=2*(x*v[1]-y*v[0]);
  const float cz = v[2] + s*tz + (x*ty - y*tx);
  return -cz;
}

int main (int argc, char **argv) {
  if (argc < 2) {
    std::fprintf (stderr,
                  "usage: %s <stream.bin> [--gestures-config <path>] "
                  "[--gestures-only] [--features] [--dump-joints] "
                  "[--no-reconstruct]\n",
                  argv[0]);
    return 1;
  }
  const char *path = argv[1];
  const char *config_path = nullptr;
  bool gestures_only = false;
  bool dump_features = false;
  bool reconstruct = true;   // metric-correct joints by default (matches runtime)
  bool dump_depth = false;   // print per-joint camera-axis depth (reconstructed)
  bool dump_joints = false;  // print all 21 reconstructed joints (world space)
  for (int i = 2; i < argc; ++i) {
    if (!std::strcmp (argv[i], "--gestures-config") && i + 1 < argc)
      config_path = argv[++i];
    else if (!std::strcmp (argv[i], "--gestures-only"))
      gestures_only = true;
    else if (!std::strcmp (argv[i], "--features"))
      dump_features = true;
    else if (!std::strcmp (argv[i], "--dump-depth"))
      dump_depth = true;     // camera-axis depth of each joint (reconstructed)
    else if (!std::strcmp (argv[i], "--dump-joints"))
      dump_joints = true;    // all 21 reconstructed joints, for offline metrics
    else if (!std::strcmp (argv[i], "--no-reconstruct"))
      reconstruct = false;   // feed RAW stream.bin joints (debugging only)
    else {
      std::fprintf (stderr, "unknown argument: %s\n", argv[i]);
      return 1;
    }
  }

  FILE *f = std::fopen (path, "rb");
  if (!f) {
    std::fprintf (stderr, "cannot open %s\n", path);
    return 1;
  }

  // Pass 1: pull all hand packets into a vector, skipping any non-hand
  // packet types.  Hand frames are small (431 B); a 10-minute clip at
  // 60 Hz × 2 hands ≈ 720 k packets = ~310 MB — fine to hold in memory.
  //
  // Bone-length depth reconstruction (unless --no-reconstruct).  The iPhone
  // streams raw LiDAR-unprojected joints, which are ~metric but ~10 mm noisy —
  // too noisy for the engine's metric thresholds.  We snap each joint onto its
  // camera ray at the anatomical bone length with the EXACT runtime code
  // (hr_reconstruct), so the eval sees the same joints the compositor would.
  // This mirrors world_model_update_hand: process packets in arrival (file)
  // order, track the latest 0x01 pose, reconstruct each 0x05 hand against it
  // with a per-hand temporal state.  Before any pose is seen, hr_reconstruct
  // no-ops (joints pass through verbatim).  See docs/hand-tracking.md.
  std::vector<HandPacket> packets;
  size_t total_pkts = 0, skipped_pkts = 0, truncated = 0, pose_pkts = 0,
         recon_pkts = 0;
  float cam_quat[4] = {0, 0, 0, 0};   // identity-zero → hr_reconstruct no-ops
  float cam_pos[3]  = {0, 0, 0};
  hr_state_t *hr_state[2] = {nullptr, nullptr};
  if (reconstruct) {
    hr_state[0] = hr_state_create ();
    hr_state[1] = hr_state_create ();
  }
  while (true) {
    uint32_t len;
    bool incomplete = false;
    if (!read_u32_le (f, &len, &incomplete)) {
      truncated = incomplete;
      break;
    }
    if (len > 16 * 1024 * 1024) {
      std::fprintf (stderr, "implausible packet length %u — bin corrupt?\n",
                    len);
      truncated = 1;
      break;
    }
    std::vector<uint8_t> payload (len);
    if (fread (payload.data (), 1, len, f) != len) {
      truncated = 1;
      break;
    }
    ++total_pkts;
    // 0x01 pose: update the latest camera pose used for reconstruction.
    if (!payload.empty () && payload[0] == PKT_POSE && len >= POSE_MIN_BYTES) {
      cam_pos[0]  = read_f32_le (&payload[9]);
      cam_pos[1]  = read_f32_le (&payload[13]);
      cam_pos[2]  = read_f32_le (&payload[17]);
      cam_quat[0] = read_f32_le (&payload[21]);
      cam_quat[1] = read_f32_le (&payload[25]);
      cam_quat[2] = read_f32_le (&payload[29]);
      cam_quat[3] = read_f32_le (&payload[33]);
      ++pose_pkts;
      continue;
    }
    if (payload.empty () || payload[0] != PKT_HAND) {
      ++skipped_pkts;
      continue;
    }
    if (len != HAND_BYTES) {
      std::fprintf (stderr, "skipping hand packet with unexpected len %u\n",
                    len);
      ++skipped_pkts;
      continue;
    }
    HandPacket hp{};
    hp.ts_ns = read_u64_le (&payload[1]);
    hp.hand_index = payload[9];
    if (payload[10] != GE_JOINT_COUNT) {
      ++skipped_pkts;
      continue;
    }
    size_t off = 11;
    for (int j = 0; j < GE_JOINT_COUNT; ++j)
      for (int k = 0; k < 5; ++k) {
        hp.joints[j][k] = read_f32_le (&payload[off]);
        off += 4;
      }
    if (reconstruct) {
      int hi = hp.hand_index <= 1 ? hp.hand_index : 0;
      if (hr_reconstruct (hp.joints, cam_quat, cam_pos, hp.ts_ns, hr_state[hi]))
        ++recon_pkts;
      if (dump_depth && (cam_quat[0] != 0.0f || cam_quat[3] != 0.0f)) {
        // Camera-axis depth (m, larger = farther) of key joints, reconstructed.
        // A palm-down reaching hand should read fingertips >= palm, i.e. tips
        // FARTHER than wrist/MCP; tips coming out CLOSER means a flipped sign.
        std::printf ("DEPTH hand=%d wrist=%.3f midMCP=%.3f idxMCP=%.3f | tips "
                     "thumb=%.3f index=%.3f mid=%.3f ring=%.3f pinky=%.3f\n",
                     hp.hand_index,
                     cam_depth (cam_quat, cam_pos, hp.joints[0]),
                     cam_depth (cam_quat, cam_pos, hp.joints[9]),
                     cam_depth (cam_quat, cam_pos, hp.joints[5]),
                     cam_depth (cam_quat, cam_pos, hp.joints[4]),
                     cam_depth (cam_quat, cam_pos, hp.joints[8]),
                     cam_depth (cam_quat, cam_pos, hp.joints[12]),
                     cam_depth (cam_quat, cam_pos, hp.joints[16]),
                     cam_depth (cam_quat, cam_pos, hp.joints[20]));
      }
    }
    if (dump_joints) {
      // All 21 joints as the engine will see them, world space — post
      // hr_reconstruct unless --no-reconstruct. Sits outside the reconstruct
      // branch on purpose: comparing the two is the whole point of the flag.
      std::printf ("JOINTS ts=%llu hand=%d",
                   (unsigned long long)hp.ts_ns, hp.hand_index);
      for (int j = 0; j < GE_JOINT_COUNT; ++j)
        std::printf (" %.5f,%.5f,%.5f,%.3f", hp.joints[j][0], hp.joints[j][1],
                     hp.joints[j][2], hp.joints[j][3]);
      std::printf ("\n");
    }
    packets.push_back (hp);
  }
  std::fclose (f);
  hr_state_destroy (hr_state[0]);
  hr_state_destroy (hr_state[1]);

  if (truncated) {
    std::fprintf (stderr, "truncated or corrupt recording: %s\n", path);
    return 1;
  }

  if (packets.empty ()) {
    std::fprintf (stderr,
                  "no hand packets in %s (total=%zu, skipped=%zu)\n",
                  path, total_pkts, skipped_pkts);
    return 2;
  }

  // Sort by timestamp.  Within a tick the L/R hand packets may arrive in
  // any order; we sort to make tick grouping unambiguous.
  std::sort (packets.begin (), packets.end (),
             [] (const HandPacket &a, const HandPacket &b) {
               return a.ts_ns < b.ts_ns;
             });

  // Pass 2: group into ticks, feed engine.
  ge_engine_t *ge = ge_create ();
  if (config_path) {
    if (!ge_load_config (ge, config_path)) {
      std::fprintf (stderr,
                    "warning: --gestures-config %s loaded no overrides\n",
                    config_path);
    }
  }

  EvalState st{};
  st.t0_ns = packets[0].ts_ns;
  st.current_ts_ns = st.t0_ns;
  st.gestures_only = gestures_only;
  ge_set_callback (ge, on_event, &st);

  // Carry forward per-hand pose between ticks (matches live-system
  // behaviour: the compositor caches the latest pose per hand and
  // republishes both every frame).  Without carry-forward a
  // recording where iPhone Vision interleaves L+R packets ~33 ms
  // apart leaves each tick with only ONE hand, and the engine's
  // per-hand state machine resets on every tick where its hand
  // isn't reported (clip 5871b8 loses its pinch_select).
  //
  // CAUSAL (no lookahead): once a hand has reported 3 consecutive
  // packets, the OFF hand's cache is forcibly invalidated.  Mirrors
  // what a production-side dedup-pipeline could do — "the user has
  // settled on this hand."  In a hand-swap case (clip 5ca5ef), the
  // FIRST few packets of the new hand-sequence pass through with
  // both hands cached, so a stale gesture may emit a brief
  // BEGIN+CANCEL pair before the off hand is invalidated.  The
  // proper production fix is upstream — see PRODUCT_OVERVIEW notes
  // on the iPhone bridge / bridge_receiver dedup heuristics.
  constexpr uint64_t HAND_STALE_NS = 50 * 1000 * 1000;
  constexpr int CONSECUTIVE_OTHER_INVALIDATE = 3;
  ge_hand_t cached[2]{};
  uint64_t  last_seen_ns[2] = {0, 0};
  int       consecutive_other[2] = {0, 0};
  uint64_t prev_tick_ts = packets[0].ts_ns;
  size_t   i = 0, ticks = 0;
  while (i < packets.size ()) {
    uint64_t tick_ts = packets[i].ts_ns;
    while (i < packets.size ()
           && packets[i].ts_ns <= tick_ts + TICK_GROUP_NS) {
      int h = packets[i].hand_index < 2 ? packets[i].hand_index : 0;
      std::memcpy (cached[h].joints, packets[i].joints,
                   sizeof (cached[h].joints));
      cached[h].present = true;
      last_seen_ns[h] = packets[i].ts_ns;
      consecutive_other[h] = 0;
      consecutive_other[1 - h]++;
      if (consecutive_other[1 - h] >= CONSECUTIVE_OTHER_INVALIDATE) {
        last_seen_ns[1 - h] = 0;
      }
      ++i;
    }
    ge_hand_t hands[2]{};
    for (int h = 0; h < 2; ++h) {
      if (last_seen_ns[h] != 0
          && tick_ts <= last_seen_ns[h] + HAND_STALE_NS) {
        hands[h] = cached[h];
      } else {
        hands[h].present = false;
      }
    }
    float dt = (float)((double)(tick_ts - prev_tick_ts) / 1e9);
    if (dt <= 0.0f || dt > 1.0f) dt = 1.0f / 60.0f;
    st.current_ts_ns = tick_ts;
    ge_update (ge, hands, dt);
    prev_tick_ts = tick_ts;
    ++ticks;

    if (dump_features) {
      static const char *names[] = {
        "thumb_index_distance", "thumb_middle_distance", "thumb_ring_distance",
        "thumb_pinky_distance",
        "thumb_curl",
        "index_curl", "middle_curl", "ring_curl", "pinky_curl",
        "all_fingers_curl",
        "thumb_on_index_projection", "thumb_to_index_line_distance",
        "thumb_along_index_tip_direction",
        "middle_tip_direction_to_thumb_cos",
        "fingertip_gather_radius",
        "palm_normal_x", "palm_normal_y", "palm_normal_z",
        "thumb_tip_confidence", "index_tip_confidence",
        "middle_tip_confidence", "ring_tip_confidence",
        "pinky_tip_confidence",
        "palm_y_flip_recency_ms",
        "thumb_index_recency_above_30mm_ms",
        "thumb_middle_recency_above_60mm_ms",
        "thumb_index_distance_min_over_50ms",
        "thumb_index_distance_min_over_200ms",
        "thumb_index_convergence_velocity_mps",
        "thumb_index_convergence_ratio",
        "thumb_middle_distance_min_over_200ms",
        "thumb_middle_convergence_velocity_mps",
        "thumb_middle_convergence_ratio",
        "thumb_to_index_line_distance_max_over_200ms",
        "index_curl_max_over_200ms",
        "middle_curl_max_over_200ms",
        "ring_curl_max_over_200ms",
        "thumb_on_index_projection_range_500ms",
        "middle_tip_confidence_min_over_200ms",
        "pinch_select_end_recency_ms",
        "palm_y_below_neg_half_recency_ms",
        "fist_launcher_end_recency_ms",
        "thumb_on_index_projection_velocity",
        "wrist_pos_x", "wrist_pos_y", "wrist_pos_z",
      };
      // Iterate ENGINE slots (0,1) — the engine remaps the iPhone
      // tracker's input slot 1 onto engine slot 0 when slot 0 is
      // empty (clip 59717c, 89449b — all hand=1 input).  Looking up
      // ge_get_feature by the input slot would print zeros.  Skip a
      // slot whose thumb_index_distance is exactly zero — that's the
      // sentinel for "engine slot has no hand bound" (a real hand has
      // a non-zero joint-pair distance).
      for (int h = 0; h < 2; ++h) {
        float ti = ge_get_feature (ge, h, "thumb_index_distance");
        if (ti == 0.0f) continue;
        std::printf ("FEAT t=%.3fs hand=%d", (double)(tick_ts - st.t0_ns)/1e9, h);
        for (const char *n : names)
          std::printf (" %s=%.3f", n, ge_get_feature (ge, h, n));
        std::printf ("\n");
      }
    }
  }

  ge_destroy (ge);

  // Summary on stderr so stdout stays event-only and machine-grep-able.
  std::fprintf (stderr,
                "\n=== eval_recording summary ===\n"
                "  file:           %s\n"
                "  packets total:  %zu\n"
                "  packets skip:   %zu\n"
                "  pose packets:   %zu\n"
                "  hand packets:   %zu\n"
                "  reconstructed:  %zu%s\n"
                "  truncated:      %s\n"
                "  ticks fed:      %zu\n"
                "  duration:       %.2fs\n",
                path, total_pkts, skipped_pkts, pose_pkts, packets.size (),
                recon_pkts, reconstruct ? "" : " (--no-reconstruct)",
                truncated ? "yes" : "no", ticks,
                (double)(packets.back ().ts_ns - packets.front ().ts_ns)
                    / 1e9);
  for (const auto &kv : st.begin_counts) {
    int ends = st.end_counts.count (kv.first) ? st.end_counts[kv.first] : 0;
    std::fprintf (stderr, "  %-22s  begin=%d  end=%d\n",
                  kv.first.c_str (), kv.second, ends);
  }
  return 0;
}
