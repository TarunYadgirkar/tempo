// replay_hands.cpp — Replay recorded hand data through the gesture engine.
//
// Reads a .bin file written by wxrd's SPATIAL_RECORD_HANDS and feeds each
// frame through ge_update(), printing computed features and gesture events.
//
// Usage: replay_hands <recording.bin> [--features] [--gestures] [--all]
//   --features  print all computed feature values per frame (default)
//   --gestures  only print frames where a gesture fires
//   --all       print both features and raw joint data

#include "gesture_engine.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

// Feature names to print (must match ge_features.cpp get_feature_by_name).
static const char *feature_names[] = {
    "thumb_index_distance",
    "thumb_middle_distance",
    "thumb_ring_distance",
    "thumb_pinky_distance",
    "thumb_on_index_projection",
    "thumb_on_middle_projection",
    "index_curl",
    "middle_curl",
    "ring_curl",
    "pinky_curl",
    "thumb_curl",
    "all_fingers_curl",
    "palm_openness",
    "fingertip_gather_radius",
    "hand_velocity",
    "palm_normal_x",
    "palm_normal_y",
    "palm_normal_z",
};
static const int num_features
    = sizeof (feature_names) / sizeof (feature_names[0]);

static const char *
action_name (int a)
{
  switch (a) {
  case GE_ACTION_POINTER_CLICK:
    return "CLICK";
  case GE_ACTION_POINTER_RIGHT_CLICK:
    return "RIGHT_CLICK";
  case GE_ACTION_SCROLL:
    return "SCROLL";
  case GE_ACTION_WINDOW_MOVE:
    return "MOVE";
  case GE_ACTION_WINDOW_RESIZE:
    return "RESIZE";
  case GE_ACTION_TOGGLE_LAUNCHER:
    return "LAUNCHER";
  case GE_ACTION_FINE_SLIDER:
    return "SLIDER";
  case GE_ACTION_WINDOW_CLOSE:
    return "CLOSE";
  default:
    return "?";
  }
}

static const char *
event_type_name (int t)
{
  switch (t) {
  case GE_EVENT_BEGIN:
    return "BEGIN";
  case GE_EVENT_UPDATE:
    return "UPDATE";
  case GE_EVENT_END:
    return "END";
  case GE_EVENT_CANCEL:
    return "CANCEL";
  default:
    return "?";
  }
}

// State shared with the callback.
static int g_frame_num = 0;
static bool g_had_event = false;

static void
on_event (const ge_event_t *ev, void *user_data)
{
  (void)user_data;
  g_had_event = true;
  printf ("  >>> GESTURE frame=%d hand=%d %s %s \"%s\"",
          g_frame_num, ev->hand_index,
          event_type_name (ev->type),
          action_name (ev->action),
          ev->gesture_name ? ev->gesture_name : "");
  if (ev->type == GE_EVENT_UPDATE)
    printf (" delta=(%.4f,%.4f,%.4f) val=%.4f",
            ev->delta[0], ev->delta[1], ev->delta[2], ev->value);
  printf ("\n");
}

struct frame_t {
  uint64_t ts_ns;
  float dt;
  ge_hand_t hands[2];
};

int
main (int argc, char *argv[])
{
  if (argc < 2) {
    fprintf (stderr,
             "Usage: %s <recording.bin> [--features|--gestures|--all] "
             "[--gestures-config <path>]\n",
             argv[0]);
    return 1;
  }

  const char *path = argv[1];
  const char *config_path = nullptr;
  bool show_features = true;
  bool show_joints = false;
  bool gestures_only = false;

  for (int i = 2; i < argc; ++i) {
    if (strcmp (argv[i], "--gestures") == 0) {
      gestures_only = true;
      show_features = false;
    } else if (strcmp (argv[i], "--features") == 0) {
      show_features = true;
    } else if (strcmp (argv[i], "--all") == 0) {
      show_features = true;
      show_joints = true;
    } else if (strcmp (argv[i], "--gestures-config") == 0) {
      if (i + 1 >= argc) {
        fprintf (stderr, "--gestures-config requires a path argument\n");
        return 1;
      }
      config_path = argv[++i];
    }
  }

  FILE *f = fopen (path, "rb");
  if (!f) {
    fprintf (stderr, "Cannot open %s\n", path);
    return 1;
  }

  // Read and validate header.
  uint8_t hdr[8];
  if (fread (hdr, 1, 8, f) != 8
      || memcmp (hdr, "SPHD", 4) != 0) {
    fprintf (stderr, "Invalid header (expected SPHD magic)\n");
    fclose (f);
    return 1;
  }
  uint16_t version = hdr[4] | ((uint16_t)hdr[5] << 8);
  uint8_t joint_count = hdr[6];
  printf ("Recording: version=%d joints=%d\n", version, joint_count);

  if (joint_count != GE_JOINT_COUNT) {
    fprintf (stderr, "Joint count mismatch: file=%d engine=%d\n",
             joint_count, GE_JOINT_COUNT);
    fclose (f);
    return 1;
  }

  // Read all frames.
  std::vector<frame_t> frames;
  while (true) {
    frame_t fr;
    if (fread (&fr.ts_ns, 8, 1, f) != 1) break;
    if (fread (&fr.dt, 4, 1, f) != 1) break;
    for (int h = 0; h < 2; ++h) {
      if (fread (fr.hands[h].joints, sizeof (fr.hands[h].joints), 1, f) != 1)
        goto done;
      uint32_t present;
      if (fread (&present, 4, 1, f) != 1) goto done;
      fr.hands[h].present = (present != 0);
    }
    frames.push_back (fr);
  }
done:
  fclose (f);

  printf ("Loaded %zu frames (%.1fs)\n\n", frames.size (),
          frames.empty () ? 0.0
                          : (double)(frames.back ().ts_ns - frames[0].ts_ns)
                                / 1e9);

  // Create engine and replay.
  ge_engine_t *ge = ge_create ();
  if (config_path) {
    if (!ge_load_config (ge, config_path)) {
      fprintf (stderr,
               "warning: --gestures-config %s did not apply any overrides\n",
               config_path);
    }
  }
  ge_set_callback (ge, on_event, nullptr);

  uint64_t t0 = frames.empty () ? 0 : frames[0].ts_ns;

  for (size_t i = 0; i < frames.size (); ++i) {
    g_frame_num = (int)i;
    g_had_event = false;

    ge_update (ge, frames[i].hands, frames[i].dt);

    double t_sec = (double)(frames[i].ts_ns - t0) / 1e9;
    bool any_present = frames[i].hands[0].present
                       || frames[i].hands[1].present;

    if (gestures_only && !g_had_event)
      continue;

    if (!any_present && !g_had_event)
      continue;

    printf ("frame %5d  t=%.3fs  dt=%.4f  hands=[%c%c]",
            (int)i, t_sec, frames[i].dt,
            frames[i].hands[0].present ? 'L' : '-',
            frames[i].hands[1].present ? 'R' : '-');
    printf ("\n");

    if (show_features && any_present) {
      for (int h = 0; h < 2; ++h) {
        if (!frames[i].hands[h].present)
          continue;
        printf ("  hand %d features:", h);
        for (int fi = 0; fi < num_features; ++fi) {
          float val = ge_get_feature (ge, h, feature_names[fi]);
          printf (" %s=%.3f", feature_names[fi], val);
          // Line-wrap every 4 features for readability.
          if (fi % 4 == 3 && fi < num_features - 1)
            printf ("\n                  ");
        }
        printf ("\n");
      }
    }

    if (show_joints && any_present) {
      for (int h = 0; h < 2; ++h) {
        if (!frames[i].hands[h].present)
          continue;
        printf ("  hand %d joints:\n", h);
        for (int j = 0; j < GE_JOINT_COUNT; ++j) {
          printf ("    [%2d] pos=(%.4f,%.4f,%.4f) conf=%.2f\n", j,
                  frames[i].hands[h].joints[j][0],
                  frames[i].hands[h].joints[j][1],
                  frames[i].hands[h].joints[j][2],
                  frames[i].hands[h].joints[j][3]);
        }
      }
    }
  }

  // Summary.
  printf ("\n--- Summary ---\n");
  int total_present = 0;
  for (auto &fr : frames)
    if (fr.hands[0].present || fr.hands[1].present)
      total_present++;
  printf ("Frames: %zu total, %d with hand data (%.0f%%)\n",
          frames.size (), total_present,
          frames.empty () ? 0.0
                          : 100.0 * total_present / frames.size ());

  ge_destroy (ge);
  return 0;
}
