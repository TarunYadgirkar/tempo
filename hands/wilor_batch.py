#!/usr/bin/env python3
"""WiLoR offline-batch hand-tracking runner for the Tempo project.

Promised to judges in the check-in-1 doc: a WiLoR trial on sponsored GPU
credits for the hand-tracking pointing path, to compare against the local
Mac RTMPose tracker (hands/, ~5.7 ms/landmark on this Mac via rtmlib on ONNX
Runtime). Runs WiLoR over a recorded set of exported frames, writes a
per-frame JSONL plus a latency/detection summary, and with --compare lines
those results up against an RTMPose JSONL frame-by-frame and reports the
per-landmark Euclidean error in metres and pixels and the latency difference.
That is the number the judges want: WiLoR accuracy + latency vs local RTMPose.

STANDALONE runner. NOT part of the hands uv project; deps NOT added to
hands/pyproject.toml. See hands/wilor_requirements.txt and hands/WILOR_DEPLOY.md.
It DOES import the existing metric pipeline from hands/src/hands (geometry.py,
handshape.py, export_reader.py) so the unprojection math is shared with the
RTMPose path rather than duplicated -- the only variable between the two
columns is the 2D landmark model.

Target model
-----------
WiLoR (Potamias, Zhang, Deng, Zafeiriou), CVPR 2025.
  arXiv: https://arxiv.org/abs/2409.12259
  Repo:  https://github.com/rolpotamias/WiLoR
  Checkpoints (freely downloadable, no auth, from the HF Space):
    pretrained_models/detector.pt        (53.6 MB, Ultralytics YOLO hand detector)
    pretrained_models/wilor_final.ckpt   (2.56 GB, PyTorch-Lightning recon head)
    pretrained_models/model_config.yaml (2.2 kB, REQUIRED by demo.py)
    https://huggingface.co/spaces/rolpotamias/WiLoR/tree/main/pretrained_models
  MANO model (required, SEPARATE license): register at
    https://mano.is.tue.mpg.de and place MANO_RIGHT.pkl under mano_data/.
    This is the one manual step; bake it into the deploy doc, not this script.
  License: CC-BY-NC-ND 4.0
    (https://github.com/rolpotamias/WiLoR/blob/main/license.txt).
    Freely downloadable for non-commercial research (the hackathon qualifies);
    NOT permissive -- cannot ship in a commercial product.

WiLoR is end-to-end: detector.pt finds hand boxes + handedness, the recon head
returns pred_keypoints_3d (B,21,3) MANO joints in a hand-root frame,
pred_vertices (B,778,3), and pred_cam (B,3) weak-perspective (scale, tx, ty).
No per-keypoint confidence from the recon head; we use the YOLO detector's
boxes.conf as the hand confidence.

How a WiLoR hand becomes a scene-frame point
------------------------------------------
To compare apples-to-apples with the RTMPose path we hold the metric pipeline
constant and vary only the 2D landmarks. WiLoR's 3D joints are in a hand-root
frame whose orientation is not the camera frame, so rather than transplanting
them we PROJECT WiLoR's 3D joints back into the exported image's pixels
(WiLoR's own weak-perspective camera, see _project_wilor_to_pixels) and then
run those 2D landmarks through the EXACT same pipeline hands/ uses:
  geometry.palm_range_m  -> one LiDAR range for the hand
  handshape.rigid_ranges -> 21 ranges from a scaled adult hand-shape model
  geometry.unproject     -> each landmark's pixel + range -> camera space
  geometry.camera_to_scene -> head pose -> the frame the panels live in
This is the same code path tracker.py drives for RTMPose, so the only thing
that differs between the two JSONL columns is the 2D landmark model.

Input frame format
-----------------
A batch set is a directory with one subdirectory per recorded frame, each
holding the shell's export triple in the export_reader format:
  <batchset>/frame_000001/latest.json   (seq, t_ns, export_ns, head{pos,quat},
    intrinsics{...}, image{file,format,width,height}, depth{file,width,height})
  <batchset>/frame_000001/latest.rgb    (or latest.jpg; format rgb8 or jpeg)
  <batchset>/frame_000001/latest.depth  (float32 metres, 0 = no reading)
The sidecar's image.file/depth.file are relative to its own subdirectory, so
hands/src/hands/export_reader.py reads each one unchanged. Use `record` to
capture such a set from a live export dir (it copies latest.* into
frame_<seq>/ each time the seq changes).

Output
------
Per-frame JSONL (--out, default wilor_results.jsonl), one line per frame:
  {seq, t_ns, export_ns, backend, latency_ms, detected, hands: [
    {chirality, confidence, landmarks_px (21x2), joints (21x3 scene m),
     range_m, depth_valid (21 bool)}]}
A summary is printed to stdout and written next to the JSONL as .summary.json:
mean/median/p95 landmark latency, detection rate, frames with a hand.

--compare <rtmpose.jsonl> aligns the two JSONLs by seq and reports, over the
frames where BOTH trackers saw a hand: mean per-landmark Euclidean error in
metres (scene-frame joints) and pixels (landmarks_px), plus the mean latency
difference (WiLoR - RTMPose). This is inter-tracker agreement; true accuracy
would need a labelled set, and the summary says so.

Usage
-----
  # capture a batch set from a live shell export (on the Mac):
  python hands/wilor_batch.py record --live $TMPDIR/spatula-frames \
      --out batchset --seconds 60

  # run WiLoR over it (on the GCE L4 VM):
  python hands/wilor_batch.py run --dir batchset --out wilor.jsonl \
      --wilor-root /path/to/WiLoR --ckpt pretrained_models/wilor_final.ckpt \
      --detector pretrained_models/detector.pt \
      --config pretrained_models/model_config.yaml --device cuda

  # produce the RTMPose column with the SAME tool (on the Mac, in the hands env):
  uv run python hands/wilor_batch.py run --dir batchset --out rtmpose.jsonl \
      --backend rtmpose

  # compare:
  python hands/wilor_batch.py compare --wilor wilor.jsonl --rtmpose rtmpose.jsonl

Author: Tarun Yadgirkar. Prep-only: written 2026-09-13, not yet run.
"""

# Standalone deps (NOT in hands/pyproject.toml; see wilor_requirements.txt):
#   --backend wilor: torch, numpy, opencv-python, ultralytics==8.1.34, and the
#     WiLoR repo on PYTHONPATH (wilor.models.load_wilor,
#     wilor.datasets.vitdet_dataset.ViTDetDataset, wilor.utils.recursive_to).
#   --backend rtmpose / compare / record: the hands uv env (numpy, pillow,
#     rtmlib, onnxruntime) -- i.e. run via uv run.
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

import numpy as np

# Shared metric pipeline (lives in the hands package). We import the math
# rather than restating it, so a change to the unprojection on the live path
# also changes this comparison. hands/src is one directory up from here.
_HANDS_SRC = Path(__file__).resolve().parent / "src"
if str(_HANDS_SRC) not in sys.path:
    sys.path.insert(0, str(_HANDS_SRC))

from hands.geometry import camera_to_scene, unproject  # noqa: E402
from hands.handshape import palm_range_m, rigid_ranges  # noqa: E402
from hands.export_reader import ExportReader  # noqa: E402


# --------------------------------------------------------------------------- #
# frame IO: a recorded batch set = one subdir per frame, each a latest.* triple
# --------------------------------------------------------------------------- #

def _list_frames(batch_dir):
    """Subdirs of batch_dir that contain a latest.json sidecar, sorted by the
    sidecar's seq (lexicographic name order as a tiebreaker)."""
    frames = [p for p in sorted(batch_dir.iterdir())
              if p.is_dir() and (p / "latest.json").exists()]

    def key(d):
        try:
            seq = int(json.loads((d / "latest.json").read_text()).get("seq", 0))
        except (OSError, ValueError):
            seq = 0
        return (seq, d.name)

    return sorted(frames, key=key)


def _read_one_frame(subdir):
    """Read one recorded frame triple via the existing ExportReader (handles
    jpeg/rgb8 + float32 depth + the sidecar schema). ExportReader polls a
    directory for a CHANGING latest.json; here the file is static, so we clear
    the mtime cache to force a fresh read."""
    reader = ExportReader(subdir)
    reader._last_mtime_ns = -1
    return reader.read()


# --------------------------------------------------------------------------- #
# record: capture a batch set from a live export dir
# --------------------------------------------------------------------------- #

def cmd_record(args):
    """Copy the live latest.* into <out>/frame_<seq>/ each time the seq changes.

    The live export dir only ever holds one rolling latest.* triple, so to run
    an offline batch you first have to snapshot it. This watches the sidecar's
    seq and copies the three files into a per-frame subdir the instant a new
    frame lands, without touching the shell."""
    live = Path(args.live)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    reader = ExportReader(live)
    seen = set()
    t0 = time.monotonic()
    n = 0
    print(f"recording from {live} into {out} for {args.seconds}s", flush=True)
    while time.monotonic() - t0 < args.seconds:
        frame = reader.wait(timeout_s=1.0)
        if frame is None or frame.seq in seen:
            continue
        seen.add(frame.seq)
        dest = out / f"frame_{frame.seq:09d}"
        dest.mkdir(parents=True, exist_ok=True)
        meta = json.loads((live / "latest.json").read_text())
        shutil.copy2(live / "latest.json", dest / "latest.json")
        img = meta.get("image") or {}
        if img.get("file"):
            shutil.copy2(live / img["file"], dest / img["file"])
        dep = meta.get("depth") or {}
        if dep.get("file"):
            shutil.copy2(live / dep["file"], dest / dep["file"])
        n += 1
        if n % 30 == 0:
            print(f"  {n} frames captured", flush=True)
    print(f"captured {n} frames into {out}", flush=True)
    return 0


# --------------------------------------------------------------------------- #
# WiLoR 2D projection
# --------------------------------------------------------------------------- #

# Reference crop size the ViTDetDataset uses for the recon head. The exact
# value is repo-version-dependent; if the projection looks offset in practice,
# read it from the model config instead of hard-coding it here.
_CROP_REF_PX = 224.0


def _project_wilor_to_pixels(kpts3d, pred_cam, box_center):
    """Project WiLoR's hand-root 3D joints back into the exported image's
    pixels, so the downstream metric pipeline is identical to the RTMPose path.

    WiLoR's recon head emits pred_cam = (scale, tx, ty) in a weak-perspective
    camera defined over the CROPPED hand box (origin at the box center, focal
    length normalised to the crop). The standard HaMeR/WiLoR remap to the full
    image is:

        focal_crop = scale * (crop_ref / 2)            # px, in crop coords
        u_crop = focal_crop * (kx / kz) + tx * (crop_ref / 2)
        v_crop = focal_crop * (-ky / kz) + ty * (crop_ref / 2)  # +Y up -> v down
        u_full = box_center_x + u_crop
        v_full = box_center_y + v_crop

    ASSUMPTION: the crop focal length uses the crop's half-extent as the
    reference, matching the ViTDetDataset crop in the WiLoR repo. The exact
    scale constant is repo-version-dependent; if the projection looks offset
    in practice, read batch['cam_crop_to_full'] and the crop focal length from
    the model config rather than reconstructing them here. This is the one
    place the comparison depends on WiLoR-internal details; the metric pipeline
    below it is shared with the RTMPose path.
    """
    kpts = np.asarray(kpts3d, dtype=np.float64)  # (21,3) hand-root, +Y up
    scale, tx, ty = (float(pred_cam[0]), float(pred_cam[1]), float(pred_cam[2]))
    focal_crop = scale * (_CROP_REF_PX / 2.0)
    x, y, z = kpts[:, 0], kpts[:, 1], kpts[:, 2]
    z = np.where(z == 0.0, 1e-6, z)
    u_crop = focal_crop * (x / z) + tx * (_CROP_REF_PX / 2.0)
    v_crop = focal_crop * (-y / z) + ty * (_CROP_REF_PX / 2.0)
    cx, cy = float(box_center[0]), float(box_center[1])
    return np.stack([cx + u_crop, cy + v_crop], axis=1)  # (21,2) px


# --------------------------------------------------------------------------- #
# WiLoR backend
# --------------------------------------------------------------------------- #

class WiLoRBackend:
    """Loads WiLoR (recon head + YOLO detector) and turns one RGB frame into a
    list of (landmarks_px, confidence, chirality) per detected hand.

    landmarks_px are WiLoR's 3D MANO joints projected back into the exported
    image's pixels by WiLoR's own weak-perspective camera, so the downstream
    metric pipeline (depth + unproject + scene) is identical to the RTMPose
    path. See _project_wilor_to_pixels for the projection and its assumptions.
    """

    def __init__(self, wilor_root, ckpt, detector, config,
                 device="cuda", conf=0.3, fast=False):
        import torch
        from ultralytics import YOLO
        wilor_root = Path(wilor_root)
        if str(wilor_root) not in sys.path:
            sys.path.insert(0, str(wilor_root))
        from wilor.models import load_wilor  # type: ignore
        from wilor.utils import recursive_to  # type: ignore
        from wilor.datasets.vitdet_dataset import ViTDetDataset  # type: ignore

        self.torch = torch
        self.recursive_to = recursive_to
        self.ViTDetDataset = ViTDetDataset
        self.device = torch.device(device if torch.cuda.is_available() else "cpu")
        model, model_cfg = load_wilor(checkpoint_path=str(ckpt), cfg_path=str(config))
        self.model = model.to(self.device).eval()
        self.model_cfg = model_cfg
        self.detector = YOLO(str(detector)).to(self.device)
        self.conf = conf
        self.fast = fast
        if fast:
            # FP16 + optional torch.compile, per the repo's --fast path.
            # ~1.6x speedup, ~0.05 mm MPJPE degradation. Safe on an L4.
            self.model = self.model.half()
            try:
                self.model = self.torch.compile(self.model)
            except Exception as exc:  # compile is optional
                print(f"wilor: torch.compile unavailable ({exc}); running eager",
                      flush=True)

    def detect(self, rgb):
        """rgb: (h,w,3) uint8 RGB. Returns list of dicts with landmarks_px
        (21,2) float px in image coords, confidence float, chirality str."""
        torch = self.torch
        img_bgr = rgb[:, :, ::-1]  # WiLoR's demo feeds BGR
        detections = self.detector(img_bgr, conf=self.conf, verbose=False)[0]
        if detections.boxes is None or len(detections.boxes) == 0:
            return []
        bboxes, is_right, confs = [], [], []
        for det in detections:
            Bbox = det.boxes.data.cpu().detach().squeeze().numpy()
            bboxes.append(Bbox[:4].tolist())
            is_right.append(int(det.boxes.cls.cpu().detach().squeeze().item()))
            confs.append(float(det.boxes.conf.cpu().detach().squeeze().item()))
        boxes = np.stack(bboxes).astype(np.float32)
        right = np.stack(is_right).astype(np.float32)

        dataset = self.ViTDetDataset(
            self.model_cfg, img_bgr, boxes, right, rescale_factor=2.0)
        batch = next(iter(torch.utils.data.DataLoader(
            dataset, batch_size=len(boxes), num_workers=0)))
        batch = self.recursive_to(batch, self.device)

        with torch.no_grad():
            out = self.model(batch)

        hands = []
        for i in range(boxes.shape[0]):
            kpts3d = out["pred_keypoints_3d"][i].detach().float().cpu().numpy()
            pred_cam = out["pred_cam"][i].detach().float().cpu().numpy()
            box_center = boxes[i].reshape(2, 2).mean(axis=0)  # (cx, cy) px
            lpx = _project_wilor_to_pixels(kpts3d, pred_cam, box_center)
            hands.append({
                "landmarks_px": lpx,
                "confidence": confs[i],
                "chirality": "right" if right[i] > 0.5 else "left",
            })
        return hands

    def close(self):
        del self.model, self.detector
        if self.torch.cuda.is_available():
            self.torch.cuda.empty_cache()


# --------------------------------------------------------------------------- #
# RTMPose backend (the local column; uses the existing hands tracker)
# --------------------------------------------------------------------------- #

class RTMPoseBatchBackend:
    """Runs the existing hands RTMPose tracker over one frame and returns the
    same dict shape as WiLoRBackend.detect, so the metric stage and the JSONL
    are identical between the two columns. This is the LOCAL baseline that the
    judges' WiLoR number is compared against; it runs on the Mac in the hands
    uv env (rtmlib + onnxruntime), NOT on the GCE VM."""

    def __init__(self, **tracker_kwargs):
        from hands.tracker import HandTracker
        self.tracker = HandTracker(backend="rtmpose", **tracker_kwargs)

    def detect(self, rgb, frame):
        """Returns list of dicts {landmarks_px, confidence, chirality, joints,
        range_m, depth_valid} for the hands the tracker found this frame.

        The hands tracker already produces scene-frame joints, so for the
        RTMPose column we take its joints directly rather than re-running the
        metric stage (which would double-apply the depth)."""
        result = self.tracker.track(frame)
        out = []
        for h in result.hands:
            out.append({
                "landmarks_px": None,  # RTMPose path does not surface raw px
                "confidence": float(h.confidence),
                "chirality": h.chirality,
                "joints": np.asarray(h.joints, dtype=np.float64),
                "range_m": float(h.range_m),
                "depth_valid": np.asarray(h.depth_valid, dtype=bool),
            })
        return out

    def close(self):
        self.tracker.close()


# --------------------------------------------------------------------------- #
# metric stage: 2D landmarks + frame -> scene-frame joints (shared with RTMPose)
# --------------------------------------------------------------------------- #

def _to_scene(frame, landmarks_px, depth_window=5, dorsal=True):
    """Run the SAME metric pipeline hands/tracker.py uses, on a set of 2D
    pixel landmarks: one LiDAR range for the hand, rigid hand-shape ranges,
    unproject to camera space, transform to the scene frame. Returns
    (joints (21,3) scene m, range_m, depth_valid (21,) bool) or None.

    This is the apples-to-apples path for WiLoR: the only thing that differs
    from the RTMPose column is the 2D landmarks fed in here."""
    if not frame.ready_for_metric_3d():
        return None
    lpx = np.asarray(landmarks_px, dtype=np.float64)
    measured, _samples = palm_range_m(
        frame.depth, lpx, frame.width, frame.height, depth_window)
    if measured <= 0.0:
        return None
    depths = rigid_ranges(
        frame.intrinsics, frame.width, frame.height, lpx, measured,
        world=None, dorsal=dorsal)
    joints = np.zeros((lpx.shape[0], 3), dtype=np.float64)
    for j, (u, v) in enumerate(lpx):
        p_cam = unproject(frame.intrinsics, frame.width, frame.height,
                          u, v, depths[j])
        if p_cam is None:
            return None
        joints[j] = camera_to_scene(frame.head_pos, frame.head_quat, p_cam)
    # depth_valid = which landmarks had a real LiDAR reading under them
    from hands.geometry import depth_patch_values
    dv = np.array([
        depth_patch_values(frame.depth, u / frame.width, v / frame.height,
                           depth_window).size > 0
        for u, v in lpx
    ], dtype=bool)
    return joints, measured, dv


# --------------------------------------------------------------------------- #
# run: walk a batch set, write the JSONL, print the summary
# --------------------------------------------------------------------------- #

def _hand_record(hand, with_joints):
    rec = {
        "chirality": hand["chirality"],
        "confidence": float(hand["confidence"]),
    }
    lpx = hand.get("landmarks_px")
    rec["landmarks_px"] = (
        np.asarray(lpx, dtype=np.float64).tolist() if lpx is not None else None)
    if with_joints and hand.get("joints") is not None:
        rec["joints"] = np.asarray(hand["joints"], dtype=np.float64).tolist()
        rec["range_m"] = float(hand.get("range_m", 0.0))
        dv = hand.get("depth_valid")
        rec["depth_valid"] = np.asarray(dv, dtype=bool).tolist() if dv is not None \
            else None
    return rec


def cmd_run(args):
    frames = _list_frames(Path(args.dir))
    if not frames:
        print(f"wilor: no frames under {args.dir} "
              f"(expected one subdir per frame, each holding latest.* )",
              file=sys.stderr)
        return 1

    backend_name = args.backend
    if backend_name == "wilor":
        if not args.ckpt or not args.detector or not args.config:
            print("wilor: --ckpt --detector --config are required for "
                  "--backend wilor", file=sys.stderr)
            return 2
        backend = WiLoRBackend(args.wilor_root, args.ckpt, args.detector,
                               args.config, device=args.device,
                               conf=args.conf, fast=args.fast)
        is_rtmpose = False
    else:
        backend = RTMPoseBatchBackend()
        is_rtmpose = True

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    latencies = []
    n_frames = 0
    n_with_hand = 0

    with out_path.open("w") as f:
        for subdir in frames:
            frame = _read_one_frame(subdir)
            if frame is None:
                continue
            n_frames += 1
            t0 = time.perf_counter()
            if is_rtmpose:
                hands = backend.detect(frame.rgb, frame)
            else:
                raw = backend.detect(frame.rgb)
                # run the shared metric stage so the joints are scene-frame m
                built = []
                for h in raw:
                    scene = _to_scene(frame, h["landmarks_px"],
                                      depth_window=args.depth_window,
                                      dorsal=not args.palmar_view)
                    if scene is None:
                        continue
                    joints, range_m, dv = scene
                    h = dict(h)
                    h["joints"] = joints
                    h["range_m"] = range_m
                    h["depth_valid"] = dv
                    built.append(h)
                hands = built
            latency_ms = (time.perf_counter() - t0) * 1000.0
            latencies.append(latency_ms)
            if hands:
                n_with_hand += 1

            rec = {
                "seq": int(frame.seq),
                "t_ns": int(frame.t_ns),
                "export_ns": int(frame.export_ns),
                "backend": backend_name,
                "latency_ms": round(latency_ms, 3),
                "detected": len(hands),
                "hands": [_hand_record(h, with_joints=True) for h in hands],
            }
            f.write(json.dumps(rec) + "\n")
            f.flush()
            if n_frames % 30 == 0:
                print(f"  {n_frames}/{len(frames)}  "
                      f"last {latency_ms:.1f} ms  hands {len(hands)}", flush=True)

    backend.close()

    summary = _summarize(latencies, n_frames, n_with_hand, backend_name)
    summary_path = out_path.with_suffix(out_path.suffix + ".summary.json")
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")
    _print_summary(summary)
    print(f"\nwrote {out_path}\nwrote {summary_path}")

    if args.compare:
        _compare_files(Path(args.compare), out_path)
    return 0


def _summarize(latencies, n_frames, n_with_hand, backend_name):
    def pct(p):
        if not latencies:
            return None
        s = sorted(latencies)
        k = max(0, min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1)))))
        return round(s[k], 3)
    return {
        "backend": backend_name,
        "frames": n_frames,
        "frames_with_hand": n_with_hand,
        "detect_rate": round(n_with_hand / n_frames, 4) if n_frames else 0.0,
        "latency_ms_mean": round(float(np.mean(latencies)), 3) if latencies else None,
        "latency_ms_median": round(float(np.median(latencies)), 3) if latencies else None,
        "latency_ms_p95": pct(95),
    }


def _print_summary(s):
    print(f"\nsummary [{s['backend']}]")
    print(f"  frames              {s['frames']}")
    print(f"  frames with a hand  {s['frames_with_hand']} "
          f"({s['detect_rate']:.1%})")
    print(f"  latency mean        {s['latency_ms_mean']} ms")
    print(f"  latency median       {s['latency_ms_median']} ms")
    print(f"  latency p95         {s['latency_ms_p95']} ms")


# --------------------------------------------------------------------------- #
# compare: align two JSONLs by seq, report per-landmark error + latency diff
# --------------------------------------------------------------------------- #

def _load_jsonl(path):
    by_seq = {}
    with Path(path).open() as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            by_seq[int(rec["seq"])] = rec
    return by_seq


def cmd_compare(args):
    return _compare_files(Path(args.wilor), Path(args.rtmpose))


def _compare_files(wilor_path, rtmpose_path):
    w = _load_jsonl(wilor_path)
    r = _load_jsonl(rtmpose_path)
    common = sorted(set(w) & set(r))
    if not common:
        print(f"compare: no common seq between {wilor_path} and {rtmpose_path}",
              file=sys.stderr)
        return 1

    # match hands by chirality within each frame
    joint_err_m = []   # per-landmark euclidean, metres
    px_err = []        # per-landmark euclidean, pixels
    lat_diff = []      # WiLoR latency - RTMPose latency, ms
    frames_both = 0

    for seq in common:
        wf = w[seq]
        rf = r[seq]
        wh = {h["chirality"]: h for h in wf.get("hands", [])}
        rh = {h["chirality"]: h for h in rf.get("hands", [])}
        matched = False
        for ch in set(wh) & set(rh):
            wj = wh[ch].get("joints")
            rj = rh[ch].get("joints")
            if wj is None or rj is None:
                continue
            wj = np.asarray(wj, dtype=np.float64)
            rj = np.asarray(rj, dtype=np.float64)
            d = np.linalg.norm(wj - rj, axis=1)  # (21,) metres
            joint_err_m.append(d.tolist())
            matched = True
            wpx = wh[ch].get("landmarks_px")
            rpx = rh[ch].get("landmarks_px")
            if wpx is not None and rpx is not None:
                wpx = np.asarray(wpx, dtype=np.float64)
                rpx = np.asarray(rpx, dtype=np.float64)
                dp = np.linalg.norm(wpx - rpx, axis=1)  # (21,) px
                px_err.append(dp.tolist())
        if matched:
            frames_both += 1
            lat_diff.append(float(wf["latency_ms"]) - float(rf["latency_ms"]))

    if not joint_err_m:
        print("compare: no frames where both trackers saw a hand with joints",
              file=sys.stderr)
        return 1

    joint_err_m = np.array(joint_err_m)        # (N, 21) metres
    per_joint_m = joint_err_m.mean(axis=0)
    overall_m = float(per_joint_m.mean())
    px_err_arr = np.array(px_err) if px_err else None
    per_joint_px = px_err_arr.mean(axis=0) if px_err_arr is not None else None
    overall_px = float(per_joint_px.mean()) if per_joint_px is not None else None

    report = {
        "frames_compared": frames_both,
        "frames_wilor_hand": sum(1 for s in common if w[s].get("hands")),
        "frames_rtmpose_hand": sum(1 for s in common if r[s].get("hands")),
        "per_landmark_error_m": [round(float(v), 4) for v in per_joint_m],
        "mean_per_landmark_error_m": round(overall_m, 4),
        "median_per_landmark_error_m": round(float(np.median(per_joint_m)), 4),
        "p95_per_landmark_error_m": round(
            float(np.percentile(per_joint_m, 95)), 4),
        "per_landmark_error_px": (
            [round(float(v), 2) for v in per_joint_px]
            if per_joint_px is not None else None),
        "mean_per_landmark_error_px": (
            round(overall_px, 2) if overall_px is not None else None),
        "latency_diff_ms_mean": round(float(np.mean(lat_diff)), 3),
        "latency_diff_ms_median": round(float(np.median(lat_diff)), 3),
        "note": (
            "Inter-tracker agreement between WiLoR and RTMPose over the same "
            "frames, holding the metric pipeline (LiDAR range + rigid hand-shape "
            "+ unproject + scene transform) constant. True accuracy would "
            "require a labelled ground-truth set."),
    }
    print(json.dumps(report, indent=2))
    cmp_path = wilor_path.with_suffix(wilor_path.suffix + ".compare.json")
    cmp_path.write_text(json.dumps(report, indent=2) + "\n")
    print(f"\nwrote {cmp_path}")
    return 0


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def build_parser():
    p = argparse.ArgumentParser(
        prog="wilor_batch",
        description="WiLoR offline-batch hand-tracking runner + RTMPose comparison",
    )
    sub = p.add_subparsers(dest="command", required=True)

    rec = sub.add_parser("record", help="snapshot a live export dir into a batch set")
    rec.add_argument("--live", required=True, help="live frame-export directory")
    rec.add_argument("--out", required=True, help="output batch-set directory")
    rec.add_argument("--seconds", type=float, default=60.0)

    run = sub.add_parser("run", help="run a backend over a batch set -> JSONL")
    run.add_argument("--dir", required=True, help="batch-set directory (one subdir per frame)")
    run.add_argument("--out", default="wilor_results.jsonl", help="output JSONL path")
    run.add_argument("--backend", choices=("wilor", "rtmpose"), default="wilor")
    # wilor-only
    run.add_argument("--wilor-root", help="path to the cloned WiLoR repo (for PYTHONPATH)")
    run.add_argument("--ckpt", help="pretrained_models/wilor_final.ckpt")
    run.add_argument("--detector", help="pretrained_models/detector.pt")
    run.add_argument("--config", help="pretrained_models/model_config.yaml")
    run.add_argument("--device", default="cuda")
    run.add_argument("--conf", type=float, default=0.3, help="YOLO hand detector confidence")
    run.add_argument("--fast", action="store_true", help="FP16 + torch.compile")
    # shared metric stage
    run.add_argument("--depth-window", type=int, default=5,
                     help="LiDAR patch side (matches hands --depth-window)")
    run.add_argument("--palmar-view", action="store_true",
                     help="camera sees palms, not backs (matches hands --palmar-view)")
    run.add_argument("--compare", help="also compare against this RTMPose JSONL after the run")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.command == "record":
        return cmd_record(args)
    if args.command == "compare":
        return cmd_compare(args)
    return cmd_run(args)


if __name__ == "__main__":
    sys.exit(main())
