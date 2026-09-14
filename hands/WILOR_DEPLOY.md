# WiLoR offline batch — GCE L4 deploy runbook

This is how to run the WiLoR hand-pose trial promised to judges in the
check-in-1 doc: a one-shot offline batch on a sponsored Google Cloud L4 GPU
VM, comparing WiLoR's accuracy + latency against the local Mac RTMPose
tracker. The runner is `hands/wilor_batch.py`; this doc is the exact sequence
of `gcloud` commands to bring a VM up, run it, pull the results back, and
tear it down.

**Hard deadline.** The Google Cloud project `eastwest72hack26bos-505` is
hard-deleted **Mon Sep 14 9 AM PT** (the final deadline). The whole run — VM
up, batch, results down, VM deleted — must finish before then. Budget ~30
minutes of VM time; start the VM no later than 8 AM PT on the 14th and delete
it the moment the JSONL is on your Mac.

**License reminder.** WiLoR is CC-BY-NC-ND 4.0 (non-commercial, no
derivatives). Fine for the hackathon; do NOT ship in a commercial product.
MANO (`MANO_RIGHT.pkl`) is a separate research-only license you accept by
registering at https://mano.is.tue.mpg.de.

## Cost (rough)

- L4 GPU VM: `g2-standard-4` (1x L4, 4 vCPU, 16 GB) is ~\$0.70/hr on-demand in
  `us-central1` (the L4 is ~\$0.56/hr + the vCPU/RAM). Spot is ~\$0.21/hr but
  can be preempted mid-batch — for a 30-min one-shot, pay on-demand.
- **~30 min of VM time ≈ \$0.35.** Round to \$1 including boot, model
  download, and a re-run. The 2.56 GB checkpoint download is the slow part.

## 0. On the Mac first — capture the batch set and the RTMPose column

The live export dir (`$TMPDIR/spatula-frames`) only ever holds one rolling
`latest.*` triple, so snapshot it into a batch set before anything else.
Bring the rig up (reopen SpatialBridge on the iPhone, `shell/scripts/run-mac.sh
--restart`), then with a hand in view:

```sh
cd "/Users/tarunyadgirkar/TarunsCode/east west hackathon/tempo"

# capture ~60s of frames into a per-frame-subdir batch set
python hands/wilor_batch.py record \
    --live "$TMPDIR/spatula-frames" --out ~/tempo-batchset --seconds 60

# produce the RTMPose column over the SAME frames (uses the hands uv env)
uv run python hands/wilor_batch.py run \
    --dir ~/tempo-batchset --out ~/rtmpose.jsonl --backend rtmpose
```

The RTMPose column has to be produced on the Mac (rtmlib + onnxruntime +
the live hands env). WiLoR runs on the VM. Both JSONLs are then compared on
either machine with `compare`.

## 1. Create the L4 VM

```sh
gcloud config set project eastwest72hack26bos-505
gcloud config set compute/zone us-central1-a

# one L4, 4 vCPU, 16 GB, Debian 12 + CUDA 12 + a 100 GB SSD
gcloud compute instances create tempo-wilor \
    --machine-type=g2-standard-4 \
    --zone=us-central1-a \
    --image-project=ml-images --image-family=common-cu120-debian12 \
    --boot-disk-size=100GB --boot-disk-type=pd-ssd \
    --provisioning-model=STANDARD
```

If `g2-standard-4` is unavailable in `us-central1-a`, try `us-central1-b`,
`us-central1-c`, or `us-central1-f`. The `ml-images` common-cu120 family
ships CUDA 12 + a recent driver; WiLoR wants CUDA 11.7 but the cu117 torch
wheel runs fine under a CUDA 12 driver (forward-compatible). If you prefer
the exact match, use `--image-family=common-cu117-debian11` instead.

## 2. Ship the code + batch set up

```sh
# the runner + the shared hands/src pipeline + the batch set
gcloud compute scp --zone=us-central1-a \
    hands/wilor_batch.py hands/wilor_requirements.txt \
    ~/tempo-batchset \
    tempo-wilor:~/wilor-run/

# the hands package source (geometry/handshape/export_reader imports)
gcloud compute scp --zone=us-central1-a --recurse \
    hands/src tempo-wilor:~/wilor-run/hands-src
```

## 3. On the VM — install deps, clone WiLoR, fetch checkpoints

```sh
gcloud compute ssh tempo-wilor --zone=us-central1-a
# --- inside the VM ---
cd ~/wilor-run

python3 -m venv .venv && source .venv/bin/activate

# torch first (CUDA 11.7 wheel; runs under the CUDA 12 driver)
pip install torch==2.0.0 torchvision==0.15.1 \
    --index-url https://download.pytorch.org/whl/cu117

# the rest of the runner's deps
pip install -r wilor_requirements.txt

# clone WiLoR and fetch its checkpoints (free, no auth)
git clone https://github.com/rolpotamias/WiLoR.git
cd WiLoR && git submodule update --init --recursive && cd ..
mkdir -p WiLoR/pretrained_models
wget https://huggingface.co/spaces/rolpotamias/WiLoR/resolve/main/pretrained_models/detector.pt \
    -O WiLoR/pretrained_models/detector.pt
wget https://huggingface.co/spaces/rolpotamias/WiLoR/resolve/main/pretrained_models/wilor_final.ckpt \
    -O WiLoR/pretrained_models/wilor_final.ckpt
wget https://huggingface.co/spaces/rolpotamias/WiLoR/resolve/main/pretrained_models/model_config.yaml \
    -O WiLoR/pretrained_models/model_config.yaml

# MANO: you must have downloaded MANO_RIGHT.pkl from mano.is.tue.mpg.de on the
# Mac (free registration). scp it up:
#   (on the Mac) gcloud compute scp --zone=us-central1-a \
#       ~/Downloads/MANO_RIGHT.pkl tempo-wilor:~/wilor-run/WiLoR/mano_data/MANO_RIGHT.pkl
mkdir -p WiLoR/mano_data
# ...place MANO_RIGHT.pkl into WiLoR/mano_data/ ...
```

## 4. On the VM — run the batch

```sh
# point the runner at hands-src so the shared pipeline imports resolve
export PYTHONPATH=~/wilor-run/hands-src:$PYTHONPATH

python wilor_batch.py run \
    --dir ~/wilor-run/tempo-batchset \
    --out ~/wilor-run/wilor.jsonl \
    --backend wilor \
    --wilor-root ~/wilor-run/WiLoR \
    --ckpt ~/wilor-run/WiLoR/pretrained_models/wilor_final.ckpt \
    --detector ~/wilor-run/WiLoR/pretrained_models/detector.pt \
    --config ~/wilor-run/WiLoR/pretrained_models/model_config.yaml \
    --device cuda --fast
```

`--fast` enables FP16 + `torch.compile` (~1.6x speedup, ~0.05 mm MPJPE
degradation). Drop it if you want the cleanest accuracy number. The run
prints a one-line-per-30-frames progress and a summary at the end; it also
writes `wilor.jsonl.summary.json`.

## 5. Pull the results back down and compare

```sh
# (back on the Mac)
gcloud compute scp --zone=us-central1-a \
    tempo-wilor:~/wilor-run/wilor.jsonl \
    tempo-wilor:~/wilor-run/wilor.jsonl.summary.json \
    ~/tempo-wilor-results/

# compare WiLoR vs the RTMPose column you produced on the Mac in step 0
python hands/wilor_batch.py compare \
    --wilor ~/tempo-wilor-results/wilor.jsonl \
    --rtmpose ~/rtmpose.jsonl
```

The compare command prints and writes `wilor.jsonl.compare.json`: per-landmark
Euclidean error in metres (scene-frame joints) and pixels (landmarks_px),
plus the mean latency difference (WiLoR - RTMPose). That is the number for
the judges: WiLoR accuracy + latency vs local RTMPose. It is inter-tracker
agreement over the same frames with the metric pipeline held constant; true
accuracy would need a labelled ground-truth set.

## 6. Tear the VM down

Do this the moment the JSONL is on your Mac — the project is deleted at 9 AM
PT and you do not want a leftover VM on a deleted project generating billing
alerts.

```sh
gcloud compute instances delete tempo-wilor --zone=us-central1-a --delete-disks
```

## Notes / gotchas

- **MANO is the one manual step.** `MANO_RIGHT.pkl` cannot be downloaded by a
  script; a human accepts the license at https://mano.is.tue.mpg.de. Do it on
  the Mac before step 2 and scp the file up.
- **ultralytics is pinned to 8.1.34.** `detector.pt` is an Ultralytics pickle
  that imports `ultralytics.nn.modules.*` symbols; a newer Ultralytics may
  fail to deserialize it.
- **chumpy + smplx are fiddly.** `chumpy @ git+https://github.com/mattloper/chumpy`
  and `smplx==0.1.28` are the versions the repo was built against. If pip
  struggles, install chumpy first, then smplx, then the rest.
- **pyrender is intentionally omitted** from `wilor_requirements.txt` (it
  pulls PyOpenGL, which wants a display on a headless VM). Landmark inference
  does not need it; only the demo's mesh renderer does.
- **The 2D projection assumption.** `wilor_batch.py` projects WiLoR's 3D joints
  back to image pixels with a weak-perspective remap whose crop reference
  size (224 px) is hard-coded. If the projected landmarks look systematically
  offset, read `batch['cam_crop_to_full']` and the crop focal length from the
  model config instead — that is the one place the comparison depends on
  WiLoR-internal details. The metric pipeline below it is shared with the
  RTMPose path, so the comparison stays apples-to-apples.
- **Inter-tracker, not ground-truth.** The compare number is agreement
  between WiLoR and RTMPose, not absolute accuracy. Say so in the writeup; the
  briefings reward an honest failure case over a polished number.
