#!/usr/bin/env bash
# fetch-model.sh — download the MediaPipe Hand Landmarker bundle into
# hands/models/. The .task file is ~7 MB of weights and is gitignored; every
# machine fetches its own copy.
#
# Usage: scripts/fetch-model.sh [-h|--help]
set -euo pipefail

usage() { awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; }
[[ "${1:-}" == "-h" || "${1:-}" == "--help" ]] && { usage; exit 0; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="$(cd "$SCRIPT_DIR/.." && pwd)/models"
MODEL="$MODEL_DIR/hand_landmarker.task"
URL="https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"

mkdir -p "$MODEL_DIR"
if [[ -s "$MODEL" ]]; then
    echo "already have $MODEL ($(wc -c <"$MODEL" | tr -d ' ') bytes)"
    exit 0
fi

echo "downloading $URL"
curl -fL --progress-bar -o "$MODEL.part" "$URL"
mv "$MODEL.part" "$MODEL"
echo "wrote $MODEL ($(wc -c <"$MODEL" | tr -d ' ') bytes)"
