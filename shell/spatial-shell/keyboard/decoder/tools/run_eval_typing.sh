#!/bin/sh
# run_eval_typing.sh — ctest wrapper for eval_typing.
#
# Walks ~/spatial-os-recordings/typing/ and runs eval_typing on each
# finalised session bundle.  SKIPs (exit 77 — meson's "test skipped"
# convention) when no sessions exist or the directory is missing, so
# CI doesn't break on a fresh checkout.
#
# Pass criteria (per session):
#   mean_CER ≤ ${SPATIAL_KEYBOARD_CER_MAX:-0.50}  (default lax — the
#   v1 baseline is "anything beats random"; tighten as tuning lands)
#
# SPDX-License-Identifier: MIT

set -u

EVAL=$1
ROOT=${SPATIAL_KEYBOARD_TYPING_ROOT:-$HOME/spatial-os-recordings/typing}
CER_MAX=${SPATIAL_KEYBOARD_CER_MAX:-0.50}

if [ ! -d "$ROOT" ]; then
    echo "eval_typing: $ROOT does not exist — SKIP"
    exit 77
fi

count=0
fails=0
for dir in "$ROOT"/T-*; do
    [ -d "$dir" ] || continue
    [ -f "$dir/transcript.json" ] || continue
    [ -f "$dir/stream.bin" ] || continue
    count=$((count + 1))
    echo "=== $dir ==="
    out=$("$EVAL" "$dir") || {
        echo "eval_typing: $dir failed to run (exit $?)"
        fails=$((fails + 1))
        continue
    }
    echo "$out"
    mean_cer=$(echo "$out" | awk '/^SESSION/ {
        for (i = 1; i <= NF; i++) {
            if ($i ~ /^mean_CER=/) {
                split($i, a, "="); print a[2]; exit
            }
        }
    }')
    [ -n "$mean_cer" ] || continue
    awk -v c="$mean_cer" -v m="$CER_MAX" 'BEGIN { exit !(c + 0 <= m + 0) }' || {
        echo "FAIL: mean_CER=$mean_cer exceeds limit $CER_MAX"
        fails=$((fails + 1))
    }
done

if [ "$count" = 0 ]; then
    echo "eval_typing: no finalised sessions under $ROOT — SKIP"
    exit 77
fi

if [ "$fails" -gt 0 ]; then
    echo "eval_typing: $fails / $count sessions failed"
    exit 1
fi

echo "eval_typing: $count sessions OK, all within CER ≤ $CER_MAX"
exit 0
