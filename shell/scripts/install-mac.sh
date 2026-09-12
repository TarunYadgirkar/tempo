#!/usr/bin/env bash
# install-mac.sh — build the signed Spatula app bundle and install it to
# /Applications/Spatula.app (falls back to ~/Applications if /Applications
# is not writable).
#
# Why an install step at all: macOS TCC keys Screen Recording / Accessibility
# grants to the app's bundle id + code signature + path. Running rebuilt,
# ad-hoc-signed bundles out of build/ made every rebuild look like a new app,
# so grants kept resetting and stale "mac-shell" rows piled up in System
# Settings. Installing one signed copy at a fixed path means you grant
# Screen Recording to "Spatula" exactly once; every later install rsyncs
# over the same path with the same identity, and the grant persists.
#
# Signing identity: pass MAC_SHELL_CODESIGN_ID=<identity> in the env to
# override; otherwise the script uses the single "Apple Development"
# identity in your keychain (errors if there are several; warns + ad-hoc
# signs if there are none — ad-hoc grants do NOT survive rebuilds).
#
# Usage: scripts/install-mac.sh [--force] [-h|--help]
#   --force   install even while the installed app is running (the rsync
#             --delete below rewrites the bundle out from under it)
# env: MAC_SHELL_CODESIGN_ID
set -euo pipefail

usage() { awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; }

FORCE=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --force)   FORCE=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "ERROR: unknown argument '$1' (see --help)"; exit 2 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

BUILD_DIR="$PROJECT_ROOT/build/mac-shell"
BUILT_APP="$BUILD_DIR/mac-shell.app"
OLD_BUNDLE_ID="com.spatialos.macshell"

# ── Where it goes, and is that copy running? ──────────────────────────────
# Resolved before the build so a refusal costs nothing. `rsync --delete`
# rewrites the bundle in place, which corrupts a running Spatula's own
# executable and resources.
if [[ -w /Applications ]]; then
    DEST="/Applications/Spatula.app"
else
    mkdir -p "$HOME/Applications"
    DEST="$HOME/Applications/Spatula.app"
    echo "==> [install-mac] /Applications not writable; using $DEST"
fi

RUNNING_PID="$(pgrep -f "$DEST/Contents/MacOS/mac-shell" 2>/dev/null \
               | head -1 || true)"
if [[ -n "$RUNNING_PID" && "$FORCE" -eq 0 ]]; then
    echo "ERROR: $DEST is running (pid $RUNNING_PID) — installing would" \
         "rewrite the bundle underneath it."
    echo "       Quit Spatula (Cmd+Q) and rerun, or use" \
         "'scripts/run-mac.sh --restart'."
    echo "       Pass --force to install anyway."
    exit 1
fi

# ── Pick the signing identity ─────────────────────────────────────────────
if [[ -z "${MAC_SHELL_CODESIGN_ID:-}" ]]; then
    IDENTITIES="$(security find-identity -v -p codesigning 2>/dev/null \
        | sed -n 's/.*"\(Apple Development: .*\)"/\1/p')"
    COUNT="$(grep -c . <<<"$IDENTITIES" || true)"
    if [[ "$COUNT" -eq 1 ]]; then
        MAC_SHELL_CODESIGN_ID="$IDENTITIES"
        echo "==> [install-mac] Signing identity: $MAC_SHELL_CODESIGN_ID"
    elif [[ "$COUNT" -gt 1 ]]; then
        echo "ERROR: multiple Apple Development identities found:"
        sed 's/^/    /' <<<"$IDENTITIES"
        echo "Pick one: MAC_SHELL_CODESIGN_ID=\"<identity>\" $0"
        exit 1
    else
        MAC_SHELL_CODESIGN_ID="-"
        echo "WARNING: no Apple Development identity in the keychain —"
        echo "         ad-hoc signing. TCC grants will NOT survive rebuilds."
        echo "         (Add an Apple ID in Xcode > Settings > Accounts.)"
    fi
fi

# ── Build the signed bundle ───────────────────────────────────────────────
echo "==> [install-mac] Building Spatula (signed bundle)..."
cmake -G Ninja -S "$PROJECT_ROOT/mac-shell" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DMAC_SHELL_CODESIGN_ID="$MAC_SHELL_CODESIGN_ID"
ninja -C "$BUILD_DIR" mac-shell-app

# ── Install to /Applications (or ~/Applications) ──────────────────────────
echo "==> [install-mac] Installing to $DEST"
rsync -a --delete "$BUILT_APP/" "$DEST/"

echo "==> [install-mac] Installed:"
codesign -dvv "$DEST" 2>&1 | grep -E '^(Identifier|Authority)=' \
    | sed 's/^/    /' || true

# ── One-time cleanup of the old identity's stale grants ───────────────────
cat <<EOF

┌──────────────────────────────────────────────────────────────────────────┐
│  ONE-TIME CLEANUP (only if you ran the old build/ mac-shell.app before)  │
│                                                                          │
│  The old bundles left stale "mac-shell" rows in System Settings.         │
│  1. System Settings > Privacy & Security > Screen Recording:             │
│     remove every "mac-shell" row with the − button (keep "Spatula").     │
│  2. Same under Accessibility, if any "mac-shell" rows exist there.       │
│  3. Optionally clear the old bundle id's TCC records (old id ONLY —      │
│     never run a bare 'tccutil reset ScreenCapture'):                     │
│                                                                          │
│       tccutil reset ScreenCapture $OLD_BUNDLE_ID              │
│       tccutil reset Accessibility $OLD_BUNDLE_ID              │
│                                                                          │
│  Then launch Spatula (./scripts/run-mac.sh), trigger a window capture,   │
│  and grant Screen Recording to "Spatula" ONCE. Future installs reuse     │
│  the same path + signature, so that grant persists.                      │
└──────────────────────────────────────────────────────────────────────────┘
EOF
