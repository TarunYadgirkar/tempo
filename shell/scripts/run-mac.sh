#!/usr/bin/env bash
# run-mac.sh — start Spatula (the macOS spatial shell) as a live rig for
# the SpatialBridge iPhone app:
#   1. Launches the installed /Applications/Spatula.app — auto-running
#      scripts/install-mac.sh first if it's missing or older than the build.
#      One signed app at one path = TCC permissions granted once, forever.
#   2. Prints the Mac's WiFi IP + UDP port (enter these in SpatialBridge)
#   3. Launches via `open` so TCC attributes Screen Recording /
#      Accessibility grants to the app bundle, not this terminal
#   4. Tails the shell log until Ctrl+C
#
# Bonjour is NOT advertised from here — the shell registers
# `Spatula (<Computer Name>)` on _spatialbridge._udp itself, so launching
# from the Dock is just as discoverable (mac-shell/src/platform/renderer.mm).
#
# Usage: scripts/run-mac.sh [flags]
#   --dev                    build + run from build/mac-shell/ instead of the
#                            installed app (TCC grants may not stick there)
#   --replay <session.bin>   replay a recorded session instead of live UDP
#   --port <n>               live UDP port (default 9898)
#   --headless               run the bare CLI binary (no window, CI mode)
#   --restart                quit an already-running Spatula first instead of
#                            refusing to start a second one
#   --check                  don't launch: query a running shell's control
#                            socket (permissions + stats) and exit
#
# Mirrors scripts/run.sh (the Linux stack launcher) for the mac-shell port.

set -euo pipefail

usage() { awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; }
[[ "${1:-}" == "-h" || "${1:-}" == "--help" ]] && { usage; exit 0; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

BUILD_DIR="$PROJECT_ROOT/build/mac-shell"
BUILT_APP="$BUILD_DIR/mac-shell.app"
CLI_BIN="$BUILD_DIR/mac-shell"
# Per-user 0700 log dir — never a predictable world-writable /tmp path.
LOG_DIR="$HOME/Library/Logs/Spatula"
LOG_FILE="$LOG_DIR/mac-shell-run.log"
# The shell itself refuses to bind outside $SPATIAL_OS_SOCK / $TMPDIR (see
# mac-shell/src/platform/control_server.cpp); mirror that here instead of
# guessing /tmp.
if [[ -n "${SPATIAL_OS_SOCK:-}" ]]; then
    SOCK_PATH="$SPATIAL_OS_SOCK"
elif [[ -n "${TMPDIR:-}" ]]; then
    SOCK_PATH="${TMPDIR%/}/spatial-os.sock"
else
    echo "ERROR: neither SPATIAL_OS_SOCK nor TMPDIR is set — no per-user" \
         "location for the control socket (refusing to use /tmp)"
    exit 1
fi

PORT=9898
REPLAY=""
HEADLESS=0
CHECK=0
DEV=0
RESTART=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dev)      DEV=1; shift ;;
        --replay)   REPLAY="$2"; shift 2 ;;
        --port)     PORT="$2"; shift 2 ;;
        --headless) HEADLESS=1; shift ;;
        --restart)  RESTART=1; shift ;;
        --check)    CHECK=1; shift ;;
        -h|--help)  usage; exit 0 ;;
        *) echo "ERROR: unknown argument '$1' (see --help)"; exit 2 ;;
    esac
done

# ── Control-socket client (permissions / stats verbs) ─────────────────────
ctl_request() {
    # ctl_request <verb> — one request/reply line over the unix socket.
    python3 - "$SOCK_PATH" "$1" <<'EOF'
import socket, sys
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.settimeout(3.0)
try:
    s.connect(sys.argv[1])
    s.sendall((sys.argv[2] + "\n").encode())
    buf = b""
    while b"\n" not in buf:
        chunk = s.recv(4096)
        if not chunk:
            break
        buf += chunk
    print(buf.decode().strip())
except (OSError, socket.timeout) as e:
    print(f"<no-reply: {e}>")
    sys.exit(1)
EOF
}

# ── --check: report a running shell's state and exit ──────────────────────
if [[ "$CHECK" -eq 1 ]]; then
    echo "==> [run-mac] Checking running Spatula via $SOCK_PATH"
    if ! PERMS="$(ctl_request permissions)"; then
        echo "ERROR: no Spatula responding on $SOCK_PATH — is it running?"
        exit 1
    fi
    STATS="$(ctl_request stats || true)"
    echo "    permissions: $PERMS"
    echo "    stats:       $STATS"
    case "$PERMS" in
        *screen_capture=1*) echo "    Screen Recording: GRANTED" ;;
        *) echo "    Screen Recording: NOT granted — System Settings >" \
                "Privacy & Security > Screen Recording > enable Spatula" ;;
    esac
    case "$PERMS" in
        *accessibility=1*) echo "    Accessibility:    GRANTED" ;;
        *) echo "    Accessibility:    NOT granted — needed only for input" \
                "injection (pinch/tap inside a panel)" ;;
    esac
    RATE="$(sed -n 's/.*packet_rate=\([0-9.]*\).*/\1/p' <<<"$STATS")"
    if [[ -n "$RATE" ]]; then
        if awk "BEGIN{exit !($RATE > 0)}"; then
            echo "    iPhone link:      LIVE ($RATE packets/s)"
        else
            echo "    iPhone link:      no packets — start streaming in" \
                 "SpatialBridge (same WiFi, Mac IP, port $PORT)"
        fi
    fi
    exit 0
fi

# ── Refuse a second copy ──────────────────────────────────────────────────
# Two Spatulas fight over the UDP port, the control socket and the Bonjour
# name, and the loser half-works. Checked before the install step so
# install-mac.sh never rsyncs over a bundle that is executing.
if [[ "$HEADLESS" -eq 0 ]]; then
    RUNNING_PID=""
    RUNNING_APP=""
    for CAND in "/Applications/Spatula.app" "$HOME/Applications/Spatula.app" \
                "$BUILT_APP"; do
        PID="$(pgrep -f "$CAND/Contents/MacOS/mac-shell" 2>/dev/null \
               | head -1 || true)"
        if [[ -n "$PID" ]]; then
            RUNNING_PID="$PID"
            RUNNING_APP="$CAND"
            break
        fi
    done
    if [[ -n "$RUNNING_PID" && "$RESTART" -eq 0 ]]; then
        echo "ERROR: Spatula is already running (pid $RUNNING_PID) — quit it" \
             "first (or pass --restart)"
        echo "       $RUNNING_APP"
        exit 1
    fi
    if [[ -n "$RUNNING_PID" ]]; then
        echo "==> [run-mac] --restart: stopping Spatula (pid $RUNNING_PID)..."
        kill -TERM "$RUNNING_PID" 2>/dev/null || true
        for _ in $(seq 1 50); do
            kill -0 "$RUNNING_PID" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 "$RUNNING_PID" 2>/dev/null; then
            echo "ERROR: pid $RUNNING_PID ignored SIGTERM for 5 s — quit it" \
                 "by hand (Cmd+Q), then rerun"
            exit 1
        fi
        echo "    stopped."
    fi
fi

# ── Resolve which app to run ──────────────────────────────────────────────
if [[ "$DEV" -eq 1 || "$HEADLESS" -eq 1 ]]; then
    # Dev / CI path: build + run out of build/. TCC keys grants partly to
    # the bundle path, so grants made against the installed app may not
    # apply here (and vice versa) — dev runs can re-prompt.
    if [[ ! -x "$CLI_BIN" || ! -d "$BUILT_APP" ]]; then
        echo "==> [run-mac] Building mac-shell..."
        cmake -G Ninja -S "$PROJECT_ROOT/mac-shell" -B "$BUILD_DIR" \
            -DCMAKE_BUILD_TYPE=RelWithDebInfo
    fi
    ninja -C "$BUILD_DIR"
    APP_BUNDLE="$BUILT_APP"
    if [[ "$DEV" -eq 1 ]]; then
        echo "==> [run-mac] DEV MODE: running $APP_BUNDLE"
        echo "    (TCC grants may not stick to build-dir bundles — use the"
        echo "    installed app for a persistent Screen Recording grant)"
    fi
else
    # Normal path: the installed, signed app. Install/refresh it when it's
    # missing or the build tree holds a newer bundle.
    INSTALLED=""
    for CAND in "/Applications/Spatula.app" "$HOME/Applications/Spatula.app"; do
        [[ -d "$CAND" ]] && INSTALLED="$CAND" && break
    done
    BUILT_EXE="$BUILT_APP/Contents/MacOS/mac-shell"
    # install-mac.sh refuses to rsync over a running bundle; surface that
    # instead of dying mid-script under `set -e`.
    if [[ -z "$INSTALLED" ]]; then
        echo "==> [run-mac] Spatula is not installed yet — installing..."
        if ! "$SCRIPT_DIR/install-mac.sh"; then
            echo "ERROR: install failed (see above) — not launching"
            exit 1
        fi
    elif [[ -x "$BUILT_EXE" \
            && "$BUILT_EXE" -nt "$INSTALLED/Contents/MacOS/mac-shell" ]]; then
        echo "==> [run-mac] Installed Spatula is older than the build —" \
             "reinstalling..."
        if ! "$SCRIPT_DIR/install-mac.sh"; then
            echo "ERROR: install failed (see above) — not launching"
            exit 1
        fi
    fi
    INSTALLED=""
    for CAND in "/Applications/Spatula.app" "$HOME/Applications/Spatula.app"; do
        [[ -d "$CAND" ]] && INSTALLED="$CAND" && break
    done
    if [[ -z "$INSTALLED" ]]; then
        echo "ERROR: install failed — no Spatula.app found"
        exit 1
    fi
    APP_BUNDLE="$INSTALLED"
fi

# ── Replay fallback sanity ────────────────────────────────────────────────
if [[ -n "$REPLAY" && ! -f "$REPLAY" ]]; then
    echo "ERROR: replay file not found: $REPLAY"
    exit 1
fi

# ── WiFi IP banner (SpatialBridge needs this) ─────────────────────────────
WIFI_IF="$(route -n get default 2>/dev/null | sed -n 's/.*interface: //p')"
WIFI_IP=""
for IF in "$WIFI_IF" en0 en1 en2; do
    [[ -n "$IF" ]] || continue
    WIFI_IP="$(ipconfig getifaddr "$IF" 2>/dev/null || true)"
    [[ -n "$WIFI_IP" ]] && break
done

echo ""
echo "┌──────────────────────────────────────────────────────────────────┐"
if [[ -n "$REPLAY" ]]; then
    echo "│  REPLAY MODE — no iPhone needed                                  │"
    echo "│  Session: $REPLAY"
elif [[ -n "$WIFI_IP" ]]; then
    echo "│                                                                  │"
    echo "│      ENTER THIS IN THE SpatialBridge iPhone APP:                 │"
    echo "│                                                                  │"
    echo "│          Mac IP:  $WIFI_IP"
    echo "│          Port:    $PORT (UDP)"
    echo "│                                                                  │"
    echo "│      iPhone and Mac must be on the SAME WiFi network.            │"
    echo "│                                                                  │"
    echo "│      Normally you never type this: Spatula advertises itself     │"
    echo "│      over Bonjour, so just tap it in SpatialBridge.              │"
    echo "│                                                                  │"
else
    echo "│  WARNING: no WiFi IP found — is this Mac on a network?           │"
    echo "│  The iPhone app needs this Mac's IP to stream (port $PORT/UDP).  │"
fi
echo "│                                                                  │"
echo "│  Status check while running:  ./scripts/run-mac.sh --check       │"
echo "│  Ctrl+C here stops the shell.                                    │"
echo "└──────────────────────────────────────────────────────────────────┘"
echo ""

# ── Launch ────────────────────────────────────────────────────────────────
SHELL_ARGS=()
[[ -n "$REPLAY" ]] && SHELL_ARGS+=(--replay "$REPLAY")
[[ "$PORT" != 9898 ]] && SHELL_ARGS+=(--port "$PORT")

APP_PID=""
cleanup() {
    echo ""
    echo "==> [run-mac] Shutting down..."
    if [[ -n "$APP_PID" ]] && kill -0 "$APP_PID" 2>/dev/null; then
        kill "$APP_PID" 2>/dev/null || true
        wait "$APP_PID" 2>/dev/null || true
    fi
    echo "==> [run-mac] Stopped."
}
trap cleanup EXIT INT TERM HUP

if [[ "$HEADLESS" -eq 1 ]]; then
    echo "==> [run-mac] Starting mac-shell (headless CLI)..."
    SPATULA_MAC_HEADLESS=1 "$CLI_BIN" ${SHELL_ARGS[@]+"${SHELL_ARGS[@]}"} &
    APP_PID=$!
    echo "    mac-shell PID: $APP_PID"
    wait "$APP_PID"
    exit $?
fi

# `open` (not direct exec) so launchd is the responsible process and the
# Screen Recording / Accessibility prompts attribute to Spatula.app.
echo "==> [run-mac] Launching $APP_BUNDLE (log: $LOG_FILE)..."
mkdir -p "$LOG_DIR"
chmod 700 "$LOG_DIR"
if [[ -L "$LOG_FILE" ]]; then
    echo "ERROR: $LOG_FILE is a symlink — refusing to truncate through it"
    exit 1
fi
: > "$LOG_FILE"
open -n "$APP_BUNDLE" --stdout "$LOG_FILE" --stderr "$LOG_FILE" \
    --args ${SHELL_ARGS[@]+"${SHELL_ARGS[@]}"}

# Resolve the app pid for the cleanup trap (open returns immediately).
for _ in $(seq 1 50); do
    APP_PID="$(pgrep -nf "$APP_BUNDLE/Contents/MacOS/mac-shell" || true)"
    [[ -n "$APP_PID" ]] && break
    sleep 0.2
done
if [[ -z "$APP_PID" ]]; then
    echo "ERROR: Spatula did not start — log tail:"
    tail -20 "$LOG_FILE" || true
    exit 1
fi
echo "    Spatula PID: $APP_PID"
echo ""

# Tail the log until the app exits or Ctrl+C (BSD tail has no --pid; poll).
tail -f "$LOG_FILE" &
TAIL_PID=$!
while kill -0 "$APP_PID" 2>/dev/null; do sleep 1; done
kill "$TAIL_PID" 2>/dev/null || true
wait "$TAIL_PID" 2>/dev/null || true
echo "==> [run-mac] Spatula exited."
