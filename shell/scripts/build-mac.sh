#!/usr/bin/env bash
# build-mac.sh — build the Spatula native core (hackathon shell/ copy):
#   cmake: gesture-engine, bridge-receiver, tests/replay-session, mac-shell
#
# The Linux/Monado/meson components of the upstream tree are not carried
# over here, so this builds the four cmake components only.
#
# Prints elapsed time per component. Companion: scripts/test-mac.sh runs the
# suites these builds produce.
#
# Usage: scripts/build-mac.sh [-h|--help]
set -euo pipefail

usage() { awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; }
[[ "${1:-}" == "-h" || "${1:-}" == "--help" ]] && { usage; exit 0; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Colour helpers (disable if not a terminal)
if [[ -t 1 ]]; then
    C_BOLD="\033[1m"
    C_GREEN="\033[1;32m"
    C_YELLOW="\033[1;33m"
    C_RED="\033[1;31m"
    C_RESET="\033[0m"
else
    C_BOLD="" C_GREEN="" C_YELLOW="" C_RED="" C_RESET=""
fi

# kind:name:source-dir (relative to project root); build dir is build/<name>
COMPONENTS=(
    "cmake:gesture-engine:gesture-engine"
    "cmake:bridge-receiver:bridge-receiver"
    "cmake:replay-session:tests/replay-session"
    "cmake:mac-shell:mac-shell"
)

banner() {
    echo ""
    echo -e "${C_BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}"
    echo -e "${C_BOLD}  BUILD: $1${C_RESET}"
    echo -e "${C_BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}"
}

build_cmake() {
    local src="$1" build="$2"
    cmake -G Ninja -S "$src" -B "$build" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        && ninja -C "$build"
}

TOTAL_START="$(date +%s)"

echo -e "${C_BOLD}==> [build-mac] Spatula — macOS native core build${C_RESET}"
echo "    Project root: $PROJECT_ROOT"
echo "    $(date)"

for entry in "${COMPONENTS[@]}"; do
    KIND="${entry%%:*}"
    REST="${entry#*:}"
    NAME="${REST%%:*}"
    SRC="$PROJECT_ROOT/${REST#*:}"
    BUILD="$PROJECT_ROOT/build/$NAME"

    banner "$NAME"
    STEP_START="$(date +%s)"

    if [[ ! -d "$SRC" ]]; then
        echo -e "${C_YELLOW}WARNING: $NAME source not found at $SRC — skipping${C_RESET}"
        continue
    fi

    case "$KIND" in
        cmake) OK=0; build_cmake "$SRC" "$BUILD" || OK=1 ;;
    esac
    if [[ "$OK" -ne 0 ]]; then
        echo ""
        echo -e "${C_RED}  $NAME FAILED. See output above.${C_RESET}"
        exit 1
    fi

    ELAPSED=$(( $(date +%s) - STEP_START ))
    echo ""
    echo -e "${C_GREEN}  $NAME complete in ${ELAPSED}s${C_RESET}"
done

TOTAL_ELAPSED=$(( $(date +%s) - TOTAL_START ))

echo ""
echo -e "${C_BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}"
echo -e "${C_GREEN}  BUILD COMPLETE in ${TOTAL_ELAPSED}s${C_RESET}"
echo -e "${C_BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}"
echo ""
echo "Artifacts:"
for entry in "${COMPONENTS[@]}"; do
    REST="${entry#*:}"
    NAME="${REST%%:*}"
    if [[ -f "$PROJECT_ROOT/build/$NAME.skipped" ]]; then
        printf "    %-17s SKIPPED (%s)\n" "$NAME:" "$(cat "$PROJECT_ROOT/build/$NAME.skipped")"
    else
        printf "    %-17s %s\n" "$NAME:" "$PROJECT_ROOT/build/$NAME/"
    fi
done
echo ""
echo "Run the test suites:"
echo "    ./scripts/test-mac.sh"
