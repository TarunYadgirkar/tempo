#!/usr/bin/env bash
# test-mac.sh — run every macOS ctest suite scripts/build-mac.sh produces and
# print a summary table:
#   ctest: gesture-engine, bridge-receiver, mac-shell
# Skipped tests (e.g. data-driven eval_dataset, or screenshot_e2e with no
# display attached) are OK; any failure exits nonzero.
# Build first: scripts/build-mac.sh.
#
# Usage: scripts/test-mac.sh [-h|--help]
# Usage: scripts/test-mac.sh [-h|--help]
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

# kind:name — build dir is build/<name>
SUITES=(
    "ctest:gesture-engine"
    "ctest:bridge-receiver"
    "ctest:mac-shell"
)

SUMMARY_NAMES=()
SUMMARY_PASSED=()
SUMMARY_FAILED=()
SUMMARY_SKIPPED=()
SUMMARY_NOTE=()
ANY_FAILED=0

# Per-suite results, set by the run_* helpers.
PASSED=0 FAILED=0 SKIPPED=0 SUITE_RC=0

count_or_zero() { grep -c "$1" <<<"$2" || true; }

run_ctest() {
    local build="$1" log
    log="$(mktemp)"
    SUITE_RC=0
    ctest --test-dir "$build" --output-on-failure 2>&1 | tee "$log" || SUITE_RC=1
    local results
    results="$(grep -E '^ *[0-9]+/[0-9]+ Test +#' "$log" || true)"
    PASSED="$(count_or_zero 'Passed' "$results")"
    SKIPPED="$(count_or_zero 'Skipped' "$results")"
    FAILED="$(grep -cE '\*\*\*(Failed|Timeout|Exception|Not Run)' <<<"$results" || true)"
    rm -f "$log"
}

echo -e "${C_BOLD}==> [test-mac] Spatula — macOS native core tests${C_RESET}"
echo "    Project root: $PROJECT_ROOT"
echo "    $(date)"

for entry in "${SUITES[@]}"; do
    KIND="${entry%%:*}"
    SUITE="${entry#*:}"
    BUILD="$PROJECT_ROOT/build/$SUITE"
    NOTE=""

    echo ""
    echo -e "${C_BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}"
    echo -e "${C_BOLD}  TEST: $SUITE${C_RESET}"
    echo -e "${C_BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}"

    case "$KIND" in
        ctest)
            if [[ ! -f "$BUILD/CTestTestfile.cmake" ]]; then
                echo -e "${C_RED}ERROR: $SUITE not built. Run ./scripts/build-mac.sh first.${C_RESET}"
                exit 1
            fi
            run_ctest "$BUILD"
            ;;
    esac

    if [[ "$SUITE_RC" -ne 0 && "$FAILED" -eq 0 ]]; then
        FAILED=1
    fi
    if [[ "$FAILED" -gt 0 ]]; then
        ANY_FAILED=1
    fi

    SUMMARY_NAMES+=("$SUITE")
    SUMMARY_PASSED+=("$PASSED")
    SUMMARY_FAILED+=("$FAILED")
    SUMMARY_SKIPPED+=("$SKIPPED")
    SUMMARY_NOTE+=("")
done

echo ""
echo -e "${C_BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}"
echo -e "${C_BOLD}  TEST SUMMARY${C_RESET}"
echo -e "${C_BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}"
printf "  %-18s %8s %8s %8s   %s\n" "Suite" "Passed" "Failed" "Skipped" "Result"
for i in "${!SUMMARY_NAMES[@]}"; do
    if [[ -n "${SUMMARY_NOTE[$i]}" ]]; then
        RESULT="${C_YELLOW}SKIPPED${C_RESET} — ${SUMMARY_NOTE[$i]}"
    elif [[ "${SUMMARY_FAILED[$i]}" -gt 0 ]]; then
        RESULT="${C_RED}FAIL${C_RESET}"
    else
        RESULT="${C_GREEN}PASS${C_RESET}"
    fi
    printf "  %-18s %8s %8s %8s   " \
        "${SUMMARY_NAMES[$i]}" "${SUMMARY_PASSED[$i]}" \
        "${SUMMARY_FAILED[$i]}" "${SUMMARY_SKIPPED[$i]}"
    echo -e "$RESULT"
done
echo ""

if [[ "$ANY_FAILED" -ne 0 ]]; then
    echo -e "${C_RED}  TESTS FAILED. See output above.${C_RESET}"
    exit 1
fi

echo -e "${C_GREEN}  ALL SUITES GREEN (skips OK)${C_RESET}"
