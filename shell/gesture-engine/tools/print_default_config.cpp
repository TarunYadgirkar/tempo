// print_default_config.cpp — Emit the canonical gesture-engine TOML.
//
// Writes the compiled-default gesture configuration to stdout in the same
// shape as sample-config/gestures.toml.  Used as the source of truth: when
// defaults change in src/engine.cpp, run this tool and check in the new
// sample.  CI verifies the two stay in sync.

#include "gesture_engine.h"

#include <cstdio>

int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    ge_print_default_config(stdout);
    return 0;
}
