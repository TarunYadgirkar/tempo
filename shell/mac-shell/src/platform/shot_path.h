// shot_path.h — where the `screenshot` control verb is allowed to write.
//
// The control plane is reachable by anything running as the user, and a
// screenshot verb that takes a path is a file-write primitive: unconstrained
// it would let a caller drop a PNG over ~/.ssh/config or /etc. So the target
// must sit inside $TMPDIR or $HOME, checked after realpath() on the parent
// directory (a symlinked directory therefore cannot escape), and the writer
// opens the file O_NOFOLLOW so a symlink planted at the target itself is
// refused rather than followed.
//
// Pure C++ so it unit-tests without AppKit (tests/test_shot_path.cpp).

#pragma once

#include <string>

namespace mac_shell {

// $TMPDIR/spatula-shot-<unix ms>.png. Empty when $TMPDIR is unset — the
// control server has already refused to bind in that case, so it is only a
// belt-and-braces return.
std::string default_shot_path();

// Canonicalises `in` (which must be absolute) into `out`. On refusal returns
// false and sets `err` to a one-word reason: not_absolute, bad_basename,
// no_such_dir, outside_allowed_roots.
bool resolve_shot_path(const std::string &in, std::string &out,
                       std::string &err);

}  // namespace mac_shell
