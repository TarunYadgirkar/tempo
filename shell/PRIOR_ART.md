# Prior art

This `shell/` directory is carried over from our team's prior private project (Spatula / Vantage, Project Ithaca) and is not hackathon work. It is the compositor base Tempo builds on.

Copied components: `mac-shell/` (Metal renderer, scene core, control socket), `bridge-receiver/` (iPhone UDP transport, hand reconstruction), `gesture-engine/`, the vendored wxrd control-plane files under `vendor/wxrd/src/` (control_protocol, control_conn, fist_rotation), the reused pure-C `spatial-shell/keyboard/decoder` and `spatial-shell/radial-menu` modules, `tests/` (replay-session harness and recorded iPhone sessions), and `scripts/` (build/run/install/test).

Everything outside `shell/` is new work written during the hackathon.

Any file listed in `shell/HACK_CHANGES.md` is also new during the hackathon.

The bundle identity `com.spatialos.spatula` is kept unchanged because macOS TCC grants are keyed to it.
