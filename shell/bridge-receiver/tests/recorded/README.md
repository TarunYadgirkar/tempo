Committed .bin session files for offline testing. Record new sessions with scripts/record-session.sh

synthetic-*.bin are generated (deterministic) by scripts/gen_synthetic_session.py:
  python3 scripts/gen_synthetic_session.py --bin bridge-receiver/tests/recorded/synthetic-pinch.bin --seed 2 --duration 6 --gestures pinch_select
  python3 scripts/gen_synthetic_session.py --bin bridge-receiver/tests/recorded/synthetic-idle.bin --seed 3 --duration 4 --gestures none
