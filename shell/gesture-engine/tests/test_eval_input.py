import argparse
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest


parser = argparse.ArgumentParser()
parser.add_argument('--eval-recording', required=True, type=Path)
args, remaining = parser.parse_known_args()
EVAL_BIN = args.eval_recording.resolve()
SCRIPT = Path(__file__).resolve().parents[2] / 'scripts' / 'eval_dataset.py'


class EvalInputTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.mirror = Path(self.tmp.name)
        self.clip = self.mirror / 'clip'
        self.clip.mkdir()
        (self.clip / 'meta.json').write_text('{}')
        (self.clip / 'labels.json').write_text(json.dumps({
            'session_id': 'clip', 'actions': [],
            'clip_window': {'start_ns': 0, 'end_ns': 1_000_000_000},
        }))
        payload = struct.pack('<BQBB', 5, 1_000_000_000, 0, 21)
        payload += struct.pack('<105f', *([0.0] * 105))
        self.valid = struct.pack('<I', len(payload)) + payload
        self.stream = self.clip / 'stream.bin'
        self.stream.write_bytes(self.valid)

    def evaluate(self):
        return subprocess.run(
            [sys.executable, str(SCRIPT), '--mirror', str(self.mirror),
             '--eval-recording', str(EVAL_BIN)],
            capture_output=True, text=True, timeout=10,
        )

    def test_valid_unkeyed_recording_remains_unscored(self):
        result = self.evaluate()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('SKIP', result.stdout)

    def test_empty_stream_is_a_failure_even_without_keyframes(self):
        self.stream.write_bytes(b'')
        result = self.evaluate()
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn('no hand packets', result.stdout)

    def test_missing_metadata_does_not_disappear_from_evaluation(self):
        (self.clip / 'meta.json').unlink()
        result = self.evaluate()
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn('meta.json', result.stdout + result.stderr)

    def test_missing_labels_cannot_hide_beside_a_valid_clip(self):
        damaged = self.mirror / 'damaged'
        damaged.mkdir()
        (damaged / 'stream.bin').write_bytes(self.valid)
        (damaged / 'meta.json').write_text('{}')
        result = self.evaluate()
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn('labels.json', result.stdout + result.stderr)

    def test_truncated_recording_never_scores_a_valid_prefix(self):
        for suffix in (b'\x01', b'\x01\x00', b'\x01\x00\x00',
                       struct.pack('<I', 12) + b'\x05',
                       struct.pack('<I', 16 * 1024 * 1024 + 1)):
            with self.subTest(suffix=suffix):
                self.stream.write_bytes(self.valid + suffix)
                result = self.evaluate()
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assertIn('truncated', result.stdout + result.stderr)

    def test_no_stream_candidates_still_signals_skip(self):
        self.stream.unlink()
        result = self.evaluate()
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0], *remaining])
