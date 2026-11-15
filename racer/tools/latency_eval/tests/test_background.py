import gzip
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from latency_eval.background import archive_file, archive_finished
from latency_eval.core import read_events, config
from latency_eval.analysis import validation_metrics
from latency_eval.readable import counts


class BackgroundTests(unittest.TestCase):
    def test_archive_preserves_event_bytes_and_can_be_read(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / 'events.jsonl'
            data = b'{"kind":"snapshot","observed_ns":123}\n'
            p.write_bytes(data)
            entry = archive_file(p)
            self.assertFalse(p.exists())
            with gzip.open(Path(tmp) / entry['archive'], 'rb') as f:
                self.assertEqual(f.read(), data)
            self.assertEqual(read_events(Path(tmp) / entry['archive'])[0]['observed_ns'], 123)

    def test_only_finished_trials_are_archived_and_metadata_survives(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name in ('trial_0001', 'trial_0002'):
                (root / name).mkdir()
                (root / name / 'events.jsonl').write_text('test')
            (root / 'trial_0001/result.json').write_text('{}')
            np.save(root / 'trial_0001/scene_points.npy', np.array([[1.,2.,3.]]))
            archive_finished(root)
            self.assertTrue((root / 'trial_0002/events.jsonl').exists())
            self.assertTrue((root / 'trial_0001/result.json').exists())
            with gzip.open(root / 'trial_0001/scene_points.npy.gz', 'rb') as f:
                np.testing.assert_array_equal(np.load(f, allow_pickle=False), [[1.,2.,3.]])
            archive_finished(root)  # idempotent; never overwrites archives

    def test_rotated_logs_are_archived_even_after_existing_marker(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            trial = root / 'trial_0001'
            trial.mkdir()
            (trial / 'result.json').write_text('{}')
            (trial / 'rosout.log').write_bytes(b'first log')
            archive_finished(root)
            before = json.loads((trial / 'archive.json').read_text())['files']
            data = b'rotated evidence\n' * 100
            (trial / 'rosout.log.1').write_bytes(data)
            archive_finished(root)
            self.assertFalse((trial / 'rosout.log.1').exists())
            with gzip.open(trial / 'rosout.log.1.gz', 'rb') as f:
                self.assertEqual(f.read(), data)
            after = json.loads((trial / 'archive.json').read_text())['files']
            self.assertEqual(after[:len(before)], before)
            self.assertEqual(len(after), len(before) + 1)
            archive_finished(root)
            self.assertEqual(json.loads((trial / 'archive.json').read_text())['files'], after)

    def test_archive_does_not_modify_referenced_previous_job(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old = root / 'old'
            old.mkdir()
            (old / 'result.json').write_text('{}')
            (old / 'rosout.log.1').write_bytes(b'keep')
            new = root / 'new'
            new.mkdir()
            (new / 'trial_0001').symlink_to(old, target_is_directory=True)
            archive_finished(new)
            self.assertEqual((old / 'rosout.log.1').read_bytes(), b'keep')
            self.assertFalse((old / 'archive.json').exists())

    def test_nonhit_trigger_rate_is_not_nonhit_rate_among_triggers(self):
        x = np.array([[1.], [1.], [0.], [0.]])
        y = np.array([True, False, False, False])
        m = validation_metrics(x, y, ((0, '>', .5),), config(), 4)
        self.assertEqual(m['target_not_rate'], .5)
        self.assertEqual(m['nonhit_sample_n'], 3)
        self.assertEqual(m['nonhit_trigger_n'], 1)
        self.assertAlmostEqual(m['nonhit_trigger_rate'], 1/3)

    def test_counts_distinguish_pending_skipped_failed_and_invalid(self):
        rows = [{'status':'not_run'}, {'status':'capture_failed', 'actual_injected':'false'},
                {'status':'completed', 'actual_injected':'false', 'contract_valid':'false'}]
        result = counts(rows, 3000)
        self.assertEqual(result['finished_rounds'], 2)
        self.assertEqual(result['failed_rounds'], 1)
        self.assertEqual(result['skipped_rounds'], 1)
        self.assertEqual(result['invalid_rounds'], 2)


if __name__ == '__main__':
    unittest.main()
