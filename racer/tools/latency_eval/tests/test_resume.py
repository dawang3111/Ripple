import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from latency_eval.cli import main, missing_labels
from latency_eval.core import config, ROOT, trial_plan, write_json
from latency_eval.process_cleanup import owned_processes, cleanup_owned


class ResumeTests(unittest.TestCase):
    def test_ownership_uses_exact_trial_prefix_and_ignores_zombie(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            trial = root / 'trial_0010'
            for pid, path, state in [(101, trial, 'S'), (102, root / 'trial_00100', 'S'),
                                     (103, trial, 'Z'), (104, root / 'other', 'S')]:
                p = root / str(pid)
                p.mkdir()
                (p / 'cmdline').write_bytes(('node\0__log:=' + str(path / 'ros_logs/a.log') + '\0').encode())
                (p / 'stat').write_text(str(pid) + ' (node with spaces) ' + ' '.join([state] + ['0'] * 18 + ['123']))
            self.assertEqual(owned_processes(trial, root), [(101, '123')])

    def test_cleanup_does_not_signal_reused_pid(self):
        with patch('latency_eval.process_cleanup.owned_processes', side_effect=[[(101, 'old')], [(101, 'new')]]), patch('os.kill') as kill:
            self.assertEqual(cleanup_owned('/tmp/trial'), [101])
            kill.assert_not_called()

    def test_transient_resource_shortage_waits_then_continues(self):
        from latency_eval.background import worker
        with tempfile.TemporaryDirectory() as tmp:
            job = Path(tmp)
            run = job / 'runs/run_test'
            trial = run / 'trial_0001'
            trial.mkdir(parents=True)
            write_json(job / 'job.json', dict(trials=1, seed=41, implementation={},
                min_runtime_available_mb=768, min_start_available_mb=1536, reserve_disk_gib=8))
            samples = [dict(disk_free_bytes=30 * 1024**3, mem_available_bytes=available * 1024**2)
                       for available in (1000, 2000)]
            def batch(argv, before_trial, resume_from):
                self.assertTrue(before_trial(run, trial))
                return 0
            with patch('latency_eval.cli.main', side_effect=batch), patch('os.nice'), patch('signal.signal'), \
                 patch('latency_eval.background.resources', side_effect=samples), \
                 patch('latency_eval.background.fingerprint', return_value={}), \
                 patch('latency_eval.background.experiment_processes', return_value=[]), \
                 patch('latency_eval.background.time.sleep') as sleep:
                self.assertEqual(worker(job), 0)
                sleep.assert_called_once_with(30)
            progress = json.loads((job / 'progress.json').read_text())
            self.assertEqual(progress['state'], 'finished')
            self.assertIsNone(progress['conflict'])

    def test_whole_batch_projection_does_not_block_next_round(self):
        from latency_eval.background import worker
        with tempfile.TemporaryDirectory() as tmp:
            job = Path(tmp)
            run = job / 'runs/run_test'
            finished = run / 'trial_0001'
            trial = run / 'trial_0002'
            finished.mkdir(parents=True)
            trial.mkdir()
            write_json(finished / 'result.json', {'status': 'completed'})
            with (finished / 'retained.bin').open('wb') as f:
                f.truncate(20 * 1024**2)
            write_json(job / 'job.json', dict(trials=3000, seed=41, implementation={},
                min_runtime_available_mb=768, min_start_available_mb=1536, reserve_disk_gib=8))
            def batch(argv, before_trial, resume_from):
                self.assertTrue(before_trial(run, trial))
                return 0
            with patch('latency_eval.cli.main', side_effect=batch), patch('os.nice'), patch('signal.signal'), \
                 patch('latency_eval.background.resources', return_value=dict(disk_free_bytes=20 * 1024**3, mem_available_bytes=2000 * 1024**2)), \
                 patch('latency_eval.background.fingerprint', return_value={}), \
                 patch('latency_eval.background.experiment_processes', return_value=[]), \
                 patch('latency_eval.background.time.sleep') as sleep:
                self.assertEqual(worker(job), 0)
                sleep.assert_not_called()
            progress = json.loads((job / 'progress.json').read_text())
            self.assertGreater(progress['projected_remaining_bytes'], 20 * 1024**3)
            self.assertEqual(progress['state'], 'finished')
            self.assertIsNone(progress['conflict'])

    def test_resume_keeps_finished_bytes_and_only_captures_pending_seed(self):
        cfg = config()
        with tempfile.TemporaryDirectory() as tmp:
            old = Path(tmp) / 'old'
            old.mkdir()
            write_json(old / 'config.json', cfg)
            write_json(old / 'manifest.json', {})
            for i in (1, 2):
                d = old / ('trial_%04d' % i)
                d.mkdir()
                plan = dict(trial_plan(i, 41, cfg), control=False,
                            map_path=str(ROOT / 'uav_simulator/map_generator/resource/pillar.pcd'), dataset_id='original')
                write_json(d / 'plan.json', plan)
                if i == 1:
                    write_json(d / 'result.json', dict(plan, status='completed', actual_injected=False,
                        contract_valid=False, labels=missing_labels('test'), features=None, result_path=str(d / 'result.json')))
            saved = (old / 'trial_0001/result.json').read_bytes()
            observed = []
            def capture(d, cfg):
                observed.append(json.loads((d / 'plan.json').read_text()))
                write_json(d / 'capture.json', {'status': 'completed', 'end_ns': 1})
            with patch('latency_eval.cli.preflight_run'), patch('latency_eval.cli.execute_capture', side_effect=capture):
                self.assertEqual(main(['run', '--trials', '2', '--seed', '41', '--output-root', str(Path(tmp) / 'new')], resume_from=old), 0)
            out = next((Path(tmp) / 'new').iterdir())
            self.assertTrue((out / 'trial_0001').is_symlink())
            self.assertEqual((old / 'trial_0001/result.json').read_bytes(), saved)
            self.assertFalse((old / 'trial_0002/result.json').exists())
            self.assertEqual(len(observed), 1)
            self.assertEqual(observed[0], json.loads((old / 'trial_0002/plan.json').read_text()))


if __name__ == '__main__':
    unittest.main()
