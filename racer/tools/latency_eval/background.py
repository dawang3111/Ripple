"""Detached, serial batch with resource checks; no management of unrelated jobs."""
import argparse
import gzip
import hashlib
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

from .core import ROOT, config, digest, unique_output, write_json
from .readable import counts, csv_rows
from .process_cleanup import cleanup_owned


def atomic_json(path, value):
    temporary = path.with_name(path.name + '.tmp')
    write_json(temporary, value)
    temporary.replace(path)


def resources(path):
    info = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        key, rest = line.split(':', 1)
        info[key] = int(rest.split()[0]) * 1024
    return {'disk_free_bytes': shutil.disk_usage(path).free, 'mem_available_bytes': info['MemAvailable'],
            'cpu_count': os.cpu_count(), 'load_average': list(os.getloadavg())}


def experiment_processes():
    matches = []
    names = {'roscore', 'rosmaster', 'roslaunch', 'exploration_nod', 'pcl_render_node', 'mtsp_node'}
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name) == os.getpid():
            continue
        try:
            name = (p / 'comm').read_text().strip()
            cmd = (p / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            worker = 'python' in name.lower() and ('latency_eval.background --worker' in cmd or 'latency_eval.rule_batch --worker' in cmd or 'racer_latency_eval.py run' in cmd)
            if name in names or worker:
                matches.append({'pid': int(p.name), 'name': name, 'command': cmd[:600]})
        except (OSError, ValueError):
            continue
    return matches


def archive_file(path):
    """Byte-exact gzip of a finished, exclusively owned trial; verify before unlink."""
    source_hash = digest(path)
    size = path.stat().st_size
    dest = path.with_name(path.name + '.gz')
    if not dest.exists():
        with path.open('rb') as src, dest.open('xb') as raw:
            with gzip.GzipFile(filename='', mode='wb', fileobj=raw, mtime=0) as out:
                shutil.copyfileobj(src, out)
    h = hashlib.sha256()
    with gzip.open(dest, 'rb') as f:
        for block in iter(lambda: f.read(1048576), b''):
            h.update(block)
    if h.hexdigest() != source_hash:
        raise ValueError('archive verification failed: ' + str(path))
    path.unlink()
    return {'original': path.name, 'raw_bytes': size, 'raw_sha256': source_hash,
            'archive': dest.name, 'archive_bytes': dest.stat().st_size, 'archive_sha256': digest(dest)}


def archive_finished(run):
    for result in run.glob('trial_*/result.json'):
        trial = result.parent
        marker = trial / 'archive.json'
        if trial.is_symlink():
            continue  # prior jobs are immutable references
        previous = json.loads(marker.read_text()) if marker.exists() else {}
        entries = list(previous.get('files', []))
        changed = not marker.exists()
        for p in sorted(trial.rglob('*')):
            rotated_log = re.search(r'\.log\.\d+$', p.name) is not None
            if p.is_symlink() or not p.is_file() or (p.suffix not in ('.log', '.jsonl', '.npy', '.atsp', '.tsp', '.par', '.tour') and not rotated_log):
                continue
            entry = archive_file(p)
            entry['relative_path'] = str(p.relative_to(trial))
            entries.append(entry)
            changed = True
        if changed:
            atomic_json(marker, {'format': 'verified-byte-exact-gzip', 'files': entries})


def fingerprint():
    paths = list((ROOT / 'tools/latency_eval').glob('*.py'))
    package = ROOT / 'swarm_exploration/exploration_manager'
    paths += [package / p for p in ('src/fast_exploration_fsm.cpp', 'src/evaluation_adapter.cpp',
              'include/exploration_manager/tx_delay_gate.h', 'launch/swarm_exploration.launch',
              'launch/single_drone_exploration.xml', 'launch/single_drone_planner.xml')]
    paths += [ROOT.parents[1] / 'devel/lib/exploration_manager/exploration_node', ROOT / 'tools/latency_eval/config.json']
    return {str(p): digest(p) for p in paths}


def worker(job):
    from .cli import main as batch_main
    plan = json.loads((job / 'job.json').read_text())
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    os.nice(10)
    os.environ['RACER_EVAL_MIN_AVAILABLE_MB'] = str(plan['min_runtime_available_mb'])
    progress = {'state': 'starting', 'pid': os.getpid(), 'planned_rounds': plan['trials'], 'seed': plan['seed'], 'job_dir': str(job)}

    def publish(state, run=None, **extra):
        progress.update(state=state, updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), **extra)
        if run:
            progress.update(counts(csv_rows(run / 'trials.csv'), plan['trials']))
            progress['run_dir'] = str(run)
        atomic_json(job / 'progress.json', progress)

    def check_before(run, trial):
        archive_finished(run)
        current = resources(job)
        reason = None
        if fingerprint() != plan['implementation']:
            reason = 'source_or_binary_changed_during_batch'
        other = experiment_processes()
        if other:
            reason = 'other_ros_or_experiment_processes:' + json.dumps(other, ensure_ascii=False)
        if current['mem_available_bytes'] < plan['min_start_available_mb'] * 1024**2:
            reason = 'insufficient_available_memory_before_next_round'
        reserve = plan['reserve_disk_gib'] * 1024**3
        if current['disk_free_bytes'] < reserve:
            reason = 'disk_reserve_reached'
        sizes = []
        for p in run.glob('trial_*/result.json'):
            result = json.loads(p.read_text())
            if result['status'] in ('completed', 'task_budget_exhausted'):
                sizes.append(sum(f.stat().st_size for f in p.parent.rglob('*') if f.is_file() and not f.is_symlink()))
        projected = None
        next_round_budget = 512 * 1024**2
        for marker in run.glob('trial_*/archive.json'):
            archived = json.loads(marker.read_text())
            raw_size = sum(entry['raw_bytes'] for entry in archived['files'])
            next_round_budget = max(next_round_budget, raw_size * 2)
        if sizes:
            remaining = plan['trials'] - len(list(run.glob('trial_*/result.json')))
            projected = remaining * max(max(sizes), sum(sizes) / len(sizes) * 1.5)
        if next_round_budget > current['disk_free_bytes'] - reserve:
            reason = 'insufficient_disk_for_next_round_and_reserve'
        if fingerprint() != plan['implementation']:
            reason = 'source_or_binary_changed_during_batch'
        publish('blocked' if reason else 'running', run, resources=current, next_trial=trial.name,
                projected_remaining_bytes=projected, next_round_disk_budget_bytes=next_round_budget,
                projection_policy='informational; gate on actual free space and next-round budget', conflict=reason)
        if reason:
            print('RESOURCE CONFLICT: ' + reason, flush=True)
        return reason is None

    def before(run, trial):
        # Sweep only finished owned trials, before archival and conflict detection.
        cleaned = []
        for result in sorted(run.glob('trial_*/result.json'))[-1:]:
            cleaned += cleanup_owned(result.parent)
        if cleaned:
            print('Cleaned owned residual PIDs: ' + str(cleaned), flush=True)
        while not check_before(run, trial):
            if progress.get('conflict') == 'source_or_binary_changed_during_batch':
                return False
            publish('waiting_resources', run)
            time.sleep(30)
        return True

    publish('starting')
    code = 1
    try:
        code = batch_main(['run', '--trials', str(plan['trials']), '--seed', str(plan['seed']),
                           '--config', str(job / 'config.json'), '--output-root', str(job / 'runs')], before_trial=before,
                          resume_from=plan.get('resume_run'))
        runs = list((job / 'runs').glob('run_*'))
        if runs:
            run = runs[0]
            archive_finished(run)
            publish('blocked' if progress.get('conflict') else 'finished' if code == 0 else 'failed_or_interrupted', run, exit_code=code)
    except BaseException as exc:
        publish('failed', error=repr(exc), exit_code=code)
        raise
    return code


def main(argv=None):
    p = argparse.ArgumentParser(description='Detach one isolated serial RACER injection batch; never stop other experiments')
    p.add_argument('--trials', type=int, default=3000)
    p.add_argument('--seed', type=int, default=20261103)
    p.add_argument('--output-root', default=str(ROOT.parents[1] / 'racer_eval_results'))
    p.add_argument('--status', metavar='JOB_DIR')
    p.add_argument('--resume', metavar='JOB_DIR', help='continue recorded plans into a fresh job, referencing finished trials')
    p.add_argument('--worker', metavar='JOB_DIR', help=argparse.SUPPRESS)
    args = p.parse_args(argv)
    if args.worker:
        return worker(Path(args.worker).resolve())
    if args.status:
        job = Path(args.status).resolve()
        print((job / 'progress.json').read_text() if (job / 'progress.json').exists() else (job / 'job.json').read_text())
        return 0
    if args.trials < 1:
        p.error('trials must be positive')
    from .cli import preflight_run
    preflight_run(args.output_root, p)
    # No allocation/launch when a known conflict already exists.
    resumed = None
    if args.resume:
        previous_job = Path(args.resume).resolve()
        resumed = json.loads((previous_job / 'job.json').read_text())
        # An active worker must never share plans or own-process cleanup with a resume.
        if any('python' in item['name'].lower() for item in experiment_processes()):
            p.error('an experiment worker is active; resume refused')
        runs = list((previous_job / 'runs').glob('run_*'))
        if len(runs) != 1:
            p.error('resume needs exactly one recorded run')
        previous_run = runs[0]
        current_hashes = fingerprint()
        lifecycle = {'background.py', 'cli.py', 'ros_capture.py', 'process_cleanup.py'}
        for filename, checksum in resumed['implementation'].items():
            if Path(filename).name not in lifecycle and current_hashes.get(filename) != checksum:
                p.error('algorithm/analysis/config changed since original job: ' + filename)
        args.trials, args.seed = resumed['trials'], resumed['seed']
        for result in previous_run.glob('trial_*/result.json'):
            cleanup_owned(result.parent)
    other = experiment_processes()
    if other:
        p.error('existing ROS/experiment processes; not starting: ' + json.dumps(other, ensure_ascii=False))
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
    ancestor = Path(args.output_root).resolve()
    while not ancestor.exists():
        ancestor = ancestor.parent
    current = resources(ancestor)
    if current['mem_available_bytes'] < 1536 * 1024**2 or current['disk_free_bytes'] < 8 * 1024**3:
        p.error('resource conflict; not starting: ' + json.dumps(current))
    job = unique_output(args.output_root, 'background')
    write_json(job / 'config.json', json.loads((previous_job / 'config.json').read_text()) if resumed else config())
    plan = {'trials': args.trials, 'seed': args.seed, 'job_dir': str(job), 'resources_before': current,
            'implementation': fingerprint(), 'reserve_disk_gib': 8, 'min_start_available_mb': 1536,
            'min_runtime_available_mb': 768, 'nice': 10, 'archive': 'verified-byte-exact-gzip',
            'environment': {'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'},
            'automatic_controls': False, 'background_command': sys.argv,
            'disk_policy': 'reserve 8 GiB plus max(512 MiB, 2x observed raw trial size); full-batch projection informational'}
    if resumed:
        plan.update(resume_job=str(previous_job), resume_run=str(previous_run),
                    original_implementation=resumed['implementation'],
                    resume_policy='new run; immutable finished trial links; original plans and seeds; no retry of finished trials')
    write_json(job / 'job.json', plan)
    env = dict(os.environ, **plan['environment'])
    env['PYTHONPATH'] = str(ROOT / 'tools') + os.pathsep + env.get('PYTHONPATH', '')
    with (job / 'background.log').open('x') as log:
        process = subprocess.Popen([sys.executable, '-u', '-m', 'latency_eval.background', '--worker', str(job)],
                                   stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    (job / 'background.pid').write_text(str(process.pid) + '\n')
    print(json.dumps({'pid': process.pid, 'job_dir': str(job), 'seed': args.seed, 'planned_rounds': args.trials}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
