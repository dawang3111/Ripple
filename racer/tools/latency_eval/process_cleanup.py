"""Signal only processes whose explicit ROS log argument belongs to this trial."""
import os
import signal
import time
from pathlib import Path


def owned_processes(trial, proc_root=Path('/proc')):
    prefix = str(Path(trial).resolve() / 'ros_logs') + '/'
    owned = []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            args = (entry / 'cmdline').read_bytes().decode(errors='replace').split('\0')
            if not any(arg.startswith('__log:=' + prefix) for arg in args):
                continue
            stat = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if stat[0] == 'Z':
                continue
            owned.append((int(entry.name), stat[19]))  # Linux starttime, survives PID reuse checks
        except (OSError, IndexError):
            continue
    return owned


def cleanup_owned(trial):
    """Children may have their own sessions; never signal an inferred process group."""
    initial = owned_processes(trial)
    for sig, seconds in ((signal.SIGINT, 2), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
        remaining = set(owned_processes(trial)) & set(initial)
        if not remaining:
            return [pid for pid, _ in initial]
        for pid, _ in remaining:
            # Recheck both explicit ownership and starttime immediately before signalling.
            if (pid, _) in owned_processes(trial):
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and set(owned_processes(trial)) & set(initial):
            time.sleep(.1)
    if owned_processes(trial):
        raise RuntimeError('owned_trial_processes_survived_cleanup:' + str(trial))
    return [pid for pid, _ in initial]
