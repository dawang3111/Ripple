#!/usr/bin/env bash
# Explicit large-batch launcher; never called automatically by pilot or imports.
set -euo pipefail
RIPPLE_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [[ $# -lt 1 ]]; then
  printf 'Usage: %s BASELINE_JSON [--targets TARGET ...] [--per-cell 50] [--seed 20261113]\n' "$0" >&2
  exit 2
fi
RIPPLE_BASELINE="$(readlink -f "$1")"; shift
[[ -f "$RIPPLE_BASELINE" ]] || { printf 'Missing baseline: %s\n' "$RIPPLE_BASELINE" >&2; exit 2; }
source /opt/ros/noetic/setup.bash
source "$RIPPLE_WS/devel/setup.bash"
mkdir -p "$RIPPLE_WS/racer_eval_results"
RIPPLE_DRIVER="$(mktemp -d "$RIPPLE_WS/racer_eval_results/intensity_driver_XXXXXXXX")"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
python3 - "$RIPPLE_WS" "$RIPPLE_BASELINE" "$RIPPLE_DRIVER" "$@" <<'PY'
import pathlib
import subprocess
import sys

workspace, baseline, driver = sys.argv[1:4]
directory = pathlib.Path(driver)
with (directory / 'background.log').open('ab') as log:
    process = subprocess.Popen(
        [sys.executable, '-u', workspace + '/src/RACER/tools/racer_intensity_eval.py',
         'run', '--baseline', baseline, '--output-root', driver] + sys.argv[4:],
        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
(directory / 'background.pid').write_text(str(process.pid) + '\n')
print('PID:', process.pid)
print('Output root:', driver)
print('Log:', directory / 'background.log')
PY
