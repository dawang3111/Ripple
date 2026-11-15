#!/bin/sh
set -eu
accuracy_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$accuracy_root"
accuracy_python=${LATENCY_PYTHON:-python3}
export MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
exec "$accuracy_python" -u -m adaptive_swarm.scripts.layered_planner.latency_eval.accuracy_experiment batch "$@"
