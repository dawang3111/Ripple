#!/bin/sh
set -eu
intensity_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$intensity_root"
intensity_python=${LATENCY_PYTHON:-python3}
export MPLBACKEND=Agg OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
exec "$intensity_python" -u -m adaptive_swarm.scripts.layered_planner.latency_eval.intensity_experiment batch "$@"
