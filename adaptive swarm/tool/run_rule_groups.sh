#!/bin/sh
# One detached batch: six targets, default 100 trials each. No nohup required.
set -eu
campaign_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$campaign_root"
campaign_python=${LATENCY_PYTHON:-python3}
if ! command -v "$campaign_python" >/dev/null 2>&1; then
    echo 'Set LATENCY_PYTHON to Python 3.8+ with requirements-latency-eval.txt installed.' >&2
    exit 1
fi
exec "$campaign_python" -m adaptive_swarm.scripts.layered_planner.latency_eval.rule_campaign start "$@"
