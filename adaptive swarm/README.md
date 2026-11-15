# Ripple for Adaptive Swarm

Source-only distribution for isolated simulation and offline analysis.
This folder contains code, tests, launch files, dependency declarations, frozen
rule definitions and detector parameters. No experimental logs, outcome tables,
figures, temporary files, or synthetic demonstration results are distributed.

## Structure

- `adaptive_swarm/scripts/layered_planner/latency_eval/`: TX injection,
  rule evaluation, intensity/accuracy experiments, detectors and analysis.
- `adaptive_swarm/launch/swarm.launch`: isolated experiment wrapper.
- `tool/run_accuracy_experiment.sh`, `tool/run_intensity_experiment.sh`,
  `tool/run_rule_groups.sh`: batch entry points.
- `test/test_latency*.py`: targeted regression tests.

The outer `adaptive swarm` folder is the repository component; the inner
`adaptive_swarm` directory is the original ROS package.

## Setup and verification

Run these commands inside this folder with a Python environment containing the
declared dependencies. Use Python 3.8 or newer for the latency evaluation code.

```bash
python3 -m pip install -r requirements-latency-eval.txt
MPLBACKEND=Agg python3 -m unittest discover -s test -p 'test_latency*.py' -v
```

The shell scripts default to `python3` on PATH; override with
`LATENCY_PYTHON=/path/to/python` when needed.

## Isolated diagnostic examples

No flight/radio launch is required. These commands explicitly opt into position
snapshot diagnostics; they are not validation of a timed-trajectory adapter.

```bash
bash tool/run_accuracy_experiment.sh --diagnostic-position-adapter \
  --targets drone_drone --groups all --episodes 2 --seed 20261112
bash tool/run_intensity_experiment.sh --diagnostic-position-adapter \
  --targets drone_drone --groups all --episodes 2 --seed 20261111
```

Accuracy groups G1/G2/G3/G4/G5 wait 0/1/2/3/5 simulation seconds before selecting
a NEW same-source publication. All use the same next-publication release policy.
Intensity groups G1-G4 request 0.2/0.4/0.6/0.8 seconds, truncated by the next
same-source message; G5 releases at that next publication boundary.
Each run creates a new UTC/UUID output directory. Use `--help` for parameters:

```bash
python3 -m adaptive_swarm.scripts.layered_planner.latency_eval.accuracy_experiment --help
python3 -m adaptive_swarm.scripts.layered_planner.latency_eval.intensity_experiment --help
```

## Scope and limitations

The current adapter broadcasts 100Hz position snapshots. Original ROS `Path`
publication stores historical poses, not future timed trajectories. Planning
failure and trajectory drift remain unknown without an actual trajectory stream.
`published_trajectory.py` provides the adapter contract; internal candidate paths
must not be treated as actual broadcasts. Formal trajectory modes fail closed.
The short native task may end before 3/5-second waits or a 30-second deadlock
window complete. Do not convert these unavailable outcomes to zero.

Frozen geometry uses a user-specified 0.1m single-UAV radius. Successful-completion
timeout is twice the frozen baseline mean. Legacy exploratory proxy thresholds
in `rule_groups.json` are distinct from strict physical outcomes. Historical
seed exclusions remain in that file as campaign configuration. The source Excel
is not required for default runs; frozen rule expressions retain cell provenance.

`swarm.launch` was added for the isolated wrapper. Its default mode remains
intensity with injection disabled; set `ripple_experiment:=accuracy` and
`accuracy_group:=G1` through `G5` for accuracy mode. ROS1/catkin/rospy are optional
for this wrapper and were not available for live launch verification. Legacy
`connect123.launch` and `layered_planner.launch` can reference real hardware;
they are not part of the isolated experiment workflow and should not be used
for these experiments.

## Attribution

Based on the Adaptive Swarm code distributed in
https://github.com/Ruoyyy/RSFuzz (source HEAD
`ac28fa0fc2b8a325f5885f2029e3f6564daefb2d` plus local changes).
Original Adaptive Swarm attribution and MIT license are retained in
`adaptive_swarm/CITATION.cff` and `adaptive_swarm/LICENSE`.
