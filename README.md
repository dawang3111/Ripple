# Ripple

Ripple studies state-aware communication-delay attacks against multi-UAV systems. This repository contains implementations and experiment tools for **EGO-Planner-v2**, **RACER**, and **Adaptive Swarm**, including message-delay injection, rule evaluation, runtime monitoring, and offline analysis.

## Repository structure

```text
.
├── EGO/                          # EGO-Planner-v2 catkin workspace
│   └── src/
│       ├── planner/plan_manage/   # Planner, launch files, and attack tools
│       └── run_headless.sh        # Simulation batch runner
├── racer/                        # RACER source packages
│   ├── swarm_exploration/         # Exploration and planning
│   ├── uav_simulator/             # Simulation packages
│   └── tools/                    # Delay experiments and analysis
└── adaptive swarm/               # Adaptive Swarm component
    ├── adaptive_swarm/            # Formation planner and evaluation modules
    ├── tool/                     # Experiment launch scripts
    └── test/                     # Evaluation tests
```

Quote paths containing `adaptive swarm`, because the directory name includes a space.

## Environment

- Ubuntu 20.04 with ROS 1 Noetic and catkin for the ROS-based simulations.
- RViz for visualization.
- Python 3.8 or newer for the Python evaluation tools.
- Native planner and simulator dependencies for each target system.

After downloading the repository, open a Bash terminal at its root:

```bash
export RIPPLE_ROOT="$(pwd)"
```

Use separate terminals and workspaces for EGO-Planner-v2 and RACER. Source the corresponding workspace before running ROS commands.

## EGO-Planner-v2

### Code functionality

The EGO component provides multi-UAV navigation, simulation launch files, communication-delay proxies, rule predicates, task-level monitors, and batch execution scripts.

Main entry points:

- [Simulation runner](EGO/src/run_headless.sh)
- [Rule-based latency proxy](EGO/src/planner/plan_manage/scripts/rule_pool_latency_proxy.py)
- [Rule predicates](EGO/src/planner/plan_manage/scripts/ddc10_rules.py)
- [Offline rule discovery](EGO/src/planner/plan_manage/scripts/discover_rules_from_attack_logs.py)
- [Task-level monitoring](EGO/src/planner/plan_manage/scripts/stl_attack_metrics_monitor.py)

### Build

With ROS Noetic and the component dependencies installed:

```bash
source /opt/ros/noetic/setup.bash
cd "$RIPPLE_ROOT/EGO"
catkin_make
source devel/setup.bash
```

### Start the navigation simulation

```bash
roslaunch ego_planner swarm.launch
```

For RViz visualization, open another terminal and run:

```bash
source /opt/ros/noetic/setup.bash
source "$RIPPLE_ROOT/EGO/devel/setup.bash"
roslaunch ego_planner rviz.launch
```

### Run a headless batch

The first five arguments select the number of runs, duration per run, output directory, collision-monitor switch, and legacy communication-interference switch:

```bash
bash src/run_headless.sh RUNS RUN_SECONDS OUTPUT_DIR \
  ENABLE_COLLISION_MONITOR ENABLE_COMM_INTERFERENCE
```

For example, run one 60-second simulation with collision monitoring enabled and legacy communication interference disabled:

```bash
cd "$RIPPLE_ROOT/EGO"
bash src/run_headless.sh 1 60 \
  "$RIPPLE_ROOT/EGO/results/baseline" true false
```

The script creates a timestamped result directory. Additional environment variables configure the rule-based latency proxy, including:

- `ENABLE_RULE_POOL_LATENCY_PROXY`
- `RULE_POOL_ACTIVE_RULE_ID`
- `RULE_POOL_MAX_INJECTIONS_PER_ROUND`
- `RULE_POOL_COOLDOWN_S`
- `RULE_POOL_MONITOR_ONLY`

For a single-rule example using the included `DDC10_2` predicate:

```bash
cd "$RIPPLE_ROOT/EGO"
ENABLE_RULE_POOL_LATENCY_PROXY=true \
RULE_POOL_ACTIVE_RULE_ID=DDC10_2 \
RULE_POOL_MAX_INJECTIONS_PER_ROUND=5 \
bash src/run_headless.sh 1 60 \
  "$RIPPLE_ROOT/EGO/results/rule_delay" true false
```

### Discover rules from collected logs

```bash
cd "$RIPPLE_ROOT/EGO/src"
python3 planner/plan_manage/scripts/discover_rules_from_attack_logs.py --help
```

To analyze an existing experiment directory:

```bash
python3 planner/plan_manage/scripts/discover_rules_from_attack_logs.py \
  --input-root /absolute/path/to/experiment_data \
  --output-dir /absolute/path/to/rule_output \
  --label-mode strict
```

The input directory must contain the captured data expected by the discovery script. Use `--help` to inspect target selection and labeling options.

## RACER

### Code functionality

The RACER component provides decentralized collaborative exploration, simulation environments, source-side delay injection, online rule monitoring, experiment batches, and result analysis.

Main entry points:

- [Native system documentation](racer/README.md)
- [Latency evaluation documentation](racer/tools/latency_eval/README.md)
- [Latency evaluation CLI](racer/tools/racer_latency_eval.py)
- [Rule-guided batch runner](racer/tools/racer_rule_batch.py)
- [Online rule monitor](racer/swarm_exploration/exploration_manager/scripts/ripple_rule_monitor.py)

### Dependencies and build

Follow the dependency instructions in [the RACER README](racer/README.md#quick-start), including NLopt, Armadillo, and LKH.

Create a separate catkin workspace with the package layout expected by the evaluation tools:

```bash
source /opt/ros/noetic/setup.bash
mkdir -p "$RIPPLE_ROOT/racer_ws/src"
ln -s "$RIPPLE_ROOT/racer" "$RIPPLE_ROOT/racer_ws/src/RACER"
cd "$RIPPLE_ROOT/racer_ws"
catkin_make
source devel/setup.bash
```

Create the symbolic link only once.

### Start collaborative exploration

In the first terminal:

```bash
source /opt/ros/noetic/setup.bash
source "$RIPPLE_ROOT/racer_ws/devel/setup.bash"
roslaunch exploration_manager rviz.launch
```

In the second terminal:

```bash
source /opt/ros/noetic/setup.bash
source "$RIPPLE_ROOT/racer_ws/devel/setup.bash"
roslaunch exploration_manager swarm_exploration.launch
```

Use the **2D Nav Goal** tool in RViz to trigger exploration.

### Delay experiments

From the RACER workspace, inspect the command-line interfaces:

```bash
cd "$RIPPLE_ROOT/racer_ws"
source devel/setup.bash
python3 src/RACER/tools/racer_latency_eval.py --help
python3 src/RACER/tools/racer_rule_batch.py --help
```

Generate one experiment plan:

```bash
python3 src/RACER/tools/racer_latency_eval.py plan \
  --trials 1 --seed 20261103 \
  --output-root "$RIPPLE_ROOT/racer_ws/racer_eval_results"
```

Execute a one-trial batch:

```bash
python3 src/RACER/tools/racer_latency_eval.py run \
  --trials 1 --seed 20261103 \
  --output-root "$RIPPLE_ROOT/racer_ws/racer_eval_results"
```

For rule-guided experiments, provide a rule directory containing the accepted rule metrics and associated mining outputs:

```bash
python3 src/RACER/tools/racer_rule_batch.py \
  --rules /absolute/path/to/frozen_rules \
  --per-target 100 --seed 20261111
```

The batch runner prints its job directory, process ID, and log location. Inspect progress with:

```bash
python3 src/RACER/tools/racer_rule_batch.py --status /absolute/path/to/job_directory
```

Reanalyze a captured run:

```bash
python3 src/RACER/tools/racer_latency_eval.py analyze \
  --input /absolute/path/to/captured_run \
  --output-root "$RIPPLE_ROOT/racer_ws/racer_eval_results"
```

See the component's [evaluation documentation](racer/tools/latency_eval/README.md) for input formats, experiment settings, and output fields.

## Adaptive Swarm

### Code functionality

The Adaptive Swarm component provides leader–follower formation planning, local motion corrections, delay transport, rule evaluation, and accuracy/intensity experiment drivers.

Main entry points:

- [Component documentation](adaptive%20swarm/README.md)
- [Evaluation modules](adaptive%20swarm/adaptive_swarm/scripts/layered_planner/latency_eval/)
- [Rule campaign launcher](adaptive%20swarm/tool/run_rule_groups.sh)
- [Accuracy experiment launcher](adaptive%20swarm/tool/run_accuracy_experiment.sh)
- [Intensity experiment launcher](adaptive%20swarm/tool/run_intensity_experiment.sh)

### Install dependencies

Use Python 3.8 or newer:

```bash
cd "$RIPPLE_ROOT/adaptive swarm"
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements-latency-eval.txt
```

The launch scripts use `python3` by default. Set `LATENCY_PYTHON=/absolute/path/to/python` to select another interpreter.

### Inspect experiment options

```bash
python3 -m adaptive_swarm.scripts.layered_planner.latency_eval.accuracy_experiment --help
python3 -m adaptive_swarm.scripts.layered_planner.latency_eval.intensity_experiment --help
bash tool/run_rule_groups.sh --help
```

### Run isolated diagnostics

Accuracy experiment using the position adapter:

```bash
bash tool/run_accuracy_experiment.sh --diagnostic-position-adapter \
  --targets drone_drone --groups all --episodes 2 --seed 20261112
```

Intensity experiment using the position adapter:

```bash
bash tool/run_intensity_experiment.sh --diagnostic-position-adapter \
  --targets drone_drone --groups all --episodes 2 --seed 20261111
```

These entry points run isolated simulation diagnostics without a flight-radio launch. Each experiment creates a new output directory. Use the component README and CLI help to select targets, groups, and campaign parameters.

### Run evaluation tests

```bash
MPLBACKEND=Agg python3 -m unittest discover \
  -s test -p 'test_latency*.py' -v
```

## Results and metrics

The evaluation tools collect experiment records, injection events, rule matches, task-level outcomes, and summaries. Exact filenames and available fields depend on the component and selected experiment mode.

For trial-level attack-effectiveness reporting:

- **Inject:** trials with at least one actual delay injection.
- **Success:** injected trials satisfying the selected strict outcome criterion.
- **ASR_inj (%):** `100 × Success / Inject`.

Rules and outcome thresholds are configured separately. Preserve the chosen rule configuration, label settings, and baseline inputs with each experiment's results.

## License

Refer to the root and component-specific license files. Third-party code retains its original license and attribution.

## Physical UAV Validation

Annotated photographs from the controlled indoor UAV experiments are available in [real/](real/README.md), grouped by the three case studies.
