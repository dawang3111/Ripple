# RACER isolated random TX evaluation

This is a separate, opt-in experiment mode. Existing `roslaunch` commands keep
their original defaults. No experiment starts on import or when running `plan`
or `analyze`. Python 3 and NumPy are required; SciPy/scikit-learn are not needed.
The KMeans implementation is deterministic k-means++/Lloyd with ten initializations.

From the workspace root:

```bash
source devel/setup.bash
cmake --build build --target exploration_node -j2

# Inspect 3000 reproducible plans without starting ROS or experiments:
python3 src/RACER/tools/racer_latency_eval.py plan --trials 3000 --seed 20261103 \
  --output-root /home/ubuntu/racer_ws/racer_eval_results

# Explicitly run exactly 3000 attempted rounds; no automatic control rounds:
python3 src/RACER/tools/racer_latency_eval.py run --trials 3000 --seed 20261103 \
  --output-root /home/ubuntu/racer_ws/racer_eval_results

# Reanalyze captured data without starting ROS; allocate a new unique directory:
python3 src/RACER/tools/racer_latency_eval.py analyze \
  --input /absolute/path/to/run_TIMESTAMP_UUID \
  --output-root /home/ubuntu/racer_ws/racer_eval_results

# Tests (including compiling/executing the actual C++ TX gate):
PYTHONPATH=src/RACER/tools python3 -m unittest discover \
  -s src/RACER/tools/latency_eval/tests -v
```

`--trials` defaults to 3000 for `plan` and `run`. All commands allocate a directory
using an atomic `mkdir` and a UUID. An existing directory is never reused.
Run/trial outputs, ROS logs, parameter snapshots and solver intermediates live
under that directory. Ctrl-C stops owned subprocesses and exports partial results;
three consecutive capture failures stop the batch and mark remaining plans not run.

The run adapter supports the repository's four active drones (IDs 1–4, configured
swarm size 5) and `pillar.pcd` by default. `--map` selects another ASCII PCD map;
it does not alter bounding-box parameters. A generated launch enables telemetry,
sets a fresh loopback-only ROS master and wall clock, and overrides solver output
paths. It never loads a realworld launch or radio bridge. Preflight rejects a
different checkout resolved by `rospack` or an out-of-date evaluation binary.

## Source TX contract

The implemented adapter is for **DroneState only**, at the actual source publish
site in `FastExplorationFSM::droneStateTimerCallback`. It does not modify payloads
or headers. Other topics remain unchanged. Cross-topic ordering and receiver
arrival/apply-time guarantees are not claimed. `tx_delay_gate.h` is the extension
interface for another source publisher; that adapter must define its own actual
cycle, protocol deadline, send/release evidence and duplicate identity. An external
topic relay is not an equivalent implementation of this source-side contract.

Each trial derives its own 64-bit seed from SHA-256(master seed, trial index).
It selects one sender, a uniform offset in the configured task-time window, and a
uniform delay fraction. It waits for all four `WAIT_TRIGGER` states and adapter/map
readiness, then sends the task trigger. The sender starts its clock in the accepted
trigger callback. The first source opportunity at/after that offset is attempted
only once. It must be PLAN_TRAJ, PUB_TRAJ or EXEC_TRAJ. Inactive state, missing period
history, no budget, missing task trigger and termination before release are explicit
skips; there is no retry or selection based on outcomes.

The delay is the median of up to eight positive measured source periods times
the sampled fraction. It is limited by the remaining 0.2 s RACER state-freshness
budget (after accounting for message age) and remaining simulation budget. There
is **no fixed 250 ms injection cap**. The separately configurable observation-gap
tolerance concerns valid negative labels, not the delay scheduler.

The source's single callback queue releases the held message on a 1 ms WallTimer.
If the next regular message arrives early, its callback releases the held message
before publishing the normal message. It neither sleeps nor delays that successor
on purpose. Each attempt gets a sidecar sequence, so identical messages remain
distinct. Scheduled/skip records have null publish timestamps. Release records
have measured publish begin/end intervals, generated time, planned/actual delay,
budget, measured period and early-release reason. The intervention cutoff is the
held message's source generation time, not its eventual release time.

ROS callbacks can block. Hard real-time deadlines cannot be guaranteed. Offline
validation requires one matching schedule/release, a positive measured delay,
strict source sequence order, release completion before the next normal publish
begins, and compliance through publish completion with the protocol/run budget.
No successor evidence or a deadline overrun makes `contract_valid=false`. An
already delayed/released message still has `actual_injected=true`; it is excluded
from valid association samples. Missing release evidence never counts as injected.
These timestamps describe ROS publish calls, not physical transmission or RX.

The adapter requires Linux same-host `steady_clock`/`CLOCK_MONOTONIC`, ROS wall time
and a single-threaded spinner. It disables itself under `/use_sim_time=true`.
Changing to a multi-threaded spinner or another clock requires a new synchronized
adapter. Seeds reproduce schedules; ROS scheduling, optimization and existing
sensor randomness do not promise bitwise-reproducible trajectories.

## Labels and observability

Six booleans are independent. Each also has a validity flag, missing reason,
event count and up to twenty evidence examples (time, IDs, FSM, planner return
code and position measurement times). A false invalid label is unknown, not a
negative. Labels cover the post-intervention observation interval. Pre-intervention
events remain in the raw log but cannot become post-intervention outcomes.

| Target | Operational definition |
| --- | --- |
| inter_drone_collision | Sampled 3-D overlap of configured spherical body proxies using time-aligned simulation positions. Not the planner's XY future-conflict log. |
| obstacle_collision | Sphere proxy overlaps occupied voxels from the **captured published global scene**. This includes geometry added by map_pub, such as the ground. |
| deadlock | In an active FSM, continuous low speed and low displacement with no decrease in active-grid count for the configured window. INIT, WAIT_TRIGGER, IDLE and FINISH are excluded. |
| delay_timeout | Receiver's active-neighbor state age exceeds RACER's 0.2 s budget; planning call exceeds configured 0.1 s budget; or configured task budget expires. Evidence subtypes stay separate. |
| planning_failure | EXPL_RESULT::FAIL. NO_GRID, no frontier and a latency warning are not automatically planning failures. |
| drift | Receiver-cache position error relative to current simulation position exceeds configured distance continuously for the configured duration. This is not localization drift or independent physical tracking error. |

`config.json` contains every threshold, its source, sampling/alignment tolerances,
history/horizon lengths and analysis settings. Body radius is an explicit evaluation
proxy initialized from map inflation, not measured airframe geometry. Deadlock and
drift thresholds are configurable evaluation definitions. No threshold adapts to
observed hit rates. Light simulation copies position commands and has no physical
collision response. Discrete, approximately aligned samples can miss brief events;
gaps/stale position measurements invalidate negative labels. Captured prediction
warnings remain separate from collision labels.

## Features and outputs

Snapshots retain original odom measurement timestamps, query timestamps and observer
receipt timestamps. Offline extraction admits only observations received by the
intervention cutoff; a message generated earlier but delivered later is excluded.
Future features evaluate only the most recently published **pre-intervention**
B-spline. They require the complete three-second horizon; no extrapolation or
post-intervention replacement plan is used. Local membership is the source and
drones within the configured radius; missing peer snapshots invalidate local maxima.
The victim is the closest currently observable neighbor, not an outcome-selected ID.

Implemented scalar features: source/local velocity and acceleration magnitudes;
source/local five-second displacement and wrapped heading change; local current
and sampled future-three-second minimum pair distance; five-second minimum-distance
change; source current/future obstacle clearance; queried planner obstacle distance;
five-second local replans, safety replans, new trajectories and replans after an
executing trajectory; source/victim/local maximum trajectory deviation; heading
angles; source active-grid/frontier counts and elapsed task time; source truth versus
receiver-cached broadcast position error. Units/provenance/missing reasons accompany
every field. There is no invented emergency-stop count, coverage percentage or map
update timestamp. Planner map-distance query time is known; its map update time is not.

Every command writes:

- `manifest.json`, `config.json`: version/dirty status, implementation/input hashes,
  parameters, seeds, thresholds, sources, clock and contract limitations.
- `trials.csv`: planned/run/skip state, actual injection, contract validation, six
  hit/valid labels, evidence JSON, planned/actual timing and per-trial result path.
- `trial_features.csv`: wide values, units, missing flags/reasons, observation/source
  time ranges and cohort/cutoff metadata. Planned/not-run rows have no feature vector.
- `cluster_assignments.csv`, `cluster_summary.csv`, `preprocessing.json`: per-target
  train/holdout membership, separate hit/miss cluster labels, original-unit centers,
  raw observed-field statistics and training-only preprocessing parameters.
- `rules_per_target.csv`, `summary_per_target.csv`, `analysis.md`.

`analyze` reads captured trial/run directories containing `plan.json`, `capture.json`,
`events.jsonl` and optionally `scene_points.npy`. It recomputes results and does not
trust cached feature/label JSON. Old rosout-only logs have no source release/cutoff
or observation provenance; they cannot support this injection/feature contract and
must not be relabeled as actual injected rounds. A new input adapter must emit the
documented structured events rather than inventing missing fields.

## Analysis protocol

Each target uses actual-injected, contract-valid, target-observable samples only.
There are no synthetic successes or automatic replacement rounds. A stable,
stratified train/holdout split happens first. Train-only medians fill missing values;
train-only means/stds scale features; all-missing training fields are excluded.
Hit and miss training groups each fit KMeans(k=2). Small/empty/identical-vector groups
are skipped explicitly. Raw observed counts, missing counts, mean/std and quantiles
are reported alongside imputed centers. Cluster numbers never mean risk ranks.

Rules use train quantiles and train cluster centers, with a train-only beam search
limited to one, two or three conditions. Ranking/selection never uses holdout metrics.
Holdout reports support, hit/non-hit rates among triggered samples, nontriggered hit
rate, percentage-point difference, lift and confidence intervals. Rule conditions
require observed values; missing-rule-field rows are excluded and counted. Rate CIs
are 95% Wilson; difference intervals use conservative Wilson endpoint differences;
lift uses fixed-seed percentile bootstrap. Undefined/small-sample metrics are blank
or marked low confidence. Multiple exploratory candidates require independent
replication; positive/negative clustering alone does not establish a useful rule.

If existing explicit controls are available, pass their run directory as an additional
`--input`. `run --control --trials N` explicitly runs N controls instead of injections;
it never doubles an injection batch. Control features use a scheduled pseudo-cutoff
after task start, without pretending an injection occurred. Rules report control
trigger rate and Wilson CI separately. Without controls, outputs are **within-injection
hit/miss associations**, not effects relative to normal operation. Even descriptive
controls alone do not establish causation.

## Detached execution with resource protection

```bash
source devel/setup.bash
python3 src/RACER/tools/racer_latency_background.py --trials 3000 --seed 20261103 \
  --output-root /home/ubuntu/racer_ws/racer_eval_results
python3 src/RACER/tools/racer_latency_background.py --status /absolute/path/to/background_TIMESTAMP_UUID
```

The launcher detaches itself, reports its PID and allocates a new unique job directory.
It refuses to start if another ROS/experiment process is detected. It uses nice=10,
one BLAS thread, serial rounds, independent loopback ROS masters and isolated solver
paths. It does not signal unrelated processes. Per-round resource checks reserve
8 GiB of disk and require 1536 MiB of available memory before another round; the
capture process stops only its own simulation if available memory falls below
768 MiB. These are resource guards, not changes to target thresholds or injected delays.

After each round, only that completed trial's raw logs, scene array and solver
intermediates are gzip-compressed. Original and compressed hashes are recorded in
`archive.json`; decompressed bytes are verified before removing the raw copy.
Offline analysis reads both raw and compressed files. Existing experiments are never
compressed or modified. The full-batch disk projection is informational. A new round requires actual free
space for the 8 GiB reserve plus max(512 MiB, twice the largest recorded raw trial).
Rotated ROS logs such as rosout.log.1 are also losslessly compressed, including
files missed by an older archive marker.
The first profiling round belongs to the requested 3000, not an extra experiment.

`job.json` records the master seed, environment, resource policy and implementation
hashes. `progress.json` reports counts after each terminal round, pending rounds and
specific resource conflicts. Source/binary changes cause a stop before another round.
Three capture failures also stop the batch; unexecuted plans remain explicitly not run.
At termination, the run's `analysis.md` includes actual counts, per-target cluster
sample sizes, top training-ranked conditions with units and holdout metrics, missing
fields and limitations. `run_summary.json` provides machine-readable counts. Rules
also report the holdout non-hit sample trigger rate (false-positive rate), whose
denominator is the eligible non-hit samples, separately from non-hit rate among triggers.
No final rates or validated rules are claimed while the batch remains incomplete.

Resume a stopped job (the original output remains unchanged):

```bash
python3 src/RACER/tools/racer_latency_background.py --resume /absolute/path/to/background_TIMESTAMP_UUID
```

Resume allocates a fresh unique job/run, references finished trial directories with
symlinks and copies only pending plans. The original trial IDs, seeds, parameters and
3000-round total remain unchanged. Completed/failed recorded rounds are not rerun;
a partially captured round without a result requires review. The new job records
its parent and original implementation hashes. Algorithm, thresholds and analysis
changes are rejected on resume; lifecycle fixes are recorded by the new hashes.
Do not remove the original job: it contains the referenced evidence.

Cleanup now checks each process's explicit `__log:=<owned trial>/ros_logs/...`
argument and Linux process start time before signalling that process individually.
This covers ROS nodes that start independent sessions and survive roslaunch exit;
it never infers ownership from a name alone. Temporary resource/other-experiment
conflicts during execution produce `waiting_resources` with 30-second retries.
Source changes and repeated capture failures still stop execution explicitly.

## One-command six-target frozen-rule batch

```bash
source devel/setup.bash
cmake --build build --target exploration_node -j2
python3 src/RACER/tools/racer_rule_batch.py --per-target 100 --seed 20261111
# The launcher prints JOB_DIR, PID and the background log path.
python3 src/RACER/tools/racer_rule_batch.py --status JOB_DIR
```

One command starts **600 planned trials**, 100 per target, interleaved over six
objectives. The launcher detaches automatically. `--rules PATH` selects a directory
containing `accepted_rule_metrics.csv`, locked training candidates and manifest
from the high-gap rule analysis. The default points to this workspace's frozen
15-percentage-point analysis. No simulations are started by imports or `--status`.

At launch, choose a subset of 2–3 accepted rules per target by training-only
P(target hit | OR group), requiring training support >=20. Record the complete
predicates, thresholds, original units, chosen training support/precision and
source hashes in `frozen_rules.json`; freeze the drift definition at 0.075 m for
1 s for this batch. This training association is not a guaranteed attack probability.
Some OR rules may overlap; each trial is counted once.

Draw one sender and a cycle-based delay fraction from each new trial seed. Within
the existing task-time window [5,30] s, check rules at most 10 Hz using only received
pre-decision observations and the existing `features.extract` implementation.
WAIT_TRIGGER/INIT/IDLE/FINISH cannot arm an injection. Unknown fields do not match.
A matching decision publishes an isolated source-arm command with a same-host
monotonic token and a 150 ms expiry (the existing alignment tolerance); this is a
freshness guard, not a cap on injected latency. There is only one decision per
trial. Expired/rejected arms, no match or missing features are explicit skips.

The opt-in C++ source gate keeps normal sends until armed. Once armed, its original
measured-cycle, protocol budget and FIFO release semantics are unchanged. Existing
random-run and launch defaults keep rule mode disabled. No radio/realworld launch
is used. All masters, logs and solver paths are isolated per trial.

Offline rule validation checks the saved pre-arm vector, timestamp expiry, actual
source injection contract, and rule satisfaction at the delayed message's generation
time. Already-published delays are still actual injections when rule validation
fails; they are separately flagged and excluded from the primary rule-group target
rate. Six labels remain independent; primary results group by the assigned objective.
No retries are selected on outcomes or added automatically.

`progress.json` and `rule_group_results.csv` always include **all six target rows**:
planned/finished counts, actual injections, valid rule injections, valid target
samples, target hits/misses/rates and Wilson intervals, task completion, skipped
injections and capture failures. Completion of 100 attempts is separate from mission
success or target hit. `all_targets_completed` becomes true only when all six have
finished their planned attempts. `rule_matches.csv`, `run/trials.csv`,
`run/trial_features.csv`, individual plans/results/decisions/events, `manifest.json`
and `report.md` preserve evidence. The report is also exported on interruption.

Disk/memory checks, verified compression (including rotated logs), owned residual
cleanup, source/binary-change detection and three-capture-failure protection are
reused. Resource conflicts wait; other experiments are never signalled. To resume
a stopped unchanged job without repeating completed trials:

```bash
python3 src/RACER/tools/racer_rule_batch.py --resume JOB_DIR
```

An incomplete capture without a result requires review; resume does not overwrite
or silently rerun it. A file lock prevents duplicate workers for a job. Outputs use
new unique job directories; normal starts never reuse older experiments. No control
trials are added. Results are prospective frozen-rule observations with new seeds;
without paired uninjected controls they do not establish causal improvement.
