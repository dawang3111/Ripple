# Bug Catalog

This document summarizes the 20 implementation findings reported for EGO-Planner-v2, RACER, and Adaptive Swarm in the Ripple study. It retains the original internal Bug IDs used by the paper: #1-#15 and #22-#26. These IDs are local catalog identifiers, not GitHub issue or pull-request numbers.

The descriptions concern the reported test configurations. Relative source links provide navigation to the relevant components in this artifact; they do not establish that every upstream version has the same behavior. Version-specific source-location differences are noted where applicable. Suggested mitigations are proposals, not claims that an upstream fix has been accepted or applied. The catalog itself does not constitute a new reproduction experiment.

All 20 findings were reported to the corresponding maintainers according to the study's disclosure records. Bugs #1, #5, and #12 have assigned CVE identifiers; the identifiers and external report links are withheld for anonymous review. A reported finding or a CVE assignment does not, by itself, imply maintainer confirmation.

## Organization and Root Causes

- [EGO-Planner-v2](#ego-planner-v2): 6 findings, #1-#6.
- [RACER](#racer): 9 findings, #7-#15.
- [Adaptive Swarm](#adaptive-swarm): 5 findings, #22-#26.

The primary root-cause labels correspond to the paper:

- **RC1 - Planning and execution inconsistency:** planned, broadcast, and executed motion are not sufficiently aligned in time or geometry.
- **RC2 - Inconsistent swarm coordination:** local state changes, participation, or task progress are not consistently reflected in collective decisions.
- **RC3 - Incomplete collision validation:** collision checks do not cover all relevant motion or are not enforced as acceptance constraints.
- **RC4 - Insufficient failure recovery:** repeated planning failures do not lead to an effective alternative or fallback.
- **RC5 - Fragile runtime state management:** flags, indices, timers, or state assumptions fail under exceptional execution sequences.

Bug #10 is an initialization-order issue. It is included in the complete catalog but is not treated as a communication-delay mechanism in the five-root-cause discussion. Not every catalog entry independently demonstrates an inter-UAV delay attack or every task-level objective.

## EGO-Planner-v2

### Bug #1: Emergency-stop re-entry without braking

- **Code location:** [ego_replan_fsm.cpp][ego-fsm], `EGOReplanFSM::execFSMCallback()`, particularly `EMERGENCY_STOP` and `REPLAN_TRAJ`.
- **Trigger:** Replanning succeeds while emergency braking is still in progress, before the normal emergency-stop recovery sequence completes. A later conflict causes another emergency-stop entry.
- **Defect:** The first emergency-stop execution clears `flag_escape_emergency_`. Successful replanning does not restore it. On re-entry, the false flag skips `callEmergencyStop()`; the alternative recovery branch requires fail-safe mode and sufficiently low speed.
- **Consequence:** The UAV can enter the emergency-stop state without issuing a new braking action or emergency-stop trajectory broadcast. Continued motion can lead to an inter-UAV collision.
- **Primary root cause:** RC5.
- **Suggested mitigation:** Restore the emergency-stop action state after successful recovery or replanning, or make the braking action depend on a correctly tracked state entry rather than a stale flag. Check recovery across all relevant transitions.
- **Disclosure:** Reported; CVE assigned, identifier withheld for anonymous review.
- **Paper connection:** Case Study 1.

### Bug #2: Emergency-stop broadcast omits the braking motion

- **Code location:** [ego_replan_fsm.cpp][ego-fsm], `callEmergencyStop()`; [planner_manager.cpp][ego-manager], `EGOPlannerManager::EmergencyStop()`.
- **Trigger:** An emergency stop is commanded while the UAV is moving.
- **Defect:** The emergency trajectory describes a stationary stop position, with zero-velocity boundary states, rather than the physical motion swept while the UAV decelerates. Representing a stop point does not account for non-instantaneous braking.
- **Consequence:** Neighboring UAVs can plan against a stationary representation while the stopping UAV is still moving, leaving the braking motion insufficiently represented in swarm collision checks.
- **Primary root cause:** RC1.
- **Suggested mitigation:** Generate and broadcast a dynamically feasible braking trajectory from the current motion state, and validate its clearance throughout deceleration.
- **Disclosure:** Reported.

### Bug #3: Collision checking omits a trajectory suffix

- **Code location:** [ego_replan_fsm.cpp][ego-fsm], `EGOReplanFSM::checkCollisionCallback()`.
- **Trigger:** The UAV approaches the unchecked end of a local trajectory, for example when replanning takes longer than expected.
- **Defect:** When the local target is not the final goal, the collision-check loop uses only three quarters of the stored check-point groups. The remaining suffix is not covered by that invocation of the check. The final-goal branch has different coverage.
- **Consequence:** The UAV can execute motion beyond the checked horizon without an equivalent check of the remaining suffix, reducing the opportunity to detect obstacles or conflicting neighboring trajectories.
- **Primary root cause:** RC3.
- **Suggested mitigation:** Validate all relevant unexecuted trajectory segments, including the suffix that may be reached during replanning, or ensure execution cannot enter an unchecked segment.
- **Disclosure:** Reported.

### Bug #4: Replanning start state becomes stale before execution

- **Code location:** [ego_replan_fsm.cpp][ego-fsm], `planFromLocalTraj()` and `callReboundReplan()`.
- **Trigger:** A new trajectory is generated while the UAV continues executing its current trajectory.
- **Defect:** The replanning start state is sampled at the beginning of planning. By the time the new trajectory is available and published, the UAV may have advanced beyond that state; elapsed motion is not fully reflected in the transition to the new plan.
- **Consequence:** The planned or broadcast motion can disagree with the UAV's actual execution state, introducing discontinuity or inaccurate motion predictions for neighboring UAVs.
- **Primary root cause:** RC1.
- **Suggested mitigation:** Align the trajectory start state and start time with the intended execution instant, account for planning latency, and validate the transition from the currently executed trajectory.
- **Disclosure:** Reported.

### Bug #5: Heartbeat recovery does not restore trajectory reception state

- **Code location:** [ego_replan_fsm.cpp][ego-fsm], `execFSMCallback()`; [traj_server.cpp][ego-server], `heartbeatCallback()`, `polyTrajCallback()`, and `cmdCallback()`.
- **Trigger:** A heartbeat timeout is detected around a replanning or trajectory-delivery transition, leaving `receive_traj_` false even though planner heartbeats subsequently resume.
- **Defect:** The command callback returns early while `receive_traj_` is false. Receipt of a heartbeat updates the heartbeat time but does not itself restore trajectory execution eligibility. Recovery therefore depends on a separate trajectory event, rather than heartbeat recovery alone.
- **Consequence:** The trajectory server can remain unable to resume normal command publication after the planner has recovered, preventing mission progress.
- **Primary root cause:** RC5.
- **Suggested mitigation:** Define an explicit recovery handshake that reconciles fresh heartbeats with valid trajectory availability, rather than leaving execution controlled by an unreconciled flag.
- **Disclosure:** Reported; CVE assigned, identifier withheld for anonymous review.

### Bug #6: Heartbeat-timeout handling falls through to trajectory execution

- **Code location:** [traj_server.cpp][ego-server], `cmdCallback()`.
- **Trigger:** The planner heartbeat is absent for more than 0.5 seconds while a trajectory has previously been received.
- **Defect:** The timeout branch clears `receive_traj_` and publishes a hold command with zero motion derivatives, but does not return. The same callback can continue evaluating and publishing the current trajectory.
- **Consequence:** A trajectory command can follow the protective hold command in the same callback, weakening the intended timeout response. This is distinct from the subsequent recovery problem in Bug #5.
- **Primary root cause:** RC5.
- **Suggested mitigation:** Return immediately after the timeout response or use mutually exclusive control flow that prevents normal trajectory commands from being issued after a timeout is handled.
- **Disclosure:** Reported.

## RACER

### Bug #7: Replanning precedes the emergency-braking feasibility decision

- **Reported code location:** Swarm-safety and emergency-response logic, identified as `ego_replan_fsm.cpp` in the original reported planner variant. The related RACER safety and replanning entry points in this artifact are [fast_exploration_fsm.cpp][racer-fsm], including `safetyCallback()`, `swarmTrajCallback()`, and `FSMCallback()`. The original source filename should not be interpreted as a file present in every RACER variant.
- **Trigger:** A conflicting trajectory is detected with limited remaining time for a safe stop.
- **Defect:** In the reported response sequence, the planner first attempts replanning and only subsequently decides whether the remaining time requires emergency braking. Planning consumes part of the available reaction window.
- **Consequence:** A braking action that was feasible at detection time can be initiated too late after replanning has consumed the safety margin.
- **Primary root cause:** RC1.
- **Suggested mitigation:** Evaluate stopping feasibility and the available safety margin before starting a potentially expensive replanning attempt, and initiate a safe fallback when the remaining margin is insufficient.
- **Disclosure:** Reported.

### Bug #8: Transition to IDLE does not broadcast a hovering trajectory

- **Code location:** [fast_exploration_fsm.cpp][racer-fsm], `FSMCallback()` and `swarmTrajTimerCallback()`.
- **Trigger:** Local exploration or planning transitions to `IDLE`, for example after the assigned grid list becomes empty.
- **Defect:** The trajectory broadcast timer handles executing trajectories and the waiting-for-trigger state, but does not provide an equivalent updated hovering trajectory for `IDLE`.
- **Consequence:** Neighboring UAVs can retain the previous moving trajectory while this UAV hovers, producing inconsistent shared motion predictions.
- **Primary root cause:** RC2.
- **Suggested mitigation:** Publish a valid stationary trajectory when entering or remaining in `IDLE`, with consistent timing and normal trajectory-message handling.
- **Disclosure:** Reported.

### Bug #9: Collision penalties do not enforce hard trajectory rejection

- **Code location:** [bspline_optimizer.cpp][racer-optimizer], `BsplineOptimizer::optimize()`, obstacle-distance costs, swarm costs, and combined-cost selection.
- **Trigger:** B-spline optimization refines a route and a low-cost candidate still violates obstacle or inter-UAV clearance.
- **Defect:** Collision-related terms contribute to the optimization objective, but a minimum weighted cost is not itself a guarantee that the resulting trajectory is collision-free. The reported path lacks an effective hard rejection of the unsafe optimized candidate before execution.
- **Consequence:** An optimized trajectory can remain unsafe even when the preceding geometric route search succeeded.
- **Primary root cause:** RC3.
- **Suggested mitigation:** Validate the complete optimized trajectory before acceptance and reject candidates that violate collision constraints, independently of their weighted optimization cost.
- **Disclosure:** Reported.

### Bug #10: State publication can precede valid odometry initialization

- **Code location:** [fast_exploration_fsm.cpp][racer-fsm], `init()`, `droneStateTimerCallback()`, and `odometryCallback()`.
- **Trigger:** A state-publication timer executes before the first valid odometry update initializes the state fields it reads.
- **Defect:** State publication can consume odometry-derived values before their initialization or readiness has been established for that callback.
- **Consequence:** Invalid initial state values can be published or used in subsequent coordination decisions.
- **Scope:** Independent initialization-order issue; not a demonstrated communication-delay mechanism and excluded from the delay-related root-cause discussion.
- **Suggested mitigation:** Initialize all state fields explicitly and guard state publication with odometry readiness until valid measurements are available.
- **Disclosure:** Reported.

### Bug #11: Old trajectory remains executable during replanning fallback

- **Code location:** [traj_server.cpp][racer-server], `replanCallback()` and `cmdCallback()`; [fast_exploration_fsm.cpp][racer-fsm], the safety-to-planning transition.
- **Trigger:** A trajectory conflict is detected and a replacement trajectory cannot be generated immediately.
- **Defect:** Replanning does not immediately invalidate unsafe portions of the currently executed trajectory. `replanCallback()` retains a continuation interval determined by `time_out` and the configured replanning time. The reported execution included continued old-trajectory motion and a delayed hovering fallback, with approximately 0.2-second and 0.4-second intervals in the reported configuration; these intervals are configuration-dependent.
- **Consequence:** The UAV can reach an already hazardous portion of the previous trajectory before the replacement or fallback takes effect.
- **Primary root cause:** RC1.
- **Suggested mitigation:** Validate the entire continuation interval and initiate a safe stop or hover before entering an unsafe segment, rather than relying on a fixed grace interval.
- **Disclosure:** Reported.

### Bug #12: Invalid viewpoint index is used without checking selection success

- **Code location:** [fast_exploration_manager.cpp][racer-manager], `FastExplorationManager::planExploreMotion()` and the selection of `points_[min_cost_id]` and `yaws_[min_cost_id]`.
- **Trigger:** No valid viewpoint is selected, including when frontier updates leave the candidate list empty.
- **Defect:** The candidate index is initialized to `-1`. If selection does not update it, the index is still used for vector access without a validity check. In C++, this is out-of-bounds access, not a valid operation for reading the last element.
- **Consequence:** The read has undefined behavior. In the reported failure sequence, an invalid target reaches planning and the UAV cannot make further exploration progress. Undefined behavior does not guarantee the same failure mode in every execution.
- **Primary root cause:** RC5.
- **Suggested mitigation:** Require successful candidate selection and an in-range index before access. If no valid candidate exists, update the exploration assignment or enter a defined safe fallback.
- **Disclosure:** Reported; CVE assigned, identifier withheld for anonymous review.
- **Paper connection:** Case Study 2.

### Bug #13: Local exploration completion does not ensure global task completion

- **Code location:** [fast_exploration_fsm.cpp][racer-fsm], the `IDLE` and return-to-start paths; [fast_exploration_manager.cpp][racer-manager], exploration and grid-assignment handling.
- **Trigger:** A UAV has no remaining local exploration work while another UAV remains stalled or repeatedly fails in an assigned region.
- **Defect:** The local completion and return logic does not establish that all remaining global frontiers and grids have been completed or successfully reassigned. The `IDLE` branch can initiate return after its waiting interval without a corresponding global-completion check.
- **Consequence:** A healthy UAV can finish its local participation while other assigned regions remain unexplored, preventing complete collective coverage.
- **Primary root cause:** RC2.
- **Suggested mitigation:** Check global remaining work before declaring completion or returning, and provide reassignment or takeover when another UAV cannot progress.
- **Disclosure:** Reported.

### Bug #14: Repeated planning failures lack an effective alternative-viewpoint policy

- **Code location:** [fast_exploration_fsm.cpp][racer-fsm], `FSMCallback()` and `callExplorationPlanner()`; [fast_exploration_manager.cpp][racer-manager], `planExploreMotion()` and `planTrajToView()`.
- **Trigger:** The UAV reaches a state from which its currently selected viewpoint cannot be reached by a feasible route.
- **Defect:** The failure path retries planning without an effective failure-history-based exclusion or replacement policy for the repeatedly unreachable target. Re-entering planning can select the same viewpoint again even when it cannot resolve the infeasibility.
- **Consequence:** Repeated planning failures can persist without exploration progress, producing a deadlock while the mission remains incomplete.
- **Primary root cause:** RC4.
- **Suggested mitigation:** Track repeated target-specific failures, select an alternative feasible viewpoint or reassign the affected task, and provide a bounded safe fallback.
- **Disclosure:** Reported.

### Bug #15: Inactive WAIT_TRIGGER UAVs can participate in task allocation

- **Code location:** [fast_exploration_fsm.cpp][racer-fsm], `optTimerCallback()`, pairwise optimization callbacks, and swarm-state publication.
- **Trigger:** A UAV remains in `WAIT_TRIGGER` while its published position or state remains eligible for grid assignment or pairwise task optimization.
- **Defect:** Allocation eligibility does not consistently distinguish a physically present, waiting UAV from an activated UAV participating in the exploration mission.
- **Consequence:** Work can be assigned to a UAV that has not started executing the mission, leaving assigned grids unexplored or distorting load balancing.
- **Primary root cause:** RC2.
- **Suggested mitigation:** Incorporate explicit mission-participation state into allocation eligibility and exclude inactive UAVs from task assignments while retaining their physical presence for collision avoidance.
- **Disclosure:** Reported.

## Adaptive Swarm

### Bug #22: Local obstacle avoidance does not validate the next position

- **Code location:** [Adaptive_swarm.py][adaptive-planner], `Robot.local_planner()`; [potential_fields.py][adaptive-fields], potential-field construction.
- **Trigger:** The combined attractive and repulsive field produces a step whose endpoint or swept segment intersects an obstacle.
- **Defect:** The local planner directly commits the computed position update without a hard geometric collision-rejection check. Repulsion influences the direction but does not guarantee a collision-free next position.
- **Consequence:** A local motion update can enter an obstacle even though the obstacle contributes a repulsive potential.
- **Primary root cause:** RC3.
- **Suggested mitigation:** Validate both the candidate endpoint and the swept motion against obstacle clearance, and reject or modify unsafe steps before execution.
- **Disclosure:** Reported.

### Bug #23: Local inter-UAV avoidance does not validate the next position

- **Code location:** [Adaptive_swarm.py][adaptive-planner], `Robot.local_planner()` and the follower-update loop; [tools.py][adaptive-tools], robot-to-polygon conversion.
- **Trigger:** A computed step places a UAV too close to another UAV, particularly when coordination relies on delayed neighbor information.
- **Defect:** Neighboring UAVs contribute repulsive terms, but the resulting next position is not subject to a hard inter-UAV clearance check before it is committed.
- **Consequence:** UAV positions can overlap or violate the required separation despite the presence of repulsive forces. This is the inter-UAV counterpart of the obstacle issue in Bug #22.
- **Primary root cause:** RC3.
- **Suggested mitigation:** Check candidate motion against neighboring UAV occupancy or predicted motion at matching times, and reject or adjust updates that violate separation constraints.
- **Disclosure:** Reported.

### Bug #24: Repulsion activation leaves insufficient reaction margin

- **Code location:** [Adaptive_swarm.py][adaptive-planner], `Params` and `Robot.local_planner()`; [potential_fields.py][adaptive-fields], repulsive influence construction.
- **Trigger:** A UAV approaches another UAV or an obstacle while the local correction is based on delayed information.
- **Defect:** In the reported configuration, the repulsive influence becomes effective with limited margin beyond collision clearance. The remaining distance is insufficient for delayed detection and the fixed-step correction to act reliably. This is a configuration-dependent margin issue, not a claim about every possible radius setting.
- **Consequence:** Avoidance that is adequate with timely information can become too late under communication delay.
- **Primary root cause:** RC3.
- **Suggested mitigation:** Choose the influence margin using relative motion, update age, and achievable corrective motion, and retain explicit next-step collision validation.
- **Disclosure:** Reported.

### Bug #25: Near-zero leader-reference direction destabilizes follower alignment

- **Code location:** [Adaptive_swarm.py][adaptive-planner], formation-reference construction in `BaselineSimulation.start()` and `Robot.local_planner()`; [tools.py][adaptive-tools], `formation()` and `normalize()`.
- **Trigger:** The virtual reference point is close to the leader position, so small position changes substantially alter their normalized difference direction. Delayed updates make the followers use inconsistent or outdated references.
- **Defect:** Follower setpoints depend on this sensitive direction, while each nonzero local-gradient update is normalized to a fixed step length. The reported configuration uses a 0.04 m step, so a large direction change is not automatically accompanied by a smaller displacement.
- **Consequence:** Followers can repeatedly change steering direction, disrupting the formation and consuming steps on alignment rather than forward progress. Collisions can also occur when geometric rejection checks are absent.
- **Primary root cause:** RC2.
- **Suggested mitigation:** Use a stable reference heading when the leader-reference displacement is small, and bound changes in formation orientation and local motion.
- **Disclosure:** Reported.
- **Paper connection:** Contributes to Case Study 3 together with Bug #26.

### Bug #26: Reference point advances without explicit formation recovery

- **Code location:** [Adaptive_swarm.py][adaptive-planner], `BaselineSimulation.start()`, centroid calculation, and the `sp_ind` advancement condition.
- **Trigger:** Followers remain misaligned or oscillate while the swarm centroid is still sufficiently close to the virtual reference point.
- **Defect:** Advancement is gated by a centroid-to-reference distance condition, without an explicit requirement that individual followers have restored the formation. The reference can therefore keep advancing while the followers are still correcting their positions.
- **Consequence:** Together with Bug #25, fixed-length follower steps can be spent mainly on repeated alignment corrections. Effective forward progress becomes small and mission completion is prolonged. The paper's task-timeout objective additionally requires eventual completion after more than twice the delay-free reference time.
- **Primary root cause:** RC2.
- **Suggested mitigation:** Gate reference advancement on explicit formation-recovery conditions, including individual follower errors, and resume advancement after recovery.
- **Disclosure:** Reported.
- **Paper connection:** Contributes to Case Study 3 together with Bug #25.

[ego-fsm]: ../EGO/src/planner/plan_manage/src/ego_replan_fsm.cpp
[ego-manager]: ../EGO/src/planner/plan_manage/src/planner_manager.cpp
[ego-server]: ../EGO/src/planner/plan_manage/src/traj_server.cpp
[racer-fsm]: ../racer/swarm_exploration/exploration_manager/src/fast_exploration_fsm.cpp
[racer-manager]: ../racer/swarm_exploration/exploration_manager/src/fast_exploration_manager.cpp
[racer-server]: ../racer/swarm_exploration/plan_manage/src/traj_server.cpp
[racer-optimizer]: ../racer/swarm_exploration/bspline_opt/src/bspline_optimizer.cpp
[adaptive-planner]: ../adaptive%20swarm/adaptive_swarm/scripts/layered_planner/Adaptive_swarm.py
[adaptive-fields]: ../adaptive%20swarm/adaptive_swarm/scripts/layered_planner/potential_fields.py
[adaptive-tools]: ../adaptive%20swarm/adaptive_swarm/scripts/layered_planner/tools.py
