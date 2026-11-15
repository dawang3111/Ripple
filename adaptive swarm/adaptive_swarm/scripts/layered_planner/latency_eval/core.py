"""Transport, observable features and independent multi-label oracles. No ROS imports."""
import heapq
import math
from dataclasses import dataclass
from itertools import combinations

import numpy as np
from matplotlib.path import Path
from shapely.geometry import Point, Polygon

TARGETS = ('drone_drone', 'drone_obstacle', 'deadlock', 'timeout', 'planning_failure', 'drift')


@dataclass
class Config:
    max_ticks: int = 300
    rrt_iterations: int = 1000
    # None means undefined, not a negative observation.
    deadlock_window_s: float = None
    deadlock_progress_m: float = None
    drift_threshold_m: float = None
    drift_window_s: float = None
    protocol_timeout_s: float = None


class TxQueue:
    """One pending injection; immutable payloads, ordered per-sender sequence.

    External adapters must supply a measured period and a known next normal TX.
    Simulation adapter has a deterministic periodic publication schedule.
    """
    def __init__(self):
        self.events = []
        self.pending = []
        self.cache = {}
        self.last_seq = {}
        self.injection = {'attempted': False, 'actually_injected': False,
                          'skip_reason': 'planned_time_not_reached', 'used_stale_count': 0}
        self.now = -math.inf

    def initialize(self, positions):
        for sender, position in positions.items():
            self.cache[sender] = dict(sender_id=sender, seq=-1, payload=tuple(position),
                                      generated_time=0.0, release_time=0.0)

    def advance(self, now):
        if now < self.now:
            raise ValueError('simulation clock must be monotonic')
        self.now = now
        while self.pending and self.pending[0][0] <= now:
            release, sender, seq, message = heapq.heappop(self.pending)
            self._release(message, release)

    def _release(self, message, release):
        sender, seq = message['sender_id'], message['seq']
        if seq <= self.cache[sender]['seq']:
            raise ValueError('duplicate or out-of-order release')
        delivered = dict(message, release_time=release)
        self.cache[sender] = delivered
        self.events.append(dict(delivered, event='release'))
        if message.get('delayed'):
            self.injection.update(actually_injected=True, actual_release_time=release, actual_tx_time=release,
                                  actual_delay_s=release-message['generated_time'], skip_reason='')

    def publish(self, sender, seq, payload, now, next_normal_time, period,
                inject=False, fraction=0.5, state='RUNNING', protocol_timeout=None,
                simulation_end=None):
        if not np.isfinite(now) or not np.isfinite(payload).all():
            raise ValueError('nonfinite message timestamp/payload')
        self.advance(now)
        if seq <= self.last_seq.get(sender, -1):
            raise ValueError('duplicate or out-of-order publication')
        if any(item[1] == sender for item in self.pending):
            raise ValueError('next TX arrived before pending message release')
        self.last_seq[sender] = seq
        message = dict(sender_id=sender, seq=seq, payload=tuple(float(x) for x in payload),
                       generated_time=now, nominal_tx_time=now, delayed=False)
        if inject and not self.injection['attempted']:
            delay = period * fraction if period is not None else None
            reason = ''
            if state != 'RUNNING':
                reason = 'task_not_running'
            elif period is None or not np.isfinite(period) or period <= 0:
                reason = 'message_period_unavailable'
            elif not 0 < fraction < 1:
                reason = 'fraction_outside_open_interval'
            elif next_normal_time is None or now + delay >= next_normal_time:
                reason = 'cannot_release_before_next_normal_tx'
            elif protocol_timeout is not None and delay >= protocol_timeout:
                reason = 'protocol_timeout_constraint'
            elif simulation_end is not None and now + delay > simulation_end:
                reason = 'simulation_end_constraint'
            self.injection.update(attempted=True, sender_id=sender, seq=seq,
                                  nominal_tx_time=now, measured_period_s=period,
                                  next_normal_time=next_normal_time, planned_delay_s=delay,
                                  planned_release_time=None if delay is None else now+delay,
                                  skip_reason=reason)
            if not reason:
                message['delayed'] = True
                self.events.append(dict(message, event='enqueue'))
                heapq.heappush(self.pending, (now+delay, sender, seq, message))
                return
        self._release(message, now)

    def receive(self, sender, receiver, now):
        self.advance(now)
        message = self.cache[sender]
        stale = any(item[1] == sender for item in self.pending)
        if stale:
            self.injection['used_stale_count'] += 1
        self.events.append(dict(message, event='receive', receiver_id=receiver,
                                receive_time=now, pending_injection=stale))
        return np.array(message['payload'])

    def finish(self, now):
        self.advance(now)
        if self.pending:
            self.injection['skip_reason'] = 'simulation_ended_before_release'
            self.events.extend(dict(item[3], event='cancel', cancel_time=now)
                               for item in self.pending)
            self.pending.clear()


def feature_vector(positions, histories, source, now, cache, targets, obstacles,
                   goal, sp_index, setpoint_count):
    """Only snapshots/history already visible at pre-enqueue time; no future input.

    histories: id -> [(source_time, xy), ...], with source timestamps <= now.
    Local cluster = all four robots in this centralized simulation.
    """
    out = {}
    times = {str(i): h[-1][0] for i, h in histories.items()}
    def add(name, value, unit, origin, reason='', source_times=None):
        if value is not None and not np.isfinite(value):
            value, reason = None, 'nonfinite_observation'
        out[name] = dict(value=None if value is None else float(value), unit=unit,
                         observed_time=now, source_times=source_times or times,
                         source=origin, missing_reason=reason)
    velocities, accelerations = {}, {}
    for i, h in histories.items():
        if any(t > now for t, _ in h):
            raise ValueError('future observation leaked into features')
        if len(h) >= 3 and h[-1][0] > h[-2][0]:
            velocities[i] = (np.array(h[-1][1])-h[-2][1])/(h[-1][0]-h[-2][0])
        if len(h) >= 4 and h[-2][0] > h[-3][0]:
            prev = (np.array(h[-2][1])-h[-3][1])/(h[-2][0]-h[-3][0])
            accelerations[i] = (velocities[i]-prev)/(h[-1][0]-h[-2][0])
    for name, data, unit in [('speed', velocities, 'm/s'), ('acceleration', accelerations, 'm/s^2')]:
        add('source_'+name, np.linalg.norm(data[source]) if source in data else None,
            unit, 'position differences / nominal simulation timestamps', 'insufficient_history' if source not in data else '',
            source_times={str(source): [t for t,_ in histories[source][-3:]]})
        add('cluster_mean_'+name, np.mean([np.linalg.norm(v) for v in data.values()]) if len(data)==len(positions) else None,
            unit, 'all robots position differences', 'insufficient_history' if len(data)!=len(positions) else '',
            source_times={str(i): [t for t,_ in h[-3:]] for i,h in histories.items()})
    h = histories[source]
    old = [p for t, p in h if t <= h[-1][0]-5.0]
    add('source_displacement_5s', np.linalg.norm(np.array(h[-1][1])-old[-1]) if old else None,
        'm', 'source position history', 'history_shorter_than_5s' if not old else '')
    turn = None
    if len(h) >= 3:
        a, b = np.array(h[-1][1])-h[-2][1], np.array(h[-2][1])-h[-3][1]
        if np.linalg.norm(a)*np.linalg.norm(b)>0:
            turn = np.arccos(np.clip(np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b)), -1, 1))
    add('source_turn_angle', turn, 'rad', 'last two displacement vectors; not body yaw',
        'insufficient_nonzero_displacements' if turn is None else '')
    pairs = [np.linalg.norm(np.array(a)-b) for a,b in combinations(positions.values(),2)]
    add('current_min_pair_distance', min(pairs), 'm', 'latest mixed-phase truth snapshots')
    past = {i: [(t,p) for t,p in h if t <= now-5.] for i,h in histories.items()}
    complete = all(past.values())
    old_min = min(np.linalg.norm(np.array(a[-1][1])-b[-1][1])
                  for a,b in combinations(past.values(),2)) if complete else None
    add('min_pair_distance_change_5s', min(pairs)-old_min if complete else None, 'm',
        'current minimum minus latest-as-of 5s historical minimum (pair may change)',
        'history_shorter_than_5s' if not complete else '',
        source_times={'current':times,'past':{str(i): h[-1][0] for i,h in past.items() if h}})
    polys = [Polygon(o) for o in obstacles]
    add('current_min_obstacle_distance', min(p.distance(Point(x)) for p in polys for x in positions.values()),
        'm', 'center point to original polygon; not body clearance')
    offsets = {i: np.linalg.norm(np.array(p)-targets[i]) for i,p in positions.items()}
    add('source_target_offset', offsets[source], 'm', 'current assigned target; not trajectory deviation')
    add('cluster_max_target_offset', max(offsets.values()), 'm', 'current assigned targets')
    add('leader_goal_distance', np.linalg.norm(np.array(positions[1])-goal), 'm', 'leader truth / goal')
    add('setpoint_progress', sp_index/max(1,setpoint_count-1), 'fraction', 'global setpoint index / last index')
    add('elapsed_task_time', now, 's', 'adapter RUNNING clock')
    add('source_truth_cached_broadcast_error', np.linalg.norm(np.array(positions[source])-cache[source]['payload']),
        'm', 'pre-enqueue truth vs last released broadcast',
        source_times={'truth': histories[source][-1][0], 'broadcast': cache[source]['generated_time']})
    angles=[]
    for a,b in combinations(velocities.values(),2):
        if np.linalg.norm(a)*np.linalg.norm(b)>0:
            angles.append(np.arccos(np.clip(np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b)),-1,1)))
    add('cluster_max_course_angle', max(angles) if angles else None, 'rad', 'displacement course; not body heading',
        'insufficient_nonzero_velocities' if not angles else '')
    return out


class Oracles:
    def __init__(self, config, crash_distance, goal_tolerance):
        self.config, self.crash_distance, self.goal_tolerance = config, crash_distance, goal_tolerance
        self.labels = {t: False for t in TARGETS}
        self.known = {t: True for t in TARGETS}
        self.known['deadlock'] = config.deadlock_window_s is not None
        self.known['drift'] = config.drift_threshold_m is not None
        self.evidence = {t: [] for t in TARGETS}
        self.history = []
        self.drift_since = {}

    def hit(self, target, now, ids, state, planning, **detail):
        self.labels[target] = True
        if not self.evidence[target]:
            self.evidence[target].append(dict(time_s=now, uav_ids=list(ids), fsm=state,
                                              planning_state=planning, **detail))

    def observe(self, now, positions, targets, obstacles, goal, sp_index,
                state='RUNNING', planning='LOCAL_APF'):
        for (i,a),(j,b) in combinations(positions.items(),2):
            distance=float(np.linalg.norm(np.array(a)-b))
            if distance < self.crash_distance:
                self.hit('drone_drone', now, [i,j], state, planning, distance_m=distance)
        for i,p in positions.items():
            for oi,o in enumerate(obstacles):
                if Path(o).contains_point(p):
                    self.hit('drone_obstacle', now, [i], state, planning, obstacle_index=oi)
        d=float(np.linalg.norm(np.array(positions[1])-goal))
        self.history.append((now,d,sp_index))
        c=self.config
        if self.known['deadlock']:
            past=[v for v in self.history if v[0] <= now-c.deadlock_window_s]
            if past:
                start=past[-1]
                window=[v for v in self.history if v[0]>=start[0]]
                if (d > self.goal_tolerance and sp_index==start[2]
                        and max(v[1] for v in window)-min(v[1] for v in window)<=c.deadlock_progress_m):
                    self.hit('deadlock',now,positions,state,planning,window_start_s=start[0],
                             definition='stalled setpoint and bounded leader goal-distance range')
        if self.known['drift']:
            for i,p in positions.items():
                error=float(np.linalg.norm(np.array(p)-targets[i]))
                if error > c.drift_threshold_m:
                    self.drift_since.setdefault(i,now)
                    if now-self.drift_since[i]>=c.drift_window_s:
                        self.hit('drift',now,[i],state,planning,offset_m=error,
                                 definition='sustained current-target tracking error')
                else:
                    self.drift_since.pop(i,None)

    def finish(self,status,now):
        if status=='timeout':
            self.hit('timeout',now,[],'TERMINATED','LOCAL_APF',definition='mission tick budget exhausted')
        elif status=='rrt_failed':
            self.hit('planning_failure',now,[],'PLANNING','GLOBAL_RRT',definition='global planning initialization failed')
            for t in TARGETS:
                if t!='planning_failure': self.known[t]=False
        elif status=='error':
            self.known={t:self.labels[t] for t in TARGETS}
