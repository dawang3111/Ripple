"""Periodic TX adapter around existing RRT/APF primitives; legacy start is untouched."""
import contextlib
import io
import time

import numpy as np
import matplotlib.pyplot as plt

from ..Adaptive_swarm_collsion_findradius_obstacle_inflation import BaselineSimulation, to_builtin
from ..rrt import rrt_path, ShortenPath
from ..tools import formation, normalize, poses2polygons, waypts2setpts
from .core import TxQueue, Oracles, feature_vector


class TxSimulation(BaselineSimulation):
    def __init__(self, episode, folder, seed, config, injection_policy=None):
        super().__init__(episode, str(folder), seed=seed)
        self.config = config
        self.transport = TxQueue()
        self.oracle = Oracles(config, self.params.crash_uav_dist, self.params.goal_tolerance)
        self.dt = 1.0/self.params.ViconRate
        # Independent RNG: injection selection cannot perturb RRT random stream.
        rng = np.random.RandomState(seed ^ 0x5A17)
        self.plan = dict(tick=int(rng.randint(1,config.max_ticks)),
                         sender_id=int(rng.randint(1,self.params.num_robots+1)),
                         fraction=float(rng.uniform(np.nextafter(0.,1.),np.nextafter(1.,0.))))
        self.control = False
        self.injection_policy = injection_policy
        self.features = {}
        self.histories = {}
        self.trace = []
        self.clock = 0.0
        self.P = []
        self.trajectory = []

    def positions(self):
        return {r.id:r.sp.copy() for r in self.robots}

    def publish(self, robot, tick, index):
        now=tick*self.dt+(index+1)*self.dt/len(self.robots)
        self.clock=now
        self.transport.advance(now)
        h=self.histories[robot.id]
        h.append((now,robot.sp.copy()))
        period=h[-1][0]-h[-2][0] if len(h)>=3 else None
        inject=tick==self.plan['tick'] and robot.id==self.plan['sender_id']
        if self.injection_policy is not None:
            inject=False
            if self.injection_policy.should_observe(robot.id,tick,period):
                snapshot=feature_vector(self.positions(),self.histories,robot.id,now,
                    self.transport.cache,{r.id:r.sp_global.copy() for r in self.robots},
                    self.obstacles,self.xy_goal,self.sp_index,len(self.trajectory))
                inject=self.injection_policy.decide(snapshot,now,tick,self.sp_index)
                if inject:
                    self.features=snapshot
                    self.plan['tick']=tick
        if inject and self.injection_policy is None:
            self.features=feature_vector(self.positions(),self.histories,robot.id,now,
                self.transport.cache,{r.id:r.sp_global.copy() for r in self.robots},
                self.obstacles,self.xy_goal,self.sp_index,len(self.trajectory))
        self.transport.publish(robot.id,tick,robot.sp,now,now+self.dt,period,
            inject=inject and not self.control,fraction=self.plan['fraction'],state='RUNNING',
            protocol_timeout=self.config.protocol_timeout_s,
            simulation_end=self.config.max_ticks*self.dt)

    def start(self):
        status,note='error',''
        try:
            status,note=self._run()
        except Exception as exc:
            status,note='error',type(exc).__name__+': '+str(exc)
        finally:
            self.transport.finish(self.clock)
            if self.injection_policy is not None and not self.transport.injection['attempted']:
                self.transport.injection['skip_reason']=self.injection_policy.skip_reason()
            if self.control:
                self.transport.injection['skip_reason']='explicit_control'
                if self.features:
                    self.transport.injection['nominal_tx_time']=next(iter(self.features.values()))['observed_time']
            self.oracle.finish(status,self.clock)
            plt.close('all')
        result=dict(episode=self.episode_id,seed=self.seed,status=status,note=note,
            tick=self.time_step,time_s=self.clock,plan=self.plan,injection=self.transport.injection,
            labels=self.oracle.labels,known=self.oracle.known,evidence=self.oracle.evidence,
            features=self.features,trace=self.trace,messages=self.transport.events,
            params=vars(self.params),config=vars(self.config),obstacles=self.obstacles,
            start=self.xy_start,goal=self.xy_goal,rrt_path=self.P,trajectory_setpoints=self.trajectory,
            rrt_plan_seconds=self.rrt_plan_seconds)
        if self.injection_policy is not None:
            result['rule_gate']=self.injection_policy.report()
        return to_builtin(result)

    def _run(self):
        started=time.monotonic()
        try:
            with contextlib.redirect_stdout(io.StringIO()):
                path=rrt_path(self.inflated_obstacles,self.xy_start,self.xy_goal,self.params,
                              max_iterations=self.config.rrt_iterations)
                self.P=ShortenPath(path,self.inflated_obstacles,smoothiters=50)
                self.trajectory=waypts2setpts(self.P,self.params)
            if len(self.trajectory)==0 or not np.isfinite(self.trajectory).all():
                raise ValueError('empty/nonfinite global trajectory')
        except Exception as exc:
            return 'rrt_failed',type(exc).__name__+': '+str(exc)
        finally:
            self.rrt_plan_seconds=time.monotonic()-started
        self.robot1.sp=self.trajectory[0].copy()
        followers=formation(self.params.num_robots,self.robot1.sp,v=np.array([0,-1]),
                            l=self.params.interrobots_dist)
        for r,p in zip(self.robots[1:],followers): r.sp=p.copy()
        for r in self.robots:
            r.route=np.array([r.sp.copy()])
            r.sp_global=r.sp.copy()
            self.histories[r.id]=[(0.,r.sp.copy())]
        self.transport.initialize(self.positions())
        self.sp_index=0
        for tick in range(self.config.max_ticks):
            self.time_step=tick
            if np.linalg.norm(self.robot1.sp-self.xy_goal)<self.params.goal_tolerance:
                return 'success',''
            self.robot1.sp_global=self.trajectory[self.sp_index].copy()
            self.robot1.local_planner(np.array(self.obstacles),self.params)
            desired=formation(self.params.num_robots,self.robot1.sp_global,
                v=normalize(self.robot1.sp_global-self.robot1.sp),l=self.params.interrobots_dist)
            if not np.isfinite(desired).all():
                raise ValueError('nonfinite formation target')
            for r,p in zip(self.robots[1:],desired): r.sp_global=p.copy()
            self.publish(self.robot1,tick,0)
            for index,r in enumerate(self.robots[1:],1):
                now=tick*self.dt+(index+1)*self.dt/len(self.robots)
                neighbors=[self.transport.receive(other.id,r.id,now)
                           for other in self.robots if other.id!=r.id]
                r.local_planner(np.array(self.obstacles+poses2polygons(neighbors)),self.params)
                self.publish(r,tick,index)
            positions=self.positions()
            if not all(np.isfinite(p).all() for p in positions.values()):
                raise ValueError('nonfinite local planner position')
            targets={r.id:r.sp_global.copy() for r in self.robots}
            self.oracle.observe(self.clock,positions,targets,self.obstacles,self.xy_goal,self.sp_index)
            self.trace.append(to_builtin(dict(tick=tick,time_s=self.clock,fsm='RUNNING',
                planning_state='LOCAL_APF',positions=positions,targets=targets,sp_index=self.sp_index)))
            if self.oracle.labels['drone_drone'] or self.oracle.labels['drone_obstacle']:
                return 'collision',''
            centroid=np.mean(list(positions.values()),axis=0)
            if self.sp_index<len(self.trajectory)-1 and np.linalg.norm(self.robot1.sp_global-centroid)<self.params.max_sp_dist:
                self.sp_index+=1
        self.time_step=self.config.max_ticks
        return 'timeout','mission tick budget exhausted'
