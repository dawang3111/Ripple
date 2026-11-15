import sys, os
import csv
import json
from os import makedirs
from time import strftime
import time
import math
import random
import numpy as np
from numpy.linalg import norm
import matplotlib
import matplotlib.pyplot as plt
from svgpath2mpl import parse_path
from matplotlib.path import Path
from shapely.geometry import Point, Polygon

# 批处理只保存图片，不需要 GUI 后端。
matplotlib.use('Agg')

# 保证能导入上级目录的依赖
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

from adaptive_swarm.scripts.layered_planner.tools import poses2polygons, init_fonts, draw_map, waypts2setpts, formation, \
    normalize
from adaptive_swarm.scripts.layered_planner.potential_fields import grid_map, combined_potential, meters2grid, \
    grid2meters
from adaptive_swarm.scripts.layered_planner.rrt import rrt_path, ShortenPath


def to_builtin(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: to_builtin(val) for key, val in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_builtin(item) for item in value]
    return value


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def safe_float(value, default=0.0):
    if value in ("", None):
        return default
    return float(value)


def random_fuzz_params(rng, latency_range, jitter_range, noise_std_range,
                       fault_start_range, fault_duration_range):
    latency_frames = int(rng.randint(latency_range[0], latency_range[1] + 1))
    jitter_frames = int(rng.randint(jitter_range[0], jitter_range[1] + 1))
    noise_std = round(float(rng.uniform(noise_std_range[0], noise_std_range[1])), 4)
    fault_start_step = int(rng.randint(fault_start_range[0], fault_start_range[1] + 1))
    fault_duration_steps = int(rng.randint(fault_duration_range[0], fault_duration_range[1] + 1))
    return build_fuzz_params(
        latency_frames, jitter_frames, noise_std,
        fault_start_step, fault_duration_steps,
        latency_range, jitter_range, noise_std_range,
    )


def build_fuzz_params(latency_frames, jitter_frames, noise_std,
                      fault_start_step, fault_duration_steps,
                      latency_range, jitter_range, noise_std_range):
    latency_frames = int(clamp(round(latency_frames), latency_range[0], latency_range[1]))
    jitter_frames = int(clamp(round(jitter_frames), jitter_range[0], jitter_range[1]))
    noise_std = round(float(clamp(noise_std, noise_std_range[0], noise_std_range[1])), 4)
    fuzz_strength = round(float(np.mean([
        latency_frames / latency_range[1] if latency_range[1] else 0.0,
        jitter_frames / jitter_range[1] if jitter_range[1] else 0.0,
        noise_std / noise_std_range[1] if noise_std_range[1] else 0.0,
    ])), 6)
    return {
        "latency_frames": latency_frames,
        "jitter_frames": jitter_frames,
        "noise_std": noise_std,
        "start_step": int(fault_start_step),
        "duration_steps": int(fault_duration_steps),
        "fuzz_strength": fuzz_strength,
    }


def mutate_fuzz_params(parent_params, rng, latency_range, jitter_range, noise_std_range,
                       fault_start_range, fault_duration_range, mutation_ratio):
    latency_span = max(1, latency_range[1] - latency_range[0])
    jitter_span = max(1, jitter_range[1] - jitter_range[0])
    noise_span = max(0.0001, noise_std_range[1] - noise_std_range[0])
    start_span = max(1, fault_start_range[1] - fault_start_range[0])
    duration_span = max(1, fault_duration_range[1] - fault_duration_range[0])

    latency_delta = rng.randint(-max(1, math.ceil(latency_span * mutation_ratio)),
                                max(1, math.ceil(latency_span * mutation_ratio)) + 1)
    jitter_delta = rng.randint(-max(1, math.ceil(jitter_span * mutation_ratio)),
                               max(1, math.ceil(jitter_span * mutation_ratio)) + 1)
    noise_delta = rng.uniform(-noise_span * mutation_ratio, noise_span * mutation_ratio)
    start_delta = rng.randint(-max(1, math.ceil(start_span * mutation_ratio)),
                              max(1, math.ceil(start_span * mutation_ratio)) + 1)
    duration_delta = rng.randint(-max(1, math.ceil(duration_span * mutation_ratio)),
                                 max(1, math.ceil(duration_span * mutation_ratio)) + 1)

    return build_fuzz_params(
        parent_params["latency_frames"] + latency_delta,
        parent_params["jitter_frames"] + jitter_delta,
        parent_params["noise_std"] + noise_delta,
        clamp(parent_params["start_step"] + start_delta, fault_start_range[0], fault_start_range[1]),
        clamp(parent_params["duration_steps"] + duration_delta,
              fault_duration_range[0], fault_duration_range[1]),
        latency_range, jitter_range, noise_std_range,
    )


def evaluate_badness(result, max_ticks, formation_reference):
    collision_score = 1.0 if result["status"] == "collision" else 0.0
    completion_time_score = clamp(safe_float(result.get("tick")) / max_ticks, 0.0, 1.0)
    formation_badness = clamp(
        safe_float(result.get("mean_formation_deviation")) / formation_reference,
        0.0, 1.0,
    )
    badness_score = round(
        0.55 * collision_score +
        0.25 * formation_badness +
        0.20 * completion_time_score,
        6,
    )
    formation_quality_score = round(1.0 - formation_badness, 6)
    return badness_score, formation_quality_score, completion_time_score, formation_badness


class Params:
    def __init__(self):
        self.animate_rrt = 0
        self.visualize = 1
        self.maxiters = 1000
        self.goal_prob = 0.05
        self.minDistGoal = 0.25
        self.extension = 0.8
        self.world_bounds_x = [-2.5, 2.5]
        self.world_bounds_y = [-2.5, 2.5]
        self.drone_vel = 4.0
        self.ViconRate = 100

        # 物理与安全参数
        self.influence_radius = 0.10  # APF 感知半径
        self.crash_uav_dist = 0.1  # 机机碰撞致死半径
        self.inflation_radius = 0.15 # 宏观 RRT 障碍物膨胀厚度

        self.goal_tolerance = 0.05
        self.num_robots = 4
        self.interrobots_dist = 0.3
        self.max_sp_dist = 0.2 * self.drone_vel
        self.dir = 0


class Robot:
    def __init__(self, id):
        self.id = id
        self.sp = np.array([0, 0])
        self.sp_global = np.array([0, 0])
        self.route = np.array([self.sp])
        self.U_a = 0
        self.U_r = 0
        self.U = 0
        self.leader = False

    def local_planner(self, obstacles, params: Params):
        obstacles_grid = grid_map(obstacles)
        self.U, self.U_a, self.U_r = combined_potential(obstacles_grid, self.sp_global, params.influence_radius)
        [gy, gx] = np.gradient(-self.U)
        iy, ix = np.array(meters2grid(self.sp), dtype=int)
        w = 20
        ax = np.mean(gx[ix - int(w / 2): ix + int(w / 2), iy - int(w / 2): iy + int(w / 2)])
        ay = np.mean(gy[ix - int(w / 2): ix + int(w / 2), iy - int(w / 2): iy + int(w / 2)])

        self.V = params.drone_vel * np.array([ax, ay])
        dt = 0.01 * params.drone_vel / norm([ax, ay]) if norm([ax, ay]) != 0 else 0.01
        self.sp = self.sp + dt * np.array([ax, ay])
        self.route = np.vstack([self.route, self.sp])


class CombinedCommNetwork:
    def __init__(self, fuzz_params, rng):
        self.latency_frames = int(fuzz_params.get("latency_frames", 0))
        self.jitter_frames = int(fuzz_params.get("jitter_frames", 0))
        self.noise_std = float(fuzz_params.get("noise_std", 0.0))
        self.start_step = int(fuzz_params.get("start_step", 0))
        self.duration_steps = int(fuzz_params.get("duration_steps", 0))
        self.rng = rng

    def in_fault_window(self, current_time):
        return self.start_step <= current_time <= self.start_step + self.duration_steps

    def get_perceived_position(self, receiver, sender, current_time):
        if not self.in_fault_window(current_time):
            return sender.sp.copy(), {
                "receiver_id": receiver.id,
                "sender_id": sender.id,
                "fault_active": False,
                "actual_delay_frames": 0,
                "latency_frames": 0,
                "jitter_sample_frames": 0,
                "noise": np.array([0.0, 0.0]),
                "true_position": sender.sp.copy(),
                "perceived_position": sender.sp.copy(),
            }

        jitter_sample = (
            int(self.rng.randint(0, self.jitter_frames + 1))
            if self.jitter_frames > 0 else 0
        )
        actual_delay = self.latency_frames + jitter_sample
        history_index = max(0, current_time - actual_delay)
        base_sp = sender.route[history_index] if history_index < len(sender.route) else sender.sp
        noise = self.rng.normal(0, self.noise_std, 2) if self.noise_std > 0 else np.array([0.0, 0.0])
        perceived_sp = base_sp + noise

        return perceived_sp, {
            "receiver_id": receiver.id,
            "sender_id": sender.id,
            "fault_active": True,
            "actual_delay_frames": actual_delay,
            "latency_frames": self.latency_frames,
            "jitter_sample_frames": jitter_sample,
            "noise": noise,
            "true_position": sender.sp.copy(),
            "base_delayed_position": np.array(base_sp).copy(),
            "perceived_position": perceived_sp.copy(),
        }


class BaselineSimulation:
    def __init__(self, episode_id, batch_folder, seed=None, fuzz_params=None):
        self.start_time = time.time()
        self.time_step = 0
        self.episode_id = episode_id
        self.save_folder_path = batch_folder
        self.seed = int(seed if seed is not None else time.time())
        self.rng = np.random.RandomState(self.seed)
        np.random.seed(self.seed)
        random.seed(self.seed)
        self.fuzz_params = fuzz_params or {}
        self.comm_net = CombinedCommNetwork(self.fuzz_params, self.rng)

        init_fonts(small=12, medium=16, big=26)
        self.params = Params()
        self.xy_start = np.array([1.2, 1.25])
        self.xy_goal = np.array([2.2, -2.2])

        self.obstacles = [
            np.array([[-1.0, 0], [2.5, 0.], [2.5, 0.3], [-1.0, 0.3]]),
            np.array([[0.5, 2.0], [0.8, 2.0], [0.8, 2.5], [0.5, 2.5]]),
            np.array([[0.0, -2.5], [0.3, -2.5], [0.3, -1.6], [0.0, -1.6]]),
            np.array([[-1.3, 0.3], [-0.7, 0.3], [-0.7, 0.6], [-1.3, 0.6]]),
            np.array([[-1.0, 1.8], [-0.7, 1.8], [-0.7, 2.5], [-1.0, 2.5]]),
            np.array([[-2.0, -2.47], [-1.4, -2.47], [-1.4, -1.2], [-2.0, -1.2]]),
            np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.47], [-2.5, -2.47]]),
            np.array([[-2.5, 2.47], [2.5, 2.47], [2.5, 2.5], [-2.5, 2.5]]),
            np.array([[-2.5, -2.47], [-2.47, -2.47], [-2.47, 2.47], [-2.5, 2.47]]),
            np.array([[2.47, -2.47], [2.5, -2.47], [2.5, 2.47], [2.47, 2.47]]),
        ]
        self.obstacle_polygons = [Polygon(obs) for obs in self.obstacles]

        self.inflated_obstacles = []
        for obs in self.obstacles:
            poly = Polygon(obs)
            inflated_poly = poly.buffer(self.params.inflation_radius, join_style=2)
            coords = np.array(inflated_poly.exterior.coords)
            self.inflated_obstacles.append(coords)

        self.robots = [Robot(i + 1) for i in range(self.params.num_robots)]
        self.robot1 = self.robots[0]
        self.robot1.leader = True

        self.rrt_plan_seconds = 0.0
        self.global_planning_failed = 0
        self.min_interrobot_distance = math.inf
        self.min_obstacle_distance = math.inf
        self.total_path_length = 0.0
        self.formation_error_sum = 0.0
        self.formation_error_max = 0.0
        self.formation_error_count = 0
        self.tick_history = []

    def build_result(self, status, collision_type="", tick=None, final_distance=None,
                     collision_log="", detail_log_json="", crash_image="", note=""):
        runtime_seconds = time.time() - self.start_time
        mean_formation_deviation = (
            self.formation_error_sum / self.formation_error_count
            if self.formation_error_count else 0.0
        )
        return {
            "episode": self.episode_id,
            "status": status,
            "success": status == "success",
            "collision": status == "collision",
            "collision_type": collision_type or "",
            "tick": self.time_step if tick is None else tick,
            "final_distance_to_goal": "" if final_distance is None else round(float(final_distance), 6),
            "runtime_seconds": round(runtime_seconds, 6),
            "global_planning_failed": self.global_planning_failed,
            "rrt_plan_seconds": round(self.rrt_plan_seconds, 6),
            "min_interrobot_distance": self.clean_metric(self.min_interrobot_distance),
            "min_obstacle_distance": self.clean_metric(self.min_obstacle_distance),
            "mean_formation_deviation": round(float(mean_formation_deviation), 6),
            "max_formation_deviation": round(float(self.formation_error_max), 6),
            "total_path_length": round(float(self.total_path_length), 6),
            "seed": self.seed,
            "latency_frames": self.fuzz_params.get("latency_frames", ""),
            "jitter_frames": self.fuzz_params.get("jitter_frames", ""),
            "noise_std": self.fuzz_params.get("noise_std", ""),
            "fault_start_step": self.fuzz_params.get("start_step", ""),
            "fault_duration_steps": self.fuzz_params.get("duration_steps", ""),
            "fuzz_strength": self.fuzz_params.get("fuzz_strength", ""),
            "collision_log": collision_log,
            "detail_log_json": detail_log_json,
            "crash_image": crash_image,
            "note": note,
        }

    def clean_metric(self, value):
        if value == math.inf:
            return ""
        return round(float(value), 6)

    def robot_pair_min_distance(self):
        min_distance = math.inf
        min_pair = None
        for i in range(len(self.robots)):
            for j in range(i + 1, len(self.robots)):
                distance = norm(self.robots[i].sp - self.robots[j].sp)
                if distance < min_distance:
                    min_distance = distance
                    min_pair = (self.robots[i].id, self.robots[j].id)
        return min_distance, min_pair

    def robot_obstacle_min_distance(self):
        min_distance = math.inf
        min_robot_id = None
        min_obstacle_index = None
        for robot in self.robots:
            point = Point(float(robot.sp[0]), float(robot.sp[1]))
            for obs_index, polygon in enumerate(self.obstacle_polygons):
                distance = polygon.distance(point)
                if distance < min_distance:
                    min_distance = distance
                    min_robot_id = robot.id
                    min_obstacle_index = obs_index
        return min_distance, min_robot_id, min_obstacle_index

    def potential_at_robot(self, robot):
        if not isinstance(robot.U, np.ndarray) or robot.U.size == 0:
            return {}

        iy, ix = np.array(meters2grid(robot.sp), dtype=int)
        ix = int(np.clip(ix, 0, robot.U.shape[0] - 1))
        iy = int(np.clip(iy, 0, robot.U.shape[1] - 1))
        values = {"total": float(robot.U[ix, iy])}
        if isinstance(robot.U_a, np.ndarray) and robot.U_a.size:
            values["attractive"] = float(robot.U_a[ix, iy])
        if isinstance(robot.U_r, np.ndarray) and robot.U_r.size:
            values["repulsive"] = float(robot.U_r[ix, iy])
        return values

    def record_tick_state(self, sp_ind, traj_global, previous_positions, followers_sp_global,
                          communication_snapshot):
        step_distances = [
            float(norm(robot.sp - previous_positions[index]))
            for index, robot in enumerate(self.robots)
        ]
        self.total_path_length += sum(step_distances)

        min_interrobot_distance, min_pair = self.robot_pair_min_distance()
        min_obstacle_distance, min_obs_robot_id, min_obstacle_index = self.robot_obstacle_min_distance()
        self.min_interrobot_distance = min(self.min_interrobot_distance, min_interrobot_distance)
        self.min_obstacle_distance = min(self.min_obstacle_distance, min_obstacle_distance)

        formation_errors = []
        for index, desired_pos in enumerate(followers_sp_global):
            follower = self.robots[index + 1]
            formation_errors.append(float(norm(follower.sp - desired_pos)))
        mean_formation_error = float(np.mean(formation_errors)) if formation_errors else 0.0
        max_formation_error = float(np.max(formation_errors)) if formation_errors else 0.0
        self.formation_error_sum += mean_formation_error
        self.formation_error_count += 1
        self.formation_error_max = max(self.formation_error_max, max_formation_error)

        centroid = sum([robot.sp for robot in self.robots]) / len(self.robots)
        self.tick_history.append({
            "tick": self.time_step,
            "sp_index": int(sp_ind),
            "leader_target": traj_global[sp_ind, :],
            "centroid": centroid,
            "distance_to_goal": float(norm(self.robot1.sp - self.xy_goal)),
            "min_interrobot_distance": float(min_interrobot_distance),
            "min_interrobot_pair": min_pair,
            "min_obstacle_distance": float(min_obstacle_distance),
            "min_obstacle_robot_id": min_obs_robot_id,
            "min_obstacle_index": min_obstacle_index,
            "mean_formation_deviation": mean_formation_error,
            "max_formation_deviation": max_formation_error,
            "total_path_length_so_far": float(self.total_path_length),
            "communication": communication_snapshot,
            "robots": [
                {
                    "id": robot.id,
                    "position": robot.sp,
                    "target": robot.sp_global,
                    "step_distance": step_distances[index],
                    "velocity": getattr(robot, "V", np.array([0.0, 0.0])),
                    "velocity_norm": float(norm(getattr(robot, "V", np.array([0.0, 0.0])))),
                    "potential_at_position": self.potential_at_robot(robot),
                    "route_length": len(robot.route),
                    "leader": robot.leader,
                }
                for index, robot in enumerate(self.robots)
            ],
        })

    def write_collision_detail_json(self, collision_type, collision_info, P, traj_global, sp_ind):
        detail_file = os.path.join(self.save_folder_path, f"Ep{self.episode_id}_collision_detail.json")
        collision_detail = {
            "episode": self.episode_id,
            "seed": self.seed,
            "fuzz_params": self.fuzz_params,
            "collision_type": collision_type,
            "collision_tick": self.time_step,
            "collision_info": self.describe_collision(collision_type, collision_info),
            "params": vars(self.params),
            "start": self.xy_start,
            "goal": self.xy_goal,
            "obstacles": self.obstacles,
            "inflated_obstacles": self.inflated_obstacles,
            "rrt_path": P,
            "trajectory_setpoints": traj_global,
            "current_sp_index": int(sp_ind),
            "summary_metrics": {
                "runtime_seconds_until_collision": time.time() - self.start_time,
                "rrt_plan_seconds": self.rrt_plan_seconds,
                "min_interrobot_distance": self.min_interrobot_distance,
                "min_obstacle_distance": self.min_obstacle_distance,
                "mean_formation_deviation": (
                    self.formation_error_sum / self.formation_error_count
                    if self.formation_error_count else 0.0
                ),
                "max_formation_deviation": self.formation_error_max,
                "total_path_length": self.total_path_length,
            },
            "robot_routes": {
                str(robot.id): robot.route
                for robot in self.robots
            },
            "tick_history": self.tick_history,
        }
        with open(detail_file, "w", encoding="utf-8") as f:
            json.dump(to_builtin(collision_detail), f, ensure_ascii=False, indent=2)
        return detail_file

    def describe_collision(self, collision_type, collision_info):
        if collision_type == "drone_drone":
            r1, r2 = collision_info
            return {
                "drone_1": {"id": r1.id, "position": r1.sp},
                "drone_2": {"id": r2.id, "position": r2.sp},
                "distance": float(norm(r1.sp - r2.sp)),
            }
        if collision_type == "drone_obs":
            robot, obs = collision_info
            return {
                "drone": {"id": robot.id, "position": robot.sp},
                "obstacle_vertices": obs,
            }
        return {}

    def check_collisions(self):
        """ 检测无人机互撞或撞墙 (依然使用真实的墙壁判定) """
        crash_uav_dist = self.params.crash_uav_dist

        for i in range(len(self.robots)):
            for j in range(i + 1, len(self.robots)):
                if norm(self.robots[i].sp - self.robots[j].sp) < crash_uav_dist:
                    return "drone_drone", (self.robots[i], self.robots[j])

        for robot in self.robots:
            for obs in self.obstacles:
                path = Path(obs)
                if path.contains_point(robot.sp):
                    return "drone_obs", (robot, obs)

        return None, None

    def start(self):
        try:
            rrt_start_time = time.time()
            P_long = rrt_path(self.inflated_obstacles, self.xy_start, self.xy_goal, self.params)
            P = ShortenPath(P_long, self.inflated_obstacles, smoothiters=50)
            self.rrt_plan_seconds = time.time() - rrt_start_time
            traj_global = waypts2setpts(P, self.params)
            P = np.vstack([P, self.xy_start])
        except Exception as e:
            self.global_planning_failed = 1
            print(f"  [提示] 第 {self.episode_id} 轮 RRT 寻路超时 (可能由于膨胀导致通道过窄)。跳过...")
            return self.build_result("rrt_failed", note=str(e))

        sp_ind = 0
        self.robot1.route = np.array([traj_global[0, :]])
        self.robot1.sp = self.robot1.route[-1, :]

        followers_sp = formation(self.params.num_robots, leader_des=self.robot1.sp, v=np.array([0, -1]),
                                 l=self.params.interrobots_dist)
        for i in range(len(followers_sp)):
            self.robots[i + 1].sp = followers_sp[i]
            self.robots[i + 1].route = np.array([followers_sp[i]])

        while self.time_step < 300:
            previous_positions = [robot.sp.copy() for robot in self.robots]
            dist_to_goal = norm(self.robot1.sp - self.xy_goal)
            if dist_to_goal < self.params.goal_tolerance:
                # <--- 优化：成功到达终点时不再绘图保存，直接静默退出 --->
                return self.build_result("success", final_distance=dist_to_goal)

            self.robot1.sp_global = traj_global[sp_ind, :]
            self.robot1.local_planner(np.array(self.obstacles), self.params)

            followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global,
                                            v=normalize(self.robot1.sp_global - self.robot1.sp),
                                            l=self.params.interrobots_dist)
            for j in range(len(followers_sp_global)):
                self.robots[j + 1].sp_global = followers_sp_global[j]

            communication_snapshot = []
            for p in range(len(followers_sp)):
                receiver_robot = self.robots[p + 1]
                perceived_neighbors_sp = []

                for other_robot in self.robots:
                    if other_robot.id == receiver_robot.id:
                        continue
                    perceived_sp, comm_event = self.comm_net.get_perceived_position(
                        receiver_robot, other_robot, self.time_step
                    )
                    perceived_neighbors_sp.append(perceived_sp)
                    communication_snapshot.append(comm_event)

                robots_obstacles = poses2polygons(perceived_neighbors_sp)
                obstacles1 = np.array(self.obstacles + robots_obstacles)
                receiver_robot.local_planner(obstacles1, self.params)
                followers_sp[p] = receiver_robot.sp

            self.record_tick_state(
                sp_ind, traj_global, previous_positions,
                followers_sp_global, communication_snapshot
            )

            collision_type, collision_info = self.check_collisions()
            if collision_type:
                detail_json_file = self.write_collision_detail_json(
                    collision_type, collision_info, P, traj_global, sp_ind
                )
                info_file = os.path.join(self.save_folder_path, f"Ep{self.episode_id}_collision_info.txt")
                with open(info_file, "w", encoding="utf-8") as f:
                    f.write(f"轮次 (Episode): {self.episode_id}\n")
                    f.write(f"随机种子 (Seed): {self.seed}\n")
                    f.write(f"碰撞发生时间步 (Tick): {self.time_step}\n")
                    f.write(f"碰撞类型: {'无人机相撞' if collision_type == 'drone_drone' else '无人机撞墙'}\n\n")
                    f.write("【综合通信故障参数】\n")
                    for key, value in self.fuzz_params.items():
                        f.write(f"  {key}: {value}\n")
                    f.write("\n")
                    if collision_type == "drone_drone":
                        r1, r2 = collision_info
                        f.write("【碰撞实体】\n")
                        f.write(f"  无人机 {r1.id}: [{r1.sp[0]:.4f}, {r1.sp[1]:.4f}]\n")
                        f.write(f"  无人机 {r2.id}: [{r2.sp[0]:.4f}, {r2.sp[1]:.4f}]\n")
                        f.write(f"  两机距离: {norm(r1.sp - r2.sp):.4f}\n\n")
                    elif collision_type == "drone_obs":
                        r, obs = collision_info
                        f.write("【碰撞实体】\n")
                        f.write(f"  无人机 {r.id}: [{r.sp[0]:.4f}, {r.sp[1]:.4f}]\n")
                        f.write(f"  障碍物顶点: {np.array2string(obs, precision=4)}\n\n")
                    f.write("【碰撞瞬间所有无人机坐标】\n")
                    for r in self.robots:
                        f.write(f"  无人机 {r.id}: [{r.sp[0]:.4f}, {r.sp[1]:.4f}]\n")
                    f.write("\n【全过程关键指标】\n")
                    f.write(f"  全过程最小机间距: {self.min_interrobot_distance:.6f}\n")
                    f.write(f"  全过程最小机障距离: {self.min_obstacle_distance:.6f}\n")
                    f.write(f"  平均队形偏离: {self.formation_error_sum / self.formation_error_count:.6f}\n")
                    f.write(f"  最大队形偏离: {self.formation_error_max:.6f}\n")
                    f.write(f"  总路径长度: {self.total_path_length:.6f}\n")
                    f.write(f"\n【详细 JSON 事故包】\n  {os.path.basename(detail_json_file)}\n")

                plt.close(1)
                plt.figure(num=1, figsize=(10, 10))
                plt.cla()
                self.visualize2D(traj_global, sp_ind, P)
                frame_path = os.path.join(self.save_folder_path,
                                          f"Ep{self.episode_id}_CRASH_tick_{self.time_step:03d}.jpg")
                plt.savefig(frame_path, dpi=100)
                plt.close('all')

                return self.build_result(
                    "collision",
                    collision_type=collision_type,
                    final_distance=norm(self.robot1.sp - self.xy_goal),
                    collision_log=os.path.basename(info_file),
                    detail_log_json=os.path.basename(detail_json_file),
                    crash_image=os.path.basename(frame_path),
                )

            centroid = sum([robot.sp for robot in self.robots]) / len(self.robots)
            if sp_ind < traj_global.shape[0] - 1 and norm(self.robot1.sp_global - centroid) < self.params.max_sp_dist:
                sp_ind += 1
            self.time_step += 1

        return self.build_result(
            "timeout",
            tick=self.time_step,
            final_distance=norm(self.robot1.sp - self.xy_goal),
            note="max time_step reached before goal",
        )

    def visualize2D(self, traj_global, sp_ind, P):
        """ 绘图函数：增加了安全气囊的可视化 """
        smiley = parse_path(
            """M458 2420 c-215 -38 -368 -257 -329 -469 34 -182 175 -314 354 -329 l57 -4 0 45 0 44 -42 7 c-101 16 -187 79 -236 171 -37 69 -38 187 -4 257 30 60 90 120 150 150 70 34 188 33 258 -4 89 -47 153 -136 169 -235 l7 -43 50 0 51 0 -6 59 c-13 147 -124 285 -268 334 -60 20 -152 28 -211 17z M1940 2417 c-172 -39 -302 -181 -317 -347 l-6 -60 51 0 50 0 12 52 c14 70 49 126 110 181 118 106 284 100 399 -14 64 -64 86 -120 86 -214 0 -67 -5 -88 -27 -130 -49 -92 -135 -155 -236 -171 l-42 -7 0 -49 0 -50 58 4 c115 8 242 91 306 200 36 61 59 177 51 248 -30 244 -260 410 -495 357z M506 2038 c-9 -12 -16 -41 -16 -64 0 -39 11 -56 158 -240 87 -110 161 -205 166 -212 5 -9 10 -382 6 -494 0 -3 -74 -97 -165 -208 l-165 -202 0 -52 c0 -68 18 -86 86 -86 40 0 55 5 80 28 17 15 112 89 211 166 l180 138 239 0 239 -1 209 -165 c203 -162 210 -166 256 -166 60 0 80 20 80 81 0 43 -8 55 -170 264 l-170 220 0 230 c0 202 2 233 18 257 9 15 86 108 170 208 l152 180 0 54 c0 65 -19 86 -76 86 -36 0 -58 -15 -234 -151 -107 -83 -205 -158 -217 -166 -19 -12 -67 -15 -260 -15 l-238 1 -209 165 -209 166 -53 0 c-43 0 -56 -4 -68 -22z M415 926 c-199 -63 -321 -258 -286 -457 31 -179 161 -309 340 -340 75 -14 171 1 248 37 116 55 209 188 220 314 l6 60 -49 0 -49 0 -17 -70 c-20 -84 -62 -147 -123 -188 -154 -102 -363 -44 -446 124 -35 72 -34 189 3 259 49 92 135 155 236 171 l42 7 0 48 0 49 -42 -1 c-24 0 -61 -6 -83 -13z M2020 882 l0 -50 43 -7 c99 -16 188 -80 235 -169 22 -43 27 -64 27 -131 0 -98 -23 -155 -90 -219 -177 -172 -471 -67 -511 183 l-7 41 -50 0 -50 0 6 -60 c11 -126 102 -257 218 -314 251 -123 542 26 590 303 39 221 -132 448 -351 468 l-60 6 0 -51z""")
        smiley.vertices -= smiley.vertices.mean(axis=0)

        for obs in self.inflated_obstacles:
            plt.fill(obs[:, 0], obs[:, 1], color='cyan', alpha=0.15, zorder=1)

        draw_map(self.obstacles)

        for i, robot in enumerate(self.robots):
            plt.plot(robot.sp[0], robot.sp[1], marker=smiley, color='blue', markersize=10, zorder=15)
            plt.annotate(str(i + 1), xy=(robot.sp[0], robot.sp[1]), xytext=(robot.sp[0] + 0.05, robot.sp[1] + 0.05))

            theta = np.linspace(0, 2 * np.pi, 100)
            x = np.cos(theta) * self.params.influence_radius + robot.sp[0]
            y = np.sin(theta) * self.params.influence_radius + robot.sp[1]
            plt.plot(x, y, color='red', alpha=0.5, zorder=10)
            plt.fill(x, y, color='green', alpha=0.1, zorder=10)

        plt.plot(P[:, 0], P[:, 1], linewidth=3, color='orange', zorder=5)
        plt.plot(traj_global[sp_ind, 0], traj_global[sp_ind, 1], marker='o', color='blue', markersize=7)
        plt.plot(self.xy_start[0], self.xy_start[1], marker='o', color='red', markersize=5)
        plt.plot(self.xy_goal[0], self.xy_goal[1], marker='o', color='green', markersize=5)

        plt.text(2.0, 2.3, f"Ep: {self.episode_id} | Tick: {self.time_step}", fontsize=12,
                 bbox=dict(facecolor='white', alpha=0.8))
        plt.text(
            -2.5, 2.3,
            "Combined Fuzz: L=%s J=%s N=%.3f" % (
                self.fuzz_params.get("latency_frames", 0),
                self.fuzz_params.get("jitter_frames", 0),
                self.fuzz_params.get("noise_std", 0.0),
            ),
            color='red',
            fontsize=11,
            weight='bold',
            bbox=dict(facecolor='white', alpha=0.8),
        )

        plt.xlim([-2.6, 2.6])
        plt.ylim([-2.6, 2.6])


if __name__ == '__main__':
    FUZZ_MODE = "random"  # "heuristic" or "random"
    TOTAL_EPISODES = 400 
    WARMUP_RATIO = 0.20
    RANDOM_EXPLORATION_PROB = 0.15
    MUTATION_RATIO = 0.10
    SEED_POOL_BADNESS_THRESHOLD = 0.35
    FORMATION_REFERENCE = 1.0
    FORMATION_QUALITY_POOL_THRESHOLD = 0.55
    COMPLETION_TIME_
    POOL_THRESHOLD = 0.75
    SEED_POOL_MAX_SIZE = 100
    MAX_TICKS = 300
    LATENCY_RANGE = (0, 15)
    JITTER_RANGE = (0, 10)
    NOISE_STD_RANGE = (0.0, 0.15)
    FAULT_START_RANGE = (20, 150)
    FAULT_DURATION_RANGE = (20, 100)

    temp_params = Params()
    inf_r = temp_params.influence_radius
    cra_r = temp_params.crash_uav_dist
    infl_r = temp_params.inflation_radius

    master_seed = int(time.time())
    master_rng = np.random.RandomState(master_seed)
    if FUZZ_MODE not in ("heuristic", "random"):
        raise ValueError('FUZZ_MODE must be "heuristic" or "random"')
    warmup_episodes = max(1, int(math.ceil(TOTAL_EPISODES * WARMUP_RATIO)))

    folder_time = strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
    batch_res_path = f'data_collection/{FUZZ_MODE}_fuzz_inf{inf_r}_cra{cra_r}_cspace{infl_r}_{folder_time}/'
    makedirs(batch_res_path, exist_ok=True)

    print(f"[{strftime('%H:%M:%S')}] {FUZZ_MODE} 综合通信故障 Fuzz 开始，保存至: {batch_res_path}")
    print("-" * 50)

    total_collisions = 0
    total_successes = 0
    total_rrt_failed = 0
    total_timeouts = 0
    crashed_episodes = []
    collision_types = {"drone_drone": 0, "drone_obs": 0}
    episode_results = []
    seed_pool = []
    seed_pool_records = []
    heuristic_events = []
    next_pool_id = 1

    for ep in range(1, TOTAL_EPISODES + 1):
        print(f"正在推演第 {ep:02d}/{TOTAL_EPISODES} 轮...", end="\r")

        episode_seed = int(master_rng.randint(1, 100000000))
        ep_rng = np.random.RandomState(episode_seed)
        parent_pool_id = ""
        mutation_applied = False

        if FUZZ_MODE == "random":
            phase = "random"
            seed_source = "random"
            fuzz_params = random_fuzz_params(
                ep_rng, LATENCY_RANGE, JITTER_RANGE, NOISE_STD_RANGE,
                FAULT_START_RANGE, FAULT_DURATION_RANGE,
            )
        else:
            phase = "warmup" if ep <= warmup_episodes else "guided"

        if FUZZ_MODE == "heuristic" and (
            phase == "warmup" or not seed_pool or master_rng.rand() < RANDOM_EXPLORATION_PROB
        ):
            seed_source = "random_warmup" if phase == "warmup" else "random_explore"
            fuzz_params = random_fuzz_params(
                ep_rng, LATENCY_RANGE, JITTER_RANGE, NOISE_STD_RANGE,
                FAULT_START_RANGE, FAULT_DURATION_RANGE,
            )
        elif FUZZ_MODE == "heuristic":
            seed_source = "seed_pool_mutation"
            weights = np.array([entry["badness_score"] + 0.01 for entry in seed_pool], dtype=float)
            weights = weights / weights.sum()
            parent_index = int(master_rng.choice(len(seed_pool), p=weights))
            parent_seed = seed_pool[parent_index]
            parent_pool_id = parent_seed["pool_id"]
            fuzz_params = mutate_fuzz_params(
                parent_seed["fuzz_params"], ep_rng,
                LATENCY_RANGE, JITTER_RANGE, NOISE_STD_RANGE,
                FAULT_START_RANGE, FAULT_DURATION_RANGE,
                MUTATION_RATIO,
            )
            mutation_applied = True

        sim = BaselineSimulation(
            episode_id=ep,
            batch_folder=batch_res_path,
            seed=episode_seed,
            fuzz_params=fuzz_params,
        )
        result = sim.start()

        badness_score, formation_quality_score, completion_time_score, formation_badness = evaluate_badness(
            result, MAX_TICKS, FORMATION_REFERENCE
        )
        entered_seed_pool = (
            result["status"] == "collision" or
            badness_score >= SEED_POOL_BADNESS_THRESHOLD or
            formation_quality_score <= FORMATION_QUALITY_POOL_THRESHOLD or
            completion_time_score >= COMPLETION_TIME_POOL_THRESHOLD
        ) if FUZZ_MODE == "heuristic" else False
        seed_pool_reason = []
        if FUZZ_MODE == "random":
            seed_pool_reason.append("random_mode_disabled")
        elif result["status"] == "collision":
            seed_pool_reason.append("collision")
        if FUZZ_MODE == "heuristic" and badness_score >= SEED_POOL_BADNESS_THRESHOLD:
            seed_pool_reason.append("badness_threshold")
        if FUZZ_MODE == "heuristic" and formation_quality_score <= FORMATION_QUALITY_POOL_THRESHOLD:
            seed_pool_reason.append("low_formation_quality")
        if FUZZ_MODE == "heuristic" and completion_time_score >= COMPLETION_TIME_POOL_THRESHOLD:
            seed_pool_reason.append("slow_completion")
        if not seed_pool_reason:
            seed_pool_reason.append("not_added")

        result.update({
            "heuristic_phase": phase,
            "seed_source": seed_source,
            "parent_pool_id": parent_pool_id,
            "mutation_applied": mutation_applied,
            "badness_score": badness_score,
            "formation_quality_score": formation_quality_score,
            "completion_time_score": round(completion_time_score, 6),
            "formation_badness": round(formation_badness, 6),
            "entered_seed_pool": entered_seed_pool,
            "seed_pool_reason": "|".join(seed_pool_reason),
            "pool_id": "",
        })

        if entered_seed_pool:
            pool_entry = {
                "pool_id": next_pool_id,
                "added_episode": ep,
                "added_phase": phase,
                "seed_source": seed_source,
                "parent_pool_id": parent_pool_id,
                "mutation_applied": mutation_applied,
                "fuzz_params": fuzz_params.copy(),
                "badness_score": badness_score,
                "formation_quality_score": formation_quality_score,
                "completion_time_score": round(completion_time_score, 6),
                "status": result["status"],
                "collision_type": result["collision_type"],
                "tick": result["tick"],
                "runtime_seconds": result["runtime_seconds"],
                "mean_formation_deviation": result["mean_formation_deviation"],
                "max_formation_deviation": result["max_formation_deviation"],
                "min_interrobot_distance": result["min_interrobot_distance"],
                "min_obstacle_distance": result["min_obstacle_distance"],
                "seed": episode_seed,
                "reason": "|".join(seed_pool_reason),
            }
            seed_pool.append(pool_entry)
            seed_pool_records.append(pool_entry.copy())
            result["pool_id"] = next_pool_id
            next_pool_id += 1
            seed_pool.sort(key=lambda item: item["badness_score"], reverse=True)
            seed_pool = seed_pool[:SEED_POOL_MAX_SIZE]

        heuristic_events.append({
            "episode": ep,
            "phase": phase,
            "seed_source": seed_source,
            "parent_pool_id": parent_pool_id,
            "mutation_applied": mutation_applied,
            "entered_seed_pool": entered_seed_pool,
            "seed_pool_reason": "|".join(seed_pool_reason),
            "pool_size_after_episode": len(seed_pool),
            "badness_score": badness_score,
            "formation_quality_score": formation_quality_score,
            "completion_time_score": round(completion_time_score, 6),
            "latency_frames": fuzz_params["latency_frames"],
            "jitter_frames": fuzz_params["jitter_frames"],
            "noise_std": fuzz_params["noise_std"],
            "fault_start_step": fuzz_params["start_step"],
            "fault_duration_steps": fuzz_params["duration_steps"],
            "fuzz_strength": fuzz_params["fuzz_strength"],
        })

        episode_results.append(result)

        if result["status"] == "success":
            total_successes += 1
        elif result["status"] == "collision":
            total_collisions += 1
            crashed_episodes.append(ep)
            c_type = result["collision_type"]
            collision_types[c_type] += 1
        elif result["status"] == "rrt_failed":
            total_rrt_failed += 1
        elif result["status"] == "timeout":
            total_timeouts += 1

    print(" " * 50, end="\r")

    csv_file = os.path.join(batch_res_path, "results.csv")
    csv_columns = [
        "row_type",
        "episode",
        "status",
        "success",
        "collision",
        "collision_type",
        "tick",
        "final_distance_to_goal",
        "runtime_seconds",
        "global_planning_failed",
        "rrt_plan_seconds",
        "min_interrobot_distance",
        "min_obstacle_distance",
        "mean_formation_deviation",
        "max_formation_deviation",
        "total_path_length",
        "seed",
        "latency_frames",
        "jitter_frames",
        "noise_std",
        "fault_start_step",
        "fault_duration_steps",
        "fuzz_strength",
        "heuristic_phase",
        "seed_source",
        "parent_pool_id",
        "mutation_applied",
        "badness_score",
        "formation_quality_score",
        "completion_time_score",
        "formation_badness",
        "entered_seed_pool",
        "seed_pool_reason",
        "pool_id",
        "collision_log",
        "detail_log_json",
        "crash_image",
        "note",
        "total_episodes",
        "success_count",
        "collision_count",
        "drone_drone_collision_count",
        "drone_obs_collision_count",
        "rrt_failed_count",
        "timeout_count",
        "collision_rate",
        "success_rate",
        "influence_radius",
        "crash_uav_dist",
        "inflation_radius",
    ]

    numeric_summary_fields = [
        "runtime_seconds",
        "rrt_plan_seconds",
        "min_interrobot_distance",
        "min_obstacle_distance",
        "mean_formation_deviation",
        "max_formation_deviation",
        "total_path_length",
        "latency_frames",
        "jitter_frames",
        "noise_std",
        "fault_duration_steps",
        "fuzz_strength",
        "badness_score",
        "formation_quality_score",
        "completion_time_score",
        "formation_badness",
    ]

    with open(csv_file, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=csv_columns)
        writer.writeheader()
        for result in episode_results:
            row = {key: "" for key in csv_columns}
            row.update(result)
            row["row_type"] = "EPISODE"
            writer.writerow(row)

        summary_row = {key: "" for key in csv_columns}
        for field in numeric_summary_fields:
            values = [
                float(result[field])
                for result in episode_results
                if result.get(field) not in ("", None)
            ]
            summary_row[field] = round(float(np.mean(values)), 6) if values else ""
        summary_row.update({
            "row_type": "SUMMARY",
            "status": "summary",
            "global_planning_failed": total_rrt_failed,
            "total_episodes": TOTAL_EPISODES,
            "success_count": total_successes,
            "collision_count": total_collisions,
            "drone_drone_collision_count": collision_types["drone_drone"],
            "drone_obs_collision_count": collision_types["drone_obs"],
            "rrt_failed_count": total_rrt_failed,
            "timeout_count": total_timeouts,
            "collision_rate": round(total_collisions / TOTAL_EPISODES, 6),
            "success_rate": round(total_successes / TOTAL_EPISODES, 6),
            "influence_radius": inf_r,
            "crash_uav_dist": cra_r,
            "inflation_radius": infl_r,
            "note": (
                f"master_seed={master_seed}; warmup={warmup_episodes}/{TOTAL_EPISODES}; "
                f"fuzz_mode={FUZZ_MODE}; "
                f"random_explore_prob={RANDOM_EXPLORATION_PROB}; mutation_ratio={MUTATION_RATIO}; "
                f"seed_pool_threshold={SEED_POOL_BADNESS_THRESHOLD}; "
                f"formation_quality_threshold={FORMATION_QUALITY_POOL_THRESHOLD}; "
                f"completion_time_threshold={COMPLETION_TIME_POOL_THRESHOLD}; latency={LATENCY_RANGE}; "
                f"jitter={JITTER_RANGE}; noise_std={NOISE_STD_RANGE}"
            ),
        })
        writer.writerow(summary_row)

    seed_pool_file = os.path.join(batch_res_path, "seed_pool.csv")
    seed_pool_columns = [
        "pool_id",
        "added_episode",
        "added_phase",
        "seed_source",
        "parent_pool_id",
        "mutation_applied",
        "reason",
        "status",
        "collision_type",
        "badness_score",
        "formation_quality_score",
        "completion_time_score",
        "tick",
        "runtime_seconds",
        "mean_formation_deviation",
        "max_formation_deviation",
        "min_interrobot_distance",
        "min_obstacle_distance",
        "seed",
        "latency_frames",
        "jitter_frames",
        "noise_std",
        "fault_start_step",
        "fault_duration_steps",
        "fuzz_strength",
    ]
    with open(seed_pool_file, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=seed_pool_columns)
        writer.writeheader()
        for entry in seed_pool_records:
            row = {key: "" for key in seed_pool_columns}
            row.update({key: entry.get(key, "") for key in seed_pool_columns})
            row.update({
                "latency_frames": entry["fuzz_params"]["latency_frames"],
                "jitter_frames": entry["fuzz_params"]["jitter_frames"],
                "noise_std": entry["fuzz_params"]["noise_std"],
                "fault_start_step": entry["fuzz_params"]["start_step"],
                "fault_duration_steps": entry["fuzz_params"]["duration_steps"],
                "fuzz_strength": entry["fuzz_params"]["fuzz_strength"],
            })
            writer.writerow(row)

    heuristic_log_file = os.path.join(batch_res_path, "heuristic_events.csv")
    heuristic_columns = [
        "episode",
        "phase",
        "seed_source",
        "parent_pool_id",
        "mutation_applied",
        "entered_seed_pool",
        "seed_pool_reason",
        "pool_size_after_episode",
        "badness_score",
        "formation_quality_score",
        "completion_time_score",
        "latency_frames",
        "jitter_frames",
        "noise_std",
        "fault_start_step",
        "fault_duration_steps",
        "fuzz_strength",
    ]
    with open(heuristic_log_file, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=heuristic_columns)
        writer.writeheader()
        writer.writerows(heuristic_events)

    conclusion_text = (
        f"=============== 事后总结报告 ===============\n"
        f"总计推演轮次 : {TOTAL_EPISODES}\n"
        f"成功到达终点 : {total_successes}\n"
        f"RRT 寻路失败 : {total_rrt_failed}\n"
        f"超时未到达 : {total_timeouts}\n"
        f"感知半径 (Influence Radius) : {inf_r} m\n"
        f"碰撞半径 (Crash Distance)   : {cra_r} m\n"
        f"障碍物膨胀 (C-Space Buffer) : {infl_r} m\n"
        f"总计碰撞次数 : {total_collisions}\n"
        f"整体碰撞概率 : {(total_collisions / TOTAL_EPISODES) * 100:.2f}%\n"
        f"Fuzz Mode : {FUZZ_MODE}\n"
        f"Master Seed : {master_seed}\n"
        f"Latency Range : {LATENCY_RANGE} frames\n"
        f"Jitter Range : {JITTER_RANGE} frames\n"
        f"Noise Std Range : {NOISE_STD_RANGE} m\n"
        f"--------------------------------------------\n"
        f"其中 机-机 相撞次数 : {collision_types['drone_drone']}\n"
        f"其中 机-墙 相撞次数 : {collision_types['drone_obs']}\n"
        f"--------------------------------------------\n"
    )

    if FUZZ_MODE == "heuristic":
        conclusion_text += (
            f"预热轮次 : {warmup_episodes}\n"
            f"正式轮次 : {TOTAL_EPISODES - warmup_episodes}\n"
            f"随机探索概率 : {RANDOM_EXPLORATION_PROB}\n"
            f"种子变异比例上限 : {MUTATION_RATIO}\n"
            f"综合坏度入池阈值 : {SEED_POOL_BADNESS_THRESHOLD}\n"
            f"队形质量入池阈值 : {FORMATION_QUALITY_POOL_THRESHOLD}\n"
            f"完成时间入池阈值 : {COMPLETION_TIME_POOL_THRESHOLD}\n"
            f"最终种子池大小 : {len(seed_pool)}\n"
            f"--------------------------------------------\n"
        )
    else:
        conclusion_text += "随机模式：禁用预热/种子池/变异，所有轮次独立随机生成 fuzz 参数。\n"
        conclusion_text += "--------------------------------------------\n"

    if total_collisions > 0:
        conclusion_text += f"发生碰撞的具体轮次 :\n{crashed_episodes}\n"
    else:
        conclusion_text += "完美！无人机编队通过安全膨胀层实现了 100% 零碰撞飞行！\n"

    conclusion_text += f"\n* 每轮结构化结果 CSV 已保存至 :\n{csv_file}\n"
    conclusion_text += f"* 种子池 CSV 已保存至 :\n{seed_pool_file}\n"
    conclusion_text += f"* 启发式事件日志已保存至 :\n{heuristic_log_file}\n"
    conclusion_text += f"* 碰撞日志和现场截图已保存至 :\n{batch_res_path}\n"
    conclusion_text += "============================================\n"

    print(conclusion_text)
