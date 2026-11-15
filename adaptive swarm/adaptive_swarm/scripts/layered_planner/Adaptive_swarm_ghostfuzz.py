import sys, os
import argparse
import csv
import json
import importlib
from os import makedirs
from time import strftime
import time
import math
import random
import numpy as np
from numpy.linalg import norm
import matplotlib
from svgpath2mpl import parse_path
from matplotlib.path import Path
from shapely.geometry import Point, Polygon

# 批处理只保存图片，不需要 GUI 后端；实时曲线模式保留系统默认交互后端。
if "--live-density-plot" not in sys.argv:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt

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


def moving_average(values, window):
    window = max(1, int(window))
    averaged = []
    for index in range(len(values)):
        start = max(0, index - window + 1)
        averaged.append(float(np.mean(values[start:index + 1])))
    return averaged


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


def median_or_empty(values):
    clean_values = [float(value) for value in values if value not in ("", None)]
    return round(float(np.median(clean_values)), 6) if clean_values else ""


def mean_or_empty(values):
    clean_values = [float(value) for value in values if value not in ("", None)]
    return round(float(np.mean(clean_values)), 6) if clean_values else ""


def rectangle_polygon(center, half_length, half_width, direction=None):
    x, y = float(center[0]), float(center[1])
    direction = np.array(direction if direction is not None else [1.0, 0.0], dtype=float)
    direction_norm = norm(direction)
    if direction_norm == 0:
        direction = np.array([1.0, 0.0])
    else:
        direction = direction / direction_norm
    normal = np.array([-direction[1], direction[0]])
    length_vec = direction * float(half_length)
    width_vec = normal * float(half_width)
    return np.array([
        np.array([x, y]) - length_vec - width_vec,
        np.array([x, y]) + length_vec - width_vec,
        np.array([x, y]) + length_vec + width_vec,
        np.array([x, y]) - length_vec + width_vec,
    ])


def oriented_axes(direction=None):
    direction = np.array(direction if direction is not None else [1.0, 0.0], dtype=float)
    direction_norm = norm(direction)
    if direction_norm == 0:
        direction = np.array([1.0, 0.0])
    else:
        direction = direction / direction_norm
    normal = np.array([-direction[1], direction[0]])
    return direction, normal


def ghost_dimensions(shape, radius, wall_length_multiplier=6.0, wall_width_multiplier=1.0):
    if shape in ("wall", "threshold"):
        length_multiplier = wall_length_multiplier if shape == "wall" else 4.0
        width_multiplier = wall_width_multiplier if shape == "wall" else 0.75
        return {
            "half_length": float(radius) * float(length_multiplier),
            "half_width": float(radius) * float(width_multiplier) / 2.0,
        }
    return {
        "half_length": float(radius),
        "half_width": float(radius),
    }


def ghost_polygons(center, radius, shape, direction=None,
                   wall_length_multiplier=6.0, wall_width_multiplier=1.0,
                   point_cluster_count=7, point_cluster_spread_multiplier=3.0,
                   point_cluster_block_multiplier=0.5):
    center = np.array(center, dtype=float)
    radius = float(radius)
    direction, normal = oriented_axes(direction)
    block_half = radius * float(point_cluster_block_multiplier) / 2.0

    if shape in ("sphere", "cube", "wall", "threshold"):
        dims = ghost_dimensions(shape, radius, wall_length_multiplier, wall_width_multiplier)
        return [rectangle_polygon(center, dims["half_length"], dims["half_width"], direction)]

    if shape == "l_shape":
        return [
            rectangle_polygon(center + direction * radius, 2.0 * radius, 0.5 * radius, direction),
            rectangle_polygon(center + normal * radius, 2.0 * radius, 0.5 * radius, normal),
        ]

    if shape == "u_shape":
        return [
            rectangle_polygon(center - direction * 1.5 * radius, 2.0 * radius, 0.5 * radius, normal),
            rectangle_polygon(center + normal * 1.5 * radius, 2.0 * radius, 0.5 * radius, direction),
            rectangle_polygon(center - normal * 1.5 * radius, 2.0 * radius, 0.5 * radius, direction),
        ]

    if shape == "cross":
        return [
            rectangle_polygon(center, 2.5 * radius, 0.45 * radius, direction),
            rectangle_polygon(center, 2.5 * radius, 0.45 * radius, normal),
        ]

    if shape == "point_cluster":
        offsets = [
            np.array([0.0, 0.0]),
            direction,
            -direction,
            normal,
            -normal,
            direction * 0.7 + normal * 0.7,
            -direction * 0.7 - normal * 0.7,
            direction * 0.7 - normal * 0.7,
            -direction * 0.7 + normal * 0.7,
        ]
        spread = radius * float(point_cluster_spread_multiplier)
        selected_offsets = offsets[:max(1, min(int(point_cluster_count), len(offsets)))]
        return [
            rectangle_polygon(center + offset * spread, block_half, block_half, direction)
            for offset in selected_offsets
        ]

    dims = ghost_dimensions(shape, radius, wall_length_multiplier, wall_width_multiplier)
    return [rectangle_polygon(center, dims["half_length"], dims["half_width"], direction)]


def ghost_clearance(point, ghost, drone_radius):
    shape = ghost.get("shape", "sphere")
    ghost_center = np.array(ghost["position"], dtype=float)
    ghost_radius = float(ghost["radius"])
    if shape == "sphere":
        return norm(np.array(point, dtype=float) - ghost_center) - drone_radius - ghost_radius

    polygons = ghost_polygons(
        ghost_center,
        ghost_radius,
        shape,
        ghost.get("direction"),
        ghost.get("wall_length_multiplier", 6.0),
        ghost.get("wall_width_multiplier", 1.0),
        ghost.get("point_cluster_count", 7),
        ghost.get("point_cluster_spread_multiplier", 3.0),
        ghost.get("point_cluster_block_multiplier", 0.5),
    )
    point_geom = Point(float(point[0]), float(point[1]))
    return min(Polygon(poly).distance(point_geom) for poly in polygons) - drone_radius


def risk_kernel(clearance, scale):
    clearance = max(0.0, float(clearance))
    scale = max(1e-6, float(scale))
    return math.exp(-math.log(10.0) * clearance / scale)


def obstacle_repulsion_direction(point, polygon):
    nearest = np.array(polygon.exterior.interpolate(polygon.exterior.project(Point(point))).coords[0])
    direction = np.array(point, dtype=float) - nearest
    direction_norm = norm(direction)
    if direction_norm == 0:
        return np.array([1.0, 0.0])
    return direction / direction_norm


def compute_collision_density(robots, obstacle_polygons, params, ghost=None):
    drone_radius = params.crash_uav_dist / 2.0
    density_by_robot = {}
    details_by_robot = {}
    for robot in robots:
        density = 0.0
        detail = {"drone_drone": [], "drone_obstacle": [], "ghost": None}

        for other in robots:
            if other.id == robot.id:
                continue
            center_distance = norm(robot.sp - other.sp)
            clearance = center_distance - params.crash_uav_dist
            contribution = risk_kernel(clearance, drone_radius)
            density += contribution
            direction = robot.sp - other.sp
            direction_norm = norm(direction)
            detail["drone_drone"].append({
                "other_id": other.id,
                "clearance": float(clearance),
                "contribution": float(contribution),
                "direction": (direction / direction_norm if direction_norm else np.array([1.0, 0.0])),
            })

        point = Point(float(robot.sp[0]), float(robot.sp[1]))
        for obs_index, polygon in enumerate(obstacle_polygons):
            clearance = polygon.distance(point) - drone_radius
            contribution = risk_kernel(clearance, drone_radius)
            density += contribution
            detail["drone_obstacle"].append({
                "obstacle_index": obs_index,
                "clearance": float(clearance),
                "contribution": float(contribution),
                "direction": obstacle_repulsion_direction(robot.sp, polygon),
            })

        ghost_visible_ids = ghost.get("visible_robot_ids", []) if ghost else []
        if ghost and ghost.get("active") and robot.id in ghost_visible_ids:
            ghost_center = np.array(ghost["position"], dtype=float)
            ghost_radius = float(ghost["radius"])
            clearance = ghost_clearance(robot.sp, ghost, drone_radius)
            contribution = risk_kernel(clearance, drone_radius)
            density += contribution
            detail["ghost"] = {
                "clearance": float(clearance),
                "contribution": float(contribution),
                "position": ghost_center,
                "radius": ghost_radius,
                "shape": ghost.get("shape", "sphere"),
            }

        density_by_robot[robot.id] = float(density)
        details_by_robot[robot.id] = detail

    return density_by_robot, details_by_robot


def ghost_risk_direction(density_detail):
    weighted_direction = np.array([0.0, 0.0])
    for item in density_detail["drone_drone"] + density_detail["drone_obstacle"]:
        weighted_direction += item["contribution"] * np.array(item["direction"], dtype=float)
    direction_norm = norm(weighted_direction)
    if direction_norm == 0:
        return np.array([1.0, 0.0])
    return weighted_direction / direction_norm


def select_ghost_position(robot, density_detail, ghost_radius, params, shape="sphere",
                          wall_length_multiplier=6.0, wall_width_multiplier=1.0):
    weighted_direction = ghost_risk_direction(density_detail)

    drone_radius = params.crash_uav_dist / 2.0
    if shape in ("wall", "threshold"):
        dims = ghost_dimensions(shape, ghost_radius, wall_length_multiplier, wall_width_multiplier)
        offset = drone_radius + dims["half_width"]
    else:
        offset = drone_radius + ghost_radius
    return robot.sp + weighted_direction * offset


class GhostController:
    def __init__(self, mode, params, rng, ghost_radius, alpha, density_threshold=None,
                 random_time_range=(0.0, 2.5), random_radius_range=None,
                 num_robots=4, hold_min_ticks=1, visible_count=1,
                 ghost_shape="sphere", wall_length_multiplier=6.0,
                 wall_width_multiplier=1.0, point_cluster_count=7,
                 point_cluster_spread_multiplier=3.0,
                 point_cluster_block_multiplier=0.5):
        self.mode = mode
        self.params = params
        self.rng = rng
        self.alpha = alpha
        self.density_threshold = density_threshold
        self.random_time_range = random_time_range
        self.random_radius_range = random_radius_range or (ghost_radius, ghost_radius)
        self.hold_min_ticks = hold_min_ticks
        self.ghost_shape = ghost_shape
        self.wall_length_multiplier = float(wall_length_multiplier)
        self.wall_width_multiplier = float(wall_width_multiplier)
        self.point_cluster_count = int(point_cluster_count)
        self.point_cluster_spread_multiplier = float(point_cluster_spread_multiplier)
        self.point_cluster_block_multiplier = float(point_cluster_block_multiplier)
        self.num_robots = int(num_robots)
        self.visible_count_requested = int(visible_count)
        self.visible_count = int(clamp(visible_count, 1, num_robots))
        self.active = False
        self.injected = False
        self.completed = False
        self.inject_tick = None
        self.release_tick = None
        self.target_robot_id = None
        self.visible_robot_ids = []
        self.position = None
        self.direction = np.array([1.0, 0.0])
        self.radius = ghost_radius
        self.inject_density = None
        self.trigger_density = None
        self.missed_before_completion = False

        if self.mode == "random":
            inject_seconds = float(rng.uniform(random_time_range[0], random_time_range[1]))
            self.inject_tick = int(round(inject_seconds * params.ViconRate))
            self.target_robot_id = int(rng.randint(1, num_robots + 1))
            self.radius = round(float(rng.uniform(self.random_radius_range[0], self.random_radius_range[1])), 6)

    def maybe_update(self, time_step, robots, obstacle_polygons, density_by_robot, details_by_robot):
        if self.completed:
            return

        if not self.injected:
            if self.mode == "ghost":
                swarm_density = float(np.mean(list(density_by_robot.values()))) if density_by_robot else 0.0
                if self.density_threshold is not None and swarm_density >= self.density_threshold:
                    self.target_robot_id = max(density_by_robot, key=density_by_robot.get)
                    self.inject_tick = time_step
                    self.inject_density = density_by_robot[self.target_robot_id]
                    self.trigger_density = swarm_density
                    self._activate(time_step, robots, details_by_robot)
            elif self.mode == "random" and time_step >= self.inject_tick:
                density_by_robot, details_by_robot = compute_collision_density(
                    robots, obstacle_polygons, self.params
                )
                self.inject_density = density_by_robot.get(self.target_robot_id, 0.0)
                self.trigger_density = float(np.mean(list(density_by_robot.values()))) if density_by_robot else 0.0
                self._activate(time_step, robots, details_by_robot)
            return

        if not self.active:
            return

        current_density = density_by_robot.get(self.target_robot_id, 0.0)
        held_long_enough = time_step - self.inject_tick >= self.hold_min_ticks
        if held_long_enough and current_density <= self.inject_density:
            self.active = False
            self.completed = True
            self.release_tick = time_step

    def _activate(self, time_step, robots, details_by_robot):
        target_robot = next(robot for robot in robots if robot.id == self.target_robot_id)
        self.visible_robot_ids = self._select_visible_robot_ids(robots, details_by_robot)
        self.direction = ghost_risk_direction(details_by_robot[self.target_robot_id])
        if self.ghost_shape in ("wall", "threshold"):
            self.direction = np.array([-self.direction[1], self.direction[0]])
        self.position = select_ghost_position(
            target_robot, details_by_robot[self.target_robot_id],
            self.radius, self.params, self.ghost_shape,
            self.wall_length_multiplier, self.wall_width_multiplier
        )
        self.injected = True
        self.active = True
        self.inject_tick = time_step

    def _select_visible_robot_ids(self, robots, details_by_robot):
        all_ids = [robot.id for robot in robots]
        if self.visible_count >= len(all_ids):
            return all_ids
        if self.mode == "random":
            remaining_ids = [robot_id for robot_id in all_ids if robot_id != self.target_robot_id]
            self.rng.shuffle(remaining_ids)
            return [self.target_robot_id] + remaining_ids[:self.visible_count - 1]

        density_rank = sorted(
            all_ids,
            key=lambda robot_id: sum(
                item["contribution"]
                for item in (
                    details_by_robot[robot_id]["drone_drone"]
                    + details_by_robot[robot_id]["drone_obstacle"]
                )
            ),
            reverse=True,
        )
        selected = [self.target_robot_id]
        for robot_id in density_rank:
            if robot_id not in selected:
                selected.append(robot_id)
            if len(selected) >= self.visible_count:
                break
        return selected

    def as_dict(self):
        return {
            "ghost_injected": self.injected,
            "ghost_active": self.active,
            "ghost_inject_tick": "" if self.inject_tick is None else self.inject_tick,
            "ghost_release_tick": "" if self.release_tick is None else self.release_tick,
            "ghost_target_robot_id": "" if self.target_robot_id is None else self.target_robot_id,
            "ghost_visible_count_requested": self.visible_count_requested,
            "ghost_visible_count_actual": self.visible_count,
            "ghost_visible_robot_ids": "|".join(map(str, self.visible_robot_ids)),
            "ghost_shape": self.ghost_shape,
            "ghost_component_count": len(self.obstacle_polygons()) if self.injected else "",
            "ghost_radius": self.radius,
            "ghost_wall_length": (
                round(float(2.0 * self.radius * self.wall_length_multiplier), 6)
                if self.ghost_shape == "wall" else ""
            ),
            "ghost_wall_width": (
                round(float(self.radius * self.wall_width_multiplier), 6)
                if self.ghost_shape == "wall" else ""
            ),
            "ghost_point_cluster_count": (
                self.point_cluster_count if self.ghost_shape == "point_cluster" else ""
            ),
            "ghost_point_cluster_spread": (
                round(float(self.radius * self.point_cluster_spread_multiplier), 6)
                if self.ghost_shape == "point_cluster" else ""
            ),
            "ghost_point_cluster_block_size": (
                round(float(self.radius * self.point_cluster_block_multiplier), 6)
                if self.ghost_shape == "point_cluster" else ""
            ),
            "ghost_position_x": "" if self.position is None else round(float(self.position[0]), 6),
            "ghost_position_y": "" if self.position is None else round(float(self.position[1]), 6),
            "ghost_direction_x": round(float(self.direction[0]), 6),
            "ghost_direction_y": round(float(self.direction[1]), 6),
            "ghost_alpha": self.alpha,
            "ghost_density_threshold": "" if self.density_threshold is None else round(float(self.density_threshold), 6),
            "ghost_trigger_density": "" if self.trigger_density is None else round(float(self.trigger_density), 6),
            "ghost_inject_density": "" if self.inject_density is None else round(float(self.inject_density), 6),
        }

    def obstacle_polygons(self):
        if not self.active or self.position is None:
            return []
        return ghost_polygons(
            np.array(self.position, dtype=float),
            self.radius,
            self.ghost_shape,
            self.direction,
            self.wall_length_multiplier,
            self.wall_width_multiplier,
            self.point_cluster_count,
            self.point_cluster_spread_multiplier,
            self.point_cluster_block_multiplier,
        )


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
    def __init__(self, episode_id, batch_folder, seed=None, fuzz_params=None,
                 ghost_controller=None, collect_baseline_density=False,
                 live_density_plot=False, save_density_curve=False,
                 density_moving_average_window=10):
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
        self.ghost_controller = ghost_controller
        self.collect_baseline_density = collect_baseline_density
        self.density_history = []
        self.live_density_plot = bool(live_density_plot)
        self.save_density_curve = bool(save_density_curve or live_density_plot)
        self.density_moving_average_window = max(1, int(density_moving_average_window))
        self.density_fig = None
        self.density_ax = None
        self.density_raw_line = None
        self.density_ma_line = None
        self.density_curve_image = ""

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
        density_curve_image = self.write_density_curve_image(status)
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
            "ghost_injected": "",
            "ghost_active": "",
            "ghost_inject_tick": "",
            "ghost_release_tick": "",
            "ghost_target_robot_id": "",
            "ghost_visible_count_requested": "",
            "ghost_visible_count_actual": "",
            "ghost_visible_robot_ids": "",
            "ghost_shape": "",
            "ghost_component_count": "",
            "ghost_radius": "",
            "ghost_wall_length": "",
            "ghost_wall_width": "",
            "ghost_point_cluster_count": "",
            "ghost_point_cluster_spread": "",
            "ghost_point_cluster_block_size": "",
            "ghost_position_x": "",
            "ghost_position_y": "",
            "ghost_direction_x": "",
            "ghost_direction_y": "",
            "ghost_alpha": "",
            "ghost_density_threshold": "",
            "ghost_trigger_density": "",
            "ghost_inject_density": "",
            "collision_log": collision_log,
            "detail_log_json": detail_log_json,
            "crash_image": crash_image,
            "density_curve_image": density_curve_image,
            "note": note,
        }
        if self.ghost_controller:
            result.update(self.ghost_controller.as_dict())
        return result

    def density_series(self):
        times = [item["time_seconds"] for item in self.density_history]
        raw = [item["swarm_density"] for item in self.density_history]
        smoothed = moving_average(raw, self.density_moving_average_window)
        return times, raw, smoothed

    def ensure_density_plot(self):
        if self.density_fig is None or self.density_ax is None:
            self.density_fig, self.density_ax = plt.subplots(num=f"density_ep_{self.episode_id}", figsize=(8, 4.5))
            self.density_raw_line, = self.density_ax.plot([], [], color="#4C78A8", alpha=0.35, linewidth=1.2,
                                                          label="raw density")
            self.density_ma_line, = self.density_ax.plot([], [], color="#F58518", linewidth=2.0,
                                                         label=f"{self.density_moving_average_window}-sample average")
            self.density_ax.set_xlabel("Time (s)")
            self.density_ax.set_ylabel("Collision density")
            self.density_ax.set_title(f"Episode {self.episode_id} collision density")
            self.density_ax.grid(True, alpha=0.25)
            self.density_ax.legend(loc="upper right")

    def update_live_density_plot(self):
        if not self.live_density_plot or not self.density_history:
            return
        self.ensure_density_plot()
        times, raw, smoothed = self.density_series()
        self.density_raw_line.set_data(times, raw)
        self.density_ma_line.set_data(times, smoothed)
        self.density_ax.relim()
        self.density_ax.autoscale_view()
        self.density_fig.canvas.draw_idle()
        plt.pause(0.001)

    def write_density_curve_image(self, status):
        if not self.save_density_curve or not self.density_history:
            return self.density_curve_image
        times, raw, smoothed = self.density_series()
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ax.plot(times, raw, color="#4C78A8", alpha=0.35, linewidth=1.2, label="raw density")
        ax.plot(times, smoothed, color="#F58518", linewidth=2.0,
                label=f"{self.density_moving_average_window}-sample average")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Collision density")
        ax.set_title(f"Episode {self.episode_id} collision density ({status})")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right")
        fig.tight_layout()
        image_name = f"Ep{self.episode_id}_collision_density_{status}.png"
        image_path = os.path.join(self.save_folder_path, image_name)
        fig.savefig(image_path, dpi=160)
        plt.close(fig)
        if self.density_fig is not None:
            plt.close(self.density_fig)
            self.density_fig = None
            self.density_ax = None
            self.density_raw_line = None
            self.density_ma_line = None
        self.density_curve_image = image_name
        return image_name

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
            "ghost_params": self.ghost_controller.as_dict() if self.ghost_controller else {},
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
                if (
                    self.ghost_controller
                    and self.ghost_controller.mode == "random"
                    and not self.ghost_controller.injected
                ):
                    self.ghost_controller.missed_before_completion = True
                    return self.build_result(
                        "injection_missed",
                        final_distance=dist_to_goal,
                        note="random ghost injection time was after task completion",
                    )
                # <--- 优化：成功到达终点时不再绘图保存，直接静默退出 --->
                return self.build_result("success", final_distance=dist_to_goal)

            self.robot1.sp_global = traj_global[sp_ind, :]
            self.robot1.local_planner(np.array(self.obstacles), self.params)

            followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global,
                                            v=normalize(self.robot1.sp_global - self.robot1.sp),
                                            l=self.params.interrobots_dist)
            for j in range(len(followers_sp_global)):
                self.robots[j + 1].sp_global = followers_sp_global[j]

            ghost_density_state = None
            if self.ghost_controller:
                ghost_density_state = {
                    "active": self.ghost_controller.active,
                    "target_robot_id": self.ghost_controller.target_robot_id,
                    "visible_robot_ids": self.ghost_controller.visible_robot_ids,
                    "shape": self.ghost_controller.ghost_shape,
                    "position": self.ghost_controller.position,
                    "direction": self.ghost_controller.direction,
                    "radius": self.ghost_controller.radius,
                    "wall_length_multiplier": self.ghost_controller.wall_length_multiplier,
                    "wall_width_multiplier": self.ghost_controller.wall_width_multiplier,
                    "point_cluster_count": self.ghost_controller.point_cluster_count,
                    "point_cluster_spread_multiplier": self.ghost_controller.point_cluster_spread_multiplier,
                    "point_cluster_block_multiplier": self.ghost_controller.point_cluster_block_multiplier,
                }
            density_by_robot, density_details = compute_collision_density(
                self.robots, self.obstacle_polygons, self.params, ghost_density_state
            )
            swarm_density = float(np.mean(list(density_by_robot.values()))) if density_by_robot else 0.0
            self.density_history.append({
                "tick": self.time_step,
                "time_seconds": self.time_step / self.params.ViconRate,
                "swarm_density": swarm_density,
                "robot_density": density_by_robot.copy(),
            })
            self.update_live_density_plot()
            if self.ghost_controller:
                self.ghost_controller.maybe_update(
                    self.time_step, self.robots, self.obstacle_polygons,
                    density_by_robot, density_details
                )

            communication_snapshot = []
            for p in range(len(followers_sp)):
                receiver_robot = self.robots[p + 1]
                perceived_neighbors_sp = [
                    other_robot.sp.copy()
                    for other_robot in self.robots
                    if other_robot.id != receiver_robot.id
                ]

                robots_obstacles = poses2polygons(perceived_neighbors_sp)
                ghost_obstacles = (
                    self.ghost_controller.obstacle_polygons()
                    if (
                        self.ghost_controller
                        and receiver_robot.id in self.ghost_controller.visible_robot_ids
                    )
                    else []
                )
                obstacles1 = np.array(self.obstacles + robots_obstacles + ghost_obstacles)
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


def parse_float_triplet(value):
    parts = [float(part.strip()) for part in value.split(",") if part.strip()]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("must provide exactly three comma-separated values")
    return tuple(parts)


def alpha_from_args(args):
    alpha_values = parse_float_triplet(args.alpha_values)
    if args.alpha is not None:
        return float(args.alpha), "custom"
    level_index = int(args.alpha_level) - 1
    return alpha_values[level_index], str(args.alpha_level)


def radius_from_args(args):
    radius_values = parse_float_triplet(args.ghost_radius_values)
    level_index = int(args.ghost_radius_level) - 1
    return radius_values[level_index], radius_values, str(args.ghost_radius_level)


def calibrate_density_profile(args, batch_folder, master_rng):
    bucket_seconds = args.bucket_seconds
    bucket_values = {}
    successful_ticks = []
    successful_coverage = []
    for ep in range(1, args.baseline_episodes + 1):
        print(f"正在校准 Baseline {ep:02d}/{args.baseline_episodes} 轮...", end="\r")
        seed = int(master_rng.randint(1, 100000000))
        sim = BaselineSimulation(
            episode_id=f"baseline_{ep}",
            batch_folder=batch_folder,
            seed=seed,
            fuzz_params={},
            ghost_controller=None,
            collect_baseline_density=True,
        )
        result = sim.start()
        if result["status"] != "success":
            continue
        successful_ticks.append(result["tick"])
        successful_coverage.append(result["total_path_length"])
        for item in sim.density_history:
            bucket = int(math.floor(item["time_seconds"] / bucket_seconds))
            bucket_values.setdefault(bucket, []).append(item["swarm_density"])

    print(" " * 60, end="\r")
    median_by_bucket = {
        bucket: float(np.median(values))
        for bucket, values in bucket_values.items()
        if values
    }
    b_max = max(median_by_bucket.values()) if median_by_bucket else 0.0
    return {
        "bucket_seconds": bucket_seconds,
        "median_by_bucket": median_by_bucket,
        "b_max": b_max,
        "baseline_success_count": len(successful_ticks),
        "baseline_completion_time_median": median_or_empty(successful_ticks),
        "baseline_coverage_median": median_or_empty(successful_coverage),
    }


def write_density_profile(profile, batch_folder):
    density_file = os.path.join(batch_folder, "baseline_density_profile.csv")
    with open(density_file, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=["bucket_index", "time_start", "time_end", "median_swarm_density"])
        writer.writeheader()
        for bucket in sorted(profile["median_by_bucket"]):
            writer.writerow({
                "bucket_index": bucket,
                "time_start": round(bucket * profile["bucket_seconds"], 6),
                "time_end": round((bucket + 1) * profile["bucket_seconds"], 6),
                "median_swarm_density": round(profile["median_by_bucket"][bucket], 6),
            })
    return density_file


def aggregate_density_histories(histories, statistic="mean"):
    max_tick = max(
        (item["tick"] for history in histories for item in history),
        default=-1,
    )
    rows = []
    for tick in range(max_tick + 1):
        values = [
            item["swarm_density"]
            for history in histories
            for item in history
            if item["tick"] == tick
        ]
        if not values:
            continue
        if statistic == "median":
            density_value = float(np.median(values))
        else:
            density_value = float(np.mean(values))
        rows.append({
            "tick": tick,
            "time_seconds": tick / Params().ViconRate,
            "density": density_value,
            "sample_count": len(values),
        })
    return rows


def smoothed_density_rows(rows, window):
    if not rows:
        return []
    smoothed = moving_average([row["density"] for row in rows], window)
    return [
        {
            **row,
            "density_smoothed": smoothed[index],
        }
        for index, row in enumerate(rows)
    ]


def write_density_comparison_csv(series_by_mode, csv_file, moving_average_window=10):
    max_tick = max(
        (row["tick"] for rows in series_by_mode.values() for row in rows),
        default=-1,
    )
    fieldnames = ["tick", "time_seconds"]
    for mode_name in series_by_mode:
        fieldnames.extend([
            f"{mode_name}_density",
            f"{mode_name}_density_ma{moving_average_window}",
            f"{mode_name}_sample_count",
        ])

    by_mode_tick = {
        mode_name: {
            row["tick"]: row
            for row in smoothed_density_rows(rows, moving_average_window)
        }
        for mode_name, rows in series_by_mode.items()
    }
    with open(csv_file, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for tick in range(max_tick + 1):
            row = {"tick": tick, "time_seconds": tick / Params().ViconRate}
            has_value = False
            for mode_name in series_by_mode:
                item = by_mode_tick[mode_name].get(tick)
                row[f"{mode_name}_density"] = "" if item is None else round(float(item["density"]), 8)
                row[f"{mode_name}_density_ma{moving_average_window}"] = (
                    "" if item is None else round(float(item["density_smoothed"]), 8)
                )
                row[f"{mode_name}_sample_count"] = "" if item is None else item["sample_count"]
                has_value = has_value or item is not None
            if has_value:
                writer.writerow(row)


def draw_density_comparison_axes(ax, series_by_mode, statistic, moving_average_window=10):
    labels = {
        "no_fuzz": "No Fuzz",
        "random_fuzz": "Random Fuzz",
        "ghost_fuzz": "GhostFuzz",
        "dpfuzz": "DPFuzz",
        "swarmfuzz": "SwarmFuzz",
    }
    colors = {
        "no_fuzz": "#2b6cb0",
        "random_fuzz": "#c05621",
        "ghost_fuzz": "#2f855a",
        "dpfuzz": "#805ad5",
        "swarmfuzz": "#d53f8c",
    }
    for mode_name, rows in series_by_mode.items():
        if not rows:
            continue
        rows = smoothed_density_rows(rows, moving_average_window)
        ax.plot(
            [row["time_seconds"] for row in rows],
            [row["density_smoothed"] for row in rows],
            linewidth=2,
            label=labels.get(mode_name, mode_name),
            color=colors.get(mode_name),
        )
    ax.set_xlabel("Task Time (s)")
    ax.set_ylabel(f"Swarm Collision Density ({statistic}, MA={moving_average_window})")
    ax.set_title("Collision Density During Task Execution")
    ax.grid(True, alpha=0.25)
    ax.legend()


def plot_density_comparison(series_by_mode, plot_file, statistic, moving_average_window=10):
    plt.close("all")
    fig, ax = plt.subplots(figsize=(10, 6))
    draw_density_comparison_axes(ax, series_by_mode, statistic, moving_average_window)
    fig.tight_layout()
    fig.savefig(plot_file, dpi=160)
    plt.close("all")


def update_live_density_comparison_plot(histories_by_mode, statistic, moving_average_window, state):
    if "fig" not in state:
        state["fig"], state["ax"] = plt.subplots(num="five_mode_density_compare", figsize=(10, 6))
    series_by_mode = {
        mode_name: aggregate_density_histories(histories, statistic)
        for mode_name, histories in histories_by_mode.items()
    }
    state["ax"].cla()
    draw_density_comparison_axes(state["ax"], series_by_mode, statistic, moving_average_window)
    state["fig"].tight_layout()
    state["fig"].canvas.draw_idle()
    plt.pause(0.001)


def build_ghost_controller(args, params, rng, ghost_radius, alpha, density_threshold,
                           radius_values, mode):
    random_time_range = (args.random_time_range[0], args.random_time_range[-1])
    return GhostController(
        mode=mode,
        params=params,
        rng=rng,
        ghost_radius=ghost_radius,
        alpha=alpha,
        density_threshold=density_threshold if mode == "ghost" else None,
        random_time_range=random_time_range,
        random_radius_range=(min(radius_values), max(radius_values)),
        num_robots=params.num_robots,
        visible_count=args.ghost_visible_count,
        ghost_shape=args.ghost_shape,
        wall_length_multiplier=args.ghost_wall_length_multiplier,
        wall_width_multiplier=args.ghost_wall_width_multiplier,
        point_cluster_count=args.ghost_point_cluster_count,
        point_cluster_spread_multiplier=args.ghost_point_cluster_spread_multiplier,
        point_cluster_block_multiplier=args.ghost_point_cluster_block_multiplier,
    )


def run_standard_density_episode(args, batch_folder, mode_name, ep, master_rng,
                                 params, ghost_radius, alpha, density_threshold,
                                 radius_values):
    latency_range = (0, 15)
    jitter_range = (0, 10)
    noise_std_range = (0.0, 0.15)
    fault_start_range = (20, 150)
    fault_duration_range = (20, 100)

    episode_seed = int(master_rng.randint(1, 100000000))
    ep_rng = np.random.RandomState(episode_seed)
    controller = None
    fuzz_params = {}
    if mode_name == "random_fuzz":
        controller = build_ghost_controller(
            args, params, ep_rng, ghost_radius, alpha,
            density_threshold, radius_values, "random"
        )
    elif mode_name == "ghost_fuzz":
        controller = build_ghost_controller(
            args, params, ep_rng, ghost_radius, alpha,
            density_threshold, radius_values, "ghost"
        )
    elif mode_name == "swarmfuzz_random":
        fuzz_params = random_fuzz_params(
            ep_rng, latency_range, jitter_range, noise_std_range,
            fault_start_range, fault_duration_range
        )

    sim = BaselineSimulation(
        episode_id=f"{mode_name}_{ep}",
        batch_folder=batch_folder,
        seed=episode_seed,
        fuzz_params=fuzz_params,
        ghost_controller=controller,
        live_density_plot=False,
        save_density_curve=False,
        density_moving_average_window=args.density_moving_average_window,
    )
    return sim.start(), sim.density_history, fuzz_params


def select_swarmfuzz_params(args, ep, master_rng, seed_pool):
    latency_range = (0, 15)
    jitter_range = (0, 10)
    noise_std_range = (0.0, 0.15)
    fault_start_range = (20, 150)
    fault_duration_range = (20, 100)
    warmup_episodes = max(1, int(math.ceil(args.episodes * args.swarmfuzz_warmup_ratio)))
    ep_rng = np.random.RandomState(int(master_rng.randint(1, 100000000)))
    if ep <= warmup_episodes or not seed_pool or master_rng.rand() < args.swarmfuzz_random_exploration_prob:
        return random_fuzz_params(
            ep_rng, latency_range, jitter_range, noise_std_range,
            fault_start_range, fault_duration_range
        ), "random_or_warmup"

    weights = np.array([entry["badness_score"] + 0.01 for entry in seed_pool], dtype=float)
    weights = weights / weights.sum()
    parent_index = int(master_rng.choice(len(seed_pool), p=weights))
    parent = seed_pool[parent_index]
    return mutate_fuzz_params(
        parent["params"], ep_rng, latency_range, jitter_range, noise_std_range,
        fault_start_range, fault_duration_range, args.swarmfuzz_mutation_ratio
    ), f"seed_pool:{parent['episode']}"


def maybe_add_swarmfuzz_seed(args, seed_pool, ep, result, fuzz_params, seed_source):
    badness_score, formation_quality_score, completion_time_score, formation_badness = evaluate_badness(
        result, 300, 1.0
    )
    entered_seed_pool = (
        result["status"] == "collision" or
        badness_score >= args.swarmfuzz_seed_pool_badness_threshold or
        formation_quality_score <= args.swarmfuzz_formation_quality_threshold or
        completion_time_score >= args.swarmfuzz_completion_time_threshold
    )
    if not entered_seed_pool:
        return
    seed_pool.append({
        "episode": ep,
        "params": fuzz_params.copy(),
        "badness_score": badness_score,
        "formation_quality_score": formation_quality_score,
        "completion_time_score": completion_time_score,
        "formation_badness": round(formation_badness, 6),
        "seed_source": seed_source,
    })
    seed_pool.sort(key=lambda item: item["badness_score"], reverse=True)
    del seed_pool[args.swarmfuzz_seed_pool_max_size:]


def run_swarmfuzz_density_episode(args, batch_folder, ep, master_rng, seed_pool):
    fuzz_params, seed_source = select_swarmfuzz_params(args, ep, master_rng, seed_pool)
    episode_seed = int(master_rng.randint(1, 100000000))
    sim = BaselineSimulation(
        episode_id=f"swarmfuzz_{ep}",
        batch_folder=batch_folder,
        seed=episode_seed,
        fuzz_params=fuzz_params,
        ghost_controller=None,
        live_density_plot=False,
        save_density_curve=False,
        density_moving_average_window=args.density_moving_average_window,
    )
    result = sim.start()
    maybe_add_swarmfuzz_seed(args, seed_pool, ep, result, fuzz_params, seed_source)
    return result, sim.density_history


def build_dpfuzz_config(args, dpfuzzer):
    config = dpfuzzer.difficulty_config(args.dpfuzzer_difficulty)
    if args.dpfuzzer_obstacle_count is not None:
        config["obstacle_count"] = args.dpfuzzer_obstacle_count
    if args.dpfuzzer_obstacle_size_min is not None:
        config["size_range"] = (args.dpfuzzer_obstacle_size_min, config["size_range"][1])
    if args.dpfuzzer_obstacle_size_max is not None:
        config["size_range"] = (config["size_range"][0], args.dpfuzzer_obstacle_size_max)
    if args.dpfuzzer_trajectory_offset_min is not None:
        config["offset_range"] = (args.dpfuzzer_trajectory_offset_min, config["offset_range"][1])
    if args.dpfuzzer_trajectory_offset_max is not None:
        config["offset_range"] = (config["offset_range"][0], args.dpfuzzer_trajectory_offset_max)
    return config


def run_dpfuzz_density_episode(args, batch_folder, ep, master_rng, dpfuzzer_state):
    dpfuzzer = dpfuzzer_state["module"]
    if ep <= args.dpfuzzer_warmup_episodes or not dpfuzzer_state["seed_pool"]:
        scenario = dpfuzzer.generate_initial_scenario(
            dpfuzzer_state["empty_route"], master_rng, dpfuzzer_state["config"],
            f"D{dpfuzzer_state['next_scenario_id']}"
        )
    else:
        parent = dpfuzzer.select_parent_seed(dpfuzzer_state["seed_pool"], master_rng)
        scenario = dpfuzzer.mutate_scenario(
            parent, master_rng, dpfuzzer_state["empty_route"], dpfuzzer_state["config"],
            f"D{dpfuzzer_state['next_scenario_id']}"
        )
    dpfuzzer_state["next_scenario_id"] += 1

    episode_seed = int(master_rng.randint(1, 100000000))
    sim = dpfuzzer.BaselineSimulation(
        episode_id=f"dpfuzz_{ep}",
        batch_folder=batch_folder,
        seed=episode_seed,
        fuzz_params={},
        obstacles_override=dpfuzzer.scenario_to_obstacles(scenario),
        scenario_id=scenario["scenario_id"],
    )
    result = sim.start()
    orf, grf = dpfuzzer.compute_orf_and_grf(sim, scenario, result)
    scenario["orf"] = orf
    scenario["grf"] = grf
    if result["status"] in ("collision", "timeout") or grf > 0.0:
        dpfuzzer_state["seed_pool"].append(scenario)
        dpfuzzer_state["seed_pool"].sort(key=lambda item: item.get("grf", 0.0), reverse=True)
        del dpfuzzer_state["seed_pool"][args.dpfuzzer_seed_pool_size:]
    return result, sim.density_history


def run_density_comparison(args):
    params = Params()
    ghost_radius, radius_values, radius_level = radius_from_args(args)
    alpha, alpha_level = alpha_from_args(args)
    master_seed = int(args.seed if args.seed is not None else time.time())
    master_rng = np.random.RandomState(master_seed)

    folder_time = strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
    batch_res_path = (
        f"data_collection/density_compare_{args.ghost_shape}_"
        f"rL{radius_level}_aL{alpha_level}_vis{args.ghost_visible_count}_{folder_time}/"
    )
    makedirs(batch_res_path, exist_ok=True)

    profile = calibrate_density_profile(args, batch_res_path, master_rng)
    density_profile_file = write_density_profile(profile, batch_res_path)
    density_threshold = alpha * profile["b_max"]

    modes = ["no_fuzz", "random_fuzz", "ghost_fuzz", "dpfuzz", "swarmfuzz"]
    histories_by_mode = {mode_name: [] for mode_name in modes}
    results_by_mode = {mode_name: [] for mode_name in modes}
    swarmfuzz_seed_pool = []
    live_density_state = {}
    dpfuzzer_state = None
    if "dpfuzz" in modes:
        dpfuzzer = importlib.import_module("adaptive_swarm.scripts.layered_planner.Adaptive_swarm_DPFuzzer")
        dpfuzzer_config = build_dpfuzz_config(args, dpfuzzer)
        empty_route, _ = dpfuzzer.sample_empty_scene_trajectory(
            batch_res_path, int(master_rng.randint(1, 100000000))
        )
        dpfuzzer_state = {
            "module": dpfuzzer,
            "config": dpfuzzer_config,
            "empty_route": empty_route,
            "seed_pool": [],
            "next_scenario_id": 1,
        }

    for mode_name in modes:
        for ep in range(1, args.episodes + 1):
            print(f"密度对比 {mode_name} 第 {ep:02d}/{args.episodes} 轮...", end="\r")
            if mode_name == "dpfuzz":
                result, density_history = run_dpfuzz_density_episode(
                    args, batch_res_path, ep, master_rng, dpfuzzer_state
                )
            elif mode_name == "swarmfuzz":
                result, density_history = run_swarmfuzz_density_episode(
                    args, batch_res_path, ep, master_rng, swarmfuzz_seed_pool
                )
            else:
                result, density_history, _ = run_standard_density_episode(
                    args, batch_res_path, mode_name, ep, master_rng,
                    params, ghost_radius, alpha, density_threshold, radius_values
                )
            histories_by_mode[mode_name].append(density_history)
            results_by_mode[mode_name].append(result)
            if args.live_density_plot:
                update_live_density_comparison_plot(
                    histories_by_mode, args.density_plot_stat,
                    args.density_moving_average_window, live_density_state
                )

    print(" " * 80, end="\r")

    series_by_mode = {
        mode_name: aggregate_density_histories(histories, args.density_plot_stat)
        for mode_name, histories in histories_by_mode.items()
    }
    csv_file = os.path.join(batch_res_path, "density_timeseries.csv")
    plot_file = os.path.join(batch_res_path, "density_compare.png")
    write_density_comparison_csv(series_by_mode, csv_file, args.density_moving_average_window)
    plot_density_comparison(
        series_by_mode, plot_file, args.density_plot_stat,
        args.density_moving_average_window
    )

    summary_file = os.path.join(batch_res_path, "density_compare_summary.csv")
    with open(summary_file, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "mode", "episodes", "success_count", "collision_count",
            "timeout_count", "injection_missed_count", "collision_rate",
            "completion_time_mean", "completion_time_median",
        ])
        writer.writeheader()
        for mode_name, results in results_by_mode.items():
            success_count = sum(1 for result in results if result["status"] == "success")
            collision_count = sum(1 for result in results if result["status"] == "collision")
            timeout_count = sum(1 for result in results if result["status"] == "timeout")
            missed_count = sum(1 for result in results if result["status"] == "injection_missed")
            completion_ticks = [
                result["tick"] for result in results
                if result["status"] == "success" and result.get("tick") not in ("", None)
            ]
            writer.writerow({
                "mode": mode_name,
                "episodes": args.episodes,
                "success_count": success_count,
                "collision_count": collision_count,
                "timeout_count": timeout_count,
                "injection_missed_count": missed_count,
                "collision_rate": round(collision_count / args.episodes, 6),
                "completion_time_mean": mean_or_empty(completion_ticks),
                "completion_time_median": median_or_empty(completion_ticks),
            })

    conclusion_text = (
        "=============== 碰撞密度对比实验 ===============\n"
        f"每组轮次 : {args.episodes}\n"
        f"对比模式 : No Fuzz / Random Fuzz / GhostFuzz / DPFuzz / SwarmFuzz\n"
        f"密度曲线统计 : {args.density_plot_stat}\n"
        f"曲线平滑窗口 : {args.density_moving_average_window} samples\n"
        f"Ghost Shape : {args.ghost_shape}\n"
        f"Ghost 可见无人机数量 : {min(args.ghost_visible_count, params.num_robots)}"
        f" / 请求 {args.ghost_visible_count}\n"
        f"Alpha : {alpha}\n"
        f"Ghost Radius : {ghost_radius} m\n"
        f"DPFuzz 难度 : {args.dpfuzzer_difficulty}\n"
        f"Baseline B_max : {profile['b_max']:.6f}\n"
        f"Ghost 触发阈值 : {density_threshold:.6f}\n"
        "--------------------------------------------\n"
        f"* 碰撞密度时间序列 CSV :\n{csv_file}\n"
        f"* 碰撞密度折线图 :\n{plot_file}\n"
        f"* 五组结果摘要 CSV :\n{summary_file}\n"
        f"* Baseline 密度剖面 :\n{density_profile_file}\n"
        "============================================\n"
    )
    conclusion_file = os.path.join(batch_res_path, "conclusion.txt")
    with open(conclusion_file, "w", encoding="utf-8") as f:
        f.write(conclusion_text)
    print(conclusion_text)


def build_arg_parser():
    parser = argparse.ArgumentParser(description="GhostFuzz / Random Ghost fuzzing for adaptive swarm.")
    parser.add_argument("--mode", choices=["ghost", "random"], default="ghost")
    parser.add_argument("--episodes", type=int, default=400)
    parser.add_argument("--baseline-episodes", type=int, default=50)
    parser.add_argument("--density-compare", action="store_true",
                        help="run no_fuzz/random_fuzz/ghost_fuzz/dpfuzz/swarmfuzz and plot density curves")
    parser.add_argument("--density-plot-stat", choices=["mean", "median"], default="mean",
                        help="aggregation statistic for density curves across episodes")
    parser.add_argument("--live-density-plot", action="store_true",
                        help="show a live collision-density line chart during each episode")
    parser.add_argument("--save-density-curve", action="store_true",
                        help="save one per-episode collision-density curve image")
    parser.add_argument("--density-moving-average-window", type=int, default=10,
                        help="sample count used by the live/final collision-density moving average")
    parser.add_argument("--dpfuzzer-difficulty", choices=["easy", "medium", "hard"], default="medium",
                        help="obstacle-generation difficulty used by the DPFuzz comparison curve")
    parser.add_argument("--dpfuzzer-obstacle-count", type=int, default=None)
    parser.add_argument("--dpfuzzer-obstacle-size-min", type=float, default=None)
    parser.add_argument("--dpfuzzer-obstacle-size-max", type=float, default=None)
    parser.add_argument("--dpfuzzer-trajectory-offset-min", type=float, default=None)
    parser.add_argument("--dpfuzzer-trajectory-offset-max", type=float, default=None)
    parser.add_argument("--dpfuzzer-warmup-episodes", type=int, default=20)
    parser.add_argument("--dpfuzzer-seed-pool-size", type=int, default=80)
    parser.add_argument("--swarmfuzz-warmup-ratio", type=float, default=0.20)
    parser.add_argument("--swarmfuzz-random-exploration-prob", type=float, default=0.15)
    parser.add_argument("--swarmfuzz-mutation-ratio", type=float, default=0.10)
    parser.add_argument("--swarmfuzz-seed-pool-badness-threshold", type=float, default=0.35)
    parser.add_argument("--swarmfuzz-formation-quality-threshold", type=float, default=0.55)
    parser.add_argument("--swarmfuzz-completion-time-threshold", type=float, default=0.75)
    parser.add_argument("--swarmfuzz-seed-pool-max-size", type=int, default=100)
    parser.add_argument("--ghost-radius-level", choices=["1", "2", "3"], default="1")
    parser.add_argument("--ghost-radius-values", default="0.05,0.10,0.15")
    parser.add_argument("--alpha-level", choices=["1", "2", "3"], default="2")
    parser.add_argument("--alpha-values", default="0.4,0.6,0.8")
    parser.add_argument("--alpha", type=float, default=None)
    parser.add_argument("--bucket-seconds", type=float, default=0.2)
    parser.add_argument("--ghost-visible-count", type=int, default=1,
                        help="number of drones that can perceive the injected ghost obstacle")
    parser.add_argument(
        "--ghost-shape",
        choices=[
            "sphere", "cube", "wall", "threshold", "l_shape",
            "u_shape", "cross", "point_cluster",
        ],
        default="sphere",
        help="2D projection shape for the injected ghost obstacle",
    )
    parser.add_argument("--ghost-wall-length-multiplier", type=float, default=6.0,
                        help="wall length is 2 * ghost_radius * this multiplier")
    parser.add_argument("--ghost-wall-width-multiplier", type=float, default=1.0,
                        help="wall width is ghost_radius * this multiplier")
    parser.add_argument("--ghost-point-cluster-count", type=int, default=7,
                        help="number of small obstacle blocks in point_cluster mode")
    parser.add_argument("--ghost-point-cluster-spread-multiplier", type=float, default=3.0,
                        help="point cluster spread radius is ghost_radius * this multiplier")
    parser.add_argument("--ghost-point-cluster-block-multiplier", type=float, default=0.5,
                        help="point cluster block size is ghost_radius * this multiplier")
    parser.add_argument("--random-time-range", type=parse_float_triplet, default=(0.0, 1.25, 2.5),
                        help="three values are accepted for convenience; min and max are used")
    parser.add_argument("--seed", type=int, default=None)
    return parser


def run_ghostfuzz():
    # zsh/bash line continuation can accidentally pass whitespace-only arguments
    # when a backslash has trailing spaces; ignore them so batch commands keep running.
    args = build_arg_parser().parse_args([arg for arg in sys.argv[1:] if arg.strip()])
    if args.density_compare:
        run_density_comparison(args)
        return

    temp_params = Params()
    ghost_radius, radius_values, radius_level = radius_from_args(args)
    alpha, alpha_level = alpha_from_args(args)
    random_time_range = (args.random_time_range[0], args.random_time_range[-1])

    master_seed = int(args.seed if args.seed is not None else time.time())
    master_rng = np.random.RandomState(master_seed)

    folder_time = strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
    batch_res_path = (
        f"data_collection/{args.mode}_ghostfuzz_"
        f"{args.ghost_shape}_rL{radius_level}_aL{alpha_level}_"
        f"vis{args.ghost_visible_count}_{folder_time}/"
    )
    makedirs(batch_res_path, exist_ok=True)

    profile = None
    density_profile_file = ""
    density_threshold = None
    if args.mode == "ghost":
        profile = calibrate_density_profile(args, batch_res_path, master_rng)
        density_profile_file = write_density_profile(profile, batch_res_path)
        density_threshold = alpha * profile["b_max"]

    print(f"[{strftime('%H:%M:%S')}] {args.mode} GhostFuzz 开始，保存至: {batch_res_path}")
    print("-" * 50)

    total_collisions = 0
    total_successes = 0
    total_rrt_failed = 0
    total_timeouts = 0
    total_injection_missed = 0
    crashed_episodes = []
    collision_types = {"drone_drone": 0, "drone_obs": 0}
    episode_results = []

    for ep in range(1, args.episodes + 1):
        print(f"正在推演第 {ep:02d}/{args.episodes} 轮...", end="\r")
        episode_seed = int(master_rng.randint(1, 100000000))
        ep_rng = np.random.RandomState(episode_seed)
        controller = GhostController(
            mode=args.mode,
            params=temp_params,
            rng=ep_rng,
            ghost_radius=ghost_radius,
            alpha=alpha,
            density_threshold=density_threshold,
            random_time_range=random_time_range,
            random_radius_range=(min(radius_values), max(radius_values)),
            num_robots=temp_params.num_robots,
            visible_count=args.ghost_visible_count,
            ghost_shape=args.ghost_shape,
            wall_length_multiplier=args.ghost_wall_length_multiplier,
            wall_width_multiplier=args.ghost_wall_width_multiplier,
            point_cluster_count=args.ghost_point_cluster_count,
            point_cluster_spread_multiplier=args.ghost_point_cluster_spread_multiplier,
            point_cluster_block_multiplier=args.ghost_point_cluster_block_multiplier,
        )
        sim = BaselineSimulation(
            episode_id=ep,
            batch_folder=batch_res_path,
            seed=episode_seed,
            fuzz_params={},
            ghost_controller=controller,
            live_density_plot=args.live_density_plot,
            save_density_curve=args.save_density_curve,
            density_moving_average_window=args.density_moving_average_window,
        )
        result = sim.start()
        completion_time_score = clamp(safe_float(result.get("tick")) / 300, 0.0, 1.0)
        result.update({
            "heuristic_phase": args.mode,
            "seed_source": "density_threshold" if args.mode == "ghost" else "random",
            "parent_pool_id": "",
            "mutation_applied": False,
            "badness_score": 1.0 if result["status"] == "collision" else completion_time_score,
            "formation_quality_score": "",
            "completion_time_score": round(completion_time_score, 6),
            "formation_badness": "",
            "entered_seed_pool": False,
            "seed_pool_reason": "ghostfuzz_no_seed_pool",
            "pool_id": "",
        })
        episode_results.append(result)

        if result["status"] == "success":
            total_successes += 1
        elif result["status"] == "collision":
            total_collisions += 1
            crashed_episodes.append(ep)
            collision_types[result["collision_type"]] += 1
        elif result["status"] == "rrt_failed":
            total_rrt_failed += 1
        elif result["status"] == "timeout":
            total_timeouts += 1
        elif result["status"] == "injection_missed":
            total_injection_missed += 1

    print(" " * 60, end="\r")

    csv_file = os.path.join(batch_res_path, "results.csv")
    csv_columns = [
        "row_type", "episode", "status", "success", "collision", "collision_type", "tick",
        "final_distance_to_goal", "runtime_seconds", "global_planning_failed", "rrt_plan_seconds",
        "min_interrobot_distance", "min_obstacle_distance", "mean_formation_deviation",
        "max_formation_deviation", "total_path_length", "seed",
        "latency_frames", "jitter_frames", "noise_std", "fault_start_step",
        "fault_duration_steps", "fuzz_strength",
        "ghost_injected", "ghost_active", "ghost_inject_tick", "ghost_release_tick",
        "ghost_target_robot_id", "ghost_visible_count_requested", "ghost_visible_count_actual",
        "ghost_visible_robot_ids", "ghost_shape", "ghost_component_count",
        "ghost_radius", "ghost_wall_length", "ghost_wall_width",
        "ghost_point_cluster_count", "ghost_point_cluster_spread",
        "ghost_point_cluster_block_size", "ghost_position_x", "ghost_position_y",
        "ghost_direction_x", "ghost_direction_y",
        "ghost_alpha", "ghost_density_threshold", "ghost_trigger_density", "ghost_inject_density",
        "heuristic_phase", "seed_source", "parent_pool_id", "mutation_applied",
        "badness_score", "formation_quality_score", "completion_time_score",
        "formation_badness", "entered_seed_pool", "seed_pool_reason", "pool_id",
        "collision_log", "detail_log_json", "crash_image", "density_curve_image", "note",
        "total_episodes", "success_count", "collision_count", "drone_drone_collision_count",
        "drone_obs_collision_count", "rrt_failed_count", "timeout_count", "injection_missed_count",
        "collision_rate", "success_rate", "completion_time_median", "completion_time_mean",
        "influence_radius", "crash_uav_dist", "inflation_radius",
    ]
    numeric_summary_fields = [
        "runtime_seconds", "rrt_plan_seconds", "min_interrobot_distance",
        "min_obstacle_distance", "mean_formation_deviation", "max_formation_deviation",
        "total_path_length", "ghost_radius", "ghost_inject_tick", "ghost_release_tick",
        "ghost_wall_length", "ghost_wall_width",
        "ghost_component_count", "ghost_point_cluster_count",
        "ghost_point_cluster_spread", "ghost_point_cluster_block_size",
        "ghost_trigger_density", "ghost_inject_density", "badness_score", "completion_time_score",
    ]

    with open(csv_file, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=csv_columns)
        writer.writeheader()
        for result in episode_results:
            row = {key: "" for key in csv_columns}
            row.update({key: result.get(key, "") for key in csv_columns})
            row["row_type"] = "EPISODE"
            writer.writerow(row)

        summary_row = {key: "" for key in csv_columns}
        for field in numeric_summary_fields:
            values = [result.get(field) for result in episode_results]
            summary_row[field] = median_or_empty(values)
        completion_ticks = [
            result["tick"] for result in episode_results
            if result["status"] == "success" and result.get("tick") not in ("", None)
        ]
        completion_time_median = median_or_empty(completion_ticks)
        completion_time_mean = mean_or_empty(completion_ticks)
        summary_row.update({
            "row_type": "SUMMARY",
            "status": "summary",
            "global_planning_failed": total_rrt_failed,
            "total_episodes": args.episodes,
            "success_count": total_successes,
            "collision_count": total_collisions,
            "drone_drone_collision_count": collision_types["drone_drone"],
            "drone_obs_collision_count": collision_types["drone_obs"],
            "rrt_failed_count": total_rrt_failed,
            "timeout_count": total_timeouts,
            "injection_missed_count": total_injection_missed,
            "collision_rate": round(total_collisions / args.episodes, 6),
            "success_rate": round(total_successes / args.episodes, 6),
            "completion_time_median": completion_time_median,
            "completion_time_mean": completion_time_mean,
            "influence_radius": temp_params.influence_radius,
            "crash_uav_dist": temp_params.crash_uav_dist,
            "inflation_radius": temp_params.inflation_radius,
            "ghost_radius": ghost_radius,
            "ghost_shape": args.ghost_shape,
            "ghost_wall_length": (
                round(float(2.0 * ghost_radius * args.ghost_wall_length_multiplier), 6)
                if args.ghost_shape == "wall" else ""
            ),
            "ghost_wall_width": (
                round(float(ghost_radius * args.ghost_wall_width_multiplier), 6)
                if args.ghost_shape == "wall" else ""
            ),
            "ghost_component_count": len(ghost_polygons(
                np.array([0.0, 0.0]),
                ghost_radius,
                args.ghost_shape,
                np.array([1.0, 0.0]),
                args.ghost_wall_length_multiplier,
                args.ghost_wall_width_multiplier,
                args.ghost_point_cluster_count,
                args.ghost_point_cluster_spread_multiplier,
                args.ghost_point_cluster_block_multiplier,
            )),
            "ghost_point_cluster_count": (
                args.ghost_point_cluster_count if args.ghost_shape == "point_cluster" else ""
            ),
            "ghost_point_cluster_spread": (
                round(float(ghost_radius * args.ghost_point_cluster_spread_multiplier), 6)
                if args.ghost_shape == "point_cluster" else ""
            ),
            "ghost_point_cluster_block_size": (
                round(float(ghost_radius * args.ghost_point_cluster_block_multiplier), 6)
                if args.ghost_shape == "point_cluster" else ""
            ),
            "ghost_alpha": alpha,
            "ghost_visible_count_requested": args.ghost_visible_count,
            "ghost_visible_count_actual": min(args.ghost_visible_count, temp_params.num_robots),
            "ghost_density_threshold": "" if density_threshold is None else round(float(density_threshold), 6),
            "note": (
                f"master_seed={master_seed}; fuzz_mode={args.mode}; "
                f"ghost_radius_level={radius_level}; ghost_radius_values={radius_values}; "
                f"alpha_level={alpha_level}; alpha_values={parse_float_triplet(args.alpha_values)}; "
                f"ghost_visible_count={args.ghost_visible_count}; "
                f"ghost_shape={args.ghost_shape}; "
                f"ghost_wall_length_multiplier={args.ghost_wall_length_multiplier}; "
                f"ghost_wall_width_multiplier={args.ghost_wall_width_multiplier}; "
                f"ghost_point_cluster_count={args.ghost_point_cluster_count}; "
                f"ghost_point_cluster_spread_multiplier={args.ghost_point_cluster_spread_multiplier}; "
                f"ghost_point_cluster_block_multiplier={args.ghost_point_cluster_block_multiplier}; "
                f"baseline_episodes={args.baseline_episodes}; random_time_range={random_time_range}; "
                f"completion_time_summary=median_and_mean"
            ),
        })
        writer.writerow(summary_row)

    seed_pool_file = os.path.join(batch_res_path, "seed_pool.csv")
    with open(seed_pool_file, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=["note"])
        writer.writeheader()
        writer.writerow({"note": "GhostFuzz does not use a mutation seed pool."})

    heuristic_log_file = os.path.join(batch_res_path, "heuristic_events.csv")
    with open(heuristic_log_file, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "episode", "mode", "ghost_injected", "ghost_inject_tick",
            "ghost_target_robot_id", "ghost_visible_count_requested",
            "ghost_visible_count_actual", "ghost_visible_robot_ids",
            "ghost_shape", "ghost_component_count", "ghost_radius",
            "ghost_wall_length", "ghost_wall_width",
            "ghost_point_cluster_count", "ghost_point_cluster_spread",
            "ghost_point_cluster_block_size",
            "ghost_alpha",
            "ghost_density_threshold", "status",
        ])
        writer.writeheader()
        for result in episode_results:
            writer.writerow({
                "episode": result["episode"],
                "mode": args.mode,
                "ghost_injected": result["ghost_injected"],
                "ghost_inject_tick": result["ghost_inject_tick"],
                "ghost_target_robot_id": result["ghost_target_robot_id"],
                "ghost_visible_count_requested": result["ghost_visible_count_requested"],
                "ghost_visible_count_actual": result["ghost_visible_count_actual"],
                "ghost_visible_robot_ids": result["ghost_visible_robot_ids"],
                "ghost_shape": result["ghost_shape"],
                "ghost_component_count": result["ghost_component_count"],
                "ghost_radius": result["ghost_radius"],
                "ghost_wall_length": result["ghost_wall_length"],
                "ghost_wall_width": result["ghost_wall_width"],
                "ghost_point_cluster_count": result["ghost_point_cluster_count"],
                "ghost_point_cluster_spread": result["ghost_point_cluster_spread"],
                "ghost_point_cluster_block_size": result["ghost_point_cluster_block_size"],
                "ghost_alpha": result["ghost_alpha"],
                "ghost_density_threshold": result["ghost_density_threshold"],
                "status": result["status"],
            })

    wall_size_text = ""
    if args.ghost_shape == "wall":
        wall_size_text = (
            f"幽灵墙体尺寸 (Wall LxW) : "
            f"{round(2.0 * ghost_radius * args.ghost_wall_length_multiplier, 6)} x "
            f"{round(ghost_radius * args.ghost_wall_width_multiplier, 6)} m\n"
        )
    component_count = len(ghost_polygons(
        np.array([0.0, 0.0]),
        ghost_radius,
        args.ghost_shape,
        np.array([1.0, 0.0]),
        args.ghost_wall_length_multiplier,
        args.ghost_wall_width_multiplier,
        args.ghost_point_cluster_count,
        args.ghost_point_cluster_spread_multiplier,
        args.ghost_point_cluster_block_multiplier,
    ))
    point_cluster_text = ""
    if args.ghost_shape == "point_cluster":
        point_cluster_text = (
            f"点云簇小块数量 : {args.ghost_point_cluster_count}\n"
            f"点云簇散布半径 : "
            f"{round(ghost_radius * args.ghost_point_cluster_spread_multiplier, 6)} m\n"
            f"点云簇小块尺寸 : "
            f"{round(ghost_radius * args.ghost_point_cluster_block_multiplier, 6)} m\n"
        )

    conclusion_text = (
        f"=============== 事后总结报告 ===============\n"
        f"总计推演轮次 : {args.episodes}\n"
        f"成功到达终点 : {total_successes}\n"
        f"RRT 寻路失败 : {total_rrt_failed}\n"
        f"超时未到达 : {total_timeouts}\n"
        f"随机注入未发生即完成 : {total_injection_missed}\n"
        f"任务完成时间中位数 : {completion_time_median} ticks\n"
        f"任务完成时间平均值 : {completion_time_mean} ticks\n"
        f"感知半径 (Influence Radius) : {temp_params.influence_radius} m\n"
        f"碰撞半径 (Crash Distance)   : {temp_params.crash_uav_dist} m\n"
        f"障碍物膨胀 (C-Space Buffer) : {temp_params.inflation_radius} m\n"
        f"幽灵障碍物形状 (Ghost Shape) : {args.ghost_shape}\n"
        f"幽灵障碍物组件数 : {component_count}\n"
        f"幽灵障碍物半径 (Ghost Radius) : {ghost_radius} m\n"
        f"{wall_size_text}"
        f"{point_cluster_text}"
        f"碰撞阈值 Alpha : {alpha}\n"
        f"Ghost 可见无人机数量 : {min(args.ghost_visible_count, temp_params.num_robots)}"
        f" / 请求 {args.ghost_visible_count}\n"
        f"总计碰撞次数 : {total_collisions}\n"
        f"整体碰撞概率 : {(total_collisions / args.episodes) * 100:.2f}%\n"
        f"Fuzz Mode : {args.mode}\n"
        f"Master Seed : {master_seed}\n"
        f"--------------------------------------------\n"
        f"其中 机-机 相撞次数 : {collision_types['drone_drone']}\n"
        f"其中 机-墙 相撞次数 : {collision_types['drone_obs']}\n"
        f"--------------------------------------------\n"
    )
    if profile:
        conclusion_text += (
            f"Baseline 成功轮次 : {profile['baseline_success_count']}/{args.baseline_episodes}\n"
            f"Baseline 任务时间中位数 : {profile['baseline_completion_time_median']} ticks\n"
            f"Baseline 探索度中位数 : {profile['baseline_coverage_median']}\n"
            f"Baseline B_max : {profile['b_max']:.6f}\n"
            f"Ghost 触发阈值 : {density_threshold:.6f}\n"
            f"密度桶宽 : {profile['bucket_seconds']} s\n"
            f"--------------------------------------------\n"
        )
    else:
        conclusion_text += (
            f"随机模式：幽灵半径在 {min(radius_values)}-{max(radius_values)} m 内随机；"
            f"注入时间在 {random_time_range[0]}-{random_time_range[1]} s 内随机；"
            f"注入无人机随机。\n"
            f"若任务完成时仍未到达注入时刻，本轮标记为 injection_missed。\n"
            f"--------------------------------------------\n"
        )
    if total_collisions > 0:
        conclusion_text += f"发生碰撞的具体轮次 :\n{crashed_episodes}\n"
    else:
        conclusion_text += "完美！GhostFuzz 本批次未触发碰撞。\n"
    conclusion_text += f"\n* 每轮结构化结果 CSV 已保存至 :\n{csv_file}\n"
    conclusion_text += f"* 种子池 CSV 已保存至 :\n{seed_pool_file}\n"
    conclusion_text += f"* 启发式事件日志已保存至 :\n{heuristic_log_file}\n"
    if density_profile_file:
        conclusion_text += f"* Baseline 密度剖面已保存至 :\n{density_profile_file}\n"
    conclusion_text += f"* 碰撞日志和现场截图已保存至 :\n{batch_res_path}\n"
    conclusion_text += "============================================\n"

    conclusion_file = os.path.join(batch_res_path, "conclusion.txt")
    with open(conclusion_file, "w", encoding="utf-8") as f:
        f.write(conclusion_text)
    print(conclusion_text)


if __name__ == '__main__':
    run_ghostfuzz()
    sys.exit(0)

    FUZZ_MODE = "random"  # "heuristic" or "random"
    TOTAL_EPISODES = 400
    WARMUP_RATIO = 0.20
    RANDOM_EXPLORATION_PROB = 0.15
    MUTATION_RATIO = 0.10
    SEED_POOL_BADNESS_THRESHOLD = 0.35
    FORMATION_REFERENCE = 1.0
    FORMATION_QUALITY_POOL_THRESHOLD = 0.55
    COMPLETION_TIME_POOL_THRESHOLD = 0.75
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
