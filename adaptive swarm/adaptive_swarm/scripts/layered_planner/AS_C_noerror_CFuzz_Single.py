import sys, os
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
from shapely.geometry import Polygon

# 强制使用 TkAgg 后端，防止 Mac 下绘图崩溃
matplotlib.use('TkAgg')

# 保证能导入上级目录的依赖
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

from adaptive_swarm.scripts.layered_planner.tools import poses2polygons, init_fonts, draw_map, waypts2setpts, formation, \
    normalize
from adaptive_swarm.scripts.layered_planner.potential_fields import grid_map, combined_potential, meters2grid, \
    grid2meters
from adaptive_swarm.scripts.layered_planner.rrt import rrt_path, ShortenPath


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

        self.influence_radius = 0.10
        self.crash_uav_dist = 0.01
        self.inflation_radius = 0.15

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


# <--- 核心修改：支持独立因子与时间窗口的通信中间件 --->
class CommNetwork:
    def __init__(self, comm_params):
        # 故障类型: 'latency', 'jitter', 'noise', 'loss'
        self.fault_type = comm_params.get('fault_type', 'none')
        # 故障强度
        self.magnitude = comm_params.get('magnitude', 0)
        # 干扰开始的帧数
        self.start_step = comm_params.get('start_step', 0)
        # 干扰持续的帧数
        self.duration_steps = comm_params.get('duration_steps', 0)

        self.last_received = {}

    def get_perceived_position(self, receiver: Robot, sender: Robot, current_time: int):
        # 检查当前是否处于“故障窗口期”内
        in_fault_window = (self.start_step <= current_time <= (self.start_step + self.duration_steps))

        if not in_fault_window:
            # 窗口期外，通信完美无瑕，并更新缓存供丢包时使用
            self.last_received[(receiver.id, sender.id)] = sender.sp
            return sender.sp

        # --- 进入故障窗口期，执行单一因子注入 ---
        if self.fault_type == 'loss':
            if np.random.rand() < self.magnitude:  # magnitude 代表丢包率
                return self.last_received.get((receiver.id, sender.id), sender.sp)
            else:
                self.last_received[(receiver.id, sender.id)] = sender.sp
                return sender.sp

        elif self.fault_type == 'latency':
            delay = int(self.magnitude)  # magnitude 代表固定延迟帧数
            history_index = max(0, current_time - delay)
            base_sp = sender.route[history_index] if history_index < len(sender.route) else sender.sp
            self.last_received[(receiver.id, sender.id)] = base_sp
            return base_sp

        elif self.fault_type == 'jitter':
            # magnitude 代表最大抖动范围
            delay = np.random.randint(0, int(self.magnitude) + 1)
            history_index = max(0, current_time - delay)
            base_sp = sender.route[history_index] if history_index < len(sender.route) else sender.sp
            self.last_received[(receiver.id, sender.id)] = base_sp
            return base_sp

        elif self.fault_type == 'noise':
            # magnitude 代表噪声高斯标准差
            noisy_sp = sender.sp + np.random.normal(0, self.magnitude, 2)
            self.last_received[(receiver.id, sender.id)] = noisy_sp
            return noisy_sp

        # 兜底返回真实坐标
        return sender.sp


# <--- 通信中间件修改结束 --->


class BaselineSimulation:
    def __init__(self, episode_id, batch_folder, seed, comm_params):
        self.start_time = time.time()
        self.time_step = 0
        self.episode_id = episode_id
        self.save_folder_path = batch_folder
        self.seed = seed
        self.comm_params = comm_params

        np.random.seed(self.seed)
        random.seed(self.seed)

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

        self.inflated_obstacles = []
        for obs in self.obstacles:
            poly = Polygon(obs)
            inflated_poly = poly.buffer(self.params.inflation_radius, join_style=2)
            coords = np.array(inflated_poly.exterior.coords)
            self.inflated_obstacles.append(coords)

        self.robots = [Robot(i + 1) for i in range(self.params.num_robots)]
        self.robot1 = self.robots[0]
        self.robot1.leader = True

    def check_collisions(self):
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
            P_long = rrt_path(self.inflated_obstacles, self.xy_start, self.xy_goal, self.params)
            P = ShortenPath(P_long, self.inflated_obstacles, smoothiters=50)
            traj_global = waypts2setpts(P, self.params)
            P = np.vstack([P, self.xy_start])
        except Exception as e:
            return False, None, None

        sp_ind = 0
        self.robot1.route = np.array([traj_global[0, :]])
        self.robot1.sp = self.robot1.route[-1, :]

        followers_sp = formation(self.params.num_robots, leader_des=self.robot1.sp, v=np.array([0, -1]),
                                 l=self.params.interrobots_dist)
        for i in range(len(followers_sp)):
            self.robots[i + 1].sp = followers_sp[i]
            self.robots[i + 1].route = np.array([followers_sp[i]])

        self.comm_net = CommNetwork(self.comm_params)

        while self.time_step < 300:
            dist_to_goal = norm(self.robot1.sp - self.xy_goal)
            if dist_to_goal < self.params.goal_tolerance:
                return False, None, None

            self.robot1.sp_global = traj_global[sp_ind, :]
            self.robot1.local_planner(np.array(self.obstacles), self.params)

            followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global,
                                            v=normalize(self.robot1.sp_global - self.robot1.sp),
                                            l=self.params.interrobots_dist)
            for j in range(len(followers_sp_global)):
                self.robots[j + 1].sp_global = followers_sp_global[j]

            for p in range(len(followers_sp)):
                receiver_robot = self.robots[p + 1]
                perceived_neighbors_sp = []

                for other_robot in self.robots:
                    if other_robot.id == receiver_robot.id:
                        continue
                    p_sp = self.comm_net.get_perceived_position(receiver_robot, other_robot, self.time_step)
                    if p_sp is not None:
                        perceived_neighbors_sp.append(p_sp)

                if perceived_neighbors_sp:
                    robots_obstacles = poses2polygons(perceived_neighbors_sp)
                    obstacles1 = np.array(self.obstacles + robots_obstacles)
                else:
                    obstacles1 = np.array(self.obstacles)

                receiver_robot.local_planner(obstacles1, self.params)
                followers_sp[p] = receiver_robot.sp

            collision_type, collision_info = self.check_collisions()
            if collision_type:
                detail_log = ""

                info_file = os.path.join(self.save_folder_path, f"Ep{self.episode_id}_collision_info.txt")
                with open(info_file, "w", encoding="utf-8") as f:
                    f.write(f"轮次 (Episode): {self.episode_id}\n")
                    f.write(f"随机种子 (Seed): {self.seed}\n")
                    f.write(f"碰撞发生时间步 (Tick): {self.time_step}\n")
                    f.write(f"碰撞类型: {'机机相撞' if collision_type == 'drone_drone' else '机墙相撞'}\n\n")

                    f.write("【导致碰撞的隔离通信故障参数】\n")
                    for k, v in self.comm_params.items():
                        f.write(f"  {k}: {v}\n")

                    f.write("\n【碰撞实体精确坐标】\n")
                    if collision_type == "drone_drone":
                        r1, r2 = collision_info
                        f.write(f"  [涉事方] 无人机 {r1.id}: [{r1.sp[0]:.4f}, {r1.sp[1]:.4f}]\n")
                        f.write(f"  [涉事方] 无人机 {r2.id}: [{r2.sp[0]:.4f}, {r2.sp[1]:.4f}]\n")
                        detail_log = f"机机相撞 (UAV {r1.id} & UAV {r2.id})"
                    elif collision_type == "drone_obs":
                        r, obs = collision_info
                        f.write(f"  [坠机者] 无人机 {r.id}: [{r.sp[0]:.4f}, {r.sp[1]:.4f}]\n")
                        f.write(f"  [被撞墙壁多边形顶点坐标]:\n")
                        for pt in obs:
                            f.write(f"    [{pt[0]:.4f}, {pt[1]:.4f}]\n")
                        detail_log = f"机墙相撞 (UAV {r.id} -> 墙壁)"

                    f.write("\n【碰撞瞬间所有无人机坐标 (全局快照)】\n")
                    for r in self.robots:
                        f.write(f"  无人机 {r.id}: [{r.sp[0]:.4f}, {r.sp[1]:.4f}]\n")

                plt.figure(num=1, figsize=(10, 10))
                plt.cla()
                self.visualize2D(traj_global, sp_ind, P)
                frame_path = os.path.join(self.save_folder_path,
                                          f"Ep{self.episode_id}_{self.comm_params['fault_type']}_CRASH.jpg")
                plt.savefig(frame_path, dpi=100)
                plt.close('all')

                return True, collision_type, detail_log

            centroid = sum([robot.sp for robot in self.robots]) / len(self.robots)
            if sp_ind < traj_global.shape[0] - 1 and norm(self.robot1.sp_global - centroid) < self.params.max_sp_dist:
                sp_ind += 1
            self.time_step += 1

        return False, None, None

    def visualize2D(self, traj_global, sp_ind, P):
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
        # 在图上直观印上导致崩溃的故障类型
        plt.text(-2.5, 2.3, f"Fault: {self.comm_params['fault_type'].upper()}", color='red', fontsize=12, weight='bold',
                 bbox=dict(facecolor='white', alpha=0.8))

        plt.xlim([-2.6, 2.6])
        plt.ylim([-2.6, 2.6])


if __name__ == '__main__':
    # <--- Fuzzer 顶层调度器 --->
    fault_factors = ['latency', 'jitter', 'noise', 'loss']
    EPISODES_PER_FACTOR = 100
    total_episodes = len(fault_factors) * EPISODES_PER_FACTOR

    master_seed = int(time.time())
    master_rng = np.random.RandomState(master_seed)

    folder_time = strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
    batch_res_path = f'data_collection/isolated_fuzz_{folder_time}/'
    makedirs(batch_res_path, exist_ok=True)

    print(f"[{strftime('%H:%M:%S')}] 隔离控制变量 Fuzzing 测试开始 (总计 {total_episodes} 轮)")
    print("-" * 50)

    # 统计数据结构
    overall_collisions = 0
    factor_stats = {f: {'runs': 0, 'crashes': 0} for f in fault_factors}
    crashed_logs = []

    episode_counter = 1

    # 核心：依次对每种干扰因素单独进行 Fuzz 测试
    for current_factor in fault_factors:
        print(f"\n>>> 正在进行隔离测试: [{current_factor.upper()}] <<<")

        for i in range(EPISODES_PER_FACTOR):
            print(f"  推演第 {episode_counter:02d}/{total_episodes} 轮...", end="\r")

            current_seed = master_rng.randint(1, 100000)
            ep_rng = np.random.RandomState(current_seed)

            # 随机生成干扰窗口期 (在飞行中途突然遭遇通信故障)
            start_time = ep_rng.randint(20, 150)
            duration_time = ep_rng.randint(20, 100)

            # 根据当前隔离测试的因素，生成变异数值
            magnitude = 0
            if current_factor == 'latency':
                magnitude = ep_rng.randint(5, 20)  # 5-20帧延迟
            elif current_factor == 'jitter':
                magnitude = ep_rng.randint(5, 15)  # 最大15帧抖动
            elif current_factor == 'noise':
                magnitude = round(ep_rng.uniform(0.05, 0.20), 3)  # 5-20cm 噪声
            elif current_factor == 'loss':
                magnitude = round(ep_rng.uniform(0.20, 0.80), 2)  # 20%-80% 剧烈丢包

            test_comm_params = {
                'fault_type': current_factor,
                'magnitude': magnitude,
                'start_step': start_time,
                'duration_steps': duration_time
            }

            sim = BaselineSimulation(episode_id=episode_counter, batch_folder=batch_res_path, seed=current_seed,
                                     comm_params=test_comm_params)
            has_crashed, c_type, detail_log = sim.start()

            factor_stats[current_factor]['runs'] += 1
            if has_crashed:
                overall_collisions += 1
                factor_stats[current_factor]['crashes'] += 1
                log_entry = f"  Ep {episode_counter:02d} | Seed: {current_seed} | {detail_log} | {current_factor.upper()} 强度: {magnitude} | 窗口: {start_time}~{start_time + duration_time}"
                crashed_logs.append(log_entry)

            episode_counter += 1

    print(" " * 60, end="\r")

    # <--- 构建高度结构化的分类总结报告 --->
    conclusion_text = (
        f"=============== Fuzzing 隔离因素总结报告 ===============\n"
        f"总计推演轮次 : {total_episodes}\n"
        f"总计碰撞次数 : {overall_collisions}\n"
        f"整体碰撞概率 : {(overall_collisions / total_episodes) * 100:.2f}%\n"
        f"----------------------------------------------------\n"
        f"【单项因素致死率排名】\n"
    )

    for f in fault_factors:
        runs = factor_stats[f]['runs']
        crashes = factor_stats[f]['crashes']
        rate = (crashes / runs) * 100 if runs > 0 else 0
        conclusion_text += f"  > {f.ljust(10).upper()}: {crashes}/{runs} 崩溃 ({rate:.1f}% 致死率)\n"

    conclusion_text += f"----------------------------------------------------\n"

    if overall_collisions > 0:
        conclusion_text += f"【发生碰撞的具体轮次及故障档案】:\n"
        for log in crashed_logs:
            conclusion_text += log + "\n"
    else:
        conclusion_text += "机群展现出了极强的鲁棒性，扛过了所有独立通信攻击！\n"

    conclusion_text += f"\n* 详细的现场截图已保存至: {batch_res_path}\n"
    conclusion_text += "=======================================================\n"

    print(conclusion_text)

    conclusion_file = os.path.join(batch_res_path, "conclusion.txt")
    with open(conclusion_file, "w", encoding="utf-8") as f:
        f.write(conclusion_text)