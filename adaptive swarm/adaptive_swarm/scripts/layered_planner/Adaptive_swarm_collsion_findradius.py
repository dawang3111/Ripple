import sys, os
from os import makedirs
from time import strftime
import time
import math
import numpy as np
from numpy.linalg import norm
import matplotlib
import matplotlib.pyplot as plt
from svgpath2mpl import parse_path
from matplotlib.path import Path  # 用于检测无人机是否进入障碍物多边形内部

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
        self.maxiters = 500
        self.goal_prob = 0.05
        self.minDistGoal = 0.25
        self.extension = 0.8
        self.world_bounds_x = [-2.5, 2.5]
        self.world_bounds_y = [-2.5, 2.5]
        self.drone_vel = 4.0
        self.ViconRate = 100
        self.influence_radius = 0.15  # 感知半径
        self.crash_uav_dist = 0.15  # <--- 新增：统一在这里管理机机碰撞半径
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
        """ 核心的 APF 局部避障算法 """
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


class BaselineSimulation:
    def __init__(self, episode_id, batch_folder):
        self.start_time = time.time()
        self.time_step = 0
        self.episode_id = episode_id
        self.save_folder_path = batch_folder

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

        self.robots = [Robot(i + 1) for i in range(self.params.num_robots)]
        self.robot1 = self.robots[0]
        self.robot1.leader = True

    def check_collisions(self):
        """ 检测无人机互撞或撞墙 """
        # <--- 从 Params 类中读取碰撞半径 --->
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
        # 静默 RRT 规划，减少控制台输出杂音
        P_long = rrt_path(self.obstacles, self.xy_start, self.xy_goal, self.params)
        P = ShortenPath(P_long, self.obstacles, smoothiters=50)
        traj_global = waypts2setpts(P, self.params)
        P = np.vstack([P, self.xy_start])

        sp_ind = 0
        self.robot1.route = np.array([traj_global[0, :]])
        self.robot1.sp = self.robot1.route[-1, :]

        followers_sp = formation(self.params.num_robots, leader_des=self.robot1.sp, v=np.array([0, -1]),
                                 l=self.params.interrobots_dist)
        for i in range(len(followers_sp)):
            self.robots[i + 1].sp = followers_sp[i]
            self.robots[i + 1].route = np.array([followers_sp[i]])

        # 核心仿真循环 (最大 300 轮)
        while self.time_step < 300:
            dist_to_goal = norm(self.robot1.sp - self.xy_goal)
            if dist_to_goal < self.params.goal_tolerance:
                # 成功到达终点，未发生碰撞
                return False, None

            self.robot1.sp_global = traj_global[sp_ind, :]
            self.robot1.local_planner(np.array(self.obstacles), self.params)

            followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global,
                                            v=normalize(self.robot1.sp_global - self.robot1.sp),
                                            l=self.params.interrobots_dist)
            for j in range(len(followers_sp_global)):
                self.robots[j + 1].sp_global = followers_sp_global[j]

            for p in range(len(followers_sp)):
                robots_obstacles_sp = [x for k, x in enumerate(followers_sp + [self.robot1.sp]) if k != p]
                robots_obstacles = poses2polygons(robots_obstacles_sp)
                obstacles1 = np.array(self.obstacles + robots_obstacles)
                self.robots[p + 1].local_planner(obstacles1, self.params)
                followers_sp[p] = self.robots[p + 1].sp

            # <--- 碰撞检测 --->
            collision_type, collision_info = self.check_collisions()
            if collision_type:
                info_file = os.path.join(self.save_folder_path, f"Ep{self.episode_id}_collision_info.txt")
                with open(info_file, "w", encoding="utf-8") as f:
                    f.write(f"轮次 (Episode): {self.episode_id}\n")
                    f.write(f"碰撞发生时间步 (Tick): {self.time_step}\n")
                    f.write(
                        f"碰撞类型: {'无人机相撞 (drone_drone)' if collision_type == 'drone_drone' else '无人机撞墙 (drone_obs)'}\n\n")
                    f.write("【碰撞瞬间所有无人机坐标】\n")
                    for r in self.robots:
                        f.write(f"  无人机 {r.id}: [{r.sp[0]:.4f}, {r.sp[1]:.4f}]\n")

                    f.write("\n【涉事对象】\n")
                    if collision_type == "drone_drone":
                        r1, r2 = collision_info
                        f.write(f"  无人机 {r1.id} 与 无人机 {r2.id}\n")
                    elif collision_type == "drone_obs":
                        r, obs = collision_info
                        f.write(f"  无人机 {r.id} 撞击障碍物\n")

                # 生成“遗照”
                plt.figure(num=1, figsize=(10, 10))
                plt.cla()
                self.visualize2D(traj_global, sp_ind, P)
                frame_path = os.path.join(self.save_folder_path,
                                          f"Ep{self.episode_id}_CRASH_tick_{self.time_step:03d}.jpg")
                plt.savefig(frame_path, dpi=100)
                plt.close('all')

                # 发生碰撞，返回 True
                return True, collision_type

            # 计算集群中心点
            centroid = sum([robot.sp for robot in self.robots]) / len(self.robots)

            if sp_ind < traj_global.shape[0] - 1 and norm(self.robot1.sp_global - centroid) < self.params.max_sp_dist:
                sp_ind += 1

            self.time_step += 1

        # 300 轮跑完没到终点，也没碰撞 (算作超时/局部极小值死锁)
        return False, None

    def visualize2D(self, traj_global, sp_ind, P):
        """ 仅在发生碰撞时被调用的画图函数 """
        smiley = parse_path(
            """M458 2420 c-215 -38 -368 -257 -329 -469 34 -182 175 -314 354 -329 l57 -4 0 45 0 44 -42 7 c-101 16 -187 79 -236 171 -37 69 -38 187 -4 257 30 60 90 120 150 150 70 34 188 33 258 -4 89 -47 153 -136 169 -235 l7 -43 50 0 51 0 -6 59 c-13 147 -124 285 -268 334 -60 20 -152 28 -211 17z M1940 2417 c-172 -39 -302 -181 -317 -347 l-6 -60 51 0 50 0 12 52 c14 70 49 126 110 181 118 106 284 100 399 -14 64 -64 86 -120 86 -214 0 -67 -5 -88 -27 -130 -49 -92 -135 -155 -236 -171 l-42 -7 0 -49 0 -50 58 4 c115 8 242 91 306 200 36 61 59 177 51 248 -30 244 -260 410 -495 357z M506 2038 c-9 -12 -16 -41 -16 -64 0 -39 11 -56 158 -240 87 -110 161 -205 166 -212 5 -9 10 -382 6 -494 0 -3 -74 -97 -165 -208 l-165 -202 0 -52 c0 -68 18 -86 86 -86 40 0 55 5 80 28 17 15 112 89 211 166 l180 138 239 0 239 -1 209 -165 c203 -162 210 -166 256 -166 60 0 80 20 80 81 0 43 -8 55 -170 264 l-170 220 0 230 c0 202 2 233 18 257 9 15 86 108 170 208 l152 180 0 54 c0 65 -19 86 -76 86 -36 0 -58 -15 -234 -151 -107 -83 -205 -158 -217 -166 -19 -12 -67 -15 -260 -15 l-238 1 -209 165 -209 166 -53 0 c-43 0 -56 -4 -68 -22z M415 926 c-199 -63 -321 -258 -286 -457 31 -179 161 -309 340 -340 75 -14 171 1 248 37 116 55 209 188 220 314 l6 60 -49 0 -49 0 -17 -70 c-20 -84 -62 -147 -123 -188 -154 -102 -363 -44 -446 124 -35 72 -34 189 3 259 49 92 135 155 236 171 l42 7 0 48 0 49 -42 -1 c-24 0 -61 -6 -83 -13z M2020 882 l0 -50 43 -7 c99 -16 188 -80 235 -169 22 -43 27 -64 27 -131 0 -98 -23 -155 -90 -219 -177 -172 -471 -67 -511 183 l-7 41 -50 0 -50 0 6 -60 c11 -126 102 -257 218 -314 251 -123 542 26 590 303 39 221 -132 448 -351 468 l-60 6 0 -51z""")
        smiley.vertices -= smiley.vertices.mean(axis=0)

        draw_map(self.obstacles)

        for i, robot in enumerate(self.robots):
            plt.plot(robot.sp[0], robot.sp[1], marker=smiley, color='blue', markersize=10, zorder=15)
            plt.annotate(str(i + 1), xy=(robot.sp[0], robot.sp[1]), xytext=(robot.sp[0] + 0.05, robot.sp[1] + 0.05))

            theta = np.linspace(0, 2 * np.pi, 100)
            x = np.cos(theta) * self.params.influence_radius + robot.sp[0]
            y = np.sin(theta) * self.params.influence_radius + robot.sp[1]
            plt.plot(x, y, color='red', alpha=0.5)
            plt.fill(x, y, color='green', alpha=0.1)

        plt.plot(P[:, 0], P[:, 1], linewidth=3, color='orange')
        plt.plot(traj_global[sp_ind, 0], traj_global[sp_ind, 1], marker='o', color='blue', markersize=7)
        plt.plot(self.xy_start[0], self.xy_start[1], marker='o', color='red', markersize=5)
        plt.plot(self.xy_goal[0], self.xy_goal[1], marker='o', color='green', markersize=5)

        # 标注属于哪一个大轮，方便排查
        plt.text(2.0, 2.3, f"Ep: {self.episode_id} | Tick: {self.time_step}", fontsize=12,
                 bbox=dict(facecolor='white', alpha=0.8))

        plt.xlim([-2.6, 2.6])
        plt.ylim([-2.6, 2.6])


if __name__ == '__main__':
    TOTAL_EPISODES = 50

    # 临时实例化 Params 以获取阈值进行命名
    temp_params = Params()
    inf_r = temp_params.influence_radius
    cra_r = temp_params.crash_uav_dist

    # 创建批处理专用的统一文件夹，将感知半径和碰撞半径拼接入命名中
    folder_time = strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
    batch_res_path = f'data_collection/batch_test_inf{inf_r}_cra{cra_r}_{folder_time}/'
    makedirs(batch_res_path, exist_ok=True)

    print(f"[{strftime('%H:%M:%S')}] 批处理开始，数据保存至: {batch_res_path}")
    print("-" * 50)

    # 统计数据
    total_collisions = 0
    crashed_episodes = []
    collision_types = {"drone_drone": 0, "drone_obs": 0}

    # 执行 50 个轮次
    for ep in range(1, TOTAL_EPISODES + 1):
        print(f"正在推演第 {ep:02d}/{TOTAL_EPISODES} 轮...", end="\r")

        sim = BaselineSimulation(episode_id=ep, batch_folder=batch_res_path)

        # start() 现在只计算不画图，执行速度极快
        has_crashed, c_type = sim.start()

        if has_crashed:
            total_collisions += 1
            crashed_episodes.append(ep)
            collision_types[c_type] += 1

    # 清理控制台加载提示
    print(" " * 50, end="\r")

    # <--- 构建事后总结报告字符串 --->
    conclusion_text = (
        f"=============== 事后总结报告 ===============\n"
        f"总计推演轮次 : {TOTAL_EPISODES}\n"
        f"感知半径 (Influence Radius) : {inf_r} m\n"
        f"碰撞半径 (Crash Distance)   : {cra_r} m\n"
        f"总计碰撞次数 : {total_collisions}\n"
        f"整体碰撞概率 : {(total_collisions / TOTAL_EPISODES) * 100:.2f}%\n"
        f"--------------------------------------------\n"
        f"其中 机-机 相撞次数 : {collision_types['drone_drone']}\n"
        f"其中 机-墙 相撞次数 : {collision_types['drone_obs']}\n"
        f"--------------------------------------------\n"
    )

    if total_collisions > 0:
        conclusion_text += f"发生碰撞的具体轮次 :\n{crashed_episodes}\n"
        conclusion_text += f"\n* 详细的坐标数据和现场截图已保存至 :\n{batch_res_path}\n"
    conclusion_text += "============================================\n"

    # 输出报告到控制台
    print(conclusion_text)

    # <--- 将报告打包进 conclusion.txt 文件 --->
    conclusion_file = os.path.join(batch_res_path, "conclusion.txt")
    with open(conclusion_file, "w", encoding="utf-8") as f:
        f.write(conclusion_text)

    print(f"【报告已生成】总结文件已存入当前批次文件夹。")