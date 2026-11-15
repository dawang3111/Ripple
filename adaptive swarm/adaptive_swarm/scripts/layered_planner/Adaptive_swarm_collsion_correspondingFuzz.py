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
        self.animate_rrt = 0  # 关掉 RRT 动画以节省时间
        self.visualize = 1
        # 最大迭代次数，RRT找500次新枝丫，没到终点就失败
        self.maxiters = 500
        # 每次迭代有0.05的概率，直接是终点，防止枝丫乱飞（是不是存在临时终点，毕竟有障碍物）
        self.goal_prob = 0.05
        # 如果离终点不足0.25m，那么就视为已经到达
        self.minDistGoal = 0.25
        # 每次枝丫延伸的长度只能为0.8m
        self.extension = 0.8
        # 枝丫生成的范围，防止指出地图外
        self.world_bounds_x = [-2.5, 2.5]
        self.world_bounds_y = [-2.5, 2.5]
        self.drone_vel = 4.0
        self.ViconRate = 100  # 动捕刷新频率 100Hz
        self.influence_radius = 0.10  # 感知半径
        self.goal_tolerance = 0.05
        self.num_robots = 4  # 4架无人机
        self.interrobots_dist = 0.3
        # 最大拉扯距离
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
        w = 20  # 平滑窗口
        ax = np.mean(gx[ix - int(w / 2): ix + int(w / 2), iy - int(w / 2): iy + int(w / 2)])
        ay = np.mean(gy[ix - int(w / 2): ix + int(w / 2), iy - int(w / 2): iy + int(w / 2)])

        self.V = params.drone_vel * np.array([ax, ay])
        dt = 0.01 * params.drone_vel / norm([ax, ay]) if norm([ax, ay]) != 0 else 0.01
        self.sp = self.sp + dt * np.array([ax, ay])
        self.route = np.vstack([self.route, self.sp])


# <--- 新增模块：通信网络中间件 --->
class CommNetwork:
    def __init__(self, comm_params):
        self.latency_steps = comm_params.get('latency_steps', 0)
        self.jitter_steps = comm_params.get('jitter_steps', 0)
        self.noise_std = comm_params.get('noise_std', 0.0)
        self.loss_rate = comm_params.get('loss_rate', 0.0)
        self.comm_range = comm_params.get('comm_range', 999.0)

        # 缓存字典，处理丢包时零阶保持
        self.last_received = {}

    def get_perceived_position(self, receiver: Robot, sender: Robot, current_time: int):
        # 5. 窄带通讯/距离限制
        if norm(receiver.sp - sender.sp) > self.comm_range:
            return None

            # 4. 丢包处理
        if np.random.rand() < self.loss_rate:
            return self.last_received.get((receiver.id, sender.id), sender.sp)

        # 1 & 2. 延迟与抖动
        actual_delay = self.latency_steps + np.random.randint(0,
                                                              self.jitter_steps + 1) if self.jitter_steps > 0 else self.latency_steps
        history_index = max(0, current_time - actual_delay)

        if history_index < len(sender.route):
            base_sp = sender.route[history_index]
        else:
            base_sp = sender.sp

        # 3. 噪声注入
        noisy_sp = base_sp + np.random.normal(0, self.noise_std, 2)

        self.last_received[(receiver.id, sender.id)] = noisy_sp
        return noisy_sp


# <--- 通信中间件结束 --->


class BaselineSimulation:
    def __init__(self, episode_idx, seed, comm_params):
        # 记录当前大轮信息
        self.episode_idx = episode_idx
        self.seed = seed
        self.comm_params = comm_params

        # 强制设置全局随机种子，确保本轮的 RRT、通信噪声完全可复现
        np.random.seed(self.seed)
        random.seed(self.seed)

        self.start_time = time.time()
        self.time_step = 0

        # 创建用于保存每一帧图片的文件夹 (文件夹名字包含 Episode)
        self.res_to_save_folder_path = 'data_collection/baseline_res/'
        self.floder_name = f"Ep{self.episode_idx}_" + strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        self.save_folder_path = self.res_to_save_folder_path + self.floder_name + '/'
        makedirs(self.save_folder_path, exist_ok=True)

        # 字体设置
        init_fonts(small=12, medium=16, big=26)
        self.params = Params()
        self.xy_start = np.array([1.2, 1.25])
        self.xy_goal = np.array([2.2, -2.2])

        # 保持与 RSfuzz完全一致的地形和障碍物
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
        # 1. 检查机机碰撞 (阈值设为 0.15 米)
        crash_uav_dist = 0.05
        for i in range(len(self.robots)):
            for j in range(i + 1, len(self.robots)):
                if norm(self.robots[i].sp - self.robots[j].sp) < crash_uav_dist:
                    return "drone_drone", (self.robots[i], self.robots[j])

        # 2. 检查机与障碍物碰撞
        for robot in self.robots:
            for obs in self.obstacles:
                path = Path(obs)
                if path.contains_point(robot.sp):
                    return "drone_obs", (robot, obs)

        return None, None

    def start(self):
        print(f"正在进行 RRT 全局路径规划 (Seed: {self.seed})...")
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

        print(f"开始飞行，大轮 {self.episode_idx} 数据将保存在: {self.save_folder_path}")

        # 初始化通信中间件
        self.comm_net = CommNetwork(self.comm_params)

        plt.figure(num=1, figsize=(10, 10))

        # 核心仿真循环 (最大 300 轮)
        while self.time_step < 300:
            dist_to_goal = norm(self.robot1.sp - self.xy_goal)
            if dist_to_goal < self.params.goal_tolerance:
                print(f"\n[大轮 {self.episode_idx}] 成功到达终点！共耗时 {self.time_step} 轮。")
                break

            self.robot1.sp_global = traj_global[sp_ind, :]
            self.robot1.local_planner(np.array(self.obstacles), self.params)

            followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global,
                                            v=normalize(self.robot1.sp_global - self.robot1.sp),
                                            l=self.params.interrobots_dist)
            for j in range(len(followers_sp_global)):
                self.robots[j + 1].sp_global = followers_sp_global[j]

            # <--- 替换原有逻辑：经过通信层污染的互斥力计算 --->
            for p in range(len(followers_sp)):
                receiver_robot = self.robots[p + 1]
                perceived_neighbors_sp = []

                # 接收者尝试获取所有队友坐标
                for other_robot in self.robots:
                    if other_robot.id == receiver_robot.id:
                        continue

                        # 经过中间件获取被污染的坐标
                    p_sp = self.comm_net.get_perceived_position(receiver_robot, other_robot, self.time_step)
                    if p_sp is not None:
                        perceived_neighbors_sp.append(p_sp)

                # 构建局部感知到的障碍物
                if perceived_neighbors_sp:
                    robots_obstacles = poses2polygons(perceived_neighbors_sp)
                    obstacles1 = np.array(self.obstacles + robots_obstacles)
                else:
                    obstacles1 = np.array(self.obstacles)

                receiver_robot.local_planner(obstacles1, self.params)
                followers_sp[p] = receiver_robot.sp
            # <--- 互斥力计算结束 --->

            # <--- 碰撞检测与文件写入 --->
            collision_type, collision_info = self.check_collisions()
            if collision_type:
                print(f"\n\n[!!! 碰撞警报 !!!] 大轮 {self.episode_idx} 发生碰撞，终止本轮仿真！")

                # 1. 写入全局信息 (增加随机种子和通信参数)
                info_file = os.path.join(self.save_folder_path, "collision_info.txt")
                with open(info_file, "w", encoding="utf-8") as f:
                    f.write(f"仿真大轮 (Episode): {self.episode_idx}\n")
                    f.write(f"随机种子 (Seed): {self.seed}\n")
                    f.write(f"碰撞发生轮数 (Tick): {self.time_step}\n")
                    f.write(
                        f"碰撞类型: {'无人机相撞 (drone_drone)' if collision_type == 'drone_drone' else '无人机撞墙 (drone_obs)'}\n\n")
                    f.write("【通信故障注入参数】\n")
                    f.write(f"延迟 (Latency): {self.comm_params['latency_steps']} 帧\n")
                    f.write(f"抖动 (Jitter): {self.comm_params['jitter_steps']} 帧\n")
                    f.write(f"噪声标准差 (Noise Std): {self.comm_params['noise_std']} 米\n")
                    f.write(f"丢包率 (Loss Rate): {self.comm_params['loss_rate']}\n")
                    f.write(f"通信受限半径 (Comm Range): {self.comm_params['comm_range']} 米\n")

                # 2. 写入所有无人机坐标
                all_pos_file = os.path.join(self.save_folder_path, "all_drones_pos.txt")
                with open(all_pos_file, "w", encoding="utf-8") as f:
                    f.write("【碰撞瞬间所有无人机真实坐标】\n")
                    for r in self.robots:
                        f.write(f"无人机 {r.id}: [{r.sp[0]:.4f}, {r.sp[1]:.4f}]\n")

                # 3. 写入碰撞双方实体坐标
                entities_file = os.path.join(self.save_folder_path, "collision_entities.txt")
                with open(entities_file, "w", encoding="utf-8") as f:
                    if collision_type == "drone_drone":
                        r1, r2 = collision_info
                        f.write("【涉事无人机坐标】\n")
                        f.write(f"无人机 {r1.id}: [{r1.sp[0]:.4f}, {r1.sp[1]:.4f}]\n")
                        f.write(f"无人机 {r2.id}: [{r2.sp[0]:.4f}, {r2.sp[1]:.4f}]\n")
                    elif collision_type == "drone_obs":
                        r, obs = collision_info
                        f.write("【涉事无人机坐标】\n")
                        f.write(f"无人机 {r.id}: [{r.sp[0]:.4f}, {r.sp[1]:.4f}]\n")
                        f.write("【涉事障碍物边界点坐标】\n")
                        for pt in obs:
                            f.write(f"[{pt[0]}, {pt[1]}]\n")

                plt.cla()
                self.visualize2D(traj_global, sp_ind, P)
                plt.pause(0.01)
                frame_path = os.path.join(self.save_folder_path, f"CRASH_frame_{self.time_step:03d}.jpg")
                plt.savefig(frame_path, dpi=100)
                print(f"【碰撞数据已全部保存至文件夹】: {self.save_folder_path}\n")

                # 大轮 break，彻底终止当前 while 循环，准备进入下一个 Episode
                break
            # <--- 碰撞检测结束 --->

            centroid = sum([robot.sp for robot in self.robots]) / len(self.robots)

            plt.cla()
            self.visualize2D(traj_global, sp_ind, P)
            plt.pause(0.01)

            frame_path = os.path.join(self.save_folder_path, f"frame_{self.time_step:03d}.jpg")
            plt.savefig(frame_path, dpi=100)

            if sp_ind < traj_global.shape[0] - 1 and norm(self.robot1.sp_global - centroid) < self.params.max_sp_dist:
                sp_ind += 1

            self.time_step += 1
            print(f"Ep {self.episode_idx} | Frame {self.time_step}/300 saved.", end='\r')

        plt.close('all')

    def visualize2D(self, traj_global, sp_ind, P):
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

        plt.text(2.0, 2.3, f"Ep:{self.episode_idx} | Tick:{self.time_step}", fontsize=12,
                 bbox=dict(facecolor='white', alpha=0.8))

        plt.xlim([-2.6, 2.6])
        plt.ylim([-2.6, 2.6])


# <--- 主程序：自动化大轮测试入口 --->
if __name__ == '__main__':
    # 假设你想跑 10 次大轮测试
    NUM_EPISODES = 10

    # 初始化一个全局的元随机数生成器 (用于决定每轮的种子)
    master_seed = int(time.time())
    master_rng = np.random.RandomState(master_seed)

    for ep in range(1, NUM_EPISODES + 1):
        print(f"\n{'=' * 20} 准备启动第 {ep}/{NUM_EPISODES} 大轮仿真 {'=' * 20}")

        # 1. 生成这大轮的全局唯一种子 (1 ~ 99999)
        current_seed = master_rng.randint(1, 100000)

        # 2. 核心修改：用当前大轮的种子，初始化一个局部的随机数生成器
        # 这样能保证：只要 current_seed 一样，下面抽出来的通信参数绝对一样！
        ep_rng = np.random.RandomState(current_seed)

        # 3. 构造通信劣化参数 (Fuzzer 在设定的边界内自动“抽卡”)
        test_comm_params = {
            # 延迟：0 ~ 20 帧 (最高 0.2 秒延迟)
            'latency_steps': ep_rng.randint(0, 21),

            # 抖动：0 ~ 5 帧 (网络忽快忽慢)
            'jitter_steps': ep_rng.randint(0, 6),

            # 噪声：0 ~ 0.15 米 (GPS 漂移，保留3位小数)
            'noise_std': round(ep_rng.uniform(0.0, 0.15), 3),

            # 丢包率：0% ~ 40% (极恶劣网络环境保留2位小数)
            'loss_rate': round(ep_rng.uniform(0.0, 0.40), 2),

            # 通信距离：0.5米 ~ 5.0米 (模拟近场通讯受限)
            'comm_range': round(ep_rng.uniform(0.5, 5.0), 2)
        }

        print(f"[参数揭晓] 本轮 Seed: {current_seed}")
        print(f"[参数揭晓] 故障组合: {test_comm_params}")

        # 4. 实例化并启动这一大轮
        sim = BaselineSimulation(episode_idx=ep, seed=current_seed, comm_params=test_comm_params)
        sim.start()

        # 等待 1 秒再开启下一轮，防止文件夹名字冲突
        time.sleep(1)

    print("\n所有自动化大轮测试执行完毕。")