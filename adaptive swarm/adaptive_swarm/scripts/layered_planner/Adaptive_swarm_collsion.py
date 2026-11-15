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
from matplotlib.path import Path  # <--- 用于检测无人机是否进入障碍物多边形内部，专门用来检测是否穿模的库

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
        self.ViconRate = 100  # <--- 补上这行，动捕刷新频率 100Hz
        self.influence_radius = 0.10  # 感知半径，只有在这个感知半径内，才会产生斥力
        self.goal_tolerance = 0.05
        self.num_robots = 4  # 4架无人机
        self.interrobots_dist = 0.3
        # 最大拉扯距离
        self.max_sp_dist = 0.2 * self.drone_vel
        self.dir = 0  # <--- 补上这行，防止底层数据记录函数报错


class Robot:
    def __init__(self, id):
        # 无人机的身份标识
        self.id = id
        # 无人机的具体坐标
        self.sp = np.array([0, 0])
        # 局部规划的牵引目标点，每步飞行，无人机不直接产生向终点的力，而是产生一个每一步的终点，然后向他有一个牵引力
        self.sp_global = np.array([0, 0])
        # 无人机的历史飞行轨迹，包含每一轮的具体坐标
        self.route = np.array([self.sp])
        # 来自牵引目标点的势能
        self.U_a = 0
        # 来自斥力的势能
        self.U_r = 0
        # 总势能
        self.U = 0
        # 领航机有一个局部牵引目标点，由它产生引力，其他无人机的牵引目标点都是以头机为倒三角的跟随无人机应该有的位置
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


class BaselineSimulation:
    def __init__(self):
        self.start_time = time.time()
        self.time_step = 0

        # 创建用于保存每一帧图片的文件夹
        self.res_to_save_folder_path = 'data_collection/baseline_res/'
        self.floder_name = strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
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
            # room，房间的尺寸大小
            np.array([[-2.5, -2.5], [2.5, -2.5], [2.5, -2.47], [-2.5, -2.47]]),
            np.array([[-2.5, 2.47], [2.5, 2.47], [2.5, 2.5], [-2.5, 2.5]]),
            np.array([[-2.5, -2.47], [-2.47, -2.47], [-2.47, 2.47], [-2.5, 2.47]]),
            np.array([[2.47, -2.47], [2.5, -2.47], [2.5, 2.47], [2.47, 2.47]]),
            # obs1 obs[-2] dia
            # np.array([[-2.6, 1.4], [-2.5, 1.4], [-2.5, 1.5], [-2.6, 1.5]]),
        ]

        # 初始化无人机，把第一个无人机设置为领航无人机
        self.robots = [Robot(i + 1) for i in range(self.params.num_robots)]
        self.robot1 = self.robots[0]
        self.robot1.leader = True

    def check_collisions(self):
        """
        判断是否存在无人机互撞，或无人机撞墙。
        返回: collision_type (str), collision_info (tuple)
        """
        # 1. 检查机机碰撞 (阈值设为 0.05 米)，出现了就返回碰撞文字，并且返回碰撞无人机的坐标
        crash_uav_dist = 0.05
        for i in range(len(self.robots)):
            for j in range(i + 1, len(self.robots)):
                if norm(self.robots[i].sp - self.robots[j].sp) < crash_uav_dist:
                    return "drone_drone", (self.robots[i], self.robots[j])

        # 2. 检查机与障碍物碰撞，同理判断是否跟障碍物碰撞
        for robot in self.robots:
            for obs in self.obstacles:
                # 通过path库，将障碍物转化为几何路径
                path = Path(obs)
                # 如果无人机坐标点落入几何路径之中
                if path.contains_point(robot.sp):
                    return "drone_obs", (robot, obs)

        return None, None

    def start(self):
        print("正在进行 RRT 全局路径规划...")
        '''
            调用RRT算法生成一条大概的无人机飞行路径，输入为障碍物坐标，起始无人机的坐标，目标点以及
        '''
        P_long = rrt_path(self.obstacles, self.xy_start, self.xy_goal, self.params)
        '''
            运用捷径法，将RRT路径生成的点中，随机找两个点，形成连线，如果直线不经过障碍物，则用连线取代原本的路径。
        '''
        P = ShortenPath(P_long, self.obstacles, smoothiters=50)
        # 牵引点，将稀疏的牵引点转化为序列
        '''
            牵引点序列，理论上讲原始的RRT只生成起点、终点以及少量拐点。
            要通过插值的方式，让无人机有每一轮的目标点，不然步长bug，够不到下一个点。
            插值的依据是，两个点之间距离为步长，也就是0.4m
        '''
        traj_global = waypts2setpts(P, self.params)
        '''
            将起点放入关键点序列之中，因为这个列表中是倒序的，终点，拐点B，拐点A。
            所以起点放在列表尾部，正好合适。然后画图的时候从终点反向画图
        '''
        P = np.vstack([P, self.xy_start])

        # 初始化牵引点的序号
        sp_ind = 0
        # 将引航无人机的初始位置以及历史轨迹都设置为第一个牵引点
        self.robot1.route = np.array([traj_global[0, :]])
        self.robot1.sp = self.robot1.route[-1, :]

        '''
            初始化阵型，将其他无人机按照引航无人机在下边，间隔为0.3米的方法，决定其他无人机的阵型。
            V为初始的机头，应该朝下，L为初始间距

            formation函数中实现的逻辑为，将初始化无人机群以等边三角形的模式摆放
            。最下边为领头机，倒三角形，初始无人机的数量，决定了倒三角行的大小
        '''
        followers_sp = formation(self.params.num_robots, leader_des=self.robot1.sp, v=np.array([0, -1]),
                                 l=self.params.interrobots_dist)
        # 将上边产生的左边赋值给，每一台无人机，注意不赋值第一台，因为它是头无人机
        for i in range(len(followers_sp)):
            self.robots[i + 1].sp = followers_sp[i]
            self.robots[i + 1].route = np.array([followers_sp[i]])

        print(f"开始飞行，图片将保存在: {self.save_folder_path}")

        plt.figure(num=1, figsize=(10, 10))

        # 核心仿真循环 (最大 300 轮)
        while self.time_step < 300:
            # 1. 检测是否到达终点
            # 因为是坐标，所以相减之后，用正则化求距离，欧式距离
            dist_to_goal = norm(self.robot1.sp - self.xy_goal)
            # 如果距离小于极限值，则判定已经到达终点
            if dist_to_goal < self.params.goal_tolerance:
                print(f"到达终点！共耗时 {self.time_step} 轮。")
                break

            # 2. 计算无人机的下一步坐标 (Adaptive Swarm 纯净逻辑)
            # 按照轮数给领头无人机设置临时终点，就是通过插值插出来的拐点路径
            self.robot1.sp_global = traj_global[sp_ind, :]
            # 通过APF算法，在只考虑静态墙壁的情况下，计算下一轮领头无人机的飞行轨迹
            self.robot1.local_planner(np.array(self.obstacles), self.params)

            # 确定了领头无人机之后，根据formation继续生成倒等边三角形的其他无人机的坐标
            followers_sp_global = formation(self.params.num_robots, self.robot1.sp_global,
                                            v=normalize(self.robot1.sp_global - self.robot1.sp),
                                            l=self.params.interrobots_dist)
            # 把坐标赋值给其他无人机
            for j in range(len(followers_sp_global)):
                self.robots[j + 1].sp_global = followers_sp_global[j]

            """
                跟随无人机有自己的理想位置，但是要考虑ARF存在的斥力，无人机彼此之间，
                无人机和障碍物之间都会存在斥力，飞行过程中的合力不仅是牵引点，还要考虑斥力，所以才不能保持倒三角。

                跟随无人机的牵引点是理论上根据头机现在的位置，产生的跟随无人机理论上应该在的位置，
                这个位置会产生引力，但是还要考虑障碍物和其他无人机的位置，所以永远到不了
            """
            for p in range(len(followers_sp)):
                # 无人机之间的互斥力计算
                robots_obstacles_sp = [x for k, x in enumerate(followers_sp + [self.robot1.sp]) if k != p]
                robots_obstacles = poses2polygons(robots_obstacles_sp)
                obstacles1 = np.array(self.obstacles + robots_obstacles)
                self.robots[p + 1].local_planner(obstacles1, self.params)
                followers_sp[p] = self.robots[p + 1].sp

            # <--- 执行碰撞检测逻辑并输出文件 --->
            # 返回的是碰撞类型跟碰撞信息，如果不碰撞就返回空
            collision_type, collision_info = self.check_collisions()
            if collision_type:
                print(f"\n\n[!!! 碰撞警报 !!!] 发生碰撞，停止迭代！发生在第 {self.time_step} 轮。")

                # 1. 写入轮数与情况
                info_file = os.path.join(self.save_folder_path, "collision_info.txt")
                with open(info_file, "w", encoding="utf-8") as f:
                    f.write(f"碰撞发生轮数: {self.time_step}\n")
                    f.write(
                        f"碰撞类型: {'无人机相撞 (drone_drone)' if collision_type == 'drone_drone' else '无人机撞墙 (drone_obs)'}\n")

                # 2. 写入所有无人机坐标
                all_pos_file = os.path.join(self.save_folder_path, "all_drones_pos.txt")
                with open(all_pos_file, "w", encoding="utf-8") as f:
                    f.write("【碰撞瞬间所有无人机坐标】\n")
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

                # 发生碰撞后，立刻绘制并保存这一瞬间的图像，然后终止
                plt.cla()
                self.visualize2D(traj_global, sp_ind, P)
                plt.pause(0.01)
                frame_path = os.path.join(self.save_folder_path, f"CRASH_frame_{self.time_step:03d}.jpg")
                plt.savefig(frame_path, dpi=100)
                print(f"【碰撞数据已全部保存至文件夹】: {self.save_folder_path}\n")
                break
            # <--- 碰撞检测与文件写入结束 --->

            # 3. 计算集群中心点，也就是集群坐标的中心位置
            centroid = sum([robot.sp for robot in self.robots]) / len(self.robots)

            # 4. 绘图与保存每一帧
            plt.cla()  # 清除上一帧
            # 把算好的坐标画上去
            self.visualize2D(traj_global, sp_ind, P)
            # 👇 --- 有一个暂停，让 Matplotlib 实时刷新画面 --- 👇
            plt.pause(0.01)

            # 保存当前帧
            frame_path = os.path.join(self.save_folder_path, f"frame_{self.time_step:03d}.jpg")
            # 固化为JPG存入硬盘
            plt.savefig(frame_path, dpi=100)

            # 5. 更新目标点索引
            '''
                第一个小于是保证领航无人机的牵引点在我们更新的牵引点列表之中，不会超出，导致数组越界。
                第二个是无人机的重心离牵引点的距离不能大于最大拉扯距离，如果大于了说明有跟随无人机没跟上，那么头机的牵引点序号不会递增。

                如果头机牵引点不递增的话，牵引点不变继续给一样的引力，只是头机牵引点不变而已
                头机和其他跟随无人机继续考虑引力和斥力的合力，直到平均距离离牵引点足够近。
            '''
            if sp_ind < traj_global.shape[0] - 1 and norm(self.robot1.sp_global - centroid) < self.params.max_sp_dist:
                sp_ind += 1

            self.time_step += 1
            print(f"Frame {self.time_step}/300 saved.", end='\r')

        print("\n仿真结束。")
        plt.close('all')

    def visualize2D(self, traj_global, sp_ind, P):
        """ 保持原汁原味的绘图逻辑，去除了绿色攻击机 """
        smiley = parse_path(
            """M458 2420 c-215 -38 -368 -257 -329 -469 34 -182 175 -314 354 -329 l57 -4 0 45 0 44 -42 7 c-101 16 -187 79 -236 171 -37 69 -38 187 -4 257 30 60 90 120 150 150 70 34 188 33 258 -4 89 -47 153 -136 169 -235 l7 -43 50 0 51 0 -6 59 c-13 147 -124 285 -268 334 -60 20 -152 28 -211 17z M1940 2417 c-172 -39 -302 -181 -317 -347 l-6 -60 51 0 50 0 12 52 c14 70 49 126 110 181 118 106 284 100 399 -14 64 -64 86 -120 86 -214 0 -67 -5 -88 -27 -130 -49 -92 -135 -155 -236 -171 l-42 -7 0 -49 0 -50 58 4 c115 8 242 91 306 200 36 61 59 177 51 248 -30 244 -260 410 -495 357z M506 2038 c-9 -12 -16 -41 -16 -64 0 -39 11 -56 158 -240 87 -110 161 -205 166 -212 5 -9 10 -382 6 -494 0 -3 -74 -97 -165 -208 l-165 -202 0 -52 c0 -68 18 -86 86 -86 40 0 55 5 80 28 17 15 112 89 211 166 l180 138 239 0 239 -1 209 -165 c203 -162 210 -166 256 -166 60 0 80 20 80 81 0 43 -8 55 -170 264 l-170 220 0 230 c0 202 2 233 18 257 9 15 86 108 170 208 l152 180 0 54 c0 65 -19 86 -76 86 -36 0 -58 -15 -234 -151 -107 -83 -205 -158 -217 -166 -19 -12 -67 -15 -260 -15 l-238 1 -209 165 -209 166 -53 0 c-43 0 -56 -4 -68 -22z M415 926 c-199 -63 -321 -258 -286 -457 31 -179 161 -309 340 -340 75 -14 171 1 248 37 116 55 209 188 220 314 l6 60 -49 0 -49 0 -17 -70 c-20 -84 -62 -147 -123 -188 -154 -102 -363 -44 -446 124 -35 72 -34 189 3 259 49 92 135 155 236 171 l42 7 0 48 0 49 -42 -1 c-24 0 -61 -6 -83 -13z M2020 882 l0 -50 43 -7 c99 -16 188 -80 235 -169 22 -43 27 -64 27 -131 0 -98 -23 -155 -90 -219 -177 -172 -471 -67 -511 183 l-7 41 -50 0 -50 0 6 -60 c11 -126 102 -257 218 -314 251 -123 542 26 590 303 39 221 -132 448 -351 468 l-60 6 0 -51z""")
        smiley.vertices -= smiley.vertices.mean(axis=0)

        draw_map(self.obstacles)

        # 绘制蓝色无人机及感知半径圈
        for i, robot in enumerate(self.robots):
            plt.plot(robot.sp[0], robot.sp[1], marker=smiley, color='blue', markersize=10, zorder=15)
            plt.annotate(str(i + 1), xy=(robot.sp[0], robot.sp[1]), xytext=(robot.sp[0] + 0.05, robot.sp[1] + 0.05))

            # 画圆
            theta = np.linspace(0, 2 * np.pi, 100)
            x = np.cos(theta) * self.params.influence_radius + robot.sp[0]
            y = np.sin(theta) * self.params.influence_radius + robot.sp[1]
            plt.plot(x, y, color='red', alpha=0.5)
            plt.fill(x, y, color='green', alpha=0.1)

        # 绘制全局路径、起点、终点和当前追踪点
        plt.plot(P[:, 0], P[:, 1], linewidth=3, color='orange')
        plt.plot(traj_global[sp_ind, 0], traj_global[sp_ind, 1], marker='o', color='blue', markersize=7)
        plt.plot(self.xy_start[0], self.xy_start[1], marker='o', color='red', markersize=5)
        plt.plot(self.xy_goal[0], self.xy_goal[1], marker='o', color='green', markersize=5)

        # 右上角标注当前轮数
        plt.text(2.0, 2.3, f"Tick: {self.time_step}", fontsize=12, bbox=dict(facecolor='white', alpha=0.8))

        plt.xlim([-2.6, 2.6])
        plt.ylim([-2.6, 2.6])


if __name__ == '__main__':
    sim = BaselineSimulation()
    sim.start()