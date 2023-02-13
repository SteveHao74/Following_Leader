"""
@作者 : 黄潇龙
@邮箱 : zjuhxl18@163.com 

动态窗口算法
该算法用于移动机器人的局部运动规划，使用障碍物点云信息规划出控制指令，完成到达目标且避障的功能
该算法适用于非完整约束的移动机器人，给出其线速度与角速度

使用示例：
DWA = DynamicWindowApproch()
DWA.config(max_speed, max_yawrate, base)
command = DWA.planning_dynamic_window_approch(pose, velocity, goal, points_cloud)
"""

import numpy as np
import math
import copy
import time


class DynamicWindowApproch():
    """
    动态窗口算法类，包含使用动态窗口算法完成局部运动规划所需要用到的函数
    """

    def __init__(self):
        """
        初始化一些全局参数
        """

    def planning(self, pose, velocity, goal, points_cloud):
        """
        使用动态窗口算法完成局部运动规划

        Args:
            pose (:obj:`list`): 机器人的2D位姿，(x,y,theta)，用list表达
            velocity (:obj:`list`): 机器人的当前速度，[线速度，角速度]，用list表达
            goal (:obj:`list`): 2D局部目标，[x,y],用list表达
            points_cloud (:obj:`list`): 障碍物点云，[[xi,yi]],用list表达

        Returns:
            :obj:`list`: 机器人的最佳速度，[线速度，角速度]，用list表达
        """

        dynamic_window = self.create_dynamic_window(velocity)
        list_V = dynamic_window[0]
        list_W = dynamic_window[1]
        total_cost = 1e6
        best_velocity = [0, 0]
        points_cloud=np.transpose(np.array(points_cloud, np.float32))
        for v in list_V:
            for w in list_W:
                new_velocity = [v, w]
                new_pose = self.motion(pose, new_velocity, self.predict_time)
                velocity_cost = self.calculate_velocity_cost(new_velocity)
                heading_cost = self.calculate_heading_cost(new_pose, goal)
                clearance_cost = self.calculate_clearance_cost(
                    pose, new_velocity, points_cloud)
                cost = velocity_cost+heading_cost+clearance_cost

                if (cost < total_cost):
                    total_cost = cost
                    best_velocity = new_velocity

        return best_velocity

    def calculate_clearance_cost(self, pose, velocity, points_cloud):
        """
        计算当前位姿下选定速度造成的碰撞代价

        Args:
            pose (:obj:`list`): 机器人的2D位姿，(x,y,theta)，用list表达
            velocity (:obj:`list`): 用于计算代价的速度，[线速度，角速度]，用list表达
            points_cloud (:obj:`list`): 障碍物点云，[[xi,yi]],用list表达

        Returns:
            :obj:`double`: 当前位姿下选定速度造成的碰撞代价
        """

        time = 0.0
        min_r = 1e6
        pose_N = pose
        while (time < self.predict_time):
            pose_N = self.motion(pose_N, velocity, self.dt)
            points_num = np.size(points_cloud, 1)

            for i in range(points_num):
                distance = math.hypot(
                    pose_N[0]-points_cloud[0, i], pose_N[1]-points_cloud[1, i])
                if (distance-self.base < 0):
                    return 1e6

                if (distance < min_r):
                    min_r = distance

            time = time+self.dt
        clearance_cost = 1/(min_r+1e-6)
        return clearance_cost

    def calculate_heading_cost(self, pose, goal):
        """
        计算当前位姿下选定速度造成的方向偏差代价

        Args:
            pose (:obj:`list`): 机器人的2D位姿，(x,y,theta)，用list表达

        Returns:
            :obj:`double`: 当前位姿下选定速度造成的方向偏差代价
        """

        dx = goal[0]-pose[0]
        dy = goal[1]-pose[1]
        angle_error = math.atan2(dy, dx)-pose[2]
        heading_cost = math.fabs(math.atan2(
            math.sin(angle_error), math.cos(angle_error)))/3.14
        return heading_cost

    def calculate_velocity_cost(self, velocity):
        """
        计算当前位姿下选定速度造成的速度代价

        Args:
            velocity (:obj:`list`): 用于计算代价的速度，[线速度，角速度]，用list表达

        Returns:
            :obj:`double`: 当前位姿下选定速度造成的速度代价
        """

        velocity_cost = (self.max_speed-velocity[0])/self.max_speed
        return velocity_cost

    def motion(self, pose, velocity, dt):
        """
        使用当前位姿和当前速度进行运动推理，计算下一时刻的位姿

        Args:
            pose (:obj:`list`): 机器人的2D位姿，(x,y,theta)，用list表达
            velocity (:obj:`list`): 用于运动推理的速度，[线速度，角速度]，用list表达

        Returns:
            :obj:`list`: 在当前位姿下以当前速度运动单位时间间隔后的位姿，(x,y,theta)，用list表达
        """

        new_pose = [0, 0, 0]
        new_pose[0] = pose[0]+velocity[0]*math.cos(pose[2])*dt
        new_pose[1] = pose[1]+velocity[0]*math.sin(pose[2])*dt
        new_pose[2] = pose[2]+velocity[1]*dt

        return new_pose

    def create_dynamic_window(self, velocity):
        """
        创建动态窗口

        Args:
            velocity (:obj:`list`): 机器人的当前速度，[线速度，角速度]，用list表达

        Returns:
            :obj:`list`: 用以规划的动态窗口，矩阵的列表示不同的线速度，行表示不同的角速度
        """

        min_V = max(self.min_speed, velocity[0]-self.dt*self.max_accel)
        max_V = min(self.max_speed, velocity[0]+self.dt*self.max_accel)
        min_W = max(-self.max_yawrate, velocity[1]-self.dt*self.max_dyawrate)
        max_W = min(self.max_yawrate, velocity[1]+self.dt*self.max_dyawrate)
        list_V = list(np.arange(min_V, max_V, self.velocity_resolution))
        list_W = list(np.arange(min_W, max_W, self.yawrate_resolution))
        dynamic_window = [list_V, list_W]

        return dynamic_window

    def config(self, max_speed, max_yawrate, base, min_speed=0, max_accel=15.0,
               max_dyawrate=np.radians(110.0), velocity_num=10,
               yawrate_num=10, dt=0.2, predict_time=1,
               heading_coef=10.0, clearance_coef=1.0, velocity_coef=0.1,
               ):
        """
        配置DWA参数
        其中max_speed,max_yawrate,base是必须给出的

        Args:
            max_speed (:obj:`double`): 最大线速度.
            max_yawrate (:obj:`double`): 最大角速度，单位rad.
            base (:obj:`double`): 机器人的半径.
            min_speed (:obj:`double`, optional): 最小线速度. Defaults to 0.
            max_accel (:obj:`double`, optional): 最大线加速度. Defaults to 15.0.
            max_dyawrate (:obj:`double`, optional): 最大角加速度. Defaults to np.radians(110.0).
            velocity_num (:obj:`int`, optional): 线速度等级数. Defaults to 10.
            yawrate_num (:obj:`int`, optional): 角速度等级数. Defaults to 10.
            dt (:obj:`double`, optional): 轨迹预测时间间隔. Defaults to 0.1.
            predict_time (:obj:`double`, optional): 轨迹预测时间. Defaults to 6.0.
            heading_coef (:obj:`double`, optional): 方向偏差的代价权重. Defaults to 3.0.
            clearance_coef (:obj:`double`, optional): 碰撞的代价权重. Defaults to 1.0.
            velocity_coef (:obj:`double`, optional): 速度不足的代价权重. Defaults to 2.0.
        """

        self.max_speed = max_speed
        self.min_speed = min_speed
        self.max_yawrate = max_yawrate
        self.max_accel = max_accel
        self.max_dyawrate = max_dyawrate
        self.velocity_resolution = (
            self.max_speed - self.min_speed)/velocity_num
        self.yawrate_resolution = 2*self.max_yawrate/yawrate_num
        self.dt = dt
        self.predict_time = predict_time
        self.heading_coef = heading_coef
        self.clearance_coef = clearance_coef
        self.velocity_coef = velocity_coef
        self.base = base
