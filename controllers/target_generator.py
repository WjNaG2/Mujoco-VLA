"""
target_generator.py —— 目标点生成器

本模块提供三种版本的目标点生成方式，用于测试末端位置跟随控制器：

- 版本 A：静态单点跟随 —— 只给一个固定目标点
- 版本 B：离散多点跟随 —— 预设一系列空间点，依次到达
- 版本 C：连续轨迹跟随 —— 给定一条轨迹（直线/圆形），连续跟踪

用法示例：
    gen = StaticTargetGenerator(pos=[0.5, 0.2, 0.4])
    target = gen.get_target(t=0.0)  # 始终返回同一个点

    gen = DiscreteTargetGenerator(waypoints=[...], dwell_time=1.0)
    target = gen.get_target(t=3.5)  # 返回当前阶段的目标点

    gen = CircleTargetGenerator(center=[0.3, 0.0, 0.4], radius=0.15)
    target = gen.get_target(t=2.0)  # 返回圆形轨迹上的点
"""
import numpy as np
from typing import List


class StaticTargetGenerator:
    """
    版本 A：静态单点跟随

    始终返回同一个固定的三维目标位置。
    适合最基础的调试：检查末端是否能稳定到达一个给定点。
    """

    def __init__(self, pos: List[float] = None):
        """
        参数:
            pos: 固定目标点位置 [x, y, z]，单位: 米
        """
        if pos is None:
            pos = [0.5, 0.2, 0.4]
        self.pos = np.array(pos, dtype=np.float32)

    def get_target(self, t: float = 0.0) -> np.ndarray:
        """始终返回同一个固定目标点"""
        return self.pos.copy()

    def __repr__(self):
        return f"StaticTargetGenerator(pos={self.pos.tolist()})"


class DiscreteTargetGenerator:
    """
    版本 B：离散多点跟随

    预设一系列空间中的目标点，机器人按顺序依次移动到各个点。
    每个点停留一段指定时间，用于测试控制器在多目标间的切换能力。
    """

    def __init__(self, waypoints: List[List[float]],
                 dwell_time: float = 2.0, loop: bool = True):
        """
        参数:
            waypoints:  一系列目标点列表 [[x1,y1,z1], [x2,y2,z2], ...]
            dwell_time: 在每个目标点停留的仿真时间（秒）
            loop:       是否循环（到达最后一个点后回到第一个）
        """
        self.waypoints = [np.array(p, dtype=np.float32) for p in waypoints]
        self.dwell_time = dwell_time
        self.loop = loop

    def get_target(self, t: float) -> np.ndarray:
        """
        根据当前时间返回对应的目标点

        参数:
            t: 当前仿真时间（秒）

        返回:
            当前阶段的目标位置 (3,)
        """
        n = len(self.waypoints)
        if n == 0:
            return np.zeros(3)
        # 计算当前处于哪个目标点
        idx = int(t / self.dwell_time)
        if self.loop:
            idx = idx % n
        else:
            idx = min(idx, n - 1)
        return self.waypoints[idx].copy()

    def __repr__(self):
        return (f"DiscreteTargetGenerator("
                f"n_waypoints={len(self.waypoints)}, "
                f"dwell={self.dwell_time}s)")


class CircleTargetGenerator:
    """
    版本 C：连续轨迹跟随 —— 圆形轨迹

    在空间中以匀速生成圆形轨迹上的连续目标点。
    用于测试控制器对连续运动的跟踪能力。
    """

    def __init__(self, center: List[float] = None,
                 radius: float = 0.2,
                 axis: str = "xz",
                 speed: float = 0.5):
        """
        参数:
            center: 圆心位置 [cx, cy, cz]，单位: 米
            radius: 圆的半径，单位: 米
            axis:   圆形平面，"xz" 表示在 XZ 平面画圆，"xy" 在 XY 平面
            speed:  角速度（弧度/秒），越大圆运动越快
        """
        if center is None:
            center = [0.4, 0.0, 0.4]
        self.center = np.array(center, dtype=np.float32)
        self.radius = radius
        self.axis = axis
        self.speed = speed

    def get_target(self, t: float) -> np.ndarray:
        """
        根据当前时间返回圆形轨迹上的目标点

        参数:
            t: 当前仿真时间（秒）

        返回:
            轨迹上的目标位置 (3,)
        """
        angle = self.speed * t
        if self.axis == "xz":
            dx = self.radius * np.cos(angle)
            dz = self.radius * np.sin(angle)
            return self.center + np.array([dx, 0.0, dz])
        elif self.axis == "xy":
            dx = self.radius * np.cos(angle)
            dy = self.radius * np.sin(angle)
            return self.center + np.array([dx, dy, 0.0])
        else:
            raise ValueError(f"不支持的轴: {self.axis}")

    def __repr__(self):
        return (f"CircleTargetGenerator("
                f"center={self.center.tolist()}, "
                f"radius={self.radius})")


class LineTargetGenerator:
    """
    版本 C：连续轨迹跟随 —— 直线轨迹

    在空间中以匀速沿直线运动。
    """

    def __init__(self, start: List[float] = None,
                 end: List[float] = None,
                 speed: float = 0.2):
        """
        参数:
            start: 起点位置 [x, y, z]
            end:   终点位置 [x, y, z]
            speed: 移动速度（米/秒）
        """
        if start is None:
            start = [0.2, 0.0, 0.3]
        if end is None:
            end = [0.6, 0.0, 0.5]
        self.start = np.array(start, dtype=np.float32)
        self.end = np.array(end, dtype=np.float32)
        self.direction = self.end - self.start
        self.length = np.linalg.norm(self.direction)
        self.speed = speed

    def get_target(self, t: float) -> np.ndarray:
        """
        根据当前时间返回直线轨迹上的目标点

        参数:
            t: 当前仿真时间（秒）

        返回:
            轨迹上的目标位置 (3,)
        """
        # 计算沿直线的归一化进度（来回循环）
        total_length = 2.0 * self.length
        pos = (t * self.speed) % total_length
        if pos > self.length:
            # 返回阶段
            progress = 1.0 - (pos - self.length) / self.length
        else:
            # 去程阶段
            progress = pos / self.length

        return self.start + self.direction * progress
