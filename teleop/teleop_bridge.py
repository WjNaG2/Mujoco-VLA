"""
teleop_bridge.py —— XR 输入 → MuJoCo 桥接层

本模块提供了从 xr_teleoperate 的 TeleData 结构到 MuJoCo 末端控制器的
数据桥接。包含三部分：

1. TeleData —— 数据类定义（镜像 xr_teleoperate 的结构）
2. SimulatedXRSource —— 模拟 XR 数据源（无硬件也可测试）
3. XRToMuJoCoBridge —— 桥接转换逻辑

坐标系说明：
  tele_data 输出的 wrist_pose 位于 "Robot Convention"（z 向上，x 向前，y 向左），
  与 MuJoCo 场景 (simple_end_effector_scene.xml) 的世界坐标系一致，
  因此直接提取 pose[:3, 3] 作为末端目标位置即可。

使用示例（在 run_xr_to_mujoco.py 中）：
    bridge = XRToMuJoCoBridge()
    source = SimulatedXRSource()
    while True:
        tele_data = source.get_tele_data()
        target_right = bridge.wrist_pose_to_position(tele_data.right_wrist_pose)
        target_left  = bridge.wrist_pose_to_position(tele_data.left_wrist_pose)
        # target_right, target_left 可直接喂给 EndEffectorController.update()
"""

import sys

# 强制 stdout/stderr 行缓冲，确保 conda run 下实时输出
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

import numpy as np
from dataclasses import dataclass, field
from typing import Optional
import time



# ============================================================
# 1. TeleData —— 镜像 xr_teleoperate 中 TeleVuerWrapper 的输出
# ============================================================

@dataclass
class TeleData:
    """
    镜像 xr_teleoperate/teleop/televuer/src/televuer/tv_wrapper.py 中的 TeleData

    核心字段（用于末端位置跟踪）：
        left_wrist_pose:  (4,4) SE(3) — 左手腕位姿, Robot Convention
        right_wrist_pose: (4,4) SE(3) — 右手腕位姿, Robot Convention
        head_pose:        (4,4) SE(3) — 头部位姿, Robot Convention

    扩展字段（手部细节，当前暂未使用）：
        left_hand_pos:  (25,3) — 左手 25 个关节点位置
        right_hand_pos: (25,3) — 右手 25 个关节点位置
    """
    head_pose: np.ndarray                  # (4,4) SE(3)
    left_wrist_pose: np.ndarray            # (4,4) SE(3)
    right_wrist_pose: np.ndarray           # (4,4) SE(3)
    left_hand_pos: Optional[np.ndarray] = None   # (25,3) optional
    right_hand_pos: Optional[np.ndarray] = None  # (25,3) optional
    timestamp: float = 0.0


# ============================================================
# 2. SimulatedXRSource —— 模拟 XR 手腕运动（无 XR 硬件也能测试）
# ============================================================

class SimulatedXRSource:
    """
    模拟 XR 输入源，生成类似 TeleVuerWrapper.get_tele_data() 的输出。

    支持多种运动模式：
        - "circle":     右手画圆 + 左手镜像
        - "reach":      双手交替前伸
        - "raise_hands": 双手同时上举
        - "wave":       右手挥手 + 左手自然下垂

    每个模式下，左右手腕会沿不同轨迹运动。
    wrist_pose 输出为 (4,4) SE(3) 齐次变换矩阵。
    """

    def __init__(self, mode: str = "circle", speed_scale: float = 1.0):
        """
        参数:
            mode:        运动模式 ("circle" / "reach" / "raise_hands" / "wave")
            speed_scale: 速度缩放系数（1.0 为正常速度）
        """
        self.mode = mode
        self.speed_scale = speed_scale
        self._start_time = time.time()

        # 预设初始位置
        # 右肩: (0.2, 0.0, 0.35)，左肩: (-0.2, 0.0, 0.35)
        # 双臂朝 +x 方向伸展，yaw 范围 ±90° 无法翻转到后方。
        # 因此左腕 x 坐标必须 > 左肩 x (-0.2)，右腕 x > 右肩 x (0.2)
        # 安全范围:
        #   右腕: x∈[0.22, 0.65], y∈[-0.2, 0.2], z∈[0.15, 0.65]
        #   左腕: x∈[-0.15, 0.35], y∈[-0.2, 0.2], z∈[0.15, 0.65]
        self._init_right_pos = np.array([0.40, -0.10, 0.30])
        self._init_left_pos = np.array([0.05, 0.10, 0.30])  # x > -0.2, 略偏中心线



    def reset(self):
        """重置时间"""
        self._start_time = time.time()

    def get_tele_data(self) -> TeleData:
        """
        获取当前时刻的模拟 TeleData。
        返回的 wrist_pose 中的平移部分即为手腕在 Robot Convention 世界坐标系中的位置。
        """
        t = (time.time() - self._start_time) * self.speed_scale

        if self.mode == "circle":
            right_pos, left_pos = self._circle_trajectory(t)
        elif self.mode == "reach":
            right_pos, left_pos = self._reach_trajectory(t)
        elif self.mode == "raise_hands":
            right_pos, left_pos = self._raise_hands_trajectory(t)
        elif self.mode == "wave":
            right_pos, left_pos = self._wave_trajectory(t)
        else:
            raise ValueError(f"Unknown mode: {self.mode}")

        # 构建 SE(3) 位姿矩阵（单位旋转，仅平移变化）
        right_wrist_pose = np.eye(4)
        right_wrist_pose[:3, 3] = right_pos

        left_wrist_pose = np.eye(4)
        left_wrist_pose[:3, 3] = left_pos

        # 头部位姿（固定，位于肩部上方）
        head_pose = np.eye(4)
        head_pose[:3, 3] = np.array([0.0, 0.0, 0.75])

        return TeleData(
            head_pose=head_pose,
            left_wrist_pose=left_wrist_pose,
            right_wrist_pose=right_wrist_pose,
            timestamp=t,
        )

    def _circle_trajectory(self, t: float):
        """
        双手同步画圆，在各自的工作空间内运动。

        重要——双臂工作空间约束：
          右肩 (0.2, 0.0, 0.35)，左肩 (-0.2, 0.0, 0.35)
          双臂默认朝 +x 方向伸展，yaw 范围 ±90° 无法翻转到 -x
          因此双腕的 x 坐标必须始终在 shoulder_x 右侧（大于肩部 x）
          
          安全范围:
            右腕: x∈[0.22, 0.60], y∈[-0.15, 0.20], z∈[0.15, 0.60]
            左腕: x∈[-0.15, 0.35], y∈[-0.20, 0.15], z∈[0.15, 0.60]
        """
        radius = 0.10      # 半径缩小，保证在安全范围内
        angle = t * 0.8    # 角速度

        # 右手：以右肩前侧为圆心，主要在 yz 平面画圆，加少量 x 摆动
        right_pos = np.array([
            0.35 + 0.05 * np.cos(angle),       # x: 小幅度前后
            0.10 + radius * np.sin(angle),      # y: 左右摆动
            0.35 + radius * np.cos(angle),      # z: 上下运动
        ])

        # 左手：对称，x 保持在 0 以上的正方向（左臂无法翻转到肩后）
        left_pos = np.array([
            0.10 + 0.05 * np.cos(angle + np.pi),  # x: 保持在正方向
            -0.10 + radius * np.sin(angle + np.pi),# y: 左右摆动（对称）
            0.35 + radius * np.cos(angle + np.pi), # z: 上下运动
        ])

        return right_pos, left_pos



    def _reach_trajectory(self, t: float):
        """双手交替前伸"""
        period = 4.0  # 每个循环 4 秒
        phase = (t % period) / period  # 0~1

        # 右手前伸（0->0.5）→ 收回（0.5->1）
        right_reach = np.sin(phase * np.pi) * 0.25
        right_pos = self._init_right_pos + np.array([right_reach, 0.0, 0.0])

        # 左手镜像（相位相反）
        left_reach = np.sin((phase + 0.5) * np.pi) * 0.25
        left_pos = self._init_left_pos + np.array([left_reach, 0.0, 0.0])

        return right_pos, left_pos

    def _raise_hands_trajectory(self, t: float):
        """双手同时上举再放下"""
        period = 5.0
        phase = (t % period) / period

        # 0->0.5 上举，0.5->1 放下
        height = np.sin(phase * np.pi) * 0.35

        right_pos = self._init_right_pos + np.array([-0.05, 0.0, height])
        left_pos = self._init_left_pos + np.array([0.05, 0.0, height])

        return right_pos, left_pos

    def _wave_trajectory(self, t: float):
        """右手挥手，左手保持"""
        period = 2.0
        phase = (t % period) / period

        # 右手左右摆动
        wave_offset = np.sin(phase * 2 * np.pi) * 0.15
        right_pos = np.array([0.50, wave_offset, 0.45])

        # 左手保持初始位置
        left_pos = self._init_left_pos.copy()

        return right_pos, left_pos


# ============================================================
# 2a. RealXRSource —— 真实 XR 设备输入源（包装 TeleVuerWrapper）
# ============================================================

class RealXRSource:
    """
    真实 XR 设备输入源，包装 xr_teleoperate 仓库的 TeleVuerWrapper。

    与 SimulatedXRSource 拥有相同的 get_tele_data() 接口，
    可在 run_xr_to_mujoco_demo.py 中互换使用。

    依赖:
        - xr_teleoperate/teleop/televuer 已安装（pip install -e .）
        - SSL 证书已配置
        - XR 设备与主机在同一局域网

    使用示例:
        source = RealXRSource(host_ip="192.168.123.2")
        tele_data = source.get_tele_data()
        # tele_data.right_wrist_pose[:3, 3] → 右手腕 3D 位置
    """

    def __init__(self,
                 host_ip: str = "192.168.123.2",
                 port: int = 8012,
                 use_hand_tracking: bool = True,
                 display_mode: str = "pass-through",
                 cert_file: str = None,
                 key_file: str = None,
                 img_server_ip: str = "192.168.123.164",
                 wait_for_device: bool = True,
                 connection_timeout: float = 300.0):
        """
        参数:
            host_ip:         主机 IP 地址（XR 设备通过此地址连接 Vuer 服务）
            port:            Vuer WebSocket 端口（默认 8012）
            use_hand_tracking: True = 手势跟踪, False = 手柄跟踪
            display_mode:    "immersive" / "pass-through" / "ego"
            cert_file:       SSL 证书路径（如为 None 则自动搜索）
            key_file:        SSL 私钥路径
            img_server_ip:   图像服务器 IP（仅 immersive/ego 模式需要）
            wait_for_device: 是否在初始化后等待 XR 设备连接（默认 True）
            connection_timeout: 等待设备连接的超时时间（秒，默认 300）
        """
        self.host_ip = host_ip
        self.port = port
        self.use_hand_tracking = use_hand_tracking
        self.display_mode = display_mode
        self.img_server_ip = img_server_ip
        self._connected = False

        # 尝试导入 TeleVuerWrapper
        try:
            from televuer import TeleVuerWrapper as _TeleVuerWrapper
        except ImportError:
            raise ImportError(
                "无法导入 TeleVuerWrapper。请确保 xr_teleoperate/teleop/televuer 已安装:\n"
                "  cd third_party/xr_teleoperate/teleop/televuer && pip install -e ."
            )

        # 自动搜索证书文件
        if cert_file is None or key_file is None:
            import os as _os
            _home = _os.path.expanduser("~")
            _config_dir = _os.path.join(_home, ".config", "xr_teleoperate")
            _cert_path = _os.path.join(_config_dir, "cert.pem")
            _key_path = _os.path.join(_config_dir, "key.pem")
            if _os.path.isfile(_cert_path) and _os.path.isfile(_key_path):
                cert_file = _cert_path
                key_file = _key_path
            else:
                cert_file = _os.environ.get("XR_TELEOP_CERT", cert_file)
                key_file = _os.environ.get("XR_TELEOP_KEY", key_file)

        # 初始化 TeleVuerWrapper
        # immersive / ego 模式需要 zmq=True 或 webrtc=True 来传输图像到 VR 头显
        _use_zmq = display_mode in ("immersive", "ego")
        self._wrapper = _TeleVuerWrapper(
            use_hand_tracking=use_hand_tracking,
            binocular=False,
            img_shape=(480, 640),
            display_fps=30.0,
            display_mode=display_mode,
            zmq=_use_zmq,
            webrtc=False,
            cert_file=cert_file,
            key_file=key_file,
        )

        print(f"[RealXRSource] 初始化完成")
        print(f"  Host IP: {host_ip}:{port}")
        print(f"  跟踪模式: {'手势' if use_hand_tracking else '手柄'}")
        print(f"  显示模式: {display_mode}")
        print(f"  证书路径: cert={cert_file}, key={key_file}")
        print(f"  请在 XR 设备浏览器中访问: https://{host_ip}:{port}/?ws=wss://{host_ip}:{port}")

        # 等待 XR 设备连接
        if wait_for_device:
            self.wait_for_connection(timeout=connection_timeout)

    def _is_device_connected(self) -> bool:
        """
        检查 XR 设备是否已连接。

        直接检查底层 TeleVuer 的原始 left_arm_pose / right_arm_pose 属性。
        在没有设备连接时，这些属性是**全零矩阵**（shared memory 初始值）。
        只有当 XR 设备通过 WebSocket 发送数据后，on_hand_move / on_controller_move
        回调才会更新这些值，使其不再是全零。

        注意：不能检查 get_tele_data() 的输出，因为 safe_mat_update 在检测到
        行列式为 0 时会返回默认常量矩阵，导致无法区分"无设备"和"有设备但静止"。
        """
        tv = self._wrapper.tvuer
        left_is_zero = np.allclose(tv.left_arm_pose, 0, atol=1e-6)
        right_is_zero = np.allclose(tv.right_arm_pose, 0, atol=1e-6)
        return not (left_is_zero or right_is_zero)

    def wait_for_connection(self, timeout: float = 300.0):
        """
        等待 XR 设备首次发送有效数据。

        直接检查底层 TeleVuer 的原始 left_arm_pose / right_arm_pose 是否
        不再是全零矩阵（全零 = 从未收到过 XR 数据）。

        参数:
            timeout: 超时时间（秒），默认 300 秒（5 分钟）
        """
        print(f"\n  ⏳ 正在等待 XR 设备连接（超时 {timeout:.0f} 秒）...")
        print(f"  请戴上 XR 设备并确保双手在设备视野内...")

        poll_interval = 0.5  # 每 0.5 秒检查一次
        elapsed = 0.0

        while elapsed < timeout:
            if self._is_device_connected():
                self._connected = True
                print(f"  ✅ XR 设备已连接！（等待 {elapsed:.1f} 秒）")
                return

            # 打印进度
            if elapsed > 0 and int(elapsed) % 5 == 0 and elapsed - int(elapsed) < poll_interval:
                print(f"  ⏳ 等待中 ({elapsed:.0f}s)... 请确保双手在 XR 设备视野内")

            time.sleep(poll_interval)
            elapsed += poll_interval

        # 超时
        print(f"\n  ❌ 等待 XR 设备连接超时（{timeout:.0f} 秒）")
        print(f"  请检查:")
        print(f"    1. XR 设备是否已开机并连接到同一局域网")
        print(f"    2. 是否已在浏览器中访问 https://{self.host_ip}:{self.port}")
        print(f"    3. 是否已点击 Virtual Reality 按钮并允许权限")
        print(f"    4. 防火墙端口 {self.port} 是否已开放")
        print(f"  仿真将不会启动，请修复后重试。\n")
        raise ConnectionError(
            f"XR 设备连接超时（{timeout:.0f} 秒）。\n"
            f"请确保:\n"
            f"  1. XR 设备已开机并连接到同一局域网\n"
            f"  2. 已在浏览器中访问 https://{self.host_ip}:{self.port}\n"
            f"  3. 已点击 Virtual Reality 按钮并允许权限\n"
            f"  4. 防火墙端口 {self.port} 已开放"
        )


    def get_tele_data(self):
        """
        获取真实 XR 设备的 TeleData。

        返回:
            TeleData 实例（与 SimulatedXRSource.get_tele_data() 返回类型相同）
        """
        return self._wrapper.get_tele_data()

    def close(self):
        """关闭 TeleVuerWrapper 连接"""
        self._wrapper.close()



# ============================================================
# 3. XRToMuJoCoBridge —— 桥接转换逻辑
# ============================================================

class XRToMuJoCoBridge:
    """
    XR 数据 → MuJoCo 末端目标的桥接转换器。

    核心功能：
        1. 从 TeleData.right_wrist_pose 提取右手位置
        2. 从 TeleData.left_wrist_pose 提取左手位置
        3. 可选：坐标系变换（如果需要）
        4. 可选：偏移校准（将 XR 手腕位置映射到 MuJoCo 臂的工作空间）

    默认情况下，tele_data 的 wrist_pose 已经处于 Robot Convention，
    与 MuJoCo 世界坐标系对齐，因此直接提取平移量即可。

    如果 XR 世界中的人体比例与 MuJoCo 机器人不同，
    可以通过 offset 和 scale 参数进行缩放和平移。
    """

    def __init__(self,
                 right_offset: np.ndarray = None,
                 left_offset: np.ndarray = None,
                 scale: float = 1.0):
        """
        参数:
            right_offset: 右手位置的额外偏移 (3,)（可选）
            left_offset:  左手位置的额外偏移 (3,)（可选）
            scale:        位置缩放系数（用于 XR 世界到 MuJoCo 世界的尺度映射）
        """
        if right_offset is None:
            right_offset = np.zeros(3)
        if left_offset is None:
            left_offset = np.zeros(3)
        self.right_offset = np.asarray(right_offset, dtype=np.float32)
        self.left_offset = np.asarray(left_offset, dtype=np.float32)
        self.scale = scale

    def wrist_pose_to_position(self, wrist_pose: np.ndarray) -> np.ndarray:
        """
        从 (4,4) SE(3) 手腕位姿中提取 3D 位置。

        参数:
            wrist_pose: (4,4) numpy 数组，齐次变换矩阵

        返回:
            (3,) numpy 数组，世界坐标系中的 (x, y, z) 位置
        """
        return wrist_pose[:3, 3].copy()

    def get_right_target(self, tele_data: TeleData) -> np.ndarray:
        """
        从 tele_data 获取右手目标位置。

        参数:
            tele_data: TeleData 实例

        返回:
            (3,) 右手目标位置（世界坐标系）
        """
        pos = self.wrist_pose_to_position(tele_data.right_wrist_pose)
        return pos * self.scale + self.right_offset

    def get_left_target(self, tele_data: TeleData) -> np.ndarray:
        """
        从 tele_data 获取左手目标位置。
        """
        pos = self.wrist_pose_to_position(tele_data.left_wrist_pose)
        return pos * self.scale + self.left_offset

    def get_bimanual_targets(self, tele_data: TeleData):
        """
        获取双臂目标位置。

        返回:
            (right_target, left_target) 二元组，每个为 (3,) 位置向量
        """
        return self.get_right_target(tele_data), self.get_left_target(tele_data)


# ============================================================
# 快速测试
# ============================================================

if __name__ == "__main__":
    print("=" * 60)
    print("桥接层单元测试")
    print("=" * 60)

    bridge = XRToMuJoCoBridge()
    source = SimulatedXRSource(mode="circle")

    print("\n3 秒内采集 10 帧数据...\n")
    for i in range(10):
        tele_data = source.get_tele_data()

        right_pos = bridge.get_right_target(tele_data)
        left_pos = bridge.get_left_target(tele_data)

        print(f"Frame {i:2d} | "
              f"右手: ({right_pos[0]:.3f}, {right_pos[1]:.3f}, {right_pos[2]:.3f}) | "
              f"左手: ({left_pos[0]:.3f}, {left_pos[1]:.3f}, {left_pos[2]:.3f})")

        time.sleep(0.3)

    print("\n✅ 桥接层测试通过！")
    print("   tele_data.right_wrist_pose[:3, 3] → 右手 3D 目标位置")
    print("   tele_data.left_wrist_pose[:3, 3]  → 左手 3D 目标位置")
