"""
upper_body_env.py —— 上半身双臂 Mujoco 场景封装器

本模块将 MuJoCo 场景（simple_end_effector_scene.xml）封装为一个 Python 类，
提供统一的 reset、step、get_observation 接口，便于后续控制、遥操和数据采集。

核心功能：
1. 加载场景并创建 MuJoCo 数据对象
2. 提供向前仿真一步的 step() 方法
3. 获取当前关节状态、末端执行器(eef)位置、目标点位置
4. 设置关节目标位置（通过 actuator 的 ctrl 数组）
5. 获取相机内参和外参

用法示例：
    env = UpperBodyEnv(scene_xml_path)
    env.reset()
    env.set_joint_targets([0.5, -0.3, 1.2, -0.5, -0.3, 1.2])
    env.step()
    obs = env.get_observation()
"""
import os
import numpy as np
import mujoco


class UpperBodyEnv:
    """上半身双臂 Mujoco 场景封装器"""

    # 关节名称列表（与 XML 中 actuator 的顺序一致）
    JOINT_NAMES = [
        "r_shoulder_yaw",    # 右肩偏航（绕 Z 轴）—— 左右摆动
        "r_shoulder_pitch",  # 右肩俯仰（绕 Y 轴）—— 抬臂/落臂
        "r_elbow",           # 右肘（绕 X 轴）—— 前臂横向摆动
        "l_shoulder_yaw",    # 左肩偏航（绕 Z 轴）
        "l_shoulder_pitch",  # 左肩俯仰（绕 Y 轴）
        "l_elbow",           # 左肘（绕 X 轴）
    ]

    # 执行器名称列表
    ACTUATOR_NAMES = [
        "r_shoulder_yaw_act",
        "r_shoulder_pitch_act",
        "r_elbow_act",
        "l_shoulder_yaw_act",
        "l_shoulder_pitch_act",
        "l_elbow_act",
    ]

    # 末端执行器 site 名称
    EE_SITE_NAMES = ["r_ee_site", "l_ee_site"]

    # 目标点 site 名称
    TARGET_SITE_NAMES = ["target_right_site", "target_left_site"]

    def __init__(self, scene_path: str = None, render_width: int = 640, render_height: int = 480):
        """
        初始化场景

        参数:
            scene_path: XML 场景文件路径（绝对或相对于项目根目录）
            render_width:  渲染宽度（像素），用于相机接口
            render_height: 渲染高度（像素）
        """
        # 解析场景文件路径
        if scene_path is None:
            # 默认使用本目录下的场景文件
            scene_path = os.path.join(
                os.path.dirname(__file__), "simple_end_effector_scene.xml"
            )
        elif not os.path.isabs(scene_path):
            # 尝试相对于项目根目录和 envs 目录解析
            project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
            candidates = [
                os.path.abspath(scene_path),
                os.path.join(project_root, scene_path),
                os.path.join(os.path.dirname(__file__), scene_path),
            ]
            for p in candidates:
                if os.path.exists(p):
                    scene_path = p
                    break
            else:
                raise FileNotFoundError(f"场景文件未找到: {scene_path}")

        # 加载 MuJoCo 模型
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        self.render_width = render_width
        self.render_height = render_height

        # 关节索引映射：名称 -> 关节 ID
        self.joint_name_to_id = {}
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            self.joint_name_to_id[name] = i

        # actuator 索引映射：名称 -> actuator ID
        self.actuator_name_to_id = {}
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            self.actuator_name_to_id[name] = i

        # site 索引映射
        self.site_name_to_id = {}
        for i in range(self.model.nsite):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, i)
            self.site_name_to_id[name] = i

        # 相机名称列表
        self.camera_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_CAMERA, i)
            for i in range(self.model.ncam)
        ]

        # 初始化仿真
        mujoco.mj_forward(self.model, self.data)

    def reset(self, joint_positions: np.ndarray = None):
        """
        重置仿真到初始状态

        参数:
            joint_positions: 可选的初始关节角度（弧度），长度为 6
                             如果为 None，则使用默认初始位形
        """
        if joint_positions is not None:
            assert len(joint_positions) == len(self.JOINT_NAMES), \
                f"关节角度数量应为 {len(self.JOINT_NAMES)}，收到 {len(joint_positions)}"
            for i, name in enumerate(self.JOINT_NAMES):
                jnt_id = self.joint_name_to_id[name]
                self.data.joint(jnt_id).qpos = joint_positions[i]
        else:
            # 默认初始位形：手臂微屈，避免雅可比奇异
            # r_shoulder_yaw=0°, r_shoulder_pitch=-20°, r_elbow=40°
            # l_shoulder_yaw=0°, l_shoulder_pitch=-20°, l_elbow=-40°
            # 注意：XML 中关节的 ref 属性定义了参考零位，
            # 这里设置的是 qpos 值（即相对于 ref 的偏移）。
            # 弧度制: -20° ≈ -0.349, 40° ≈ 0.698
            default_qpos = np.array([0.0, -0.349, 0.698, 0.0, -0.349, -0.698])
            for i, name in enumerate(self.JOINT_NAMES):
                jnt_id = self.joint_name_to_id[name]
                self.data.joint(jnt_id).qpos = default_qpos[i]

        # 重置 actuator 控制量为 0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def step(self):
        """
        向前仿真一步（使用当前 ctrl 值驱动 actuator）
        """
        mujoco.mj_step(self.model, self.data)

    def set_joint_targets(self, targets: np.ndarray):
        """
        设置关节目标位置（即 actuator 的 ctrl 值）

        参数:
            targets: 长度为 6 的 numpy 数组，顺序与 JOINT_NAMES 一致
                     单位：弧度 (rad)
        """
        assert len(targets) == len(self.ACTUATOR_NAMES), \
            f"目标数量应为 {len(self.ACTUATOR_NAMES)}，收到 {len(targets)}"
        for i, name in enumerate(self.ACTUATOR_NAMES):
            act_id = self.actuator_name_to_id[name]
            self.data.ctrl[act_id] = targets[i]

    def get_observation(self) -> dict:
        """
        获取当前完整观测

        返回字典包含：
            - joint_positions:  关节角度 (6,)
            - joint_velocities: 关节速度 (6,)
            - r_ee_pos:         右手末端位置 (3,)
            - l_ee_pos:         左手末端位置 (3,)
            - target_right_pos: 右手目标位置 (3,)
            - target_left_pos:  左手目标位置 (3,)
            - timestamp:        当前仿真时间戳
        """
        obs = {}

        # 关节位置和速度
        joint_pos = []
        joint_vel = []
        for name in self.JOINT_NAMES:
            jnt_id = self.joint_name_to_id[name]
            joint_pos.append(self.data.joint(jnt_id).qpos.copy())
            joint_vel.append(self.data.joint(jnt_id).qvel.copy())
        obs["joint_positions"] = np.array(joint_pos).flatten()
        obs["joint_velocities"] = np.array(joint_vel).flatten()

        # 末端执行器位置（通过 site 获取世界坐标）
        for site_name in self.EE_SITE_NAMES:
            site_id = self.site_name_to_id[site_name]
            pos = self.data.site(site_id).xpos.copy()
            key = site_name.replace("_site", "_pos")
            obs[key] = pos

        # 目标点位置
        for site_name in self.TARGET_SITE_NAMES:
            site_id = self.site_name_to_id[site_name]
            pos = self.data.site(site_id).xpos.copy()
            key = site_name.replace("_site", "_pos")
            obs[key] = pos

        obs["timestamp"] = self.data.time

        return obs

    def get_camera_info(self, camera_name: str) -> dict:
        """
        获取指定相机的内参和外参

        参数:
            camera_name: 相机名称（如 "camera_front"）

        返回:
            dict 包含:
                - intrinsics: 内参矩阵 (3x3)
                - extrinsics: 外参矩阵 (4x4)
                - fovy:       垂直视场角（度）
                - width:      图像宽度（像素）
                - height:     图像高度（像素）
        """
        cam_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name
        )
        if cam_id == -1:
            raise ValueError(f"相机 '{camera_name}' 不存在，可用相机: {self.camera_names}")

        # fovy 和宽高比
        fovy = float(self.model.cam_fovy[cam_id])
        # 注意：scene.xml 中的相机没有显式设置分辨率，这里使用传入的渲染尺寸
        h = self.render_height
        w = self.render_width
        aspect = w / h

        # 计算内参矩阵 K
        # 参考: fx = fy = (h/2) / tan(fovy/2)   (像素单位)
        fov_rad = np.deg2rad(fovy)
        fy = (h / 2.0) / np.tan(fov_rad / 2.0)
        fx = fy * aspect  # 保持像素方形
        cx = w / 2.0
        cy = h / 2.0
        K = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ])

        # 外参：从 data.cam_xpos 和 data.cam_xmat 获取
        # cam_xmat 是 9 个 float（按行优先排列的 3x3 旋转矩阵）
        R = self.data.cam_xmat[cam_id].reshape(3, 3).copy()
        t = self.data.cam_xpos[cam_id].copy().reshape(3, 1)

        # 构建 4x4 外参矩阵 [R | t; 0 0 0 1]
        extrinsics = np.eye(4)
        extrinsics[:3, :3] = R
        extrinsics[:3, 3] = t.flatten()

        return {
            "intrinsics": K,
            "extrinsics": extrinsics,
            "rotation": R,
            "translation": t.flatten(),
            "fovy": fovy,
            "width": w,
            "height": h,
        }

    def set_target_position(self, side: str, pos: np.ndarray):
        """
        移动世界坐标系中的目标点位置（通过修改 body 的 pos）

        参数:
            side: "right" 或 "left"，指定要移动哪个目标点
            pos:  3D 位置 (x, y, z)
        """
        body_name = f"target_{side}"
        body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, body_name
        )
        if body_id == -1:
            raise ValueError(f"目标体 '{body_name}' 不存在")
        # 修改世界坐标系中的位置
        self.model.body(body_id).pos[:] = pos
        mujoco.mj_forward(self.model, self.data)

    def close(self):
        """
        清理资源（目前无需特殊清理）
        """
        pass

    def launch_viewer(self):
        """
        启动 MuJoCo 交互式 viewer，用于可视化仿真。

        返回:
            viewer 对象，可用于后续调用 viewer.sync() 或 viewer.close().
        """
        try:
            from mujoco.viewer import launch_passive
        except ImportError as e:
            raise RuntimeError(
                "无法导入 mujoco.viewer，无法启动可视化窗口。"
            ) from e

        return launch_passive(self.model, self.data)
