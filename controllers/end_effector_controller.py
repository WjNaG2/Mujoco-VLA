"""
end_effector_controller.py —— 末端位置跟随控制器

本模块实现了从"期望末端位置"到"关节目标角度"的映射，
使得仿真机器人能够将末端执行器移动到目标位置附近。

======================================================================
## 控制策略：基于雅可比伪逆（Jacobian Pseudoinverse）的 IK
======================================================================

给定期望末端位置 p_des 和当前末端位置 p_curr，位置误差为：

    e = p_des - p_curr

末端速度与关节速度的关系通过雅可比矩阵 J 描述：

    v_eef = J · q_dot

因此，我们可以通过以下方式计算关节速度修正量：

    q_dot = J^† · v_eef    其中 J^† 是 J 的伪逆

控制器在每个仿真步长中：
1. 计算当前末端位置（通过 site 的 xpos）
2. 计算位置误差
3. 用比例控制得到期望末端速度: v = Kp * e
4. 通过雅可比伪逆映射到关节速度
5. 积分得到关节目标位置
6. 将关节目标位置发送给 actuator（通过 ctrl）

======================================================================
## 使用示例
======================================================================

    from envs.upper_body_env import UpperBodyEnv
    from controllers.end_effector_controller import EndEffectorController

    env = UpperBodyEnv()
    controller = EndEffectorController(env, "right", kp=3.0)

    target_pos = np.array([0.5, 0.2, 0.4])
    for i in range(500):
        env.step()
        error = controller.update(target_pos)
        env.set_joint_targets(controller.joint_targets)
"""
import numpy as np
import mujoco


class EndEffectorController:
    """
    末端位置跟随控制器

    基于雅可比伪逆的 IK 控制器，驱动指定臂的末端执行器到达目标位置。

    支持 "right"（右臂）和 "left"（左臂）两个末端。
    """

    def __init__(self, env, side: str = "right", kp: float = 3.0,
                 joint_velocity_limit: float = 2.0):
        """
        初始化末端控制器

        参数:
            env:                   UpperBodyEnv 实例
            side:                  "right" 或 "left"
            kp:                    比例增益系数，越大响应越快（但可能导致振荡）
            joint_velocity_limit:  关节速度上限（弧度/秒），用于安全性限制
        """
        self.env = env
        self.side = side
        self.kp = kp
        self.joint_velocity_limit = joint_velocity_limit

        # 根据选择确定对应的 site 名称和关节索引范围
        if side == "right":
            self.ee_site_name = "r_ee_site"
            # 右臂关节在 JOINT_NAMES 中对应 [0, 1, 2]
            self.joint_indices = [0, 1, 2]
        elif side == "left":
            self.ee_site_name = "l_ee_site"
            # 左臂关节在 JOINT_NAMES 中对应 [3, 4, 5]
            self.joint_indices = [3, 4, 5]
        else:
            raise ValueError(f"side 必须是 'right' 或 'left'，收到 '{side}'")

        # 获取关节在自由度空间（dof）中的索引
        # 注意：mj_jacSite 返回的雅可比矩阵是 (3 x nv)，列索引对应 dof 编号
        self.dof_indices = []
        for idx in self.joint_indices:
            jnt_name = env.JOINT_NAMES[idx]
            jnt_id = env.joint_name_to_id[jnt_name]
            dof_adr = env.model.jnt_dofadr[jnt_id]
            self.dof_indices.append(dof_adr)

        # 关节目标位置（弧度），初始为当前关节角度
        self.joint_targets = np.zeros(len(env.JOINT_NAMES))
        current_q = env.get_observation()["joint_positions"]
        for idx in self.joint_indices:
            self.joint_targets[idx] = current_q[idx]

        # actuator 索引映射
        self.actuator_indices = []
        for name in [env.ACTUATOR_NAMES[i] for i in self.joint_indices]:
            self.actuator_indices.append(env.actuator_name_to_id[name])

    def update(self, target_pos: np.ndarray) -> float:
        """
        执行一步 IK 控制，更新关节目标位置

        参数:
            target_pos: 期望末端位置 (x, y, z)

        返回:
            error_norm: 当前末端位置与目标位置之间的欧氏距离（用于监控）
        """
        # 1. 获取当前末端位置
        obs = self.env.get_observation()
        ee_pos_key = f"{self.side[:1]}_ee_pos"
        ee_pos = obs[ee_pos_key]

        # 2. 计算位置误差
        error = target_pos - ee_pos
        error_norm = np.linalg.norm(error)

        # 3. 计算期望末端速度（比例控制）
        desired_ee_vel = self.kp * error

        # 4. 通过雅可比伪逆得到关节速度
        site_id = self.env.site_name_to_id[self.ee_site_name]
        jac = np.zeros((3, self.env.model.nv))  # 雅可比矩阵 (3 x nv)
        mujoco.mj_jacSite(
            self.env.model, self.env.data, jac, None, site_id
        )
        # 提取当前臂对应的列
        jac_arm = jac[:, self.dof_indices]  # (3, num_joints)

        # 伪逆: J^† = J^T (J J^T + λI)^(-1)，加入正则化防止奇异
        jtj = jac_arm @ jac_arm.T
        reg = 1e-4  # 正则化系数，防止雅可比奇异时矩阵不可逆
        jtj_inv = np.linalg.inv(jtj + reg * np.eye(3))
        jac_pinv = jac_arm.T @ jtj_inv  # (num_joints, 3)

        # 关节速度 = J^† * desired_ee_vel
        joint_vel = jac_pinv @ desired_ee_vel

        # 5. 限制关节速度（安全性）
        joint_vel = np.clip(
            joint_vel,
            -self.joint_velocity_limit,
            self.joint_velocity_limit
        )

        # 6. 积分得到关节目标位置
        dt = self.env.model.opt.timestep
        for i, joint_idx in enumerate(self.joint_indices):
            self.joint_targets[joint_idx] += joint_vel[i] * dt

        # 确保目标位置不超出关节限位
        self._clamp_joint_targets()

        return error_norm

    def _clamp_joint_targets(self):
        """
        将关节目标位置限制在关节限位范围内。
        MuJoCo 内部 jnt_range 单位为弧度，直接使用即可。
        """
        for i, jnt_name in enumerate(self.env.JOINT_NAMES):
            if i not in self.joint_indices:
                continue
            jnt_id = self.env.joint_name_to_id[jnt_name]
            range_min = self.env.model.jnt_range[jnt_id, 0]   # 弧度
            range_max = self.env.model.jnt_range[jnt_id, 1]   # 弧度
            self.joint_targets[i] = np.clip(
                self.joint_targets[i], range_min, range_max
            )

    def reset(self):
        """
        重置控制器状态，将关节目标设为当前关节位置
        """
        obs = self.env.get_observation()
        current_q = obs["joint_positions"]
        for idx in self.joint_indices:
            self.joint_targets[idx] = current_q[idx]


class BimanualController:
    """
    双臂联合控制器 —— 同时控制左右末端到达各自的目标位置

    内部维护两个 EndEffectorController 实例（左臂和右臂），
    在 update() 中依次计算并合并关节目标。
    """

    def __init__(self, env, kp: float = 3.0, joint_velocity_limit: float = 2.0):
        """
        初始化双臂控制器

        参数:
            env:                   UpperBodyEnv 实例
            kp:                    比例增益
            joint_velocity_limit:  关节速度上限
        """
        self.env = env
        self.right_ctrl = EndEffectorController(
            env, side="right", kp=kp, joint_velocity_limit=joint_velocity_limit
        )
        self.left_ctrl = EndEffectorController(
            env, side="left", kp=kp, joint_velocity_limit=joint_velocity_limit
        )

    def update(self, target_right: np.ndarray, target_left: np.ndarray) -> dict:
        """
        同时更新双臂的关节目标位置

        参数:
            target_right: 右手期望末端位置 (3,)
            target_left:  左手期望末端位置 (3,)

        返回:
            {"right_error": float, "left_error": float}
        """
        right_error = self.right_ctrl.update(target_right)
        left_error = self.left_ctrl.update(target_left)

        # 合并关节目标到 right_ctrl 的 joint_targets 中
        for i, joint_idx in enumerate(self.left_ctrl.joint_indices):
            self.right_ctrl.joint_targets[joint_idx] = \
                self.left_ctrl.joint_targets[joint_idx]

        return {"right_error": right_error, "left_error": left_error}

    def get_joint_targets(self) -> np.ndarray:
        """获取合并后的关节目标位置（长度为 6）"""
        targets = self.right_ctrl.joint_targets.copy()
        for i, joint_idx in enumerate(self.left_ctrl.joint_indices):
            targets[joint_idx] = self.left_ctrl.joint_targets[joint_idx]
        return targets

    def reset(self):
        """重置双臂控制器"""
        self.right_ctrl.reset()
        self.left_ctrl.reset()
