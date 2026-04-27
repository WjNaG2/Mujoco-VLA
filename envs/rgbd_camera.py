"""
rgbd_camera.py —— ⭐ RGBD 相机接口（核心模块）

本模块实现了一组统一的 RGBD 相机读取接口，用于从 MuJoCo 仿真场景中
同步获取 RGB 图像和 Depth 图像，并附带对应的相机内参和外参。

======================================================================
## ⭐ 为什么这是核心模块？
======================================================================

在 VLA / imitation learning pipeline 中，**视觉观测是最关键的输入通道**。
每一帧训练数据都包含：
  - RGB 图像：提供语义信息（物体颜色、纹理、类别）
  - Depth 图像：提供几何信息（距离、形状、深度关系）
  - 相机参数：将 2D 像素映射回 3D 世界坐标的必要条件

本模块打通了"从仿真到数据"的完整链路：
  MuJoCo 仿真 → 离屏渲染 → RGB + Depth + 参数 → 统一字典输出 → 数据存储

======================================================================
## 接口设计
======================================================================

```python
camera = RGBDCamera(model, "camera_front", width=640, height=480)

# 每帧调用一次 capture
result = camera.capture(data)
# result = {
#     "rgb":         ndarray (H, W, 3), dtype=uint8, 范围 [0, 255]
#     "depth":       ndarray (H, W),   dtype=float32, 单位: 米
#     "intrinsics":  ndarray (3, 3),   相机内参矩阵 K
#     "extrinsics":  ndarray (4, 4),   相机外参矩阵 [R | t]
#     "timestamp":   float,            仿真时间戳
#     "camera_name": str,              相机名称
# }
```

======================================================================
## RGB 图像实现原理
======================================================================

1. 使用 `mujoco.Renderer` 创建**离屏渲染器**（不弹出窗口）
2. 每次调用 `capture()` 时：
   a. 调用 `update_scene(data)` 将当前仿真状态同步到渲染场景
   b. 调用 `render()` 获取 RGB 缓冲区 - 输出为 (H, W, 3) 的 uint8 数组

======================================================================
## Depth 图像实现原理
======================================================================

1. 通过 `enable_depth_rendering()` 开启深度渲染模式
2. 再次调用 `update_scene()` + `render()` 可获得深度图
3. MuJoCo 输出的深度是**视空间深度**（从相机到物体的垂直距离），单位: 米
4. 有效范围通常在 [near, far] 之间，far 以外的像素值为 50（默认远平面）
5. ⚠️ 每帧需要渲染两次（一次 RGB、一次 Depth），但同帧数据严格对齐

======================================================================
## 相机内参计算
======================================================================

MuJoCo 相机通过 `fovy`（垂直视场角）和图像宽高比定义投影，
没有直接给出像素焦距。我们手动计算内参矩阵 K：

    fy = (height/2) / tan(fov_rad/2)
    fx = fy * aspect
    cx = width/2,  cy = height/2

======================================================================
## 相机外参获取
======================================================================

- 相机世界位置: `data.cam_xpos[cam_id]`
- 相机旋转矩阵: `data.cam_xmat[cam_id]`（9个float，按行排列为 3x3）

======================================================================
## 使用示例
======================================================================

    from envs.upper_body_env import UpperBodyEnv
    from envs.rgbd_camera import RGBDCamera

    env = UpperBodyEnv()
    cam = RGBDCamera(env.model, env.data, "camera_front")

    for i in range(100):
        env.step()
        obs = cam.capture()
        # obs["rgb"], obs["depth"] 可用
"""
import numpy as np
import mujoco


class RGBDCamera:
    """
    RGBD 相机接口 —— 从 MuJoCo 场景中同步采集 RGB + Depth + 相机参数

    每个 RGBDCamera 实例绑定一个特定的 MuJoCo 相机（由 camera_name 指定），
    内部维护一个独立的离屏渲染器（Renderer）用于图像采集。

    注意：如果要支持多相机，可以为每个相机单独创建一个 RGBDCamera 实例。
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 camera_name: str, width: int = 640, height: int = 480):
        """
        初始化 RGBD 相机

        参数:
            model:       MuJoCo 模型对象（MjModel）
            data:        MuJoCo 数据对象（MjData）
            camera_name: 相机名称，必须匹配场景 XML 中定义的 camera name
            width:       渲染图像宽度（像素）
            height:      渲染图像高度（像素）
        """
        self.model = model
        self.data = data
        self.camera_name = camera_name
        self.width = width
        self.height = height

        # 验证相机是否存在
        self.cam_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name
        )
        if self.cam_id == -1:
            available = [
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
                for i in range(model.ncam)
            ]
            raise ValueError(
                f"相机 '{camera_name}' 不存在！可用相机: {available}"
            )

        # 【关键】创建离屏渲染器 —— 不弹出窗口，直接在内存中渲染
        # 使用 enable_depth_rendering() 来开启深度图输出
        self.renderer = mujoco.Renderer(model, height=height, width=width)

        # 预计算相机内参（相机参数不变，只需计算一次）
        self._intrinsics = self._compute_intrinsics()

    # ------------------------------------------------------------------
    # 内部方法：计算相机内参矩阵 K
    # ------------------------------------------------------------------
    def _compute_intrinsics(self) -> np.ndarray:
        """
        根据 MuJoCo 相机的 fovy 和图像尺寸计算内参矩阵 K

        内参矩阵 K：
            [fx,  0, cx]
            [ 0, fy, cy]
            [ 0,  0,  1]

        其中：
            fy = (height/2) / tan(fov_rad/2)
            fx = fy * aspect（假设像素是正方形的）
            cx = width/2,  cy = height/2
        """
        fovy = float(self.model.cam_fovy[self.cam_id])
        fov_rad = np.deg2rad(fovy)
        aspect = self.width / self.height

        fy = (self.height / 2.0) / np.tan(fov_rad / 2.0)
        fx = fy * aspect
        cx = self.width / 2.0
        cy = self.height / 2.0

        K = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ])
        return K

    # ------------------------------------------------------------------
    # 核心方法：采集一帧 RGBD 数据
    # ------------------------------------------------------------------
    def capture(self, data: mujoco.MjData = None) -> dict:
        """
        采集一帧 RGB + Depth 图像，并返回对应的相机参数

        参数:
            data: 可选，MuJoCo 数据对象。如果为 None，使用初始化时传入的 data

        返回:
            dict 包含:
                - rgb:        ndarray (H, W, 3), uint8, RGB 图像 [0, 255]
                - depth:      ndarray (H, W),   float32, 深度图（米）
                - intrinsics: ndarray (3, 3),   相机内参矩阵
                - extrinsics: ndarray (4, 4),   相机外参矩阵 [R | t; 0 0 0 1]
                - timestamp:  float,            当前仿真时间戳
                - camera_name: str              绑定的相机名称
        """
        if data is not None:
            self.data = data

        # ==============================================================
        # Step 1: 采集 RGB 图像
        # ==============================================================
        # update_scene 将当前仿真状态同步到渲染场景，并指定使用哪个相机视角
        self.renderer.update_scene(self.data, camera=self.camera_name)
        rgb = self.renderer.render().copy()
        # render() 返回 (H, W, 3) 的 uint8 数组，范围 [0, 255]

        # ==============================================================
        # Step 2: 采集 Depth 图像
        # ==============================================================
        # 开启深度渲染模式，再次 update_scene + render 得到深度图
        self.renderer.enable_depth_rendering()
        self.renderer.update_scene(self.data, camera=self.camera_name)
        depth = self.renderer.render().copy()
        # render() 返回 (H, W) 的 float32 数组
        # 值含义: 视空间深度（相机到物体的垂直距离），单位: 米
        # 远平面之外的像素值为 50（MuJoCo 默认 far=50）
        self.renderer.disable_depth_rendering()

        # ==============================================================
        # Step 3: 获取外参
        # ==============================================================
        # cam_xmat 是 9 个 float 的行优先 3x3 旋转矩阵
        R = self.data.cam_xmat[self.cam_id].reshape(3, 3).copy()
        t = self.data.cam_xpos[self.cam_id].copy()

        extrinsics = np.eye(4)
        extrinsics[:3, :3] = R
        extrinsics[:3, 3] = t

        # ==============================================================
        # Step 4: 组装返回结果
        # ==============================================================
        result = {
            "rgb": rgb,                                    # RGB 图像
            "depth": depth,                                # 深度图
            "intrinsics": self._intrinsics.copy(),         # 内参矩阵
            "extrinsics": extrinsics,                      # 外参矩阵
            "timestamp": self.data.time,                   # 仿真时间戳
            "camera_name": self.camera_name,               # 相机名称
        }
        return result

    # ------------------------------------------------------------------
    # 附加功能：将深度图转换为点云（第二版扩展内容）
    # ------------------------------------------------------------------
    def depth_to_pointcloud(self, depth: np.ndarray) -> np.ndarray:
        """
        将深度图转换为 3D 点云（世界坐标系）

        参数:
            depth: ndarray (H, W), float32, 深度图（米）

        返回:
            points: ndarray (H*W, 3), float32, 世界坐标系下的 3D 点
                   注意：剔除深度值为 50（远平面）的无效点
        """
        H, W = depth.shape
        K = self._intrinsics
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        # 生成像素网格
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        u = u.astype(np.float32)
        v = v.astype(np.float32)

        # 将像素坐标转换为相机坐标系下的 3D 点
        # X_cam = (u - cx) * depth / fx
        # Y_cam = (v - cy) * depth / fy
        # Z_cam = depth
        x_cam = (u - cx) * depth / fx
        y_cam = (v - cy) * depth / fy
        z_cam = depth

        # 堆叠为 (H, W, 3)
        points_cam = np.stack([x_cam, y_cam, z_cam], axis=-1)

        # 将相机坐标系转换到世界坐标系
        # 外参的 R 是 world->camera 旋转，需要转置
        R = self.data.cam_xmat[self.cam_id].reshape(3, 3)
        t = self.data.cam_xpos[self.cam_id]

        # points_world = R^T * (points_cam - t) ... 不准确
        # 实际上 cam_xmat 是 world->camera 旋转, cam_xpos 是 camera 在世界中的位置
        # camera->world: points_world = R^T * points_cam + t
        points_world = points_cam @ R + t  # 利用广播

        # 重塑为 (H*W, 3) 并剔除无效深度点
        points_world = points_world.reshape(-1, 3)
        valid = depth.flatten() < 49.0  # 远平面距离为 50
        return points_world[valid]

    def close(self):
        """
        释放离屏渲染器资源
        """
        if hasattr(self, 'renderer') and self.renderer is not None:
            self.renderer.close()

    def __del__(self):
        self.close()
