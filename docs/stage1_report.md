# 阶段 1 技术报告：Mujoco 上半身场景搭建 + RGBD 相机接口打通

## 1. 概述

本报告对应项目计划中的 **阶段 1（Phase 1）**，核心目标是完成两件事：

1. **搭建上半身双臂 Mujoco 仿真场景**，支持末端位置跟随控制
2. **打通 RGBD 相机接口**，实现从仿真中同步采集 RGB + Depth 图像及相机参数

当前阶段**不做抓取、不做手指精细控制、不做物体接触判定**。重点是让场景先跑起来，让相机先读出来。

---

## 2. 目录结构

```
mujoco_vla_project/                              # 项目根目录
├── docs/
│   └── stage1_report.md              <-- 本文件：Phase 1 技术报告
│
├── envs/                                        # MuJoCo 场景与封装
│   ├── simple_end_effector_scene.xml  # 双臂场景 XML 定义
│   ├── upper_body_env.py             # 场景 Python 封装器
│   ├── rgbd_camera.py                # ⭐ RGBD 相机接口（核心）
│   └── launch_scene.py               # 旧版场景启动器
│
├── controllers/                                 # 控制算法
│   ├── end_effector_controller.py    # 末端位置 IK 控制器
│   └── target_generator.py           # 目标点生成器（A/B/C 版本）
│
├── configs/
│   └── camera_config.yaml            # 相机配置文件
│
├── scripts/                                     # 演示与工具脚本
│   ├── launch_scene.sh               # 场景启动脚本（更新版）
│   ├── capture_rgbd_demo.py          # RGBD 相机采集演示
│   ├── run_follow_demo.py            # 完整末端跟随演示（后台运行 + 数据保存）
│   ├── run_follow_visual.py          # 带 MuJoCo 可视化窗口的跟随演示
│   └── inspect_recorded_data.py      # ⭐ 数据可视化核查脚本（新增）
│
├── data/
│   └── samples/
│       ├── camera_captures/          # RGBD 采集输出（单帧验证）
│       └── follow_demo_*/            # 跟随演示输出（完整 episode）
│
└── ... (其他目录结构保持不变)
```

---

## 3. 各文件功能说明

### 3.1 场景定义：`envs/simple_end_effector_scene.xml`

定义了一个包含**左右双臂**的 Mujoco 仿真场景：

| 组件 | 说明 |
|------|------|
| 右臂 | 3 DOF：shoulder_yaw → shoulder_pitch → elbow_pitch |
| 左臂 | 3 DOF：shoulder_yaw → shoulder_pitch → elbow_pitch |
| 目标点 | 红色球（右手目标）、蓝色球（左手目标） |
| 相机 | camera_front（前置）、camera_side（侧上方） |
| 执行器 | 全部使用 position actuator，通过 ctrl 发送关节目标角度 |

每臂包含 3 个关节，共 **6 DOF**，足以验证上半身末端位置跟随的基本功能。

### 3.2 场景封装：`envs/upper_body_env.py`

将 MuJoCo 场景封装为 Python 类 `UpperBodyEnv`，提供以下核心接口：

| 方法 | 功能 |
|------|------|
| `reset()` | 重置仿真到初始状态 |
| `step()` | 向前仿真一步 |
| `set_joint_targets()` | 设置关节目标位置（写入 ctrl） |
| `get_observation()` | 获取完整观测：关节角度、末端位置、目标位置、时间戳 |
| `get_camera_info()` | 计算指定相机的内参和外参矩阵 |
| `set_target_position()` | 移动目标点的三维位置 |

### 3.3 ⭐ RGBD 相机接口：`envs/rgbd_camera.py`（核心模块）

这是**本阶段最重要的模块**，请参见第 4 节的详细说明。

### 3.4 末端控制器：`controllers/end_effector_controller.py`

实现了基于**雅可比伪逆（Jacobian Pseudoinverse）**的 IK 控制器：

- `EndEffectorController`：单臂末端跟随，支持 "right" 和 "left"
- `BimanualController`：双臂联合控制器，同时控制左右末端

控制原理：
1. 计算末端位置误差: e = p_desired - p_current
2. 比例控制得到期望速度: v = Kp * e
3. 通过雅可比伪逆映射到关节速度: q_dot = J^† · v
4. 积分得到关节目标位置: q_target += q_dot * dt

### 3.5 目标点生成器：`controllers/target_generator.py`

提供三种版本的目标点生成：

| 版本 | 类名 | 用途 |
|------|------|------|
| A：静态单点 | `StaticTargetGenerator` | 基础调试，检查是否能到达一个点 |
| B：离散多点 | `DiscreteTargetGenerator` | 测试多目标切换性能 |
| C：连续轨迹 | `CircleTargetGenerator` | 测试轨迹跟踪能力 |
| C：连续轨迹 | `LineTargetGenerator` | 测试直线轨迹跟踪 |

### 3.6 RGBD 采集演示：`scripts/capture_rgbd_demo.py`

独立的 RGBD 采集脚本，演示如何：
1. 加载场景 → 2. 初始化 RGBDCamera → 3. 运行仿真 → 4. 采集并保存 RGB + Depth

输出文件：
```
data/samples/camera_captures/
├── camera_front_rgb.png         # RGB 图像
├── camera_front_depth.npy       # Depth 原始数据（float32，单位：米）
├── camera_front_depth_vis.png   # 深度图伪彩色可视化
├── camera_side_rgb.png
├── camera_side_depth.npy
├── camera_side_depth_vis.png
└── camera_params.txt            # 相机内参和外参
```

### 3.7 跟随演示：`scripts/run_follow_demo.py`

整合场景 + 控制器 + 目标生成器 + RGBD 相机采集的完整演示：

```bash
python scripts/run_follow_demo.py --mode static      # 静态点跟随
python scripts/run_follow_demo.py --mode circle       # 圆形轨迹
python scripts/run_follow_demo.py --mode bimanual     # 双臂跟随
```

输出包括：跟踪误差曲线、状态序列、RGBD 序列。

### 3.8 相机配置：`configs/camera_config.yaml`

YAML 格式的相机分辨率与启用配置，便于扩展多相机设置。

### 3.9 ⭐ 数据核查脚本：`scripts/inspect_recorded_data.py`

数据可视化核查脚本，用于检查采集到的 RGBD 和 episode 数据是否正确。
支持两种检查模式：

**模式 A：检查 RGBD 采集输出**（camera_captures 目录）
- 显示 RGB 图像和 Depth 伪彩色图
- 检查深度值范围是否合理
- 验证 camera_params.txt 中的内外参

**模式 B：检查跟随演示输出**（episode_data.npz）
- 绘制关节目标 vs 实际位置对比图（验证控制输入是否正确）
- 验证 actuator_ctrl 是否与 joint_targets 一致（验证 set_joint_targets 写入）
- 绘制末端跟踪误差曲线（验证最终输出质量）
- 绘制 3D 轨迹对比图（目标轨迹 vs 实际轨迹）

详见第 7 节的使用说明。

---

## 4. ⭐ RGBD 相机接口实现详解（核心内容）

### 4.1 为什么这是核心模块

在 VLA / Imitation Learning pipeline 中，**视觉观测是最关键的输入通道**。
每一帧训练数据包含：
- **RGB 图像**：提供语义信息（颜色、纹理、物体类别）
- **Depth 图像**：提供几何信息（距离、形状、深度关系）
- **相机参数**：将 2D 像素映射回 3D 世界的必要条件

本模块打通了**"从仿真到数据"的完整链路**：

```
MuJoCo 仿真 → 离屏渲染 → RGB + Depth + 参数 → 统一字典输出 → 数据存储
```

### 4.2 接口设计

```python
class RGBDCamera:
    def __init__(self, model, data, camera_name, width=640, height=480):
        ...

    def capture(self, data=None) -> dict:
        # 返回：
        #   rgb:         (H, W, 3) uint8,  [0, 255]
        #   depth:       (H, W)   float32, 单位: 米
        #   intrinsics:  (3, 3)   float32, 相机内参 K
        #   extrinsics:  (4, 4)   float32, 相机外参 [R | t]
        #   timestamp:   float,   仿真时间
        #   camera_name: str
        ...

    def depth_to_pointcloud(self, depth) -> (N, 3) ndarray:
        # 将深度图转为世界坐标系下的点云
        ...
```

### 4.3 RGB 图像实现原理

1. 使用 `mujoco.Renderer` 创建**离屏渲染器**
   - 不弹出窗口，直接在 GPU 内存中渲染
   - 通过 `width` / `height` 参数控制输出图像分辨率
2. 每次调用 `capture()` 时：
   - `renderer.update_scene(data, camera=camera_name)`：将当前仿真状态同步到渲染场景
   - `renderer.render()`：执行渲染并返回 RGB 缓冲区

```
renderer = mujoco.Renderer(model, height=480, width=640)
renderer.update_scene(data, camera='camera_front')
rgb = renderer.render()  # 返回 (480, 640, 3), dtype=uint8
```

### 4.4 Depth 图像实现原理

1. 通过 `renderer.enable_depth_rendering()` 开启深度渲染模式
2. 再次调用 `update_scene()` + `render()` 获得深度图
3. **重要**：同一帧中 RGB 和 Depth 需要分别渲染，但同帧数据严格对齐

```
renderer.enable_depth_rendering()
renderer.update_scene(data, camera='camera_front')
depth = renderer.render()  # 返回 (480, 640), dtype=float32
renderer.disable_depth_rendering()
```

深度值含义：
- **视空间深度**（从相机到物体的垂直距离），单位：**米**
- 有效范围通常在 `[near, far]` 之间
- 远平面以外的像素值为 `50.0`（MuJoCo 默认 `far=50`）

### 4.5 相机内参矩阵计算

MuJoCo 相机通过 `fovy`（垂直视场角）和图像宽高比定义投影，
没有直接给出像素焦距。我们手动计算内参矩阵 K：

```python
fov_rad = np.deg2rad(fovy)
fy = (height / 2) / tan(fov_rad / 2)
fx = fy * aspect         # 假设像素是正方形的
cx = width / 2
cy = height / 2

K = [[fx,   0, cx],
     [ 0,  fy, cy],
     [ 0,   0,  1]]
```

**为什么这样计算？**
- `fovy` 定义了垂直方向的视角范围
- 在针孔相机模型下，传感器半高度与焦距满足：`tan(fov/2) = (h/2) / fy`
- 因此 `fy = (h/2) / tan(fov/2)`
- `fx` 通过宽高比推得，保证像素是正方形

### 4.6 相机外参获取

MuJoCo 在 `data` 对象中已经计算好了世界坐标系下的相机位姿：

| 字段 | 说明 | 获取方式 |
|------|------|----------|
| 位置 | 相机在世界坐标系中的位置 | `data.cam_xpos[cam_id]` (3,) |
| 旋转 | 从世界到相机的旋转矩阵 | `data.cam_xmat[cam_id]` (9,) → reshape(3,3) |

外参矩阵：
```python
R = data.cam_xmat[cam_id].reshape(3, 3)  # world → camera
t = data.cam_xpos[cam_id]                 # camera world position
extrinsics = [[R, t], [0, 0, 0, 1]]       # (4, 4)
```

### 4.7 点云生成（第二版可扩展）

`depth_to_pointcloud()` 方法将深度图转换为世界坐标系下的 3D 点云：

```
像素 (u, v) → 相机坐标系：
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    Z = depth

相机坐标系 → 世界坐标系：
    P_world = R^T * P_cam + t     (其中 R 是 cam_xmat)
```

注意：`cam_xmat` 是 `world → camera` 的旋转，从相机到世界的转换需要用 `R^T`。

### 4.8 时间戳对齐

RGB 和 Depth 在同一帧中先后采集，使用同样的 `data.time` 作为时间戳，
保证 RGB 与 Depth 的帧级严格对齐。

### 4.9 多相机支持

为每个相机创建一个独立的 `RGBDCamera` 实例即可支持多视角：

```python
cam_front = RGBDCamera(model, data, "camera_front")
cam_side  = RGBDCamera(model, data, "camera_side")
```

### 4.10 采集验证结果

运行 `scripts/capture_rgbd_demo.py` 后典型输出：

| 项目 | 说明 |
|------|------|
| RGB 图像 | 640×480, uint8, 范围 [0, 255] |
| Depth 图像 | 640×480, float32, 范围 [1.4, 50.0] 米 |
| 内参 K | fx≈570, fy≈570, cx=320, cy=240（640×480 下） |
| 外参 | 4×4 矩阵，包含旋转和平移 |
| 时间戳 | 与仿真时间对齐 |

---

## 5. 使用教程

### 5.1 环境激活

```bash
conda activate mujoco_vla
# 或
source .venv/bin/activate
```

### 5.2 采集 RGBD 图像

```bash
python scripts/capture_rgbd_demo.py
```

### 5.3 运行末端跟随演示

#### 方式 A：后台运行 + 数据保存（无窗口）

适合批量测试和数据采集，运行后自动保存结果到 `data/samples/`：

```bash
# 静态单点跟随（版本 A）
python scripts/run_follow_demo.py --mode static

# 圆形轨迹跟随（版本 C）
python scripts/run_follow_demo.py --mode circle

# 双臂同时跟随
python scripts/run_follow_demo.py --mode bimanual
```

#### 方式 B：带 MuJoCo 可视化窗口（推荐新手使用）

适合调试和观察，会弹出 MuJoCo 交互式窗口，实时显示机器人跟随目标点的动画：

```bash
# 静态单点跟随（默认）
python scripts/run_follow_visual.py

# 圆形轨迹跟随（目标点沿圆形移动）
python scripts/run_follow_visual.py --mode circle

# 双臂同时跟随
python scripts/run_follow_visual.py --mode bimanual
```

操作说明（MuJoCo viewer）：
- 鼠标左键拖拽：旋转视角
- 鼠标滚轮：缩放
- 空格键：暂停/继续仿真
- ESC 键：退出

### 5.4 启动可视化场景

```bash
python envs/launch_scene.py --scene envs/simple_end_effector_scene.xml
```

---

## 6. 数据输出与验证（新增章节）

本章回答两个关键问题：
1. **控制输出（joint target / actuator control / joint velocity）被输出到了哪里？**
2. **如何直观地获取和验证 RGBD 相机记录的数据是否正确？**

---

### 6.1 控制输入/输出数据的流向

完整的数据流如下：

```
【用户输入】给定末端目标位置 (p_target_right, p_target_left)
    │
    ▼
IK 控制器 (EndEffectorController.update)
    │  计算: e = p_target - p_current
    │       v = Kp * e
    │       q_dot = J^† · v
    │       q_target += q_dot * dt
    │
    ▼  ~~~~~~~~~ 【控制输入】记录到 episode_data.npz ~~~~~~~~~
joint_targets (6,)   ← IK 控制器输出的**期望**关节角度（弧度）
    │
    ▼
env.set_joint_targets(joint_targets)  → 写入 data.ctrl
    │
    ▼  ~~~~~~~~~ 【控制输入】记录到 episode_data.npz ~~~~~~~~~
actuator_ctrl (6,)   ← 实际写入 MuJoCo actuator 的 ctrl 值（弧度）
    │
    ▼
env.step()  →  MuJoCo 物理仿真
    │
    ▼  ~~~~~~~~~ 【机器人状态】记录到 episode_data.npz ~~~~~~~~~
joint_positions (6,)  ← 仿真后的**实际**关节角度（弧度）
joint_velocities (6,) ← 仿真后的**实际**关节速度（弧度/秒）
r_ee_pos / l_ee_pos  ← 末端实际位置（米，世界坐标系）
    │
    ▼
误差 = ‖p_target - p_actual‖  →  记录到 episode_data.npz 的 errors
```

#### 关键验证点

- **joint_targets vs joint_positions**: 表示"期望"和"实际"的差距。理想情况下两者应该非常接近（因为 position actuator 的跟踪精度很高）。
- **actuator_ctrl vs joint_targets**: 应该完全一致，因为 `set_joint_targets()` 只是将 joint_targets 拷贝到 `data.ctrl`。如果两者有差异说明代码有 bug。
- **errors**: 末端位置的最终跟踪误差。应该随时间收敛到接近 0。

---

### 6.2 episode_data.npz 字段详解

运行 `python scripts/run_follow_demo.py` 后，输出目录下的 `episode_data.npz` 包含以下字段：

| 字段名 | 形状 | 含义 | 类别 |
|--------|------|------|------|
| `joint_targets` | (N, 6) | 【控制输入】IK 控制器输出的期望关节角度（弧度） | 控制输入 |
| `actuator_ctrl` | (N, 6) | 【控制输入】实际写入 MuJoCo ctrl 的值（应与 joint_targets 一致） | 控制输入 |
| `joint_positions` | (N, 6) | 【机器人状态】MuJoCo 仿真后的实际关节角度（弧度） | 机器人状态 |
| `joint_velocities` | (N, 6) | 【机器人状态】MuJoCo 仿真后的实际关节速度（弧度/秒） | 机器人状态 |
| `errors` | (N,) | 【末端跟踪】末端目标位置与实际位置的欧氏距离（米） | 末端跟踪 |
| `target_positions` | (N, 3) | 【末端跟踪】给定的末端目标位置（世界坐标系，米） | 末端跟踪 |
| `actual_positions` | (N, 3) | 【末端跟踪】仿真得到的末端实际位置（世界坐标系，米） | 末端跟踪 |
| `timestamps` | (N,) | 【时间戳】每步的仿真时间（秒） | 元信息 |
| `mode` | str | 【元信息】运行模式名称 | 元信息 |

其中 N = 仿真总步数（默认 2000）。

> **注意**：`joint_targets` 是 6 维数组，顺序与场景 XML 中定义的 actuator 一致：
> [r_shoulder_yaw, r_shoulder_pitch, r_elbow, l_shoulder_yaw, l_shoulder_pitch, l_elbow]

---

### 6.3 RGBD 相机采集输出详解

运行 `python scripts/capture_rgbd_demo.py` 后输出到 `data/samples/camera_captures/`：

#### `camera_front_rgb.png` 是什么？

这是 MuJoCo 离屏渲染器输出的 RGB 图像。它展示的是**从前置相机视角看到的仿真场景**：

```
┌──────────────────────────────────────┐
│  RGB 图像 (640×480, uint8)           │
│                                      │
│  你可以直观看到：                      │
│  • 机器人上半身（双肩+双肘）            │
│  • 红色球 = 右手目标点                 │
│  • 蓝色球 = 左手目标点                 │
│  • 地面网格                           │
│                                      │
│  用途：检查场景是否正常渲染、             │
│  目标点位置是否在视野内、                 │
│  相机视角是否合适                       │
└──────────────────────────────────────┘
```

#### `camera_front_depth.npy` 是什么？

这是与 RGB 图像**同帧**的深度图，保存为 NumPy 数组：

```python
depth = np.load("camera_front_depth.npy")
# depth.shape = (480, 640), dtype = float32
# 每个像素值 = 相机到物体的垂直距离（米）
# 远平面（>50米）的像素值为 50.0
```

#### `camera_front_depth_vis.png` 是什么？

这是深度图的伪彩色可视化（便于肉眼查看）：
- 颜色越偏紫 → 越近
- 颜色越偏黄 → 越远
- 使用 viridis 颜色映射

#### 如何验证 RGB 和 Depth 是否对齐？

1. **时间戳验证**：RGB 和 Depth 使用同一个 `data.time` 作为时间戳，帧级严格对齐
2. **可视化验证**：运行 `python scripts/inspect_recorded_data.py --dir data/samples/camera_captures` 会弹出对比图，左侧是 RGB，右侧是 Depth 伪彩色图，可以肉眼确认两者内容一致
3. **尺寸验证**：RGB 和 Depth 的分辨率完全相同（640×480），每个像素一一对应

#### RGBD 采集输出文件说明

| 文件 | 含义 | 如何查看 |
|------|------|----------|
| `camera_front_rgb.png` | RGB 彩色图像 | 用图片浏览器直接打开 |
| `camera_front_depth.npy` | 深度原始数据（float32 米） | `np.load(...)` 加载 |
| `camera_front_depth_vis.png` | 深度伪彩色可视化 | 用图片浏览器直接打开 |
| `camera_params.txt` | 相机内参 K 和外参 [R\|t] | 文本编辑器打开 |

---

### 6.4 使用数据核查脚本验证数据

`scripts/inspect_recorded_data.py` 提供了一个统一的入口，用于直观检查采集到的数据是否正确。

#### 检查 RGBD 采集输出

```bash
# 检查 camera_captures 目录下的 RGBD 数据
python scripts/inspect_recorded_data.py --dir data/samples/camera_captures
```

会依次弹出：
- 每个相机的 RGB 图像 vs Depth 伪彩色对比图
- 控制台输出深度值范围、有效/无效点统计

✅ **通过标准**：
- RGB 图像清晰可见机器人上半身和目标点
- Depth 有效值范围合理（通常 1~5 米）
- Depth 中机器人轮廓与 RGB 中对应位置一致

#### 检查跟随演示输出

```bash
# 检查 episode_data.npz 中的完整数据
python scripts/inspect_recorded_data.py --dir data/samples/follow_demo_static

# 或直接指定 npz 文件
python scripts/inspect_recorded_data.py --npz data/samples/follow_demo_static/episode_data.npz
```

会依次弹出 **4 张检查图**：

**图 1：关节目标 vs 实际位置**（控制输入 vs 机器人状态）
- 蓝色虚线 = `joint_targets`（期望角度）
- 红色实线 = `joint_positions`（实际角度）
- ✅ 两者应几乎重合（差异 < 0.01 rad）

**图 2：actuator_ctrl 验证**
- 绘制 `|actuator_ctrl - joint_targets|`
- ✅ 偏差应为 0（如果显示"完全一致"绿色提示）

**图 3：末端跟踪误差**
- 蓝色曲线 = 末端位置误差随时间变化
- 红色虚线 = 5cm 阈值
- ✅ 误差应收敛到 5cm 以下

**图 4：3D 轨迹对比**
- 红色虚线 = 目标轨迹
- 蓝色实线 = 实际轨迹
- ✅ 两者应基本重合

---

## 7. 完成标准检查

| 标准 | 状态 | 如何验证 |
|------|------|----------|
| 能在 Mujoco 中给定末端目标点 | ✅ | 通过 `set_joint_targets()` |
| 机器人上半身能稳定接近目标点 | ✅ | 检查 `errors` 是否 < 5cm |
| 能同步记录 RGB 图像 | ✅ | `capture()` 返回 `rgb` 字段 |
| 能同步记录 Depth 图像 | ✅ | `capture()` 返回 `depth` 字段 |
| 能记录控制输入（joint_targets） | ✅ | `episode_data.npz` 的 `joint_targets` |
| 能记录控制输入（actuator_ctrl） | ✅ | `episode_data.npz` 的 `actuator_ctrl` |
| 能记录机器人状态（joint_positions + joint_velocities） | ✅ | `episode_data.npz` 的对应字段 |
| 能记录 target position | ✅ | `episode_data.npz` 的 `target_positions` |
| 能记录 eef position | ✅ | `episode_data.npz` 的 `actual_positions` |
| 多次重复运行结果一致 | ✅ | 确定性仿真 |
| RGB 与 Depth 通过时间戳对齐 | ✅ | 同帧 `data.time` |
| 有相机内参和外参输出 | ✅ | `capture()` 的 `intrinsics` / `extrinsics` |

---

## 8. 常见问题

### Q: camera_front_rgb.png 这个图片体现了什么信息？

A: 这张图是 MuJoCo 场景从前置相机视角的截图，展示了：
- 仿真环境中的机器人上半身模型（双肩、双肘）
- 红色球体（右手目标点）、蓝色球体（左手目标点）
- 地面和背景
- 你可以通过它检查：场景是否正常加载、目标点位置是否正确、相机视角是否合适

### Q: 为什么 RGB 和 Depth 需要渲染两次？
A: MuJoCo 的 Renderer 在单次渲染下只能输出一种数据类型。需要先渲染 RGB，开启 depth mode 后再渲染一次得到 depth。这是 MuJoCo 的设计限制，但同帧数据严格对齐。

### Q: 为什么深度图的远平面值是 50？
A: 这是 MuJoCo 的默认 `far` 平面距离（50 米）。在场景中距离超过 50 米的物体不会被渲染，表现为 50.0 值。

### Q: 控制器无法到达目标点怎么办？
A: 检查以下几点：
1. 目标点是否在机器人工作空间内（reachable workspace）
2. `kp` 增益是否合适（太小响应慢，太大可能振荡）
3. 关节是否达到限位（检查 `_clamp_joint_targets`）

### Q: 如何添加新的相机？
A: 两步操作：
1. 在 `simple_end_effector_scene.xml` 中添加 `<camera>` 定义
2. 在 `camera_config.yaml` 中添加对应相机的配置

### Q: 如何使用 inspect_recorded_data.py 检查我的数据？
A: 提供了三种使用方式：
```bash
# 1. 检查 RGBD 采集输出
python scripts/inspect_recorded_data.py --dir data/samples/camera_captures

# 2. 检查跟随演示输出（自动找到 npz 文件）
python scripts/inspect_recorded_data.py --dir data/samples/follow_demo_static

# 3. 直接指定 npz 文件
python scripts/inspect_recorded_data.py --npz data/samples/follow_demo_static/episode_data.npz
```
该脚本会弹出多个对比图表，让你直观验证数据是否正确。

---

## 9. 下一步（Phase 2）

阶段 1 完成后，下一阶段（Phase 2）将把 `xr_teleoperate` 的遥操输入接入 Mujoco，
实现 XR → Bridge → 末端目标位置 → 机器人跟随的完整链路。
