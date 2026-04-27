# Mujoco 上半身遥操与 Benchmark 项目

本仓库用于搭建 **Mujoco 上半身末端跟随 + XR 遥操 + 数据采集 + 策略 Benchmark** 的基础项目结构。

完整的项目规划请参见 [`undergrad_mujoco_teleop_plan(1).md`](./undergrad_mujoco_teleop_plan(1).md)。

---

## 项目目标

逐步完成以下四件事：

1. ✅ **（已完成）** 在 Mujoco 中搭建上半身双臂场景，打通 **RGBD 相机接口**
2. □ 将 XR 遥操输入接入 Mujoco，实现上半身末端位置跟随
3. □ 采集 50 组仅上半身的遥操作数据
4. □ 编写统一 simulation benchmark，对比 ACT / GR00T / DP3

---

## 目录结构

```text
mujoco_vla_project/
├── README.md                          # 本项目 README
├── undergrad_mujoco_teleop_plan(1).md # 完整项目规划文档
├── environment.yml                    # Conda 环境文件
├── requirements.txt                   # Pip 依赖文件
│
├── assets/                            # Mujoco XML / URDF / 场景资源
│   └── scene.xml
│
├── configs/                           # 配置文件
│   └── camera_config.yaml             # ⭐ 相机分辨率与启用配置
│
├── envs/                              # Mujoco 场景封装
│   ├── simple_end_effector_scene.xml  # ⭐ 双臂上半身场景 XML（6 DOF）
│   ├── upper_body_env.py              # ⭐ 场景 Python 封装器
│   ├── rgbd_camera.py                 # ⭐⭐ RGBD 相机接口（核心模块）
│   └── launch_scene.py                # 场景启动器
│
├── controllers/                       # 控制算法
│   ├── end_effector_controller.py     # ⭐ 基于雅可比伪逆的 IK 控制器
│   └── target_generator.py            # ⭐ 目标点生成器（A/B/C 版本）
│
├── scripts/                           # 演示与工具脚本
│   ├── launch_scene.sh                # 场景启动脚本
│   ├── capture_rgbd_demo.py           # ⭐ RGBD 相机采集演示
│   ├── run_follow_demo.py             # ⭐⭐ 完整末端跟随演示（含 RGBD 采集）
│   ├── run_teleop.sh                  # XR 遥操启动
│   ├── record_data.sh                 # 数据录制
│   ├── record_data.py                 # 数据录制脚本
│   ├── train_act.sh                   # ACT 训练
│   └── eval_all.sh                    # Benchmark 评估
│
├── teleop/                            # XR 遥操桥接代码
│   └── run_teleop.py
│
├── data/
│   ├── raw/                           # 原始录制数据
│   ├── processed/                     # 整理后的数据
│   └── samples/                       # ⭐ 样例数据输出目录
│       ├── camera_captures/           #   RGBD 采集输出
│       └── follow_demo_*/             #   跟随演示输出
│
├── benchmark/                         # Benchmark 相关
│   ├── datasets/
│   ├── policies/
│   │   └── train_act.py
│   ├── evaluators/
│   │   └── run_benchmark.py
│   └── results/
│
├── docs/
│   ├── stage1_report.md               # ⭐⭐ 阶段 1 技术报告（含 RGBD 接口详解）
│   └── undergrad_guide.md             # 本科生快速上手指南
│
└── third_party/
    └── xr_teleoperate/                # 第三方: XR 遥操仓库
```

> `⭐` 标记为阶段 1 的核心产出，`⭐⭐` 为最核心模块。

---

## 环境准备

### 1. 创建 Python 环境

使用 conda（推荐）：

```bash
conda env create -f environment.yml
conda activate mujoco_vla
```

或使用 venv：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 2. 安装 MuJoCo

```bash
pip install mujoco
```

MuJoCo 从 3.x 版本起已开源，无需 license key。直接 `pip install mujoco` 即可。

### 3. 克隆 xr_teleoperate（后续阶段使用）

```bash
mkdir -p third_party
git clone https://github.com/unitreerobotics/xr_teleoperate.git third_party/xr_teleoperate
```

---

## 阶段 1：场景搭建 + RGBD 相机接口（已完成）

### 功能概览

| 模块 | 文件 | 功能 |
|------|------|------|
| 场景定义 | `envs/simple_end_effector_scene.xml` | 双臂 6 DOF 上半身 + 目标点 + 双视角相机 |
| 场景封装 | `envs/upper_body_env.py` | `reset()` / `step()` / `get_observation()` 统一接口 |
| ⭐⭐ RGBD 相机 | `envs/rgbd_camera.py` | RGB + Depth + 内参 + 外参 + 时间戳 统一输出 |
| IK 控制器 | `controllers/end_effector_controller.py` | 雅可比伪逆末端位置跟随（收敛误差 < 1mm） |
| 目标生成器 | `controllers/target_generator.py` | 静态点 / 圆轨迹 / 直线轨迹 / 离散点 |
| 相机配置 | `configs/camera_config.yaml` | 分辨率、启用/关闭配置 |

### 运行演示

#### 方式 A：后台运行 + 数据保存（无窗口）

适合批量测试和数据采集，运行后自动保存结果到 `data/samples/`：

```bash
# 1. 采集 RGBD 图像（输出到 data/samples/camera_captures/）
python scripts/capture_rgbd_demo.py

# 2. 静态单点末端跟随 + RGBD 同步采集
python scripts/run_follow_demo.py --mode static

# 3. 圆形轨迹跟随
python scripts/run_follow_demo.py --mode circle

# 4. 双臂同时跟随
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

#### 方式 C：仅启动可视化场景

```bash
python envs/launch_scene.py --scene envs/simple_end_effector_scene.xml
```

### RGBD 相机接口说明（核心）

`envs/rgbd_camera.py` 中的 `RGBDCamera` 类封装了 MuJoCo 离屏渲染，输出格式为：

```python
cam = RGBDCamera(model, data, "camera_front", width=640, height=480)
result = cam.capture()
# result = {
#     "rgb":          (480, 640, 3) uint8,     # RGB 图像
#     "depth":        (480, 640)   float32,    # 深度图（米）
#     "intrinsics":   (3, 3)       float32,    # 相机内参 K
#     "extrinsics":   (4, 4)       float32,    # 相机外参 [R | t]
#     "timestamp":    float,                   # 仿真时间戳
#     "camera_name":  str                      # 相机名称
# }
```

详细实现原理（内参计算、外参获取、RGB/Depth 渲染、时间戳对齐）请见 [`docs/stage1_report.md`](./docs/stage1_report.md) 第 4 节。

### IK 控制器说明

基于**雅可比伪逆**的末端位置控制器：

1. 计算位置误差: `e = p_target - p_current`
2. 比例控制: `v = Kp * e`
3. 雅可比伪逆: `q_dot = J^T * (J*J^T + λI)^(-1) * v`
4. 积分: `q_target += q_dot * dt`

关键修复：确保 `jnt_range` 使用弧度单位（MuJoCo 内部使用弧度），避免限位钳位错误。

---

## 阶段 1 验证结果

| 测试项 | 结果 |
|--------|------|
| 单臂静态点收敛误差 | **< 0.001 m**（收敛到目标点） |
| 双臂静态点跟随 | 右臂误差 0.02m, 左臂误差 0.25m（受限于运动学范围） |
| 圆轨迹跟踪平均误差 | **0.021 m** |
| RGB 图像输出 | (480, 640, 3), uint8, [0, 255] |
| Depth 图像输出 | (480, 640), float32, 单位米 |
| 相机内参 fx/fy | ~772 / ~579（640×480分辨率下） |
| RGB 与 Depth 对齐 | ✅ 同帧 `data.time` 严格对齐 |
| 雅可比满秩 | ✅ 双臂均满秩 (rank 3/3) |

---

## 后续阶段

- **Phase 2**：将 XR Teleoperate 接入 Mujoco（bridge 层 + 坐标转换）
- **Phase 3**：采集 50 组标准上半身数据
- **Phase 4**：统一 Benchmark（ACT / GR00T / DP3）

详细规划请参见 [`undergrad_mujoco_teleop_plan(1).md`](./undergrad_mujoco_teleop_plan(1).md)。

---

## 新手建议

如果你是本科生新手，请按以下顺序推进：

1. **先跑场景**：`python scripts/capture_rgbd_demo.py`
2. **再看相机**：理解 `rgbd_camera.py` 的输出
3. **再调跟随**：`python scripts/run_follow_demo.py --mode static`
4. **再接入遥操**：Phase 2
5. **再录数据、跑 benchmark**：Phase 3 / 4

**每次只做一个闭环，确认无误后再叠加功能。**

---

## 更多文档

- [`docs/stage1_report.md`](./docs/stage1_report.md) — 阶段 1 详细技术报告（含 RGBD 相机实现原理）
- [`docs/undergrad_guide.md`](./docs/undergrad_guide.md) — 本科生快速上手指南
- [`undergrad_mujoco_teleop_plan(1).md`](./undergrad_mujoco_teleop_plan(1).md) — 完整项目规划
