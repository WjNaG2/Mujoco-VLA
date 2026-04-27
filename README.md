# Mujoco 上半身遥控与 Benchmark 项目

本仓库用于搭建 Mujoco 上半身末端跟随 + XR 遥操 + 数据采集 + benchmark 的基础项目结构。

## 目录结构

```text
mujoco_vla_project/
├── README.md
├── assets/                  # Mujoco XML / URDF / 场景资源
├── configs/                 # 相机、控制、数据格式配置
├── teleop/                  # XR 遥操桥接代码
├── envs/                    # Mujoco 场景与封装
├── controllers/             # 末端跟随控制器
├── data/
│   ├── raw/                 # 原始录制数据
│   ├── processed/           # 整理后的数据
│   └── samples/             # 样例数据
├── benchmark/
│   ├── datasets/
│   ├── policies/
│   ├── evaluators/
│   └── results/
├── scripts/
│   ├── launch_scene.sh
│   ├── run_teleop.sh
│   ├── record_data.sh
│   ├── train_act.sh
│   └── eval_all.sh
└── docs/
    └── undergrad_guide.md
```

## 环境准备

### 1. 创建 Python 环境

建议使用 `conda` 或 `python -m venv`。

使用 conda：

```bash
conda env create -f environment.yml
conda activate mujoco_vla
```

使用 venv：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 2. 安装 MuJoCo

1. 下载并安装 `mujoco` 或 `mujoco-py`。
2. 设置 `MUJOCO_KEY_PATH` 或 `MUJOCO_LICENSE_KEY` 环境变量。
3. 根据所选版本，确保 `LD_LIBRARY_PATH` 包含 MuJoCo 动态库路径。

示例：

```bash
export MUJOCO_LICENSE_KEY=$HOME/.mujoco/mjkey.txt
export LD_LIBRARY_PATH=$HOME/.mujoco/mujoco210/bin:$LD_LIBRARY_PATH
```

### 3. 克隆 `xr_teleoperate`

建议把 `xr_teleoperate` 克隆到 `third_party/xr_teleoperate`，以便后续开发。

```bash
mkdir -p third_party
git clone https://github.com/unitreerobotics/xr_teleoperate.git third_party/xr_teleoperate
```

### 4. 启动基础脚本

- `scripts/launch_scene.sh`：启动 Mujoco 场景
- `scripts/run_teleop.sh`：启动 XR 遥操桥接
- `scripts/record_data.sh`：录制数据
- `scripts/train_act.sh`：训练 ACT baseline
- `scripts/eval_all.sh`：运行 benchmark

### 5. 最小验证场景

本仓库提供一个用于验证末端控制链路的最小 Mujoco 场景。该场景设计为可重复、易调试，并支持直接可视化末端和目标点。

推荐场景配置：

- 一个机器人上半身模型（简化的上肢末端链路）
- 地面或桌面
- 一个或多个可视化目标点（marker / sphere）
- 1 到 2 个固定视角相机

推荐任务形式：

#### 版本 A：静态单点跟随

- 只控制一个末端，例如右手／前臂末端
- 给一个固定的三维目标点
- 机器人将末端移动到目标点附近

运行方式：

```bash
scripts/launch_scene.sh
```

或者指定场景文件：

```bash
python envs/launch_scene.py --scene envs/simple_end_effector_scene.xml
```

## 运行检查

当前阶段的目标是：

- 搭建项目结构
- 安装依赖
- 确认基础 demo 或依赖可运行
- 编写环境说明

> 现在可以先运行 `scripts/launch_scene.sh` 验证项目目录是否正常。
