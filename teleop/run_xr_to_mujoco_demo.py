#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_xr_to_mujoco_demo.py —— XR 输入 → MuJoCo 仿真机器人跟随 完整演示

完整链路：
    XR 输入 (SimulatedXRSource) → 桥接层 (XRToMuJoCoBridge)
    → 末端目标位置 → 控制器 (BimanualController) → MuJoCo 仿真机器人

运行方式：
    # 带 MuJoCo viewer（可视化）
    conda run -n mujoco_vla python teleop/run_xr_to_mujoco_demo.py --mode circle --viewer

    # 多种运动模式切换
    conda run -n mujoco_vla python teleop/run_xr_to_mujoco_demo.py --mode raise_hands --viewer
    conda run -n mujoco_vla python teleop/run_xr_to_mujoco_demo.py --mode reach --viewer
    conda run -n mujoco_vla python teleop/run_xr_to_mujoco_demo.py --mode wave --viewer

    # 无 viewer（纯仿真，速度更快，输出统计数据）
    conda run -n mujoco_vla python teleop/run_xr_to_mujoco_demo.py --mode circle --steps 3000

输出：
    data/samples/xr_teleop_demo/
    ├── episode_data.npz      # 状态/动作/误差序列
    ├── tracking_error.png    # 跟踪误差曲线
    └── summary.txt           # 运行摘要
"""

import os
import sys
import argparse
import time as time_module

# 将项目根目录加入 Python 路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import numpy as np
import matplotlib
matplotlib.use('Agg')  # 非交互式后端，避免 viewer 冲突
import matplotlib.pyplot as plt

from envs.upper_body_env import UpperBodyEnv
from controllers.end_effector_controller import BimanualController
from teleop.teleop_bridge import SimulatedXRSource, XRToMuJoCoBridge


def plot_tracking_error(errors_right, errors_left, save_path):
    """绘制双臂跟踪误差曲线"""
    plt.figure(figsize=(12, 5))

    plt.subplot(1, 2, 1)
    plt.plot(errors_right, label="Right Arm", color="red", linewidth=1)
    plt.xlabel("Simulation Step")
    plt.ylabel("Position Error (m)")
    plt.title("Right Arm Tracking Error")
    plt.grid(True, alpha=0.3)
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.plot(errors_left, label="Left Arm", color="blue", linewidth=1)
    plt.xlabel("Simulation Step")
    plt.ylabel("Position Error (m)")
    plt.title("Left Arm Tracking Error")
    plt.grid(True, alpha=0.3)
    plt.legend()

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close()
    print(f"  [Saved] Error plot -> {save_path}")


def main():
    parser = argparse.ArgumentParser(
        description="XR → MuJoCo 完整链路演示"
    )
    parser.add_argument(
        "--mode", type=str, default="circle",
        choices=["circle", "reach", "raise_hands", "wave"],
        help="模拟 XR 运动模式（default: circle）"
    )
    parser.add_argument(
        "--steps", type=int, default=3000,
        help="仿真总步数（default: 3000）"
    )
    parser.add_argument(
        "--viewer", action="store_true",
        help="启用 MuJoCo viewer（可视化）"
    )
    parser.add_argument(
        "--kp", type=float, default=3.0,
        help="控制器比例增益（default: 3.0）"
    )
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="XR 运动速度缩放系数（default: 1.0）"
    )
    args = parser.parse_args()

    print("=" * 70)
    print("XR → MuJoCo 完整链路演示")
    print(f"  运动模式: {args.mode}")
    print(f"  仿真步数: {args.steps}")
    print(f"  启用 Viewer: {args.viewer}")
    print("=" * 70)

    # ==========================================================
    # 1. 初始化 MuJoCo 环境
    # ==========================================================
    print("\n[1/5] 初始化 MuJoCo 场景...")
    env = UpperBodyEnv()
    env.reset()

    if args.viewer:
        # 启动 MuJoCo 交互式 viewer
        print("  启用 MuJoCo viewer（按 ESC 退出）...")
        try:
            viewer = env.launch_viewer()
        except Exception as e:
            print(f"  ⚠️  viewer 启动失败: {e}")
            print("  继续无 viewer 运行")
            args.viewer = False
            viewer = None
    else:
        viewer = None
    print("  ✅ 场景加载完成")

    # ==========================================================
    # 2. 初始化 XR 数据源和桥接层
    # ==========================================================
    print("\n[2/5] 初始化 XR 桥接层...")
    xr_source = SimulatedXRSource(mode=args.mode, speed_scale=args.speed)
    bridge = XRToMuJoCoBridge()
    print(f"  XR Source: {args.mode}, speed={args.speed}")
    print("  ✅ 桥接层就绪")

    # ==========================================================
    # 3. 初始化双臂控制器
    # ==========================================================
    print("\n[3/5] 初始化双臂控制器...")
    controller = BimanualController(env, kp=args.kp)
    print(f"  控制器: kp={args.kp}")

    # 让控制器目标跟踪当前关节位置作为初始状态
    controller.reset()
    print("  ✅ 控制器就绪")

    # ==========================================================
    # 4. 运行主循环
    # ==========================================================
    print(f"\n[4/5] 开始仿真 ({args.steps} 步)...")

    # 记录数据
    right_errors = []
    left_errors = []
    right_targets = []
    left_targets = []
    right_actuals = []
    left_actuals = []
    joint_positions_log = []
    timestamps = []

    render_skip = max(1, args.steps // 3000) if args.viewer else 0  # viewer 渲染间隔

    real_start = time_module.time()

    for step in range(args.steps):
        # ---- A. 从 XR 源获取 tele_data ----
        tele_data = xr_source.get_tele_data()

        # ---- B. 桥接层：提取手腕目标位置 ----
        target_right, target_left = bridge.get_bimanual_targets(tele_data)

        # ---- C. 控制器：计算关节目标 ----
        error_info = controller.update(target_right, target_left)
        joint_targets = controller.get_joint_targets()

        # ---- D. 设置关节目标并仿真 ----
        env.set_joint_targets(joint_targets)
        env.step()

        # ---- E. 记录数据 ----
        obs = env.get_observation()
        right_errors.append(error_info["right_error"])
        left_errors.append(error_info["left_error"])
        right_targets.append(target_right)
        left_targets.append(target_left)
        right_actuals.append(obs["r_ee_pos"].copy())
        left_actuals.append(obs["l_ee_pos"].copy())
        joint_positions_log.append(obs["joint_positions"].copy())
        timestamps.append(obs["timestamp"])

        # ---- F. 更新 MuJoCo viewer 中的目标点位置 ----
        if args.viewer:
            # 直接更新场景中的目标点位置，使可视化与控制目标一致
            env.set_target_position("right", target_right)
            env.set_target_position("left", target_left)

        # ---- G. 渲染 viewer（降低频率以避免卡顿） ----
        if args.viewer and step % render_skip == 0 and step > 0:
            viewer.sync()

        # 进度打印
        if step > 0 and step % 500 == 0:
            print(f"  步 {step}/{args.steps} | "
                  f"右误差={np.mean(right_errors[-500:]):.4f}m | "
                  f"左误差={np.mean(left_errors[-500:]):.4f}m")

    real_elapsed = time_module.time() - real_start
    sim_elapsed = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0

    # ==========================================================
    # 5. 保存结果和分析
    # ==========================================================
    print(f"\n[5/5] 保存结果和分析...")

    output_dir = os.path.join(project_root, "data", "samples", "xr_teleop_demo")
    os.makedirs(output_dir, exist_ok=True)

    # 保存 episode 数据
    data_path = os.path.join(output_dir, "episode_data.npz")
    np.savez(data_path,
        mode=args.mode,
        timestamps=np.array(timestamps),
        joint_positions=np.array(joint_positions_log),
        right_errors=np.array(right_errors),
        left_errors=np.array(left_errors),
        right_targets=np.array(right_targets),
        left_targets=np.array(left_targets),
        right_actuals=np.array(right_actuals),
        left_actuals=np.array(left_actuals),
    )
    print(f"  [Saved] Episode data -> {data_path}")

    # 绘制误差曲线
    error_plot_path = os.path.join(output_dir, "tracking_error.png")
    plot_tracking_error(np.array(right_errors), np.array(left_errors), error_plot_path)

    # 计算统计
    final_right_error = right_errors[-1] if right_errors else 0
    final_left_error = left_errors[-1] if left_errors else 0
    avg_right_error = np.mean(right_errors)
    avg_left_error = np.mean(left_errors)

    # 保存摘要
    summary_path = os.path.join(output_dir, "summary.txt")
    with open(summary_path, "w") as f:
        f.write("XR → MuJoCo 完整链路演示 - 运行摘要\n")
        f.write("=" * 50 + "\n")
        f.write(f"XR 模式: {args.mode}\n")
        f.write(f"总仿真步数: {args.steps}\n")
        f.write(f"仿真时间: {sim_elapsed:.2f} s\n")
        f.write(f"实际用时: {real_elapsed:.2f} s\n")
        f.write(f"控制器增益 kp: {args.kp}\n\n")
        f.write("--- 跟踪误差统计 (米) ---\n")
        f.write(f"右臂 最终误差: {final_right_error:.4f}\n")
        f.write(f"右臂 平均误差: {avg_right_error:.4f}\n")
        f.write(f"左臂 最终误差: {final_left_error:.4f}\n")
        f.write(f"左臂 平均误差: {avg_left_error:.4f}\n")

    print(f"  [Saved] Summary -> {summary_path}")

    # 打印结果
    print("\n" + "-" * 50)
    print("运行结果汇总:")
    print(f"  右臂平均跟踪误差: {avg_right_error:.4f} m")
    print(f"  左臂平均跟踪误差: {avg_left_error:.4f} m")
    print(f"  实际用时: {real_elapsed:.2f}s, 仿真时间: {sim_elapsed:.2f}s")
    print(f"  输出目录: {output_dir}")

    # 清理
    if viewer is not None:
        viewer.close()
    env.close()

    print("\n" + "=" * 70)
    print("✅ 完整链路演示完成！")
    print("=" * 70)


if __name__ == "__main__":
    main()
