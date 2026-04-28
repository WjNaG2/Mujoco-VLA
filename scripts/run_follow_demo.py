#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_follow_demo.py —— 末端位置跟随完整演示脚本

本脚本整合了场景、控制器、目标点生成器、RGBD 相机采集，演示完整的"给定末端
目标位置 → 机器人稳定跟随 → 同步录制 RGBD"流程。

支持三种跟随模式：
- static:   静态单点跟随（版本 A）
- discrete: 离散多点跟随（版本 B）
- circle:   圆形轨迹跟随（版本 C）
- line:     直线轨迹跟随（版本 C）
- bimanual: 双臂同时跟随

运行方式：
    # 单臂静态点跟随
    python scripts/run_follow_demo.py --mode static

    # 圆形轨迹跟随
    python scripts/run_follow_demo.py --mode circle

    # 双臂跟随
    python scripts/run_follow_demo.py --mode bimanual

输出：
    data/samples/follow_demo/
    ├── rgb/          # 每帧 RGB 图像
    ├── depth/        # 每帧 Depth 数据
    ├── episode_data.npz   # 状态/动作/误差序列
    ├── tracking_error.png  # 跟踪误差曲线图
    └── summary.txt         # 运行摘要
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
import matplotlib.pyplot as plt
import imageio.v3 as iio

from envs.upper_body_env import UpperBodyEnv
from envs.rgbd_camera import RGBDCamera
from controllers.end_effector_controller import EndEffectorController, BimanualController
from controllers.target_generator import (
    StaticTargetGenerator, DiscreteTargetGenerator,
    CircleTargetGenerator, LineTargetGenerator,
)


def tracking_error_to_img(errors: np.ndarray, save_path: str):
    """
    Plot tracking error curve

    Args:
        errors: shape (N,) error sequence
        save_path: save path
    """
    plt.figure(figsize=(10, 4))
    plt.plot(errors, label="Tracking Error (m)", color="blue", linewidth=1)
    plt.xlabel("Simulation Step")
    plt.ylabel("End-effector Position Error (m)")
    plt.title("End-effector Tracking Error")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close()
    print(f"  [Saved] Error plot -> {save_path}")


def main():
    parser = argparse.ArgumentParser(
        description="末端位置跟随演示 —— 整合场景、控制器、RGBD 采集"
    )
    parser.add_argument(
        "--mode", type=str, default="static",
        choices=["static", "discrete", "circle", "line", "bimanual"],
        help="跟随模式（default: static）"
    )
    parser.add_argument(
        "--side", type=str, default="right",
        choices=["right", "left"],
        help="目标臂（单臂模式下使用，default: right）"
    )
    parser.add_argument(
        "--steps", type=int, default=2000,
        help="仿真总步数（default: 2000）"
    )
    parser.add_argument(
        "--output", type=str, default=None,
        help="输出目录（default: data/samples/follow_demo_<mode>）"
    )
    parser.add_argument(
        "--save-every", type=int, default=10,
        help="每隔多少步保存一次 RGBD 图像（default: 10）"
    )
    parser.add_argument(
        "--kp", type=float, default=3.0,
        help="控制器比例增益（default: 3.0，越大响应越快但可能振荡）"
    )
    args = parser.parse_args()

    # ==========================================================
    # 1. 初始化场景
    # ==========================================================
    print("=" * 60)
    print(f"末端位置跟随演示 —— 模式: {args.mode}")
    print("=" * 60)

    print("\n[1/5] 初始化场景...")
    env = UpperBodyEnv()
    env.reset()
    print(f"  场景加载完成，可用相机: {env.camera_names}")

    # ==========================================================
    # 2. 初始化 RGBD 相机（仅前置相机，用于录制）
    # ==========================================================
    print("\n[2/5] 初始化 RGBD 相机...")
    cam_front = RGBDCamera(env.model, env.data, "camera_front")
    print(f"  前置相机初始化完成")

    # ==========================================================
    # 3. 初始化和配置目标生成器和控制器
    # ==========================================================
    print("\n[3/5] 配置目标点和控制器...")

    if args.mode == "static":
        # 版本 A：静态单点跟随
        # 【可达性】根据选择的臂设置不同的目标点（确保在工作空间内）
        if args.side == "right":
            # 右肩位置 [0.2, 0.0, 0.35]
            # 目标 [0.5, 0.1, 0.4] 需 yaw 约 18° ✅ 可达（0.0000m 误差）
            # 注意 y=0.2 会导致 yaw 达到 90° 限位，无法收敛
            static_pos = [0.5, 0.1, 0.4]
        else:
            # 【可达性】左臂初始 EE 在 [0.238, 0.0, 0.282]，需向左摆动到负 x 区域
            # 左肩位置 [-0.2, 0.0, 0.35]。目标 [-0.15, -0.15, 0.35] 需 yaw 约 -72° ✅ 
            static_pos = [-0.15, -0.15, 0.35]
        target_gen = StaticTargetGenerator(pos=static_pos)
        controller = EndEffectorController(env, side=args.side, kp=args.kp)
        print(f"  目标生成器: {target_gen}")
        print(f"  控制器: 单臂({args.side})")

    elif args.mode == "discrete":
        # 版本 B：离散多点跟随
        # 【可达性】根据选择的臂设置不同的路点
        if args.side == "right":
            waypoints = [
                [0.3, 0.2, 0.3],    # 距右肩约 0.26m ✅
                [0.5, 0.2, 0.4],    # 距右肩约 0.30m ✅
                [0.4, -0.2, 0.5],   # 距右肩约 0.27m ✅
                [0.6, 0.1, 0.3],    # 距右肩约 0.40m ✅
            ]
        else:
            waypoints = [
                [-0.25, -0.15, 0.3],  # 距左肩约 0.18m ✅
                [-0.35, -0.1, 0.35],  # 距左肩约 0.17m ✅
                [-0.3,  -0.2, 0.4],   # 距左肩约 0.15m ✅
                [-0.5,  -0.1, 0.35],  # 距左肩约 0.30m ✅
            ]
        target_gen = DiscreteTargetGenerator(waypoints, dwell_time=3.0)
        controller = EndEffectorController(env, side=args.side, kp=args.kp)
        print(f"  目标生成器: {target_gen}")
        print(f"  控制器: 单臂({args.side})")

    elif args.mode == "circle":
        # 版本 C：圆形轨迹
        # 【可达性】根据选择的臂设置不同的圆心位置
        if args.side == "right":
            # 圆心距右肩 ~0.32m，半径 0.18m ✅ 可达
            circle_center = [0.4, 0.0, 0.4]
            circle_radius = 0.18
        else:
            # 圆心距左肩 ~0.14m，半径 0.12m ✅ 可达
            circle_center = [-0.3, 0.0, 0.3]
            circle_radius = 0.12
        target_gen = CircleTargetGenerator(
            center=circle_center, radius=circle_radius, axis="xz", speed=0.8
        )
        controller = EndEffectorController(env, side=args.side, kp=args.kp)
        print(f"  目标生成器: {target_gen}")
        print(f"  控制器: 单臂({args.side})")

    elif args.mode == "line":
        # 版本 C：直线轨迹
        # 【可达性】根据选择的臂设置不同的起点/终点
        if args.side == "right":
            line_start = [0.2, 0.0, 0.3]
            line_end = [0.6, 0.0, 0.5]
        else:
            line_start = [-0.6, 0.0, 0.3]
            line_end = [-0.2, 0.0, 0.5]
        target_gen = LineTargetGenerator(
            start=line_start, end=line_end, speed=0.3
        )
        controller = EndEffectorController(env, side=args.side, kp=args.kp)
        print(f"  目标生成器: {target_gen}")
        print(f"  控制器: 单臂({args.side})")

    elif args.mode == "bimanual":
        # 双臂跟随：右手画圆，左手画直线
        # 【可达性】右手圆心 [0.4, 0.2, 0.4] 距右肩约 0.40m ✅
        #           左手直线 [-0.4~-0.6, -0.1, 0.3~0.5] 距左肩约 0.23~0.44m ✅
        target_gen_right = CircleTargetGenerator(
            center=[0.4, 0.2, 0.4], radius=0.15, axis="xz", speed=0.6
        )
        target_gen_left = LineTargetGenerator(
            start=[-0.4, -0.1, 0.3], end=[-0.6, -0.1, 0.5], speed=0.2
        )
        controller = BimanualController(env, kp=args.kp)
        print(f"  目标生成器: 右手画圆 + 左手画直线")
        print(f"  控制器: 双臂")

    # ==========================================================
    # 4. 运行仿真主循环
    # ==========================================================
    print(f"\n[4/5] 开始仿真 ({args.steps} 步)...")

    # 准备输出目录
    if args.output is None:
        output_dir = os.path.join(
            project_root, "data", "samples", f"follow_demo_{args.mode}"
        )
    else:
        output_dir = args.output
    rgb_dir = os.path.join(output_dir, "rgb")
    depth_dir = os.path.join(output_dir, "depth")
    os.makedirs(rgb_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)

    # === 记录数据 ===
    # 【控制输入】IK 控制器输出的关节目标位置（弧度）—— 发送给 actuator 的期望值
    joint_targets_log = []
    # 【控制输入】actuator 实际接收到的 ctrl 值（通过 set_joint_targets 写入）
    actuator_ctrl_log = []
    # 【机器人状态】当前关节角度（弧度）—— MuJoCo 仿真后的实际位置
    joint_positions_log = []
    # 【机器人状态】当前关节速度（弧度/秒）
    joint_velocities_log = []
    # 【末端误差】末端目标位置 vs 实际位置之间的欧氏距离
    errors = []
    # 【目标位置】给定的末端目标位置（世界坐标系）
    target_positions = []
    # 【实际位置】仿真后的末端实际位置（世界坐标系）
    actual_positions = []
    # 【双臂专用】左臂记录（bimanual 模式下使用）
    left_errors = []
    left_target_positions = []
    left_actual_positions = []
    timestamps = []
    frame_count = 0

    real_start_time = time_module.time()

    for step in range(args.steps):
        # ---- 获取当前目标点 ----
        t = env.data.time
        if args.mode == "bimanual":
            target_right = target_gen_right.get_target(t)
            target_left = target_gen_left.get_target(t)
            error_info = controller.update(target_right, target_left)
            joint_targets = controller.get_joint_targets()
            errors.append(error_info["right_error"])
            target_positions.append(target_right)
            actual_positions.append(env.get_observation()["r_ee_pos"])
            # 【双臂专用】记录左臂数据
            left_target_positions.append(target_left)
            left_actual_positions.append(env.get_observation()["l_ee_pos"])
            left_errors.append(error_info["left_error"])
        else:
            target = target_gen.get_target(t)
            error = controller.update(target)
            joint_targets = controller.joint_targets
            errors.append(error)
            target_positions.append(target)
            obs = env.get_observation()
            ee_key = f"{args.side[:1]}_ee_pos"
            actual_positions.append(obs[ee_key])

        # ---- 设置关节目标并仿真一步 ----
        env.set_joint_targets(joint_targets)
        env.step()

        # ---- 记录状态 ----
        obs = env.get_observation()

        # 【控制输入】记录 IK 控制器输出的关节目标位置（弧度）
        joint_targets_log.append(joint_targets.copy())
        # 【控制输入】记录 actuator 实际接收到的 ctrl 值（写入 data.ctrl 后的值）
        actuator_ctrl_log.append(env.data.ctrl.copy())
        # 【机器人状态】当前关节角度（弧度）—— MuJoCo 仿真后的实际位置
        joint_positions_log.append(obs["joint_positions"])
        # 【机器人状态】当前关节速度（弧度/秒）
        joint_velocities_log.append(obs["joint_velocities"])
        # 【时间戳】
        timestamps.append(obs["timestamp"])

        # ---- 每隔 save_every 步保存一帧 RGBD ----
        if step % args.save_every == 0:
            # 采集 RGBD
            cam_result = cam_front.capture()

            # 保存 RGB
            rgb_path = os.path.join(rgb_dir, f"frame_{frame_count:06d}.png")
            iio.imwrite(rgb_path, cam_result["rgb"])

            # 保存 Depth
            depth_path = os.path.join(depth_dir, f"frame_{frame_count:06d}.npy")
            np.save(depth_path, cam_result["depth"])

            frame_count += 1

        # 每 500 步打印一次进度
        if step > 0 and step % 500 == 0:
            avg_error = np.mean(errors[-500:])
            print(f"  步 {step}/{args.steps}, 时间={t:.2f}s, "
                  f"平均误差={avg_error:.4f}m")

    real_elapsed = time_module.time() - real_start_time
    sim_elapsed = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0

    # ==========================================================
    # 5. 保存结果和分析
    # ==========================================================
    print(f"\n[5/5] 保存结果和分析...")

    # 转为 numpy 数组
    errors_arr = np.array(errors)
    target_arr = np.array(target_positions)
    actual_arr = np.array(actual_positions)
    joint_arr = np.array(joint_positions_log)

    # 将新增记录数据转为 numpy 数组
    joint_targets_arr = np.array(joint_targets_log)
    actuator_ctrl_arr = np.array(actuator_ctrl_log)
    joint_velocities_arr = np.array(joint_velocities_log)

    # 保存 episode 数据（包含完整的控制输入 + 机器人状态）
    data_path = os.path.join(output_dir, "episode_data.npz")
    save_dict = {
        # 【控制输入】关节目标位置（弧度）—— IK 控制器输出，set_joint_targets 发送的值
        "joint_targets": joint_targets_arr,
        # 【控制输入】actuator 控制值（弧度）—— 实际写入 data.ctrl 的值
        "actuator_ctrl": actuator_ctrl_arr,
        # 【机器人状态】关节角度（弧度）—— MuJoCo 仿真后的实际位置
        "joint_positions": joint_arr,
        # 【机器人状态】关节速度（弧度/秒）
        "joint_velocities": joint_velocities_arr,
        # 【末端跟踪】末端位置误差（米）
        "errors": errors_arr,
        # 【末端跟踪】给定的末端目标位置（世界坐标系，米）
        "target_positions": target_arr,
        # 【末端跟踪】仿真后的末端实际位置（世界坐标系，米）
        "actual_positions": actual_arr,
        # 【时间戳】仿真时间（秒）
        "timestamps": np.array(timestamps),
        # 【元信息】运行模式
        "mode": args.mode,
    }
    # 【双臂专用】在 bimanual 模式下额外保存左臂数据
    if args.mode == "bimanual" and len(left_errors) > 0:
        save_dict["left_errors"] = np.array(left_errors)
        save_dict["left_target_positions"] = np.array(left_target_positions)
        save_dict["left_actual_positions"] = np.array(left_actual_positions)
    np.savez(data_path, **save_dict)
    print(f"  [保存] 完整 episode 数据 -> {data_path}")

    # 绘制误差曲线
    error_plot_path = os.path.join(output_dir, "tracking_error.png")
    tracking_error_to_img(errors_arr, error_plot_path)

    # 计算统计指标
    final_error = errors_arr[-1] if len(errors_arr) > 0 else 0
    avg_error = np.mean(errors_arr)
    max_error = np.max(errors_arr)
    converge_steps = np.argmax(errors_arr < 0.05) if any(errors_arr < 0.05) else -1

    # 保存摘要
    summary_path = os.path.join(output_dir, "summary.txt")
    with open(summary_path, "w") as f:
        f.write("End-effector Following Demo - Run Summary\n")
        f.write("=" * 50 + "\n")
        f.write(f"Mode: {args.mode}\n")
        f.write(f"Total Steps: {args.steps}\n")
        f.write(f"Simulation Time: {sim_elapsed:.2f} s\n")
        f.write(f"Wall Clock Time: {real_elapsed:.2f} s\n")
        f.write(f"Saved RGBD Frames: {frame_count}\n\n")
        f.write("--- Tracking Error Statistics ---\n")
        f.write(f"Final Error: {final_error:.4f} m\n")
        f.write(f"Average Error: {avg_error:.4f} m\n")
        f.write(f"Max Error: {max_error:.4f} m\n")
        if converge_steps >= 0:
            f.write(f"First entry into 5cm error range: step {converge_steps}\n")
        else:
            f.write("Did not enter 5cm error range\n")
    print(f"  [Saved] Run summary -> {summary_path}")

    # 打印摘要
    print("\n" + "-" * 50)
    print("Result Summary:")
    print(f"  Final Error: {final_error:.4f} m")
    print(f"  Average Error: {avg_error:.4f} m")
    print(f"  Max Error: {max_error:.4f} m")
    if converge_steps >= 0:
        print(f"  First entry into 5cm error range: step {converge_steps}")
    else:
        print(f"  Did not enter 5cm error range")
    print(f"  Output Directory: {output_dir}")

    # 清理资源
    cam_front.close()
    env.close()

    print("\n" + "=" * 60)
    print("✅ 演示完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()
