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
    绘制跟踪误差曲线图

    参数:
        errors: shape (N,) 的误差序列
        save_path: 保存路径
    """
    plt.figure(figsize=(10, 4))
    plt.plot(errors, label="跟踪误差 (m)", color="blue", linewidth=1)
    plt.xlabel("仿真步数")
    plt.ylabel("末端位置误差 (米)")
    plt.title("末端位置跟踪误差")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close()
    print(f"  [保存] 误差曲线图 -> {save_path}")


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
        target_gen = StaticTargetGenerator(pos=[0.5, 0.2, 0.4])
        controller = EndEffectorController(env, side=args.side, kp=3.0)
        print(f"  目标生成器: {target_gen}")
        print(f"  控制器: 单臂({args.side})")

    elif args.mode == "discrete":
        # 版本 B：离散多点跟随
        waypoints = [
            [0.3, 0.2, 0.3],
            [0.5, 0.2, 0.4],
            [0.4, -0.2, 0.5],
            [0.6, 0.1, 0.3],
        ]
        target_gen = DiscreteTargetGenerator(waypoints, dwell_time=3.0)
        controller = EndEffectorController(env, side=args.side, kp=3.0)
        print(f"  目标生成器: {target_gen}")
        print(f"  控制器: 单臂({args.side})")

    elif args.mode == "circle":
        # 版本 C：圆形轨迹
        target_gen = CircleTargetGenerator(
            center=[0.4, 0.0, 0.4], radius=0.18, axis="xz", speed=0.8
        )
        controller = EndEffectorController(env, side=args.side, kp=4.0)
        print(f"  目标生成器: {target_gen}")
        print(f"  控制器: 单臂({args.side})")

    elif args.mode == "line":
        # 版本 C：直线轨迹
        target_gen = LineTargetGenerator(
            start=[0.2, 0.0, 0.3], end=[0.6, 0.0, 0.5], speed=0.3
        )
        controller = EndEffectorController(env, side=args.side, kp=4.0)
        print(f"  目标生成器: {target_gen}")
        print(f"  控制器: 单臂({args.side})")

    elif args.mode == "bimanual":
        # 双臂跟随：右手画圆，左手画直线
        target_gen_right = CircleTargetGenerator(
            center=[0.4, 0.2, 0.4], radius=0.15, axis="xz", speed=0.6
        )
        target_gen_left = LineTargetGenerator(
            start=[-0.4, -0.1, 0.3], end=[-0.6, -0.1, 0.5], speed=0.2
        )
        controller = BimanualController(env, kp=3.0)
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

    # 记录数据
    errors = []
    target_positions = []
    actual_positions = []
    joint_positions_log = []
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
        joint_positions_log.append(obs["joint_positions"])
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

    # 保存 episode 数据
    data_path = os.path.join(output_dir, "episode_data.npz")
    np.savez(
        data_path,
        errors=errors_arr,
        target_positions=target_arr,
        actual_positions=actual_arr,
        joint_positions=joint_arr,
        timestamps=np.array(timestamps),
        mode=args.mode,
    )
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
        f.write("末端位置跟随演示 —— 运行摘要\n")
        f.write("=" * 50 + "\n")
        f.write(f"模式: {args.mode}\n")
        f.write(f"总步数: {args.steps}\n")
        f.write(f"仿真时间: {sim_elapsed:.2f} 秒\n")
        f.write(f"实际运行时间: {real_elapsed:.2f} 秒\n")
        f.write(f"保存 RGBD 帧数: {frame_count}\n\n")
        f.write("--- 跟踪误差统计 ---\n")
        f.write(f"最终误差: {final_error:.4f} 米\n")
        f.write(f"平均误差: {avg_error:.4f} 米\n")
        f.write(f"最大误差: {max_error:.4f} 米\n")
        if converge_steps >= 0:
            f.write(f"首次进入 5cm 误差范围: 第 {converge_steps} 步\n")
        else:
            f.write("未进入 5cm 误差范围\n")
    print(f"  [保存] 运行摘要 -> {summary_path}")

    # 打印摘要
    print("\n" + "-" * 50)
    print("📊 运行结果摘要:")
    print(f"  最终误差: {final_error:.4f} 米")
    print(f"  平均误差: {avg_error:.4f} 米")
    print(f"  最大误差: {max_error:.4f} 米")
    if converge_steps >= 0:
        print(f"  首次进入 5cm 误差范围: 第 {converge_steps} 步")
    else:
        print(f"  ⚠️ 未进入 5cm 误差范围")
    print(f"  输出目录: {output_dir}")

    # 清理资源
    cam_front.close()
    env.close()

    print("\n" + "=" * 60)
    print("✅ 演示完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()
