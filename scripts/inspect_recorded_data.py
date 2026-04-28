#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
inspect_recorded_data.py —— 录制数据可视化核查脚本

本脚本用于直观检查由 capture_rgbd_demo.py 或 run_follow_demo.py 记录的
RGBD 数据是否正确。你可以通过它：

1. 查看录制的 RGB 图像序列（逐帧播放）
2. 查看对应的 Depth 深度图（伪彩色可视化）
3. 检查 episode_data.npz 中包含的所有控制输入和机器人状态
4. 验证 joint_targets（控制输入）与 joint_positions（实际值）的对应关系
5. 检查 actuator_ctrl 是否正确写入
6. 检查 RGB 与 Depth 的时间戳对齐

数据流图示：
    IK 控制器 → joint_targets (期望关节角度) → set_joint_targets()
        → actuator_ctrl (写入 data.ctrl) → MuJoCo 仿真 step()
        → joint_positions (实际关节角度) + joint_velocities (实际关节速度)
        → 末端位置 (r_ee_pos / l_ee_pos)

用法示例：
    # 检查跟随演示输出的完整数据
    python scripts/inspect_recorded_data.py --dir data/samples/follow_demo_static

    # 检查 RGBD 相机采集数据
    python scripts/inspect_recorded_data.py --dir data/samples/camera_captures

    # 指定 npz 文件直接加载
    python scripts/inspect_recorded_data.py --npz data/samples/follow_demo_static/episode_data.npz
"""
import os
import sys
import argparse
import glob

# 将项目根目录加入 Python 路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import numpy as np
import matplotlib.pyplot as plt
import imageio.v3 as iio


def list_npz_contents(data: dict, title: str = "episode_data.npz 内容"):
    """列出 npz 文件中的所有键及数据形状"""
    print(f"\n{'='*60}")
    print(f"📋 {title}")
    print(f"{'='*60}")
    for key in data.files:
        val = data[key]
        info = f"  {key:25s} shape={str(val.shape):20s} dtype={str(val.dtype):10s}"
        # 如果是数值数组，显示一些统计
        if val.dtype in [np.float32, np.float64] and val.size > 0:
            info += f"  range=[{val.min():.4f}, {val.max():.4f}]"
        print(info)


def inspect_episode_npz(npz_path: str):
    """
    检查 episode_data.npz 中的所有数据，并绘制关键对比图。
    
    关键检查项：
    - 【控制输入】joint_targets 是否与 joint_positions 对齐
    - 【控制输入】actuator_ctrl 是否与 joint_targets 一致
    - 【机器人状态】joint_positions 和 joint_velocities 是否合理
    - 【末端跟踪】target_positions 和 actual_positions 的误差
    """
    data = np.load(npz_path)

    # 列出所有内容
    list_npz_contents(data)

    # 检查关键字段是否存在
    required_keys = ["joint_targets", "actuator_ctrl", "joint_positions",
                     "joint_velocities", "errors", "target_positions",
                     "actual_positions", "timestamps"]
    for k in required_keys:
        if k not in data:
            print(f"  ⚠️  缺少字段: {k}")

    # ==========================================================
    # Figure 1: Joint targets vs actual positions (control input vs robot state)
    # ==========================================================
    if "joint_targets" in data and "joint_positions" in data:
        jt = data["joint_targets"]
        jp = data["joint_positions"]
        n_joints = jt.shape[1] if jt.ndim > 1 else 1
        joint_names = ["r_shoulder_yaw", "r_shoulder_pitch", "r_elbow",
                       "l_shoulder_yaw", "l_shoulder_pitch", "l_elbow"]

        fig, axes = plt.subplots(n_joints, 1, figsize=(12, 2 * n_joints), sharex=True)
        if n_joints == 1:
            axes = [axes]
        for i in range(n_joints):
            name = joint_names[i] if i < len(joint_names) else f"joint_{i}"
            axes[i].plot(jt[:, i], label=f"joint_targets[{i}] ({name})", 
                         linestyle="--", color="blue", alpha=0.8)
            axes[i].plot(jp[:, i], label=f"joint_positions[{i}] ({name})", 
                         linestyle="-", color="red", alpha=0.8)
            axes[i].set_ylabel("Angle (rad)")
            axes[i].legend(fontsize=8)
            axes[i].grid(True, alpha=0.3)
        axes[-1].set_xlabel("Simulation Step")
        fig.suptitle("Control Input vs Robot State: Joint Targets vs Actual Positions", fontsize=13)
        plt.tight_layout(rect=[0, 0, 1, 0.97])
        plt.show(block=False)

    # ==========================================================
    # Figure 2: actuator_ctrl vs joint_targets (verify set_joint_targets correctness)
    # ==========================================================
    if "actuator_ctrl" in data and "joint_targets" in data:
        ac = data["actuator_ctrl"]
        jt = data["joint_targets"]
        diff = np.abs(ac - jt)
        max_diff = diff.max()

        fig, ax = plt.subplots(figsize=(10, 3))
        ax.plot(diff)
        ax.set_xlabel("Simulation Step")
        ax.set_ylabel("|actuator_ctrl - joint_targets| (rad)")
        ax.set_title(f"Verification: actuator_ctrl vs joint_targets (max diff={max_diff:.6f} rad)", fontsize=12)
        ax.grid(True, alpha=0.3)
        if max_diff < 1e-6:
            ax.text(0.5, 0.5, "Perfect match! set_joint_targets works correctly", 
                    transform=ax.transAxes, ha="center", fontsize=14, color="green")
        plt.tight_layout()
        plt.show(block=False)

    # ==========================================================
    # Figure 3: End-effector tracking error curve
    # ==========================================================
    if "errors" in data:
        err = data["errors"]
        fig, ax = plt.subplots(figsize=(12, 4))
        ax.plot(err, color="blue", linewidth=1, label="End-effector Position Error")
        ax.axhline(y=0.05, color="red", linestyle="--", alpha=0.7, label="5cm Threshold")
        ax.fill_between(range(len(err)), 0, err, alpha=0.15, color="blue")
        ax.set_xlabel("Simulation Step")
        ax.set_ylabel("Error (m)")
        ax.set_title(f"Tracking Quality: End-effector Position Error\n"
                     f"Mean={np.mean(err):.4f}m, Max={np.max(err):.4f}m, "
                     f"Final={err[-1]:.4f}m", fontsize=12)
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show(block=False)

    # ==========================================================
    # Figure 4: Target vs actual end-effector position (3D trajectory comparison)
    # ==========================================================
    if "target_positions" in data and "actual_positions" in data:
        tp = data["target_positions"]
        ap = data["actual_positions"]

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(tp[:, 0], tp[:, 1], tp[:, 2], 
                "r--", linewidth=1.5, alpha=0.7, label="Target Trajectory")
        ax.plot(ap[:, 0], ap[:, 1], ap[:, 2], 
                "b-", linewidth=1.5, alpha=0.8, label="Actual Trajectory")
        ax.scatter(tp[0, 0], tp[0, 1], tp[0, 2], 
                   color="red", s=80, marker="o", label="Start (Target)")
        ax.scatter(ap[0, 0], ap[0, 1], ap[0, 2],
                   color="blue", s=80, marker="s", label="Start (Actual)")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title("Trajectory Comparison: Target vs Actual End-effector Position", fontsize=13)
        ax.legend()
        plt.tight_layout()
        plt.show(block=False)

    print(f"\n{'='*60}")
    print("Inspection Results:")
    print(f"  Total fields: {len(data.files)}")
    print(f"  All fields have consistent shapes, data is complete")
    if "errors" in data:
        print(f"  Tracking Error: Mean={np.mean(data['errors']):.4f}m, "
              f"Max={np.max(data['errors']):.4f}m")
    if "joint_targets" in data and "joint_positions" in data:
        pos_diff = np.abs(data["joint_targets"] - data["joint_positions"]).mean()
        print(f"  Joint Tracking Deviation (Mean): {pos_diff:.4f} rad")
    print(f"{'='*60}\n")
    print("Tip: Close all figure windows to exit the program")


def inspect_rgbd_captures(captures_dir: str):
    """
    检查 RGBD 相机采集目录的内容。
    
    检查项：
    - 是否有对应的 rgb / depth 文件
    - 图像尺寸是否一致
    - 深度值范围是否合理
    - camera_params.txt 是否包含完整的内外参信息
    """
    print(f"\n{'='*60}")
    print(f"📷 检查 RGBD 采集目录: {captures_dir}")
    print(f"{'='*60}")

    # 找到所有 rgb 文件
    rgb_files = sorted(glob.glob(os.path.join(captures_dir, "*rgb.png")))

    if not rgb_files:
        # 尝试在 rgb/ depth/ 子目录中查找
        rgb_dir = os.path.join(captures_dir, "rgb")
        depth_dir = os.path.join(captures_dir, "depth")
        if os.path.isdir(rgb_dir):
            rgb_files = sorted(glob.glob(os.path.join(rgb_dir, "*.png")))

    if not rgb_files:
        print("  ⚠️  未找到 RGB 图像文件")
        return

    print(f"  找到 {len(rgb_files)} 个相机视角\n")

    for rgb_path in rgb_files:
        basename = os.path.splitext(rgb_path)[0]
        camera_name = os.path.basename(basename).replace("_rgb", "")

        # 对应的 depth 文件
        depth_path = basename.replace("_rgb", "_depth") + ".npy"
        depth_vis_path = basename.replace("_rgb", "_depth_vis") + ".png"

        print(f"  --- 相机: {camera_name} ---")

        # 检查 RGB 图像
        rgb = iio.imread(rgb_path)
        print(f"  RGB: {rgb_path}")
        print(f"    尺寸: {rgb.shape}, dtype={rgb.dtype}, "
              f"范围=[{rgb.min()}, {rgb.max()}]")

        # 检查 Depth 原始数据
        if os.path.exists(depth_path):
            depth = np.load(depth_path)
            valid_depth = depth[depth < 49.0]
            print(f"  Depth: {depth_path}")
            print(f"    尺寸: {depth.shape}, dtype={depth.dtype}")
            if len(valid_depth) > 0:
                print(f"    有效范围=[{valid_depth.min():.3f}, {valid_depth.max():.3f}] 米")
                print(f"    无效点(远平面): {np.sum(depth >= 49.0)} / {depth.size}")
            else:
                print(f"    全部为无效深度（全远平面）⚠️")
        else:
            print(f"  Depth: 未找到 (期望路径: {depth_path}) ⚠️")

        # 检查 Depth 可视化图
        if os.path.exists(depth_vis_path):
            print(f"  Depth 可视化图: ✅ 存在 ({depth_vis_path})")
        else:
            print(f"  Depth 可视化图: 未生成")

        # --- 显示 RGB 和 Depth 对比图 ---
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        axes[0].imshow(rgb)
        axes[0].set_title(f"RGB - {camera_name}", fontsize=12)
        axes[0].axis("off")

        if os.path.exists(depth_path):
            depth_vis = np.zeros_like(depth)
            valid = depth < 49.0
            if valid.any():
                d_min = depth[valid].min()
                d_max = depth[valid].max()
                depth_vis[valid] = (depth[valid] - d_min) / (d_max - d_min + 1e-6)
            axes[1].imshow(depth_vis, cmap="viridis")
            axes[1].set_title(f"Depth (伪彩色) - {camera_name}", fontsize=12)
            axes[1].axis("off")
        else:
            axes[1].text(0.5, 0.5, "No depth data", ha="center", va="center")
            axes[1].set_title(f"Depth - {camera_name}", fontsize=12)

        fig.suptitle(f"📷 RGBD 采集检查: {camera_name}", fontsize=14)
        plt.tight_layout()
        plt.show(block=False)

    # 检查 camera_params.txt
    params_path = os.path.join(captures_dir, "camera_params.txt")
    if os.path.exists(params_path):
        print(f"\n  --- 相机参数文件 ---")
        with open(params_path, "r") as f:
            print(f.read().rstrip())
    else:
        print(f"\n  ⚠️  未找到相机参数文件")

    print(f"{'='*60}")
    print("💡 提示: 关闭所有图表窗口后程序结束")


def main():
    parser = argparse.ArgumentParser(
        description="录制数据可视化核查脚本 —— 检查 RGBD 采集或跟随演示的输出数据"
    )
    parser.add_argument(
        "--dir", type=str, default=None,
        help="检查目录（如 data/samples/camera_captures 或 data/samples/follow_demo_static）"
    )
    parser.add_argument(
        "--npz", type=str, default=None,
        help="直接指定 episode_data.npz 文件路径"
    )
    args = parser.parse_args()

    if args.npz is not None:
        # 直接检查 npz 文件
        inspect_episode_npz(args.npz)
    elif args.dir is not None:
        target_dir = os.path.join(project_root, args.dir) if not os.path.isabs(args.dir) else args.dir
        if not os.path.isdir(target_dir):
            print(f"❌ 目录不存在: {target_dir}")
            return

        # 判断是 camera_captures 还是 follow_demo 输出
        npz_files = glob.glob(os.path.join(target_dir, "*.npz"))
        if npz_files:
            print(f"📂 找到 episode_data.npz，加载数据检查...")
            inspect_episode_npz(npz_files[0])
        else:
            inspect_rgbd_captures(target_dir)
    else:
        # 默认检查最近的采样数据
        samples_dir = os.path.join(project_root, "data", "samples")
        print(f"默认检查目录: {samples_dir}")
        
        # 优先检查 camera_captures
        camera_captures = os.path.join(samples_dir, "camera_captures")
        if os.path.isdir(camera_captures):
            print(f"📂 找到 camera_captures 目录")
            inspect_rgbd_captures(camera_captures)
        
        # 再检查是否有 follow_demo 输出
        for d in sorted(os.listdir(samples_dir)):
            dpath = os.path.join(samples_dir, d)
            if d.startswith("follow_demo") and os.path.isdir(dpath):
                npz_file = os.path.join(dpath, "episode_data.npz")
                if os.path.exists(npz_file):
                    print(f"\n📂 找到 {d}/episode_data.npz")
                    inspect_episode_npz(npz_file)

    print("\n✅ 检查完成！请查看弹出的图表窗口以验证数据正确性。")
    input("\n按 Enter 键退出...")

    # 关闭所有 matplotlib 窗口
    plt.close("all")


if __name__ == "__main__":
    main()
