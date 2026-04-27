#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
capture_rgbd_demo.py —— RGBD 相机采集演示脚本

本脚本演示如何：
1. 加载 Mujoco 场景
2. 使用 RGBDCamera 接口采集一帧 RGB + Depth 图像
3. 将 RGB 和 Depth 保存到本地文件
4. 打印相机内参和外参

运行方式：
    python scripts/capture_rgbd_demo.py

输出：
    data/samples/camera_captures/
    ├── camera_front_rgb.png
    ├── camera_front_depth.npy
    ├── camera_front_depth_vis.png   (深度图可视化，便于肉眼查看)
    ├── camera_side_rgb.png
    ├── camera_side_depth.npy
    └── camera_side_depth_vis.png
"""
import os
import argparse
import sys

# 将项目根目录加入 Python 路径，确保可以 import envs 和 controllers
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import numpy as np
import imageio.v3 as iio
import matplotlib.pyplot as plt

from envs.upper_body_env import UpperBodyEnv
from envs.rgbd_camera import RGBDCamera


def visualize_depth(depth: np.ndarray, save_path: str):
    """
    将深度图可视化为灰度图并保存

    参数:
        depth:    ndarray (H, W), float32, 深度值（米）
        save_path: 保存路径
    """
    # 将深度值归一化到 [0, 1] 范围便于显示
    valid = depth < 49.0  # 剔除远平面
    depth_vis = np.zeros_like(depth)
    if valid.any():
        d_min = depth[valid].min()
        d_max = depth[valid].max()
        depth_vis[valid] = (depth[valid] - d_min) / (d_max - d_min + 1e-6)
    # 保存为伪彩色图
    plt.imsave(save_path, depth_vis, cmap="viridis")
    print(f"  [保存] 深度可视化图 -> {save_path}")


def main():
    parser = argparse.ArgumentParser(
        description="RGBD 相机采集演示 —— 从场景中采集 RGB + Depth 并保存"
    )
    parser.add_argument(
        "--scene", default=None,
        help="场景 XML 文件路径（可选，默认使用 simple_end_effector_scene.xml）"
    )
    parser.add_argument(
        "--output", default=None,
        help="输出目录（可选，默认使用 data/samples/camera_captures）"
    )
    parser.add_argument(
        "--width", type=int, default=640,
        help="渲染宽度（像素），默认 640"
    )
    parser.add_argument(
        "--height", type=int, default=480,
        help="渲染高度（像素），默认 480"
    )
    parser.add_argument(
        "--steps", type=int, default=100,
        help="仿真步数，默认 100 步后采集"
    )
    args = parser.parse_args()

    # ==========================================================
    # 1. 初始化场景
    # ==========================================================
    print("=" * 60)
    print("RGBD 相机采集演示")
    print("=" * 60)
    print("\n[1/4] 初始化场景...")
    env = UpperBodyEnv(scene_path=args.scene,
                       render_width=args.width,
                       render_height=args.height)
    env.reset()
    print(f"  场景加载完成")
    print(f"  可用相机: {env.camera_names}")

    # ==========================================================
    # 2. 初始化 RGBD 相机（前置 + 侧上方）
    # ==========================================================
    print("\n[2/4] 初始化 RGBD 相机...")
    cameras = {}
    for cam_name in env.camera_names:
        cam = RGBDCamera(
            env.model, env.data, cam_name,
            width=args.width, height=args.height
        )
        cameras[cam_name] = cam
        print(f"  相机 '{cam_name}' 初始化完成")

    # ==========================================================
    # 3. 运行仿真若干步，让场景稳定
    # ==========================================================
    print(f"\n[3/4] 运行仿真 {args.steps} 步...")
    for i in range(args.steps):
        env.step()
    print(f"  仿真时间: {env.data.time:.3f} 秒")

    # ==========================================================
    # 4. 采集并保存 RGBD 数据
    # ==========================================================
    print("\n[4/4] 采集 RGBD 数据...")

    # 确定输出目录
    if args.output is None:
        output_dir = os.path.join(project_root, "data", "samples", "camera_captures")
    else:
        output_dir = args.output
    os.makedirs(output_dir, exist_ok=True)

    for cam_name, cam in cameras.items():
        print(f"\n  --- 相机: {cam_name} ---")

        # 采集一帧 RGBD 数据
        result = cam.capture()

        # 获取图像
        rgb = result["rgb"]
        depth = result["depth"]
        intrinsics = result["intrinsics"]
        extrinsics = result["extrinsics"]
        timestamp = result["timestamp"]

        print(f"  RGB 图像尺寸: {rgb.shape}, dtype={rgb.dtype}, "
              f"范围 [{rgb.min()}, {rgb.max()}]")
        print(f"  Depth 图像尺寸: {depth.shape}, dtype={depth.dtype}, "
              f"范围 [{depth.min():.3f}, {depth.max():.3f}] 米")
        print(f"  内参矩阵 K:\n{intrinsics}")
        print(f"  外参矩阵 [R|t]:\n{extrinsics[:3]}")
        print(f"  时间戳: {timestamp:.4f} 秒")

        # ---- 保存 RGB 图像 ----
        rgb_path = os.path.join(output_dir, f"{cam_name}_rgb.png")
        iio.imwrite(rgb_path, rgb)
        print(f"  [保存] RGB -> {rgb_path}")

        # ---- 保存 Depth 原始数据（npy 格式） ----
        depth_path = os.path.join(output_dir, f"{cam_name}_depth.npy")
        np.save(depth_path, depth)
        print(f"  [保存] Depth 原始数据 -> {depth_path}")

        # ---- 保存 Depth 可视化图 ----
        depth_vis_path = os.path.join(output_dir, f"{cam_name}_depth_vis.png")
        visualize_depth(depth, depth_vis_path)

    # 保存相机参数到文本文件
    print("\n  --- 保存相机参数 ---")
    params_path = os.path.join(output_dir, "camera_params.txt")
    with open(params_path, "w") as f:
        for cam_name, cam in cameras.items():
            result = cam.capture()
            f.write(f"相机: {cam_name}\n")
            f.write(f"  内参 K:\n{result['intrinsics']}\n")
            f.write(f"  外参 [R|t]:\n{result['extrinsics'][:3]}\n")
            f.write(f"  图像尺寸: {result['rgb'].shape[1]}x{result['rgb'].shape[0]}\n\n")
    print(f"  [保存] 相机参数 -> {params_path}")

    # ==========================================================
    # 5. 清理资源
    # ==========================================================
    for cam in cameras.values():
        cam.close()
    env.close()

    print("\n" + "=" * 60)
    print(f"✅ 采集完成！所有文件已保存到: {output_dir}")
    print("=" * 60)


if __name__ == "__main__":
    main()
