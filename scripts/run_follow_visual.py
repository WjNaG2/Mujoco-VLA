#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_follow_visual.py —— 带 MuJoCo 可视化窗口的末端跟随演示

本脚本在 MuJoCo 的交互式 viewer 中运行末端位置跟随演示，
用户可以实时看到机器人手臂跟随目标点运动的动画效果。

支持三种跟随模式：
- static:   静态单点跟随（版本 A）
- circle:   圆形轨迹跟随（版本 C）
- bimanual: 双臂同时跟随

运行方式：
    # 静态单点跟随（默认）
    python scripts/run_follow_visual.py

    # 圆形轨迹跟随
    python scripts/run_follow_visual.py --mode circle

    # 双臂跟随
    python scripts/run_follow_visual.py --mode bimanual

操作说明（MuJoCo viewer）：
    - 鼠标左键拖拽：旋转视角
    - 鼠标滚轮：缩放
    - 鼠标右键拖拽：平移视角
    - 空格键：暂停/继续仿真
    - ESC 键：退出
"""
import os
import sys
import argparse
import threading
import time as time_module

# 将项目根目录加入 Python 路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import numpy as np
import mujoco
from mujoco.viewer import launch_passive

from envs.upper_body_env import UpperBodyEnv
from controllers.end_effector_controller import EndEffectorController, BimanualController
from controllers.target_generator import (
    StaticTargetGenerator, CircleTargetGenerator, LineTargetGenerator,
)


def main():
    parser = argparse.ArgumentParser(
        description="带 MuJoCo 可视化窗口的末端跟随演示"
    )
    parser.add_argument(
        "--mode", type=str, default="static",
        choices=["static", "circle", "bimanual"],
        help="跟随模式（default: static）"
    )
    parser.add_argument(
        "--side", type=str, default="right",
        choices=["right", "left"],
        help="目标臂（单臂模式下使用，default: right）"
    )
    parser.add_argument(
        "--steps", type=int, default=3000,
        help="仿真总步数（default: 3000）"
    )
    parser.add_argument(
        "--kp", type=float, default=3.0,
        help="控制器比例增益（default: 3.0）"
    )
    parser.add_argument(
        "--dt", type=float, default=0.02,
        help="每步之间的实际等待时间（秒），控制动画速度（default: 0.02，即 50 FPS）"
    )
    args = parser.parse_args()

    # ==========================================================
    # 1. 初始化场景
    # ==========================================================
    print("=" * 60)
    print(f"带可视化窗口的末端跟随演示 —— 模式: {args.mode}")
    print("=" * 60)

    print("\n[1/3] 初始化场景...")
    env = UpperBodyEnv()
    env.reset()
    print(f"  场景加载完成")

    # ==========================================================
    # 2. 配置目标生成器和控制器
    # ==========================================================
    print("\n[2/3] 配置目标点和控制器...")

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
        print(f"  控制器: 单臂({args.side}), kp={args.kp}")

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
        print(f"  控制器: 单臂({args.side}), kp={args.kp}")

    elif args.mode == "bimanual":
        # 双臂跟随：右手画圆，左手画直线
        target_gen_right = CircleTargetGenerator(
            center=[0.4, 0.2, 0.4], radius=0.15, axis="xz", speed=0.6
        )
        target_gen_left = LineTargetGenerator(
            start=[-0.4, -0.1, 0.3], end=[-0.6, -0.1, 0.5], speed=0.2
        )
        controller = BimanualController(env, kp=args.kp)
        print(f"  目标生成器: 右手画圆 + 左手画直线")
        print(f"  控制器: 双臂, kp={args.kp}")

    # ==========================================================
    # 3. 启动 MuJoCo viewer 并运行仿真
    # ==========================================================
    print(f"\n[3/3] 启动 MuJoCo 可视化窗口...")
    print(f"  总步数: {args.steps}")
    print(f"  操作提示: 鼠标拖拽旋转视角 | 滚轮缩放 | 空格暂停 | ESC 退出")
    print()

    # 记录误差
    errors = []
    step = 0

    # 启动被动 viewer（不阻塞主线程）
    with launch_passive(env.model, env.data) as viewer:
        # 设置相机视角
        viewer.cam.azimuth = 90   # 水平角度
        viewer.cam.elevation = -20  # 俯仰角度
        viewer.cam.distance = 2.0   # 相机距离
        viewer.cam.lookat[:] = [0.0, 0.0, 0.4]  # 看向场景中心

        real_start = time_module.time()

        # 主仿真循环
        while viewer.is_running() and step < args.steps:
            # ---- 获取当前目标点 ----
            t = env.data.time
            if args.mode == "bimanual":
                target_right = target_gen_right.get_target(t)
                target_left = target_gen_left.get_target(t)
                error_info = controller.update(target_right, target_left)
                joint_targets = controller.get_joint_targets()
                errors.append(error_info["right_error"])
                # 更新可视化目标点位置
                env.set_target_position("right", target_right)
                env.set_target_position("left", target_left)
            else:
                target = target_gen.get_target(t)
                error = controller.update(target)
                joint_targets = controller.joint_targets
                errors.append(error)
                # 【修复】根据 args.side 更新对应的目标球位置
                # 红色球 = "right" 目标，蓝色球 = "left" 目标
                # 静态模式仅在开始时设置一次（后续位置不变，但为确保首次设置正确也更新）
                if args.side == "right":
                    env.set_target_position("right", target)
                else:  # left
                    env.set_target_position("left", target)

            # ---- 设置关节目标并仿真一步 ----
            env.set_joint_targets(joint_targets)
            env.step()

            # ---- 更新 viewer ----
            viewer.sync()

            # ---- 控制动画速度：等待 dt 秒，让用户能看清运动过程 ----
            time_module.sleep(args.dt)

            step += 1

            # 每 500 步打印一次进度
            if step % 500 == 0:
                avg_error = np.mean(errors[-500:])
                print(f"  步 {step}/{args.steps}, 时间={t:.2f}s, "
                      f"平均误差={avg_error:.4f}m")

        real_elapsed = time_module.time() - real_start

    # ==========================================================
    # 4. 输出结果
    # ==========================================================
    print()
    print("-" * 50)
    print("Result Summary:")
    if len(errors) > 0:
        errors_arr = np.array(errors)
        print(f"  Final Error: {errors_arr[-1]:.4f} m")
        print(f"  Average Error: {np.mean(errors_arr):.4f} m")
        print(f"  Max Error: {np.max(errors_arr):.4f} m")
    print(f"  Simulation Steps: {step}")
    print(f"  Wall Clock Time: {real_elapsed:.2f} s")
    print()
    print("=" * 60)
    print("Demo complete! Visualization window closed.")
    print("=" * 60)

    env.close()


if __name__ == "__main__":
    main()
