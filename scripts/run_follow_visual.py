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
        target_gen = StaticTargetGenerator(pos=[0.5, 0.2, 0.4])
        controller = EndEffectorController(env, side=args.side, kp=args.kp)
        print(f"  目标生成器: {target_gen}")
        print(f"  控制器: 单臂({args.side}), kp={args.kp}")

    elif args.mode == "circle":
        # 版本 C：圆形轨迹
        target_gen = CircleTargetGenerator(
            center=[0.4, 0.0, 0.4], radius=0.18, axis="xz", speed=0.8
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
                # 更新可视化目标点位置（圆形轨迹时目标点在移动）
                if args.mode == "circle":
                    env.set_target_position("right", target)

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
    print("📊 运行结果摘要:")
    if len(errors) > 0:
        errors_arr = np.array(errors)
        print(f"  最终误差: {errors_arr[-1]:.4f} 米")
        print(f"  平均误差: {np.mean(errors_arr):.4f} 米")
        print(f"  最大误差: {np.max(errors_arr):.4f} 米")
    print(f"  仿真步数: {step}")
    print(f"  实际运行时间: {real_elapsed:.2f} 秒")
    print()
    print("=" * 60)
    print("✅ 演示完成！可视化窗口已关闭。")
    print("=" * 60)

    env.close()


if __name__ == "__main__":
    main()
