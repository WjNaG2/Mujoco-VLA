#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_real_xr_to_mujoco.py —— 真实 XR 设备 → MuJoCo 仿真机器人跟随

完整链路：
    真实 XR 设备 → TeleVuerWrapper → RealXRSource → XRToMuJoCoBridge
    → 末端目标位置 → BimanualController → MuJoCo 仿真机器人

依赖（必读）：
    1. 本脚本需要 xr_teleoperate/teleop/televuer 已安装：
           cd third_party/xr_teleoperate/teleop/televuer && pip install -e .
    2. SSL 证书已配置（见 stage2_report.md 第 10 节或 xr_teleoperate 官方文档）
    3. XR 设备与主机在同一局域网
    4. 防火墙端口已开放（sudo ufw allow 8012）

运行方式：
    # 简单模式（pass-through + 手势跟踪）
    conda run -n mujoco_vla python teleop/run_real_xr_to_mujoco.py \
        --host-ip 192.168.123.2

    # 沉浸模式（需配置图像服务）
    conda run -n mujoco_vla python teleop/run_real_xr_to_mujoco.py \
        --host-ip 192.168.123.2 --display-mode immersive --img-server-ip 192.168.123.164

    # 手柄跟踪模式
    conda run -n mujoco_vla python teleop/run_real_xr_to_mujoco.py \
        --host-ip 192.168.123.2 --use-controller
"""

import os
import sys

# 自定义 print 函数，强制每次输出后 flush，确保 conda run 下实时显示
_print = print
def print(*args, **kwargs):
    kwargs.setdefault("flush", True)
    _print(*args, **kwargs)

import argparse


import time as time_module

# 将项目根目录加入 Python 路径

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from envs.upper_body_env import UpperBodyEnv
from controllers.end_effector_controller import BimanualController
from teleop.teleop_bridge import RealXRSource, XRToMuJoCoBridge


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
        description="真实 XR 设备 → MuJoCo 完整链路演示"
    )
    parser.add_argument("--host-ip", type=str, default="192.168.123.2",
                        help="主机 IP 地址（XR 设备通过此地址连接）")
    parser.add_argument("--port", type=int, default=8012,
                        help="Vuer WebSocket 端口")
    parser.add_argument("--display-mode", type=str, default="pass-through",
                        choices=["immersive", "pass-through", "ego"],
                        help="XR 显示模式（默认 pass-through，无需图像服务）")
    parser.add_argument("--img-server-ip", type=str, default="192.168.123.164",
                        help="图像服务器 IP（仅 immersive/ego 模式需要）")
    parser.add_argument("--use-controller", action="store_true",
                        help="使用手柄跟踪（默认使用手势跟踪）")
    parser.add_argument("--steps", type=int, default=3000,
                        help="仿真总步数")
    parser.add_argument("--viewer", action="store_true",
                        help="启用 MuJoCo 可视化窗口")
    parser.add_argument("--kp", type=float, default=3.0,
                        help="控制器比例增益")
    parser.add_argument("--cert-file", type=str, default=None,
                        help="SSL 证书路径（可选，默认自动搜索）")
    parser.add_argument("--key-file", type=str, default=None,
                        help="SSL 私钥路径（可选，默认自动搜索）")
    parser.add_argument("--connection-timeout", type=float, default=300.0,
                        help="等待 XR 设备连接的超时时间（秒，默认 300）")

    args = parser.parse_args()

    print("=" * 70)
    print("真实 XR 设备 → MuJoCo 完整链路演示")
    print(f"  主机 IP:     {args.host_ip}:{args.port}")
    print(f"  显示模式:    {args.display_mode}")
    print(f"  跟踪模式:    {'手柄' if args.use_controller else '手势'}")
    print(f"  启用 Viewer: {args.viewer}")
    print("=" * 70)

    # ==========================================================
    # 1. 初始化 MuJoCo 环境
    # ==========================================================
    print("\n[1/5] 初始化 MuJoCo 场景...")
    env = UpperBodyEnv()
    env.reset()

    if args.viewer:
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
    # 2. 初始化真实 XR 数据源
    # ==========================================================
    print("\n[2/5] 初始化真实 XR 数据源...")
    print(f"  请确保:")
    print(f"    1. XR 设备已连接到同一局域网")
    print(f"    2. 在 XR 设备浏览器中访问:")
    print(f"       https://{args.host_ip}:{args.port}/?ws=wss://{args.host_ip}:{args.port}")
    print(f"    3. 点击 Virtual Reality 按钮并允许权限")
    print(f"  正在等待 XR 设备连接...")

    xr_source = RealXRSource(
        host_ip=args.host_ip,
        port=args.port,
        use_hand_tracking=not args.use_controller,
        display_mode=args.display_mode,
        cert_file=args.cert_file,
        key_file=args.key_file,
        img_server_ip=args.img_server_ip,
        connection_timeout=args.connection_timeout,
    )

    bridge = XRToMuJoCoBridge()
    print("  ✅ 桥接层就绪")

    # ==========================================================
    # 3. 初始化双臂控制器
    # ==========================================================
    print("\n[3/5] 初始化双臂控制器...")
    controller = BimanualController(env, kp=args.kp)
    controller.reset()
    print(f"  ✅ 控制器就绪 (kp={args.kp})")

    # ==========================================================
    # 4. 运行主循环
    # ==========================================================
    print(f"\n[4/5] 开始仿真 ({args.steps} 步)...")
    print(f"  请在终端按 Ctrl+C 提前退出")

    # 记录数据
    right_errors = []
    left_errors = []
    right_targets = []
    left_targets = []
    right_actuals = []
    left_actuals = []
    joint_positions_log = []
    timestamps = []

    render_skip = max(1, args.steps // 3000) if args.viewer else 0
    real_start = time_module.time()

    try:
        for step in range(args.steps):
            # ---- A. 从真实 XR 设备获取 tele_data ----
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

            # ---- F. 更新目标点可视化 ----
            if args.viewer:
                env.set_target_position("right", target_right)
                env.set_target_position("left", target_left)

            # ---- G. 渲染 viewer ----
            if args.viewer and step % render_skip == 0 and step > 0:
                viewer.sync()

            # 进度打印
            if step > 0 and step % 500 == 0:
                print(f"  步 {step}/{args.steps} | "
                      f"右误差={np.mean(right_errors[-500:]):.4f}m | "
                      f"左误差={np.mean(left_errors[-500:]):.4f}m")

    except KeyboardInterrupt:
        print(f"\n  ⏹️  用户中断，提前退出 (已运行 {step+1} 步)")

    real_elapsed = time_module.time() - real_start
    sim_elapsed = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0

    # ==========================================================
    # 5. 保存结果
    # ==========================================================
    print(f"\n[5/5] 保存结果...")

    output_dir = os.path.join(project_root, "data", "samples", "xr_teleop_demo")
    os.makedirs(output_dir, exist_ok=True)

    data_path = os.path.join(output_dir, "episode_data.npz")
    np.savez(data_path,
        source="real_xr",
        host_ip=args.host_ip,
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

    error_plot_path = os.path.join(output_dir, "tracking_error.png")
    plot_tracking_error(np.array(right_errors), np.array(left_errors), error_plot_path)

    avg_right_error = np.mean(right_errors) if right_errors else 0
    avg_left_error = np.mean(left_errors) if left_errors else 0

    summary_path = os.path.join(output_dir, "summary.txt")
    with open(summary_path, "w") as f:
        f.write("真实 XR 设备 → MuJoCo 完整链路演示 - 运行摘要\n")
        f.write("=" * 50 + "\n")
        f.write(f"数据源: 真实 XR 设备\n")
        f.write(f"主机 IP: {args.host_ip}:{args.port}\n")
        f.write(f"显示模式: {args.display_mode}\n")
        f.write(f"跟踪模式: {'手柄' if args.use_controller else '手势'}\n")
        f.write(f"总仿真步数: {step+1}\n")
        f.write(f"仿真时间: {sim_elapsed:.2f} s\n")
        f.write(f"实际用时: {real_elapsed:.2f} s\n")
        f.write(f"控制器增益 kp: {args.kp}\n\n")
        f.write("--- 跟踪误差统计 (米) ---\n")
        f.write(f"右臂 平均误差: {avg_right_error:.4f}\n")
        f.write(f"左臂 平均误差: {avg_left_error:.4f}\n")

    print(f"  [Saved] Summary -> {summary_path}")

    print("\n" + "-" * 50)
    print("运行结果汇总:")
    print(f"  右臂平均跟踪误差: {avg_right_error:.4f} m")
    print(f"  左臂平均跟踪误差: {avg_left_error:.4f} m")
    print(f"  实际用时: {real_elapsed:.2f}s, 仿真时间: {sim_elapsed:.2f}s")
    print(f"  输出目录: {output_dir}")

    # 清理
    xr_source.close()
    if viewer is not None:
        viewer.close()
    env.close()

    print("\n" + "=" * 70)
    print("✅ 演示完成！")
    print("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except ConnectionError as e:
        print(f"\n❌ 连接错误: {e}")
        print("请修复后重试。")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\n⏹️  用户中断")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ 运行时错误: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

