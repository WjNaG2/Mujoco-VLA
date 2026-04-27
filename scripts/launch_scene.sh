#!/usr/bin/env bash
set -e

# ============================================================
# 启动 MuJoCo 场景的入口脚本
#
# 用法：
#   ./scripts/launch_scene.sh                        # 启动默认场景（可视化）
#   ./scripts/launch_scene.sh --scene envs/simple_end_effector_scene.xml  # 指定场景
#
# Phase 1 新增功能：
#   ./scripts/launch_scene.sh --demo static           # 运行静态点跟随演示
#   ./scripts/launch_scene.sh --demo circle           # 运行圆形轨迹跟随演示
#   ./scripts/launch_scene.sh --demo bimanual         # 运行双臂跟随演示
#   ./scripts/launch_scene.sh --capture-rgbd          # 运行 RGBD 采集演示
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "[launch_scene] 项目根目录: $PROJECT_ROOT"

# 检查是否带 --demo 参数
if [[ "$1" == "--demo" ]]; then
    MODE="${2:-static}"
    echo "[launch_scene] 启动跟随演示: mode=$MODE"
    exec python "$PROJECT_ROOT/scripts/run_follow_demo.py" --mode "$MODE"
fi

# 检查是否带 --capture-rgbd 参数
if [[ "$1" == "--capture-rgbd" ]]; then
    echo "[launch_scene] 启动 RGBD 相机采集演示"
    exec python "$PROJECT_ROOT/scripts/capture_rgbd_demo.py" "${@:2}"
fi

# 默认：启动可视化场景（带 MuJoCo 窗口）
echo "[launch_scene] 启动可视化场景..."
exec python "$PROJECT_ROOT/envs/launch_scene.py" "$@"
