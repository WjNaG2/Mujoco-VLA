#!/usr/bin/env bash
set -e

# 启动 XR 遥操桥接的入口脚本。
# TODO: 根据 xr_teleoperate 安装路径和实际桥接代码更新命令。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "[run_teleop] project root: $PROJECT_ROOT"

python "$PROJECT_ROOT/teleop/run_teleop.py" "$@"
