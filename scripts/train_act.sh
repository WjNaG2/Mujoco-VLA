#!/usr/bin/env bash
set -e

# 训练 ACT baseline 的入口脚本。
# TODO: 实现 ACT 训练流程。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "[train_act] project root: $PROJECT_ROOT"

python "$PROJECT_ROOT/benchmark/policies/train_act.py" "$@"
