#!/usr/bin/env bash
set -e

# 运行 benchmark 评估的入口脚本。
# TODO: 实现统一评估流程。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "[eval_all] project root: $PROJECT_ROOT"

python "$PROJECT_ROOT/benchmark/evaluators/run_benchmark.py" "$@"
