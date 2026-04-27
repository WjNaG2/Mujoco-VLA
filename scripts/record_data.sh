#!/usr/bin/env bash
set -e

# 录制数据脚本入口。
# TODO: 实现数据录制逻辑。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "[record_data] project root: $PROJECT_ROOT"

python "$PROJECT_ROOT/scripts/record_data.py" "$@"
