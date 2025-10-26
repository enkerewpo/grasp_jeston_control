#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

# 启动容器
docker compose up -d --build

# 查看容器日志
echo "=== Press Ctrl+C to exit logs ==="
docker compose logs -f