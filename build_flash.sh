#!/usr/bin/env bash
# 编译 + 自动烧录(编译成功后立即刷写并复位)。
# 用法: ./build_flash.sh [端口] ,默认端口 /dev/ttyUSB0
cd "$(dirname "$0")" || exit 1
PORT="${1:-/dev/ttyUSB0}"
. "$HOME/.espressif/v6.1/esp-idf/export.sh" >/dev/null || { echo "IDF export 失败"; exit 1; }
idf.py build || { echo "编译失败,跳过烧录"; exit 1; }
echo ">>> 编译完成,自动烧录到 $PORT ..."
idf.py -p "$PORT" flash
