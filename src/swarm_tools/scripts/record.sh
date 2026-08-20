#!/bin/bash
# ============================================================
# record.sh —— 一键录制轨迹跟踪数据 (阶段4)
#
# 用法:
#   ./record.sh [轨迹类型] [控制器类型] [录制时长]
# 示例:
#   ./record.sh fig8 pd 60    # 8字轨迹 + PD控制器 + 60秒
#   ./record.sh circle pd 45  # 圆形轨迹 + PD控制器 + 45秒
#
# 功能:
#   - 一次录 4 个话题: odom / ref / cmd_vel / ctrl_debug
#   - bag 自动命名: 日期_轨迹类型_控制器类型.bag
#   - 保存到 swarm_tools/bags/ (该目录已被 .gitignore 忽略)
# ============================================================

set -e

TRAJ=${1:-fig8}      # 第1个参数: 轨迹类型, 默认 fig8
CTRL=${2:-pd}        # 第2个参数: 控制器类型, 默认 pd
DUR=${3:-60}         # 第3个参数: 录制时长(秒), 默认 60

DATE=$(date +%Y%m%d_%H%M%S)               # 如 20260817_153000
NAME="${DATE}_${TRAJ}_${CTRL}.bag"        # 自动命名

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd) # 脚本所在目录(绝对路径)
BAG_DIR="$SCRIPT_DIR/../bags"             # 上一级的 bags 目录
mkdir -p "$BAG_DIR"
OUT="$BAG_DIR/$NAME"

echo "=============================================="
echo " 开始录制: $OUT"
echo " 话题    : /robot_1/odom"
echo "           /robot_1/ref"
echo "           /robot_1/cmd_vel"
echo "           /robot_1/ctrl_debug"
echo " 时长    : ${DUR} 秒 (Ctrl+C 可提前结束)"
echo "=============================================="

rosbag record -O "$OUT" --duration="$DUR" \
  /robot_1/odom \
  /robot_1/ref \
  /robot_1/cmd_vel \
  /robot_1/ctrl_debug

echo ""
echo "录制完成: $OUT"
echo "画图命令: python3 $SCRIPT_DIR/plot_tracking.py $OUT"
