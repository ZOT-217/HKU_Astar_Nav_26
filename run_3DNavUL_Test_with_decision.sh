#!/bin/bash
set -eo pipefail

cd /home/calibur/HKU_Astar_Nav_26

LOG_DIR="/home/calibur/HKU_Astar_Nav_26/logs"
mkdir -p "$LOG_DIR"
exec >> "$LOG_DIR/3dnav_with_decision_autostart.log" 2>&1

echo "==== $(date '+%F %T') starting 3DNav autostart ===="

export HOME=/home/calibur
export ROS_HOME=/home/calibur/.ros
export ROS_DISTRO=noetic
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1

# Source ROS and all related workspaces
source /opt/ros/noetic/setup.bash
source /home/calibur/HKU_Astar_Nav_26/livox_ws/devel/setup.bash --extend
source /home/calibur/HKU_Astar_Nav_26/sim_nav/devel/setup.bash --extend
source /home/calibur/HKU_Astar_Nav_26/Navigation-filter-test/devel/setup.bash --extend
if [ -f /home/calibur/DecisionNode/devel/setup.bash ]; then
  source /home/calibur/DecisionNode/devel/setup.bash --extend
fi

# Short, configurable startup waits (systemd already orders network and ttyUSB0)
BOOT_SETTLE_SECONDS="${BOOT_SETTLE_SECONDS:-2}"
TTYUSB_WAIT_SECONDS="${TTYUSB_WAIT_SECONDS:-5}"
ROSCORE_WAIT_SECONDS="${ROSCORE_WAIT_SECONDS:-8}"

if [ "$BOOT_SETTLE_SECONDS" -gt 0 ] 2>/dev/null; then
  echo "[$(date '+%F %T')] settling for ${BOOT_SETTLE_SECONDS}s"
  sleep "$BOOT_SETTLE_SECONDS"
fi

# Wait briefly for the MCU serial device if needed
if [ -e /dev/ttyUSB0 ]; then
  echo "[$(date '+%F %T')] detected /dev/ttyUSB0"
else
  for ((i=1; i<=TTYUSB_WAIT_SECONDS; i++)); do
    if [ -e /dev/ttyUSB0 ]; then
      echo "[$(date '+%F %T')] detected /dev/ttyUSB0"
      break
    fi
    echo "[$(date '+%F %T')] waiting for /dev/ttyUSB0 ($i/${TTYUSB_WAIT_SECONDS})"
    sleep 1
  done
fi

if [ ! -e /dev/ttyUSB0 ]; then
  echo "[$(date '+%F %T')] /dev/ttyUSB0 still missing after ${TTYUSB_WAIT_SECONDS}s, continuing launch"
fi

# Start ROS master explicitly if it is not up yet
if ! timeout 3s rostopic list >/dev/null 2>&1; then
  echo "[$(date '+%F %T')] roscore not detected, starting it..."
  roscore >> "$LOG_DIR/roscore.log" 2>&1 &

  for ((i=1; i<=ROSCORE_WAIT_SECONDS; i++)); do
    if timeout 1s rostopic list >/dev/null 2>&1; then
      echo "[$(date '+%F %T')] roscore is ready"
      break
    fi
    sleep 1
  done
fi
# ============================================================
# 高效滚动 rosbag（rosbag 自带 --split 每60秒分片，退出时合并为一个文件）
# 分片存入隐藏目录 .rosbag_chunks/，用户只看到最终单个 .bag 文件
# ============================================================
BASENAME="rosbag_$(date '+%Y%m%d_%H%M%S')"
CHUNK_DIR="$LOG_DIR/.rosbag_chunks"
mkdir -p "$CHUNK_DIR"

echo "[$(date '+%F %T')] rosbag output: $LOG_DIR/${BASENAME}.bag"

# 启动 rosbag，每60秒自动分割为一个独立 .bag 文件
# rosbag 内部处理分片：chunk_N.bag.active -> chunk_N.bag（完成时自动重命名）
# 任何时候最多只有一个 .bag.active（当前分片）
rosbag record -a --split --duration=60 -o "$CHUNK_DIR/chunk" --lz4 &
ROSBAG_PID=$!

function merge_chunks_into_bag() {
  local out="$LOG_DIR/${BASENAME}.bag"
  local chunks=("$CHUNK_DIR"/chunk_*.bag)

  if [ ${#chunks[@]} -eq 0 ] || [ ! -f "${chunks[0]}" ]; then
    return 0
  fi

  # 只有一个分片，直接 rename
  if [ ${#chunks[@]} -eq 1 ]; then
    mv "${chunks[0]}" "$out"
    echo "[$(date '+%F %T')] rosbag finalized: $out"
    return 0
  fi

  # 多个分片，合并为一个文件（仅在退出时执行一次）
  echo "[$(date '+%F %T')] merging ${#chunks[@]} chunks into $out..."
  python3 << PYCHUNK 2>&1 | head -1
import rosbag, glob, os
chunks = sorted(glob.glob('$CHUNK_DIR/chunk_*.bag'))
if not chunks:
    exit(0)
out_path = '$out'
out = rosbag.Bag(out_path, 'w')
for f in chunks:
    with rosbag.Bag(f) as b:
        for t, m, s in b.read_messages():
            out.write(t, m, s)
out.close()
print('merged {} chunks into {}'.format(len(chunks), out_path))
PYCHUNK
  echo "[$(date '+%F %T')] merge done"
}

function stop_rosbag() {
  echo "[$(date '+%F %T')] stopping rosbag..."
  if [ -n "$ROSBAG_PID" ] && kill -0 "$ROSBAG_PID" 2>/dev/null; then
    kill -2 "$ROSBAG_PID"      # SIGINT -> 最终分片 .bag.active -> .bag
    wait "$ROSBAG_PID" 2>/dev/null || true
    ROSBAG_PID=""
  fi
  # 兜底：处理任何残留的 .bag.active
  for f in "$CHUNK_DIR"/*.bag.active; do
    [ -f "$f" ] && mv "$f" "${f%.active}"
  done
  merge_chunks_into_bag
  rm -rf "$CHUNK_DIR"
  echo "[$(date '+%F %T')] rosbag done"
}

trap stop_rosbag SIGINT SIGTERM

echo "[$(date '+%F %T')] launching 3DNavUL_Test_with_decision.launch"
roslaunch --wait /home/calibur/HKU_Astar_Nav_26/3DNavUL_Test_with_decision.launch

# roslaunch 退出后清理
stop_rosbag
# 创建一个 systemd 服务文件（如 /etc/systemd/system/3dnav_with_decision.service）。
# 在服务文件中配置 ExecStart 指向你的脚本路径。
# 重新加载 systemd 配置：sudo systemctl daemon-reload
# 启用服务ccz：sudo systemctl enable 3dnav_with_decision
# 启动服务：sudo systemctl start 3dnav_with_decision