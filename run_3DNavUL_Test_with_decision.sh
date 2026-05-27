#!/bin/bash
set -eo pipefail

OLD_NAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASTAR_ROOT="$(dirname "$OLD_NAV_ROOT")"
DECISION_WS="${DECISION_WS:-$ASTAR_ROOT/DecisionNode}"
LIVOX_SETUP="$OLD_NAV_ROOT/livox_ws/devel/setup.bash"
SIM_NAV_SETUP="$OLD_NAV_ROOT/sim_nav/devel/setup.bash"
EXPECTED_BOT_SIM="$OLD_NAV_ROOT/sim_nav/src/bot_sim"
EXPECTED_LIVOX_DRIVER="$OLD_NAV_ROOT/livox_ws/src/livox_ros_driver2"

function prepend_old_nav_package_path() {
  if [[ -n "${ROS_PACKAGE_PATH:-}" ]]; then
    export ROS_PACKAGE_PATH="$OLD_NAV_ROOT/sim_nav/src:$OLD_NAV_ROOT/livox_ws/src:$ROS_PACKAGE_PATH"
  else
    export ROS_PACKAGE_PATH="$OLD_NAV_ROOT/sim_nav/src:$OLD_NAV_ROOT/livox_ws/src"
  fi
}

function assert_ros_package_path() {
  local package_name="$1"
  local expected_path="$2"
  local actual_path

  actual_path="$(rospack find "$package_name" 2>/dev/null || true)"
  if [[ "$actual_path" != "$expected_path" ]]; then
    echo "[$(date '+%F %T')] ERROR: ROS package '$package_name' resolves to '$actual_path'"
    echo "[$(date '+%F %T')] ERROR: Expected '$expected_path'"
    echo "[$(date '+%F %T')] ERROR: Refusing to launch with a shadowed Old_nav package"
    exit 1
  fi
}

cd "$OLD_NAV_ROOT"

LOG_DIR="$OLD_NAV_ROOT/logs"
mkdir -p "$LOG_DIR"
exec >> "$LOG_DIR/3dnav_with_decision_autostart.log" 2>&1

echo "==== $(date '+%F %T') starting 3DNav autostart ===="

export HOME="${HOME:-/home/sentry_train_test}"
export ROS_HOME="${ROS_HOME:-$HOME/.ros}"
export ROS_DISTRO=noetic
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1

# Source ROS and all related workspaces. Source Old_nav last so its packages win
# over any inherited or decision-workspace packages with the same names.
source /opt/ros/noetic/setup.bash
if [ -f "$OLD_NAV_ROOT/Navigation-filter-test/devel/setup.bash" ]; then
  source "$OLD_NAV_ROOT/Navigation-filter-test/devel/setup.bash" --extend
fi
if [ -f "$DECISION_WS/devel/setup.bash" ]; then
  source "$DECISION_WS/devel/setup.bash" --extend
fi
if [[ ! -f "$LIVOX_SETUP" ]]; then
  echo "[$(date '+%F %T')] ERROR: Missing Livox workspace setup: $LIVOX_SETUP"
  exit 1
fi
if [[ ! -f "$SIM_NAV_SETUP" ]]; then
  echo "[$(date '+%F %T')] ERROR: Missing sim_nav workspace setup: $SIM_NAV_SETUP"
  exit 1
fi
source "$LIVOX_SETUP" --extend
source "$SIM_NAV_SETUP" --extend
prepend_old_nav_package_path
rospack profile >/dev/null 2>&1 || true
assert_ros_package_path bot_sim "$EXPECTED_BOT_SIM"
assert_ros_package_path livox_ros_driver2 "$EXPECTED_LIVOX_DRIVER"
echo "[$(date '+%F %T')] bot_sim package: $(rospack find bot_sim)"
echo "[$(date '+%F %T')] livox_ros_driver2 package: $(rospack find livox_ros_driver2)"

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


# 生成以当前时间命名的rosbag文件名


# 定义下电时自动保存rosbag的处理函数
function stop_rosbag() {
  echo "[$(date '+%F %T')] stopping rosbag..."
  if [ -n "$ROSBAG_PID" ] && kill -0 "$ROSBAG_PID" 2>/dev/null; then
    # 生成唯一关机rosbag名
    SHUTDOWN_TIME=$(date '+%Y%m%d_%H%M%S')
    SHUTDOWN_BAG_FILE="$LOG_DIR/rosbag_shutdown_${SHUTDOWN_TIME}.bag"
    kill -2 "$ROSBAG_PID"
    wait "$ROSBAG_PID"
    # 如果最后一个rosbag文件存在且未被重命名，则重命名为shutdown专用名
    if [ -f "$BAG_FILE" ]; then
      mv "$BAG_FILE" "$SHUTDOWN_BAG_FILE"
      echo "[$(date '+%F %T')] rosbag saved to $SHUTDOWN_BAG_FILE (shutdown)"
    else
      echo "[$(date '+%F %T')] rosbag saved to $BAG_FILE (shutdown)"
    fi
    ROSBAG_PID=""
  fi
echo "[$(date '+%F %T')] launching 3DNavUL_Test_with_decision.launch"

}
trap stop_rosbag SIGINT SIGTERM

# === 新增：每1分钟分割rosbag ===
SPLIT_COUNT=0
SPLIT_INTERVAL=60  # 单位：秒，每1分钟分割一次

function start_rosbag() {
  BAG_START_TIME=$(date '+%Y%m%d_%H%M%S')
  BAG_FILE="$LOG_DIR/rosbag_${BAG_START_TIME}_$SPLIT_COUNT.bag"
  echo "[$(date '+%F %T')] starting rosbag record to $BAG_FILE"
  rosbag record -a -O "$BAG_FILE" --lz4 &
  ROSBAG_PID=$!
}

function stop_rosbag_if_running() {
  if [ -n "$ROSBAG_PID" ] && kill -0 "$ROSBAG_PID" 2>/dev/null; then
    kill -2 "$ROSBAG_PID"
    wait "$ROSBAG_PID"
    echo "[$(date '+%F %T')] rosbag saved to $BAG_FILE"
    ROSBAG_PID=""
  fi
}



# 启动rosbag分割和roslaunch并发，主进程等待roslaunch退出
start_rosbag
(while true; do
  sleep "$SPLIT_INTERVAL"
  stop_rosbag_if_running
  SPLIT_COUNT=$((SPLIT_COUNT+1))
  start_rosbag
done) &
SPLIT_PID=$!

echo "[$(date '+%F %T')] launching 3DNavUL_Test_with_decision.launch"
roslaunch --wait "$OLD_NAV_ROOT/3DNavUL_Test_with_decision.launch" old_nav_root:="$OLD_NAV_ROOT"

# 退出时清理
kill $SPLIT_PID 2>/dev/null || true
stop_rosbag_if_running

# # 如需自动重启，请取消注释以下内容：
# while true; do
#   roslaunch --wait "$OLD_NAV_ROOT/3DNavUL_Test_with_decision.launch" old_nav_root:="$OLD_NAV_ROOT"
#   echo "[$(date '+%F %T')] 3DNavUL_Test_with_decision.launch exited, restarting..."
#   sleep 2
# done
# systemctl stop 
# 创建一个 systemd 服务文件（如 /etc/systemd/system/3dnav_with_decision.service）。
# 在服务文件中配置 ExecStart 指向你的脚本路径。
# 重新加载 systemd 配置：sudo systemctl daemon-reload
# 启用服务ccz：sudo systemctl enable 3dnav_with_decision
# 启动服务：sudo systemctl start 3dnav_with_decision