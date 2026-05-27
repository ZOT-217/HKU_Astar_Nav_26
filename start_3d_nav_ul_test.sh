#!/usr/bin/env bash
set -euo pipefail

OLD_NAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIVOX_SETUP="$OLD_NAV_ROOT/livox_ws/devel/setup.bash"
SIM_NAV_SETUP="$OLD_NAV_ROOT/sim_nav/devel/setup.bash"
EXPECTED_BOT_SIM="$OLD_NAV_ROOT/sim_nav/src/bot_sim"
EXPECTED_LIVOX_DRIVER="$OLD_NAV_ROOT/livox_ws/src/livox_ros_driver2"

prepend_old_nav_package_path() {
  if [[ -n "${ROS_PACKAGE_PATH:-}" ]]; then
    export ROS_PACKAGE_PATH="$OLD_NAV_ROOT/sim_nav/src:$OLD_NAV_ROOT/livox_ws/src:$ROS_PACKAGE_PATH"
  else
    export ROS_PACKAGE_PATH="$OLD_NAV_ROOT/sim_nav/src:$OLD_NAV_ROOT/livox_ws/src"
  fi
}

assert_ros_package_path() {
  local package_name="$1"
  local expected_path="$2"
  local actual_path

  actual_path="$(rospack find "$package_name" 2>/dev/null || true)"
  if [[ "$actual_path" != "$expected_path" ]]; then
    echo "ERROR: ROS package '$package_name' resolves to '$actual_path'" >&2
    echo "ERROR: Expected '$expected_path'" >&2
    echo "ERROR: Start from this script so Old_nav packages stay ahead of other workspaces." >&2
    exit 1
  fi
}

if [[ ! -f "$LIVOX_SETUP" ]]; then
  echo "ERROR: Missing Livox workspace setup: $LIVOX_SETUP" >&2
  exit 1
fi

if [[ ! -f "$SIM_NAV_SETUP" ]]; then
  echo "ERROR: Missing sim_nav workspace setup: $SIM_NAV_SETUP" >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
source "$LIVOX_SETUP" --extend
source "$SIM_NAV_SETUP" --extend
prepend_old_nav_package_path
rospack profile >/dev/null 2>&1 || true
assert_ros_package_path bot_sim "$EXPECTED_BOT_SIM"
assert_ros_package_path livox_ros_driver2 "$EXPECTED_LIVOX_DRIVER"

exec roslaunch "$OLD_NAV_ROOT/3DNavUL_Test.launch" \
  old_nav_root:="$OLD_NAV_ROOT" \
  enable_cmd_vel_markers:=true \
  enable_world_align:=true \
  "$@"