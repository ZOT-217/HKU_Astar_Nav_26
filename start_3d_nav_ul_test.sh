#!/usr/bin/env bash
set -euo pipefail

OLD_NAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIVOX_SETUP="$OLD_NAV_ROOT/livox_ws/devel/setup.bash"
SIM_NAV_SETUP="$OLD_NAV_ROOT/sim_nav/devel/setup.bash"

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

exec roslaunch "$OLD_NAV_ROOT/3DNavUL_Test.launch" old_nav_root:="$OLD_NAV_ROOT" "$@"