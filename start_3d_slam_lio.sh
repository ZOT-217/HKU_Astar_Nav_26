#!/usr/bin/env bash
set -euo pipefail

OLD_NAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIVOX_CONFIG="$OLD_NAV_ROOT/livox_ws/src/livox_ros_driver2/config/MID360_config.json"
LIVOX_SETUP="$OLD_NAV_ROOT/livox_ws/devel/setup.bash"
SIM_NAV_SETUP="$OLD_NAV_ROOT/sim_nav/devel/setup.bash"

if [[ ! -f "$LIVOX_SETUP" ]]; then
	echo "ERROR: Missing Livox workspace setup: $LIVOX_SETUP" >&2
	echo "Build Old_nav/livox_ws before starting SLAM." >&2
	exit 1
fi

if [[ ! -f "$SIM_NAV_SETUP" ]]; then
	echo "ERROR: Missing sim_nav workspace setup: $SIM_NAV_SETUP" >&2
	echo "Build Old_nav/sim_nav before starting SLAM." >&2
	exit 1
fi

if pgrep -f "livox_ros_driver2_node" >/dev/null 2>&1; then
	echo "ERROR: A Livox driver process is already running." >&2
	echo "Stop the existing SLAM/Livox launch before starting another one." >&2
	exit 2
fi

if [[ "${SKIP_LIVOX_IP_CHECK:-0}" != "1" ]]; then
	if [[ ! -f "$LIVOX_CONFIG" ]]; then
		echo "ERROR: Missing Livox config: $LIVOX_CONFIG" >&2
		exit 1
	fi

	HOST_IP="$(grep -m1 '"cmd_data_ip"' "$LIVOX_CONFIG" | sed -E 's/.*"cmd_data_ip"[[:space:]]*:[[:space:]]*"([^"]+)".*/\1/')"
	if [[ -z "$HOST_IP" ]]; then
		echo "ERROR: Could not read host cmd_data_ip from $LIVOX_CONFIG" >&2
		exit 1
	fi

	if ! ip -o -4 addr show | awk '{print $4}' | cut -d/ -f1 | grep -qx "$HOST_IP"; then
		echo "ERROR: Livox host IP $HOST_IP is not assigned to any local interface." >&2
		echo "The Livox SDK would fail with: bind failed / Create detection socket failed." >&2
		echo "Current IPv4 addresses:" >&2
		ip -br -4 addr show >&2
		echo "Assign $HOST_IP/24 to the Ethernet interface connected to the MID360, then retry." >&2
		exit 3
	fi
fi

source /opt/ros/noetic/setup.bash
source "$LIVOX_SETUP" --extend
source "$SIM_NAV_SETUP" --extend

exec roslaunch "$OLD_NAV_ROOT/3DSlamFinal_lio.launch" old_nav_root:="$OLD_NAV_ROOT" "$@"