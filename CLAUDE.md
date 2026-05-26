# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

HKU Astar autonomous navigation system for a ground robot (RoboMaster competition). ROS 1 Noetic on Ubuntu. The robot uses a Livox MID360 3D LiDAR and IMU for SLAM-based localization and D*-Lite / DWA for path planning with obstacle avoidance.

## Build

Three catkin workspaces must be sourced in dependency order:

```bash
# 1. Livox LiDAR driver
cd livox_ws && catkin_make

# 2. Main navigation workspace (depends on livox_ws)
cd sim_nav && catkin_make

# 3. Navigation filter tests (optional submodule)
cd Navigation-filter-test && catkin_make
```

**Runtime environment** — source all workspaces before launching:
```bash
source /opt/ros/noetic/setup.bash
source livox_ws/devel/setup.bash --extend
source sim_nav/devel/setup.bash --extend
source Navigation-filter-test/devel/setup.bash --extend   # if present
```

Compiled with `-std=c++14` and `-O3`.

## Docker (macOS development)

```bash
# Build image
docker compose build

# Build all workspaces and enter shell
docker compose run --rm nav

# Subsequent runs (skip rebuild, just enter shell)
docker compose run --rm nav bash

# With RViz GUI (requires XQuartz running first: xhost +localhost)
docker compose run --rm -e DISPLAY=host.docker.internal:0 nav bash
```

After entering the container, source the built workspaces:
```bash
source /workspace/livox_ws/devel/setup.bash --extend
source /workspace/sim_nav/devel/setup.bash --extend
```

### If at mainland China:
```bash
# config .env to set proxy, see .env.example for reference
cp .env.example .env
# pull osrf/ros:noetic-desktop-full manually (may be blocked by GFW, use VPN or mirror registry if needed)
docker pull osrf/ros:noetic-desktop-full
# build image with no-cache to ensure all layers are built with proxy settings
docker compose build --no-cache
```

## Architecture

### Workspaces

| Workspace | Purpose |
|---|---|
| `sim_nav/` | Main workspace — all navigation, planning, control, and SLAM packages |
| `livox_ws/` | Livox ROS 1 driver (`livox_ros_driver2`) |

### Key packages in `sim_nav/src/`

**`bot_sim`** — Core package. Contains all planning, control, transform, and sensor processing nodes:
- `dstarlite.cpp` — D*-Lite global planner consuming `nav_msgs/OccupancyGrid` (legacy DBSCAN pipeline)
- `dstarlite_esdf.cpp` — D*-Lite planner querying ROG-Map ESDF directly (current recommended planner)
- `dwa.cpp` — DWA local planner
- `ser2msg_decision_givepoint.cpp` — Serial ↔ ROS bridge, TF broadcast chain (rotbase → virtual → gimbal → LiDAR), and goal relay to planner
- `imu_filter.cpp` — Removes gravity from raw IMU data
- `real_robot_transform.cpp` — Computes intermediate coordinate frames between LiDAR odom and robot
- `rog_map_node.cpp` — Stage 1: passive 3D ROG-Map observer (PointCloud2 → voxel grid visualization)
- `rog_map_to_grid_node.cpp` — Stage 2: ROG-Map 3D → `nav_msgs/OccupancyGrid` 2D projection bridge
- `dbscan_bfs_3D.cpp` — DBSCAN clustering + BFS for traversability (legacy occupancy grid pipeline)
- `world_align_node.cpp` — World frame alignment for coordinate systems
- `cmd_vel_marker.cpp` — Visualizes `/cmd_vel` as RViz arrow markers
- `velocity_pid_controller.hpp` — Header-only PID velocity controller between planner output and `/cmd_vel`

**`rog_map`** — 3D occupancy map library (`rog_map/rog_map.h`):
- `prob_map` — Probabilistic occupancy
- `inf_map` — Inflation map
- `esdf_map` — Euclidean Signed Distance Field (used by `dstarlite_esdf`)
- `sliding_map`, `counter_map`, `free_cnt_map` — Internal map utilities

**`hdl_graph_slam`** — Offline/online graph-based SLAM (g2o backend). Produces PCD maps for later localization. Supports IMU fusion via `hdl_graph_slam_imu.launch`.

**`hdl_localization`** — Real-time LiDAR localization via NDT or GICP scan-to-map registration against a prior PCD map. Publishes `geometry_msgs/PoseStamped` for the robot pose.

**`hdl_global_localization`** — Global localization using PCL feature matching (used for initial pose estimation before fine registration).

**`Point-LIO`** — LiDAR-inertial odometry (tightly-coupled ESIKF). Alternative frontend for SLAM mapping.

**`fast_gicp` / `ndt_omp`** — Point cloud registration backends (GICP and NDT variants, OpenMP-accelerated).

**`dm_imu`** — DM IMU sensor driver.

### Navigation pipeline (launch file: `3DNavUL_Test.launch`)

```
LiDAR driver (MID360) ──► imu_filter (optional) ──► hdl_localization (NDT)
                                                          │
                                                    TF: map → aft_mapped
                                                          │
                                              real_robot_transform
                                                          │
                                          TF chain: map → virtual_frame
                                                          │
                          ┌───────────────────────────────┤
                          ▼                               ▼
              dstarlite (D*-Lite)              ser2msg_decision_givepoint
              publishes /cmd_vel               (serial ↔ MCU, TF pub, goal relay)
```

Optional stages (gated by launch args):
- **Stage 1** (`enable_rog_map_observer:=true`): `rog_map_node` publishes 3D voxel visualization
- **Stage 2** (`enable_rog_costmap:=true`): `rog_map_to_grid_node` bridges 3D → 2D grid; `dstarlite` consumes `/rog_grid` instead of `/grid`

`nav_esdf.launch` is a simplified variant that uses `dstarlite_esdf` (direct ESDF integration, no 2D grid round-trip).

`3DNavUL_Test_with_decision.launch` adds MCU serial communication and strategy decision nodes from the external `DecisionNode` package.

### TF frame hierarchy

```
map ──► virtual_frame ──► rotbase_frame ──► gimbal_frame ──► aft_mapped (LiDAR odom output)
```

`ser2msg_decision_givepoint` broadcasts `rotbase_frame → virtual_frame` (rotation via yaw angle) and `gimbal_frame → rotbase_frame` (static). The `real_robot_transform` node provides additional intermediate frames.

### Planner overview

**D*-Lite** (`dstarlite.cpp`, `dstarlite_esdf.cpp`): Incremental heuristic search on a grid. Uses a sigmoid edge-cost function on obstacle probability. Velocity is modulated by distance to obstacle and proximity to goal. PID controller (`velocity_pid_controller.hpp`) corrects the raw planner output using TF-derived velocity feedback.

The ESDF variant queries `rog_map::ROGMap` directly for obstacle distances, replacing the legacy DBSCAN → OccupancyGrid pipeline. Configuration loaded via `config/rog_map_config.yaml`.

## Launch files at repo root

| File | Purpose |
|---|---|
| `3DNavUL_Test.launch` | Main nav pipeline (no decision/strategy) |
| `3DNavUL_Test_with_decision.launch` | Nav + MCU comm + strategy decision |
| `nav_esdf.launch` | ESDF-based nav (dstarlite_esdf) |
| `3DSlamFinal_lio.launch` | SLAM mapping with Point-LIO |
| `hdl_graph_slam_mapping.launch` | SLAM mapping with HDL Graph SLAM |

## Common operations

**Start full navigation (no decision):**
```bash
roslaunch 3DNavUL_Test.launch
```

**Start nav with ESDF planner:**
```bash
roslaunch nav_esdf.launch
```

**Start nav with ROG-Map costmap integration:**
```bash
roslaunch 3DNavUL_Test.launch enable_rog_costmap:=true costmap_source:=rog
```

**Record all topics:**
```bash
rosbag record -a -O <output_path>
```

**Teleoperation:**
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

**Send a navigation goal** — publish a `geometry_msgs/PointStamped` to `/clicked_point`.

## Submodule dependencies

- `Navigation-filter-test` — filter testing utilities (branch `RMUC2026`)
- `livox_ws/src/livox_ros_driver2` — Livox ROS driver (forked, branch `parameter_RMUL2026_gimbal`)

Both are git submodules. After cloning, run:
```bash
git submodule update --init --recursive
```

## Key configuration

- Serial device: `/dev/ttyUSB0` at 921600 baud
- PID parameters are in `dstarlite.launch` / `dstarlite_esdf.launch` under the node's private namespace
- ROG-Map config: `sim_nav/src/bot_sim/config/rog_map_config.yaml`
- Static map: `sim_nav/src/bot_sim/map/innowing.yaml` (PGM format, served by `map_server`)
- The `TELEOP` button on the MCU must be held for serial communication to relay navigation commands (safety interlock)

