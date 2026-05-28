# HKU Astar Autonomous Navigation System

Autonomous navigation system for a RoboMaster ground-based sentry robot. Real-time 3D LiDAR localization, dynamic obstacle mapping, D*-Lite global planning with PID velocity tracking, and BehaviorTree-based strategy decision — all integrated over ROS 1 Noetic.

**Key technologies:** Livox MID360 · NDT scan-to-map localization (hdl_localization) · ROG-Map 3D occupancy · D*-Lite global planner · BehaviorTree.CPP v3 · MCU serial bridge (HK protocol)

---

## System Dependencies

### OS & ROS

| Component | Version |
|-----------|---------|
| Ubuntu | 20.04 |
| ROS | Noetic (desktop-full) |
| C++ standard | C++14 |

### System Packages

```bash
apt-get install -y \
    libpcl-dev libeigen3-dev libopencv-dev \
    ros-noetic-libg2o ros-noetic-serial ros-noetic-geodesy \
    ros-noetic-nmea-msgs ros-noetic-interactive-markers \
    ros-noetic-costmap-converter ros-noetic-teb-local-planner \
    libapr1-dev libdw-dev libomp-dev \
    libyaml-cpp-dev libzmq3-dev libsqlite3-dev \
    python3-scipy python3-progressbar
```

### External Libraries (built from source)

| Library | Version | Required by |
|---------|---------|-------------|
| [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2) | latest | livox_ros_driver2 |
| [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) | v3.8 | decision_node |

### ROS Packages by Workspace

| Workspace | Package | Role |
|-----------|---------|------|
| `livox_ws/` | `livox_ros_driver2` | Livox MID360 LiDAR driver |
| `sim_nav/` | `bot_sim` | Planning, control, TF, IMU filter, ROG-Map bridge |
| | `hdl_localization` | NDT scan-to-map localization |
| | `hdl_graph_slam` | PCD map provider (dependency) |
| | `hdl_global_localization` | Global localization (dependency) |
| | `ndt_omp` | NDT registration backend |
| | `fast_gicp` | GICP registration backend |
| | `rog_map` | 3D occupancy map library |
| `DecisionNode/` | `decision_node` | BehaviorTree strategy + MCU communication |

### Key Package Dependency Chains

```
decision_node ──► behaviortree_cpp_v3, serial, tf2
bot_sim ──► rog_map, costmap_converter, serial, tf2
hdl_localization ──► ndt_omp, fast_gicp, hdl_global_localization, pcl_ros
```

---

## Build

Three catkin workspaces must be built in dependency order.

### Prerequisites

```bash
source /opt/ros/noetic/setup.bash
```

livox_ros_driver2 requires the ROS 1 package manifest:

```bash
cd livox_ws/src/livox_ros_driver2
cp -f package_ROS1.xml package.xml
```

### Build Order

```bash
# 1. Livox driver
cd livox_ws && catkin_make -DROS_EDITION=ROS1 -j4

# 2. Main navigation (needs livox_ws sourced)
source livox_ws/devel/setup.bash --extend
cd sim_nav && catkin_make -j4

# 3. Decision node (independent)
cd DecisionNode && catkin_make -j4
```

### Docker (macOS / CI)

```bash
# Build and run (builds all 3 workspaces automatically)
docker compose run --rm nav

# Subsequent runs (skip rebuild)
docker compose run --rm nav bash

# With RViz GUI (requires XQuartz: xhost +localhost first)
docker compose run --rm -e DISPLAY=host.docker.internal:0 nav bash
```

After entering the container, source the workspaces:

```bash
source /workspace/livox_ws/devel/setup.bash --extend
source /workspace/sim_nav/devel/setup.bash --extend
source /workspace/DecisionNode/devel/setup.bash --extend
```

---

## Launch

### Source Workspaces

```bash
source /opt/ros/noetic/setup.bash
source <path>/livox_ws/devel/setup.bash --extend
source <path>/sim_nav/devel/setup.bash --extend
source <path>/DecisionNode/devel/setup.bash --extend
```

### Start the Full System

```bash
roslaunch 3DNavUL_Test_with_decision.launch
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `enable_rog_map_observer` | `false` | Stage 1: ROG-Map 3D occupancy observer (visualization) |
| `enable_rog_costmap` | `false` | Stage 2: ROG-Map → 2D grid bridge for D*-Lite |
| `costmap_source` | `dbscan` | Costmap for D*-Lite: `dbscan` (/grid) or `rog` (/rog_grid) |
| `serial_port` | `/dev/ttyUSB0` | MCU serial device |
| `baudrate` | `921600` | Serial baud rate |
| `nav_frequency` | `50` | Navigation command frequency (Hz) |
| `mcu_output` | `screen` | MCU node log output target |

### Launch with Options

```bash
# Default (static map only, no ROG-Map)
roslaunch 3DNavUL_Test_with_decision.launch

# With ROG-Map visualization
roslaunch 3DNavUL_Test_with_decision.launch enable_rog_map_observer:=true

# Full pipeline with dynamic ROG-Map costmap
roslaunch 3DNavUL_Test_with_decision.launch \
    enable_rog_map_observer:=true \
    enable_rog_costmap:=true \
    costmap_source:=rog
```

### Required Data Files

| File | Path | Purpose |
|------|------|---------|
| Static map PGM | `sim_nav/src/bot_sim/map/innowing.yaml` | Served by map_server as `/map` |
| NDT localization PCD | `sim_nav/src/hdl_graph_slam/map/innowing.pcd` | Reference map for NDT scan matching (generate from prior SLAM session) |
| LiDAR config | `livox_ros_driver2/config/MID360_config.json` | Livox MID360 parameters |
| BehaviorTree XML | `DecisionNode/src/decision_node/config/vision_test.xml` | BT structure for strategy_node |
| ROG-Map config | `sim_nav/src/bot_sim/config/rog_map_passive.yaml` | Stage 1 ROG-Map parameters |
| Strategy goals | `DecisionNode/src/decision_node/config/strategy_tree.yaml` | Red/blue team navigation waypoints |

---

## Architecture

### Pipeline Overview

```
┌── SENSOR ──────────────────────────────────────────────────┐
│  Livox MID360  ─►  /livox/lidar (PointCloud2, aft_mapped)  │
│  Livox IMU     ─►  /livox/imu (Imu)                        │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌── LOCALIZATION & MAPPING ───────────────────────────────────┐
│  imu_filter        /livox/imu ─► /livox/imu_filtered        │
│  hdl_localization  /livox/lidar + /livox/imu_filtered       │
│                    + innowing.pcd (NDT scan-to-map)          │
│                    ─► /odom, /aligned_points                 │
│                    ─► TF: map → aft_mapped                   │
│                                                              │
│  [optional] rog_map_node (Stage 1)                           │
│    /aligned_points + /odom ─► 3D occupancy voxel grid       │
│  [optional] rog_map_to_grid_node (Stage 2)                   │
│    infl_occ + /odom ─► /rog_grid (2D OccupancyGrid)         │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌── PLANNING & CONTROL ───────────────────────────────────────┐
│  map_server        ─► /map (static OccupancyGrid)            │
│  dstarlite (D*-Lite)                                         │
│    Input:  /map + /clicked_point + dynamic grid              │
│    Output: /cmd_vel (Twist, PID-corrected)                   │
│            /dstar_path (Path)                                │
│            /dstar_status (Bool)                              │
│    PID velocity tracking (feedforward + feedback from TF)    │
└──────────────────────┬──────────────────────────────────────┘
                       │
┌── DECISION & COMMUNICATION ─────────────────────────────────┐
│  strategy_node (BehaviorTree)                                │
│    Input:  30+ referee/game state topics, /odom, /dstar_status│
│    Output: /clicked_point, /motion, /spin, /target_yaw       │
│                                                              │
│  mcu_communicator (Serial ↔ ROS bridge)                      │
│    Serial → ROS: /referee/*, /robot/*, /enemy/*, /radar/*   │
│    ROS → Serial: /cmd_vel, /dstar_status, /motion, /spin    │
│                                                              │
│  ser2msg_decision_givepoint (TF Broadcaster)                 │
│    TF: map → virtual_frame                                   │
│    Pub: /sentinel_nav_position                               │
│                                                              │
│  real_robot_transform (TF Broadcaster)                       │
│    TF: aft_mapped → gimbal_frame (gravity-aligned)           │
│                                                              │
│  world_align_node                                            │
│    /sentinel_nav_position ─► /sentinel_world_position        │
└──────────────────────────────────────────────────────────────┘
```

### Node Inventory (16 active nodes)

| # | Node | Package | Type | Role |
|---|------|---------|------|------|
| 1 | `map_server` | map_server | map_server | Serve static PGM map |
| 2 | `imu_filter` | bot_sim | imu_filter | Gravity removal from IMU |
| 3 | `livox_lidar_publisher2` | livox_ros_driver2 | livox_ros_driver2_node | MID360 LiDAR driver |
| 4 | `velodyne_nodelet_manager` | nodelet | nodelet (manager) | Nodelet container |
| 5 | `globalmap_server_nodelet` | hdl_localization | GlobalmapServerNodelet | Load PCD map for NDT |
| 6 | `hdl_localization_nodelet` | hdl_localization | HdlLocalizationNodelet | NDT scan-to-map localization |
| 7 | `real_robot_transform` | bot_sim | real_robot_transform | TF aft_mapped → gimbal_frame |
| 8 | `dstarlite` | bot_sim | dstarlite | D*-Lite planner + PID |
| 9 | `cmd_vel_marker` | bot_sim | cmd_vel_marker | Visualize /cmd_vel in RViz |
| 10 | `raw_cmd_vel_marker` | bot_sim | cmd_vel_marker | Visualize raw planner output |
| 11 | `ser2msg_decision_givepoint` | bot_sim | ser2msg_decision_givepoint | TF broadcaster map → virtual_frame |
| 12 | `world_align_node` | bot_sim | world_align_node | Coordinate transform map → world |
| 13 | `mcu_communicator` | decision_node | mcu_communicator | MCU serial bridge |
| 14 | `strategy_node` | decision_node | strategy_node | BehaviorTree strategy engine |

**Conditional nodes** (gated by launch args):

| # | Node | Package | Gate | Role |
|---|------|---------|------|------|
| 15 | `rog_map_world_link` | tf2_ros | `enable_rog_map_observer` | Static TF map → world |
| 16 | `rog_map_node` | bot_sim | `enable_rog_map_observer` | Stage 1 ROG-Map observer |
| 17 | `rog_map_to_grid_node` | bot_sim | `enable_rog_costmap` | Stage 2 ROG-Map → grid bridge |

### Data Flow Summary

1. **LiDAR → Localization**: Livox publishes `/livox/lidar` + `/livox/imu` → `imu_filter` removes gravity → `hdl_localization` performs NDT scan-to-map registration, publishes `/odom` and TF `map → aft_mapped`
2. **Localization → Planning**: `real_robot_transform` broadcasts `aft_mapped → gimbal_frame` (gravity-aligned) → `ser2msg_decision_givepoint` broadcasts `map → virtual_frame` (robot nav base) → `dstarlite` uses `virtual_frame` for planning
3. **Goal → Planning**: `strategy_node` publishes goals to `/clicked_point` → `dstarlite` computes D*-Lite path, applies PID velocity tracking, publishes `/cmd_vel`
4. **Planning → MCU**: `mcu_communicator` receives `/cmd_vel` and `/dstar_status`, sends navigation commands over serial to the physical MCU

---

## Parameter Configuration

### IMU Filter (`/imu_filter/`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_imu_topic` | `/livox/imu` | Raw IMU input topic |
| `output_imu_topic` | `/livox/imu_filtered` | Gravity-corrected output |
| `gravity` | `9.81` | Gravity magnitude (m/s²) |

### D*-Lite Planner (`/dstarlite/`)

**Core:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_topic_name` | `/map` | Static map topic |
| `map_frame_name` | `map` | Map reference frame |
| `robot_frame_name` | `virtual_frame` | Robot base frame |
| `dynamic_map_topic_name` | `/grid` | Dynamic obstacle map topic |
| `goal_topic_name` | `/clicked_point` | Navigation goal topic |
| `control_rate_hz` | `10.0` | Control loop rate (Hz) |
| `cmd_vel_tf_timeout` | `0.3` | TF lookup timeout (s) |

**Edge Cost (sigmoid on obstacle probability):**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `x0_grid` | `70` | Sigmoid inflection point |
| `k_grid` | `0.2` | Sigmoid steepness |
| `L_grid` | `80` | Max cost multiplier |
| `x0_velocity` | `130` | Velocity sigmoid inflection |
| `k_velocity` | `-0.25` | Velocity sigmoid steepness |
| `L_velocity` | `1.5` | Max linear velocity (m/s) |

**Goal Approach:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `start_decrease_dis` | `2.0` | Start decelerating at this distance (m) |
| `min_velocity_rate` | `0.2` | Min velocity ratio near goal |

**PID Velocity Tracking:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `pid_enabled` | `true` | Enable PID correction |
| `pid_feedforward` | `0.6` | Feedforward gain |
| `pid_kp_x / pid_kp_y` | `0.8` | Proportional gain |
| `pid_ki_x / pid_ki_y` | `0.5` | Integral gain |
| `pid_kd_x / pid_kd_y` | `0.2` | Derivative gain |
| `pid_integral_limit` | `1.5` | Integral windup limit |
| `pid_output_limit` | `3.0` | Output magnitude limit (m/s) |
| `pid_accel_limit` | `4.0` | Acceleration limit (m/s²) |
| `pid_deadband` | `0.01` | Output deadband (m/s) |
| `pid_feedback_timeout` | `0.3` | Velocity feedback timeout (s) |
| `pid_reset_on_stop` | `true` | Reset integrator on zero command |

**Slope Boost (disabled by default):**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `slope_boost_enabled` | `false` | Enable uphill velocity boost |
| `slope_boost_start_deg` | `4.0` | Min slope to activate (deg) |
| `slope_boost_full_deg` | `25.0` | Slope for full boost (deg) |
| `slope_boost_max_scale` | `1.6` | Max velocity scale factor |
| `slope_boost_uphill_sign` | `-1.0` | Sign convention for uphill |

### hdl_localization (NDT)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `points_topic` | `livox/lidar` | Input point cloud (remapped) |
| `odom_child_frame_id` | `aft_mapped` | Odometry child frame |
| `use_imu` | `true` | Enable IMU fusion |
| `invert_imu_acc` | `true` | Invert IMU acceleration |
| `imu_topic` | `/livox/imu_filtered` | IMU input topic |
| `reg_method` | `NDT_OMP` | Registration method |
| `ndt_resolution` | `1.0` | NDT voxel resolution (m) |
| `ndt_neighbor_search_radius` | `0.5` | NDT search radius (m) |
| `downsample_resolution` | `0.1` | Point cloud downsample (m) |
| `globalmap_pcd` | `$(find hdl_graph_slam)/map/innowing.pcd` | Reference PCD map |
| `cool_time_duration` | `2.0` | IMU cooldown after startup (s) |

### ROG-Map Stage 1 (Passive Observer, `/rog_map_node/rog_map/`)

Loaded from `rog_map_passive.yaml`. Only active when `enable_rog_map_observer:=true`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `resolution` | `0.1` | Voxel resolution (m/cell) |
| `map_size` | `[30, 30, 8]` | Map dimensions [x, y, z] (m) |
| `virtual_ceil_height` | `5.0` | Ceiling cutoff (m) |
| `virtual_ground_height` | `-1.0` | Floor cutoff (m) |
| `map_sliding.threshold` | `0.1` | Sliding window trigger distance (m) |
| `ros_callback.cloud_topic` | `/aligned_points` | Input point cloud (hdl_localization output) |
| `ros_callback.odom_topic` | `/odom` | Odometry input |
| `raycasting.ray_range` | `[0.3, 12.0]` | Raycast range [min, max] (m) |
| `raycasting.local_update_box` | `[14, 14, 8]` | Local update region (m) |
| `raycasting.p_hit / p_miss` | `0.70 / 0.20` | Hit / miss probability |
| `raycasting.p_occ / p_free` | `0.65 / 0.30` | Occupied / free threshold |
| `visualization.enable` | `true` | Publish voxel visualization |

### ROG-Map Stage 2 (Grid Bridge, `/rog_map_to_grid_node/`)

Only active when `enable_rog_costmap:=true`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `z_min` | `0.05` | Projection floor (m) |
| `z_max` | `2.0` | Projection ceiling (m) |
| `res` | `0.05` | Output grid resolution (must match /map) |
| `window_m` | `10.0` | Robot-centric window half-size (m) |
| `voxel_lattice` | `0.2` | Voxel subsampling step (m) |
| `soft_rim_cells` | `2` | Inflation cells around obstacles |
| `soft_rim_value` | `60` | Occupancy value for inflated rim (0-100) |
| `cloud_timeout` | `0.5` | Max cloud age before all-zero grid (s) |
| `startup_wait_s` | `5.0` | waitForMessage timeout at startup (s) |

### Strategy Node (`/strategy_node/`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tick_hz` | `20` | BehaviorTree tick rate (Hz) |
| `danger_hp` | `200` | HP threshold for danger mode |
| `max_hp` | `380` | Maximum HP reference |
| `sufficient_bullet` | `150` | Sufficient bullet count |
| `max_bullet` | `750` | Maximum bullet capacity |
| `fixed_supply` | `50` | Bullets per supply visit |
| `attack_threshold` | `30` | Attack distance threshold |
| `harm_threshold_on` | `50` | Harm level to trigger evasion |
| `harm_threshold_off` | `10` | Harm level to stop evasion |
| `bt_xml` | `$(find decision_node)/config/vision_test.xml` | BehaviorTree XML file |

### ser2msg_decision_givepoint (`/ser2msg_decision_givepoint/`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `virtual_frame` | `virtual_frame` | Robot navigation frame |
| `rotbase_frame` | `rotbase_frame` | Rotation base frame |
| `gimbal_frame` | `gimbal_frame` | Gimbal frame |
| `_3DLidar_frame` | `aft_mapped` | LiDAR frame |
| `delta_time` | `0.04` | Serial poll interval (s) |
| `vel_topic` | `/cmd_vel` | Velocity command topic |
| `theta` | `1.572418` | Map alignment rotation (rad) |
| `shift_x` | `15.652537` | Map alignment X shift (m) |
| `shift_y` | `3.212167` | Map alignment Y shift (m) |
| `K` | `2.5` | Gain factor |

### real_robot_transform (`/real_robot_transform/`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gimbal_frame` | `gimbal_frame` | Gimbal frame ID |
| `_3DLidar_frame` | `aft_mapped` | 3D LiDAR frame |
| `odom_frame` | `map` | Odometry/map frame |
| `roll_offset_deg` | `0.0` | Roll calibration offset |
| `pitch_offset_deg` | `0.0` | Pitch calibration offset |
| `yaw_offset_deg` | `0.0` | Yaw calibration offset |

### world_align_node (`/world_align_node/`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_topic` | `/sentinel_nav_position` | Nav frame position input |
| `output_topic` | `/sentinel_world_position` | World frame position output |
| `target_world_frame` | `world` | Output frame |
| `source_map_frame` | `map` | Input frame |
| `publish_world_to_map_tf` | `false` | Publish world → map TF |
| `use_ref_points` | `true` | Auto-solve transform from reference points |

### MCU Communicator (`/mcu_communicator/`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Serial device |
| `baudrate` | `921600` | Serial baud rate |
| `nav_frequency` | `50` | Nav command frequency (Hz) |

---

## ROS Topic Communication

### Sensor Domain

| Publisher | Topic | Type | Subscriber |
|-----------|-------|------|------------|
| livox_lidar_publisher2 | `/livox/lidar` | PointCloud2 | hdl_localization, rog_map_node (opt.) |
| livox_lidar_publisher2 | `/livox/imu` | Imu | imu_filter |
| imu_filter | `/livox/imu_filtered` | Imu | hdl_localization |

### Localization Domain

| Publisher | Topic | Type | Subscriber |
|-----------|-------|------|------------|
| hdl_localization | `/odom` | Odometry | dstarlite (via TF), rog_map_node, rog_map_to_grid_node, strategy_node |
| hdl_localization | `/aligned_points` | PointCloud2 | rog_map_node (Stage 1) |

### Planning Domain

| Publisher | Topic | Type | Subscriber |
|-----------|-------|------|------------|
| map_server | `/map` | OccupancyGrid | dstarlite |
| rog_map_to_grid_node | `/rog_grid` | OccupancyGrid | dstarlite (when costmap_source=rog) |
| strategy_node | `/clicked_point` | PointStamped | dstarlite |
| ser2msg_decision_givepoint | `/clicked_point` | PointStamped | dstarlite |
| dstarlite | `/cmd_vel` | Twist | mcu_communicator, cmd_vel_marker, ser2msg_decision_givepoint |
| dstarlite | `/dstarlite/raw_cmd_vel` | Twist | raw_cmd_vel_marker |
| dstarlite | `/dstar_path` | Path | (RViz visualization) |
| dstarlite | `/dstar_status` | Bool | mcu_communicator, ser2msg_decision_givepoint, strategy_node |
| dstarlite | `/all_map_status` | OccupancyGrid | (RViz visualization) |

### MCU / Referee Domain

All published by `mcu_communicator` (parsed from MCU serial frames), subscribed by `strategy_node`:

| Topic | Type |
|-------|------|
| `/mcu/yaw_angle` | Float32 |
| `/mcu/chassis_imu` | Float32 |
| `/referee/game_progress` | UInt8 |
| `/referee/stage_remain_time` | UInt16 |
| `/referee/ally_base_hp` | UInt16 |
| `/referee/enemy_base_hp` | UInt16 |
| `/referee/ally_1_robot_HP` through `ally_4_robot_HP` | UInt16 |
| `/referee/ally_outpost_hp` | UInt16 |
| `/referee/central_ground_status` | UInt8 |
| `/referee/trap_ground_status` | UInt8 |
| `/referee/fortress_status` | UInt8 |
| `/referee/outpost_status` | UInt8 |
| `/referee/projectile_17mm` | UInt16 |
| `/referee/projectile_fortress` | UInt16 |
| `/referee/remaining_gold` | UInt16 |
| `/referee/accumulated_bullet` | UInt16 |
| `/referee/can_exchange_respawn` | Bool |
| `/referee/respawn_money` | UInt16 |
| `/referee/out_of_combat` | Bool |
| `/referee/projectile_allowance` | UInt16 |
| `/referee/power_rune_available` | Bool |
| `/referee/operator` | Point |
| `/referee/supplement_resource` | Bool |
| `/referee/supplement_nonresource` | Bool |
| `/robot/robot_id` | UInt8 |
| `/robot/self_hp` | UInt16 |
| `/enemy/hero_position` | Point |
| `/enemy/engineer_position` | Point |
| `/enemy/standard_3_position` | Point |
| `/enemy/standard_4_position` | Point |
| `/enemy/sentry_position` | Point |
| `/radar/suggested_target` | UInt8 |
| `/radar/radar_flags` | UInt16 |

### Decision → MCU Commands

| Publisher | Topic | Type | Subscriber |
|-----------|-------|------|------------|
| strategy_node | `/motion` | UInt8 | mcu_communicator |
| strategy_node | `/spin` | UInt8 | mcu_communicator |
| strategy_node | `/spin_velo` | UInt8 | (published) |
| strategy_node | `/recover` | UInt8 | (published) |
| strategy_node | `/bullet_num` | UInt8 | (published) |
| strategy_node | `/target_yaw` | Float32 | mcu_communicator |
| strategy_node | `/activate_power_rune` | UInt8 | mcu_communicator |
| strategy_node | `/exchange_respwan` | UInt8 | mcu_communicator |

### Coordinate Domain

| Publisher | Topic | Type | Subscriber |
|-----------|-------|------|------------|
| ser2msg_decision_givepoint | `/sentinel_nav_position` | PointStamped | world_align_node |
| world_align_node | `/sentinel_world_position` | PointStamped | (strategy consumers) |

---

## TF Tree

```
world  (ROG-Map viz frame, identity TF from map)
  │  [static_transform_publisher: (0,0,0,0,0,0) map → world]
map  (navigation origin, static map frame)
  │  [ser2msg_decision_givepoint: chain through gimbal→rotbase→virtual]
  │  [world_align_node (optional): world → map]
virtual_frame  (robot navigation base, used by D*-Lite as robot_frame)
  │
rotbase_frame  (intermediate frame, 0.35 m below gimbal)
  │
gimbal_frame  (gimbal / pitch frame)
  │  [real_robot_transform: gravity-aligned rotation, zero translation]
aft_mapped  (LiDAR sensor frame, frame_id of /livox/lidar)
  │  [hdl_localization: NDT odometry]
odom  (= map in this config)
```

### Frame Details

| Frame | Parent | Broadcaster | Description |
|-------|--------|-------------|-------------|
| `world` | `map` | static_transform_publisher (identity) | ROG-Map visualization frame. Hardcoded in rog_map source. Identity TF to `map` so RViz (fixed_frame=map) can render ROG voxels. |
| `map` | — (root) | — | Navigation origin, same as PGM map origin. Static. |
| `virtual_frame` | `map` | ser2msg_decision_givepoint | Robot navigation base. Computed by composing: map→gimbal_frame→rotbase_frame→virtual_frame. D*-Lite uses this as `robot_frame_name`. |
| `rotbase_frame` | `gimbal_frame` | (composed in ser2msg) | Intermediate rotation base. 0.35 m below gimbal_frame. |
| `gimbal_frame` | `aft_mapped` | real_robot_transform | Gimbal frame. Zero translation, gravity-aligned rotation. Extracts roll/pitch from aft_mapped→map TF, zeroes yaw, inverts. |
| `aft_mapped` | `map` (via /odom) | hdl_localization | LiDAR sensor frame. NDT localization publishes odometry from this frame to map. |
| `odom` | = `map` | — | hdl_localization uses `odom_child_frame_id=aft_mapped`; odom parent = map in this config. |

**Key notes:**

- `odom` frame is identical to `map` in this configuration — hdl_localization publishes `/odom` expressing `aft_mapped` pose in the `map` frame.
- `real_robot_transform` removes yaw from the lidar-to-map transform, keeping only gravity-aligned roll/pitch. This prevents LiDAR pitch/roll from being misinterpreted as robot body tilt.
- `ser2msg_decision_givepoint` publishes only the composed `map → virtual_frame` TF. Intermediate frames (`gimbal→rotbase`, `rotbase→virtual`) are NOT broadcast individually.
- The `world` frame exists solely because rog_map hardcodes "world" as its visualization frame_id. The static TF publisher provides an identity link to `map`.

---

## Git Submodules

| Path | Remote | Branch | Purpose |
|------|--------|--------|---------|
| `livox_ws/src/livox_ros_driver2` | github.com/fffcr/livox_ros_driver2 | `parameter_RMUL2026_gimbal` | Livox MID360 ROS driver (HKU fork) |
| `DecisionNode` | github.com/HKUAstar/DecisionNode | `RMUC2026` | BehaviorTree strategy + MCU communication |
| `Navigation-filter-test` | github.com/HKUAstar/Navigation-filter-test | `RMUC2026` | Optional filter testing (not launched) |

```bash
git submodule update --init --recursive
```

---

## Files Excluded from This Pipeline

The repository contains additional files not part of the `3DNavUL_Test_with_decision.launch` pipeline:

| File / Directory | Reason |
|------------------|--------|
| `nav_esdf.launch` | Alternative ESDF-based planner entry point (separate pipeline) |
| `3DSlamFinal_lio.launch` | SLAM mapping with Point-LIO (offline) |
| `hdl_graph_slam_mapping.launch` | SLAM mapping with HDL Graph SLAM (offline) |
| `dbscan_bfs_3D.cpp` / `dbscan_bfs_3D.launch.backup` | Legacy DBSCAN clustering (disabled, commented out) |
| `dwa.cpp` / `dwa_optimizer.launch.backup` | DWA local planner (not launched; D*-Lite publishes cmd_vel directly) |
| `config/arbotix/` | Unused robot controller config |
| `config/teb_local_planner_params.yaml` | TEB planner config (not launched) |
| `sim_nav/src/Point-LIO/` | Alternative SLAM frontend (not in this pipeline) |
| `sim_nav/src/fast_gicp/` | Registration backend (build dependency only) |
| `sim_nav/src/ndt_omp/` | Registration backend (build dependency only) |
| `sim_nav/src/dm_imu/` | Separate IMU driver (not launched) |
| `Navigation-filter-test/` | Optional submodule (not launched) |
