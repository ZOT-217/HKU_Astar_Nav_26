# Super Detailed Navigation Pipeline Report for Old_nav

Generated on 2026-05-04 for `/home/sentry/AstarTraining/Old_nav`.

This report explains the active navigation pipeline in plain engineering language. It is written for a new teammate who needs to understand how sensor data becomes robot motion, where goals enter the system, which frames matter, and where to debug when the robot does not move correctly.

## 1. One-Screen Summary

The active `Old_nav` navigation stack does this:

```text
Livox MID360 LiDAR and IMU
        |
        v
Livox ROS driver
        |
        +--> /livox/lidar ----------------------+
        |                                       |
        +--> /livox/imu -> imu_filter           |
                           -> /livox/imu_filtered
                                                |
                                                v
                                hdl_localization with NDT_OMP
                                                |
                                                v
                                      TF map -> aft_mapped
                                                |
                                                v
                                real_robot_transform
                                                |
                                                v
                                  TF aft_mapped -> gimbal_frame
                                                |
                                                v
                 ser2msg_decision_givepoint plus world_align
                                                |
                                                v
                         TF gimbal_frame -> rotbase_frame -> virtual_frame
                                                |
                                                v
                     D*-Lite planner reads robot pose from TF
                                                |
/map from map_server --------------------------+
/clicked_point goal ---------------------------+
optional dynamic costmap ----------------------+
                                                |
                                                v
                                 raw desired velocity
                                                |
                                                v
                                   PID velocity tracking
                                                |
                                                v
                                           /cmd_vel
```

In the full robot launch, `/cmd_vel` is then consumed by the MCU communication node outside `Old_nav`. Inside `Old_nav`, the planner and visualization are the important consumers/producers.

The navigation planner is not `move_base`. It is a custom D*-Lite implementation in `bot_sim`.

## 2. Active Entrypoints

### 2.1 Production wrapper

File:

```text
run_3DNavUL_Test_with_decision.sh
```

This shell wrapper is the production-style startup script. It:

1. Changes working directory to `Old_nav`.
2. Creates and writes logs under `Old_nav/logs`.
3. Exports ROS environment variables.
4. Sources these workspaces:
   - `/opt/ros/noetic/setup.bash`
   - `Old_nav/livox_ws/devel/setup.bash`
   - `Old_nav/sim_nav/devel/setup.bash`
   - `Old_nav/Navigation-filter-test/devel/setup.bash`
   - external `DecisionNode/devel/setup.bash` if present
5. Waits briefly for `/dev/ttyUSB0`.
6. Starts `roscore` if no master is available.
7. Starts continuous `rosbag record -a -O ... --lz4`.
8. Rotates rosbag files every 60 seconds.
9. Launches:

```text
3DNavUL_Test_with_decision.launch
```

Important operator note: even if `/dev/ttyUSB0` is missing, the script continues. That is useful for boot robustness, but it means nav can start even when the MCU bridge has no serial device yet.

### 2.2 Full robot launch aggregator

File:

```text
3DNavUL_Test_with_decision.launch
```

This launch file includes three pieces:

1. `Old_nav/3DNavUL_Test.launch` for navigation.
2. External `mcu_communicator.launch` for serial bridge.
3. External `strategy_decision_tmp.launch` for BehaviorTree decisions.

From the `Old_nav` viewpoint, this file is the bridge between nav and the rest of the robot. It does not implement logic; it assembles launch files.

### 2.3 Nav-only launch

File:

```text
3DNavUL_Test.launch
```

This is the main navigation launch. It includes these components in order:

1. `bot_sim/launch_real/map_server.launch`
2. `bot_sim/launch_real/imu_filter.launch`
3. `livox_ros_driver2/launch_ROS1/rviz_MID360.launch`
4. `hdl_localization/launch/hdl_localization.launch`
5. `bot_sim/launch_real/real_robot_transform.launch`
6. Optional ROG-Map observer.
7. Optional ROG-Map costmap bridge.
8. `bot_sim/launch_real/dstarlite.launch`
9. `bot_sim/launch_real/ser2msg_tf_decision_givepoint.launch`

The important behavior is that localization and TF are brought up before D*-Lite needs a robot pose.

## 3. Package Roles

### 3.1 `livox_ws`

Purpose:

- Talks to the Livox MID360.
- Publishes point cloud and IMU data.

Important topics:

```text
/livox/lidar
/livox/imu
```

The launch used by nav is:

```text
livox_ros_driver2/launch_ROS1/rviz_MID360.launch
```

### 3.2 `sim_nav/src/hdl_localization`

Purpose:

- Performs online localization against a pre-built 3D PCD map.
- Uses NDT_OMP scan matching.
- Publishes odometry and TF.

Important file:

```text
sim_nav/src/hdl_localization/launch/hdl_localization.launch
```

Important input topics:

```text
livox/lidar
/livox/imu_filtered
```

Important output:

```text
TF map -> aft_mapped
/odom
```

The global map loaded by localization is:

```text
$(find hdl_graph_slam)/map/innowing.pcd
```

### 3.3 `sim_nav/src/bot_sim`

Purpose:

- Owns most of the active navigation glue.
- Runs map server launch.
- Filters IMU.
- Bridges TF frames.
- Runs D*-Lite planner.
- Publishes `/cmd_vel`.
- Visualizes `/cmd_vel`.

Key executables in the active path:

```text
imu_filter
real_robot_transform
dstarlite
cmd_vel_marker
ser2msg_decision_givepoint
world_align_node
```

Optional or experimental path:

```text
rog_map_node
rog_map_to_grid_node
dstarlite_esdf
```

## 4. Maps

The navigation stack uses two map representations.

### 4.1 2D occupancy grid for planning

Launch:

```text
sim_nav/src/bot_sim/launch_real/map_server.launch
```

Default map:

```text
sim_nav/src/bot_sim/map/innowing.yaml
```

This creates `/map`, a standard ROS `nav_msgs/OccupancyGrid`.

D*-Lite uses this map as the static obstacle layer. Static obstacles are expanded into a soft cost field during planner initialization. Hard occupied cells start at occupancy 100. Neighboring cells get decreasing obstacle possibility through a breadth-first propagation.

### 4.2 3D PCD map for localization

Launch:

```text
sim_nav/src/hdl_localization/launch/hdl_localization.launch
```

Default PCD:

```text
hdl_graph_slam/map/innowing.pcd
```

This map is loaded by `globalmap_server_nodelet`, downsampled, and used by `hdl_localization_nodelet` for NDT scan matching.

The planner does not directly use the PCD. The PCD gives pose. The 2D occupancy grid gives planning cost.

## 5. Sensor Pipeline

### 5.1 LiDAR point cloud

```text
Livox MID360 -> livox_ros_driver2 -> /livox/lidar
```

`/livox/lidar` goes to `hdl_localization` as the live scan. In optional ROG-Map mode it can also feed the local ESDF map.

### 5.2 IMU data

```text
Livox MID360 -> /livox/imu -> imu_filter -> /livox/imu_filtered
```

File:

```text
sim_nav/src/bot_sim/src/imu_filter.cpp
```

Launch:

```text
sim_nav/src/bot_sim/launch_real/imu_filter.launch
```

The filter does three things:

1. Copies the incoming IMU message.
2. Optionally changes the frame id.
3. Applies sign/gravity scaling and a fixed -20 degree x-axis rotation to linear acceleration and angular velocity.

The output is:

```text
/livox/imu_filtered
```

`hdl_localization` subscribes to this filtered IMU.

## 6. Localization Pipeline

File:

```text
sim_nav/src/hdl_localization/launch/hdl_localization.launch
```

Nodes/nodelets:

1. `velodyne_nodelet_manager`
2. `globalmap_server_nodelet`
3. `hdl_localization_nodelet`

Important parameters:

| Param | Value | Meaning |
|---|---:|---|
| `points_topic` | `livox/lidar` | Live point cloud input. |
| `imu_topic` | `/livox/imu_filtered` | Filtered Livox IMU input. |
| `odom_child_frame_id` | `aft_mapped` | Child frame of localization output. |
| `use_imu` | `true` | Use IMU in localization. |
| `reg_method` | `NDT_OMP` | Registration method. |
| `ndt_neighbor_search_method` | `DIRECT7` | Fast NDT neighbor search. |
| `ndt_resolution` | `1.0` | NDT grid resolution. |
| `downsample_resolution` | `0.1` | Cloud downsample resolution. |
| `use_global_localization` | `false` | Global relocalization disabled by default. |

Hard-coded initial pose:

```text
position = (10.107939720153809, -4.187480926513672, 0.0)
orientation = (w=0.05464960747042481, x=0, y=0, z=-0.9985055935763848)
```

Primary output:

```text
TF map -> aft_mapped
```

This transform is the root pose source used later by the planner.

## 7. TF Frame Pipeline

The navigation stack is very frame-sensitive. The simplified active chain is:

```text
map
  -> aft_mapped
      -> gimbal_frame
          -> rotbase_frame
              -> virtual_frame
```

### 7.1 `map`

The global frame. Static occupancy grid, PCD map, localization output, and planner path are all expressed relative to this frame.

### 7.2 `aft_mapped`

The child frame from `hdl_localization`. This is the localized LiDAR/odometry frame.

Source:

```text
hdl_localization_nodelet
```

### 7.3 `gimbal_frame`

Produced by:

```text
real_robot_transform
```

Files:

```text
sim_nav/src/bot_sim/src/real_robot_transform.cpp
sim_nav/src/bot_sim/launch_real/real_robot_transform.launch
```

This node listens to the localization transform and publishes:

```text
aft_mapped -> gimbal_frame
```

Its job is to normalize the LiDAR/gimbal relationship so later nodes can reason about chassis direction separately from localization.

### 7.4 `rotbase_frame`

Produced by:

```text
ser2msg_decision_givepoint
```

This node consumes MCU yaw-related topics and publishes:

```text
gimbal_frame -> rotbase_frame
```

The rotation accounts for the gimbal relative yaw. In the current source, serial reading inside this node is disabled; it consumes ROS topics instead.

Important inputs:

```text
/mcu/yaw_angle
/mcu/chassis_imu
```

### 7.5 `virtual_frame`

Also produced by `ser2msg_decision_givepoint` and `world_align` logic.

Launch parameters:

```text
theta   = 1.572418
shift_x = 15.652537
shift_y = 3.212167
K       = 2.5
```

The planner uses:

```text
robot_frame_name = virtual_frame
```

That means D*-Lite asks TF for the pose of `virtual_frame` in `map` and treats that as the robot position.

## 8. Goal Pipeline

The planner subscribes to:

```text
/clicked_point
```

Message type:

```text
geometry_msgs/PointStamped
```

In full robot mode, the external decision node publishes tactical goals to this topic. In nav-only testing, a developer can publish to `/clicked_point` manually or from RViz-like tools.

Inside D*-Lite:

1. The goal point is converted from meters to map grid cells.
2. The current robot pose is read from TF.
3. The robot pose is converted to a start grid cell.
4. If start or goal is outside the map, the goal is rejected.
5. If accepted, D*-Lite state is reset.
6. The goal node gets `rhs=0` and is inserted into the D*-Lite open set.

Important behavior:

- A repeated identical goal is ignored.
- A new valid goal clears the old planner graph state.
- Arrival is checked by comparing current map-frame robot position against the final goal cell position with a 0.25 m x/y threshold.

## 9. D*-Lite Planner Internals

Main file:

```text
sim_nav/src/bot_sim/src/dstarlite.cpp
```

Launch:

```text
sim_nav/src/bot_sim/launch_real/dstarlite.launch
```

### 9.1 Inputs

| Input | Default | Type | Role |
|---|---|---|---|
| Static map | `/map` | `nav_msgs/OccupancyGrid` | Base map loaded at startup. |
| Dynamic map | `/grid` | `nav_msgs/OccupancyGrid` | Optional dynamic obstacles. |
| Goal | `/clicked_point` | `geometry_msgs/PointStamped` | Desired target. |
| Robot pose | TF `map -> virtual_frame` | TF | Current pose. |

### 9.2 Outputs

| Output | Type | Role |
|---|---|---|
| `/dstar_path` | `nav_msgs/Path` | Planned path visualization/debug. |
| `/all_map_status` | `nav_msgs/OccupancyGrid` | Planner's internal obstacle possibility map. |
| `/cmd_vel` | `geometry_msgs/Twist` | Final velocity command. |
| `/dstar_status` | `std_msgs/Bool` | Arrival flag. |
| `/dstarlite/raw_cmd_vel` | `geometry_msgs/Twist` | Pre-PID desired command telemetry. |

### 9.3 Static obstacle expansion

When the planner receives the static map at startup, it creates a grid of `Node` objects. Occupied cells initialize with obstacle possibility 100. Then nearby cells receive decaying obstacle possibility through an 8-neighbor propagation.

This creates a soft cost field around walls. The planner does not only avoid hard walls; it also prefers paths farther away from obstacles.

### 9.4 Edge cost

The planner uses sigmoid-shaped edge costs.

Parameters from launch:

```text
x0_grid = 70
k_grid  = 0.2
L_grid  = 80
```

The cost increases sharply around higher obstacle possibility. This encourages D*-Lite to choose routes away from inflated cells.

### 9.5 Dynamic obstacle updates

Classic D*-Lite can subscribe to a dynamic occupancy grid. In the launch file this defaults to:

```text
/grid
```

The parent launch can switch the planner to consume ROG-Map bridge output instead:

```text
costmap_source = rog
dynamic_map_topic_name = /rog_grid
```

The dynamic map callback:

1. Reads dynamic map dimensions, resolution, origin, and frame id.
2. Looks up the transform from the dynamic map frame to `map`.
3. Converts each dynamic cell into the static map grid.
4. Updates obstacle possibility when it changed meaningfully.
5. Calls `dstar_update_node()` only for changed cells.
6. Decays stale dynamic cells back to static map values.

Important safety update:

- If the dynamic map TF lookup fails, the dynamic update is skipped. The planner no longer continues with a stale transform.

### 9.6 Path maintenance

D*-Lite tracks consistency with:

```text
dis_to_goal
rhs
succ
```

It also uses a link-cut tree to quickly check whether the current start node is connected to the goal through successor links.

The planner avoids full re-planning when the old path still works:

```text
old_path_still_work(start_x, start_y)
```

If the old path is still valid, it returns early.

### 9.7 Lookahead point

When publishing velocity, D*-Lite does not aim only at the immediate next grid cell. It walks up to 12 successor steps ahead:

```text
cur -> succ -> succ -> ... up to 12 steps
```

That lookahead target creates a smoother direction than chasing one cell at a time.

### 9.8 Velocity calculation

Launch parameters:

```text
x0_velocity       = 130
k_velocity        = -0.25
L_velocity        = 1.5
start_decrease_dis = 2
min_velocity_rate  = 0.2
```

The planner calculates speed from obstacle possibility and distance to goal. Close to the goal, it scales speed down toward `min_velocity_rate`.

Important fixed behavior:

- The code now returns the calculated velocity instead of always returning `L_velocity`.

That means the approach slowdown now actually affects `/cmd_vel`.

## 10. PID Velocity Tracking in the Current Pipeline

PID now sits inside the D*-Lite publish path.

### 10.1 Before PID

```text
D*-Lite path direction -> transformed body-frame command -> /cmd_vel
```

### 10.2 After PID

```text
D*-Lite path direction
  -> transformed body-frame desired command
  -> /dstarlite/raw_cmd_vel
  -> PID velocity tracking
  -> /cmd_vel
```

### 10.3 What PID controls

PID controls linear velocity components:

```text
linear.x
linear.y
```

It preserves:

```text
angular.z
```

The current D*-Lite command path mostly uses x/y velocity. Angular control should not be expanded until chassis yaw semantics are validated against gimbal and virtual frames.

### 10.4 What PID uses as feedback

PID estimates velocity from TF:

```text
measured_vx_map = delta_x_map / dt
measured_vy_map = delta_y_map / dt
```

Then it rotates that map-frame velocity into the robot frame:

```text
robot_frame_name = virtual_frame
```

So the PID error is computed in the same frame as the desired command.

### 10.5 Why raw command telemetry matters

The new raw topic lets you answer this question quickly:

```text
Did the planner command something bad, or did PID change it badly?
```

If `/dstarlite/raw_cmd_vel` is already wrong, debug planner, TF, or map. If raw is good but `/cmd_vel` is wrong, tune or disable PID.

## 11. Arrival and Stop Behavior

D*-Lite publishes arrival on:

```text
/dstar_status
```

When the robot is within 0.25 m in both x and y from the final goal cell position:

1. `have_first_goal=false`
2. `publish_stop()` sends zero velocity
3. `/dstar_status` publishes true once
4. PID state is reset

When not arrived:

1. D*-Lite updates path if needed.
2. D*-Lite publishes path and velocity.
3. `/dstar_status` publishes false.

When there is no active goal:

1. The planner repeatedly publishes stop.
2. PID remains reset.

## 12. Optional ROG-Map and ESDF Paths

The current `3DNavUL_Test.launch` includes optional ROG-Map stages controlled by args:

```text
enable_rog_map_observer = false
enable_rog_costmap      = false
costmap_source          = dbscan
```

### 12.1 Stage 1 observer

Launch:

```text
sim_nav/src/bot_sim/launch_real/rog_map_observability.launch
```

Purpose:

- Run ROG-Map passively for visualization/observability.
- Does not control the planner by default.

### 12.2 Stage 2 costmap bridge

Launch:

```text
sim_nav/src/bot_sim/launch_real/rog_map_costmap_bridge.launch
```

Purpose:

- Converts ROG-Map style local obstacle information into a dynamic occupancy grid.
- Publishes `/rog_grid`.
- Lets the classic `dstarlite` consume ROG-derived dynamic obstacles.

### 12.3 ESDF planner executable

File:

```text
sim_nav/src/bot_sim/src/dstarlite_esdf.cpp
```

Launch:

```text
sim_nav/src/bot_sim/launch_real/dstarlite_esdf.launch
```

This variant queries the ROG-Map ESDF API directly instead of subscribing to `/grid`. It scans a local radius around the robot, converts distance to obstacle possibility, updates D*-Lite cells, and decays stale dynamic obstacle cells.

Important ESDF parameters:

```text
esdf_robot_radius = 0.6
esdf_scan_radius  = 6.0
```

Important ROG-Map config:

```text
sim_nav/src/bot_sim/config/rog_map_config.yaml
```

The YAML top-level key is `rog_map`, so when it is loaded inside the node private namespace the parameters are available under:

```text
/dstarlite_esdf/rog_map/...
```

or under the actual node name used by the launch.

## 13. Visualization

The planner launches:

```text
cmd_vel_marker
```

Source:

```text
sim_nav/src/bot_sim/src/cmd_vel_marker.cpp
```

Launch parameters:

```text
cmd_vel_topic = /cmd_vel
marker_topic  = /cmd_vel_marker
frame_id      = virtual_frame
```

It publishes a marker arrow and text showing the latest command. If the command is stale, the marker color changes.

This is useful because `/cmd_vel` is otherwise just a topic. The marker makes command direction visible in RViz.

## 14. Full Robot Boundary

Inside `Old_nav`, the final navigation output is:

```text
/cmd_vel
```

In full robot launch, the external MCU communication node subscribes to `/cmd_vel` and sends velocity commands to the STM32 over serial.

The top-level launch also includes the external decision node, which publishes `/clicked_point` goals.

So the boundary between `Old_nav` and the rest of the system is simple:

```text
Decision -> /clicked_point -> Old_nav planner -> /cmd_vel -> MCU bridge
```

## 15. Important Topics

### 15.1 Inputs

| Topic | Type | Producer | Consumer |
|---|---|---|---|
| `/livox/lidar` | point cloud | Livox driver | localization, optional ROG-Map |
| `/livox/imu` | `sensor_msgs/Imu` | Livox driver | `imu_filter` |
| `/livox/imu_filtered` | `sensor_msgs/Imu` | `imu_filter` | `hdl_localization` |
| `/map` | `nav_msgs/OccupancyGrid` | `map_server` | D*-Lite |
| `/clicked_point` | `geometry_msgs/PointStamped` | decision or user | D*-Lite |
| `/grid` | `nav_msgs/OccupancyGrid` | optional legacy dynamic layer | D*-Lite |
| `/rog_grid` | `nav_msgs/OccupancyGrid` | optional ROG bridge | D*-Lite when enabled |
| `/mcu/yaw_angle` | `std_msgs/Float32` | MCU bridge | `ser2msg_decision_givepoint` |
| `/mcu/chassis_imu` | `std_msgs/Float32` | MCU bridge | `ser2msg_decision_givepoint` |

### 15.2 Outputs

| Topic | Type | Producer | Consumer |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | `hdl_localization` | ROG-Map, debug |
| `/dstar_path` | `nav_msgs/Path` | D*-Lite | RViz/debug |
| `/all_map_status` | `nav_msgs/OccupancyGrid` | D*-Lite | RViz/debug |
| `/cmd_vel` | `geometry_msgs/Twist` | D*-Lite after PID | MCU bridge, marker |
| `/dstarlite/raw_cmd_vel` | `geometry_msgs/Twist` | D*-Lite before PID | debug/tuning |
| `/cmd_vel_marker` | `visualization_msgs/Marker` | `cmd_vel_marker` | RViz |
| `/dstar_status` | `std_msgs/Bool` | D*-Lite | decision, MCU bridge |

## 16. Expected Rates and Timing

Approximate runtime expectations:

| Component | Expected behavior |
|---|---|
| Livox driver | Publishes LiDAR and IMU continuously. |
| `imu_filter` | Callback-driven by `/livox/imu`. |
| `hdl_localization` | Scan matching around LiDAR frame rate. |
| `real_robot_transform` | Publishes TF at 20 Hz. |
| `dstarlite` main loop | Runs at 100 Hz in source. |
| `cmd_vel_marker` | Publishes marker at 15 Hz by default. |
| MCU bridge in full launch | Sends navigation frames at configured `nav_frequency`, default 50 Hz. |

Important practical note: the planner loop can run at 100 Hz, but meaningful localization updates may arrive slower. PID feedback uses TF timestamp deltas and ignores stale or too-small deltas.

## 17. Common Failure Modes

### 17.1 Robot does not move

Check:

1. Is `/clicked_point` being published?
2. Does D*-Lite accept the goal, or does it print `Out Of Map!!!`?
3. Does TF contain `map -> virtual_frame`?
4. Is `/dstar_path` non-empty?
5. Is `/dstarlite/raw_cmd_vel` non-zero?
6. Is `/cmd_vel` non-zero?
7. Is the MCU bridge connected if running full robot mode?

Interpretation:

- Raw zero and path empty means planner/map/goal issue.
- Raw non-zero but `/cmd_vel` zero means PID or stop safety path.
- `/cmd_vel` non-zero but chassis still does not move means downstream MCU/serial/chassis issue.

### 17.2 Robot drives in wrong direction

Check frames:

```text
map -> aft_mapped
aft_mapped -> gimbal_frame
gimbal_frame -> rotbase_frame
rotbase_frame -> virtual_frame
```

Most wrong-direction failures are frame failures, not D*-Lite failures.

Also compare:

```text
/dstarlite/raw_cmd_vel
/cmd_vel
```

If both point wrong, debug TF and world alignment. If only `/cmd_vel` points wrong, debug PID feedback transform.

### 17.3 Planner avoids too much or too little

Tune:

```text
x0_grid
k_grid
L_grid
```

Current classic launch values are intentionally strong:

```text
x0_grid = 70
k_grid  = 0.2
L_grid  = 80
```

These make the planner strongly avoid high-obstacle-probability cells.

### 17.4 Robot arrives too fast or too slow

Tune:

```text
start_decrease_dis
min_velocity_rate
L_velocity
pid_accel_limit
```

Remember that `calculate_velocity()` now actually returns the computed velocity, so these parameters matter.

### 17.5 Repeated TF warnings

If logs show:

```text
ERROR IN MAP TO ROBOT
Velocity PID feedback TF unavailable
```

Debug localization and TF before tuning PID.

Likely causes:

- `hdl_localization` not publishing `map -> aft_mapped`.
- `real_robot_transform` not publishing `aft_mapped -> gimbal_frame`.
- `ser2msg_decision_givepoint` not publishing `gimbal_frame -> rotbase_frame -> virtual_frame`.
- MCU yaw topics unavailable or invalid.

## 18. Debugging Checklist

### 18.1 Start nav-only

```bash
roslaunch /home/sentry/AstarTraining/Old_nav/3DNavUL_Test.launch
```

### 18.2 Verify core topics

```text
/livox/lidar
/livox/imu
/livox/imu_filtered
/map
/odom
/clicked_point
/dstar_path
/cmd_vel
/dstar_status
```

### 18.3 Verify TF chain

The planner needs `map -> virtual_frame`. If this one transform is missing, D*-Lite cannot safely compute robot pose.

### 18.4 Send a simple goal

Use a goal known to be inside the current `innowing.yaml` map. If the goal is outside the map, D*-Lite will reject it.

### 18.5 Compare raw and final velocity

```text
/dstarlite/raw_cmd_vel
/cmd_vel
```

For ESDF planner:

```text
/dstarlite_esdf/raw_cmd_vel
/cmd_vel
```

### 18.6 Watch arrival

```text
/dstar_status
```

It should be false while driving and true once when the goal is reached.

## 19. How to Explain the Pipeline to a New Teammate

Use this version:

1. The LiDAR tells us what the world currently looks like.
2. The localization node compares LiDAR scans to a saved 3D map and tells us where the robot is in `map`.
3. Several TF nodes convert that pose into the chassis-aligned `virtual_frame` that the planner understands.
4. The map server provides a 2D grid for planning.
5. The decision system or operator publishes a goal on `/clicked_point`.
6. D*-Lite searches the grid from the goal backward and maintains a path to the robot.
7. D*-Lite picks a lookahead point on that path and creates a desired velocity.
8. PID compares desired velocity against TF-estimated measured velocity.
9. The corrected command goes out as `/cmd_vel`.
10. The MCU bridge sends `/cmd_vel` to the chassis in full robot mode.

That is the whole loop.

## 20. Current Architecture Strengths

- The active planner is simple to inspect because planning and command generation are in one source file.
- D*-Lite supports incremental updates instead of full re-planning every time.
- Static and dynamic obstacle costs are represented in one internal grid.
- TF separates localization pose from chassis/gimbal alignment.
- Raw command telemetry now makes controller debugging easier.
- PID can be disabled from launch without changing code.

## 21. Current Architecture Risks

- The planner is monolithic, so path search, cost logic, velocity generation, and control are tightly coupled.
- TF correctness is critical. A missing frame breaks downstream behavior.
- Dynamic obstacle behavior depends heavily on frame transforms and map resolution matching.
- The ESDF path uses `AsyncSpinner`, so shared callback state needs care.
- `hdl_global_localization` is available but disabled, so severe localization failure has no automatic global recovery.
- The full launch mixes navigation with external decision and MCU components, so startup race debugging can be noisy.

## 22. Recommended Next Work

1. Add a PID debug message/topic.
2. Add dynamic reconfigure for PID and D*-Lite velocity parameters.
3. Split D*-Lite velocity generation into a helper class.
4. Add an offline bag analysis script for raw command, final command, measured velocity, and goal distance.
5. Add automatic localization health checks based on `hdl_localization` status.
6. Add a relocalization trigger path for scan-match divergence.
7. Create a minimal nav-only test launch with fake goal and recorded bag playback.
8. Document the exact map coordinate conventions for `innowing.yaml` and `innowing.pcd` together.

## 23. File Index

Entrypoints:

```text
run_3DNavUL_Test_with_decision.sh
3DNavUL_Test_with_decision.launch
3DNavUL_Test.launch
```

Localization:

```text
sim_nav/src/hdl_localization/launch/hdl_localization.launch
```

Navigation launch files:

```text
sim_nav/src/bot_sim/launch_real/map_server.launch
sim_nav/src/bot_sim/launch_real/imu_filter.launch
sim_nav/src/bot_sim/launch_real/real_robot_transform.launch
sim_nav/src/bot_sim/launch_real/dstarlite.launch
sim_nav/src/bot_sim/launch_real/dstarlite_esdf.launch
sim_nav/src/bot_sim/launch_real/ser2msg_tf_decision_givepoint.launch
```

Navigation sources:

```text
sim_nav/src/bot_sim/src/imu_filter.cpp
sim_nav/src/bot_sim/src/real_robot_transform.cpp
sim_nav/src/bot_sim/src/ser2msg_decision_givepoint.cpp
sim_nav/src/bot_sim/src/dstarlite.cpp
sim_nav/src/bot_sim/src/dstarlite_esdf.cpp
sim_nav/src/bot_sim/src/cmd_vel_marker.cpp
sim_nav/src/bot_sim/include/bot_sim/velocity_pid_controller.hpp
```

Maps and config:

```text
sim_nav/src/bot_sim/map/innowing.yaml
sim_nav/src/bot_sim/config/rog_map_config.yaml
```

## 24. Final Mental Model

The current navigation system is best understood as four stacked loops:

```text
Localization loop:
  LiDAR + IMU -> hdl_localization -> map/aft_mapped pose

Frame loop:
  aft_mapped -> gimbal_frame -> rotbase_frame -> virtual_frame

Planning loop:
  /map + dynamic obstacles + /clicked_point + virtual_frame pose -> D*-Lite path

Control loop:
  D*-Lite desired velocity -> PID tracking -> /cmd_vel
```

When debugging, start from the top. If localization or TF is wrong, planner and PID behavior will look wrong even if their code is correct.
