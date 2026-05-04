# PID Integration Report for the Old_nav Navigation System

Generated on 2026-05-04 for the active `Old_nav` navigation stack.

Scope: this report only describes files and runtime behavior under `Old_nav`. The top-level combined launch includes the external decision and MCU workspaces, but the PID integration itself lives entirely in `Old_nav/sim_nav/src/bot_sim`.

## 1. Executive Summary

The current navigation controller is a custom D*-Lite planner that directly publishes `/cmd_vel`. Before this change, D*-Lite did three jobs at once:

1. Build and update a grid path from the static map and optional dynamic obstacles.
2. Pick a short lookahead point on the path.
3. Convert the path direction into a chassis velocity command.

PID belongs after step 3, not inside the graph search itself. D*-Lite should remain responsible for deciding where the robot should move. PID should be responsible for making the robot's measured velocity track the desired velocity more smoothly and more accurately.

The integration now adds a reusable header-only controller:

- `sim_nav/src/bot_sim/include/bot_sim/velocity_pid_controller.hpp`

And wires it into both planner executables:

- `sim_nav/src/bot_sim/src/dstarlite.cpp`
- `sim_nav/src/bot_sim/src/dstarlite_esdf.cpp`

Launch parameters were added to:

- `sim_nav/src/bot_sim/launch_real/dstarlite.launch`
- `sim_nav/src/bot_sim/launch_real/dstarlite_esdf.launch`

The final behavior is:

```text
D*-Lite path and speed logic
        |
        v
raw desired body-frame velocity
        |-----------------------> /dstarlite/raw_cmd_vel or /dstarlite_esdf/raw_cmd_vel
        v
VelocityPidController
        |
        v
/cmd_vel final command
        |
        v
MCU bridge or visualization consumer
```

The raw planner command is still observable through the node-private `raw_cmd_vel` topic. The final `/cmd_vel` is PID-corrected when feedback is healthy, and falls back to the raw command when feedback is unavailable.

## 2. Why PID Was Integrated at This Point

The active planner is not `move_base`, TEB, or a standard ROS controller stack. The executable `dstarlite` performs path planning and command generation in one node. Its output goes directly to `/cmd_vel`, which is consumed by the MCU communication bridge in the full robot launch.

That makes the best integration point the final velocity publication step inside `publish()`:

```text
Path nodes -> lookahead vector -> transform into robot frame -> desired Twist -> PID -> /cmd_vel
```

This placement has several advantages:

- It does not disturb D*-Lite's graph search, edge costs, or dynamic re-planning.
- It keeps static and dynamic obstacle avoidance behavior unchanged.
- It works for both the classic `/grid` planner and the ESDF/ROG-Map variant.
- It lets operators compare raw vs corrected command without adding a separate ROS node.
- It can be disabled by setting `pid_enabled=false` in launch files.

PID should not be placed in the MCU bridge for this codebase because the MCU bridge only sees `/cmd_vel`; it does not know the path, the goal state, or the planner's safety state. PID should also not be placed inside the D*-Lite edge cost function because edge cost changes route selection, not velocity tracking.

## 3. Files Changed

### 3.1 New controller header

Path:

```text
sim_nav/src/bot_sim/include/bot_sim/velocity_pid_controller.hpp
```

Purpose:

- Load PID parameters from the planner node's private namespace.
- Estimate measured robot velocity using TF.
- Compare desired body-frame velocity against measured body-frame velocity.
- Apply feedforward plus PID correction on x and y axes.
- Clamp final velocity magnitude.
- Limit acceleration between PID outputs.
- Reset safely on stop, stale feedback, or TF failure.

### 3.2 Classic D*-Lite planner

Path:

```text
sim_nav/src/bot_sim/src/dstarlite.cpp
```

Changes:

- Includes `bot_sim/velocity_pid_controller.hpp`.
- Adds a `VelocityPidController velocity_pid_` member.
- Adds a node-private `raw_cmd_vel` publisher.
- Loads PID params from `~` in the constructor.
- Publishes raw command before PID correction.
- Publishes corrected command on `/cmd_vel`.
- Adds `publish_stop()` to reset PID and publish zero consistently.
- Fixes the old `calculate_velocity()` bug where the computed speed was discarded by `return L1`.
- Adds zero-length path guards.
- Stops safely on TF lookup failure instead of using stale transforms.

### 3.3 D*-Lite ESDF variant

Path:

```text
sim_nav/src/bot_sim/src/dstarlite_esdf.cpp
```

Changes mirror `dstarlite.cpp`, plus:

- The ESDF goal callback now protects `goal_pose_msg` with a mutex because this node uses `ros::AsyncSpinner(2)`.
- `get_goal_info.exchange(false)` avoids repeated processing of the same goal while preserving retry behavior.
- TF failure while processing a new goal re-sets `get_goal_info=true` so the goal is not lost.

### 3.4 Launch files

Paths:

```text
sim_nav/src/bot_sim/launch_real/dstarlite.launch
sim_nav/src/bot_sim/launch_real/dstarlite_esdf.launch
```

Both now expose the same PID parameters.

## 4. Runtime Data Flow After PID Integration

### 4.1 Classic planner path

```text
/map
  -> dstarlite static map cache

/grid or /rog_grid
  -> dynamic obstacle update
  -> D*-Lite incremental re-plan

/clicked_point
  -> goal cell
  -> D*-Lite path
  -> lookahead path direction
  -> raw desired body-frame velocity
  -> VelocityPidController
  -> /cmd_vel
```

### 4.2 ESDF planner path

```text
/map
  -> dstarlite_esdf static map cache

ROG-Map ESDF API
  -> local distance query around robot
  -> obstacle_possibility updates
  -> D*-Lite incremental re-plan

/clicked_point
  -> goal cell
  -> D*-Lite path
  -> lookahead path direction
  -> raw desired body-frame velocity
  -> VelocityPidController
  -> /cmd_vel
```

### 4.3 Feedback path used by PID

The PID controller estimates measured velocity from TF instead of subscribing to odometry twist.

```text
TF map -> virtual_frame
  -> current robot pose in map frame
  -> pose delta / dt
  -> measured velocity in map frame
  -> rotate into virtual_frame
  -> measured body-frame vx, vy
```

The planner already depends on the same TF relationship for robot position, so this is the least invasive feedback source.

## 5. PID Algorithm

For each linear axis:

```text
raw_error = desired_velocity - measured_velocity

control_error = 0 if abs(raw_error) < pid_deadband
              = raw_error otherwise

integral = clamp(integral + control_error * dt, +/- pid_integral_limit)

derivative = (raw_error - previous_raw_error) / dt

output = pid_feedforward * desired_velocity
       + Kp * control_error
       + Ki * integral
       + Kd * derivative
```

Important details:

- The proportional and integral terms use the deadbanded error.
- The derivative uses raw error so it stays continuous when the error enters or exits the deadband.
- Derivative is zeroed when both current and previous raw errors are inside the deadband.
- The final x/y vector is magnitude-limited by `pid_output_limit`.
- Successive outputs are acceleration-limited by `pid_accel_limit`.
- The first usable output seeds the acceleration limiter instead of assuming the previous command was zero.

## 6. PID Parameters

Default values added to both launch files:

| Param | Default | Meaning |
|---|---:|---|
| `pid_enabled` | `true` | Master switch for the PID controller. |
| `pid_feedforward` | `1.0` | Scales the raw D*-Lite desired command before correction. |
| `pid_kp_x` | `0.35` | Proportional gain for body-frame x velocity. |
| `pid_ki_x` | `0.0` | Integral gain for body-frame x velocity. |
| `pid_kd_x` | `0.02` | Derivative gain for body-frame x velocity. |
| `pid_kp_y` | `0.35` | Proportional gain for body-frame y velocity. |
| `pid_ki_y` | `0.0` | Integral gain for body-frame y velocity. |
| `pid_kd_y` | `0.02` | Derivative gain for body-frame y velocity. |
| `pid_integral_limit` | `0.5` | Anti-windup clamp for each axis integral. |
| `pid_output_limit` | `1.5` | Final x/y vector magnitude clamp in m/s. |
| `pid_accel_limit` | `2.0` | Maximum change in x/y command vector per second. |
| `pid_deadband` | `0.03` | Velocity error ignored near zero, in m/s. |
| `pid_feedback_timeout` | `0.25` | Maximum usable TF pose delta age in seconds. |
| `pid_reset_on_stop` | `true` | Reset integrators and output state when desired speed is near zero. |

## 7. How to Tune PID on the Robot

### 7.1 Safe starting configuration

Start conservative:

```xml
<param name="pid_enabled" type="bool" value="true" />
<param name="pid_feedforward" type="double" value="1.0" />
<param name="pid_kp_x" type="double" value="0.2" />
<param name="pid_kp_y" type="double" value="0.2" />
<param name="pid_ki_x" type="double" value="0.0" />
<param name="pid_ki_y" type="double" value="0.0" />
<param name="pid_kd_x" type="double" value="0.01" />
<param name="pid_kd_y" type="double" value="0.01" />
<param name="pid_output_limit" type="double" value="1.2" />
<param name="pid_accel_limit" type="double" value="1.0" />
```

The current checked-in defaults are slightly stronger but still modest.

### 7.2 What to observe

Record or echo:

```text
/cmd_vel
/dstarlite/raw_cmd_vel
/dstarlite_esdf/raw_cmd_vel
/dstar_status
/tf
/odom
```

Compare raw speed and final speed:

```text
raw_speed = sqrt(raw.linear.x^2 + raw.linear.y^2)
pid_speed = sqrt(cmd.linear.x^2 + cmd.linear.y^2)
```

Healthy behavior:

- `/cmd_vel` should follow the raw command direction.
- `/cmd_vel` should not spike above `pid_output_limit`.
- When goal is reached, raw and final command should both become zero.
- When TF is temporarily unavailable, the planner should stop in the places where robot pose is needed, and PID should pass through the raw command only in its internal feedback fallback case.
- `/dstar_status` should still become true on arrival.

### 7.3 Tuning order

1. Set `pid_ki_x=0` and `pid_ki_y=0`.
2. Start with small `Kp`, such as `0.15` to `0.25`.
3. Drive a straight segment and compare raw vs final `/cmd_vel`.
4. Increase `Kp` until tracking improves but command oscillation is still absent.
5. Add small `Kd`, such as `0.01` to `0.03`, if command changes look too sharp.
6. Keep `Ki` at zero unless there is a clear steady-state velocity error.
7. If adding `Ki`, start at `0.01` or lower and keep `pid_integral_limit` small.
8. Use `pid_accel_limit` to control comfort and chassis jerk.
9. Use `pid_output_limit` as the final safety cap.

### 7.4 Symptom table

| Symptom | Likely cause | First adjustment |
|---|---|---|
| Final command oscillates around raw command | `Kp` or `Kd` too high | Lower `pid_kp_*`, then lower `pid_kd_*`. |
| Robot feels sluggish | `pid_accel_limit` too low or `Kp` too low | Increase `pid_accel_limit`, then `pid_kp_*`. |
| Final speed saturates constantly | `pid_output_limit` too low or `Kp` too high | Raise output limit carefully or lower gains. |
| Small noisy command near zero | Deadband too low | Increase `pid_deadband` to `0.04` or `0.05`. |
| Slow drift remains after long straight motion | No integral action | Add tiny `Ki`, keep integral limit small. |
| Sudden stop on TF warning | Pose TF unavailable | Debug `map -> virtual_frame` TF chain. |

## 8. Safety Behavior Added or Preserved

### 8.1 Stop resets PID

`publish_stop()` now does three things:

1. Creates a zero `geometry_msgs::Twist`.
2. Resets the PID controller state.
3. Publishes zero to both raw command telemetry and final `/cmd_vel`.

This prevents stale integral or previous-output state from affecting the next goal.

### 8.2 TF failure no longer uses stale transforms

The planner had several existing patterns like this:

```cpp
try {
    transformStamped = tfBuffer.lookupTransform(...);
} catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
}

// old behavior: transformStamped was still used here
```

That is unsafe because a failed lookup can leave `transformStamped` stale or uninitialized.

The patched behavior is:

- In dynamic map update: skip the update if the dynamic map transform is unavailable.
- In velocity publication: publish stop and return.
- In main loop robot pose lookup: publish stop, sleep/rate-control, and retry next loop.
- In ESDF goal processing: retry the goal after TF becomes available.

### 8.3 PID feedback failure fallback

If the PID controller cannot estimate measured velocity because TF feedback is missing or stale, it returns the desired command unchanged and seeds output state. That means a transient PID feedback problem does not create a sudden acceleration limiter discontinuity.

The planner-level TF lookups are stricter than the PID fallback. If the planner cannot know the robot pose, it stops. If only the PID feedback delta is temporarily unusable, the command can pass through as raw planner output.

## 9. Subagent Inspection and Fix Iterations

Three read-only subagent inspection rounds were run over the Old_nav-scoped PID integration.

### Round 1 findings and fixes

Useful findings:

- Deadband derivative continuity could produce a correction jump when error exits the deadband.
- `dstarlite_esdf` uses `AsyncSpinner`, so the goal callback handoff should be protected.

Fixes made:

- PID derivative now uses raw error while P/I use deadbanded control error.
- `goal_pose_msg` in `dstarlite_esdf.cpp` is protected with `std::mutex`.
- `get_goal_info.exchange(false)` is used to consume goal events safely.

### Round 2 findings and fixes

Useful finding:

- Acceleration limiting should seed from the first controlled output rather than assuming zero in any edge case.

Fix made:

- `limitAcceleration()` now remembers the first output and returns immediately when no prior output exists.

False positives ignored:

- Unused member declarations do not create link errors when not ODR-used.
- Different `dstar_main()` signatures are isolated in separate executables.
- The ROG-Map launch namespace comment is correct because the YAML top-level key is `rog_map`.

### Round 3 findings and fixes

Useful finding:

- Multiple TF lookup failures were logged but then followed by continued use of the transform object.

Fixes made:

- Classic D*-Lite and ESDF D*-Lite now stop or skip safely on TF lookup failure.
- ESDF goal retry is preserved when TF is unavailable during goal processing.

## 10. Verification Performed

Completed:

- VS Code diagnostics checked for:
  - `velocity_pid_controller.hpp`
  - `dstarlite.cpp`
  - `dstarlite_esdf.cpp`
  - `dstarlite.launch`
  - `dstarlite_esdf.launch`
- No editor diagnostics were reported for those files.
- CMake Tools was invoked, but this VS Code workspace does not expose a configured CMake target for the catkin tree and returned `Unable to configure the project`.
- CMake Tools diagnostics returned no CMake diagnostics.

Not completed:

- A full catkin compile was not completed through VS Code CMake Tools because the workspace is not configured as a normal CMake Tools project.
- Live robot validation was not run in this session.

## 11. Deployment Checklist

Before using on the robot:

1. Source the catkin workspaces exactly as `run_3DNavUL_Test_with_decision.sh` does.
2. Build the `bot_sim` package in the `Old_nav/sim_nav` catkin workspace.
3. Launch nav-only first:

```bash
roslaunch /home/sentry/AstarTraining/Old_nav/3DNavUL_Test.launch
```

4. Confirm TF chain exists:

```text
map -> aft_mapped -> gimbal_frame -> rotbase_frame -> virtual_frame
```

5. Confirm planner command topics:

```text
/cmd_vel
/dstarlite/raw_cmd_vel
/dstar_status
```

6. Send a simple `/clicked_point` goal.
7. Verify `/dstarlite/raw_cmd_vel` appears before comparing final `/cmd_vel`.
8. Watch logs for:

```text
Velocity PID feedback TF unavailable
ERROR IN MAP TO ROBOT
```

9. If either warning repeats, fix TF/localization before increasing PID gains.
10. Run with `pid_enabled=false` once to compare baseline behavior.
11. Re-enable PID and tune gains slowly.

## 12. Recommended Next Improvements

The current PID is intentionally conservative and low-intrusion. Good future improvements are:

1. Add a dedicated debug topic for PID error terms:
   - desired vx/vy
   - measured vx/vy
   - P/I/D contributions
   - output clamp state

2. Add dynamic reconfigure for PID tuning.

3. Use odometry twist directly if a reliable body-frame twist source becomes available.

4. Add angular velocity PID only after validating the chassis and gimbal yaw frame semantics.

5. Add a bag-analysis script that plots raw command, final command, and measured velocity.

6. Add a small rostest or offline unit test around `VelocityPidController` math.

## 13. Short Mental Model

Think of the navigation stack as two layers:

```text
D*-Lite: decides what velocity the robot should want.
PID: adjusts that wanted velocity so the robot actually follows it.
```

The D*-Lite planner still owns obstacle avoidance and goal progress. PID only shapes the final tracking command.
