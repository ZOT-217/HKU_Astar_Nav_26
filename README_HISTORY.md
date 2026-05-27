# Old_nav 部署到新机器人 — 变更历史 / Change History

> 目标：把旧机器人上的导航系统（`Old_nav/`）部署到新机器人上。
> **第一阶段（本次完成）**：仅打通 SLAM 建图。导航 / 决策 / MCU 留待后续。
>
> 关键差异：**旧机器人雷达朝上（且有 20° 倾角），新机器人雷达朝下（绕 X 轴 180°）。**

---

## 1. 硬件 / 系统约束

| 项目 | 取值 |
|---|---|
| OS / ROS | Ubuntu 20.04 + ROS Noetic |
| 用户 | `sentry_train_test`（属于 dialout/sudo/docker） |
| 编译器 | gcc，CMake 4.2.1（`~/.local/bin/cmake`） |
| 雷达 | Livox **MID360**，IP `192.168.1.107`，当前主机网卡为 `192.168.1.50/24` |
| MCU | `/dev/ttyUSB0` @ 921600（**SLAM 阶段不使用**） |
| 雷达姿态 | 旧机器人：朝上 + 20° 前倾；新机器人：**朝下（Roll ≈ 180°）** |

---

## 2. 解决的环境问题

### 2.1 硬编码路径 `/home/sentry/...`
若干脚本（如 `run_3DNavUL_Test_with_decision.sh`）硬编码 `/home/sentry/AstarTraining/Old_nav`。
**修复：** 一次性创建用户主目录软链：

```bash
sudo ln -s /home/sentry_train_test /home/sentry
```

> 之后**不再使用 sudo**（按用户要求）。

### 2.2 Git 子模块缺失 / 失效
- `Old_nav/sim_nav/src/FAST_LIO/` 和 `Old_nav/sim_nav/src/LiDAR_IMU_Init/` 仅是“幽灵 gitlink”，无实际代码。
  **修复：** 加 `CATKIN_IGNORE` 空文件，避免 catkin 扫描。
- `Old_nav/Navigation-filter-test/` 上游 404，且本阶段不需要。**跳过。**
- `Old_nav/livox_ws/src/livox_ros_driver2/` 子模块原 init 失败。
  **修复：**
  ```bash
  cd Old_nav/livox_ws/src
  rm -rf livox_ros_driver2
  git clone --branch parameter_RMUL2026_gimbal --recursive \
      https://github.com/fffcr/livox_ros_driver2.git
  ```

### 2.3 CMake 版本兼容
新版 CMake (≥4) 拒绝 `cmake_minimum_required` 太低的工程。
**修复：** 编译时附加 `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`。

### 2.4 `libdw.so` 链接失败（`/usr/bin/ld: cannot find -ldw`）
PCL/VTK 传递依赖了 `-ldw`，但系统只装了 `libdw1`（运行时），缺 `libdw-dev` 的开发软链。
**修复（不使用 sudo）：** 在用户目录创建本地软链：
```bash
mkdir -p ~/.local/lib
ln -sf /lib/x86_64-linux-gnu/libdw.so.1 ~/.local/lib/libdw.so
```
编译时通过 `-DCMAKE_EXE_LINKER_FLAGS=-L$HOME/.local/lib -DCMAKE_SHARED_LINKER_FLAGS=-L$HOME/.local/lib` 注入。

### 2.5 `livox_ros_driver2` 的 `package.xml` 切换
该包用 `package_ROS1.xml` / `package_ROS2.xml`，需手动选择一份为 `package.xml` 后才能被 catkin 识别。
**修复：**
```bash
cd Old_nav/livox_ws/src/livox_ros_driver2
cp -f package_ROS1.xml package.xml
```

---

## 3. 代码改动

### 3.1 `sim_nav/src/bot_sim/src/imu_filter.cpp` — 参数化重构 
原代码硬编码：
1. `linear_acceleration *= -gravity`（同时做单位换算 g→m/s² 与符号翻转，对应旧机器人 IMU 倒装）；
2. 绕 X 轴旋转 **-20°**（对应旧机器人 20° 机械倾角）。

**改为 ROS 参数化，所有默认值与旧行为完全一致**，旧机器人无需任何配置改动。新增参数：

| 参数 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `accel_unit_scale` | double | 9.81 | 线加速度统一系数（g→m/s²） |
| `accel_invert_x` | bool | true | x 轴是否取反 |
| `accel_invert_y` | bool | true | y 轴是否取反 |
| `accel_invert_z` | bool | true | z 轴是否取反 |
| `tilt_x_deg` | double | -20.0 | 绕 X 轴附加旋转角（度） |
| `apply_tilt_to_accel` | bool | true | 是否对线加速度施加 tilt |
| `apply_tilt_to_gyro` | bool | true | 是否对角速度施加 tilt |
| `gravity` (legacy) | double | — | 如果设置，覆盖 `accel_unit_scale` 默认 |

**新机器人建议** (导航阶段才会用到 imu_filter)：
```yaml
accel_unit_scale: 9.81
accel_invert_x: false
accel_invert_y: false
accel_invert_z: false
tilt_x_deg: 0.0
```
（雷达 180° 翻转的处理改交由 `real_robot_transform` 的 `roll_offset_deg: 180.0` 或 Point-LIO 的 `gravity_align` 处理，更不易出错。）

---

## 4. 编译

### 4.1 Livox 工作区
```bash
cd Old_nav/livox_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1 \
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```
 已编译通过，生成 `devel/lib/livox_ros_driver2/livox_ros_driver2_node`。

### 4.2 Sim_nav 主工作区
```bash
cd Old_nav/sim_nav
source /opt/ros/noetic/setup.bash
source ../livox_ws/devel/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
            -DCMAKE_EXE_LINKER_FLAGS=-L$HOME/.local/lib \
            -DCMAKE_SHARED_LINKER_FLAGS=-L$HOME/.local/lib
```
 全部包编译通过：`pointlio_mapping`、`hdl_localization_nodelet`、`hdl_graph_slam_nodelet`、`rog_map_node`、`dstarlite`、`dstarlite_esdf`、`threeD_lidar_*`、`ser2msg_*` 等。

---

## 5. SLAM 建图运行

### 5.1 设计选择
- **使用 Point-LIO 做 SLAM**（`3DSlamFinal_lio.launch`）。
  - Point-LIO 配置 `mapping.gravity_align: true` + `start_in_aggressive_motion: false`，**会在启动时自动估计重力方向并对齐世界坐标系**。
  - 当前新机器人使用 Livox 驱动层姿态补偿：`msg_MID360.launch` → `MID360_config.json`（`extrinsic.roll = -180.0`），使 Point-LIO 直接消费的 `/livox/lidar` 方向正确。
  - `/3Dlidar` 是独立显示/调试点云，当前 `threeD_lidar_merge_pointcloud/roll_deg = 0.0`，避免在驱动层已经翻转后再次翻转。
  - `imu_filter` 在该 launch 中已注释掉（Point-LIO 直接订阅 `/livox/imu`，配置 `acc_norm: 1` 表示输入是 g）。
- 不依赖 gtsam（缺失）；hdl_graph_slam 虽然编译通过，但建图阶段不启用。

### 5.2 启动步骤
1. **配置主机网卡**（**用户手动执行**，本工具不跑 sudo）：
   ```bash
  sudo ip addr add 192.168.1.50/24 dev <网卡名>     # 当前实测网卡 enp2s0 已是 192.168.1.50/24
   sudo ip link set <网卡名> up
   ping 192.168.1.107      # 应能 ping 通
   ```
2. **新终端启动 ROS / SLAM**（脚本内部会 source ROS、Livox workspace、sim_nav workspace）：
   ```bash
  cd /home/sentry_train_test/AstarTraining/Old_nav
  ./start_3d_slam_lio.sh
   ```
3. **可选 RViz 验证**：
   ```bash
   rviz -d ~/AstarTraining/Old_nav/sim_nav/src/Point-LIO/rviz_cfg/loam_livox.rviz
   ```

### 5.3 验证清单
- `rostopic hz /livox/lidar` ≈ 10 Hz
- `rostopic hz /livox/imu` ≈ 200 Hz
- `rostopic hz /3Dlidar` ≈ 10 Hz（注意 topic 是小写 `l`，不是 `/3DLidar`）
- `rostopic hz /cloud_registered` ≈ 10 Hz
- `rostopic echo -n1 /aft_mapped_to_init`（Point-LIO odometry）有数据
- RViz 中点云在重力对齐的世界坐标系下稳定，没有翻转或漂移

---

## 6. 2026-05-15 `/3Dlidar` 无数据问题修复

### 6.1 现象
用户启动 `Old_nav/3DSlamFinal_lio.launch` 后检查 `/3DLidar`，看不到期望的数据。

### 6.2 根因
1. 该工程实际使用的 topic 名是 `/3Dlidar`（小写 `l`），不是 `/3DLidar`。
2. 当时运行中的 `livox_lidar_publisher2` 被 ROS 环境解析到了 `Old_nav` 目录外的 `ws_livox`，其参数为：
  - `user_config_path=/home/sentry_train_test/AstarTraining/ws_livox/src/livox_ros_driver2/config/dual_MID360_config.json`
  - `multi_topic=1`
  - merge 节点订阅 `/livox/lidar_192_168_1_3` 与 `/livox/lidar_192_168_1_105`
3. 外部配置要求主机 IP `192.168.1.5`，但当前机器实际网卡是 `192.168.1.50/24`，导致 Livox 驱动节点虽然存在，却没有发布有效点云。

### 6.3 修复
1. 修改 `3DSlamFinal_lio.launch`：显式 include `Old_nav/livox_ws/src/livox_ros_driver2/launch_ROS1/msg_MID360.launch`，避免目录外 workspace 通过 `$(find livox_ros_driver2)` 抢先命中。
2. 修改 `livox_ws/src/livox_ros_driver2/config/MID360_config.json`：
  - `host_net_info.*_ip` 改为 `192.168.1.50`
  - `extrinsic_parameter.roll` 从旧机器人使用的 `20.0` 改为 `0.0`，保持 MID360 原始点云/IMU一致性。
3. 同步修改 `mixed_HAP_MID360_config.json` 的 host IP，避免后续误用 mixed launch 时再次连到错误主机 IP。

### 6.4 验证结果
修复后重新启动：

```bash
roslaunch /home/sentry_train_test/AstarTraining/Old_nav/3DSlamFinal_lio.launch
```

实测数据：

| Topic | 结果 |
|---|---|
| `/livox/lidar` | 约 10 Hz |
| `/livox/imu` | 约 200 Hz |
| `/3Dlidar` | 约 10 Hz |
| `/cloud_registered` | 约 10 Hz |
| `/aft_mapped_to_init` | 有 `nav_msgs/Odometry` 数据 |

Livox driver 日志确认：
- 加载配置：`Old_nav/livox_ws/src/livox_ros_driver2/config/MID360_config.json`
- 识别雷达：`192.168.1.107`
- 成功进入 Normal 模式
- 成功 enable Livox IMU

---

## 7. 2026-05-15 `/3Dlidar` 颠倒与 PCD 保存崩溃修复

### 7.1 现象
1. `/3Dlidar` 在 RViz 中上下颠倒。
2. `/cloud_registered` 看不见；终端显示 `laserMapping` 在保存 PCD 时崩溃：
  ```text
  current scan saved to /PCD//home/sentry_train_test/AstarTraining/Old_nav/sim_nav/src/Point-LIO/PCD/scans_1.pcd
  terminate called after throwing an instance of 'pcl::IOException'
  what():  : [pcl::PCDWriter::writeBinary] Error during open!
  ```

### 7.2 根因
1. `threeD_lidar_merge_pointcloud` 仍按旧机器人/双雷达逻辑处理点云：
  - `scan_topic_left` 和 `scan_topic_right` 都订阅 `/livox/lidar`，等于把单个 MID360 点云重复合并。
  - 没有对新机器人雷达朝下（Roll=180°）做输出点云补偿。
2. Point-LIO 保存 PCD 时直接写入 `Point-LIO/PCD/*.pcd`，但代码没有确保 `PCD/` 目录存在；`writeBinary()` 抛异常后进程直接退出。

### 7.3 修复
1. `sim_nav/src/bot_sim/src/threeD_lidar_merge_pointcloud.cpp`
  - 新增参数：`single_lidar_mode`、`roll_deg`、`pitch_deg`、`yaw_deg`、`translate_x/y/z`。
  - 在输出 `/3Dlidar` 前对点云做可配置 RPY 旋转和平移。
2. `sim_nav/src/bot_sim/launch_real/lidar_merge_pointcloud.launch`
  - 当时用于修正 `/3Dlidar` 显示方向的临时配置（已被第 8 节替代）：
    ```xml
    <param name="single_lidar_mode" type="bool" value="true" />
    <param name="roll_deg" type="double" value="180.0" />
    ```
  - 这样 `/3Dlidar` 不再重复合并同一个 MID360，并对雷达朝下做 X 轴 180° 补偿。
3. `sim_nav/src/Point-LIO/src/laserMapping.cpp`
  - 新增 `ensure_pcd_directory()`，启动时如 `pcd_save_en=true` 就创建 `Point-LIO/PCD/`。
  - 新增 `save_pcd_file()`，PCD 写入失败时只打印 ROS error，不再让 `laserMapping` 崩溃。

### 7.4 验证结果
重新编译 `sim_nav` 并再次启动：

```bash
roslaunch /home/sentry_train_test/AstarTraining/Old_nav/3DSlamFinal_lio.launch
```

实测结果：

| 项目 | 结果 |
|---|---|
| `/laserMapping` | 运行中 |
| `/livox_lidar_publisher2` | 运行中 |
| `/threeD_lidar_merge_pointcloud` | 运行中 |
| `/3Dlidar` | 约 10 Hz，`frame_id=aft_mapped` |
| `/cloud_registered` | 约 10 Hz，`frame_id=camera_init` |
| `/aft_mapped_to_init` | 有里程计输出 |
| `Point-LIO/PCD/` | 已在启动时创建 |
| `/threeD_lidar_merge_pointcloud/roll_deg` | 当时为 `180.0`，后续见第 8 节已改为 `0.0` |
| `/threeD_lidar_merge_pointcloud/single_lidar_mode` | `true` |

当前启动的 Point-LIO 会持续累计地图点云；按配置会每 1000 帧保存一次 `scans_*.pcd`，并在正常退出时尝试保存 `scans.pcd`。

---

## 8. 2026-05-15 `/cloud_registered` 颠倒修复

### 8.1 现象
用户在 RViz 中确认：
- `/3Dlidar` 方向正确。
- `/cloud_registered` 仍然上下颠倒。

### 8.2 根因
`/cloud_registered` 由 Point-LIO 直接订阅 `/livox/lidar` 生成，不经过 `threeD_lidar_merge_pointcloud`。
因此只在 `/3Dlidar` 显示节点里设置 `roll_deg=180.0` 只能修正调试点云，不能修正 Point-LIO 地图点云。

### 8.3 修复
1. `livox_ws/src/livox_ros_driver2/config/MID360_config.json`
  - 将当前 MID360 的 `extrinsic_parameter.roll` 从 `0.0` 改为 `-180.0`。
  - 这是当前 SLAM launch 实际加载的配置文件。
2. `livox_ws/src/livox_ros_driver2/config/mixed_HAP_MID360_config.json`
  - 同步将 MID360 相关 `extrinsic_parameter.roll` 改为 `-180.0`，避免后续误用 mixed launch 时方向再次不一致。
3. `sim_nav/src/bot_sim/launch_real/lidar_merge_pointcloud.launch`
  - 将 `/threeD_lidar_merge_pointcloud/roll_deg` 从 `180.0` 改为 `0.0`。
  - 原因：驱动层已经对 `/livox/lidar` 做 Roll -180° 补偿，显示节点不能再做二次翻转。

该修复只修改 JSON / launch 配置，不需要重新编译。

### 8.4 验证结果
重新启动：

```bash
roslaunch /home/sentry_train_test/AstarTraining/Old_nav/3DSlamFinal_lio.launch
```

启动日志确认：
- Livox driver 加载配置：`/home/sentry_train_test/AstarTraining/Old_nav/livox_ws/src/livox_ros_driver2/config/MID360_config.json`
- `successfully set lidar attitude, ip: 192.168.1.107`
- `/threeD_lidar_merge_pointcloud/roll_deg = 0.0`
- Point-LIO 继续保存 PCD，例如 `Point-LIO/PCD/scans_1.pcd`

实测 5 秒 topic 频率：

| Topic | 结果 |
|---|---|
| `/3Dlidar` | 约 10 Hz，`frame_id=aft_mapped` |
| `/cloud_registered` | 约 10 Hz，`frame_id=camera_init` |
| `/livox/lidar` | 约 10 Hz |
| `/livox/imu` | 约 199 Hz |
| `/aft_mapped_to_init` | 有里程计输出 |

---

## 9. 2026-05-15 默认 `Old_nav` 路径与免 source 启动入口

### 9.1 目标
用户希望默认路径固定在 `Old_nav` 内部，日常启动 SLAM 时不再手动执行多条 `source .../setup.bash`。

### 9.2 修复
1. `3DSlamFinal_lio.launch`
  - `old_nav_root` 默认值从 `$(env HOME)/AstarTraining/Old_nav` 固定为 `/home/sentry_train_test/AstarTraining/Old_nav`。
  - 新增 `old_nav_sim_src=$(arg old_nav_root)/sim_nav/src`。
  - `lidar_merge_pointcloud.launch` 和 `mapping_avia.launch` 改为通过 `old_nav_sim_src` 的绝对路径 include，减少外部 workspace 对 `$(find ...)` 的影响。
2. `sim_nav/src/Point-LIO/launch/mapping_avia.launch`
  - 新增 `point_lio_root` 参数。
  - `avia.yaml` 与 RViz 配置路径改为使用 `$(arg point_lio_root)`，由主 launch 传入 `Old_nav` 内部路径。
3. 新增 `start_3d_slam_lio.sh`
  - 自动 source：`/opt/ros/noetic/setup.bash`、`Old_nav/livox_ws/devel/setup.bash`、`Old_nav/sim_nav/devel/setup.bash`。
  - 最终执行：`roslaunch "$OLD_NAV_ROOT/3DSlamFinal_lio.launch" old_nav_root:="$OLD_NAV_ROOT"`。
  - 启动前检查 `livox_ws` / `sim_nav` 的 `devel/setup.bash` 是否存在。
  - 启动前检查是否已有 `livox_ros_driver2_node` 进程，避免重复启动 Livox driver 抢占 UDP 端口。
  - 启动前读取 `MID360_config.json` 的 `cmd_data_ip`，确认该 IP 已分配到本机网卡；否则提前退出并打印当前 IPv4 地址。

之后启动 SLAM 只需要：

```bash
cd /home/sentry_train_test/AstarTraining/Old_nav
./start_3d_slam_lio.sh
```

### 9.3 验证结果
- `start_3d_slam_lio.sh` 已设置可执行权限，并通过 `bash -n` 语法检查。
- `roslaunch --files /home/sentry_train_test/AstarTraining/Old_nav/3DSlamFinal_lio.launch` 可成功解析：`msg_MID360.launch`、`lidar_merge_pointcloud.launch`、`mapping_avia.launch`。
- 当前已有 SLAM 实例在运行，因此本次没有再启动第二个实例，避免节点和雷达端口冲突。

---

## 10. 2026-05-15 Livox `bind failed` 诊断与启动预检查

### 10.1 现象
用户运行 SLAM 启动入口后，Livox driver 报错：

```text
bind failed
Create detection socket failed.
Create detection channel failed.
Failed to init livox lidar sdk.
Init lds lidar failed!
```

### 10.2 根因
本次诊断确认：
- 没有正在运行的 `livox_ros_driver2_node` / Point-LIO / `/3Dlidar` 相关 ROS 节点。
- 没有进程占用 Livox 常用 UDP 端口段。
- `MID360_config.json` 的本机 host IP 仍配置为 `192.168.1.50`。
- 当前本机 IPv4 地址中没有 `192.168.1.50`，只有 `127.0.0.1`、`172.29.2.43`、`100.121.10.94`、`192.168.122.1`、`172.17.0.1`。

因此这次 `bind failed` 不是第二个 Livox driver 抢端口，而是 Livox SDK 试图绑定到本机不存在的地址 `192.168.1.50`。Linux 无法在不存在的本机 IP 上创建 UDP socket，所以 SDK 初始化失败。

### 10.3 处理
1. `start_3d_slam_lio.sh` 已新增预检查：如果 `MID360_config.json` 中的 host IP 没有分配到任何本机网卡，会在启动 ROS 前直接报错，并打印当前 IPv4 地址。
2. 用户需要手动把连接 MID360 的网卡配置回 `192.168.1.50/24`，再启动脚本。例如：

```bash
sudo ip addr add 192.168.1.50/24 dev <连接 MID360 的网卡名>
sudo ip link set <连接 MID360 的网卡名> up
```

本工具不执行 sudo 命令。若 `192.168.1.50` 已被其他地址配置替代，则必须同时修改 `MID360_config.json` 的 `host_net_info.*_ip`，并保证该地址与 MID360 `192.168.1.107` 在同一网段。

### 10.4 验证结果
- `start_3d_slam_lio.sh` 通过 `bash -n` 语法检查。
- 当前环境下运行脚本会提前输出：`ERROR: Livox host IP 192.168.1.50 is not assigned to any local interface.`
- 该错误比 Livox SDK 的 `bind failed` 更早、更明确，能直接指向网卡 IP 配置问题。

---

## 11. 2026-05-15 保存的 PCD 上下颠倒修复

### 11.1 现象
用户确认：建图输出的 `Point-LIO/PCD/scans*.pcd` 上下颠倒。

### 11.2 根因
Point-LIO 的保存路径在 `publish_frame_world()` 中直接把 `feats_down_world` 累加到 `pcl_wait_save`，再写入 `PCD/scans_*.pcd` / `PCD/scans.pcd`。此前只有实时显示链路和 Livox 输入链路做过方向补偿，PCD 写盘阶段没有独立的导出坐标修正。

### 11.3 修复
1. `sim_nav/src/Point-LIO/src/parameters.h` / `parameters.cpp`
  - 新增 PCD 导出参数：`pcd_save/transform_en`、`roll_deg`、`pitch_deg`、`yaw_deg`、`translate_x/y/z`。
2. `sim_nav/src/Point-LIO/src/laserMapping.cpp`
  - 新增 `configure_pcd_save_transform()`，启动时根据参数构造 RPY 旋转和平移。
  - 新增 `copy_point_for_pcd_save()`，只在写入 `pcl_wait_save` 前对点做导出变换。
  - 实时 `/cloud_registered`、里程计 `/aft_mapped_to_init`、建图状态估计不做额外改变。
3. `sim_nav/src/Point-LIO/config/avia.yaml`
  - 当前新机器人 PCD 导出配置：
    ```yaml
    pcd_save:
        transform_en: true
        roll_deg: 180.0
        pitch_deg: 0.0
        yaw_deg: 0.0
        translate_x: 0.0
        translate_y: 0.0
        translate_z: 0.0
    ```

该修复只改变后续新保存的 PCD；之前已经保存的旧 PCD 文件不会自动重写。

### 11.4 验证结果
- `Old_nav/sim_nav` 已重新 `catkin_make`，`pointlio_mapping` 编译通过。
- `roslaunch --files /home/sentry_train_test/AstarTraining/Old_nav/3DSlamFinal_lio.launch` 解析成功。
- VS Code 诊断只剩 ROS includePath 提示；实际 catkin 构建通过。
- 当前网卡缺少 `192.168.1.50`，未启动实机 SLAM 生成新 PCD；需要恢复 MID360 网卡 IP 后重新建图验证视觉方向。

---

## 12. 2026-05-15 全流程默认使用 `Old_nav` 内部文件

### 12.1 目标
用户要求检查以下流程是否默认使用 `Old_nav` 内部文件：

1. `3DSlamFinal_lio.launch` 建图，生成 `Point-LIO/PCD/scans.pcd`。
2. 将 PCD 放到 `sim_nav/src/hdl_graph_slam/map/innowing.pcd`。
3. `hdl_localization.launch` 默认加载该 `innowing.pcd`。
4. `pcd2pgm_hit723_test.launch` 默认从 `Old_nav` 内部 `hdl_graph_slam/map` 读取 PCD。
5. `map_saver.launch` 默认保存到 `Old_nav/sim_nav/src/bot_sim/map/innowing`。
6. `3DNavUL_Test.launch` / RViz 流程默认使用 `Old_nav` 内部 launch 和配置。

### 12.2 修复
1. Livox launch
  - `msg_MID360.launch` / `rviz_MID360.launch` 新增 `user_config_path` 参数，默认指向 `Old_nav/livox_ws/src/livox_ros_driver2/config/MID360_config.json`。
  - `rviz_MID360.launch` 的 RViz 配置也默认指向 `Old_nav` 内部文件。
2. 导航主入口
  - `3DNavUL_Test.launch` 新增 `old_nav_root`、`old_nav_sim_src`、`old_nav_livox_src`、`globalmap_pcd`、`map_root` 参数。
  - 默认 include 改为 `Old_nav` 内部的 `map_server.launch`、`imu_filter.launch`、`rviz_MID360.launch`、`hdl_localization.launch`、`real_robot_transform.launch`、`dstarlite.launch`、`ser2msg_tf_decision_givepoint.launch`。
  - `3DNavUL_Test_with_decision.launch` 的 `nav_launch` 默认改为 `$(arg old_nav_root)/3DNavUL_Test.launch`。
3. 定位与地图文件
  - `hdl_localization.launch` 新增 `globalmap_pcd`，默认加载 `Old_nav/sim_nav/src/hdl_graph_slam/map/innowing.pcd`。
  - `map_server.launch` 新增 `map_root`，默认加载 `Old_nav/sim_nav/src/bot_sim/map/innowing.yaml`。
  - `map_saver.launch` 默认保存为 `Old_nav/sim_nav/src/bot_sim/map/innowing.{pgm,yaml}`。
4. PCD 转 PGM
  - `pcd2pgm.launch`、`pcd2pgm_hit723.launch`、`pcd2pgm_hit723_test.launch` 默认读取 `Old_nav/sim_nav/src/hdl_graph_slam/map/innowing.pcd`。
  - 修复 `pcd2pgm_hit723.launch` 中 `thre_z_max` 行尾多余的 `+`。
5. 脚本入口
  - 新增 `start_3d_nav_ul_test.sh`，自动 source ROS、`Old_nav/livox_ws`、`Old_nav/sim_nav`，并启动 `Old_nav/3DNavUL_Test.launch`。
  - `run_3DNavUL_Test_with_decision.sh` 改为从脚本所在目录推导 `OLD_NAV_ROOT`，不再硬编码 `/home/sentry/AstarTraining/Old_nav`。
  - `sim_nav/bot_sim/setup_custom_pythonpath.sh` 的 PythonPath 修正为 `Old_nav/sim_nav/devel/lib/python3/dist-packages`。

### 12.3 当前推荐流程
建图：

```bash
cd /home/sentry_train_test/AstarTraining/Old_nav
./start_3d_slam_lio.sh
```

保存的 Point-LIO PCD 默认在：

```text
/home/sentry_train_test/AstarTraining/Old_nav/sim_nav/src/Point-LIO/PCD/scans.pcd
```

作为定位地图使用时，放到：

```text
/home/sentry_train_test/AstarTraining/Old_nav/sim_nav/src/hdl_graph_slam/map/innowing.pcd
```

PCD 转 PGM：

```bash
source /opt/ros/noetic/setup.bash
source /home/sentry_train_test/AstarTraining/Old_nav/sim_nav/devel/setup.bash --extend
roslaunch /home/sentry_train_test/AstarTraining/Old_nav/sim_nav/src/pcd2pgm_package/pcd2pgm/launch/pcd2pgm_hit723_test.launch
```

保存二维地图：

```bash
roslaunch /home/sentry_train_test/AstarTraining/Old_nav/sim_nav/src/bot_sim/launch_real/map_saver.launch
```

定位导航：

```bash
cd /home/sentry_train_test/AstarTraining/Old_nav
./start_3d_nav_ul_test.sh
```

### 12.4 验证结果
- `start_3d_nav_ul_test.sh`、`start_3d_slam_lio.sh`、`run_3DNavUL_Test_with_decision.sh` 均通过 `bash -n`。
- `roslaunch --files` 已验证：`3DSlamFinal_lio.launch`、`3DNavUL_Test.launch`、`pcd2pgm.launch`、`pcd2pgm_hit723.launch`、`pcd2pgm_hit723_test.launch`、`map_server.launch`、`map_saver.launch` 均可解析。
- 运行相关的 `.launch` / `.sh` 文件中，过滤 `build`、`devel`、`AAAnotes`、`catkin_generated` 后，未再发现 `/home/sentry/AstarTraining/Old_nav` 或 `/home/sentry_train_test/AstarTraining/sim_nav` 旧路径。

---

## 13. 2026-05-15 hdl_localization 垂直下坠修复

### 13.1 现象
启动 `Old_nav/3DNavUL_Test.launch` 后，在 RViz 中给出正确的初始位姿，定位会先短暂稳定，然后 `/odom` / TF 在 z 方向快速下坠。

### 13.2 诊断结果
实测初始异常状态：

| 项目 | 结果 |
|---|---|
| `/livox/imu` 原始加速度 z | 约 `-0.999 g` |
| `/livox/imu_filtered` 加速度 z | 约 `+9.809 m/s²` |
| `/hdl_localization_nodelet/invert_acc` | `true` |
| `/odom.z` 下坠速度 | 曾达到约 `-1670 m/s` |
| `/status` | 一度出现 `matching_error=nan`、`inlier_fraction=0.0` |

根因是导航定位链路仍带有旧机器人 IMU 配置：

1. `imu_filter` 已经把当前 MID360 原始 `-1 g` 修正成 hdl_localization 过程模型需要的 `+9.81 m/s²`。
2. `hdl_localization` 又因为 `invert_acc=true` 再次把加速度取反，UKF prediction 等效看到错误重力方向，于是 z 轴快速积分发散。
3. `imu_filter.launch` 仍保留旧机器人 `-20°` tilt 逻辑；新机器人本轮配置中不应再额外施加该倾角。

### 13.3 修复
1. `sim_nav/src/hdl_localization/launch/hdl_localization.launch`
  - 将 `invert_imu_acc` 默认值改为 `false`，避免 hdl_localization 对已经过滤正确的 IMU 加速度再次取反。
2. `sim_nav/src/bot_sim/launch_real/imu_filter.launch`
  - 新机器人导航实测配置固定为：
    ```xml
    <param name="accel_unit_scale" type="double" value="9.81" />
    <param name="accel_invert_x" type="bool" value="true" />
    <param name="accel_invert_y" type="bool" value="true" />
    <param name="accel_invert_z" type="bool" value="true" />
    <param name="tilt_x_deg" type="double" value="0.0" />
    <param name="apply_tilt_to_accel" type="bool" value="false" />
    <param name="apply_tilt_to_gyro" type="bool" value="false" />
    ```
  - 注意：这里三个 `accel_invert_*` 保持 `true` 是当前 MID360 驱动姿态补偿后的实测结果，用来把原始静止 z 轴约 `-1 g` 转成 hdl_localization 期望的 `+9.81 m/s²`。

### 13.4 验证结果
修复后重启 `./start_3d_nav_ul_test.sh`，并在 RViz 中重新给初始位姿：

| 项目 | 结果 |
|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2`，约 `10 Hz`，`frame_id=aft_mapped` |
| `/aligned_points` | 约 `10 Hz` |
| `/livox/imu_filtered` 平均加速度 | `x=-0.1078`、`y=-0.0875`、`z=9.8076` |
| `/status` | `has_converged=True`，`matching_error≈0.007~0.008`，`inlier_fraction≈0.989` |
| 25 秒 `/odom.z` | `-0.1866 -> -0.1956`，约 `-0.0004 m/s` |
| 20 秒复测 `/odom.z` | `-0.1999 -> -0.2025`，约 `-0.0001 m/s` |

结论：此前“突然垂直下降”的主因已消除；当前 NDT 匹配质量恢复正常，z 轴仅有厘米级小幅波动。

---

## 14. 2026-05-17 `cmd_vel_marker` / `world_align_node` 启动报错修复

### 14.1 现象
直接执行：

```bash
roslaunch /home/sentry/AstarTraining/Old_nav/3DNavUL_Test.launch
```

启动过程中出现：

```text
ERROR: cannot launch node of type [bot_sim/cmd_vel_marker]
ERROR: cannot launch node of type [bot_sim/world_align_node]
```

### 14.2 诊断结果
检查 `Old_nav` 内部构建产物后确认：

| 项目 | 结果 |
|---|---|
| `sim_nav/src/bot_sim/src/cmd_vel_marker.cpp` | 存在 |
| `sim_nav/src/bot_sim/src/world_align_node.cpp` | 存在 |
| `sim_nav/devel/lib/bot_sim/cmd_vel_marker` | 存在且可执行 |
| `sim_nav/devel/lib/bot_sim/world_align_node` | 存在且可执行 |
| 正确 source `Old_nav/sim_nav/devel/setup.bash` 后 `rosrun` | 两个节点都能解析 |

因此问题不是源码或可执行权限丢失，而是直接 `roslaunch` 时当前 shell 的 `ROS_PACKAGE_PATH` 可能先命中了 `Old_nav` 外部的 `bot_sim`。外部 `bot_sim` 里有部分老节点，所以 `ser2msg_decision_givepoint` 能启动，但没有新加入的 `cmd_vel_marker` 和 `world_align_node`，于是 roslaunch 报 “Cannot locate node”。

### 14.3 修复
1. `3DNavUL_Test.launch`
  - 新增：
    ```xml
    <arg name="enable_cmd_vel_markers" default="false"/>
    <arg name="enable_world_align" default="false"/>
    ```
  - 默认直接 `roslaunch` 时不再启动这三个辅助节点，避免外部 `bot_sim` 阴影环境下报错。
  - 向 `dstarlite.launch` 传递 `enable_cmd_vel_markers`。
  - 向 `ser2msg_tf_decision_givepoint.launch` 传递 `old_nav_root` 与 `enable_world_align`。
2. `sim_nav/src/bot_sim/launch_real/dstarlite.launch`
  - 新增 `enable_cmd_vel_markers` 参数。
  - `/cmd_vel_marker` 与 `/dstarlite/raw_cmd_vel_marker` 仅在该参数为 `true` 时启动。
3. `sim_nav/src/bot_sim/launch_real/ser2msg_tf_decision_givepoint.launch`
  - 新增 `enable_world_align` 参数。
  - `world_align.launch` 仅在该参数为 `true` 时 include。
4. `start_3d_nav_ul_test.sh`
  - 仍然先 source `Old_nav/livox_ws` 和 `Old_nav/sim_nav`。
  - 启动时显式传入：
    ```bash
    enable_cmd_vel_markers:=true
    enable_world_align:=true
    ```
  - 因此推荐入口仍保留完整辅助可视化和世界坐标对齐节点。

### 14.4 验证结果
验证命令结果：

| 验证项 | 结果 |
|---|---|
| `bash -n start_3d_nav_ul_test.sh` | 通过 |
| 默认 `roslaunch --nodes 3DNavUL_Test.launch` | 不再列出 `cmd_vel_marker`、`raw_cmd_vel_marker`、`world_align_node` |
| `enable_cmd_vel_markers:=true enable_world_align:=true` | 三个辅助节点会被列出 |
| 正确 source `Old_nav` 后 `roslaunch --args /cmd_vel_marker ...` | 解析到 `Old_nav/sim_nav/devel/lib/bot_sim/cmd_vel_marker` |
| 正确 source `Old_nav` 后 `roslaunch --args /world_align_node ...` | 解析到 `Old_nav/sim_nav/devel/lib/bot_sim/world_align_node` |
| 只 source 外部 `sim_nav` 的阴影环境 | 默认不再列出三个辅助节点，避免复现该报错 |

推荐实机完整导航入口仍是：

```bash
cd /home/sentry_train_test/AstarTraining/Old_nav
./start_3d_nav_ul_test.sh
```

如果必须直接 `roslaunch`，且希望打开这三个辅助节点，需要先 source `Old_nav` 的两个 workspace，再手动传入：

```bash
source /opt/ros/noetic/setup.bash
source /home/sentry_train_test/AstarTraining/Old_nav/livox_ws/devel/setup.bash --extend
source /home/sentry_train_test/AstarTraining/Old_nav/sim_nav/devel/setup.bash --extend
roslaunch /home/sentry_train_test/AstarTraining/Old_nav/3DNavUL_Test.launch \
  enable_cmd_vel_markers:=true \
  enable_world_align:=true
```

---

## 15. 2026-05-17 `/dstar_path` 起点落在 `map` 原点的诊断与默认启动防呆

### 15.1 现象
用户观察到 `/dstar_path` 的第一点接近 `map` 原点，而不是机器人当前定位点。现场检查结果：

| 项目 | 结果 |
|---|---|
| `/dstar_path` 第一项 | 约 `x=-0.019`、`y=-0.037` |
| `map -> aft_mapped` | 约 `x=3.64`、`y=-1.41`、`z=0.17` |
| `map -> gimbal_frame` | 存在 |
| `map -> rotbase_frame` | 存在 |
| `map -> virtual_frame` | 不存在 |
| `/sentinel_nav_position` | 不存在 |

### 15.2 根因
`/dstar_path` 保持 `frame_id=map` 是正确设计，因为 D*-Lite 在 `/map` 栅格上做全局规划；但路径第一点应来自 `map -> virtual_frame`，不应固定在 `map` 原点。

本次现场确认当前 ROS master 中运行的关键节点并不是 `Old_nav` 内的可执行文件，而是外层工作区：

| 节点 | 实际可执行文件 |
|---|---|
| `/dstarlite` | `/home/sentry_train_test/AstarTraining/sim_nav/devel/lib/bot_sim/dstarlite` |
| `/ser2msg_decision_givepoint` | `/home/sentry_train_test/AstarTraining/sim_nav/devel/lib/bot_sim/ser2msg_decision_givepoint` |
| `/real_robot_transform` | `/home/sentry_train_test/AstarTraining/sim_nav/devel/lib/bot_sim/real_robot_transform` |

因此问题不是 `/dstar_path` 应改成 `aft_mapped`，而是直接 launch 时 ROS package 解析被外层 `sim_nav` 的 `bot_sim` 遮蔽。外层节点没有发布当前 `Old_nav` 版本需要的 `virtual_frame` / `/sentinel_nav_position`，导致 D*-Lite 起点退化到原点附近。

### 15.3 修复
1. `start_3d_nav_ul_test.sh`
  - 启动前将 `Old_nav/sim_nav/src` 与 `Old_nav/livox_ws/src` 放到 `ROS_PACKAGE_PATH` 最前面。
  - 执行 `rospack find bot_sim` 与 `rospack find livox_ros_driver2` 检查。
  - 若 `bot_sim` 或 `livox_ros_driver2` 没有解析到 `Old_nav` 内部路径，直接报错退出，避免再次用外层工作区节点启动导航。
2. `run_3DNavUL_Test_with_decision.sh`
  - 调整 source 顺序：先 source ROS / 可选外部工作区，最后 source `Old_nav/livox_ws` 与 `Old_nav/sim_nav`。
  - 同样将 `Old_nav` package path 前置，并检查 `bot_sim` / `livox_ros_driver2` 的解析路径。
  - 日志中打印实际命中的 `bot_sim` 与 `livox_ros_driver2` package 路径。

### 15.4 推荐操作
当前推荐只使用脚本入口启动导航：

```bash
cd /home/sentry_train_test/AstarTraining/Old_nav
./start_3d_nav_ul_test.sh
```

不建议在未 source `Old_nav` 的 shell 中直接执行：

```bash
roslaunch /home/sentry_train_test/AstarTraining/Old_nav/3DNavUL_Test.launch
```

直接 `roslaunch` 虽然会读取 `Old_nav` 的 launch 文件，但其中 `pkg="bot_sim"` 的节点仍会按当前 shell 的 ROS package 环境解析，可被目录外同名包遮蔽。

### 15.5 重启验证结果
用户重新配置主机网卡 IP 后，按推荐入口重启：

```bash
cd /home/sentry_train_test/AstarTraining/Old_nav
./start_3d_nav_ul_test.sh
```

验证结果：

| 项目 | 结果 |
|---|---|
| `192.168.1.50` | 已挂到 `enp2s0` |
| Livox driver | `Init livox lidars succ.`，MID360 `192.168.1.107` 进入 Normal，IMU / PointCloud2 publisher 创建成功 |
| `rospack find bot_sim` | `/home/sentry_train_test/AstarTraining/Old_nav/sim_nav/src/bot_sim` |
| `rospack find livox_ros_driver2` | `/home/sentry_train_test/AstarTraining/Old_nav/livox_ws/src/livox_ros_driver2` |
| `/sentinel_nav_position` | `map` 系约 `x=3.701`、`y=-1.592`、`z=-0.271` |
| `map -> virtual_frame` | 约 `x=3.700`、`y=-1.584`、`z=-0.270` |
| `/dstar_path` 第一项 | `frame_id=map`，约 `x=3.681`、`y=-1.587`、`z=0.0` |

结论：`/dstar_path` 保持 `map` frame，且第一点已经恢复到机器人当前规划点附近，不再落在 `map` 原点。原问题由外层/残留同名节点与 `virtual_frame` TF 异常引起，默认启动脚本的 Old_nav package-path 检查可以防止再次误启外层 `bot_sim`。

---

## 16. 2026-05-17 古早 `/grid` 动态避障链路确认

### 16.1 用户更正
本节记录的是 ROG-Map 之前那套更古早的动态避障链路，不是 ROG-Map Stage 2。用户指定可以参考外层 `/home/sentry_train_test/AstarTraining/sim_nav`，但当前实际改动仍只记录在 `Old_nav`。

### 16.2 外层 `sim_nav` 中的原始链路
参考外层 `sim_nav` 可确认，古早动态避障由 `/grid` 驱动：

```text
Livox CustomMsg
  -> threeD_lidar_filter
  -> /test_scan
  -> dbscan_bfs_3D
  -> /grid
  -> dstarlite dynamic_map_topic_name
```

关键文件与行为：

| 文件 | 作用 |
|---|---|
| `/home/sentry_train_test/AstarTraining/sim_nav/src/bot_sim/launch_real/lidar_filter.launch` | 启动 `threeD_lidar_filter`，把 Livox 点云按 `gimbal_frame` / `aft_mapped` TF 过滤后发布到 `/test_scan` |
| `/home/sentry_train_test/AstarTraining/sim_nav/src/bot_sim/launch_real/dbscan_bfs_3D.launch` | 启动 `dbscan_bfs_3D` |
| `/home/sentry_train_test/AstarTraining/sim_nav/src/bot_sim/src/dbscan_bfs_3D.cpp` | 订阅 `/test_scan`，DBSCAN 聚类后 BFS 膨胀，发布 `nav_msgs/OccupancyGrid` 到 `/grid` |
| `/home/sentry_train_test/AstarTraining/sim_nav/src/bot_sim/launch_real/dstarlite.launch` | `dynamic_map_topic_name=/grid`，D*-Lite 把 `/grid` 融合进动态障碍代价 |

外层 `node_watcher_uc.py` 也体现了这套启动顺序：`lidar_filter.launch` -> `dbscan_bfs_3D.launch` -> `dstarlite.launch`。

### 16.3 `Old_nav` 当前状态
`Old_nav` 里 planner 侧仍然保留了 `/grid` 接口，本轮已把古早发布端正式接回主导航：

| 项目 | 当前状态 |
|---|---|
| `dstarlite.launch` | 默认 `dynamic_map_topic_name=/grid` |
| `dstarlite.cpp` | 收到动态 `OccupancyGrid` 后调用 `when_receive_new_dynamic_map()`，用动态占据值更新节点代价并触发 D*-Lite 更新 |
| `dbscan_bfs_3D.cpp` | 源码仍存在，CMake 仍构建 `dbscan_bfs_3D` |
| `lidar_filter.launch` | 已从 `.backup` 恢复为普通 launch，启动 `threeD_lidar_filter` 并发布 `/test_scan` |
| `dbscan_bfs_3D.launch` | 已从 `.backup` 恢复为普通 launch，启动 `dbscan_bfs_3D` 并发布 `/grid` |
| `3DNavUL_Test.launch` | 新增 `dynamic_avoidance_source`，默认 `dbscan`；默认 include `lidar_filter.launch` 和 `dbscan_bfs_3D.launch` |

因此当前推荐入口 `./start_3d_nav_ul_test.sh` 不加参数时会默认启用古早 `/grid` 动态避障。

### 16.4 古早 `/grid` 链路的开启方法
保持当前 `dstarlite` 默认分支即可，不需要设置 `costmap_source:=rog`。现在默认启动就是古早 `/grid`：

```bash
cd /home/sentry_train_test/AstarTraining/Old_nav
./start_3d_nav_ul_test.sh
```

如果要显式指定，也可以写成：

```bash
./start_3d_nav_ul_test.sh dynamic_avoidance_source:=dbscan
```

### 16.5 ROG Stage 2 / 关闭动态发布端的切换方法
新增参数：

| 参数值 | 行为 |
|---|---|
| `dynamic_avoidance_source:=dbscan` | 默认；启动 `threeD_lidar_filter` + `dbscan_bfs_3D`，D*-Lite 消费 `/grid` |
| `dynamic_avoidance_source:=rog` | 启动 ROG-Map observer + bridge，D*-Lite 消费 `/rog_grid` |
| `dynamic_avoidance_source:=none` | 不启动古早 `/grid` 或 ROG 发布端；D*-Lite 仍启动，但没有动态发布端输入 |

ROG Stage 2：

```bash
./start_3d_nav_ul_test.sh dynamic_avoidance_source:=rog
```

临时关闭动态发布端：

```bash
./start_3d_nav_ul_test.sh dynamic_avoidance_source:=none
```

旧参数 `costmap_source:=rog` 仍保留兼容：传入后也会让 D*-Lite 消费 `/rog_grid`，并阻止默认古早 `/grid` 发布端启动。

### 16.6 启动后验证
古早 `/grid` 动态避障真正开启时，应满足：

```bash
rosnode list | grep -E 'threeD_lidar_filter|dbscan_bfs_3D|dstarlite'
rostopic hz /test_scan
rostopic hz /grid
rostopic info /grid
rosparam get /dstarlite/dynamic_map_topic_name
```

期望结果：

| 验证项 | 期望 |
|---|---|
| `threeD_lidar_filter` | 存在 |
| `dbscan_bfs_3D` | 存在 |
| `/test_scan` | 有 `sensor_msgs/PointCloud2` 输出 |
| `/grid` | 有 `nav_msgs/OccupancyGrid` 输出 |
| `rostopic info /grid` | `/dstarlite` 在 Subscribers 中 |
| `/dstarlite/dynamic_map_topic_name` | `/grid` |

需要特别注意：`lidar_filter.launch.backup` 的参数是从旧链路继承来的。它把 `scan_topic_left` 和 `scan_topic_right` 都设成 `/livox/lidar`，用于当前单 MID360 时可以让两个回调都收到同一份 Livox CustomMsg，但它仍沿用 `threeD_lidar_filter.cpp` 中旧机器人相关的点云高度过滤和几何假设。实机恢复前应先在 RViz 同时看 `/test_scan` 与 `/grid`，确认动态障碍落点、范围和方向正确，再让机器人按该代价层自主运动。

### 16.7 本轮静态验证
已完成静态 launch 检查：

| 启动参数 | 结果 |
|---|---|
| 默认启动 | 包含 `/threeD_lidar_filter`、`/dbscan_bfs_3D`、`/dstarlite`；不包含 ROG 节点 |
| `dynamic_avoidance_source:=rog` | 包含 `/rog_map_node`、`/rog_map_to_grid_node`、`/dstarlite`；不包含古早 `/grid` 发布节点 |
| `dynamic_avoidance_source:=none` | 只保留 `/dstarlite`，不启动两套动态发布端 |

`roslaunch --args` 也确认 `threeD_lidar_filter`、`dbscan_bfs_3D`、`dstarlite`、`rog_map_node`、`rog_map_to_grid_node` 的可执行文件均解析到 `Old_nav/sim_nav/devel/lib/bot_sim/`。`start_3d_nav_ul_test.sh` 已通过 `bash -n` 语法检查。

### 16.8 启动后无实际避障效果的修复
用户实机启动后反馈没有实际避障效果。运行态日志显示根因不是 D*-Lite 没订阅 `/grid`，而是古早输入滤波节点仍按旧机器人接口订阅 Livox `CustomMsg`：

```text
Client [/threeD_lidar_filter] wants topic /livox/lidar to have datatype/md5sum [livox_ros_driver2/CustomMsg/...],
but our version has [sensor_msgs/PointCloud2/...]. Dropping connection.
```

当前 MID360 导航 launch 使用 `rviz_MID360.launch`，`/livox/lidar` 实际类型是 `sensor_msgs/PointCloud2`。因此旧版 `threeD_lidar_filter` 虽然节点存在，但收不到有效输入，导致 `/test_scan` 和 `/grid` 没有稳定动态障碍数据。

修复：

1. `sim_nav/src/bot_sim/src/threeD_lidar_filter.cpp`
  - 输入从 `livox_ros_driver2/CustomMsg` 改为 `sensor_msgs/PointCloud2`。
  - 使用 `pcl::fromROSMsg()` 转换点云。
  - 保持输出 `/test_scan` 为 `sensor_msgs/PointCloud2`，frame 为 `gimbal_frame`，后级 `dbscan_bfs_3D` 接口不变。
  - 新增 `single_lidar_mode` 参数，当前单 MID360 模式只要求一个 `/livox/lidar` 输入。
2. `sim_nav/src/bot_sim/launch_real/lidar_filter.launch`
  - 新增 `single_lidar_mode=true`。

验证：

| 项目 | 结果 |
|---|---|
| `catkin_make --pkg bot_sim` | 通过 |
| `/threeD_lidar_filter` | 可执行文件解析到 `Old_nav/sim_nav/devel/lib/bot_sim/threeD_lidar_filter` |
| `/test_scan` | `sensor_msgs/PointCloud2`，约 10 Hz，frame=`gimbal_frame` |
| `/grid` | `nav_msgs/OccupancyGrid`，约 5 Hz，frame=`gimbal_frame` |
| `/grid` 数据 | `100x100`，resolution=`0.05`，采样中 `max=100`，`count_100=2126`，说明已有动态障碍占据单元 |

---

## 17. 待办（后续阶段）

- [x] 网络 / 雷达上电验证（`/livox/lidar`、`/livox/imu`、`/3Dlidar`、`/cloud_registered` 已有频率）
- [x] SLAM topic 跑通（`/cloud_registered` 已恢复）
- [x] 导航定位垂直下坠修复（IMU 重力方向与 hdl_localization `invert_acc` 已校正）
- [x] 导航 launch 缺失 `cmd_vel_marker` / `world_align_node` 报错修复
- [x] `/dstar_path` 起点落在 `map` 原点的 package 遮蔽诊断与默认启动防呆
- [x] 古早 `/grid` 动态避障链路定位与开启方法确认
- [ ] 现场移动建完整地图并保存 pcd
- [ ] 导航阶段：
  - [x] 为新机器人创建专用 `imu_filter.launch` 实测参数
  - [x] hdl_localization 初始化后 z 轴稳定性验证
  - [ ] `real_robot_transform.launch` 中 `roll_offset_deg: 180.0`
  - [ ] hdl_localization 初始化位姿（针对新建图）
  - [ ] MCU / 决策接入

---

## 18. 关键文件索引

| 文件 | 状态 |
|---|---|
| [Old_nav/sim_nav/src/bot_sim/src/imu_filter.cpp](sim_nav/src/bot_sim/src/imu_filter.cpp) | 已参数化（默认兼容旧机器人） |
| [Old_nav/sim_nav/src/FAST_LIO/CATKIN_IGNORE](sim_nav/src/FAST_LIO/CATKIN_IGNORE) | 新增（空文件） |
| [Old_nav/sim_nav/src/LiDAR_IMU_Init/CATKIN_IGNORE](sim_nav/src/LiDAR_IMU_Init/CATKIN_IGNORE) | 新增（空文件） |
| [Old_nav/livox_ws/src/livox_ros_driver2/](livox_ws/src/livox_ros_driver2/) | 子模块重新克隆 (`parameter_RMUL2026_gimbal`) |
| [Old_nav/livox_ws/src/livox_ros_driver2/package.xml](livox_ws/src/livox_ros_driver2/package.xml) | 由 `package_ROS1.xml` 复制 |
| `/home/sentry → /home/sentry_train_test` | 软链（绕过硬编码） |
| `~/.local/lib/libdw.so → /lib/x86_64-linux-gnu/libdw.so.1` | 链接修复 |
| [Old_nav/3DSlamFinal_lio.launch](3DSlamFinal_lio.launch) | 默认根路径固定为 `Old_nav`，并显式 include 内部 Livox / bot_sim / Point-LIO launch |
| [Old_nav/start_3d_slam_lio.sh](start_3d_slam_lio.sh) | 新增免手动 source 的 SLAM 启动入口，并带 Livox 进程 / host IP 预检查 |
| [Old_nav/start_3d_nav_ul_test.sh](start_3d_nav_ul_test.sh) | 免手动 source 的定位导航启动入口；前置 Old_nav package path，并检查 `bot_sim` / `livox_ros_driver2` 必须解析到 Old_nav 内部 |
| [Old_nav/3DNavUL_Test.launch](3DNavUL_Test.launch) | 默认 include 路径固定到 `Old_nav` 内部 Livox / bot_sim / hdl_localization；直接 launch 默认关闭易受外部 `bot_sim` 阴影影响的辅助节点 |
| [Old_nav/3DNavUL_Test_with_decision.launch](3DNavUL_Test_with_decision.launch) | 导航子 launch 默认指向 `Old_nav/3DNavUL_Test.launch` |
| [Old_nav/run_3DNavUL_Test_with_decision.sh](run_3DNavUL_Test_with_decision.sh) | 从脚本目录推导 `OLD_NAV_ROOT`；最后 source Old_nav 工作区并检查同名 ROS package 不被外层工作区遮蔽 |
| [Old_nav/livox_ws/src/livox_ros_driver2/config/MID360_config.json](livox_ws/src/livox_ros_driver2/config/MID360_config.json) | host IP 改为 `192.168.1.50`，roll 改为 `-180.0` |
| [Old_nav/livox_ws/src/livox_ros_driver2/config/mixed_HAP_MID360_config.json](livox_ws/src/livox_ros_driver2/config/mixed_HAP_MID360_config.json) | host IP 同步改为 `192.168.1.50`，MID360 roll 同步为 `-180.0` |
| [Old_nav/sim_nav/src/bot_sim/src/threeD_lidar_merge_pointcloud.cpp](sim_nav/src/bot_sim/src/threeD_lidar_merge_pointcloud.cpp) | 新增单雷达模式与 RPY/平移输出补偿 |
| [Old_nav/sim_nav/src/bot_sim/launch_real/lidar_merge_pointcloud.launch](sim_nav/src/bot_sim/launch_real/lidar_merge_pointcloud.launch) | 新机器人 `/3Dlidar` 配置为单雷达，当前 Roll `0.0`（驱动层已补偿） |
| [Old_nav/sim_nav/src/bot_sim/launch_real/lidar_filter.launch](sim_nav/src/bot_sim/launch_real/lidar_filter.launch) | 古早 `/grid` 动态避障输入端；启动 `threeD_lidar_filter`，发布 `/test_scan` |
| [Old_nav/sim_nav/src/bot_sim/launch_real/dbscan_bfs_3D.launch](sim_nav/src/bot_sim/launch_real/dbscan_bfs_3D.launch) | 古早 `/grid` 动态避障发布端；启动 `dbscan_bfs_3D` |
| [Old_nav/sim_nav/src/bot_sim/src/threeD_lidar_filter.cpp](sim_nav/src/bot_sim/src/threeD_lidar_filter.cpp) | 已改为订阅当前 MID360 的 `sensor_msgs/PointCloud2`，输出 `/test_scan` 给古早 `/grid` 链路 |
| [Old_nav/sim_nav/src/bot_sim/src/dbscan_bfs_3D.cpp](sim_nav/src/bot_sim/src/dbscan_bfs_3D.cpp) | 订阅 `/test_scan`，DBSCAN + BFS 膨胀后发布 `/grid` 给 D*-Lite |
| [Old_nav/sim_nav/src/Point-LIO/launch/mapping_avia.launch](sim_nav/src/Point-LIO/launch/mapping_avia.launch) | 新增 `point_lio_root` 参数，配置路径可由主 launch 固定到 `Old_nav` 内部 |
| [Old_nav/sim_nav/src/Point-LIO/config/avia.yaml](sim_nav/src/Point-LIO/config/avia.yaml) | PCD 保存启用导出 Roll 180° 补偿 |
| [Old_nav/sim_nav/src/Point-LIO/src/parameters.h](sim_nav/src/Point-LIO/src/parameters.h) | 新增 PCD 导出变换参数声明 |
| [Old_nav/sim_nav/src/Point-LIO/src/parameters.cpp](sim_nav/src/Point-LIO/src/parameters.cpp) | 读取 PCD 导出 RPY / 平移参数 |
| [Old_nav/sim_nav/src/Point-LIO/src/laserMapping.cpp](sim_nav/src/Point-LIO/src/laserMapping.cpp) | PCD 目录自动创建，写入失败不再崩溃；保存 PCD 前可单独应用导出变换 |
| [Old_nav/sim_nav/src/bot_sim/launch_real/imu_filter.launch](sim_nav/src/bot_sim/launch_real/imu_filter.launch) | 导航实测参数：输出静止重力约 `+9.81 m/s²`，禁用旧机器人 `-20°` tilt |
| [Old_nav/sim_nav/src/bot_sim/launch_real/dstarlite.launch](sim_nav/src/bot_sim/launch_real/dstarlite.launch) | `cmd_vel_marker` / `raw_cmd_vel_marker` 由 `enable_cmd_vel_markers` 控制；默认 `dynamic_map_topic_name=/grid` |
| [Old_nav/sim_nav/src/bot_sim/launch_real/ser2msg_tf_decision_givepoint.launch](sim_nav/src/bot_sim/launch_real/ser2msg_tf_decision_givepoint.launch) | `world_align.launch` 由 `enable_world_align` 控制 |
| [Old_nav/sim_nav/src/hdl_localization/launch/hdl_localization.launch](sim_nav/src/hdl_localization/launch/hdl_localization.launch) | 默认加载 `Old_nav` 内部 `hdl_graph_slam/map/innowing.pcd`，`invert_imu_acc=false` |
| [Old_nav/sim_nav/src/bot_sim/launch_real/map_server.launch](sim_nav/src/bot_sim/launch_real/map_server.launch) | 默认加载 `Old_nav` 内部 `bot_sim/map/innowing.yaml` |
| [Old_nav/sim_nav/src/bot_sim/launch_real/map_saver.launch](sim_nav/src/bot_sim/launch_real/map_saver.launch) | 默认保存 `Old_nav` 内部 `bot_sim/map/innowing` |
| [Old_nav/sim_nav/src/pcd2pgm_package/pcd2pgm/launch/pcd2pgm_hit723_test.launch](sim_nav/src/pcd2pgm_package/pcd2pgm/launch/pcd2pgm_hit723_test.launch) | 默认读取 `Old_nav` 内部 `hdl_graph_slam/map/innowing.pcd` |

---

## 19. 动态避障 Pipeline 技术报告

> 撰写日期：2026-05-17
> 本节以中文详细描述系统中两条动态避障链路的完整数据流、各节点作用、关键参数及适用场景。

---

### 19.1 总览

系统提供两条相互独立的动态避障 pipeline，通过 `3DNavUL_Test.launch` 的 `dynamic_avoidance_source` 参数选择：

| 参数值 | Pipeline | 输出话题 |
|--------|----------|----------|
| `dbscan`（默认） | 原有 DBSCAN Pipeline | `/grid` |
| `rog` | ROG 二阶段 Pipeline | `/rog_grid` |
| `none` | 关闭动态避障 | 无 |

两条链路的**终点**都是 D\*-Lite 规划器（`dstarlite` 节点），它通过 `dynamic_map_topic_name` 参数订阅对应话题，将动态障碍叠加到静态地图上进行增量重规划。

---

### 19.2 原有动态避障 Pipeline（`/grid`，DBSCAN 链路）

#### 19.2.1 完整数据流

```
Livox MID360 固态雷达
        │
        │  /livox/lidar
        │  sensor_msgs/PointCloud2, ~10 Hz, frame=livox_frame
        ▼
┌─────────────────────────────────────────────────────────────────┐
│  threeD_lidar_filter（节点：threeD_lidar_filter）               │
│  launch：lidar_filter.launch                                     │
│                                                                 │
│  功能：                                                          │
│  1. 通过 TF 查询 gimbal_frame ← aft_mapped，将 3D 点云           │
│     从 aft_mapped 坐标系变换到机器人本体 gimbal_frame             │
│  2. 距原点 < first_RADIUS(0.4m) 的点丢弃（机器人自身结构遮挡）   │
│  3. 高度 > max_height(0.3m) 的点丢弃（天花板/高处无关障碍）       │
│  4. 输出过滤后的点云，frame_id = gimbal_frame                    │
│                                                                 │
│  关键参数（来自 lidar_filter.launch）：                           │
│    base_frame     = gimbal_frame  (本体坐标系，grid 发布基准)     │
│    laser_frame    = aft_mapped    (HDL 定位输出坐标系)            │
│    scan_topic_left = /livox/lidar                                │
│    single_lidar_mode = true                                      │
│    first_RADIUS   = 0.4  m  (内圆过滤，去机器人自遮挡)           │
│    second_RADIUS  = 0.4  m  (中间圈半径，当前未激活)             │
│    start_height   = 0.3  m  (中间圈高度阈值)                     │
│    max_height     = 0.3  m  (外圈最大保留高度)                   │
│                                                                 │
│  ▲ 历史故障根因（已修复）：                                        │
│    原版订阅 livox_ros_driver2/CustomMsg（旧驱动格式），但当前      │
│    livox_ros_driver2 v1.2.4 发布 sensor_msgs/PointCloud2，       │
│    类型不匹配导致回调从不触发，/test_scan 始终空白，              │
│    整条动态避障链路完全失效。                                     │
│    修复：将订阅类型改为 sensor_msgs::PointCloud2::ConstPtr        │
└─────────────────────────────────────────────────────────────────┘
        │
        │  /test_scan
        │  sensor_msgs/PointCloud2, ~10 Hz, frame=gimbal_frame
        │  （已过滤的近地障碍点云，只含 z < 0.3m 的障碍物）
        ▼
┌─────────────────────────────────────────────────────────────────┐
│  dbscan_bfs_3D（节点：dbscan_bfs_3D）                            │
│  launch：dbscan_bfs_3D.launch                                    │
│                                                                 │
│  功能（三步流水线）：                                             │
│                                                                 │
│  步骤1 - DBSCAN 聚类：                                           │
│    将输入点云中的所有点送入 DBSCAN 算法                           │
│    epsilon = 0.2m：两个点距离 ≤ 0.2m 即视为邻居                   │
│    minPts  = 10：核心点需至少有 10 个邻居（过滤单点噪声）          │
│    小于 minPts 的孤立点标记为 noise，不参与后续膨胀               │
│                                                                 │
│  步骤2 - 遮挡阴影标记（obscured_point_filter）：                   │
│    使用线段树算法，对格栅中每个点计算从中心（机器人位置）出发       │
│    的角度和距离排名；若某个格子在同一角度方向有比它更近的障碍物    │
│    遮挡（cnt > 1），则标记为 -1（未知/阴影区），                   │
│    D*-Lite 遇到 -1 不更新障碍概率（不会错误清除静态地图障碍）     │
│                                                                 │
│  步骤3 - BFS 膨胀：                                              │
│    以聚类点为初始值 100，用 BFS 向外扩散：                        │
│    上下左右方向每步衰减 dfs_decrease(3)                          │
│    斜方向每步衰减 3 × 1.414 ≈ 4.2                                │
│    衰减到 0 停止；障碍核心值 100，外围逐渐衰减                     │
│    有效膨胀半径 ≈ 100/3 ≈ 33 格 × 0.05m ≈ 1.65m                 │
│                                                                 │
│  输出：100×100 格栅，resolution=0.05m，5m×5m 视野                │
│        以机器人 gimbal_frame 为中心（origin 在 -2.5m, -2.5m）     │
│        格值：100=障碍核心, 1~99=膨胀边缘, 0=空闲, -1=遮挡阴影    │
│                                                                 │
│  关键参数（来自 dbscan_bfs_3D.launch）：                          │
│    epsilon       = 0.2   (聚类邻域半径, m)                       │
│    minPts        = 10    (核心点最小邻居数)                       │
│    dfs_decrease  = 3     (BFS 每格衰减量)                        │
│    dfs_threshold = 75    (阈值参数，预留)                         │
│    child_frame   = gimbal_frame  (grid 的 frame_id)              │
│    parent_frame  = gimbal_frame                                  │
└─────────────────────────────────────────────────────────────────┘
        │
        │  /grid
        │  nav_msgs/OccupancyGrid, ~5 Hz
        │  frame=gimbal_frame, 100×100, res=0.05m
        │  max 值 100（有确认障碍），-1（遮挡阴影）
        ▼
┌─────────────────────────────────────────────────────────────────┐
│  dstarlite（D*-Lite 规划器）                                     │
│  dynamic_map_topic_name = /grid                                  │
│                                                                 │
│  处理动态地图的逻辑：                                             │
│  1. 收到 /grid 后查询 TF：map ← gimbal_frame                     │
│  2. 将格栅中每个格子的世界坐标投影到 D*-Lite 的全局地图索引       │
│  3. 若 |新 obstacle_possibility - 旧值| ≥ 5，触发增量重规划      │
│     （小幅抖动不触发，避免频繁重规划）                            │
│  4. 约 15 帧（~1.5s）内未再被 /grid 更新的格子                   │
│     自动衰减回 static_obstacle_possibility（静态地图值）          │
│  5. 障碍格的 edge_cost 按 sigmoid 曲线: L/(1+exp(-k*(op-x0)))   │
│     op=100(硬墙): cost ≈ 40.5× 基础值                           │
│     op=60(软边): cost ≈ 5×                                       │
│     op=20(自由): cost ≈ 0.5×                                     │
│                                                                 │
│  输出：/dstar_path (nav_msgs/Path), /cmd_vel (速度指令)           │
└─────────────────────────────────────────────────────────────────┘
```

#### 19.2.2 为什么之前不工作

| 问题层 | 现象 | 根因 | 修复 |
|--------|------|------|------|
| launch 层 | `threeD_lidar_filter` 和 `dbscan_bfs_3D` 从不启动 | `lidar_filter.launch` 和 `dbscan_bfs_3D.launch` 曾被重命名为 `.backup`，从未被主 launch 包含 | 恢复为 `.launch` 扩展名，并在 `3DNavUL_Test.launch` 中添加 `dynamic_avoidance_source` 条件 include |
| 消息类型层 | `/test_scan` 始终为空，`dbscan_bfs_3D` 订阅到 0 个点 | `threeD_lidar_filter.cpp` 订阅 `livox_ros_driver2/CustomMsg`（旧驱动格式），但当前 `livox_ros_driver2` v1.2.4 发布 `sensor_msgs/PointCloud2` | 修改 `threeD_lidar_filter.cpp` 订阅类型为 `sensor_msgs::PointCloud2::ConstPtr`，重新编译 |
| 验证结果 | `/test_scan` ~10Hz ✓，`/grid` ~5Hz，max=100，count_100=2126 ✓ | — | — |

---

### 19.3 ROG 二阶段 Pipeline（`/rog_grid`）

#### 19.3.1 完整数据流

```
HDL Localization（hdl_localization 节点）
        │
        ├── /aligned_points  sensor_msgs/PointCloud2
        │   （注册对齐后的点云，已在全局 map 坐标系下，~10 Hz）
        │
        └── /odom  nav_msgs/Odometry
            （机器人位姿里程计，frame=map）
        ▼
┌─────────────────────────────────────────────────────────────────┐
│  rog_map_node（Stage 1）                                         │
│  launch：rog_map_observability.launch                            │
│  config：rog_map_passive.yaml                                    │
│                                                                 │
│  功能：维护一个以机器人为中心、随机器人滑动的 3D 概率体素地图     │
│                                                                 │
│  核心算法 - Bayesian 概率光线投射：                               │
│  · 对 /aligned_points 中每条光线，从机器人位置到终点做 raycast:   │
│    终点附近 voxel：概率 × p_hit(0.70) → 增大占据概率             │
│    沿途 free voxel：概率 × (1 - p_miss(0.20)) / ... →降低占据概率│
│  · 占据概率 ≥ p_occ(0.65) 的 voxel 被标记为已占据               │
│  · 概率被上下夹在 [p_min(0.12), p_max(0.65)] 之间               │
│    → p_max=0.65 使障碍最多只需 1 条 free 光线就能降至空闲        │
│    （遗忘速度较快，移开的障碍会被清除）                           │
│                                                                 │
│  膨胀：                                                          │
│  · 每个已占据 voxel 向外膨胀 inflation_step(1) 格                │
│  · 膨胀分辨率 inflation_resolution = 0.2m                        │
│  · 膨胀半径 = 1 × 0.2m = 0.2m（约机器人半身宽）                  │
│                                                                 │
│  地图滑动（map_sliding）：                                        │
│  · 机器人移动 ≥ 0.1m 时，地图边界随之滑动                        │
│  · 滑出窗口的老 voxel 被驱逐，不再累积错误历史                    │
│                                                                 │
│  发布：                                                          │
│  · /rog_map_node/rog_map/inf_occ                                 │
│    类型：sensor_msgs/PointCloud2                                  │
│    内容：所有当前膨胀占据 voxel 的中心点，frame_id="world"        │
│    （ROG-Map 源码硬编码 "world"；由 rog_map_world_link 节点       │
│     发布 map→world 的 identity 静态 TF 来桥接两个坐标系名称）     │
│                                                                 │
│  关键参数（rog_map_passive.yaml）：                               │
│    resolution           = 0.1   (voxel 分辨率, m)               │
│    inflation_resolution = 0.2   (膨胀 voxel 分辨率, m)           │
│    inflation_step       = 1     (膨胀步数 = 1×0.2 = 0.2m 半径)  │
│    map_size             = [30, 30, 8]  (地图范围, m)             │
│    virtual_ceil_height  = 5.0   (必须 > hdl /odom z 漂移量)      │
│    virtual_ground_height= -1.0  (必须 < hdl /odom z ~-0.01)      │
│    p_hit / p_miss       = 0.70 / 0.20 (光线投射概率模型)          │
│    p_max / p_occ        = 0.65 / 0.65 (概率天花板=容易遗忘)       │
│    ray_range            = [0.3, 12.0]  (有效光线范围, m)          │
│    cloud_topic          = /aligned_points  (HDL 对齐点云)         │
│    odom_topic           = /odom                                   │
│    map_sliding.threshold= 0.1   (0.1m 位移就触发地图滑动)         │
└─────────────────────────────────────────────────────────────────┘
        │
        │  /rog_map_node/rog_map/inf_occ
        │  sensor_msgs/PointCloud2, ~10 Hz, frame="world"
        │  （膨胀占据 voxel 的中心点集合，已在全局坐标系）
        ▼
┌─────────────────────────────────────────────────────────────────┐
│  rog_map_to_grid_node（Stage 2 Bridge）                          │
│  launch：rog_map_costmap_bridge.launch                           │
│                                                                 │
│  功能：将 3D 点云膨胀地图投影为 2D OccupancyGrid，                │
│        供 D*-Lite 使用                                           │
│                                                                 │
│  算法：                                                          │
│  1. 以 /odom 中机器人 (rx, ry) 为中心，构建 10m×10m 格栅         │
│  2. origin 对齐到 ROG-Map 的 0.2m voxel lattice 整数倍：         │
│     origin_x = floor((rx - 5.0) / 0.2) × 0.2                   │
│     → 确保每个 voxel 中心恰好落在格栅的 4×4 格（0.2/0.05=4）中心 │
│  3. 遍历 /inf_occ 点云中每个点 (px, py, pz)：                    │
│     · 若 z_min(0.05) ≤ pz ≤ z_max(2.0)，则该点投影到格栅        │
│     · 对应格子：ix = (px - origin_x) / 0.05，iy = 类似           │
│     · 以该格子为中心，paint 4×4 区域 = 值 100                    │
│     · 再向外画 soft_rim_cells(2) 格软边缘 = 值 60                │
│  4. 若 /inf_occ 超过 cloud_timeout(0.5s) 未更新，发布全零格栅    │
│     → D*-Lite merge 后静态障碍依然有效（不被错误清除）            │
│  5. 每秒 heartbeat_hz(1Hz) 强制发布一次（保活）                   │
│                                                                 │
│  关键参数（rog_map_costmap_bridge.launch）：                      │
│    res            = 0.05  (格栅分辨率，必须 = 静态地图分辨率)     │
│    window_m       = 10.0  (格栅窗口尺寸, m)                      │
│    voxel_lattice  = 0.2   (必须 = ROG-Map inflation_resolution)  │
│    z_min / z_max  = 0.05 / 2.0  (有效障碍高度范围, m)            │
│    soft_rim_cells = 2     (软边缘格数)                            │
│    soft_rim_value = 60    (软边缘格值，让规划器倾向绕行)           │
│    cloud_timeout  = 0.5   (点云超时秒数)                          │
│    cloud_in  ← remap ← /rog_map_node/rog_map/inf_occ            │
│    odom_in   ← remap ← /odom                                    │
│    grid_out  → remap → /rog_grid                                 │
│                                                                 │
│  ⚠ 重要约束：                                                    │
│    res 必须与 map_server 加载的静态 PGM 地图分辨率 (0.05m) 一致！ │
│    若不一致，D*-Lite 内部投影出现间隔条纹，路径可穿越障碍墙        │
└─────────────────────────────────────────────────────────────────┘
        │
        │  /rog_grid
        │  nav_msgs/OccupancyGrid, ~10 Hz
        │  frame="world"，10m×10m, res=0.05m
        │  格值：100=硬障碍, 60=软边缘, 0=空闲
        ▼
┌─────────────────────────────────────────────────────────────────┐
│  dstarlite（D*-Lite 规划器）                                     │
│  dynamic_map_topic_name = /rog_grid                              │
│                                                                 │
│  处理与 /grid 完全相同：                                         │
│  · 通过 TF 查询 map ← world（identity，由 rog_map_world_link      │
│    静态 TF 节点在 rog_map_observability.launch 中发布）          │
│  · 投影到全局地图坐标，触发增量重规划                             │
└─────────────────────────────────────────────────────────────────┘
```

#### 19.3.2 ROG-Map 的 "world" 坐标系说明

ROG-Map 源码将可视化话题（包括 `inf_occ`）的 `frame_id` 硬编码为 `"world"`，无法通过参数修改。为让 D\*-Lite（使用 `map` 坐标系）能通过 TF 正确投影，`rog_map_observability.launch` 中启动了一个静态 TF 广播节点：

```xml
<node pkg="tf2_ros" type="static_transform_publisher"
      name="rog_map_world_link"
      args="0 0 0 0 0 0 map world"/>
```

这是一个 identity TF（平移=0, 旋转=0），相当于声明 `world` 坐标系与 `map` 坐标系完全重合。对于本系统（HDL Localization 全局地图从不移动），这个假设是成立的。

---

### 19.4 两条 Pipeline 详细对比

| 对比维度 | DBSCAN Pipeline (`/grid`) | ROG 二阶段 Pipeline (`/rog_grid`) |
|---------|--------------------------|----------------------------------|
| **数据源** | 原始 `/livox/lidar`（本地点云） | HDL 定位后 `/aligned_points`（全局对齐点云） |
| **坐标系** | `gimbal_frame`（机器人相对坐标） | `world`/`map`（全局绝对坐标） |
| **障碍历史记忆** | 无（单帧，约 1.5s 后自动衰减） | 有（Bayesian 概率更新，被遮挡后保留一段时间） |
| **3D 高度感知** | 简单截断（z < 0.3m） | 可配置（`z_min=0.05` ~ `z_max=2.0`） |
| **计算延迟** | 低（简单几何滤波 + DBSCAN） | 较高（光线投射 + 概率更新 + 坐标投影） |
| **发布频率** | `/test_scan` ~10Hz, `/grid` ~5Hz | `/inf_occ` ~10Hz, `/rog_grid` ~10Hz |
| **抗遮挡能力** | 弱（障碍转出视野立即消失） | 强（短暂遮挡后仍保留占据概率） |
| **受定位精度影响** | 较小（本体坐标，不需精确全局位姿） | 较大（依赖 HDL 定位稳定，位姿漂移会错移障碍） |
| **配置复杂度** | 低（5 个参数） | 较高（20+ 参数，需调 Bayesian 概率模型） |
| **已验证状态** | ✅ 实测通过，`/grid` 有正确障碍数据 | ⚠ 代码完整但尚未完整实车测试 |
| **适用场景** | 快速移动障碍（人、其他机器人） | 半静态障碍（设备箱、临时遮挡物） |
| **主要优点** | 轻量简单，延迟低，经过充分调试 | 全局一致，障碍记忆，不易漏检 |
| **主要缺点** | 无记忆，障碍转出视野即消失 | 需 HDL 稳定定位；配置参数多；`res` 必须严格对齐 |

---

### 19.5 启动方式

```bash
# 方式 1：DBSCAN Pipeline（默认，已验证可用）
cd /home/sentry_train_test/AstarTraining/Old_nav
./start_3d_nav_ul_test.sh
# 等同于：
./start_3d_nav_ul_test.sh dynamic_avoidance_source:=dbscan

# 方式 2：ROG 二阶段 Pipeline（需 HDL 定位稳定后再启用）
./start_3d_nav_ul_test.sh dynamic_avoidance_source:=rog

# 方式 3：无动态避障（仅静态地图，调试用）
./start_3d_nav_ul_test.sh dynamic_avoidance_source:=none
```

切换后可通过以下命令验证话题是否在发布：

```bash
# DBSCAN Pipeline 验证
rostopic hz /test_scan      # 应约 10 Hz
rostopic hz /grid           # 应约 5 Hz
rostopic echo /grid | grep max  # 应有 100

# ROG Pipeline 验证
rostopic hz /rog_map_node/rog_map/inf_occ  # 应约 10 Hz
rostopic hz /rog_grid                       # 应约 10 Hz
```

---

### 19.6 关键代码文件速查

| 文件 | 作用 |
|------|------|
| [sim_nav/src/bot_sim/src/threeD_lidar_filter.cpp](sim_nav/src/bot_sim/src/threeD_lidar_filter.cpp) | DBSCAN 链路第 1 步：PointCloud2 高度/距离过滤，输出 `/test_scan` |
| [sim_nav/src/bot_sim/src/dbscan_bfs_3D.cpp](sim_nav/src/bot_sim/src/dbscan_bfs_3D.cpp) | DBSCAN 链路第 2 步：DBSCAN 聚类 + 遮挡滤波 + BFS 膨胀，输出 `/grid` |
| [sim_nav/src/bot_sim/src/dstarlite.cpp](sim_nav/src/bot_sim/src/dstarlite.cpp) | D\*-Lite 规划器，通过 `dynamic_map_topic_name` 参数订阅 `/grid` 或 `/rog_grid` |
| [sim_nav/src/bot_sim/src/rog_map_node.cpp](sim_nav/src/bot_sim/src/rog_map_node.cpp) | ROG 链路 Stage 1：构造 ROGMap 实例，输入 `/aligned_points`+`/odom`，输出 `inf_occ` |
| [sim_nav/src/bot_sim/src/rog_map_to_grid_node.cpp](sim_nav/src/bot_sim/src/rog_map_to_grid_node.cpp) | ROG 链路 Stage 2：3D PointCloud2 → 2D OccupancyGrid，输出 `/rog_grid` |
| [sim_nav/src/bot_sim/launch_real/lidar_filter.launch](sim_nav/src/bot_sim/launch_real/lidar_filter.launch) | 启动 `threeD_lidar_filter`，配置过滤参数 |
| [sim_nav/src/bot_sim/launch_real/dbscan_bfs_3D.launch](sim_nav/src/bot_sim/launch_real/dbscan_bfs_3D.launch) | 启动 `dbscan_bfs_3D`，配置 DBSCAN 参数 |
| [sim_nav/src/bot_sim/launch_real/rog_map_observability.launch](sim_nav/src/bot_sim/launch_real/rog_map_observability.launch) | 启动 `rog_map_node` + `world→map` identity TF |
| [sim_nav/src/bot_sim/launch_real/rog_map_costmap_bridge.launch](sim_nav/src/bot_sim/launch_real/rog_map_costmap_bridge.launch) | 启动 `rog_map_to_grid_node`，配置 bridge 参数 |
| [sim_nav/src/bot_sim/config/rog_map_passive.yaml](sim_nav/src/bot_sim/config/rog_map_passive.yaml) | ROG-Map 全部参数（概率模型、地图尺寸、光线投射范围等） |
| [3DNavUL_Test.launch](3DNavUL_Test.launch) | 主 launch，`dynamic_avoidance_source` 参数控制选择哪条 pipeline |
