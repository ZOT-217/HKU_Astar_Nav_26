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

## 14. 待办（后续阶段）

- [x] 网络 / 雷达上电验证（`/livox/lidar`、`/livox/imu`、`/3Dlidar`、`/cloud_registered` 已有频率）
- [x] SLAM topic 跑通（`/cloud_registered` 已恢复）
- [x] 导航定位垂直下坠修复（IMU 重力方向与 hdl_localization `invert_acc` 已校正）
- [ ] 现场移动建完整地图并保存 pcd
- [ ] 导航阶段：
  - [x] 为新机器人创建专用 `imu_filter.launch` 实测参数
  - [x] hdl_localization 初始化后 z 轴稳定性验证
  - [ ] `real_robot_transform.launch` 中 `roll_offset_deg: 180.0`
  - [ ] hdl_localization 初始化位姿（针对新建图）
  - [ ] MCU / 决策接入

---

## 15. 关键文件索引

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
| [Old_nav/start_3d_nav_ul_test.sh](start_3d_nav_ul_test.sh) | 新增免手动 source 的定位导航启动入口 |
| [Old_nav/3DNavUL_Test.launch](3DNavUL_Test.launch) | 默认 include 路径固定到 `Old_nav` 内部 Livox / bot_sim / hdl_localization |
| [Old_nav/3DNavUL_Test_with_decision.launch](3DNavUL_Test_with_decision.launch) | 导航子 launch 默认指向 `Old_nav/3DNavUL_Test.launch` |
| [Old_nav/run_3DNavUL_Test_with_decision.sh](run_3DNavUL_Test_with_decision.sh) | 从脚本目录推导 `OLD_NAV_ROOT`，不再硬编码 `/home/sentry/.../Old_nav` |
| [Old_nav/livox_ws/src/livox_ros_driver2/config/MID360_config.json](livox_ws/src/livox_ros_driver2/config/MID360_config.json) | host IP 改为 `192.168.1.50`，roll 改为 `-180.0` |
| [Old_nav/livox_ws/src/livox_ros_driver2/config/mixed_HAP_MID360_config.json](livox_ws/src/livox_ros_driver2/config/mixed_HAP_MID360_config.json) | host IP 同步改为 `192.168.1.50`，MID360 roll 同步为 `-180.0` |
| [Old_nav/sim_nav/src/bot_sim/src/threeD_lidar_merge_pointcloud.cpp](sim_nav/src/bot_sim/src/threeD_lidar_merge_pointcloud.cpp) | 新增单雷达模式与 RPY/平移输出补偿 |
| [Old_nav/sim_nav/src/bot_sim/launch_real/lidar_merge_pointcloud.launch](sim_nav/src/bot_sim/launch_real/lidar_merge_pointcloud.launch) | 新机器人 `/3Dlidar` 配置为单雷达，当前 Roll `0.0`（驱动层已补偿） |
| [Old_nav/sim_nav/src/Point-LIO/launch/mapping_avia.launch](sim_nav/src/Point-LIO/launch/mapping_avia.launch) | 新增 `point_lio_root` 参数，配置路径可由主 launch 固定到 `Old_nav` 内部 |
| [Old_nav/sim_nav/src/Point-LIO/config/avia.yaml](sim_nav/src/Point-LIO/config/avia.yaml) | PCD 保存启用导出 Roll 180° 补偿 |
| [Old_nav/sim_nav/src/Point-LIO/src/parameters.h](sim_nav/src/Point-LIO/src/parameters.h) | 新增 PCD 导出变换参数声明 |
| [Old_nav/sim_nav/src/Point-LIO/src/parameters.cpp](sim_nav/src/Point-LIO/src/parameters.cpp) | 读取 PCD 导出 RPY / 平移参数 |
| [Old_nav/sim_nav/src/Point-LIO/src/laserMapping.cpp](sim_nav/src/Point-LIO/src/laserMapping.cpp) | PCD 目录自动创建，写入失败不再崩溃；保存 PCD 前可单独应用导出变换 |
| [Old_nav/sim_nav/src/bot_sim/launch_real/imu_filter.launch](sim_nav/src/bot_sim/launch_real/imu_filter.launch) | 导航实测参数：输出静止重力约 `+9.81 m/s²`，禁用旧机器人 `-20°` tilt |
| [Old_nav/sim_nav/src/hdl_localization/launch/hdl_localization.launch](sim_nav/src/hdl_localization/launch/hdl_localization.launch) | 默认加载 `Old_nav` 内部 `hdl_graph_slam/map/innowing.pcd`，`invert_imu_acc=false` |
| [Old_nav/sim_nav/src/bot_sim/launch_real/map_server.launch](sim_nav/src/bot_sim/launch_real/map_server.launch) | 默认加载 `Old_nav` 内部 `bot_sim/map/innowing.yaml` |
| [Old_nav/sim_nav/src/bot_sim/launch_real/map_saver.launch](sim_nav/src/bot_sim/launch_real/map_saver.launch) | 默认保存 `Old_nav` 内部 `bot_sim/map/innowing` |
| [Old_nav/sim_nav/src/pcd2pgm_package/pcd2pgm/launch/pcd2pgm_hit723_test.launch](sim_nav/src/pcd2pgm_package/pcd2pgm/launch/pcd2pgm_hit723_test.launch) | 默认读取 `Old_nav` 内部 `hdl_graph_slam/map/innowing.pcd` |
