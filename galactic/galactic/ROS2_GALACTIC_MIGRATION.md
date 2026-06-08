# ROS 2 Galactic 仿真迁移执行说明

目标：先不管真实 AGV 接口，先把当前 standalone 仿真迁到 ROS 2 Galactic
Docker 环境里。也就是说，优先让 `sim.sh` 这条链路在 Galactic 容器中逐步跑通：

```text
Gazebo/Ignition world
-> lidar bridge
-> agv_two_axis_simulator_node
-> kinematic_state_to_gazebo_node
-> dynamic_obstacle_simulator_node
-> RViz/map/marker
```

当前宿主机只有 Humble，没有 Galactic，所以采用 Docker，避免污染现有环境。

## Docker 方案

已新增：

- `galactic/docker/Dockerfile.galactic`
- `tools/galactic_docker.sh`

如果宿主机还没有 Docker，先安装：

```bash
sudo apt update
sudo apt install -y docker.io
sudo usermod -aG docker "$USER"
newgrp docker
```

如果 `sudo` 要求密码，需要你在终端里手动执行。安装完以后再继续下面步骤。

构建 Galactic 镜像：

```bash
./galactic/tools/galactic_docker.sh build
```

如果 Docker Hub 网络超时，先只重试 build，不要连着执行 shell：

```bash
./galactic/tools/galactic_docker.sh build
```

如果报错里出现类似下面的 IPv6 timeout：

```text
dial tcp [2a03:...]:443: i/o timeout
```

先在宿主机运行临时 IPv4 workaround：

```bash
./galactic/tools/docker_ipv4_temporarily.sh
```

它会临时禁用 IPv6 并重启 Docker。这个修改不写入永久配置，重启系统后会恢复。

如果 `osrf/ros:galactic-desktop` 拉不到，也可以换 ROS 官方基础镜像：

```bash
AGV_GALACTIC_BASE_IMAGE=ros:galactic-ros-base ./galactic/tools/galactic_docker.sh build
```

如果 Docker Hub registry 完全超时，可以临时用镜像代理拉基础镜像：

```bash
AGV_GALACTIC_BASE_IMAGE=dockerproxy.net/osrf/ros:galactic-desktop ./galactic/tools/galactic_docker.sh build
```

如果你的网络或单位有指定 Docker registry mirror，优先使用自己的镜像源。

如果拉取镜像 layer 经常卡住，先把 Docker 下载并发降到 1：

```bash
./tools/docker_slow_network_mode.sh
```

然后重试小镜像：

```bash
AGV_GALACTIC_BASE_IMAGE=dockerproxy.net/library/ros:galactic-ros-base ./galactic/tools/galactic_docker.sh build
```

如果 ROS base 镜像仍然卡住，可以改用 Ubuntu 20.04 基础镜像，在镜像里通过
apt 安装 ROS Galactic：

```bash
AGV_GALACTIC_DOCKERFILE=galactic/docker/Dockerfile.galactic-apt ./galactic/tools/galactic_docker.sh build
```

进入 Galactic 容器：

```bash
./galactic/tools/galactic_docker.sh shell
```

默认 `shell` 是低权限编译/调试容器，不使用 `--privileged`、`--net host` 或
`--ipc host`。等编译跑通后，如果要测试 Gazebo/RViz GUI，再使用：

```bash
./galactic/tools/galactic_docker.sh gui
```

容器会把当前 workspace 挂载到：

```text
/workspace/AGV_sim_ws
```

所以宿主机上编辑文件，容器里可以直接编译测试。

Galactic 容器建议使用独立的 colcon 目录，避免读取宿主机 Humble 的
`build/install/log`：

```text
build_galactic
install_galactic
log_galactic
```

## 容器内第一步

进入容器后：

```bash
source /opt/ros/galactic/setup.bash
cd /workspace/AGV_sim_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --build-base build_galactic \
  --install-base install_galactic \
  --log-base log_galactic
```

注意：第一次 `colcon build` 很可能不会直接成功。失败点预计集中在：

- `ros_gz_*` 包名在 Galactic 里可能需要换成 `ros_ign_*`
- Autoware Humble 消息包在 Galactic 里可能不存在
- `autoware_simple_planning_simulator` API 可能和 Galactic 版本不同

这是正常的。下一步应该按错误逐个降级/适配。

## 为什么先做 Docker

不要直接在当前机器安装 Galactic：

- Galactic 已经过 EOL
- Galactic 官方主要对应 Ubuntu 20.04
- 当前仿真主环境是 Humble
- 混装容易污染 apt、CMake、ament、Autoware overlay

Docker 让 Galactic 成为一次性隔离环境。迁移成功后，再决定是否单独建真实车
Galactic workspace。

## 当前包的迁移优先级

| 包 | 优先级 | 风险 |
| --- | --- | --- |
| `agv_maps` | 1 | 低，主要是 map 和 launch |
| `agv_map_visualizer` | 2 | 中，依赖 Autoware map msg/lanelet utils |
| `agv_gazebo` | 3 | 高，Gazebo bridge 包名可能不同 |
| `agv_vehicle_model` | 4 | 高，依赖 Autoware control/vehicle msg 和 simple simulator |
| `agv_bringup` | 5 | 最高，Autoware launch 版本差异最大 |

建议先让 `agv_gazebo` + `agv_vehicle_model` 的 standalone 链路跑起来，最后再碰
`agv_bringup/autosim.sh`。

## Galactic 预期修改点

### Gazebo/Ignition

当前 Humble 仿真使用：

```text
ros_gz_sim
ros_gz_bridge
ros_gz_interfaces
```

Galactic 时代常见包名可能是：

```text
ros_ign_gazebo
ros_ign_bridge
ros_ign_interfaces
```

所以这些文件大概率需要适配：

- `src/agv_gazebo/package.xml`
- `src/agv_gazebo/CMakeLists.txt`
- `src/agv_gazebo/launch/agv_gazebo_sim.launch.py`
- `src/agv_gazebo/launch/stage4_gazebo_sim.launch.py`
- `src/agv_gazebo/src/kinematic_state_to_gazebo_node.cpp`
- `src/agv_gazebo/src/dynamic_obstacle_simulator_node.cpp`

### Autoware 消息

当前车辆模型直接依赖：

```text
autoware_control_msgs
autoware_vehicle_msgs
autoware_adapi_v1_msgs
autoware_simple_planning_simulator
```

如果 Galactic Autoware 没有这些包，先不要改动力学模型，优先考虑做一层
compatibility adapter。

## 当前 Humble 控制接口

| Topic 或 Service | 类型 |
| --- | --- |
| `/control/command/control_cmd` | `autoware_control_msgs/msg/Control` |
| `/control/command/gear_cmd` | `autoware_vehicle_msgs/msg/GearCommand` |
| `/control/command/turn_indicators_cmd` | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand` |
| `/control/command/hazard_lights_cmd` | `autoware_vehicle_msgs/msg/HazardLightsCommand` |
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` |
| `/vehicle/status/gear_status` | `autoware_vehicle_msgs/msg/GearReport` |
| `/vehicle/status/control_mode` | `autoware_vehicle_msgs/msg/ControlModeReport` |
| `/api/operation_mode/state` | `autoware_adapi_v1_msgs/msg/OperationModeState` |
| `/system/operation_mode/state` | `autoware_adapi_v1_msgs/msg/OperationModeState` |
| `/control/control_mode_request` | `autoware_vehicle_msgs/srv/ControlModeCommand` |

## Standalone 仿真验证清单

在 Galactic 容器里最终要验证：

- [ ] `colcon build --symlink-install` 能通过 standalone 必需包
- [ ] Gazebo/Ignition world 能启动
- [ ] `/world/changxing_empty/set_pose` 或对应 Galactic 服务存在
- [ ] `/localization/kinematic_state` 正常发布
- [ ] `map -> base_link` TF 正常发布
- [ ] 四个 lidar point cloud topic 正常发布
- [ ] moving vehicle 和 pedestrian 只由一个动态障碍物节点更新
- [ ] RViz 中 Gazebo 模型、AGV marker、lidar marker 对齐

## 真实车接口采集

这一步先不做，但脚本已经准备好：

```bash
./tools/collect_galactic_interfaces.sh
```

等 standalone Galactic 仿真跑起来后，再用它采集真实车 topic/msg/service。

## 下一步

1. 先 build Docker 镜像。
2. 进入容器。
3. 跑 `rosdep install`。
4. 跑 `colcon build --symlink-install`。
5. 根据第一个编译错误开始做 `ros_gz` 到 `ros_ign` 或 Autoware msg 兼容。

这才是真正迁移的第一轮。
