# AGV Simulation Workspace

ROS 2 Galactic Docker workspace for the Changxing AGV simulation. It keeps the
standalone Gazebo/RViz simulation runnable in a Galactic container while the
host stays on Humble.

## Contents

- `src/agv_gazebo`: Gazebo world, AGV model with four lidars, dynamic obstacles,
  Gazebo/ROS bridge, and RViz launch support.
- `src/agv_vehicle_model`: two-axis steering simulator node and keyboard teleop.
- `src/agv_map_visualizer`: Lanelet2, vehicle, and sampled lidar marker
  publishers for RViz.
- `src/agv_mpc_controller`: centerline trajectory publisher and lightweight
  sampling MPC tracker for the standalone simulation.
- `src/autoware_*_msgs`: local Autoware-style message definitions used by the
  simulator interface. This workspace is Autoware-style, not a full Autoware
  checkout.
- `src/agv_maps/map/changxing_v1.osm`: Changxing Lanelet2-style vector map.
- `src/agv_bringup`: Autoware + Gazebo integration launch files and route helper.

## Build

The usual path is to let `sim.sh` build on first run. To build manually:

```bash
cd /data/projects/changxing-agv-sim/galactic
./galactic/tools/galactic_docker.sh build
./docker.sh
```

Inside the container:

```bash
cd /workspace/AGV_sim_ws
colcon --log-base log_galactic build \
  --build-base build_galactic \
  --install-base install_galactic \
  --symlink-install
```

The workspace is normally built with `--symlink-install`, so launch/config edits
under `src/` are reflected through `install_galactic/` without rebuilding.
Rebuild after changing C++ code.

## Run Standalone Gazebo

From the host:

```bash
cd /data/projects/changxing-agv-sim/galactic
./sim.sh
```

`sim.sh` starts the Galactic Docker container with GUI support, rebuilds the
workspace on first run if `install_galactic` is missing, then launches:

```bash
ros2 launch agv_gazebo agv_gazebo_sim.launch.py
```

It starts Gazebo, the AGV two-axis simulator, Gazebo pose follower, dynamic
traffic/pedestrian obstacles, standalone map visualization, centerline
trajectory publishing, sampling MPC control, RViz, and visualization marker
nodes.

Useful launch arguments:

```bash
./sim.sh use_rviz:=false
./sim.sh use_gazebo_gui:=false
./sim.sh enable_mpc:=false
./sim.sh initial_x:=0.0674 initial_y:=-57.6716 initial_yaw:=-0.7297
./sim.sh steering_mode:=crab
```

MPC tracking errors are written by default inside the container to:

```text
/tmp/agv_mpc_error.csv
```

The main standalone control chain is:

```text
centerline_trajectory_publisher
-> sampling_mpc_controller
-> /control/command/control_cmd
-> agv_two_axis_simulator_node
-> /localization/kinematic_state
```

If you enter the container manually with:

```bash
./galactic/tools/galactic_docker.sh gui
```

do not run `galactic_docker.sh gui` again from inside the container. The prompt
will look like `agv@...:/workspace/AGV_sim_ws$`; from there run:

```bash
bash galactic/sim_galactic.sh
```

Galactic's Ignition 5 GPU lidar server still needs an X display even when
`use_gazebo_gui:=false`. If there is no `DISPLAY`, Ogre2 can fail to initialize.
Use the host `./sim.sh` wrapper or `galactic_docker.sh gui` so X11 is passed
through. Some `libGL` driver warnings in Docker are expected as long as the
lidar topics and MPC error log continue updating.

To stop the running simulation container:

```bash
sg docker -c 'docker rm -f agv-sim-galactic-sim'
```

To enter the Galactic container without launching simulation:

```bash
./docker.sh
./docker.sh gui
./docker.sh root
```

## Run With Autoware

Autoware integration is not the primary Galactic Docker path yet. The current
Galactic target is the standalone Gazebo/RViz chain; keep Humble as the
Autoware reference until the Galactic Autoware interfaces are collected and
adapted.

There is no Galactic `autosim.sh` one-command entry yet. The Humble workspace is
still the reference for Autoware planning/control integration.

## Control Interfaces

The standalone simulator and Autoware launch are aligned on the same control
topics:

- `/control/command/control_cmd`: `autoware_control_msgs/msg/Control`
- `/control/command/gear_cmd`: `autoware_vehicle_msgs/msg/GearCommand`
- `/control/command/turn_indicators_cmd`:
  `autoware_vehicle_msgs/msg/TurnIndicatorsCommand`
- `/control/command/hazard_lights_cmd`:
  `autoware_vehicle_msgs/msg/HazardLightsCommand`
- `/vehicle/status/control_mode`: `autoware_vehicle_msgs/msg/ControlModeReport`
- `/control/control_mode_request`: `autoware_vehicle_msgs/srv/ControlModeCommand`
- `/api/operation_mode/state` and `/system/operation_mode/state`:
  `autoware_adapi_v1_msgs/msg/OperationModeState`

The simulator also accepts legacy `/vehicle/engage`
(`autoware_vehicle_msgs/msg/Engage`), but Autoware mode changes primarily use
the AD API operation mode services.

## Dynamic Obstacles

`dynamic_obstacle_simulator_node` animates the moving vehicles and crossing
pedestrians by calling:

```text
/world/changxing_empty/set_pose
```

The entities are defined in `src/agv_gazebo/worlds/changxing_empty.sdf`:

- `moving_vehicle_1`
- `moving_vehicle_2`
- `crossing_pedestrian_1` through `crossing_pedestrian_6`

If these models flicker or jump, first check for stale Gazebo or duplicate
simulation processes:

```bash
ps -ef | rg "ros2 launch|ign gazebo|gz sim|parameter_bridge|dynamic_obstacle|rviz2"
```

If an old `ign gazebo server` remains after closing a launch, stop it before
starting a new simulation:

```bash
kill <pid>
```

Avoid running multiple `sim.sh` instances at the same time unless they use
separate ROS/Gazebo domains and worlds.

## Lidar And RViz Notes

RViz uses `map` as the fixed frame. The AGV simulator publishes `map ->
base_link`, and the launch files publish static transforms from `base_link` to
the sensor frames.

GNSS and IMU topics:

- `/chnav/fix`: `sensor_msgs/msg/NavSatFix`, frame `gnss_link`
- `/chnav/imu/data`: `sensor_msgs/msg/Imu`, frame `imu_link`

GNSS and IMU frames:

- `imu_link`
- `gnss_link`

Lidar topics and frames:

- `/rslidar_points_2`: right-front Helios-32, frame `rslidar2`
- `/rslidar_points_4`: left-rear Helios-32, frame `rslidar4`
- `/hesai_left_front`: left-front Pandar32, frame `hesai_left_front`
- `/hesai_right_rear`: right-rear Pandar32, frame `hesai_right_rear`

The IMU/GNSS extrinsics are launch arguments such as `imu_x`, `imu_y`, `imu_z`,
`gnss_x`, `gnss_y`, and `gnss_z`. These static transforms must match the sensor
poses in the Gazebo SDF model or real calibration. A height or frame mismatch can
make lidar markers appear offset or flickery in RViz.

Lidar extrinsics are launch arguments too, using each frame prefix:
`rslidar2_*`, `rslidar4_*`, `hesai_left_front_*`, and `hesai_right_rear_*`,
where `*` is `x`, `y`, `z`, `yaw`, `pitch`, or `roll`.

## ROS 2 Galactic Migration

The real AGV runs ROS 2 Galactic. Keep the Humble workspace as the reference and
use Docker for the Galactic migration so the host environment stays clean.

Migration guide:

```bash
less galactic/ROS2_GALACTIC_MIGRATION.md
```

Build and run the Galactic Docker simulation:

```bash
./galactic/tools/galactic_docker.sh build
./sim.sh
```

Enter the Galactic Docker environment without launching simulation:

```bash
./galactic/tools/galactic_docker.sh build
./docker.sh
```

Run `shell` only after `build` completes successfully. If Docker Hub times out,
retry the build by itself. A fallback base image can be selected with:

```bash
AGV_GALACTIC_BASE_IMAGE=ros:galactic-ros-base ./galactic/tools/galactic_docker.sh build
```

If Docker Hub itself is unreachable, try a mirror/proxy base image:

```bash
AGV_GALACTIC_BASE_IMAGE=dockerproxy.net/osrf/ros:galactic-desktop ./galactic/tools/galactic_docker.sh build
```

If the error is an IPv6 timeout such as `dial tcp [2a03:...]:443: i/o timeout`,
run the temporary host workaround and retry:

```bash
./galactic/tools/docker_ipv4_temporarily.sh
./galactic/tools/galactic_docker.sh build
```

Inside the container, start with:

```bash
source /opt/ros/galactic/setup.bash
cd /workspace/AGV_sim_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --build-base build_galactic \
  --install-base install_galactic \
  --log-base log_galactic
```

For an interactive GUI container without auto-launching the sim, use:

```bash
./docker.sh gui
```

Interface collection script for a Galactic vehicle/development machine:

```bash
./tools/collect_galactic_interfaces.sh
```

The report is written to `/tmp/agv_galactic_interface_report`. Use it to decide
whether the simulation can be remapped directly or needs a Galactic adapter.
