# AGV Simulation Workspace

ROS 2 Humble workspace for the Changxing AGV simulation. It contains the Gazebo
world, Lanelet2 map, RViz visualization helpers, a two-axis AGV vehicle model,
and launch files for running either standalone simulation or Autoware planning
and control against the same Gazebo scene.

## Contents

- `src/agv_gazebo`: Gazebo world, AGV model with four lidars, dynamic obstacles,
  Gazebo/ROS bridge, and RViz launch support.
- `src/agv_vehicle_model`: two-axis steering simulator node and keyboard teleop.
- `src/agv_map_visualizer`: Lanelet2, vehicle, and sampled lidar marker
  publishers for RViz.
- `src/agv_maps/map/changxing_v1.osm`: Changxing Lanelet2-style vector map.
- `src/agv_bringup`: Autoware + Gazebo integration launch files and route helper.

## Build

```bash
source /opt/ros/humble/setup.bash
cd /data/projects/AGV_sim_humble
colcon build --symlink-install
source install/setup.bash
```

The workspace is normally built with `--symlink-install`, so launch/config edits
under `src/` are reflected through `install/` without rebuilding. Rebuild after
changing C++ code.

## Run Standalone Gazebo

```bash
./sim.sh
```

This sources ROS Humble and this workspace, then launches:

```bash
ros2 launch agv_gazebo agv_gazebo_sim.launch.py
```

It starts Gazebo, the AGV two-axis simulator, Gazebo pose follower, dynamic
traffic/pedestrian obstacles, map loaders, RViz, and visualization marker nodes.

Useful launch arguments:

```bash
./sim.sh use_rviz:=false
./sim.sh initial_x:=0.0674 initial_y:=-57.6716 initial_yaw:=-0.7297
./sim.sh steering_mode:=crab
```

## Run With Autoware

```bash
./autosim.sh
```

This additionally sources `/data/projects/autoware/install/setup.bash` and
launches:

```bash
ros2 launch agv_bringup autoware_gazebo_sim.launch.py
```

`autosim.sh` includes the same Gazebo launch used by `sim.sh`, then starts
Autoware planning, control, API, and system modules. Autoware vehicle and vehicle
interface launches are disabled because the local `agv_two_axis_simulator_node`
provides the simulated vehicle interface.

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

Avoid running `sim.sh` and `autosim.sh` at the same time unless they use separate
ROS/Gazebo domains and worlds.

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

