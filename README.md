# Changxing AGV Simulation

This repository keeps the Humble and Galactic simulation workspaces side by side.

## Workspaces

- `humble/`: local ROS 2 Humble Gazebo/RViz simulation.
- `galactic/`: ROS 2 Galactic Docker simulation workspace.

## Humble

```bash
cd humble
source /opt/ros/humble/setup.bash
colcon build --symlink-install
./sim.sh
```

## Galactic

From the host:

```bash
cd galactic
./galactic/docker_galatic.sh
```

Inside the Docker container:

```bash
cd /workspace/AGV_sim_ws
colcon --log-base log_galactic build \
  --build-base build_galactic \
  --install-base install_galactic \
  --symlink-install
./galactic/sim_galactic.sh
```
