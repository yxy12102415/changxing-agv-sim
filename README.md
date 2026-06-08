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
./sim.sh
```

If the Docker image has not been built yet:

```bash
cd galactic
./galactic/tools/galactic_docker.sh build
./sim.sh
```

`galactic/sim.sh` runs from the host, starts the Galactic Docker container with
GUI support, builds the workspace on first run if `install_galactic` is missing,
and launches the standalone Gazebo/RViz simulation.
