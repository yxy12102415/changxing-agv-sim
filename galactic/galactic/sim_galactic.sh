#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WS_DIR}"

source /opt/ros/galactic/setup.bash
if [ ! -f install_galactic/setup.bash ]; then
  cat >&2 <<EOF
install_galactic/setup.bash not found.

Build this Galactic workspace inside the container first:

  colcon --log-base log_galactic build \\
    --build-base build_galactic \\
    --install-base install_galactic \\
    --symlink-install

Then run:

  ./galactic/sim_galactic.sh

EOF
  exit 1
fi

source install_galactic/setup.bash

ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
  use_rviz:=true \
  use_autoware_map:=false \
  "$@"
