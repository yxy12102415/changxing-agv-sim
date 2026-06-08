#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

if [ ! -f /opt/ros/galactic/setup.bash ]; then
  if ! docker info >/dev/null 2>&1 && getent group docker | awk -F: -v user="${USER}" '
    $1 == "docker" {
      split($4, members, ",")
      for (i in members) {
        if (members[i] == user) {
          found = 1
        }
      }
    }
    END { exit found ? 0 : 1 }
  '; then
    exec sg docker -c "$(printf "%q " "$0" "$@")"
  fi
  exec ./galactic/tools/galactic_docker.sh sim "$@"
fi

set +u
source /opt/ros/galactic/setup.bash
set -u

if [ ! -f install_galactic/setup.bash ]; then
  colcon --log-base log_galactic build \
    --build-base build_galactic \
    --install-base install_galactic \
    --symlink-install
fi

set +u
source install_galactic/setup.bash
set -u

ros2 launch agv_gazebo agv_gazebo_sim.launch.py \
  use_rviz:=true \
  use_autoware_map:=false \
  "$@"
