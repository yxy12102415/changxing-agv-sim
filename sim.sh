#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash" 2>/tmp/agv_sim_setup_warnings.log

ros2 launch agv_gazebo agv_gazebo_sim.launch.py use_autoware_map:=false "$@"
