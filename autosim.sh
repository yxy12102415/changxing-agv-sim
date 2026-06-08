#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

source /opt/ros/humble/setup.bash
source /data/projects/autoware/install/setup.bash 2>/tmp/autoware_sim_setup_warnings.log
source "${SCRIPT_DIR}/install/setup.bash" 2>/tmp/agv_autosim_setup_warnings.log

ros2 launch agv_bringup autoware_gazebo_sim.launch.py "$@"
