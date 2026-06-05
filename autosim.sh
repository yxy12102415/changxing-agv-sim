cd /data/projects/AGV_sim_ws

source /opt/ros/humble/setup.bash
source /data/projects/autoware/install/setup.bash 2>/tmp/autoware_sim_setup_warnings.log
source /data/projects/AGV_sim_ws/install/setup.bash 2>/tmp/agv_autosim_setup_warnings.log

ros2 launch agv_bringup autoware_gazebo_sim.launch.py "$@"
