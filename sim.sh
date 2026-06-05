cd /data/projects/AGV_sim_ws

source /opt/ros/humble/setup.bash
source /data/projects/AGV_sim_ws/install/setup.bash 2>/tmp/agv_sim_setup_warnings.log

ros2 launch agv_gazebo agv_gazebo_sim.launch.py "$@"
