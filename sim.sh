cd /data/projects/AGV_sim_ws

source /opt/ros/humble/setup.bash
source /data/projects/autoware/install/setup.bash
source /data/projects/AGV_sim_ws/install/setup.bash

ros2 launch agv_bringup stage3_autoware_planning_control.launch.py use_rviz:=true use_gazebo:=true