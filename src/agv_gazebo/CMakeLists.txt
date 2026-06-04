cmake_minimum_required(VERSION 3.8)
project(agv_gazebo)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(kinematic_state_to_gazebo_node
  src/kinematic_state_to_gazebo_node.cpp
)
target_compile_features(kinematic_state_to_gazebo_node PUBLIC cxx_std_17)
ament_target_dependencies(kinematic_state_to_gazebo_node
  geometry_msgs
  nav_msgs
  rclcpp
  ros_gz_interfaces
)

add_executable(gnss_imu_simulator_node
  src/gnss_imu_simulator_node.cpp
)
target_compile_features(gnss_imu_simulator_node PUBLIC cxx_std_17)
ament_target_dependencies(gnss_imu_simulator_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
)

add_executable(dynamic_obstacle_simulator_node
  src/dynamic_obstacle_simulator_node.cpp
)
target_compile_features(dynamic_obstacle_simulator_node PUBLIC cxx_std_17)
ament_target_dependencies(dynamic_obstacle_simulator_node
  geometry_msgs
  rclcpp
  ros_gz_interfaces
)

install(
  DIRECTORY config launch models scripts worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    kinematic_state_to_gazebo_node
    gnss_imu_simulator_node
    dynamic_obstacle_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
