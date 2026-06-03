import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")
    steering_mode = LaunchConfiguration("steering_mode")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo = LaunchConfiguration("use_gazebo")
    data_path = LaunchConfiguration("data_path")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    rviz_config = LaunchConfiguration("rviz_config")

    default_map_path = PathJoinSubstitution([FindPackageShare("agv_maps"), "map"])
    stage2_launch = PathJoinSubstitution(
        [FindPackageShare("agv_bringup"), "launch", "stage2_two_axis_sim.launch.py"]
    )
    autoware_launch = PathJoinSubstitution(
        [FindPackageShare("autoware_launch"), "launch", "autoware.launch.xml"]
    )
    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("agv_gazebo"), "launch", "gazebo_follow_autoware.launch.py"]
    )
    rviz_library_path = os.pathsep.join(
        path for path in os.environ.get("LD_LIBRARY_PATH", "").split(os.pathsep)
        if path and not path.startswith("/snap/")
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("agv_gazebo"), "config", "stage4_lidar.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_path", default_value=default_map_path),
            DeclareLaunchArgument("steering_mode", default_value="crab"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_gazebo", default_value="true"),
            DeclareLaunchArgument("data_path", default_value="/home/yxy/autoware_data/ml_models"),
            DeclareLaunchArgument("initial_x", default_value="0.0674"),
            DeclareLaunchArgument("initial_y", default_value="-57.6716"),
            DeclareLaunchArgument("initial_yaw", default_value="-0.7297"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(stage2_launch),
                        launch_arguments={
                            "map_path": PathJoinSubstitution([map_path, "changxing_v1.osm"]),
                            "projector_info_path": PathJoinSubstitution(
                                [map_path, "map_projector_info.yaml"]
                            ),
                            "steering_mode": steering_mode,
                            "use_rviz": "false",
                            "initial_x": initial_x,
                            "initial_y": initial_y,
                            "initial_yaw": initial_yaw,
                        }.items(),
                    )
                ],
                scoped=True,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "initial_x": initial_x,
                    "initial_y": initial_y,
                    "initial_yaw": initial_yaw,
                }.items(),
                condition=IfCondition(use_gazebo),
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(autoware_launch),
                launch_arguments={
                    "map_path": map_path,
                    "lanelet2_map_file": "changxing_v1.osm",
                    "pointcloud_map_file": "pointcloud_map.pcd",
                    "vehicle_model": "sample_vehicle",
                    "sensor_model": "sample_sensor_kit",
                    "data_path": data_path,
                    "planning_module_preset": "default",
                    "control_module_preset": "default",
                    "launch_vehicle": "false",
                    "launch_vehicle_interface": "false",
                    "launch_system": "true",
                    "launch_map": "false",
                    "launch_sensing": "false",
                    "launch_localization": "false",
                    "launch_perception": "false",
                    "launch_planning": "true",
                    "launch_control": "true",
                    "launch_api": "true",
                    "rviz": "false",
                    "enable_all_modules_auto_mode": "true",
                    "is_simulation": "true",
                    "use_sim_time": "false",
                }.items(),
            ),
            Node(
                package="autoware_dummy_perception_publisher",
                executable="empty_objects_publisher",
                name="empty_objects_publisher",
                output="screen",
                remappings=[
                    ("~/output/objects", "/perception/object_recognition/objects"),
                ],
            ),
            Node(
                package="topic_tools",
                executable="relay",
                name="operation_mode_state_relay",
                output="screen",
                arguments=[
                    "/control/vehicle_cmd_gate/operation_mode",
                    "/system/operation_mode/state",
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_stage3",
                output="screen",
                additional_env={"LD_LIBRARY_PATH": rviz_library_path},
                arguments=[
                    "-d",
                    rviz_config,
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
