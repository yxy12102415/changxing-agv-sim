from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")
    projector_info_path = LaunchConfiguration("projector_info_path")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    steering_mode = LaunchConfiguration("steering_mode")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    default_map_path = PathJoinSubstitution(
        [FindPackageShare("agv_maps"), "map", "changxing_v1.osm"]
    )
    default_projector_info_path = PathJoinSubstitution(
        [FindPackageShare("agv_maps"), "map", "map_projector_info.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("autoware_launch"), "rviz", "autoware.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_path", default_value=default_map_path),
            DeclareLaunchArgument(
                "projector_info_path", default_value=default_projector_info_path
            ),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("steering_mode", default_value="crab"),
            DeclareLaunchArgument("initial_x", default_value="0.0674"),
            DeclareLaunchArgument("initial_y", default_value="-57.6716"),
            DeclareLaunchArgument("initial_yaw", default_value="-0.7297"),
            Node(
                package="autoware_map_projection_loader",
                executable="autoware_map_projection_loader_node",
                name="map_projection_loader",
                output="screen",
                parameters=[
                    {
                        "map_projector_info_path": projector_info_path,
                        "lanelet2_map_path": map_path,
                    }
                ],
            ),
            Node(
                package="autoware_map_loader",
                executable="autoware_lanelet2_map_loader",
                name="lanelet2_map_loader",
                output="screen",
                parameters=[
                    {
                        "allow_unsupported_version": True,
                        "center_line_resolution": 5.0,
                        "use_waypoints": True,
                        "lanelet2_map_path": map_path,
                    }
                ],
            ),
            Node(
                package="agv_map_visualizer",
                executable="lanelet2_marker_publisher",
                name="lanelet2_marker_publisher",
                output="screen",
            ),
            Node(
                package="agv_vehicle_model",
                executable="agv_two_axis_simulator_node",
                name="agv_two_axis_simulator",
                output="screen",
                parameters=[
                    {
                        "frame_id": "map",
                        "child_frame_id": "base_link",
                        "steering_mode": steering_mode,
                        "dt": 0.025,
                        "vx_lim": 5.0,
                        "steer_lim": 0.7,
                        "vx_rate_lim": 2.0,
                        "steer_rate_lim": 1.5,
                        "wheelbase_f": 1.0,
                        "wheelbase_r": 1.0,
                        "vx_delay": 0.1,
                        "vx_time_constant": 0.2,
                        "steer_delay": 0.1,
                        "steer_time_constant": 0.2,
                        "steer_dead_band": 0.0,
                        "steer_bias": 0.0,
                        "initial_x": initial_x,
                        "initial_y": initial_y,
                        "initial_yaw": initial_yaw,
                    }
                ],
            ),
            Node(
                package="agv_map_visualizer",
                executable="agv_vehicle_marker_publisher",
                name="agv_vehicle_marker_publisher",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/localization/kinematic_state",
                        "output_topic": "/visualization/agv_vehicle_marker",
                        "length": 3.0,
                        "width": 1.0,
                        "height": 0.8,
                        "wheelbase": 2.0,
                        "wheel_tread": 1.36,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
