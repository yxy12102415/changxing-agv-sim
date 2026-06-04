import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")
    projector_info_path = LaunchConfiguration("projector_info_path")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    reference_latitude = LaunchConfiguration("reference_latitude")
    reference_longitude = LaunchConfiguration("reference_longitude")
    reference_altitude = LaunchConfiguration("reference_altitude")
    pkg_share = get_package_share_directory("agv_gazebo")
    models_path = os.path.join(pkg_share, "models")
    world_path = os.path.join(pkg_share, "worlds", "changxing_empty.sdf")
    default_map_path = PathJoinSubstitution(
        [FindPackageShare("agv_maps"), "map", "changxing_v1.osm"]
    )
    default_projector_info_path = PathJoinSubstitution(
        [FindPackageShare("agv_maps"), "map", "map_projector_info.yaml"]
    )

    gz_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    lidar_bridges = [
        "/sensing/lidar/front_left@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/sensing/lidar/front_left/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "/sensing/lidar/front_right@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/sensing/lidar/front_right/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "/sensing/lidar/rear_left@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/sensing/lidar/rear_left/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "/sensing/lidar/rear_right@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/sensing/lidar/rear_right/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/world/changxing_empty/set_pose@ros_gz_interfaces/srv/SetEntityPose",
    ]
    lidar_static_transforms = [
        ("1.35", "0.45", "0.87", "0.785398", "ego_agv/front_left_lidar_link/front_left_lidar"),
        ("1.35", "-0.45", "0.87", "-0.785398", "ego_agv/front_right_lidar_link/front_right_lidar"),
        ("-1.35", "0.45", "0.87", "2.35619", "ego_agv/rear_left_lidar_link/rear_left_lidar"),
        ("-1.35", "-0.45", "0.87", "-2.35619", "ego_agv/rear_right_lidar_link/rear_right_lidar"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_path", default_value=default_map_path),
            DeclareLaunchArgument(
                "projector_info_path", default_value=default_projector_info_path
            ),
            DeclareLaunchArgument("initial_x", default_value="0.0674"),
            DeclareLaunchArgument("initial_y", default_value="-57.6716"),
            DeclareLaunchArgument("initial_yaw", default_value="-0.7297"),
            DeclareLaunchArgument("reference_latitude", default_value="31.2304"),
            DeclareLaunchArgument("reference_longitude", default_value="121.4737"),
            DeclareLaunchArgument("reference_altitude", default_value="0.0"),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    models_path,
                    os.pathsep,
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={"gz_args": ["-r -v 4 ", world_path]}.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="agv_lidar_bridge",
                output="screen",
                arguments=lidar_bridges,
            ),
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
                        "initial_x": initial_x,
                        "initial_y": initial_y,
                        "initial_yaw": initial_yaw,
                    }
                ],
            ),
            Node(
                package="agv_gazebo",
                executable="kinematic_state_to_gazebo_node",
                name="kinematic_state_to_gazebo",
                output="screen",
                parameters=[
                    {
                        "odom_topic": "/localization/kinematic_state",
                        "service_name": "/world/changxing_empty/set_pose",
                        "entity_name": "ego_agv",
                        "z_offset": 0.4,
                        "initial_x": initial_x,
                        "initial_y": initial_y,
                        "initial_yaw": initial_yaw,
                    }
                ],
            ),
            Node(
                package="agv_gazebo",
                executable="gnss_imu_simulator_node",
                name="gnss_imu_simulator",
                output="screen",
                parameters=[
                    {
                        "odom_topic": "/localization/kinematic_state",
                        "imu_topic": "/sensing/imu/imu_data",
                        "gnss_topic": "/sensing/gnss/nav_sat_fix",
                        "imu_frame_id": "base_link",
                        "gnss_frame_id": "base_link",
                        "reference_latitude": reference_latitude,
                        "reference_longitude": reference_longitude,
                        "reference_altitude": reference_altitude,
                    }
                ],
            ),
            Node(
                package="agv_gazebo",
                executable="dynamic_obstacle_simulator_node",
                name="dynamic_obstacle_simulator",
                output="screen",
                parameters=[
                    {
                        "service_name": "/world/changxing_empty/set_pose",
                        "update_rate": 20.0,
                        "traffic_vehicle_speed": 1.5,
                        "pedestrian_speed": 0.8,
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
                package="agv_map_visualizer",
                executable="lidar_point_marker_publisher",
                name="lidar_point_marker_publisher",
                output="screen",
                parameters=[
                    {
                        "output_topic": "/visualization/lidar_point_markers",
                        "point_size": 0.07,
                        "sample_step": 18,
                    }
                ],
            ),
            *[
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"{child_frame.split('/')[-1]}_tf",
                    output="screen",
                    arguments=[
                        x,
                        y,
                        z,
                        yaw,
                        "0.0",
                        "0.0",
                        "base_link",
                        child_frame,
                    ],
                )
                for x, y, z, yaw, child_frame in lidar_static_transforms
            ],
        ]
    )
