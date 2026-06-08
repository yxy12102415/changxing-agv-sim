import os
import shutil
import tempfile

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def gazebo_ros_packages():
    try:
        get_package_share_directory("ros_gz_sim")
        return {
            "sim_package": "ros_gz_sim",
            "sim_launch": "gz_sim.launch.py",
            "sim_args": "gz_args",
            "bridge_package": "ros_gz_bridge",
            "interfaces_package": "ros_gz_interfaces",
            "msgs_package": "gz.msgs",
            "supports_service_bridge": True,
        }
    except PackageNotFoundError:
        return {
            "sim_package": "ros_ign_gazebo",
            "sim_launch": "ign_gazebo.launch.py",
            "sim_args": "ign_args",
            "bridge_package": "ros_ign_bridge",
            "interfaces_package": "ros_ign_interfaces",
            "msgs_package": "ignition.msgs",
            "supports_service_bridge": False,
        }


def ignition_compat_paths(world_path, models_path):
    replacements = {
        '<sdf version="1.9">': '<sdf version="1.8">',
        'filename="gz-sim-physics-system" name="gz::sim::systems::Physics"':
            'filename="libignition-gazebo-physics-system.so" '
            'name="ignition::gazebo::systems::Physics"',
        'filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"':
            'filename="libignition-gazebo-user-commands-system.so" '
            'name="ignition::gazebo::systems::UserCommands"',
        'filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"':
            'filename="libignition-gazebo-scene-broadcaster-system.so" '
            'name="ignition::gazebo::systems::SceneBroadcaster"',
        'filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"':
            'filename="libignition-gazebo-sensors-system.so" '
            'name="ignition::gazebo::systems::Sensors"',
    }
    with open(world_path, "r", encoding="utf-8") as src:
        world = src.read()
    for old, new in replacements.items():
        world = world.replace(old, new)

    patched_path = os.path.join(tempfile.gettempdir(), "agv_changxing_empty_ignition.sdf")
    with open(patched_path, "w", encoding="utf-8") as dst:
        dst.write(world)

    patched_models_path = os.path.join(tempfile.gettempdir(), "agv_galactic_models")
    patched_model_path = os.path.join(patched_models_path, "agv_with_lidars")
    source_model_path = os.path.join(models_path, "agv_with_lidars")
    if os.path.exists(patched_model_path):
        shutil.rmtree(patched_model_path)
    shutil.copytree(source_model_path, patched_model_path)
    model_sdf_path = os.path.join(patched_model_path, "model.sdf")
    with open(model_sdf_path, "r", encoding="utf-8") as src:
        model = (
            src.read()
            .replace('<sdf version="1.9">', '<sdf version="1.8">')
            .replace("<samples>1800</samples>", "<samples>360</samples>")
            .replace("<samples>32</samples>", "<samples>16</samples>")
        )
    with open(model_sdf_path, "w", encoding="utf-8") as dst:
        dst.write(model)

    return patched_path, os.pathsep.join([patched_models_path, models_path])


def generate_launch_description():
    map_path = LaunchConfiguration("map_path")
    projector_info_path = LaunchConfiguration("projector_info_path")
    pkg_share = get_package_share_directory("agv_gazebo")
    models_path = os.path.join(pkg_share, "models")
    world_path = os.path.join(pkg_share, "worlds", "changxing_empty.sdf")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    steering_mode = LaunchConfiguration("steering_mode")
    imu_x = LaunchConfiguration("imu_x")
    imu_y = LaunchConfiguration("imu_y")
    imu_z = LaunchConfiguration("imu_z")
    imu_yaw = LaunchConfiguration("imu_yaw")
    imu_pitch = LaunchConfiguration("imu_pitch")
    imu_roll = LaunchConfiguration("imu_roll")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    gnss_x = LaunchConfiguration("gnss_x")
    gnss_y = LaunchConfiguration("gnss_y")
    gnss_z = LaunchConfiguration("gnss_z")
    gnss_yaw = LaunchConfiguration("gnss_yaw")
    gnss_pitch = LaunchConfiguration("gnss_pitch")
    gnss_roll = LaunchConfiguration("gnss_roll")
    gnss_frame_id = LaunchConfiguration("gnss_frame_id")
    imu_topic = LaunchConfiguration("imu_topic")
    gnss_topic = LaunchConfiguration("gnss_topic")
    reference_latitude = LaunchConfiguration("reference_latitude")
    reference_longitude = LaunchConfiguration("reference_longitude")
    reference_altitude = LaunchConfiguration("reference_altitude")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    use_standalone_map = LaunchConfiguration("use_standalone_map")
    use_autoware_map = LaunchConfiguration("use_autoware_map")
    rviz_library_path = os.pathsep.join(
        path for path in os.environ.get("LD_LIBRARY_PATH", "").split(os.pathsep)
        if path and not path.startswith("/snap/")
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("agv_gazebo"), "config", "stage4_lidar.rviz"]
    )
    default_map_path = PathJoinSubstitution(
        [FindPackageShare("agv_maps"), "map", "changxing_v1.osm"]
    )
    default_projector_info_path = PathJoinSubstitution(
        [FindPackageShare("agv_maps"), "map", "map_projector_info.yaml"]
    )
    gazebo_packages = gazebo_ros_packages()
    if not gazebo_packages["supports_service_bridge"]:
        world_path, models_path = ignition_compat_paths(world_path, models_path)

    gz_launch = PathJoinSubstitution(
        [
            FindPackageShare(gazebo_packages["sim_package"]),
            "launch",
            gazebo_packages["sim_launch"],
        ]
    )
    set_pose_service_type = f"{gazebo_packages['interfaces_package']}/srv/SetEntityPose"
    gazebo_msgs = gazebo_packages["msgs_package"]

    bridges = [
        f"/sensing/lidar/front_left@sensor_msgs/msg/LaserScan[{gazebo_msgs}.LaserScan",
        f"/sensing/lidar/front_left/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/sensing/lidar/front_right@sensor_msgs/msg/LaserScan[{gazebo_msgs}.LaserScan",
        f"/sensing/lidar/front_right/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/sensing/lidar/rear_left@sensor_msgs/msg/LaserScan[{gazebo_msgs}.LaserScan",
        f"/sensing/lidar/rear_left/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/sensing/lidar/rear_right@sensor_msgs/msg/LaserScan[{gazebo_msgs}.LaserScan",
        f"/sensing/lidar/rear_right/points@sensor_msgs/msg/PointCloud2[{gazebo_msgs}.PointCloudPacked",
        f"/clock@rosgraph_msgs/msg/Clock[{gazebo_msgs}.Clock",
    ]
    if gazebo_packages["supports_service_bridge"]:
        bridges.append(
            f"/world/changxing_empty/set_pose@{set_pose_service_type}"
            f"@{gazebo_msgs}.Pose@{gazebo_msgs}.Boolean"
        )
    lidar_calibration_defaults = [
        ("rslidar2", "1.35", "-0.45", "0.47", "-0.785398", "0.0", "0.0"),
        ("hesai_left_front", "1.35", "0.45", "0.47", "0.785398", "0.0", "0.0"),
        ("rslidar4", "-1.35", "0.45", "0.47", "2.35619", "0.0", "0.0"),
        ("hesai_right_rear", "-1.35", "-0.45", "0.47", "-2.35619", "0.0", "0.0"),
    ]
    lidar_static_transforms = [
        (
            LaunchConfiguration(f"{frame_id}_x"),
            LaunchConfiguration(f"{frame_id}_y"),
            LaunchConfiguration(f"{frame_id}_z"),
            LaunchConfiguration(f"{frame_id}_yaw"),
            LaunchConfiguration(f"{frame_id}_pitch"),
            LaunchConfiguration(f"{frame_id}_roll"),
            frame_id,
        )
        for frame_id, *_ in lidar_calibration_defaults
    ]
    lidar_frame_republishers = [
        ("/sim/lidar/rslidar_points_2_raw", "/rslidar_points_2", "rslidar2"),
        ("/sim/lidar/hesai_left_front_raw", "/hesai_left_front", "hesai_left_front"),
        ("/sim/lidar/rslidar_points_4_raw", "/rslidar_points_4", "rslidar4"),
        ("/sim/lidar/hesai_right_rear_raw", "/hesai_right_rear", "hesai_right_rear"),
    ]
    ignition_pointcloud_bridges = []
    if not gazebo_packages["supports_service_bridge"]:
        ignition_pointcloud_bridges = [
            (
                "/sensing/lidar/front_left/points",
                "/sim/lidar/hesai_left_front_raw",
                "hesai_left_front",
            ),
            (
                "/sensing/lidar/front_right/points",
                "/sim/lidar/rslidar_points_2_raw",
                "rslidar2",
            ),
            ("/sensing/lidar/rear_left/points", "/sim/lidar/rslidar_points_4_raw", "rslidar4"),
            (
                "/sensing/lidar/rear_right/points",
                "/sim/lidar/hesai_right_rear_raw",
                "hesai_right_rear",
            ),
        ]
    sensor_static_transforms = [
        (imu_x, imu_y, imu_z, imu_yaw, imu_pitch, imu_roll, imu_frame_id, "imu_tf"),
        (gnss_x, gnss_y, gnss_z, gnss_yaw, gnss_pitch, gnss_roll, gnss_frame_id, "gnss_tf"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("initial_x", default_value="0.0674"),
            DeclareLaunchArgument("initial_y", default_value="-57.6716"),
            DeclareLaunchArgument("initial_yaw", default_value="-0.7297"),
            DeclareLaunchArgument("steering_mode", default_value="crab"),
            DeclareLaunchArgument("imu_x", default_value="0.0"),
            DeclareLaunchArgument("imu_y", default_value="0.0"),
            DeclareLaunchArgument("imu_z", default_value="0.60"),
            DeclareLaunchArgument("imu_yaw", default_value="0.0"),
            DeclareLaunchArgument("imu_pitch", default_value="0.0"),
            DeclareLaunchArgument("imu_roll", default_value="0.0"),
            DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
            DeclareLaunchArgument("gnss_x", default_value="0.0"),
            DeclareLaunchArgument("gnss_y", default_value="0.0"),
            DeclareLaunchArgument("gnss_z", default_value="0.95"),
            DeclareLaunchArgument("gnss_yaw", default_value="0.0"),
            DeclareLaunchArgument("gnss_pitch", default_value="0.0"),
            DeclareLaunchArgument("gnss_roll", default_value="0.0"),
            DeclareLaunchArgument("gnss_frame_id", default_value="gnss_link"),
            *[
                DeclareLaunchArgument(f"{frame_id}_{axis}", default_value=default_value)
                for frame_id, x, y, z, yaw, pitch, roll in lidar_calibration_defaults
                for axis, default_value in (
                    ("x", x),
                    ("y", y),
                    ("z", z),
                    ("yaw", yaw),
                    ("pitch", pitch),
                    ("roll", roll),
                )
            ],
            DeclareLaunchArgument("imu_topic", default_value="/chnav/imu/data"),
            DeclareLaunchArgument("gnss_topic", default_value="/chnav/fix"),
            DeclareLaunchArgument("map_path", default_value=default_map_path),
            DeclareLaunchArgument(
                "projector_info_path", default_value=default_projector_info_path
            ),
            DeclareLaunchArgument("reference_latitude", default_value="31.2304"),
            DeclareLaunchArgument("reference_longitude", default_value="121.4737"),
            DeclareLaunchArgument("reference_altitude", default_value="0.0"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_gazebo_gui",
                default_value="true" if os.environ.get("DISPLAY") else "false",
            ),
            DeclareLaunchArgument("use_standalone_map", default_value="true"),
            DeclareLaunchArgument("use_autoware_map", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    models_path,
                    os.pathsep,
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=[
                    models_path,
                    os.pathsep,
                    EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(name="IGN_PARTITION", value="agv_sim"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={gazebo_packages["sim_args"]: ["-r -v 2 ", world_path]}.items(),
                condition=IfCondition(use_gazebo_gui),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                launch_arguments={
                    gazebo_packages["sim_args"]: ["-r -s -v 2 ", world_path]
                }.items(),
                condition=UnlessCondition(use_gazebo_gui),
            ),
            Node(
                package=gazebo_packages["bridge_package"],
                executable="parameter_bridge",
                name="agv_gazebo_bridge",
                output="screen",
                arguments=bridges,
                remappings=[
                    ("/sensing/lidar/front_left/points", "/sim/lidar/hesai_left_front_raw"),
                    ("/sensing/lidar/front_right/points", "/sim/lidar/rslidar_points_2_raw"),
                    ("/sensing/lidar/rear_left/points", "/sim/lidar/rslidar_points_4_raw"),
                    ("/sensing/lidar/rear_right/points", "/sim/lidar/hesai_right_rear_raw"),
                ],
            ),
            *[
                Node(
                    package="agv_gazebo",
                    executable="ign_pointcloud_bridge_node",
                    name=f"{frame_id}_ign_pointcloud_bridge",
                    output="screen",
                    parameters=[
                        {
                            "ign_topic": ign_topic,
                            "ros_topic": ros_topic,
                            "fallback_frame_id": frame_id,
                        }
                    ],
                )
                for ign_topic, ros_topic, frame_id in ignition_pointcloud_bridges
            ],
            *[
                Node(
                    package="agv_gazebo",
                    executable="pointcloud_frame_republisher_node",
                    name=f"{frame_id}_frame_republisher",
                    output="screen",
                    parameters=[
                        {
                            "input_topic": input_topic,
                            "output_topic": output_topic,
                            "frame_id": frame_id,
                        }
                    ],
                )
                for input_topic, output_topic, frame_id in lidar_frame_republishers
            ],
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
                condition=IfCondition(use_autoware_map),
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
                condition=IfCondition(use_autoware_map),
            ),
            Node(
                package="agv_map_visualizer",
                executable="lanelet2_marker_publisher",
                name="lanelet2_marker_publisher",
                output="screen",
                condition=IfCondition(use_autoware_map),
            ),
            Node(
                package="agv_map_visualizer",
                executable="osm_marker_publisher.py",
                name="osm_marker_publisher",
                output="screen",
                parameters=[
                    {
                        "map_path": map_path,
                        "output_topic": "/map/vector_map_marker",
                        "frame_id": "map",
                    }
                ],
                condition=IfCondition(use_standalone_map),
            ),
            Node(
                package="agv_vehicle_model",
                executable="agv_standalone_simulator_node",
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
                        "imu_topic": imu_topic,
                        "gnss_topic": gnss_topic,
                        "imu_frame_id": imu_frame_id,
                        "gnss_frame_id": gnss_frame_id,
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
                        "sample_step": 3,
                        "max_points_per_cloud": 20000,
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
                        pitch,
                        roll,
                        "base_link",
                        child_frame,
                    ],
                )
                for x, y, z, yaw, pitch, roll, child_frame in lidar_static_transforms
            ],
            *[
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=node_name,
                    output="screen",
                    arguments=[
                        x,
                        y,
                        z,
                        yaw,
                        pitch,
                        roll,
                        "base_link",
                        child_frame,
                    ],
                )
                for x, y, z, yaw, pitch, roll, child_frame, node_name in sensor_static_transforms
            ],
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_gazebo",
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
