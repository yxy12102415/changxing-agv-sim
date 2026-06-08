#!/usr/bin/env bash
set -uo pipefail

REPORT_DIR="${1:-/tmp/agv_galactic_interface_report}"
mkdir -p "${REPORT_DIR}"

run_capture() {
  local name="$1"
  shift
  {
    echo "$ $*"
    "$@"
  } > "${REPORT_DIR}/${name}.txt" 2>&1
}

capture_topic() {
  local topic="$1"
  local safe_name
  safe_name="$(echo "${topic}" | sed 's#^/##; s#/#__#g')"
  run_capture "topic_info__${safe_name}" ros2 topic info "${topic}" --verbose
}

capture_interface() {
  local interface="$1"
  local safe_name
  safe_name="$(echo "${interface}" | sed 's#/#__#g')"
  run_capture "interface__${safe_name}" ros2 interface show "${interface}"
}

{
  echo "AGV Galactic interface report"
  echo "Generated at: $(date -Is)"
  echo
  echo "ROS_DISTRO=${ROS_DISTRO:-}"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"
  echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-}"
} > "${REPORT_DIR}/environment.txt"

run_capture "ros2_doctor" ros2 doctor --report
run_capture "topic_list" ros2 topic list -t
run_capture "service_list" ros2 service list -t
run_capture "node_list" ros2 node list
run_capture "processes" ps -ef

topics=(
  /control/command/control_cmd
  /control/command/gear_cmd
  /control/command/turn_indicators_cmd
  /control/command/hazard_lights_cmd
  /vehicle/command/manual_control_cmd
  /vehicle/command/manual_gear_command
  /vehicle/engage
  /vehicle/status/velocity_status
  /vehicle/status/steering_status
  /vehicle/status/gear_status
  /vehicle/status/control_mode
  /api/operation_mode/state
  /system/operation_mode/state
  /localization/kinematic_state
  /localization/acceleration
  /sensing/imu/imu_data
  /sensing/gnss/nav_sat_fix
  /tf
  /tf_static
)

for topic in "${topics[@]}"; do
  capture_topic "${topic}"
done

interfaces=(
  autoware_control_msgs/msg/Control
  autoware_vehicle_msgs/msg/GearCommand
  autoware_vehicle_msgs/msg/GearReport
  autoware_vehicle_msgs/msg/TurnIndicatorsCommand
  autoware_vehicle_msgs/msg/TurnIndicatorsReport
  autoware_vehicle_msgs/msg/HazardLightsCommand
  autoware_vehicle_msgs/msg/HazardLightsReport
  autoware_vehicle_msgs/msg/ControlModeReport
  autoware_vehicle_msgs/msg/VelocityReport
  autoware_vehicle_msgs/msg/SteeringReport
  autoware_vehicle_msgs/msg/Engage
  autoware_vehicle_msgs/srv/ControlModeCommand
  autoware_adapi_v1_msgs/msg/OperationModeState
  autoware_adapi_v1_msgs/srv/ChangeOperationMode
  geometry_msgs/msg/Twist
  nav_msgs/msg/Odometry
)

for interface in "${interfaces[@]}"; do
  capture_interface "${interface}"
done

cat > "${REPORT_DIR}/README.txt" <<EOF
把整个目录打包发回开发机：

  tar czf /tmp/agv_galactic_interface_report.tgz -C /tmp agv_galactic_interface_report

重点看：

  topic_list.txt
  service_list.txt
  topic_info__control__command__control_cmd.txt
  interface__autoware_control_msgs__msg__Control.txt

如果某个文件里显示 "Unknown topic" 或 "Unknown package"，说明真实车 Galactic 环境
没有当前 Humble 仿真假设的接口，需要做 adapter。
EOF

echo "Report written to ${REPORT_DIR}"
