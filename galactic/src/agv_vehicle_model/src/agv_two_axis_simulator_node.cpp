#include "simple_planning_simulator/vehicle_model/sim_model_delay_steer_vel_two_axis.hpp"

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::simulator::simple_planning_simulator::SimModelDelaySteerVelTwoAxis;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using autoware_control_msgs::msg::Control;
using autoware_vehicle_msgs::msg::ControlModeReport;
using autoware_vehicle_msgs::msg::Engage;
using autoware_vehicle_msgs::msg::GearCommand;
using autoware_vehicle_msgs::msg::GearReport;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::HazardLightsReport;
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using autoware_vehicle_msgs::msg::VelocityReport;
using autoware_vehicle_msgs::srv::ControlModeCommand;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Float32;
using std_msgs::msg::String;

geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::optional<double> extract_osm_tag_value(const std::string & line, const std::string & key)
{
  const std::string key_pattern = "k=\"" + key + "\"";
  if (line.find(key_pattern) == std::string::npos) {
    return std::nullopt;
  }

  const auto value_key = std::string{"v=\""};
  const auto begin = line.find(value_key);
  if (begin == std::string::npos) {
    return std::nullopt;
  }
  const auto value_begin = begin + value_key.size();
  const auto value_end = line.find('"', value_begin);
  if (value_end == std::string::npos || value_end <= value_begin) {
    return std::nullopt;
  }

  try {
    return std::stod(line.substr(value_begin, value_end - value_begin));
  } catch (const std::exception &) {
    return std::nullopt;
  }
}
}  // namespace

class AgvTwoAxisSimulatorNode : public rclcpp::Node
{
public:
  AgvTwoAxisSimulatorNode() : Node("agv_two_axis_simulator")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    steering_mode_ = declare_parameter<std::string>("steering_mode", "opposite_4ws");
    dt_ = declare_parameter<double>("dt", 0.025);
    vx_lim_ = declare_parameter<double>("vx_lim", 5.0);
    steer_lim_ = declare_parameter<double>("steer_lim", 0.7);
    vx_rate_lim_ = declare_parameter<double>("vx_rate_lim", 2.0);
    steer_rate_lim_ = declare_parameter<double>("steer_rate_lim", 1.5);
    wheelbase_f_ = declare_parameter<double>("wheelbase_f", 1.0);
    wheelbase_r_ = declare_parameter<double>("wheelbase_r", 1.0);
    vx_delay_ = declare_parameter<double>("vx_delay", 0.1);
    vx_time_constant_ = declare_parameter<double>("vx_time_constant", 0.2);
    steer_delay_ = declare_parameter<double>("steer_delay", 0.1);
    steer_time_constant_ = declare_parameter<double>("steer_time_constant", 0.2);
    steer_dead_band_ = declare_parameter<double>("steer_dead_band", 0.0);
    steer_bias_ = declare_parameter<double>("steer_bias", 0.0);
    steering_mode_transition_time_ = declare_parameter<double>("steering_mode_transition_time", 0.45);
    wheel_tread_ = declare_parameter<double>("wheel_tread", 1.36);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.28);
    wheel_frame_prefix_ = declare_parameter<std::string>("wheel_frame_prefix", "");
    initial_x_ = declare_parameter<double>("initial_x", 0.0);
    initial_y_ = declare_parameter<double>("initial_y", 0.0);
    initial_yaw_ = declare_parameter<double>("initial_yaw", 0.0);
    map_path_ = declare_parameter<std::string>("map_path", "");
    use_ground_height_ = declare_parameter<bool>("use_ground_height", true);
    ground_height_offset_ = declare_parameter<double>("ground_height_offset", 0.0);

    load_ground_points(map_path_);
    target_rear_steer_ratio_ = rear_steer_ratio_for_mode(steering_mode_);
    current_rear_steer_ratio_ = target_rear_steer_ratio_;

    model_ = std::make_unique<SimModelDelaySteerVelTwoAxis>(
      vx_lim_, steer_lim_, vx_rate_lim_, steer_rate_lim_, wheelbase_f_, wheelbase_r_, dt_,
      vx_delay_, vx_time_constant_, steer_delay_, steer_time_constant_, steer_dead_band_,
      steer_bias_);
    set_model_pose(initial_x_, initial_y_, initial_yaw_);

    sub_control_ = create_subscription<Control>(
      "/control/command/control_cmd", rclcpp::QoS{1},
      [this](const Control::ConstSharedPtr msg) { current_control_ = *msg; });
    sub_manual_control_ = create_subscription<Control>(
      "/vehicle/command/manual_control_cmd", rclcpp::QoS{1},
      [this](const Control::ConstSharedPtr msg) { current_manual_control_ = *msg; });
    sub_gear_ = create_subscription<GearCommand>(
      "/control/command/gear_cmd", rclcpp::QoS{1},
      std::bind(&AgvTwoAxisSimulatorNode::on_gear_cmd, this, std::placeholders::_1));
    sub_manual_gear_ = create_subscription<GearCommand>(
      "/vehicle/command/manual_gear_command", rclcpp::QoS{1},
      std::bind(&AgvTwoAxisSimulatorNode::on_gear_cmd, this, std::placeholders::_1));
    sub_turn_indicators_ = create_subscription<TurnIndicatorsCommand>(
      "/control/command/turn_indicators_cmd", rclcpp::QoS{1},
      std::bind(&AgvTwoAxisSimulatorNode::on_turn_indicators_cmd, this, std::placeholders::_1));
    sub_hazard_lights_ = create_subscription<HazardLightsCommand>(
      "/control/command/hazard_lights_cmd", rclcpp::QoS{1},
      std::bind(&AgvTwoAxisSimulatorNode::on_hazard_lights_cmd, this, std::placeholders::_1));
    sub_engage_ = create_subscription<Engage>(
      "/vehicle/engage", rclcpp::QoS{1},
      std::bind(&AgvTwoAxisSimulatorNode::on_engage, this, std::placeholders::_1));
    sub_initial_pose_ = create_subscription<PoseWithCovarianceStamped>(
      "/initialpose3d", rclcpp::QoS{1},
      std::bind(&AgvTwoAxisSimulatorNode::on_initial_pose, this, std::placeholders::_1));
    sub_rviz_initial_pose_ = create_subscription<PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS{1},
      std::bind(&AgvTwoAxisSimulatorNode::on_initial_pose, this, std::placeholders::_1));
    sub_steering_mode_ = create_subscription<String>(
      "/agv/steering_mode_cmd", rclcpp::QoS{1},
      std::bind(&AgvTwoAxisSimulatorNode::on_steering_mode_cmd, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pub_odom_ = create_publisher<Odometry>("/localization/kinematic_state", rclcpp::QoS{1});
    pub_velocity_ =
      create_publisher<VelocityReport>("/vehicle/status/velocity_status", rclcpp::QoS{1});
    pub_steering_ =
      create_publisher<SteeringReport>("/vehicle/status/steering_status", rclcpp::QoS{1});
    pub_gear_ = create_publisher<GearReport>("/vehicle/status/gear_status", rclcpp::QoS{1});
    pub_turn_indicators_ =
      create_publisher<TurnIndicatorsReport>("/vehicle/status/turn_indicators_status", rclcpp::QoS{1});
    pub_hazard_lights_ =
      create_publisher<HazardLightsReport>("/vehicle/status/hazard_lights_status", rclcpp::QoS{1});
    pub_control_mode_ =
      create_publisher<ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS{1});
    pub_operation_mode_ =
      create_publisher<OperationModeState>("/api/operation_mode/state", rclcpp::QoS{1});
    pub_system_operation_mode_ =
      create_publisher<OperationModeState>("/system/operation_mode/state", rclcpp::QoS{1});
    pub_steering_mode_ = create_publisher<String>("/agv/status/steering_mode", rclcpp::QoS{1});
    pub_front_steer_ = create_publisher<Float32>("/agv/status/front_steer", rclcpp::QoS{1});
    pub_rear_steer_ = create_publisher<Float32>("/agv/status/rear_steer", rclcpp::QoS{1});
    srv_control_mode_ = create_service<ControlModeCommand>(
      "/control/control_mode_request",
      std::bind(
        &AgvTwoAxisSimulatorNode::on_control_mode_request, this, std::placeholders::_1,
        std::placeholders::_2));
    srv_change_to_stop_ = create_operation_mode_service(
      "/api/operation_mode/change_to_stop", OperationModeState::STOP);
    srv_change_to_autonomous_ = create_operation_mode_service(
      "/api/operation_mode/change_to_autonomous", OperationModeState::AUTONOMOUS);
    srv_change_to_local_ = create_operation_mode_service(
      "/api/operation_mode/change_to_local", OperationModeState::LOCAL);
    srv_change_to_remote_ = create_operation_mode_service(
      "/api/operation_mode/change_to_remote", OperationModeState::REMOTE);

    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(dt_),
      std::bind(&AgvTwoAxisSimulatorNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(), "AGV two-axis simulator started: mode=%s, wheelbase=%.2f m",
      steering_mode_.c_str(), wheelbase_f_ + wheelbase_r_);
  }

private:
  void on_initial_pose(const PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    set_model_pose(
      msg->pose.pose.position.x, msg->pose.pose.position.y,
      yaw_from_quaternion(msg->pose.pose.orientation));
    RCLCPP_INFO(get_logger(), "Set initial AGV pose from /initialpose3d");
  }

  void set_model_pose(const double x, const double y, const double yaw)
  {
    Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
    state(0) = x;
    state(1) = y;
    state(2) = yaw;
    model_->setState(state);
  }

  void on_steering_mode_cmd(const String::ConstSharedPtr msg)
  {
    if (
      msg->data == "front_only" || msg->data == "opposite_4ws" ||
      msg->data == "asymmetric_4ws" || msg->data == "crab") {
      steering_mode_ = msg->data;
      target_rear_steer_ratio_ = rear_steer_ratio_for_mode(steering_mode_);
      RCLCPP_INFO(
        get_logger(), "Changed steering_mode to '%s' with rear ratio target %.3f",
        steering_mode_.c_str(), target_rear_steer_ratio_);
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "Rejected unknown steering_mode '%s'. Use front_only, opposite_4ws, asymmetric_4ws, or crab.",
      msg->data.c_str());
  }

  void on_gear_cmd(const GearCommand::ConstSharedPtr msg)
  {
    if (msg->command != GearCommand::NONE) {
      current_gear_ = msg->command;
    }
  }

  void on_turn_indicators_cmd(const TurnIndicatorsCommand::ConstSharedPtr msg)
  {
    if (msg->command != TurnIndicatorsCommand::NO_COMMAND) {
      current_turn_indicators_ = msg->command;
    }
  }

  void on_hazard_lights_cmd(const HazardLightsCommand::ConstSharedPtr msg)
  {
    if (msg->command != HazardLightsCommand::NO_COMMAND) {
      current_hazard_lights_ = msg->command;
    }
  }

  void on_engage(const Engage::ConstSharedPtr msg)
  {
    engaged_ = msg->engage;
    if (!engaged_) {
      current_operation_mode_ = OperationModeState::STOP;
      current_control_mode_ = ControlModeReport::MANUAL;
    } else if (current_operation_mode_ == OperationModeState::STOP) {
      current_operation_mode_ = OperationModeState::AUTONOMOUS;
      current_control_mode_ = ControlModeReport::AUTONOMOUS;
    }
  }

  void on_control_mode_request(
    const std::shared_ptr<ControlModeCommand::Request> request,
    const std::shared_ptr<ControlModeCommand::Response> response)
  {
    if (request->mode == ControlModeCommand::Request::NO_COMMAND) {
      response->success = true;
      return;
    }

    if (
      request->mode == ControlModeCommand::Request::AUTONOMOUS ||
      request->mode == ControlModeCommand::Request::AUTONOMOUS_STEER_ONLY ||
      request->mode == ControlModeCommand::Request::AUTONOMOUS_VELOCITY_ONLY ||
      request->mode == ControlModeCommand::Request::MANUAL) {
      current_control_mode_ = request->mode;
      response->success = true;
      RCLCPP_INFO(get_logger(), "Changed control_mode to %u", current_control_mode_);
      return;
    }

    response->success = false;
    RCLCPP_WARN(get_logger(), "Rejected unknown control_mode %u", request->mode);
  }

  rclcpp::Service<ChangeOperationMode>::SharedPtr create_operation_mode_service(
    const std::string & service_name, const uint8_t operation_mode)
  {
    return create_service<ChangeOperationMode>(
      service_name,
      [this, operation_mode](
        const std::shared_ptr<ChangeOperationMode::Request>,
        const std::shared_ptr<ChangeOperationMode::Response> response) {
        current_operation_mode_ = operation_mode;
        engaged_ = operation_mode != OperationModeState::STOP;
        current_control_mode_ =
          operation_mode == OperationModeState::AUTONOMOUS ? ControlModeReport::AUTONOMOUS :
          ControlModeReport::MANUAL;
        response->status.success = true;
        response->status.code = 0;
        response->status.message = "accepted";
        RCLCPP_INFO(get_logger(), "Changed operation_mode to %u", current_operation_mode_);
      });
  }

  void on_timer()
  {
    const auto control = select_control_command();
    const double command_vx =
      std::clamp(static_cast<double>(control.longitudinal.velocity), -vx_lim_, vx_lim_);
    const double command_steer = std::clamp(
      static_cast<double>(control.lateral.steering_tire_angle), -steer_lim_, steer_lim_);

    update_steering_mode_transition();
    const auto [steer_f, steer_r] = split_steer(command_steer);
    last_front_steer_cmd_ = steer_f;
    last_rear_steer_cmd_ = steer_r;
    Eigen::VectorXd input(3);
    input << command_vx, steer_f, steer_r;
    model_->setInput(input);
    model_->update(dt_);

    publish_state();
  }

  std::pair<double, double> split_steer(const double command_steer)
  {
    return {command_steer, command_steer * current_rear_steer_ratio_};
  }

  double rear_steer_ratio_for_mode(const std::string & mode) const
  {
    if (mode == "front_only") {
      return 0.0;
    }
    if (mode == "crab") {
      return 1.0;
    }
    if (mode == "opposite_4ws") {
      return -1.0;
    }
    if (mode == "asymmetric_4ws") {
      return -1.0 / 3.0;
    }
    return -1.0;
  }

  void update_steering_mode_transition()
  {
    const double transition_time = std::max(1.0e-3, steering_mode_transition_time_);
    const double max_step = dt_ / transition_time;
    const double error = target_rear_steer_ratio_ - current_rear_steer_ratio_;
    current_rear_steer_ratio_ += std::clamp(error, -max_step, max_step);
  }

  std::string wheel_frame_id(const std::string & name) const
  {
    return wheel_frame_prefix_.empty() ? name : wheel_frame_prefix_ + name;
  }

  geometry_msgs::msg::TransformStamped make_wheel_transform(
    const rclcpp::Time & stamp, const std::string & name, const double x, const double y,
    const double steer) const
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = child_frame_id_;
    transform.child_frame_id = wheel_frame_id(name);
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = wheel_radius_;
    transform.transform.rotation = yaw_to_quaternion(steer);
    return transform;
  }

  void publish_wheel_transforms(
    const rclcpp::Time & stamp, const double front_steer, const double rear_steer)
  {
    const std::array<geometry_msgs::msg::TransformStamped, 4> wheel_transforms{{
      make_wheel_transform(
        stamp, "front_left_wheel", wheelbase_f_, wheel_tread_ * 0.5, front_steer),
      make_wheel_transform(
        stamp, "front_right_wheel", wheelbase_f_, -wheel_tread_ * 0.5, front_steer),
      make_wheel_transform(
        stamp, "rear_left_wheel", -wheelbase_r_, wheel_tread_ * 0.5, rear_steer),
      make_wheel_transform(
        stamp, "rear_right_wheel", -wheelbase_r_, -wheel_tread_ * 0.5, rear_steer),
    }};

    for (const auto & transform : wheel_transforms) {
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void publish_state()
  {
    const auto stamp = now();
    const double ground_z = get_ground_height(model_->getX(), model_->getY());
    const double front_steer_angle = model_->getSteerF();
    const double rear_steer_angle = model_->getSteerR();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = frame_id_;
    transform.child_frame_id = child_frame_id_;
    transform.transform.translation.x = model_->getX();
    transform.transform.translation.y = model_->getY();
    transform.transform.translation.z = ground_z;
    transform.transform.rotation = yaw_to_quaternion(model_->getYaw());
    tf_broadcaster_->sendTransform(transform);
    publish_wheel_transforms(stamp, front_steer_angle, rear_steer_angle);

    Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = model_->getX();
    odom.pose.pose.position.y = model_->getY();
    odom.pose.pose.position.z = ground_z;
    odom.pose.pose.orientation = yaw_to_quaternion(model_->getYaw());
    odom.twist.twist.linear.x = model_->getVx();
    odom.twist.twist.linear.y = model_->getVy();
    odom.twist.twist.angular.z = model_->getWz();
    pub_odom_->publish(odom);

    VelocityReport velocity;
    velocity.header.stamp = stamp;
    velocity.header.frame_id = child_frame_id_;
    velocity.longitudinal_velocity = static_cast<float>(model_->getVx());
    velocity.lateral_velocity = static_cast<float>(model_->getVy());
    velocity.heading_rate = static_cast<float>(model_->getWz());
    pub_velocity_->publish(velocity);

    SteeringReport steering;
    steering.stamp = stamp;
    steering.steering_tire_angle = static_cast<float>(front_steer_angle);
    pub_steering_->publish(steering);

    GearReport gear;
    gear.stamp = stamp;
    gear.report = current_gear_;
    pub_gear_->publish(gear);

    TurnIndicatorsReport turn_indicators;
    turn_indicators.stamp = stamp;
    turn_indicators.report = current_turn_indicators_;
    pub_turn_indicators_->publish(turn_indicators);

    HazardLightsReport hazard_lights;
    hazard_lights.stamp = stamp;
    hazard_lights.report = current_hazard_lights_;
    pub_hazard_lights_->publish(hazard_lights);

    ControlModeReport control_mode;
    control_mode.stamp = stamp;
    control_mode.mode = current_control_mode_;
    pub_control_mode_->publish(control_mode);

    OperationModeState operation_mode;
    operation_mode.stamp = stamp;
    operation_mode.mode = current_operation_mode_;
    operation_mode.is_autoware_control_enabled =
      engaged_ && current_operation_mode_ == OperationModeState::AUTONOMOUS;
    operation_mode.is_in_transition = false;
    operation_mode.is_stop_mode_available = true;
    operation_mode.is_autonomous_mode_available = true;
    operation_mode.is_local_mode_available = true;
    operation_mode.is_remote_mode_available = true;
    pub_operation_mode_->publish(operation_mode);
    pub_system_operation_mode_->publish(operation_mode);

    String mode;
    mode.data = steering_mode_;
    pub_steering_mode_->publish(mode);

    Float32 front_steer;
    front_steer.data = static_cast<float>(front_steer_angle);
    pub_front_steer_->publish(front_steer);

    Float32 rear_steer;
    rear_steer.data = static_cast<float>(rear_steer_angle);
    pub_rear_steer_->publish(rear_steer);
  }

  Control select_control_command() const
  {
    if (current_control_mode_ == ControlModeReport::MANUAL) {
      return current_manual_control_;
    }
    if (current_control_mode_ == ControlModeReport::AUTONOMOUS_STEER_ONLY) {
      Control mixed = current_manual_control_;
      mixed.lateral = current_control_.lateral;
      return mixed;
    }
    if (current_control_mode_ == ControlModeReport::AUTONOMOUS_VELOCITY_ONLY) {
      Control mixed = current_manual_control_;
      mixed.longitudinal = current_control_.longitudinal;
      return mixed;
    }
    return current_control_;
  }

  struct GroundPoint
  {
    double x;
    double y;
    double z;
  };

  void load_ground_points(const std::string & map_path)
  {
    if (!use_ground_height_ || map_path.empty()) {
      return;
    }

    std::ifstream file(map_path);
    if (!file.is_open()) {
      RCLCPP_WARN(get_logger(), "Failed to open map_path for ground height: %s", map_path.c_str());
      return;
    }

    bool in_node = false;
    std::optional<double> x;
    std::optional<double> y;
    std::optional<double> z;
    std::string line;
    while (std::getline(file, line)) {
      if (line.find("<node ") != std::string::npos) {
        in_node = true;
        x.reset();
        y.reset();
        z.reset();
        continue;
      }
      if (!in_node) {
        continue;
      }

      if (auto value = extract_osm_tag_value(line, "local_x")) {
        x = value;
      } else if (auto value = extract_osm_tag_value(line, "local_y")) {
        y = value;
      } else if (auto value = extract_osm_tag_value(line, "ele")) {
        z = value;
      }

      if (line.find("</node>") != std::string::npos) {
        if (x && y && z) {
          ground_points_.push_back({*x, *y, *z});
        }
        in_node = false;
      }
    }

    RCLCPP_INFO(
      get_logger(), "Loaded %zu ground height points from %s", ground_points_.size(),
      map_path.c_str());
  }

  double get_ground_height(const double x, const double y) const
  {
    if (!use_ground_height_ || ground_points_.empty()) {
      return ground_height_offset_;
    }

    std::vector<std::pair<double, double>> nearest;
    nearest.reserve(8);
    for (const auto & point : ground_points_) {
      const double dx = point.x - x;
      const double dy = point.y - y;
      const double dist2 = dx * dx + dy * dy;
      nearest.emplace_back(dist2, point.z);
    }
    const auto count = std::min<size_t>(8, nearest.size());
    if (count < nearest.size()) {
      std::nth_element(nearest.begin(), nearest.begin() + count, nearest.end());
    }

    double weighted_z = 0.0;
    double weight_sum = 0.0;
    for (auto it = nearest.begin(); it != nearest.begin() + count; ++it) {
      if (it->first < 1.0e-6) {
        return it->second + ground_height_offset_;
      }
      const double weight = 1.0 / it->first;
      weighted_z += weight * it->second;
      weight_sum += weight;
    }

    if (weight_sum < 1.0e-9) {
      return ground_height_offset_;
    }
    return weighted_z / weight_sum + ground_height_offset_;
  }

  std::string frame_id_;
  std::string child_frame_id_;
  std::string steering_mode_;
  std::string map_path_;
  double dt_;
  double vx_lim_;
  double steer_lim_;
  double vx_rate_lim_;
  double steer_rate_lim_;
  double wheelbase_f_;
  double wheelbase_r_;
  double vx_delay_;
  double vx_time_constant_;
  double steer_delay_;
  double steer_time_constant_;
  double steer_dead_band_;
  double steer_bias_;
  double steering_mode_transition_time_;
  double wheel_tread_;
  double wheel_radius_;
  std::string wheel_frame_prefix_;
  double initial_x_;
  double initial_y_;
  double initial_yaw_;
  bool use_ground_height_;
  double ground_height_offset_;
  double target_rear_steer_ratio_ = -1.0;
  double current_rear_steer_ratio_ = -1.0;
  double last_front_steer_cmd_ = 0.0;
  double last_rear_steer_cmd_ = 0.0;
  uint8_t current_control_mode_ = ControlModeReport::AUTONOMOUS;
  uint8_t current_operation_mode_ = OperationModeState::AUTONOMOUS;
  uint8_t current_gear_ = GearReport::DRIVE;
  uint8_t current_turn_indicators_ = TurnIndicatorsReport::DISABLE;
  uint8_t current_hazard_lights_ = HazardLightsReport::DISABLE;
  bool engaged_ = true;

  Control current_control_;
  Control current_manual_control_;
  std::vector<GroundPoint> ground_points_;
  std::unique_ptr<SimModelDelaySteerVelTwoAxis> model_;
  rclcpp::Subscription<Control>::SharedPtr sub_control_;
  rclcpp::Subscription<Control>::SharedPtr sub_manual_control_;
  rclcpp::Subscription<GearCommand>::SharedPtr sub_gear_;
  rclcpp::Subscription<GearCommand>::SharedPtr sub_manual_gear_;
  rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr sub_turn_indicators_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr sub_hazard_lights_;
  rclcpp::Subscription<Engage>::SharedPtr sub_engage_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_rviz_initial_pose_;
  rclcpp::Subscription<String>::SharedPtr sub_steering_mode_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<VelocityReport>::SharedPtr pub_velocity_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<GearReport>::SharedPtr pub_gear_;
  rclcpp::Publisher<TurnIndicatorsReport>::SharedPtr pub_turn_indicators_;
  rclcpp::Publisher<HazardLightsReport>::SharedPtr pub_hazard_lights_;
  rclcpp::Publisher<ControlModeReport>::SharedPtr pub_control_mode_;
  rclcpp::Publisher<OperationModeState>::SharedPtr pub_operation_mode_;
  rclcpp::Publisher<OperationModeState>::SharedPtr pub_system_operation_mode_;
  rclcpp::Publisher<String>::SharedPtr pub_steering_mode_;
  rclcpp::Publisher<Float32>::SharedPtr pub_front_steer_;
  rclcpp::Publisher<Float32>::SharedPtr pub_rear_steer_;
  rclcpp::Service<ControlModeCommand>::SharedPtr srv_control_mode_;
  rclcpp::Service<ChangeOperationMode>::SharedPtr srv_change_to_stop_;
  rclcpp::Service<ChangeOperationMode>::SharedPtr srv_change_to_autonomous_;
  rclcpp::Service<ChangeOperationMode>::SharedPtr srv_change_to_local_;
  rclcpp::Service<ChangeOperationMode>::SharedPtr srv_change_to_remote_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvTwoAxisSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
