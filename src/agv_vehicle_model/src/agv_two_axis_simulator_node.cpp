#include "simple_planning_simulator/vehicle_model/sim_model_delay_steer_vel_two_axis.hpp"

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace
{
using autoware::simulator::simple_planning_simulator::SimModelDelaySteerVelTwoAxis;
using autoware_control_msgs::msg::Control;
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Float32;
using std_msgs::msg::String;
using tf2_msgs::msg::TFMessage;

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
    initial_x_ = declare_parameter<double>("initial_x", 0.0);
    initial_y_ = declare_parameter<double>("initial_y", 0.0);
    initial_yaw_ = declare_parameter<double>("initial_yaw", 0.0);

    model_ = std::make_unique<SimModelDelaySteerVelTwoAxis>(
      vx_lim_, steer_lim_, vx_rate_lim_, steer_rate_lim_, wheelbase_f_, wheelbase_r_, dt_,
      vx_delay_, vx_time_constant_, steer_delay_, steer_time_constant_, steer_dead_band_,
      steer_bias_);
    set_model_pose(initial_x_, initial_y_, initial_yaw_);

    sub_control_ = create_subscription<Control>(
      "/control/command/control_cmd", rclcpp::QoS{1},
      [this](const Control::ConstSharedPtr msg) { current_control_ = *msg; });
    sub_initial_pose_ = create_subscription<PoseWithCovarianceStamped>(
      "/initialpose3d", rclcpp::QoS{1},
      std::bind(&AgvTwoAxisSimulatorNode::on_initial_pose, this, std::placeholders::_1));
    sub_rviz_initial_pose_ = create_subscription<PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS{1},
      std::bind(&AgvTwoAxisSimulatorNode::on_initial_pose, this, std::placeholders::_1));
    sub_steering_mode_ = create_subscription<String>(
      "/agv/steering_mode_cmd", rclcpp::QoS{1},
      std::bind(&AgvTwoAxisSimulatorNode::on_steering_mode_cmd, this, std::placeholders::_1));

    pub_tf_ = create_publisher<TFMessage>("/tf", rclcpp::QoS{1});
    pub_odom_ = create_publisher<Odometry>("/localization/kinematic_state", rclcpp::QoS{1});
    pub_velocity_ =
      create_publisher<VelocityReport>("/vehicle/status/velocity_status", rclcpp::QoS{1});
    pub_steering_ =
      create_publisher<SteeringReport>("/vehicle/status/steering_status", rclcpp::QoS{1});
    pub_steering_mode_ = create_publisher<String>("/agv/status/steering_mode", rclcpp::QoS{1});
    pub_front_steer_ = create_publisher<Float32>("/agv/status/front_steer", rclcpp::QoS{1});
    pub_rear_steer_ = create_publisher<Float32>("/agv/status/rear_steer", rclcpp::QoS{1});

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
    if (msg->data == "front_only" || msg->data == "opposite_4ws" || msg->data == "crab") {
      steering_mode_ = msg->data;
      RCLCPP_INFO(get_logger(), "Changed steering_mode to '%s'", steering_mode_.c_str());
      return;
    }

    RCLCPP_WARN(
      get_logger(), "Rejected unknown steering_mode '%s'. Use front_only, opposite_4ws, or crab.",
      msg->data.c_str());
  }

  void on_timer()
  {
    const double command_vx =
      std::clamp(static_cast<double>(current_control_.longitudinal.velocity), -vx_lim_, vx_lim_);
    const double command_steer = std::clamp(
      static_cast<double>(current_control_.lateral.steering_tire_angle), -steer_lim_, steer_lim_);

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
    if (steering_mode_ == "front_only") {
      return {command_steer, 0.0};
    }
    if (steering_mode_ == "crab") {
      return {command_steer, command_steer};
    }
    if (steering_mode_ == "opposite_4ws") {
      return {command_steer, -command_steer};
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Unknown steering_mode '%s', using opposite_4ws",
      steering_mode_.c_str());
    return {command_steer, -command_steer};
  }

  void publish_state()
  {
    const auto stamp = now();

    TFMessage tf_msg;
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = frame_id_;
    transform.child_frame_id = child_frame_id_;
    transform.transform.translation.x = model_->getX();
    transform.transform.translation.y = model_->getY();
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = yaw_to_quaternion(model_->getYaw());
    tf_msg.transforms.push_back(transform);
    pub_tf_->publish(tf_msg);

    Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = model_->getX();
    odom.pose.pose.position.y = model_->getY();
    odom.pose.pose.position.z = 0.0;
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
    steering.steering_tire_angle = static_cast<float>(model_->getSteer());
    pub_steering_->publish(steering);

    String mode;
    mode.data = steering_mode_;
    pub_steering_mode_->publish(mode);

    Float32 front_steer;
    front_steer.data = static_cast<float>(last_front_steer_cmd_);
    pub_front_steer_->publish(front_steer);

    Float32 rear_steer;
    rear_steer.data = static_cast<float>(last_rear_steer_cmd_);
    pub_rear_steer_->publish(rear_steer);
  }

  std::string frame_id_;
  std::string child_frame_id_;
  std::string steering_mode_;
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
  double initial_x_;
  double initial_y_;
  double initial_yaw_;
  double last_front_steer_cmd_ = 0.0;
  double last_rear_steer_cmd_ = 0.0;

  Control current_control_;
  std::unique_ptr<SimModelDelaySteerVelTwoAxis> model_;
  rclcpp::Subscription<Control>::SharedPtr sub_control_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_rviz_initial_pose_;
  rclcpp::Subscription<String>::SharedPtr sub_steering_mode_;
  rclcpp::Publisher<TFMessage>::SharedPtr pub_tf_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<VelocityReport>::SharedPtr pub_velocity_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<String>::SharedPtr pub_steering_mode_;
  rclcpp::Publisher<Float32>::SharedPtr pub_front_steer_;
  rclcpp::Publisher<Float32>::SharedPtr pub_rear_steer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvTwoAxisSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
