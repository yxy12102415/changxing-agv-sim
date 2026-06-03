#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace
{
geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
}  // namespace

class KinematicStateToGazeboNode : public rclcpp::Node
{
public:
  KinematicStateToGazeboNode() : Node("kinematic_state_to_gazebo")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/localization/kinematic_state");
    service_name_ =
      declare_parameter<std::string>("service_name", "/world/changxing_empty/set_pose");
    entity_name_ = declare_parameter<std::string>("entity_name", "ego_agv");
    z_offset_ = declare_parameter<double>("z_offset", 0.4);
    initial_x_ = declare_parameter<double>("initial_x", 0.0674);
    initial_y_ = declare_parameter<double>("initial_y", -57.6716);
    initial_yaw_ = declare_parameter<double>("initial_yaw", -0.7297);

    client_ = create_client<ros_gz_interfaces::srv::SetEntityPose>(service_name_);
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS{1},
      std::bind(&KinematicStateToGazeboNode::on_odom, this, std::placeholders::_1));
    initial_pose_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&KinematicStateToGazeboNode::on_initial_pose_timer, this));

    RCLCPP_INFO(
      get_logger(), "Syncing %s from %s to Gazebo service %s",
      entity_name_.c_str(), odom_topic_.c_str(), service_name_.c_str());
  }

private:
  void on_initial_pose_timer()
  {
    if (received_odom_) {
      initial_pose_timer_->cancel();
      return;
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = initial_x_;
    pose.position.y = initial_y_;
    pose.position.z = z_offset_;
    pose.orientation = yaw_to_quaternion(initial_yaw_);
    send_pose(pose);
  }

  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    received_odom_ = true;
    if (initial_pose_timer_) {
      initial_pose_timer_->cancel();
    }

    auto pose = msg->pose.pose;
    pose.position.z += z_offset_;
    send_pose(pose);
  }

  void send_pose(const geometry_msgs::msg::Pose & pose)
  {
    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for Gazebo set_pose service: %s",
        service_name_.c_str());
      return;
    }

    if (request_in_flight_) {
      return;
    }

    auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
    request->entity.name = entity_name_;
    request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
    request->pose = pose;

    request_in_flight_ = true;
    auto response_cb =
      [this](rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture future) {
        request_in_flight_ = false;
        if (!future.get()->success) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "Gazebo rejected pose update for %s",
            entity_name_.c_str());
        }
      };
    client_->async_send_request(request, response_cb);
  }

  std::string odom_topic_;
  std::string service_name_;
  std::string entity_name_;
  double z_offset_;
  double initial_x_;
  double initial_y_;
  double initial_yaw_;
  bool request_in_flight_ = false;
  bool received_odom_ = false;

  rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::TimerBase::SharedPtr initial_pose_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicStateToGazeboNode>());
  rclcpp::shutdown();
  return 0;
}
