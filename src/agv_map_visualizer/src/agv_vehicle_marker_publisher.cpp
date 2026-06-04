#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <string>

namespace
{
using nav_msgs::msg::Odometry;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

std_msgs::msg::ColorRGBA color(const float r, const float g, const float b, const float a)
{
  std_msgs::msg::ColorRGBA out;
  out.r = r;
  out.g = g;
  out.b = b;
  out.a = a;
  return out;
}

double yaw_from_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quaternion_from_rpy(
  const double roll, const double pitch, const double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

geometry_msgs::msg::Quaternion multiply(
  const geometry_msgs::msg::Quaternion & a, const geometry_msgs::msg::Quaternion & b)
{
  geometry_msgs::msg::Quaternion q;
  q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return q;
}

geometry_msgs::msg::Point offset_point(
  const geometry_msgs::msg::Pose & pose, const double local_x, const double local_y,
  const double local_z)
{
  const double yaw = yaw_from_pose(pose);
  geometry_msgs::msg::Point point;
  point.x = pose.position.x + std::cos(yaw) * local_x - std::sin(yaw) * local_y;
  point.y = pose.position.y + std::sin(yaw) * local_x + std::cos(yaw) * local_y;
  point.z = pose.position.z + local_z;
  return point;
}

geometry_msgs::msg::Pose offset_pose(
  const geometry_msgs::msg::Pose & pose, const double local_x, const double local_y,
  const double local_z, const double local_roll = 0.0, const double local_pitch = 0.0,
  const double local_yaw = 0.0)
{
  geometry_msgs::msg::Pose out;
  out.position = offset_point(pose, local_x, local_y, local_z);
  out.orientation = multiply(pose.orientation, quaternion_from_rpy(local_roll, local_pitch, local_yaw));
  return out;
}

Marker base_marker(
  const std::string & frame_id, const rclcpp::Time & stamp, const std::string & ns,
  const int32_t id, const int32_t type)
{
  Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.pose.orientation.w = 1.0;
  return marker;
}
}  // namespace

class AgvVehicleMarkerPublisher : public rclcpp::Node
{
public:
  AgvVehicleMarkerPublisher() : Node("agv_vehicle_marker_publisher")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/localization/kinematic_state");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/visualization/agv_vehicle_marker");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    length_ = declare_parameter<double>("length", 3.0);
    width_ = declare_parameter<double>("width", 1.0);
    height_ = declare_parameter<double>("height", 0.8);
    wheelbase_ = declare_parameter<double>("wheelbase", 2.0);
    wheel_tread_ = declare_parameter<double>("wheel_tread", 1.36);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.28);
    wheel_width_ = declare_parameter<double>("wheel_width", 0.24);
    lidar_radius_ = declare_parameter<double>("lidar_radius", 0.13);
    lidar_height_ = declare_parameter<double>("lidar_height", 0.14);
    lidar_cap_radius_ = declare_parameter<double>("lidar_cap_radius", 0.145);
    lidar_cap_height_ = declare_parameter<double>("lidar_cap_height", 0.035);

    pub_marker_ = create_publisher<MarkerArray>(output_topic_, rclcpp::QoS{1});
    sub_odom_ = create_subscription<Odometry>(
      input_topic_, rclcpp::QoS{10},
      std::bind(&AgvVehicleMarkerPublisher::on_odometry, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Publishing AGV vehicle markers: %s -> %s", input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  void on_odometry(const Odometry::ConstSharedPtr msg)
  {
    const auto stamp = now();
    const auto frame = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;
    const auto & pose = msg->pose.pose;

    Marker body = base_marker(frame, stamp, "agv_body", 0, Marker::CUBE);
    body.pose = pose;
    body.pose.position.z += height_ * 0.5;
    body.scale.x = length_;
    body.scale.y = width_;
    body.scale.z = height_;
    body.color = color(0.08F, 0.42F, 0.50F, 0.95F);

    Marker front_marker = base_marker(frame, stamp, "agv_front_marker", 1, Marker::CUBE);
    front_marker.pose = offset_pose(pose, length_ * 0.5 + 0.025, 0.0, 0.15);
    front_marker.scale.x = 0.05;
    front_marker.scale.y = 0.85;
    front_marker.scale.z = 0.12;
    front_marker.color = color(0.95F, 0.95F, 0.15F, 0.96F);

    MarkerArray markers;
    markers.markers.push_back(body);
    markers.markers.push_back(front_marker);

    int32_t wheel_id = 10;
    for (const double x : {-wheelbase_ * 0.5, wheelbase_ * 0.5}) {
      for (const double y : {-wheel_tread_ * 0.5, wheel_tread_ * 0.5}) {
        Marker wheel = base_marker(frame, stamp, "agv_wheels", wheel_id++, Marker::CYLINDER);
        wheel.pose = offset_pose(pose, x, y, wheel_radius_, 1.5708, 0.0, 0.0);
        wheel.scale.x = wheel_radius_ * 2.0;
        wheel.scale.y = wheel_radius_ * 2.0;
        wheel.scale.z = wheel_width_;
        wheel.color = color(0.01F, 0.01F, 0.01F, 0.98F);
        markers.markers.push_back(wheel);
      }
    }

    struct LidarVisual
    {
      double x;
      double y;
      double yaw;
      float r;
      float g;
      float b;
    };
    const std::array<LidarVisual, 4> lidars{{
      {1.35, 0.45, 0.785398, 1.0F, 0.12F, 0.08F},
      {1.35, -0.45, -0.785398, 0.1F, 0.95F, 0.15F},
      {-1.35, 0.45, 2.35619, 0.15F, 0.45F, 1.0F},
      {-1.35, -0.45, -2.35619, 1.0F, 0.75F, 0.08F},
    }};

    int32_t lidar_id = 20;
    for (const auto & lidar : lidars) {
      Marker shell = base_marker(frame, stamp, "agv_lidar_shells", lidar_id++, Marker::CYLINDER);
      shell.pose = offset_pose(pose, lidar.x, lidar.y, height_ + lidar_height_ * 0.5, 0.0, 0.0, lidar.yaw);
      shell.scale.x = lidar_radius_ * 2.0;
      shell.scale.y = lidar_radius_ * 2.0;
      shell.scale.z = lidar_height_;
      shell.color = color(0.05F, 0.05F, 0.05F, 0.98F);
      markers.markers.push_back(shell);

      Marker cap = base_marker(frame, stamp, "agv_lidar_caps", lidar_id++, Marker::CYLINDER);
      cap.pose = offset_pose(
        pose, lidar.x, lidar.y, height_ + lidar_height_ + lidar_cap_height_ * 0.5, 0.0, 0.0,
        lidar.yaw);
      cap.scale.x = lidar_cap_radius_ * 2.0;
      cap.scale.y = lidar_cap_radius_ * 2.0;
      cap.scale.z = lidar_cap_height_;
      cap.color = color(lidar.r, lidar.g, lidar.b, 0.96F);
      markers.markers.push_back(cap);
    }

    pub_marker_->publish(markers);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double length_;
  double width_;
  double height_;
  double wheelbase_;
  double wheel_tread_;
  double wheel_radius_;
  double wheel_width_;
  double lidar_radius_;
  double lidar_height_;
  double lidar_cap_radius_;
  double lidar_cap_height_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgvVehicleMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
