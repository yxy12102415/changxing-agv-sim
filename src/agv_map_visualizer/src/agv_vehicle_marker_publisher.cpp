#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
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
    wheel_tread_ = declare_parameter<double>("wheel_tread", 0.8);
    wheel_length_ = declare_parameter<double>("wheel_length", 0.75);
    wheel_width_ = declare_parameter<double>("wheel_width", 0.28);
    wheel_height_ = declare_parameter<double>("wheel_height", 0.42);

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
    body.color = color(0.05F, 0.35F, 0.95F, 0.78F);

    Marker cabin = base_marker(frame, stamp, "agv_cabin", 1, Marker::CUBE);
    cabin.pose = pose;
    cabin.pose.position = offset_point(pose, length_ * 0.08, 0.0, height_ + 0.28);
    cabin.scale.x = length_ * 0.42;
    cabin.scale.y = width_ * 0.72;
    cabin.scale.z = 0.55;
    cabin.color = color(0.05F, 0.75F, 0.95F, 0.62F);

    Marker arrow = base_marker(frame, stamp, "agv_heading", 2, Marker::ARROW);
    arrow.points.push_back(offset_point(pose, length_ * 0.08, 0.0, height_ + 0.75));
    arrow.points.push_back(offset_point(pose, length_ * 0.55, 0.0, height_ + 0.75));
    arrow.scale.x = 0.12;
    arrow.scale.y = 0.28;
    arrow.scale.z = 0.32;
    arrow.color = color(1.0F, 0.72F, 0.12F, 0.95F);

    Marker axle_lines = base_marker(frame, stamp, "agv_axles", 3, Marker::LINE_LIST);
    axle_lines.scale.x = 0.07;
    axle_lines.color = color(0.95F, 0.95F, 0.95F, 0.9F);
    for (const double x : {-wheelbase_ * 0.5, wheelbase_ * 0.5}) {
      axle_lines.points.push_back(offset_point(pose, x, -wheel_tread_ * 0.58, 0.22));
      axle_lines.points.push_back(offset_point(pose, x, wheel_tread_ * 0.58, 0.22));
    }

    MarkerArray markers;
    markers.markers.push_back(body);
    markers.markers.push_back(cabin);
    markers.markers.push_back(arrow);
    markers.markers.push_back(axle_lines);

    int32_t wheel_id = 10;
    for (const double x : {-wheelbase_ * 0.5, wheelbase_ * 0.5}) {
      for (const double y : {-wheel_tread_ * 0.5, wheel_tread_ * 0.5}) {
        Marker wheel = base_marker(frame, stamp, "agv_wheels", wheel_id++, Marker::CUBE);
        wheel.pose = pose;
        wheel.pose.position = offset_point(pose, x, y, wheel_height_ * 0.5);
        wheel.scale.x = wheel_length_;
        wheel.scale.y = wheel_width_;
        wheel.scale.z = wheel_height_;
        wheel.color = color(0.04F, 0.04F, 0.04F, 0.96F);
        markers.markers.push_back(wheel);
      }
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
  double wheel_length_;
  double wheel_width_;
  double wheel_height_;
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
