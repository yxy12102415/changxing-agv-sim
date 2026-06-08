#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

struct Obstacle
{
  std::string name;
  int32_t type;
  double x;
  double y;
  double z;
  double yaw;
  double sx;
  double sy;
  double sz;
  float r;
  float g;
  float b;
};

geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

std_msgs::msg::ColorRGBA color(const float r, const float g, const float b, const float a)
{
  std_msgs::msg::ColorRGBA out;
  out.r = r;
  out.g = g;
  out.b = b;
  out.a = a;
  return out;
}
}  // namespace

class StaticObstacleMarkerPublisher : public rclcpp::Node
{
public:
  StaticObstacleMarkerPublisher() : Node("static_obstacle_marker_publisher")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/visualization/gazebo_obstacle_markers");

    const auto qos = rclcpp::QoS{1}.transient_local().reliable();
    pub_ = create_publisher<MarkerArray>(output_topic_, qos);
    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&StaticObstacleMarkerPublisher::publish_markers, this));

    RCLCPP_INFO(get_logger(), "Publishing static obstacle markers on %s", output_topic_.c_str());
  }

private:
  void publish_markers()
  {
    const std::vector<Obstacle> obstacles{
      {"front_wall", Marker::CUBE, 6.03, -63.005, 0.75, -0.7297, 0.35, 5.0, 1.5, 0.9F, 0.25F,
       0.18F},
      {"left_box", Marker::CUBE, 6.382, -56.611, 0.65, -0.35, 1.4, 1.2, 1.3, 0.2F, 0.75F,
       0.28F},
      {"right_box", Marker::CUBE, -0.284, -64.065, 0.65, 0.4, 1.3, 1.3, 1.3, 0.25F, 0.42F,
       0.95F},
      {"rear_cylinder", Marker::CYLINDER, -3.659, -54.338, 0.8, 0.0, 0.9, 0.9, 1.6, 0.95F,
       0.85F, 0.2F},
      {"far_box", Marker::CUBE, 13.993, -66.102, 1.0, 0.2, 1.8, 1.0, 2.0, 0.82F, 0.35F,
       0.95F},
    };

    MarkerArray markers;
    const auto stamp = now();
    int32_t id = 0;
    for (const auto & obstacle : obstacles) {
      Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = stamp;
      marker.ns = "gazebo_obstacles";
      marker.id = id++;
      marker.type = obstacle.type;
      marker.action = Marker::ADD;
      marker.pose.position.x = obstacle.x;
      marker.pose.position.y = obstacle.y;
      marker.pose.position.z = obstacle.z;
      marker.pose.orientation = yaw_to_quaternion(obstacle.yaw);
      marker.scale.x = obstacle.sx;
      marker.scale.y = obstacle.sy;
      marker.scale.z = obstacle.sz;
      marker.color = color(obstacle.r, obstacle.g, obstacle.b, 0.86F);
      markers.markers.push_back(marker);
    }

    pub_->publish(markers);
  }

  std::string frame_id_;
  std::string output_topic_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticObstacleMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
