#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#ifdef AGV_USE_ROS_IGN_INTERFACES
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/transport/Node.hh>
#endif
#include <rclcpp/rclcpp.hpp>
#ifdef AGV_USE_ROS_IGN_INTERFACES
#include <ros_ign_interfaces/msg/entity.hpp>
#include <ros_ign_interfaces/srv/set_entity_pose.hpp>
#else
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{
#ifdef AGV_USE_ROS_IGN_INTERFACES
namespace gazebo_interfaces = ros_ign_interfaces;
#else
namespace gazebo_interfaces = ros_gz_interfaces;
#endif

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Quaternion yaw_to_quaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double lerp(const double a, const double b, const double t)
{
  return a + (b - a) * t;
}

double yaw_between(const double x0, const double y0, const double x1, const double y1)
{
  return std::atan2(y1 - y0, x1 - x0);
}

double distance(const double x0, const double y0, const double x1, const double y1)
{
  return std::hypot(x1 - x0, y1 - y0);
}

#ifdef AGV_USE_ROS_IGN_INTERFACES
ignition::msgs::Pose to_ign_pose(
  const std::string & entity_name, const geometry_msgs::msg::Pose & pose)
{
  ignition::msgs::Pose out;
  out.set_name(entity_name);
  out.mutable_position()->set_x(pose.position.x);
  out.mutable_position()->set_y(pose.position.y);
  out.mutable_position()->set_z(pose.position.z);
  out.mutable_orientation()->set_x(pose.orientation.x);
  out.mutable_orientation()->set_y(pose.orientation.y);
  out.mutable_orientation()->set_z(pose.orientation.z);
  out.mutable_orientation()->set_w(pose.orientation.w);
  return out;
}
#endif
}  // namespace

class DynamicObstacleSimulatorNode : public rclcpp::Node
{
public:
  DynamicObstacleSimulatorNode() : Node("dynamic_obstacle_simulator")
  {
    service_name_ =
      declare_parameter<std::string>("service_name", "/world/changxing_empty/set_pose");
    update_rate_ = declare_parameter<double>("update_rate", 20.0);
    traffic_vehicle_speed_ = declare_parameter<double>("traffic_vehicle_speed", 1.5);
    pedestrian_speed_ = declare_parameter<double>("pedestrian_speed", 0.8);

#ifndef AGV_USE_ROS_IGN_INTERFACES
    client_ = create_client<gazebo_interfaces::srv::SetEntityPose>(service_name_);
#endif
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, update_rate_))),
      std::bind(&DynamicObstacleSimulatorNode::on_timer, this));

    start_time_ = now();
    RCLCPP_INFO(get_logger(), "Animating traffic vehicle and pedestrians via %s", service_name_.c_str());
  }

private:
  struct SegmentMotion
  {
    std::string entity_name;
    double x0;
    double y0;
    double x1;
    double y1;
    double z;
    double speed;
    bool ping_pong;
  };

  struct Waypoint
  {
    double x;
    double y;
  };

  void on_timer()
  {
#ifndef AGV_USE_ROS_IGN_INTERFACES
    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for Gazebo set_pose service: %s",
        service_name_.c_str());
      return;
    }
#endif

    const double t = (now() - start_time_).seconds();
    const std::vector<Waypoint> vehicle_inner_loop{
      {8.86, -65.53},
      {13.38, -69.58},
      {17.91, -73.63},
      {22.44, -77.68},
      {27.80, -80.36},
      {33.84, -80.58},
      {39.61, -78.85},
      {43.82, -74.51},
      {47.84, -69.96},
      {51.86, -65.41},
      {55.88, -60.86},
      {59.84, -56.26},
      {60.84, -50.42},
      {57.90, -45.20},
      {53.40, -41.13},
      {48.86, -37.10},
      {44.32, -33.06},
      {39.78, -29.03},
      {35.24, -25.00},
      {30.70, -20.97},
      {26.16, -16.93},
      {21.62, -12.90},
      {17.08, -8.87},
      {12.54, -4.84},
      {7.88, -0.96},
      {2.02, -0.01},
      {-2.50, -3.92},
      {-6.50, -8.49},
      {-10.50, -13.06},
      {-14.50, -17.63},
      {-18.50, -22.19},
      {-22.50, -26.76},
      {-25.39, -31.93},
      {-22.83, -37.19},
      {-18.30, -41.24},
      {-13.77, -45.29},
      {-9.25, -49.34},
      {-4.72, -53.39},
      {-0.20, -57.44},
      {4.33, -61.48},
    };
    const std::vector<Waypoint> vehicle_outer_loop{
      {-24.16, -38.69},
      {-19.73, -42.65},
      {-15.30, -46.61},
      {-10.88, -50.57},
      {-6.45, -54.53},
      {-2.02, -58.49},
      {2.41, -62.45},
      {6.83, -66.41},
      {11.26, -70.37},
      {15.69, -74.33},
      {20.12, -78.29},
      {25.00, -81.59},
      {30.80, -82.67},
      {36.69, -82.11},
      {41.96, -79.54},
      {45.94, -75.13},
      {49.87, -70.68},
      {53.80, -66.23},
      {57.74, -61.77},
      {61.58, -57.25},
      {63.00, -51.58},
      {61.11, -46.02},
      {57.08, -41.68},
      {52.64, -37.74},
      {48.19, -33.80},
      {43.75, -29.86},
      {39.31, -25.91},
      {34.86, -21.97},
      {30.42, -18.03},
      {25.98, -14.08},
      {21.53, -10.14},
      {17.09, -6.20},
      {12.65, -2.26},
      {7.90, 1.27},
      {2.10, 2.03},
      {-2.74, -1.17},
      {-6.66, -5.64},
      {-10.57, -10.10},
      {-14.48, -14.57},
      {-18.40, -19.04},
      {-22.31, -23.51},
      {-26.14, -28.04},
      {-27.20, -33.73},
    };
    const std::vector<SegmentMotion> motions{
      {"crossing_pedestrian_1", -16.81, -48.61, -10.82, -41.90, 0.875, pedestrian_speed_, true},
      {"crossing_pedestrian_2", 12.03, -74.41, 18.03, -67.70, 0.875, pedestrian_speed_, true},
      {"crossing_pedestrian_3", 50.27, -74.01, 43.53, -68.05, 0.875, pedestrian_speed_, true},
      {"crossing_pedestrian_4", 55.58, -37.05, 49.61, -43.78, 0.875, pedestrian_speed_, true},
      {"crossing_pedestrian_5", 26.65, -11.35, 20.67, -18.08, 0.875, pedestrian_speed_, true},
      {"crossing_pedestrian_6", -9.74, -5.36, -2.97, -11.29, 0.875, pedestrian_speed_, true},
    };

    publish_loop_pose("moving_vehicle_1", vehicle_inner_loop, 0.4, traffic_vehicle_speed_, t);
    publish_loop_pose("moving_vehicle_2", vehicle_outer_loop, 0.4, traffic_vehicle_speed_, t + 45.0);
    for (const auto & motion : motions) {
      publish_motion_pose(motion, t);
    }
  }

  void publish_loop_pose(
    const std::string & entity_name, const std::vector<Waypoint> & waypoints, const double z,
    const double speed, const double elapsed)
  {
    if (waypoints.size() < 2) {
      return;
    }

    double loop_length = 0.0;
    std::vector<double> segment_lengths;
    segment_lengths.reserve(waypoints.size());
    for (size_t i = 0; i < waypoints.size(); ++i) {
      const auto & start = waypoints.at(i);
      const auto & end = waypoints.at((i + 1) % waypoints.size());
      const double segment_length = distance(start.x, start.y, end.x, end.y);
      segment_lengths.push_back(segment_length);
      loop_length += segment_length;
    }

    if (loop_length < 1.0e-3) {
      return;
    }

    double path_distance = std::fmod(elapsed * std::max(0.1, speed), loop_length);
    size_t segment_index = 0;
    while (segment_index + 1 < segment_lengths.size() &&
           path_distance > segment_lengths.at(segment_index)) {
      path_distance -= segment_lengths.at(segment_index);
      ++segment_index;
    }

    const auto & start = waypoints.at(segment_index);
    const auto & end = waypoints.at((segment_index + 1) % waypoints.size());
    const double segment_length = std::max(1.0e-3, segment_lengths.at(segment_index));
    const double phase = std::clamp(path_distance / segment_length, 0.0, 1.0);

    geometry_msgs::msg::Pose pose;
    pose.position.x = lerp(start.x, end.x, phase);
    pose.position.y = lerp(start.y, end.y, phase);
    pose.position.z = z;
    pose.orientation = yaw_to_quaternion(yaw_between(start.x, start.y, end.x, end.y));
    send_pose(entity_name, pose);
  }

  void publish_motion_pose(const SegmentMotion & motion, const double elapsed)
  {
    const double length = std::max(1.0e-3, distance(motion.x0, motion.y0, motion.x1, motion.y1));
    const double period = length / std::max(0.1, motion.speed);
    double phase = std::fmod(elapsed / period, 1.0);
    double yaw = yaw_between(motion.x0, motion.y0, motion.x1, motion.y1);

    if (motion.ping_pong) {
      const bool backward = std::fmod(elapsed / period, 2.0) >= 1.0;
      if (backward) {
        phase = 1.0 - phase;
        yaw += kPi;
      }
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = lerp(motion.x0, motion.x1, phase);
    pose.position.y = lerp(motion.y0, motion.y1, phase);
    pose.position.z = motion.z;
    pose.orientation = yaw_to_quaternion(yaw);
    send_pose(motion.entity_name, pose);
  }

  void send_pose(const std::string & entity_name, const geometry_msgs::msg::Pose & pose)
  {
#ifdef AGV_USE_ROS_IGN_INTERFACES
    const auto request = to_ign_pose(entity_name, pose);
    ignition::msgs::Boolean response;
    bool result = false;
    constexpr unsigned int timeout_ms = 10;
    const bool executed = ign_node_.Request(service_name_, request, timeout_ms, response, result);
    if (!executed || !result || !response.data()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignition rejected pose update for %s via %s",
        entity_name.c_str(), service_name_.c_str());
    }
#else
    auto request = std::make_shared<gazebo_interfaces::srv::SetEntityPose::Request>();
    request->entity.name = entity_name;
    request->entity.type = gazebo_interfaces::msg::Entity::MODEL;
    request->pose = pose;
    client_->async_send_request(request);
#endif
  }

  std::string service_name_;
  double update_rate_;
  double traffic_vehicle_speed_;
  double pedestrian_speed_;
  rclcpp::Time start_time_;
#ifdef AGV_USE_ROS_IGN_INTERFACES
  ignition::transport::Node ign_node_;
#endif
  rclcpp::Client<gazebo_interfaces::srv::SetEntityPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicObstacleSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
