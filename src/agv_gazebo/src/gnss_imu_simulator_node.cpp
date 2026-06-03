#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

namespace
{
constexpr double kEarthRadiusMeters = 6378137.0;
constexpr double kPi = 3.14159265358979323846;
}  // namespace

class GnssImuSimulatorNode : public rclcpp::Node
{
public:
  GnssImuSimulatorNode() : Node("gnss_imu_simulator")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/localization/kinematic_state");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/sensing/imu/imu_data");
    gnss_topic_ = declare_parameter<std::string>("gnss_topic", "/sensing/gnss/nav_sat_fix");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "base_link");
    gnss_frame_id_ = declare_parameter<std::string>("gnss_frame_id", "base_link");
    reference_latitude_ = declare_parameter<double>("reference_latitude", 31.2304);
    reference_longitude_ = declare_parameter<double>("reference_longitude", 121.4737);
    reference_altitude_ = declare_parameter<double>("reference_altitude", 0.0);
    position_stddev_ = declare_parameter<double>("position_stddev", 0.03);
    orientation_stddev_ = declare_parameter<double>("orientation_stddev", 0.01);
    angular_velocity_stddev_ = declare_parameter<double>("angular_velocity_stddev", 0.01);
    linear_acceleration_stddev_ = declare_parameter<double>("linear_acceleration_stddev", 0.1);

    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::QoS{10});
    pub_gnss_ = create_publisher<sensor_msgs::msg::NavSatFix>(gnss_topic_, rclcpp::QoS{10});
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS{10},
      std::bind(&GnssImuSimulatorNode::on_odom, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Publishing simulated IMU [%s] and GNSS [%s] from %s",
      imu_topic_.c_str(), gnss_topic_.c_str(), odom_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "GNSS reference origin: lat=%.8f lon=%.8f alt=%.2f",
      reference_latitude_, reference_longitude_, reference_altitude_);
  }

private:
  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    publish_imu(*msg);
    publish_gnss(*msg);

    previous_odom_ = *msg;
    has_previous_odom_ = true;
  }

  void publish_imu(const nav_msgs::msg::Odometry & odom)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = odom.header.stamp;
    imu.header.frame_id = imu_frame_id_;
    imu.orientation = odom.pose.pose.orientation;
    imu.angular_velocity.z = odom.twist.twist.angular.z;

    if (has_previous_odom_) {
      const double dt = std::max(
        1.0e-6,
        (rclcpp::Time(odom.header.stamp) - rclcpp::Time(previous_odom_.header.stamp)).seconds());
      imu.linear_acceleration.x =
        (odom.twist.twist.linear.x - previous_odom_.twist.twist.linear.x) / dt;
      imu.linear_acceleration.y =
        (odom.twist.twist.linear.y - previous_odom_.twist.twist.linear.y) / dt;
      imu.linear_acceleration.z = 0.0;
    }

    set_diagonal_covariance(imu.orientation_covariance, orientation_stddev_);
    set_diagonal_covariance(imu.angular_velocity_covariance, angular_velocity_stddev_);
    set_diagonal_covariance(imu.linear_acceleration_covariance, linear_acceleration_stddev_);

    pub_imu_->publish(imu);
  }

  void publish_gnss(const nav_msgs::msg::Odometry & odom)
  {
    const double origin_lat_rad = reference_latitude_ * kPi / 180.0;
    const double north = odom.pose.pose.position.y;
    const double east = odom.pose.pose.position.x;

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = odom.header.stamp;
    fix.header.frame_id = gnss_frame_id_;
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
                         sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
                         sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
    fix.latitude = reference_latitude_ + (north / kEarthRadiusMeters) * 180.0 / kPi;
    fix.longitude = reference_longitude_ +
                    (east / (kEarthRadiusMeters * std::cos(origin_lat_rad))) * 180.0 / kPi;
    fix.altitude = reference_altitude_ + odom.pose.pose.position.z;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    fix.position_covariance[0] = position_stddev_ * position_stddev_;
    fix.position_covariance[4] = position_stddev_ * position_stddev_;
    fix.position_covariance[8] = position_stddev_ * position_stddev_;

    pub_gnss_->publish(fix);
  }

  void set_diagonal_covariance(std::array<double, 9> & covariance, const double stddev) const
  {
    covariance.fill(0.0);
    const double variance = stddev * stddev;
    covariance[0] = variance;
    covariance[4] = variance;
    covariance[8] = variance;
  }

  std::string odom_topic_;
  std::string imu_topic_;
  std::string gnss_topic_;
  std::string imu_frame_id_;
  std::string gnss_frame_id_;
  double reference_latitude_;
  double reference_longitude_;
  double reference_altitude_;
  double position_stddev_;
  double orientation_stddev_;
  double angular_velocity_stddev_;
  double linear_acceleration_stddev_;
  bool has_previous_odom_{false};
  nav_msgs::msg::Odometry previous_odom_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gnss_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssImuSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
