#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>

class PointCloudFrameRepublisherNode : public rclcpp::Node
{
public:
  PointCloudFrameRepublisherNode() : Node("pointcloud_frame_republisher")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/input/points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/output/points");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::QoS{10});
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::QoS{10},
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        auto out = *msg;
        out.header.frame_id = frame_id_;
        pub_->publish(out);
        ++publish_count_;
        if (publish_count_ % 100 == 1) {
          RCLCPP_INFO(
            get_logger(), "Republished %zu PointCloud2 messages on %s", publish_count_,
            output_topic_.c_str());
        }
      });

    RCLCPP_INFO(
      get_logger(), "Republishing PointCloud2 %s -> %s with frame_id=%s", input_topic_.c_str(),
      output_topic_.c_str(), frame_id_.c_str());
  }

private:
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  size_t publish_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFrameRepublisherNode>());
  rclcpp::shutdown();
  return 0;
}
