#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarProcessor : public rclcpp::Node
{
public:
  LidarProcessor(rclcpp::NodeOptions options);

private:
  void raw_pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void raw_ls_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pcl_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pcl_subscription_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_ls_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr raw_ls_subscription_;
};

