#include <memory>
#include <functional>

#include "lidar_processor/lidar_processor.h"

LidarProcessor::LidarProcessor(rclcpp::NodeOptions options) : Node("lidar_processor", options)
{
  filtered_pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/filtered_points", 10);
  raw_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  "/lidar/raw_points", 10, std::bind(&LidarProcessor::raw_pcl_callback, this, std::placeholders::_1));

  filtered_ls_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/lidar/filtered_scan", 10);
  raw_ls_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
  "/lidar/raw_scan", 10, std::bind(&LidarProcessor::raw_ls_callback, this, std::placeholders::_1));
}

void LidarProcessor::raw_pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  msg->header.frame_id = "laser_link"; // fix weird pointcloud frame?
  filtered_pcl_publisher_->publish(*msg);
}

void LidarProcessor::raw_ls_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  msg->header.frame_id = "laser_link"; // fix weird scan frame?
  filtered_ls_publisher_->publish(*msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto lp_node = std::make_shared<LidarProcessor>(options);
  exec.add_node(lp_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}