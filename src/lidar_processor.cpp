// MIT License
//
// Copyright (c) 2021 David Cutting
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <memory>
#include <functional>

#include "lidar_processor/lidar_processor.hpp"

namespace LidarProcessor
{
LidarProcessor::LidarProcessor(rclcpp::NodeOptions options)
: Node("lidar_processor", options)
{
  filtered_pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/lidar/filtered_points", 10);
  raw_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar/raw_points", 10,
    std::bind(&LidarProcessor::raw_pcl_callback, this, std::placeholders::_1));

  filtered_ls_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "/lidar/filtered_scan", 10);
  raw_ls_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar/raw_scan", 10,
    std::bind(&LidarProcessor::raw_ls_callback, this, std::placeholders::_1));
}

void LidarProcessor::raw_pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  msg->header.stamp = this->get_clock()->now();  // rewrite time
  msg->header.frame_id = "laser_link";  // fix weird pointcloud frame?
  filtered_pcl_publisher_->publish(*msg);
}

void LidarProcessor::raw_ls_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  msg->header.stamp = this->get_clock()->now();  // rewrite time
  msg->header.frame_id = "laser_link";  // fix weird scan frame?
  filtered_ls_publisher_->publish(*msg);
}

}  // namespace LidarProcessor

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto lp_node = std::make_shared<LidarProcessor::LidarProcessor>(options);
  exec.add_node(lp_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
