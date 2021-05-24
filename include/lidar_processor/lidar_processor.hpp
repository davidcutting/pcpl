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

#ifndef LIDAR_PROCESSOR__LIDAR_PROCESSOR_HPP_
#define LIDAR_PROCESSOR__LIDAR_PROCESSOR_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace LidarProcessor
{
class LidarProcessor : public rclcpp::Node
{
public:
  explicit LidarProcessor(rclcpp::NodeOptions options);

private:
  void raw_pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void raw_ls_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pcl_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pcl_subscription_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_ls_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr raw_ls_subscription_;
};
}  // namespace LidarProcessor

#endif  // LIDAR_PROCESSOR__LIDAR_PROCESSOR_HPP_
