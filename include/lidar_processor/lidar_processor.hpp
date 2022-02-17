// MIT License
//
// Copyright (c) 2021 David Cutting, Avery Girven
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

#pragma once

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace LidarProcessor
{
class LidarProcessor : public rclcpp::Node
{
public:
  explicit LidarProcessor(rclcpp::NodeOptions options);

private:
  float ground_point_model_threshold_{};
  sensor_msgs::msg::Imu::SharedPtr last_imu_{};
  rclcpp::TimerBase::SharedPtr param_update_timer_;

  void passthrough_stage();
  void ground_segmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr ground);

  void update_params();
  void raw_ls_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void raw_pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr raw_ls_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pc_subscription_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr unfiltered_ls_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unfiltered_pc_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_ls_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pc_publisher_;

  // TF
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};
}  // namespace LidarProcessor
