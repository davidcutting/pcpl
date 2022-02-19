// MIT License
//
// Copyright (c) 2021 David Cutting, Avery Girven, Andrew Ealovega
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

#include "lidar_processor/lidar_processor.hpp"
#include <lidar_processor/utils.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <memory>
#include <functional>

namespace LidarProcessor
{
LidarProcessor::LidarProcessor(rclcpp::NodeOptions options)
: Node("lidar_processor", options)
{
  raw_ls_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar/raw_scan", 10,
    std::bind(&LidarProcessor::raw_ls_callback, this, std::placeholders::_1));
  raw_pc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar/raw_points", rclcpp::SensorDataQoS(),
    std::bind(&LidarProcessor::raw_pc_callback, this, std::placeholders::_1));

  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/lidar/imu", rclcpp::SensorDataQoS(),
    std::bind(&LidarProcessor::imu_callback, this, std::placeholders::_1));

  // Frame ID/timing fix
  unfiltered_ls_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "/lidar/unfiltered_scan", rclcpp::SensorDataQoS());
  unfiltered_pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/lidar/unfiltered_points", rclcpp::SensorDataQoS());

  // Filtered outputs
  filtered_ls_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "/lidar/filtered_scan", rclcpp::SensorDataQoS());
  filtered_pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/lidar/filtered_points", rclcpp::SensorDataQoS());

  ground_pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/lidar/ground_points", rclcpp::SensorDataQoS());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  using namespace std::chrono_literals;
  this->declare_parameter<float>("ground_point_model_threshold", 0.1f);
  param_update_timer_ = this->create_wall_timer(
      1000ms, std::bind(&LidarProcessor::update_params, this)
      );
}

void LidarProcessor::update_params()
{
  this->get_parameter("ground_point_model_threshold", ground_point_model_threshold_);
}

void LidarProcessor::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  last_imu_ = msg;
}

void LidarProcessor::passthrough_stage(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  // Crop out robot body
  pcl::CropBox<pcl::PointXYZI> crop_box;
  crop_box.setInputCloud(cloud);
  crop_box.setMin(Eigen::Vector4f(-0.2f, -0.2f, -0.2f, 1));
  crop_box.setMax(Eigen::Vector4f(0.2f, 0.2f, 0.2f, 1));
  crop_box.setNegative(true); // filter out points in box
  crop_box.filter(*cloud);

  // Perform Passthrough filtering
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("intensity");
  pass.setFilterLimits(0.0, 0.6);
  pass.setNegative(false);
  pass.filter(*cloud);
}

void LidarProcessor::ground_segmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr ground)
{
  // Find inlier points to plane model: Segment ground plane
  RModel fit_model = Model::Plane{};

  // Use gravitational acceleration vector as plane normal
  // Ignoring orientation for now, as we assume that we are relatively flat on the ground
  NormalVector norm{0.0f, 0.0f, last_imu_ != nullptr ? last_imu_->linear_acceleration.z : -1.0f};

  // Get point in center of robot base footprint
  auto trans = tf_buffer_->lookupTransform("base_footprint", "laser_link", tf2::TimePointZero);
  pcl::PointXYZI point;
  point.x = trans.transform.translation.x;
  point.y = trans.transform.translation.y;
  point.z = trans.transform.translation.z;

  // Segment
  find_plane_coefficients(fit_model, norm, point);
  naive_fit(fit_model, cloud, ground, ground_point_model_threshold_);
}

void LidarProcessor::raw_ls_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  msg->header.stamp = this->get_clock()->now();  // rewrite time
  msg->header.frame_id = "laser_link";  // fix weird scan frame?
  unfiltered_ls_publisher_->publish(*msg);

  // begin filter
  filtered_ls_publisher_->publish(*msg);
}

void LidarProcessor::raw_pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  msg->header.frame_id = "laser_link";  // fix weird pointcloud frame?
  unfiltered_pc_publisher_->publish(*msg);

  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>);

  // Convert to PCL data type
  pcl::fromROSMsg(*msg, *cloud);

  passthrough_stage(cloud);

  ground_segmentation(cloud, ground_points);

  // Convert to ROS data type
  sensor_msgs::msg::PointCloud2 output;
  sensor_msgs::msg::PointCloud2 ground_output;
  pcl::toROSMsg(*cloud, output);
  pcl::toROSMsg(*ground_points, ground_output);

  // rewrite time and space
  output.header.stamp = this->get_clock()->now();
  ground_output.header.stamp = output.header.stamp;
  ground_output.header.frame_id = output.header.frame_id;

  // publish filtered pointclouds
  filtered_pc_publisher_->publish(output);
  ground_pc_publisher_->publish(ground_output);

  last_pcl_ = msg;
}

}  // namespace LidarProcessor
