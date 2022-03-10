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
  raw_pc_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar/raw_points", rclcpp::SensorDataQoS(),
    std::bind(&LidarProcessor::raw_pc_callback, this, std::placeholders::_1));

  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/lidar/imu", rclcpp::SensorDataQoS(),
    std::bind(&LidarProcessor::imu_callback, this, std::placeholders::_1));

  // Frame ID/timing fix
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
  this->declare_parameter<bool>("debug_cloud", false);
  param_update_timer_ = this->create_wall_timer(
      1000ms, std::bind(&LidarProcessor::update_params, this)
      );
}

void LidarProcessor::update_params()
{
  this->get_parameter("ground_point_model_threshold", ground_point_model_threshold_);
  this->get_parameter("debug_cloud", debug_cloud_);
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


  // Perform intensity filtering
  float min_intensity = std::numeric_limits<float>().max();
  float max_intensity = 0.0f;
  double sum_of_intensity = 0.0;
  int sample_size = 0;
  double mean_intensity;

  // Calculate statistics
  for (auto& point : *cloud)
  {
    if (point.intensity < min_intensity) min_intensity = point.intensity;
    if (point.intensity > max_intensity) max_intensity = point.intensity;
    sum_of_intensity = sum_of_intensity + point.intensity;
    sample_size = sample_size + 1;
  }
  mean_intensity = sum_of_intensity / sample_size;

  // calc standard dev
  double numerator = 0.0;
  for (auto& point : *cloud)
  {
    numerator = point.intensity - mean_intensity;
  }
  double std_dev_intensity = sqrt((numerator * numerator) / sample_size);
  double threshold = mean_intensity - (2 * std_dev_intensity);

  // Filter out points
  pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud->begin();
  while (it != cloud->end())
  {
    if (it->intensity < threshold)
    {
      it = cloud->erase(it);
    }
    else
    {
      it++;
    }
  }
}

void LidarProcessor::ground_segmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr ground)
{
  // Find inlier points to plane model: Segment ground plane
  RModel fit_model = Model::Plane{};

  // Use gravitational acceleration vector as plane normal
  // Ignoring orientation for now, as we assume that we are relatively flat on the ground
  NormalVector norm{0.0f, 0.0f, static_cast<float>(last_imu_ != nullptr ? last_imu_->linear_acceleration.z : -1.0f)};

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

void LidarProcessor::project_to_laserscan(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  // fill out configuration information
  // TODO: PARAMETERIZE
  sensor_msgs::msg::LaserScan output_scan;
  output_scan.header.stamp = this->get_clock()->now();
  output_scan.header.frame_id = "laser_link";
  output_scan.angle_min = -M_PI;
  output_scan.angle_max = M_PI;
  output_scan.angle_increment = M_PI / 180.0;
  output_scan.time_increment = 0.0; // TODO(dcutting133): we can assume we dont move fast enough for this to matter?
  output_scan.scan_time = 1.0 / 30.0;
  output_scan.range_min = std::numeric_limits<float>().infinity();
  output_scan.range_max = 0;

  // initialize output laserscan ranges to infinity
  uint32_t ranges_size = (uint32_t) std::ceil((output_scan.angle_max - output_scan.angle_min) / output_scan.angle_increment);
  output_scan.ranges.assign(ranges_size, std::numeric_limits<float>::infinity());
  output_scan.intensities.assign(ranges_size, 0.0);
  assert(output_scan.ranges.size() == ranges_size && "Somehow, the scan vector size isn't right.");

  // Fill laserscan with data.
  // Idea:  Basically you need to find vector between ReturnPoint(R) = (x,y) and LidarPose(L) = (x,y).
  //        You'd use the magnitude of vector RL for the range and you'd find the angle between RL and vector <0,0>
  //        which is the 0 rad in the lidar's view. You'd ignore Z because ground points are removed.
  // Note:  A simple method is just converting cartesian to polar coordinates.
  // x = rcos(theta)
  // y = rsin(theta)
  for (pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud->begin(); it != cloud->end(); it++)
  {
    float range = std::hypot(it->x, it->y); // sqrt of x^2 + y^2
    float angle = std::atan(it->y / it->x); // tan inverse of y/x

    // Sample the pointcloud so that we dont stuff more points into the laser scan than what we decided as our resolution
    uint32_t index = (uint32_t) std::floor((angle - output_scan.angle_min) / output_scan.angle_increment);
    assert((index > 0 || index < ranges_size) && "Accessing indices out of range.");
    
    // TODO: Parameterize epsilon
    if (range < output_scan.ranges[index] && std::abs(range - output_scan.ranges[index]) > 0.01)
    {
      output_scan.ranges[index] = range;
      output_scan.intensities[index] = it->intensity;
    }

    // Populate range min and maxes from data
    if (range < output_scan.range_min)
    {
      output_scan.range_min = range;
    }
    if (range > output_scan.range_max)
    {
      output_scan.range_max = range;
    }
  }
  filtered_ls_publisher_->publish(output_scan);
}

void LidarProcessor::raw_pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  msg->header.frame_id = "laser_link";  // fix weird pointcloud frame?
  if (debug_cloud_)
  {
    unfiltered_pc_publisher_->publish(*msg);
  }

  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>);

  // Convert to PCL data type
  pcl::fromROSMsg(*msg, *cloud);

  passthrough_stage(cloud);

  ground_segmentation(cloud, ground_points);

  // this is outputting a scan that already has ground points filtered out from above
  project_to_laserscan(cloud);

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

  // do last in case comparison between last_pcl and msg is necessary in filters
  last_pcl_ = msg;
}

}  // namespace LidarProcessor
