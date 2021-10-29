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

#include "lidar_processor/lidar_processor.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>


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
    "/lidar/raw_points", 10,
    std::bind(&LidarProcessor::raw_pc_callback, this, std::placeholders::_1));

  // Frame ID/timing fix
  unfiltered_ls_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "/lidar/unfilterered_scan", rclcpp::SensorDataQoS());

  unfiltered_pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/lidar/unfiltered_points", rclcpp::SensorDataQoS());

  // Filtered outputs
  filtered_ls_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "/lidar/filterered_scan", rclcpp::SensorDataQoS());

  filtered_pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/lidar/filtered_points", rclcpp::SensorDataQoS());
}

void LidarProcessor::raw_ls_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // msg->header.stamp = this->get_clock()->now();  // rewrite time
  msg->header.frame_id = "laser_link";  // fix weird scan frame?
  unfiltered_ls_publisher_->publish(*msg);

  // begin filter
  filtered_ls_publisher_->publish(*msg);
}

void LidarProcessor::raw_pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // msg->header.stamp = this->get_clock()->now();  // rewrite time
  msg->header.frame_id = "laser_link";  // fix weird pointcloud frame?
  unfiltered_pc_publisher_->publish(*msg);

  // *** Begin filter ***
  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCL data type
  pcl::fromROSMsg(*msg, *cloud);
  
  // Perform the Passthrough filtering
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 0.5);
  pass.filter(*cloud);
  
  // Perform Radius Statistical Outlier Filtering
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(0.8);
  outrem.setMinNeighborsInRadius (2);
  outrem.setKeepOrganized(true);
  outrem.filter (*cloud);

  // Perform Voxel Grid filtering Filtering
  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize (0.01f, 0.01f, 0.01f);
  vox.filter(*cloud);
  
  // Perform Plane Segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  
  // Perform Surface reconstruction by using Moving Least Squares
  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;
  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals (true);
  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);
  // Reconstruct
  mls.process (mls_points);
  
  // Convert to ROS data type
  sensor_msgs::msg::PointCloud2::SharedPtr output(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*cloud, *output);

  // Publish filtered cloud
  filtered_pc_publisher_->publish(*output);
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
