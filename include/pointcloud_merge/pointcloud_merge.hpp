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

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace message_filters;

class PointCloudMerge : public rclcpp::Node
{
public:
    explicit PointCloudMerge(rclcpp::NodeOptions options);

private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg1, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg2);
    void update_params();

    // message filters
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_subscription_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> camera_subscription_;
    typedef sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_pc_publisher_;

    // Tf stuff
    std::shared_ptr<tf2_ros::TransformListener> transform_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    float tf_timeout;

    /// Transform to apply to the camera points
    std::optional<geometry_msgs::msg::TransformStamped> camera_trans{};
    std::string camera_trans_source;
    std::string camera_trans_dest;

    // timers
    rclcpp::TimerBase::SharedPtr param_update_timer_;
};
