#pragma once

#include <lidar_processor/lidar_processor.hpp>

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