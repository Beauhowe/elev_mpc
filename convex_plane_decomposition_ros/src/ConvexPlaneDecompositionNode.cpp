#include "convex_plane_decomposition_ros/ConvexPlaneDecompositionRos.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.use_global_arguments(true);
  auto node = std::make_shared<rclcpp::Node>("convex_plane_decomposition_ros", node_options);

  const double frequency = node->declare_parameter<double>("frequency", 20.0);
  convex_plane_decomposition::ConvexPlaneExtractionROS convex_plane_decomposition_ros(node);

  rclcpp::Rate rate(frequency);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
