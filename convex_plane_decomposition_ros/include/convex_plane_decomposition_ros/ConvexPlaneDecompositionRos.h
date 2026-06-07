#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <convex_plane_decomposition/Timer.h>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>

namespace convex_plane_decomposition {

class PlaneDecompositionPipeline;

class ConvexPlaneExtractionROS {
 public:
  explicit ConvexPlaneExtractionROS(const rclcpp::Node::SharedPtr& node);

  ~ConvexPlaneExtractionROS();

 private:
  bool loadParameters();

  void callback(const grid_map_msgs::msg::GridMap::SharedPtr message);

  Eigen::Isometry3d getTransformToTargetFrame(const std::string& sourceFrame);

  rclcpp::Node::SharedPtr node_;

  std::string elevationMapTopic_;
  std::string elevationLayer_;
  std::string targetFrameId_;
  double subMapWidth_{};
  double subMapLength_{};
  bool publishToController_{};

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr elevationMapSubscriber_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr filteredmapPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundaryPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr insetPublisher_;
  rclcpp::Publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr regionPublisher_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  std::unique_ptr<PlaneDecompositionPipeline> planeDecompositionPipeline_;

  Timer callbackTimer_;
};

}  // namespace convex_plane_decomposition
