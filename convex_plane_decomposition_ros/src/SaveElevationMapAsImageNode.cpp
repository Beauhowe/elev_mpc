//
// Created by rgrandia on 11.06.20.
//

#include <rclcpp/rclcpp.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <opencv2/imgcodecs.hpp>

namespace {

int count = 0;
double frequency;
std::string elevationMapTopic;
std::string elevationLayer;
std::string imageName;

void callback(const grid_map_msgs::msg::GridMap::SharedPtr message) {
  grid_map::GridMap messageMap;
  grid_map::GridMapRosConverter::fromMessage(*message, messageMap);

  const auto& data = messageMap[elevationLayer];
  float maxHeight = std::numeric_limits<float>::lowest();
  float minHeight = std::numeric_limits<float>::max();
  for (int i = 0; i < data.rows(); i++) {
    for (int j = 0; j < data.cols(); j++) {
      const auto value = data(i, j);
      if (!std::isnan(value)) {
        maxHeight = std::max(maxHeight, value);
        minHeight = std::min(minHeight, value);
      }
    }
  }

  cv::Mat image;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(messageMap, elevationLayer, CV_8UC1, minHeight, maxHeight, image);

  int range = 100 * (maxHeight - minHeight);
  cv::imwrite(imageName + "_" + std::to_string(count++) + "_" + std::to_string(range) + "cm.png", image);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("save_elevation_map_to_image");

  frequency = node->declare_parameter<double>("frequency", 0.1);
  elevationMapTopic = node->declare_parameter<std::string>("elevation_topic", "");
  elevationLayer = node->declare_parameter<std::string>("height_layer", "elevation");
  imageName = node->declare_parameter<std::string>("imageName", "elevationMap");

  if (elevationMapTopic.empty()) {
    RCLCPP_ERROR(node->get_logger(), "[ConvexPlaneExtractionROS] Could not read parameter `elevation_topic`.");
    return 1;
  }

  auto elevationMapSubscriber = node->create_subscription<grid_map_msgs::msg::GridMap>(elevationMapTopic, 1, &callback);

  rclcpp::Rate rate(frequency);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
