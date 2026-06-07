//
// Created by rgrandia on 25.10.21.
//

#include <rclcpp/rclcpp.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <grid_map_filters_rsl/inpainting.hpp>
#include <grid_map_filters_rsl/smoothing.hpp>

namespace {

double noiseUniform;
double noiseGauss;
double outlierPercentage;
bool blur;
double frequency;
std::string elevationMapTopicIn;
std::string elevationMapTopicOut;
std::string elevationLayer;
rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher;
grid_map::GridMap::Matrix noiseLayer;

void createNoise(size_t row, size_t col) {
  grid_map::GridMap::Matrix u1 = 0.5 * grid_map::GridMap::Matrix::Random(row, col).array() + 0.5;
  grid_map::GridMap::Matrix u2 = 0.5 * grid_map::GridMap::Matrix::Random(row, col).array() + 0.5;
  grid_map::GridMap::Matrix gauss01 =
      u1.binaryExpr(u2, [&](float v1, float v2) { return static_cast<float>(std::sqrt(-2.0f * log(v1)) * cos(2.0f * M_PIf32 * v2)); });

  noiseLayer = noiseUniform * grid_map::GridMap::Matrix::Random(row, col) + noiseGauss * gauss01;
}

void callback(const grid_map_msgs::msg::GridMap::SharedPtr message) {
  grid_map::GridMap messageMap;
  grid_map::GridMapRosConverter::fromMessage(*message, messageMap);

  if (blur) {
    auto originalMap = messageMap.get(elevationLayer);

    grid_map::inpainting::minValues(messageMap, elevationLayer, "i");
    grid_map::smoothing::boxBlur(messageMap, "i", elevationLayer, 3, 1);
    messageMap.get(elevationLayer) = (originalMap.array().isFinite()).select(messageMap.get(elevationLayer), originalMap);
  }

  auto& elevation = messageMap.get(elevationLayer);
  if (noiseLayer.size() != elevation.size()) {
    createNoise(elevation.rows(), elevation.cols());
  }

  elevation += noiseLayer;

  auto messageMapOut = grid_map::GridMapRosConverter::toMessage(messageMap);
  publisher->publish(*messageMapOut);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("noise_node");

  frequency = node->declare_parameter<double>("frequency", 30.0);
  noiseGauss = node->declare_parameter<double>("noiseGauss", 0.0);
  noiseUniform = node->declare_parameter<double>("noiseUniform", 0.0);
  blur = node->declare_parameter<bool>("blur", false);
  outlierPercentage = node->declare_parameter<double>("outlier_percentage", 0.0);
  elevationMapTopicIn = node->declare_parameter<std::string>("elevation_topic_in", "");
  elevationMapTopicOut = node->declare_parameter<std::string>("elevation_topic_out", "");
  elevationLayer = node->declare_parameter<std::string>("height_layer", "elevation");

  if (elevationMapTopicIn.empty()) {
    RCLCPP_ERROR(node->get_logger(), "[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `elevation_topic_in`.");
    return 1;
  }
  if (elevationMapTopicOut.empty()) {
    RCLCPP_ERROR(node->get_logger(), "[ConvexPlaneExtractionROS:NoiseNode] Could not read parameter `elevation_topic_out`.");
    return 1;
  }

  publisher = node->create_publisher<grid_map_msgs::msg::GridMap>(elevationMapTopicOut, 1);
  auto elevationMapSubscriber = node->create_subscription<grid_map_msgs::msg::GridMap>(elevationMapTopicIn, 1, &callback);

  rclcpp::Rate rate(frequency);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
