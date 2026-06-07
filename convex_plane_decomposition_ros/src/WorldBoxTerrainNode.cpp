#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <tinyxml2.h>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <convex_plane_decomposition/PlaneDecompositionPipeline.h>
#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "convex_plane_decomposition_ros/MessageConversion.h"
#include "convex_plane_decomposition_ros/ParameterLoading.h"
#include "convex_plane_decomposition_ros/RosVisualizations.h"

namespace convex_plane_decomposition {
namespace {

struct Pose2d {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
};

struct Box {
  Pose2d pose;
  double sizeX = 0.0;
  double sizeY = 0.0;
  double sizeZ = 0.0;
};

std::vector<double> parseDoubles(const char* text) {
  std::vector<double> values;
  if (text == nullptr) {
    return values;
  }
  std::istringstream stream(text);
  double value = 0.0;
  while (stream >> value) {
    values.push_back(value);
  }
  return values;
}

Pose2d parsePose(const tinyxml2::XMLElement* element) {
  Pose2d pose;
  if (element == nullptr) {
    return pose;
  }
  const auto values = parseDoubles(element->GetText());
  if (values.size() >= 3) {
    pose.x = values[0];
    pose.y = values[1];
    pose.z = values[2];
  }
  if (values.size() >= 6) {
    pose.yaw = values[5];
  }
  return pose;
}

Pose2d compose(const Pose2d& parent, const Pose2d& child) {
  const double c = std::cos(parent.yaw);
  const double s = std::sin(parent.yaw);
  Pose2d result;
  result.x = parent.x + c * child.x - s * child.y;
  result.y = parent.y + s * child.x + c * child.y;
  result.z = parent.z + child.z;
  result.yaw = parent.yaw + child.yaw;
  return result;
}

void addBoxCornersToBounds(const Box& box, double& minX, double& minY, double& maxX, double& maxY) {
  const double c = std::cos(box.pose.yaw);
  const double s = std::sin(box.pose.yaw);
  const double hx = 0.5 * box.sizeX;
  const double hy = 0.5 * box.sizeY;
  for (const double localX : {-hx, hx}) {
    for (const double localY : {-hy, hy}) {
      const double x = box.pose.x + c * localX - s * localY;
      const double y = box.pose.y + s * localX + c * localY;
      minX = std::min(minX, x);
      minY = std::min(minY, y);
      maxX = std::max(maxX, x);
      maxY = std::max(maxY, y);
    }
  }
}

bool containsXY(const Box& box, const grid_map::Position& position) {
  const double dx = position.x() - box.pose.x;
  const double dy = position.y() - box.pose.y;
  const double c = std::cos(box.pose.yaw);
  const double s = std::sin(box.pose.yaw);
  const double localX = c * dx + s * dy;
  const double localY = -s * dx + c * dy;
  return std::abs(localX) <= 0.5 * box.sizeX && std::abs(localY) <= 0.5 * box.sizeY;
}

}  // namespace

class WorldBoxTerrainNode {
 public:
  explicit WorldBoxTerrainNode(const rclcpp::Node::SharedPtr& node)
      : node_(node), tfBuffer_(node_->get_clock()), tfListener_(tfBuffer_) {
    loadParameters();
    boxes_ = loadWorldBoxes(worldFile_);
    fullMap_ = buildFullMap(boxes_);

    rawMapPublisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(rawElevationTopic_, 1);
    filteredMapPublisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(filteredMapTopic_, 1);
    boundaryPublisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(boundariesTopic_, 1);
    insetPublisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(insetsTopic_, 1);
    regionPublisher_ = node_->create_publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>(planarTerrainTopic_, 1);

    PlaneDecompositionPipeline::Config config;
    config.preprocessingParameters = loadPreprocessingParameters(*node_, "preprocessing.");
    config.contourExtractionParameters = loadContourExtractionParameters(*node_, "contour_extraction.");
    config.ransacPlaneExtractorParameters = loadRansacPlaneExtractorParameters(*node_, "ransac_plane_refinement.");
    config.slidingWindowPlaneExtractorParameters = loadSlidingWindowPlaneExtractorParameters(*node_, "sliding_window_plane_extractor.");
    config.postprocessingParameters = loadPostprocessingParameters(*node_, "postprocessing.");
    planeDecompositionPipeline_ = std::make_unique<PlaneDecompositionPipeline>(config);

    timer_ = node_->create_wall_timer(std::chrono::duration<double>(1.0 / frequency_), std::bind(&WorldBoxTerrainNode::update, this));

    RCLCPP_INFO(node_->get_logger(), "[WorldBoxTerrainNode] loaded %zu box collisions from %s", boxes_.size(), worldFile_.c_str());
  }

  ~WorldBoxTerrainNode() {
    if (callbackTimer_.getNumTimedIntervals() == 0 || planeDecompositionPipeline_ == nullptr) {
      return;
    }

    std::stringstream infoStream;
    infoStream << "\n########################################################################\n";
    infoStream << "The benchmarking is computed over " << callbackTimer_.getNumTimedIntervals() << " iterations.\n";
    infoStream << "PlaneExtraction Benchmarking    : Average time [ms], Max time [ms]\n";
    auto printLine = [](std::string name, const Timer& timer) {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(2);
      ss << "\t" << name << "\t: " << std::setw(17) << timer.getAverageInMilliseconds() << ", " << std::setw(13)
         << timer.getMaxIntervalInMilliseconds() << "\n";
      return ss.str();
    };
    infoStream << printLine("Pre-process        ", planeDecompositionPipeline_->getPrepocessTimer());
    infoStream << printLine("Sliding window     ", planeDecompositionPipeline_->getSlidingWindowTimer());
    infoStream << printLine("Contour extraction ", planeDecompositionPipeline_->getContourExtractionTimer());
    infoStream << printLine("Post-process       ", planeDecompositionPipeline_->getPostprocessTimer());
    infoStream << printLine("Total callback     ", callbackTimer_);
    std::cerr << infoStream.str() << std::endl;
  }

 private:
  void loadParameters() {
    worldFile_ = node_->declare_parameter<std::string>("world_file", "");
    targetFrameId_ = node_->declare_parameter<std::string>("target_frame_id", "odom");
    baseFrameId_ = node_->declare_parameter<std::string>("base_frame_id", "base");
    elevationLayer_ = node_->declare_parameter<std::string>("height_layer", "elevation");
    subMapWidth_ = node_->declare_parameter<double>("submap.width", 3.0);
    subMapLength_ = node_->declare_parameter<double>("submap.length", 3.0);
    resolution_ = node_->declare_parameter<double>("resolution", 0.03);
    mapPadding_ = node_->declare_parameter<double>("map_padding", 1.0);
    groundHeight_ = node_->declare_parameter<double>("ground_height", 0.0);
    frequency_ = node_->declare_parameter<double>("frequency", 20.0);
    useRobotSubmap_ = node_->declare_parameter<bool>("use_robot_submap", true);
    publishToController_ = node_->declare_parameter<bool>("publish_to_controller", true);
    rawElevationTopic_ = node_->declare_parameter<std::string>("raw_elevation_topic", "/elevation_mapping/elevation_map_raw");
    planarTerrainTopic_ = node_->declare_parameter<std::string>("planar_terrain_topic", "planar_terrain");
    filteredMapTopic_ = node_->declare_parameter<std::string>("filtered_map_topic", "filtered_map");
    boundariesTopic_ = node_->declare_parameter<std::string>("boundaries_topic", "boundaries");
    insetsTopic_ = node_->declare_parameter<std::string>("insets_topic", "insets");

    if (worldFile_.empty()) {
      throw std::runtime_error("Parameter `world_file` is required.");
    }
    if (resolution_ <= 0.0) {
      throw std::runtime_error("Parameter `resolution` must be positive.");
    }
    if (frequency_ <= 0.0) {
      throw std::runtime_error("Parameter `frequency` must be positive.");
    }
  }

  std::vector<Box> loadWorldBoxes(const std::string& worldFile) {
    tinyxml2::XMLDocument document;
    const auto result = document.LoadFile(worldFile.c_str());
    if (result != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error("Could not load world file: " + worldFile);
    }

    const auto* sdf = document.FirstChildElement("sdf");
    const auto* world = sdf != nullptr ? sdf->FirstChildElement("world") : nullptr;
    if (world == nullptr) {
      throw std::runtime_error("World file does not contain sdf/world: " + worldFile);
    }

    std::vector<Box> boxes;
    for (const auto* model = world->FirstChildElement("model"); model != nullptr; model = model->NextSiblingElement("model")) {
      const Pose2d modelPose = parsePose(model->FirstChildElement("pose"));
      for (const auto* link = model->FirstChildElement("link"); link != nullptr; link = link->NextSiblingElement("link")) {
        const Pose2d linkPose = compose(modelPose, parsePose(link->FirstChildElement("pose")));
        for (const auto* collision = link->FirstChildElement("collision"); collision != nullptr;
             collision = collision->NextSiblingElement("collision")) {
          const auto* geometry = collision->FirstChildElement("geometry");
          const auto* boxElement = geometry != nullptr ? geometry->FirstChildElement("box") : nullptr;
          const auto* sizeElement = boxElement != nullptr ? boxElement->FirstChildElement("size") : nullptr;
          const auto size = parseDoubles(sizeElement != nullptr ? sizeElement->GetText() : nullptr);
          if (size.size() < 3) {
            continue;
          }

          Box box;
          box.pose = compose(linkPose, parsePose(collision->FirstChildElement("pose")));
          box.sizeX = size[0];
          box.sizeY = size[1];
          box.sizeZ = size[2];
          boxes.push_back(box);
        }
      }
    }

    if (boxes.empty()) {
      throw std::runtime_error("World file does not contain box collisions: " + worldFile);
    }
    return boxes;
  }

  grid_map::GridMap buildFullMap(const std::vector<Box>& boxes) {
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();
    for (const auto& box : boxes) {
      addBoxCornersToBounds(box, minX, minY, maxX, maxY);
    }

    minX -= mapPadding_;
    minY -= mapPadding_;
    maxX += mapPadding_;
    maxY += mapPadding_;

    grid_map::GridMap map({elevationLayer_});
    map.setFrameId(targetFrameId_);
    map.setGeometry(grid_map::Length(maxX - minX, maxY - minY), resolution_, grid_map::Position(0.5 * (minX + maxX), 0.5 * (minY + maxY)));
    map[elevationLayer_].setConstant(groundHeight_);

    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      grid_map::Position position;
      map.getPosition(*iterator, position);
      double height = groundHeight_;
      for (const auto& box : boxes) {
        if (containsXY(box, position)) {
          height = std::max(height, box.pose.z + 0.5 * box.sizeZ);
        }
      }
      map.at(elevationLayer_, *iterator) = height;
    }
    return map;
  }

  bool getSubmapCenter(grid_map::Position& position) {
    if (!useRobotSubmap_) {
      position = fullMap_.getPosition();
      return true;
    }

    try {
      const auto transform = tfBuffer_.lookupTransform(targetFrameId_, baseFrameId_, tf2::TimePointZero);
      position = grid_map::Position(transform.transform.translation.x, transform.transform.translation.y);
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[WorldBoxTerrainNode] %s", ex.what());
      return false;
    }
  }

  void update() {
    callbackTimer_.startTimer();

    const auto now = node_->get_clock()->now();
    fullMap_.setTimestamp(now.nanoseconds());
    auto rawMessage = grid_map::GridMapRosConverter::toMessage(fullMap_);
    rawMapPublisher_->publish(*rawMessage);

    grid_map::Position submapPosition;
    if (!getSubmapCenter(submapPosition)) {
      callbackTimer_.endTimer();
      return;
    }

    bool success = false;
    grid_map::GridMap elevationMap = fullMap_.getSubmap(submapPosition, Eigen::Array2d(subMapLength_, subMapWidth_), success);
    if (!success) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "[WorldBoxTerrainNode] Could not extract submap");
      callbackTimer_.endTimer();
      return;
    }

    const grid_map::Matrix elevationRaw = elevationMap.get(elevationLayer_);
    planeDecompositionPipeline_->update(std::move(elevationMap), elevationLayer_);
    auto& planarTerrain = planeDecompositionPipeline_->getPlanarTerrain();

    if (publishToController_) {
      regionPublisher_->publish(toMessage(planarTerrain));
    }

    planarTerrain.gridMap.add("elevation_raw", elevationRaw);
    planarTerrain.gridMap.add("segmentation");
    planeDecompositionPipeline_->getSegmentation(planarTerrain.gridMap.get("segmentation"));

    auto outputMessage = grid_map::GridMapRosConverter::toMessage(planarTerrain.gridMap);
    filteredMapPublisher_->publish(*outputMessage);

    const double lineWidth = 0.005;
    boundaryPublisher_->publish(convertBoundariesToRosMarkers(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
                                                             planarTerrain.gridMap.getTimestamp(), lineWidth));
    insetPublisher_->publish(convertInsetsToRosMarkers(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
                                                      planarTerrain.gridMap.getTimestamp(), lineWidth));

    callbackTimer_.endTimer();
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr rawMapPublisher_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr filteredMapPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundaryPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr insetPublisher_;
  rclcpp::Publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr regionPublisher_;

  std::string worldFile_;
  std::string targetFrameId_;
  std::string baseFrameId_;
  std::string elevationLayer_;
  std::string rawElevationTopic_;
  std::string planarTerrainTopic_;
  std::string filteredMapTopic_;
  std::string boundariesTopic_;
  std::string insetsTopic_;
  double subMapWidth_ = 3.0;
  double subMapLength_ = 3.0;
  double resolution_ = 0.03;
  double mapPadding_ = 1.0;
  double groundHeight_ = 0.0;
  double frequency_ = 20.0;
  bool useRobotSubmap_ = true;
  bool publishToController_ = true;

  std::vector<Box> boxes_;
  grid_map::GridMap fullMap_;
  std::unique_ptr<PlaneDecompositionPipeline> planeDecompositionPipeline_;
  Timer callbackTimer_;
};

}  // namespace convex_plane_decomposition

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions nodeOptions;
  nodeOptions.use_global_arguments(true);
  auto node = std::make_shared<rclcpp::Node>("convex_plane_decomposition_ros", nodeOptions);

  try {
    auto worldBoxTerrainNode = std::make_shared<convex_plane_decomposition::WorldBoxTerrainNode>(node);
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "[WorldBoxTerrainNode] %s", error.what());
  }

  rclcpp::shutdown();
  return 0;
}
