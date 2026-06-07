#include "convex_plane_decomposition_ros/ConvexPlaneDecompositionRos.h"

#include <iomanip>
#include <sstream>

#include <tf2/time.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/GridMapCvProcessing.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <convex_plane_decomposition/PlaneDecompositionPipeline.h>

#include "convex_plane_decomposition_ros/MessageConversion.h"
#include "convex_plane_decomposition_ros/ParameterLoading.h"
#include "convex_plane_decomposition_ros/RosVisualizations.h"

namespace convex_plane_decomposition {

ConvexPlaneExtractionROS::ConvexPlaneExtractionROS(const rclcpp::Node::SharedPtr& node)
    : node_(node), tfBuffer_(node_->get_clock()), tfListener_(tfBuffer_) {
  bool parametersLoaded = loadParameters();

  if (parametersLoaded) {
    elevationMapSubscriber_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(
        elevationMapTopic_, rclcpp::QoS(1), std::bind(&ConvexPlaneExtractionROS::callback, this, std::placeholders::_1));
    filteredmapPublisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>("filtered_map", 1);
    boundaryPublisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("boundaries", 1);
    insetPublisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("insets", 1);
    regionPublisher_ = node_->create_publisher<convex_plane_decomposition_msgs::msg::PlanarTerrain>("planar_terrain", 1);
  }
}

ConvexPlaneExtractionROS::~ConvexPlaneExtractionROS() {
  if (callbackTimer_.getNumTimedIntervals() > 0 && planeDecompositionPipeline_ != nullptr) {
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
}
bool ConvexPlaneExtractionROS::loadParameters() {
  elevationMapTopic_ = node_->declare_parameter<std::string>("elevation_topic", "");
  targetFrameId_ = node_->declare_parameter<std::string>("target_frame_id", "");
  elevationLayer_ = node_->declare_parameter<std::string>("height_layer", "elevation");
  subMapWidth_ = node_->declare_parameter<double>("submap.width", 3.0);
  subMapLength_ = node_->declare_parameter<double>("submap.length", 3.0);
  publishToController_ = node_->declare_parameter<bool>("publish_to_controller", true);

  if (elevationMapTopic_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[ConvexPlaneExtractionROS] Could not read parameter `elevation_topic`.");
    return false;
  }
  if (targetFrameId_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[ConvexPlaneExtractionROS] Could not read parameter `target_frame_id`.");
    return false;
  }

  PlaneDecompositionPipeline::Config config;
  config.preprocessingParameters = loadPreprocessingParameters(*node_, "preprocessing.");
  config.contourExtractionParameters = loadContourExtractionParameters(*node_, "contour_extraction.");
  config.ransacPlaneExtractorParameters = loadRansacPlaneExtractorParameters(*node_, "ransac_plane_refinement.");
  config.slidingWindowPlaneExtractorParameters = loadSlidingWindowPlaneExtractorParameters(*node_, "sliding_window_plane_extractor.");
  config.postprocessingParameters = loadPostprocessingParameters(*node_, "postprocessing.");

  planeDecompositionPipeline_ = std::make_unique<PlaneDecompositionPipeline>(config);

  return true;
}

void ConvexPlaneExtractionROS::callback(const grid_map_msgs::msg::GridMap::SharedPtr message) {
  callbackTimer_.startTimer();

  grid_map::GridMap messageMap;
  std::vector<std::string> layers{elevationLayer_};
  grid_map::GridMapRosConverter::fromMessage(*message, messageMap, layers, false, false);
  if (!containsFiniteValue(messageMap.get(elevationLayer_))) {
    RCLCPP_WARN(node_->get_logger(), "[ConvexPlaneExtractionROS] map does not contain any values");
    callbackTimer_.endTimer();
    return;
  }

  if (targetFrameId_ != messageMap.getFrameId()) {
    std::string errorMsg;
    if (tfBuffer_.canTransform(targetFrameId_, messageMap.getFrameId(), tf2::TimePointZero, tf2::durationFromSec(0.1), &errorMsg)) {
      messageMap.setFrameId(targetFrameId_);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "[ConvexPlaneExtractionROS] %s", errorMsg.c_str());
      callbackTimer_.endTimer();
      return;
    }
  }

  bool success;
  const grid_map::Position submapPosition = [&]() {
    grid_map::Index centerIndex;
    grid_map::Position centerPosition;
    messageMap.getIndex(messageMap.getPosition(), centerIndex);
    messageMap.getPosition(centerIndex, centerPosition);
    return centerPosition;
  }();
  grid_map::GridMap elevationMap = messageMap.getSubmap(submapPosition, Eigen::Array2d(subMapLength_, subMapWidth_), success);
  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "[ConvexPlaneExtractionROS] Could not extract submap");
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
  filteredmapPublisher_->publish(*outputMessage);

  const double lineWidth = 0.005;
  boundaryPublisher_->publish(convertBoundariesToRosMarkers(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
                                                           planarTerrain.gridMap.getTimestamp(), lineWidth));
  insetPublisher_->publish(convertInsetsToRosMarkers(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
                                                    planarTerrain.gridMap.getTimestamp(), lineWidth));

  callbackTimer_.endTimer();
}

Eigen::Isometry3d ConvexPlaneExtractionROS::getTransformToTargetFrame(const std::string& sourceFrame) {
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer_.lookupTransform(targetFrameId_, sourceFrame, tf2::TimePointZero);
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(node_->get_logger(), "[ConvexPlaneExtractionROS] %s", ex.what());
    return Eigen::Isometry3d::Identity();
  }

  Eigen::Isometry3d transformation = Eigen::Isometry3d::Identity();
  transformation.translation().x() = transformStamped.transform.translation.x;
  transformation.translation().y() = transformStamped.transform.translation.y;
  transformation.translation().z() = transformStamped.transform.translation.z;

  Eigen::Quaterniond rotationQuaternion(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x,
                                        transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
  transformation.linear() = rotationQuaternion.toRotationMatrix();
  return transformation;
}

}  // namespace convex_plane_decomposition
