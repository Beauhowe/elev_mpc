//
// Created by rgrandia on 10.06.20.
//

#include "convex_plane_decomposition_ros/ParameterLoading.h"

namespace convex_plane_decomposition {

template <typename T>
bool loadParameter(rclcpp::Node& node, const std::string& prefix, const std::string& param, T& value) {
  const std::string name = prefix + param;
  value = node.declare_parameter<T>(name, value);
  return true;
}

PreprocessingParameters loadPreprocessingParameters(rclcpp::Node& node, const std::string& prefix) {
  PreprocessingParameters preprocessingParameters;
  loadParameter(node, prefix, "resolution", preprocessingParameters.resolution);
  loadParameter(node, prefix, "kernelSize", preprocessingParameters.kernelSize);
  loadParameter(node, prefix, "numberOfRepeats", preprocessingParameters.numberOfRepeats);
  return preprocessingParameters;
}

contour_extraction::ContourExtractionParameters loadContourExtractionParameters(rclcpp::Node& node,
                                                                                const std::string& prefix) {
  contour_extraction::ContourExtractionParameters contourParams;
  loadParameter(node, prefix, "marginSize", contourParams.marginSize);
  return contourParams;
}

ransac_plane_extractor::RansacPlaneExtractorParameters loadRansacPlaneExtractorParameters(rclcpp::Node& node,
                                                                                          const std::string& prefix) {
  ransac_plane_extractor::RansacPlaneExtractorParameters ransacParams;
  loadParameter(node, prefix, "probability", ransacParams.probability);
  loadParameter(node, prefix, "min_points", ransacParams.min_points);
  loadParameter(node, prefix, "epsilon", ransacParams.epsilon);
  loadParameter(node, prefix, "cluster_epsilon", ransacParams.cluster_epsilon);
  loadParameter(node, prefix, "normal_threshold", ransacParams.normal_threshold);
  return ransacParams;
}

sliding_window_plane_extractor::SlidingWindowPlaneExtractorParameters loadSlidingWindowPlaneExtractorParameters(
    rclcpp::Node& node, const std::string& prefix) {
  sliding_window_plane_extractor::SlidingWindowPlaneExtractorParameters swParams;
  loadParameter(node, prefix, "kernel_size", swParams.kernel_size);
  loadParameter(node, prefix, "planarity_opening_filter", swParams.planarity_opening_filter);
  if (loadParameter(node, prefix, "plane_inclination_threshold_degrees", swParams.plane_inclination_threshold)) {
    swParams.plane_inclination_threshold = std::cos(swParams.plane_inclination_threshold * M_PI / 180.0);
  }
  if (loadParameter(node, prefix, "local_plane_inclination_threshold_degrees", swParams.local_plane_inclination_threshold)) {
    swParams.local_plane_inclination_threshold = std::cos(swParams.local_plane_inclination_threshold * M_PI / 180.0);
  }
  loadParameter(node, prefix, "plane_patch_error_threshold", swParams.plane_patch_error_threshold);
  loadParameter(node, prefix, "min_number_points_per_label", swParams.min_number_points_per_label);
  loadParameter(node, prefix, "connectivity", swParams.connectivity);
  loadParameter(node, prefix, "include_ransac_refinement", swParams.include_ransac_refinement);
  loadParameter(node, prefix, "global_plane_fit_distance_error_threshold", swParams.global_plane_fit_distance_error_threshold);
  loadParameter(node, prefix, "global_plane_fit_angle_error_threshold_degrees",
                swParams.global_plane_fit_angle_error_threshold_degrees);
  return swParams;
}

PostprocessingParameters loadPostprocessingParameters(rclcpp::Node& node, const std::string& prefix) {
  PostprocessingParameters postprocessingParameters;
  loadParameter(node, prefix, "extracted_planes_height_offset", postprocessingParameters.extracted_planes_height_offset);
  loadParameter(node, prefix, "nonplanar_height_offset", postprocessingParameters.nonplanar_height_offset);
  loadParameter(node, prefix, "nonplanar_horizontal_offset", postprocessingParameters.nonplanar_horizontal_offset);
  loadParameter(node, prefix, "smoothing_dilation_size", postprocessingParameters.smoothing_dilation_size);
  loadParameter(node, prefix, "smoothing_box_kernel_size", postprocessingParameters.smoothing_box_kernel_size);
  loadParameter(node, prefix, "smoothing_gauss_kernel_size", postprocessingParameters.smoothing_gauss_kernel_size);
  return postprocessingParameters;
}

}  // namespace convex_plane_decomposition
