cmake_minimum_required(VERSION 3.10)
project(convex_plane_decomposition_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_filters_rsl REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(convex_plane_decomposition REQUIRED)
find_package(convex_plane_decomposition_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(PACKAGE_DEPENDENCIES
  rclcpp
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  grid_map_filters_rsl
  geometry_msgs
  std_msgs
  convex_plane_decomposition
  convex_plane_decomposition_msgs
  tf2
  tf2_ros
  visualization_msgs
)

add_library(${PROJECT_NAME}
  src/ConvexPlaneDecompositionRos.cpp
  src/MessageConversion.cpp
  src/ParameterLoading.cpp
  src/RosVisualizations.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} Eigen3::Eigen)

add_executable(${PROJECT_NAME}_node src/ConvexPlaneDecompositionNode.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node ${PACKAGE_DEPENDENCIES})

add_executable(${PROJECT_NAME}_add_noise src/noiseNode.cpp)
target_link_libraries(${PROJECT_NAME}_add_noise ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_add_noise ${PACKAGE_DEPENDENCIES})

add_executable(${PROJECT_NAME}_save_elevationmap src/SaveElevationMapAsImageNode.cpp)
target_link_libraries(${PROJECT_NAME}_save_elevationmap ${PROJECT_NAME} ${OpenCV_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_save_elevationmap ${PACKAGE_DEPENDENCIES})

add_executable(${PROJECT_NAME}_world_box_terrain src/WorldBoxTerrainNode.cpp)
target_link_libraries(${PROJECT_NAME}_world_box_terrain ${PROJECT_NAME} tinyxml2::tinyxml2)
ament_target_dependencies(${PROJECT_NAME}_world_box_terrain ${PACKAGE_DEPENDENCIES})

add_executable(${PROJECT_NAME}_approximation_demo_node src/ConvexApproximationDemoNode.cpp)
target_link_libraries(${PROJECT_NAME}_approximation_demo_node ${PROJECT_NAME} ${OpenCV_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_approximation_demo_node ${PACKAGE_DEPENDENCIES})

add_executable(${PROJECT_NAME}_TestShapeGrowing test/TestShapeGrowing.cpp)
target_link_libraries(${PROJECT_NAME}_TestShapeGrowing ${PROJECT_NAME} ${OpenCV_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_TestShapeGrowing ${PACKAGE_DEPENDENCIES})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_add_noise
  ${PROJECT_NAME}_save_elevationmap
  ${PROJECT_NAME}_world_box_terrain
  ${PROJECT_NAME}_approximation_demo_node
  ${PROJECT_NAME}_TestShapeGrowing
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config data launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_DEPENDENCIES} Eigen3)

ament_package()
