cmake_minimum_required(VERSION 3.10)
project(convex_plane_decomposition)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_filters_rsl REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(CGAL REQUIRED)
find_package(GMP REQUIRED)
find_package(MPFR REQUIRED)

add_library(${PROJECT_NAME}
  src/contour_extraction/ContourExtraction.cpp
  src/contour_extraction/Upsampling.cpp
  src/ransac/RansacPlaneExtractor.cpp
  src/sliding_window_plane_extraction/SlidingWindowPlaneExtractor.cpp
  src/ConvexRegionGrowing.cpp
  src/Draw.cpp
  src/GridMapPreprocessing.cpp
  src/LoadGridmapFromImage.cpp
  src/PlanarRegion.cpp
  src/PlaneDecompositionPipeline.cpp
  src/Postprocessing.cpp
  src/SegmentedPlaneProjection.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  grid_map_core
  grid_map_cv
  grid_map_filters_rsl
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
  ${GMP_LIBRARIES}
  ${MPFR_LIBRARIES}
)


target_compile_options(${PROJECT_NAME} PUBLIC -DCGAL_HAS_THREADS)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(grid_map_core grid_map_cv grid_map_filters_rsl Eigen3)

ament_package()
