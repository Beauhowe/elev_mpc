cmake_minimum_required(VERSION 3.10)
project(grid_map_sdf)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(${PROJECT_NAME}
  src/SignedDistance2d.cpp
  src/SignedDistanceField.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  grid_map_core
)

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
ament_export_dependencies(grid_map_core Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

  ament_add_gtest(${PROJECT_NAME}-test
    test/test3dLookup.cpp
    test/test_grid_map_sdf.cpp
    test/testDerivatives.cpp
    test/testPixelBorderDistance.cpp
    test/testSignedDistance2d.cpp
    test/testSignedDistance3d.cpp
  )
  target_include_directories(${PROJECT_NAME}-test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test/include
  )
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
  )
endif()

ament_package()
