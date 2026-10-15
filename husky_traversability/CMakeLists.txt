cmake_minimum_required(VERSION 3.16)
project(husky_traversability)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

set(INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ------------------------------------------------------------------
# ROS-independent libraries
# ------------------------------------------------------------------
add_library(ground_segmenter SHARED src/ground_segmenter.cpp)
target_include_directories(ground_segmenter PUBLIC ${INCLUDE_DIRS})
target_link_libraries(ground_segmenter PUBLIC Eigen3::Eigen)

add_library(traversability_grid SHARED src/traversability_grid.cpp)
target_include_directories(traversability_grid PUBLIC ${INCLUDE_DIRS})
target_link_libraries(traversability_grid PUBLIC Eigen3::Eigen)

# ------------------------------------------------------------------
# Ground segmentation node
# ------------------------------------------------------------------
add_executable(ground_segmentation_node src/ground_segmentation_node.cpp)
target_link_libraries(ground_segmentation_node ground_segmenter)
ament_target_dependencies(ground_segmentation_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_eigen)

# ------------------------------------------------------------------
# Nav2 costmap layer plugin
# ------------------------------------------------------------------
add_library(traversability_layer SHARED src/traversability_layer.cpp)
target_link_libraries(traversability_layer traversability_grid)
ament_target_dependencies(traversability_layer
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_ros
  tf2_eigen
  nav2_costmap_2d
  pluginlib)

# Registers the plugin with pluginlib under the nav2_costmap_2d base class
pluginlib_export_plugin_description_file(nav2_costmap_2d traversability_layer_plugin.xml)

# ------------------------------------------------------------------
# Install
# ------------------------------------------------------------------
install(TARGETS ground_segmenter traversability_grid traversability_layer
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS ground_segmentation_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES traversability_layer_plugin.xml DESTINATION share/${PROJECT_NAME})

# ------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ground_segmenter test/test_ground_segmenter.cpp)
  target_link_libraries(test_ground_segmenter ground_segmenter)
  ament_add_gtest(test_traversability_grid test/test_traversability_grid.cpp)
  target_link_libraries(test_traversability_grid traversability_grid)
endif()

ament_package()
