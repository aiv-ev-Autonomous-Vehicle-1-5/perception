cmake_minimum_required(VERSION 3.8)
project(velodyne_ground_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create shared library component
add_library(ground_segmentation_component SHARED
  src/ground_segmentation_component.cpp
)

target_include_directories(ground_segmentation_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(ground_segmentation_component PUBLIC cxx_std_17)

ament_target_dependencies(ground_segmentation_component
  rclcpp
  rclcpp_components
  sensor_msgs
)

# Register the component with rclcpp_components
rclcpp_components_register_nodes(ground_segmentation_component
  "velodyne_ground_segmentation::GroundSegmentationComponent")

# Install the library
install(TARGETS ground_segmentation_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(ground_segmentation_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
)

ament_package()
