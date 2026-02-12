cmake_minimum_required(VERSION 3.8)
project(image_filtering)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV 4.5.4 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)



# ---- Core library (no ROS) ----
set(LIB "${PROJECT_NAME}_core")

add_library(${LIB} SHARED
  src/lib/utilities.cpp
)

target_link_libraries(${LIB} PUBLIC
  ${OpenCV_LIBS}
)

target_include_directories(${LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${LIB} PUBLIC
  spdlog
  fmt
)

# ---- ROS library (depends on core) ----
set(ROS_LIB "${PROJECT_NAME}_component")

add_library(${ROS_LIB} SHARED
  src/ros/image_filtering_ros.cpp
  src/ros/image_filtering_ros_utils.cpp
)

target_link_libraries(${ROS_LIB} PUBLIC
  ${LIB}
)

target_include_directories(${ROS_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${ROS_LIB} PUBLIC
  rclcpp
  rclcpp_components
  cv_bridge
  sensor_msgs
  spdlog
  fmt
)

rclcpp_components_register_node(
  ${ROS_LIB}
  PLUGIN "ImageFilteringNode"
  EXECUTABLE ${PROJECT_NAME}_node
)



# Export BOTH targets
ament_export_targets(export_${LIB} HAS_LIBRARY_TARGET)
ament_export_targets(export_${ROS_LIB} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(OpenCV spdlog fmt)
ament_export_dependencies(rclcpp rclcpp_components cv_bridge sensor_msgs)

install(TARGETS ${LIB}
  EXPORT export_${LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${ROS_LIB}
  EXPORT export_${ROS_LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
