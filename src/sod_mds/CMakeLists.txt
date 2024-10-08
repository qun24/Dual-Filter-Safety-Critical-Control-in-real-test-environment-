cmake_minimum_required(VERSION 3.8)
project(sod_mds)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(sod_mbs_node 
      src/sod_mbs_node.cpp
      src/MBS.cpp)

ament_target_dependencies(sod_mbs_node
  rclcpp
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  pcl_ros
  pcl_conversions
  message_filters
  realsense2
)

target_include_directories(sod_mbs_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Find headers
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)


target_link_libraries(sod_mbs_node
  ${OpenCV_LIBRARIES}
)





target_compile_features(sod_mbs_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS sod_mbs_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
