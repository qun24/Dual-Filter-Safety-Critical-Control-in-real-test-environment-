cmake_minimum_required(VERSION 3.8)
project(lidar_odom_from_scratch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(Boost 1.74 REQUIRED COMPONENTS filesystem)
find_package(Boost 1.71 REQUIRED COMPONENTS filesystem)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(lidar_odom_from_scratch_node src/lidar_odom_from_scratch_node.cpp)

ament_target_dependencies(lidar_odom_from_scratch_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  laser_geometry
  pcl_ros
  tf2
  tf2_ros
  tf2_geometry_msgs)

target_include_directories(lidar_odom_from_scratch_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GSL_INCLUDE_DIRS})

target_link_libraries(lidar_odom_from_scratch_node
  csm
  gsl
  gslcblas)

target_compile_features(lidar_odom_from_scratch_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS lidar_odom_from_scratch_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)


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
