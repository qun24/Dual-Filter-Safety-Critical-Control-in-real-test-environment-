cmake_minimum_required(VERSION 3.8)
project(pointcloud_handle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

# OpenMP support
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS} # 添加这一行
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

# Declare C++ executables
add_executable(obs_pcl_circle src/obs_pcl_circle.cpp)
ament_target_dependencies(obs_pcl_circle
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  PCL
  pcl_ros
)

target_link_libraries(obs_pcl_circle
  ${rclcpp_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(obs_circle_rviz src/obs_circle_rviz.cpp)
ament_target_dependencies(obs_circle_rviz
  rclcpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  PCL
)

target_link_libraries(obs_circle_rviz
  ${rclcpp_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(obs_pcl_filter src/obs_pcl_filter.cpp)
ament_target_dependencies(obs_pcl_filter
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_ros
  PCL
)

target_link_libraries(obs_pcl_filter
  ${PCL_LIBRARIES}
  ${CGAL_LIBRARIES}
)


install(TARGETS
  obs_pcl_circle
  obs_circle_rviz
  obs_pcl_filter
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
