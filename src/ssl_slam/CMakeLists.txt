cmake_minimum_required(VERSION 3.8)
project(ssl_slam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer thread)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)

include_directories(include)
ament_export_include_directories(include)

set(SSL_SLAM2_DEPS
  rclcpp
  rclcpp_components
  rclpy
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  eigen3_cmake_module
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  std_msgs
  pcl_msgs
  pcl_conversions

)


add_executable(ssl_slam_laser_processing_node src/laser_processing_node.cpp src/laser_processing_class.cpp src/lidar.cpp)

target_link_libraries(
ssl_slam_laser_processing_node
${EIGEN3_LIBRARIES}
${PCL_LIBRARIES}
${CERES_LIBRARIES}
)
ament_target_dependencies(
	ssl_slam_laser_processing_node
	${SSL_SLAM2_DEPS}
)

add_executable(ssl_slam_odom_estimation_mapping_node src/odom_estimation_mapping_node.cpp src/lidar.cpp src/odom_estimation_mapping_class.cpp src/lidar_optimization.cpp)
target_link_libraries(
ssl_slam_odom_estimation_mapping_node
${EIGEN3_LIBRARIES}
${PCL_LIBRARIES}
${CERES_LIBRARIES})

ament_target_dependencies(
	ssl_slam_odom_estimation_mapping_node
	${SSL_SLAM2_DEPS}
)

add_executable(ssl_slam_map_optimization_node src/map_optimization_class.cpp src/map_optimization_node.cpp src/lidar.cpp src/lidar_optimization.cpp)
target_link_libraries(
ssl_slam_map_optimization_node
${EIGEN3_LIBRARIES}
${PCL_LIBRARIES}
${CERES_LIBRARIES}
gtsam
)

ament_target_dependencies(
	ssl_slam_map_optimization_node
	${SSL_SLAM2_DEPS}
)

install(TARGETS
	ssl_slam_laser_processing_node
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
	ssl_slam_odom_estimation_mapping_node
	DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
	ssl_slam_map_optimization_node
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
 launch
 rviz
 DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
