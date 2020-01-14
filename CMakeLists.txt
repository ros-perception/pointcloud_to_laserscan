cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_laserscan)

find_package(ament_cmake REQUIRED)

find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(include)

add_library(laserscan_to_pointcloud SHARED
  src/laserscan_to_pointcloud_node.cpp)
target_compile_definitions(laserscan_to_pointcloud
  PRIVATE "POINTCLOUD_TO_LASERSCAN_BUILDING_DLL")
ament_target_dependencies(laserscan_to_pointcloud
  "laser_geometry"
  "message_filters"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_sensor_msgs"
)
rclcpp_components_register_node(laserscan_to_pointcloud
  PLUGIN "pointcloud_to_laserscan::LaserScanToPointCloudNode"
  EXECUTABLE laserscan_to_pointcloud_node)

add_library(pointcloud_to_laserscan SHARED
  src/pointcloud_to_laserscan_node.cpp)
target_compile_definitions(pointcloud_to_laserscan
  PRIVATE "POINTCLOUD_TO_LASERSCAN_BUILDING_DLL")
ament_target_dependencies(pointcloud_to_laserscan
  "laser_geometry"
  "message_filters"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "tf2_sensor_msgs"
)
rclcpp_components_register_node(pointcloud_to_laserscan
  PLUGIN "pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE pointcloud_to_laserscan_node)

add_executable(dummy_pointcloud_publisher
  src/dummy_pointcloud_publisher.cpp
)
ament_target_dependencies(dummy_pointcloud_publisher
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS
  laserscan_to_pointcloud
  pointcloud_to_laserscan
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(TARGETS
  dummy_pointcloud_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
