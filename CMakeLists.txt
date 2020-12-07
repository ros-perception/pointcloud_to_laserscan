cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_to_laserscan)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-Wall -Wno-psabi -Wextra)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  message_filters
  nodelet
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laserscan_to_pointcloud pointcloud_to_laserscan
  CATKIN_DEPENDS laser_geometry message_filters nodelet roscpp sensor_msgs tf2 tf2_ros tf2_sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(laserscan_to_pointcloud src/laserscan_to_pointcloud_nodelet.cpp)
target_link_libraries(laserscan_to_pointcloud ${catkin_LIBRARIES})

add_executable(laserscan_to_pointcloud_node src/laserscan_to_pointcloud_node.cpp)
target_link_libraries(laserscan_to_pointcloud_node laserscan_to_pointcloud ${catkin_LIBRARIES})

add_library(pointcloud_to_laserscan src/pointcloud_to_laserscan_nodelet.cpp)
target_link_libraries(pointcloud_to_laserscan ${catkin_LIBRARIES})

add_executable(pointcloud_to_laserscan_node src/pointcloud_to_laserscan_node.cpp)
target_link_libraries(pointcloud_to_laserscan_node pointcloud_to_laserscan ${catkin_LIBRARIES})

install(TARGETS
  laserscan_to_pointcloud
  laserscan_to_pointcloud_node
  pointcloud_to_laserscan
  pointcloud_to_laserscan_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
