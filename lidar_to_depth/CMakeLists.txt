cmake_minimum_required(VERSION 3.0.2)
project(lidar_to_depth)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  cv_bridge
)

## Find OpenCV

find_package(OpenCV REQUIRED)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros cv_bridge
)
message(STATUS "OpenCV include directories: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBRARIES}")
## Build node
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  
)

add_executable(lidar_to_depth src/main.cpp)
target_link_libraries(lidar_to_depth
  ${catkin_LIBRARIES}  
  ${OpenCV_LIBRARIES}  
  
)