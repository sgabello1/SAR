cmake_minimum_required(VERSION 2.8.3)
project(spatially_ar)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  genmsg 
  cv_bridge
  pcl_conversions
  pcl_ros 
)

find_package(PCL 1.7 REQUIRED)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(laser_calibration src/laser_calibration.cpp)
target_link_libraries(laser_calibration  ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(sar src/sar.cpp)
target_link_libraries(sar  ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(sar_ros src/sar_ros.cpp)
target_link_libraries(sar_ros  ${catkin_LIBRARIES} ${PCL_LIBRARIES})


