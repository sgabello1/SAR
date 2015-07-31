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

add_executable(calibrate src/calibrate.cpp)
target_link_libraries(calibrate  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  opencv_nonfree  )

add_executable(perspective src/perspective.cpp)
target_link_libraries(perspective ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  opencv_nonfree  )

add_executable(calibration2 src/calibration2.cpp)
target_link_libraries(calibration2 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  opencv_nonfree  )

add_executable(spatially_ar src/spatially_ar.cpp)
target_link_libraries(spatially_ar  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  opencv_nonfree  )

add_executable(rectangleRecongnition src/rectangleRecongnition.cpp)
target_link_libraries(rectangleRecongnition  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  opencv_nonfree  )

add_executable(ye_sar src/ye_sar.cpp)
target_link_libraries(ye_sar  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  opencv_nonfree  )

add_executable(ye_sar2 src/ye_sar2.cpp)
target_link_libraries(ye_sar2  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  opencv_nonfree  )

add_executable(sar src/sar.cpp)
target_link_libraries(sar  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}  opencv_nonfree  )
