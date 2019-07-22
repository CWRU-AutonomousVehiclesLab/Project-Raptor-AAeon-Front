cmake_minimum_required(VERSION 2.8.3)
project(laneFollower)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

catkin_package(
  DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(laneMarker src/laneMarker.cpp)


target_link_libraries(
 laneMarker 
 ${OpenCV_LIBS}
 ${catkin_LIBRARIES}
)
