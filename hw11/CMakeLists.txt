cmake_minimum_required(VERSION 2.8.3)
project(hw11)

set(CMAKE_BUILD_TYPE "Debug")
#set C++ version
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hw11
#  CATKIN_DEPENDS roscpp rospy std_msgs
DEPENDS EIGEN3
)

#inlude myself library
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/include)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(kf_test 
src/kf_test.cpp
src/kalman.cpp)
target_link_libraries(kf_test ${catkin_LIBRARIES})
