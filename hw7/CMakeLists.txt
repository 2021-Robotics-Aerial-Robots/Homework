cmake_minimum_required(VERSION 2.8.3)
project(hw7)

set(CMAKE_BUILD_TYPE "Debug")

#set C++ version
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  tf
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include 
#  LIBRARIES week_11
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
DEPENDS EIGEN3
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(tf_cf1 src/tf_cf1.cpp)
target_link_libraries(tf_cf1 ${catkin_LIBRARIES} )

add_executable(tf_cf2 src/tf_cf2.cpp)
target_link_libraries(tf_cf2 ${catkin_LIBRARIES} )

add_executable(tf_cf3 src/tf_cf3.cpp)
target_link_libraries(tf_cf3 ${catkin_LIBRARIES} )

add_executable(tf_cf4 src/tf_cf4.cpp)
target_link_libraries(tf_cf4 ${catkin_LIBRARIES} )
