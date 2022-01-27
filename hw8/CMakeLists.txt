cmake_minimum_required(VERSION 2.8.3)
project(hw8)

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
)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include 
#  LIBRARIES week_11
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
DEPENDS EIGEN3 
)

#inlude myself library
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/include)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)


add_executable(cere_solver_tutorial src/cere_solver_tutorial.cpp)
target_link_libraries(cere_solver_tutorial ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(cere_solver_tutorial2 src/cere_solver_tutorial2.cpp)
target_link_libraries(cere_solver_tutorial2 ${catkin_LIBRARIES} ${CERES_LIBRARIES})

