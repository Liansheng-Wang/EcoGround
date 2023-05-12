cmake_minimum_required(VERSION 3.0.2)
project(eco_ground)

add_compile_options(-std=c++17)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
)

add_executable(lightcontrol src/LightControlNode.cpp)
target_link_libraries(lightcontrol ${catkin_LIBRARIES} )
