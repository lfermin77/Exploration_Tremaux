cmake_minimum_required(VERSION 2.8.3)
project(exp_tremaux)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  cv_bridge
  image_transport
)





catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dude_ros
#  CATKIN_DEPENDS tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include  
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")




######## exploration
add_executable(exp_tremaux src/exp_tremaux.cpp  include/graph.cpp )

target_link_libraries(exp_tremaux
   ${catkin_LIBRARIES}
 )


add_dependencies(exp_tremaux ${catkin_LIBRARIES})







