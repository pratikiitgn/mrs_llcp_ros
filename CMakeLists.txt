cmake_minimum_required(VERSION 3.1.2)
project(mrs_llcp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DLLCP_LITTLE_ENDIAN)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  nodelet
  mrs_msgs
  )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES LlcpRos
  CATKIN_DEPENDS roscpp mrs_msgs
  )

include_directories(
  include
  mrs_llcp/include
  ${catkin_INCLUDE_DIRS}
  )

add_library(llcp
  mrs_llcp/src/llcp.c
)

add_library(LlcpRos src/llcp_ros.cpp src/serial_port.cpp)
target_link_libraries(LlcpRos
  llcp
  ${catkin_LIBRARIES}
  )

