cmake_minimum_required(VERSION 2.8.3)
project(rovy_base_controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp tf2_geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(rovy_base_controller src/rovy_base_controller.cpp)
target_link_libraries(rovy_base_controller ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-Wformat -Wformat-security -Wall -lmotor-control ${CMAKE_CXX_FLAGS}")