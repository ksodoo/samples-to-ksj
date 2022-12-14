# setting min version 
cmake_minimum_required(VERSION 3.1.0)

project(tutorial)

find_package(OpenCV REQUIRED)
set(DEPENDENCIES realsense2 ${OpenCV_LIBS})

message( STATUS "OPENCV VERSION ${OpenCV_VERSION}")

add_executable(mask mask.cpp cv-helpers.hpp)
set_property(TARGET mask PROPERTY CXX_STANDART 11)
target_link_libraries(mask ${DEPENDENCIES})