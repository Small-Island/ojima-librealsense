cmake_minimum_required(VERSION 3.1.0)

include_directories(/home/ojima/librealsense/include /home/ojima/librealsense/src /home/ojima/librealsense/third_party/realsense-file)

add_library(rs2 SHARED /home/ojima/librealsense/src/rs.cpp /home/ojima/librealsense/src/core/roi.h)

add_executable(rs-tracking-and-depth ../rs-tracking-and-depth.cpp)
