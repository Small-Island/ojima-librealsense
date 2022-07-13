# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesPointcloud)

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-pointcloud rs-pointcloud.cpp ../example.hpp)
    set_property(TARGET rs-pointcloud PROPERTY CXX_STANDARD 11)
    target_link_libraries(rs-pointcloud ${DEPENDENCIES})
    include_directories(../)
    set_target_properties (rs-pointcloud PROPERTIES FOLDER Examples)
    install(TARGETS rs-pointcloud RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

    add_executable(ojima-pointcloud ojima-pointcloud.cpp ../example.hpp)
    set_property(TARGET ojima-pointcloud PROPERTY CXX_STANDARD 11)
    target_link_libraries(ojima-pointcloud ${DEPENDENCIES})
    include_directories(../)
    set_target_properties (ojima-pointcloud PROPERTIES FOLDER Examples)
    install(TARGETS ojima-pointcloud RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

    add_executable(ojima-pointcloud-gui ojima-pointcloud-gui.cpp ../example.hpp)
    set_property(TARGET ojima-pointcloud-gui PROPERTY CXX_STANDARD 11)
    target_link_libraries(ojima-pointcloud-gui ${DEPENDENCIES})
    include_directories(../)
    set_target_properties (ojima-pointcloud-gui PROPERTIES FOLDER Examples)
    install(TARGETS ojima-pointcloud-gui RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

    add_executable(ojima-pointcloud_A1 ojima-pointcloud_A1.cpp ../example.hpp)
    set_property(TARGET ojima-pointcloud_A1 PROPERTY CXX_STANDARD 11)
    target_link_libraries(ojima-pointcloud_A1 ${DEPENDENCIES})
    include_directories(../)
    set_target_properties (ojima-pointcloud_A1 PROPERTIES FOLDER Examples)
    install(TARGETS ojima-pointcloud_A1 RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
