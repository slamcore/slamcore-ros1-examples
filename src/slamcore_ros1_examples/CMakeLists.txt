# ============================================================================ #
#
# SLAMcore Confidential
# ---------------------
#
# SLAMcore Limited All Rights Reserved. (C) Copyright 2021
#
# NOTICE:
#
# All information contained herein is, and remains the property of SLAMcore
# Limited and its suppliers, if any. The intellectual and technical concepts
# contained herein are proprietary to SLAMcore Limited and its suppliers and may
# be covered by patents in process, and are protected by trade secret or
# copyright law. Dissemination of this information or reproduction of this
# material is strictly forbidden unless prior written permission is obtained
# from SLAMcore Limited.
#
# ============================================================================ #

# ============================================================================ #
# CMake Initialisation
# ============================================================================ #
cmake_minimum_required(VERSION 3.5.1)
project(
  slamcore_ros1_examples
  VERSION 0.2.0
  LANGUAGES C CXX)

# ============================================================================ #
# CMake Scripts Location & Misc config
# ============================================================================ #

# ============================================================================ #
# Project dependencies
# ============================================================================ #

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             geometry_msgs
             nav_msgs
             kobuki_msgs
             sensor_msgs
             std_msgs
             tf
             tf2
             tf2_geometry_msgs)

# ============================================================================ #
# Messages, Services, Actions
# ============================================================================ #

# ============================================================================ #
# Libraries, Executables, catkin_package
# ============================================================================ #

catkin_package(CATKIN_DEPENDS roscpp geometry_msgs nav_msgs std_msgs
               sensor_msgs)

add_executable(kobuki_joystick src/kobuki_joystick.cpp)
target_include_directories(kobuki_joystick PRIVATE include
                                                   ${catkin_INCLUDE_DIRS})
target_link_libraries(kobuki_joystick ${catkin_LIBRARIES})

# ============================================================================ #
# Installation
# ============================================================================ #
install(DIRECTORY launch scripts rqt rviz
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
