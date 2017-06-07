# Copyright: (C) 2016 Social Robotics Lab - Yale University
# Author: Alessandro Roncone <alessandro.roncone@yale.edu>
# CopyPolicy: Released under the terms of the LGPLv2.1 .

cmake_minimum_required(VERSION 2.8.3)
project(baxter_collaboration_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             aruco
             aruco_ros
             baxter_collaboration_msgs
             baxter_core_msgs
             cv_bridge
             image_transport
             rosconsole
             roscpp
             tf # Even if I'm not explicitly using it, adding tf is the only way
                # I found to avoid the undefined reference error to ros::init.
                # See http://goo.gl/GlbKgM
             trac_ik_lib
             )

find_package(OpenCV 2.4 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_utils robot_interface robot_perception
  DEPENDS trac_ik_lib aruco_ros aruco
)

###########
## Build ##
###########

ADD_DEFINITIONS ( -Wall -Wextra -Wno-unused-parameter -Werror)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
                    ${OpenCV_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# For the record, robot_utils is decoupled from robot_interface
# so that we can link robot_perception against robot_utils only
# without the need to link against the full robot_interface lib,
# which would be an unnecessary overhead.
add_library(robot_utils     include/robot_utils/utils.h
                            include/robot_utils/baxter_trac_ik.h
                            include/robot_utils/ros_thread.h
                            include/robot_utils/ros_thread_obj.h
                            include/robot_utils/ros_thread_image.h
                            src/robot_utils/utils.cpp
                            src/robot_utils/baxter_trac_ik.cpp
                            src/robot_utils/ros_thread.cpp
                            src/robot_utils/ros_thread_obj.cpp
                            src/robot_utils/ros_thread_image.cpp)

add_library(robot_interface include/robot_interface/robot_interface.h
                            include/robot_interface/gripper.h
                            include/robot_interface/arm_ctrl.h
                            src/robot_interface/robot_interface.cpp
                            src/robot_interface/gripper.cpp
                            src/robot_interface/arm_ctrl.cpp)

add_library(robot_perception    include/robot_perception/hsv_detection.h
                                include/robot_perception/aruco_client.h
                                include/robot_perception/cartesian_estimator.h
                                include/robot_perception/cartesian_estimator_hsv.h
                                include/robot_perception/cartesian_estimator_client.h
                                src/robot_perception/hsv_detection.cpp
                                src/robot_perception/aruco_client.cpp
                                src/robot_perception/cartesian_estimator.cpp
                                src/robot_perception/cartesian_estimator_hsv.cpp
                                src/robot_perception/cartesian_estimator_client.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(robot_utils        ${catkin_EXPORTED_TARGETS})

add_dependencies(robot_interface    robot_utils
                                    ${catkin_EXPORTED_TARGETS})

add_dependencies(robot_perception   robot_utils
                                    ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(robot_utils           ${catkin_LIBRARIES})

target_link_libraries(robot_interface       robot_utils
                                            ${catkin_LIBRARIES})

target_link_libraries(robot_perception      robot_utils
                                            ${OpenCV_LIBS}
                                            ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark libraries for installation
install (TARGETS robot_utils robot_interface robot_perception
         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY   ${PROJECT_SOURCE_DIR}/include/robot_utils
                    ${PROJECT_SOURCE_DIR}/include/robot_interface
                    ${PROJECT_SOURCE_DIR}/include/robot_perception
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest            REQUIRED)
  find_package(baxter_description REQUIRED)

  ## Robot util tests
  catkin_add_gtest(test_utils_lib test/test_utils_lib.cpp)
  target_link_libraries(test_utils_lib robot_utils)

  ## Robot interface tests
  add_rostest_gtest(test_robot_interface test/test_robot_interface.test
                                         test/test_robot_interface.cpp)
  target_link_libraries(test_robot_interface  robot_interface ${catkin_LIBRARIES})

  ## Robot gripper tests
  add_rostest_gtest(test_gripper test/test_gripper.test
                                         test/test_gripper.cpp)
  target_link_libraries(test_gripper robot_interface ${catkin_LIBRARIES})

  ## ROS thread obj tests
  catkin_add_gtest(test_ros_thread_obj test/test_ros_thread_obj.cpp)
  target_link_libraries(test_ros_thread_obj robot_utils)

  ## Robot ROS thread image test
  add_rostest_gtest(test_ros_thread_image test/test_ros_thread_image.test
                                          test/test_ros_thread_image.cpp)
  target_link_libraries(test_ros_thread_image robot_utils)

  ## Robot Arm Test
  add_rostest_gtest(test_arm_ctrl test/test_arm_ctrl.test
  test/test_arm_ctrl.cpp) 
  target_link_libraries(test_arm_ctrl robot_interface)

endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
