# Copyright: (C) 2016 Social Robotics Lab - Yale University
# Author: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.3)
project(baxterCollaboration)

# find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
# find_package(OpenCV REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

## Declare the C++ libraries
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
                                src/robot_perception/hsv_detection.cpp
                                src/robot_perception/aruco_client.cpp
                                src/robot_perception/cartesian_estimator.cpp
                                src/robot_perception/cartesian_estimator_hsv.cpp)

add_library(flatpack_furniture  include/flatpack_furniture/artag_ctrl.h
                                include/flatpack_furniture/hold_ctrl.h
                                src/flatpack_furniture/artag_ctrl.cpp
                                src/flatpack_furniture/hold_ctrl.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(robot_utils        baxter_collaboration_generate_messages_cpp
                                    ${catkin_EXPORTED_TARGETS})

add_dependencies(robot_interface    robot_utils
                                    ${catkin_EXPORTED_TARGETS})

add_dependencies(robot_perception   robot_utils
                                    ${catkin_EXPORTED_TARGETS})

add_dependencies(flatpack_furniture robot_interface
                                    robot_perception
                                    ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(robot_utils           ${catkin_LIBRARIES})

target_link_libraries(robot_interface       robot_utils
                                            ${catkin_LIBRARIES})

target_link_libraries(robot_perception      robot_utils
                                            ${OpenCV_LIBS}
                                            ${catkin_LIBRARIES})

target_link_libraries(flatpack_furniture    robot_interface
                                            robot_perception
                                            ${catkin_LIBRARIES})

## Mark libraries for installation
install (TARGETS robot_utils robot_interface robot_perception
                 flatpack_furniture
         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY   ${PROJECT_SOURCE_DIR}/include/robot_utils
                    ${PROJECT_SOURCE_DIR}/include/robot_interface
                    ${PROJECT_SOURCE_DIR}/include/robot_perception
                    ${PROJECT_SOURCE_DIR}/include/flatpack_furniture
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)
