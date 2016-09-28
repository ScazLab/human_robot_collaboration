# Copyright: (C) 2016 Social Robotics Lab - Yale University
# Authors: Álvaro Castro González, Olivier Mangin, Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.3)
project(baxterCollaboration)

# find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
# find_package(OpenCV REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

## Declare a C++ library
add_library(robot_interface include/robot_interface/utils.h
                            include/robot_interface/ros_thread.h
                            include/robot_interface/robot_interface.h
                            include/robot_interface/gripper.h
                            include/robot_interface/arm_ctrl.h
                            src/robot_interface/utils.cpp
                            src/robot_interface/ros_thread.cpp
                            src/robot_interface/baxter_trac_ik.cpp
                            src/robot_interface/robot_interface.cpp
                            src/robot_interface/gripper.cpp
                            src/robot_interface/arm_ctrl.cpp)

add_library(robot_perception    include/robot_perception/aruco_client.h
                                src/robot_perception/aruco_client.cpp)

add_library(flatpack_furniture  include/flatpack_furniture/artag_ctrl.h
                                include/flatpack_furniture/hold_ctrl.h
                                src/flatpack_furniture/artag_ctrl.cpp
                                src/flatpack_furniture/hold_ctrl.cpp)

add_library(tower_building  include/tower_building/cube_picker.h
                            src/tower_building/cube_picker.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(robot_interface        baxter_collaboration_generate_messages_cpp
                                        ${catkin_EXPORTED_TARGETS})

add_dependencies(robot_perception       baxter_collaboration_generate_messages_cpp
                                        ${catkin_EXPORTED_TARGETS})

add_dependencies(flatpack_furniture     baxter_collaboration_generate_messages_cpp
                                        robot_interface
                                        robot_perception
                                        ${catkin_EXPORTED_TARGETS})

add_dependencies(tower_building     baxter_collaboration_generate_messages_cpp
                                    robot_interface
                                    robot_perception
                                    flatpack_furniture
                                    ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(robot_interface       ${OpenCV_LIBS}
                                            ${catkin_LIBRARIES})

target_link_libraries(robot_perception      ${OpenCV_LIBS}
                                            ${catkin_LIBRARIES})

target_link_libraries(flatpack_furniture    robot_interface
                                            robot_perception
                                            ${catkin_LIBRARIES})

target_link_libraries(tower_building    robot_interface
                                        flatpack_furniture
                                        ${catkin_LIBRARIES})

## Mark libraries for installation
install (TARGETS robot_interface robot_perception flatpack_furniture tower_building
         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/robot_interface
                  ${PROJECT_SOURCE_DIR}/include/flatpack_furniture
                  ${PROJECT_SOURCE_DIR}/include/tower_building
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)
