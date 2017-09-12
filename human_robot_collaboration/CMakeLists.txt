# Copyright: (C) 2017 Social Robotics Lab - Yale University
# Author: Alessandro Roncone <alessandro.roncone@yale.edu>
# CopyPolicy: Released under the terms of the LGPLv2.1.

cmake_minimum_required(VERSION 2.8.3)
project(human_robot_collaboration)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             aruco
             human_robot_collaboration_lib
             baxter_core_msgs
             cv_bridge
             image_transport
             rosconsole
             roscpp
             rospy
             )

find_package(OpenCV 2.4 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

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
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS human_robot_collaboration_lib
)

###########
## Build ##
###########

ADD_DEFINITIONS ( -Wall -Wextra -Wno-unused-parameter -Werror)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS}
                           ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(baxter_controller        src/baxter_controller/baxter_controller.cpp)
add_executable(flatpack_action_provider src/flatpack_furniture/left_ctrl.h
                                        src/flatpack_furniture/right_ctrl.h
                                        src/flatpack_furniture/action_provider.cpp)
add_executable(tower_action_provider    src/tower_building/left_ctrl.h
                                        src/tower_building/action_provider.cpp)
add_executable(modular_action_provider  src/modular_furniture/right_ctrl.h
                                        src/modular_furniture/left_ctrl.h
                                        src/modular_furniture/action_provider.cpp)
add_executable(sp_pred_action_provider  src/speech_prediction/right_ctrl.h
                                        src/modular_furniture/left_ctrl.h
                                        src/speech_prediction/action_provider.cpp)
add_executable(baxter_display           src/baxter_display.cpp)
add_executable(hsv_detector             src/hsv_detector.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(baxter_controller        ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                          ${catkin_EXPORTED_TARGETS})
add_dependencies(flatpack_action_provider ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                          ${catkin_EXPORTED_TARGETS})
add_dependencies(tower_action_provider    ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                          ${catkin_EXPORTED_TARGETS})
add_dependencies(hsv_detector             ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                          ${catkin_EXPORTED_TARGETS})
add_dependencies(modular_action_provider  ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                          ${catkin_EXPORTED_TARGETS})
add_dependencies(sp_pred_action_provider  ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                          ${catkin_EXPORTED_TARGETS})
add_dependencies(baxter_display           ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                          ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(baxter_controller                         ${catkin_LIBRARIES} )
target_link_libraries(flatpack_action_provider                  ${catkin_LIBRARIES} )
target_link_libraries(tower_action_provider                     ${catkin_LIBRARIES} )
target_link_libraries(hsv_detector                              ${catkin_LIBRARIES} )
target_link_libraries(modular_action_provider                   ${catkin_LIBRARIES} )
target_link_libraries(sp_pred_action_provider                   ${catkin_LIBRARIES} )
target_link_libraries(baxter_display            ${OpenCV_LIBS}  ${catkin_LIBRARIES} )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
        scripts/remote_control_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS baxter_controller flatpack_action_provider tower_action_provider
                hsv_detector modular_action_provider baxter_display
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY lib/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
