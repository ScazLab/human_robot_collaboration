cmake_minimum_required(VERSION 2.8.3)
project(baxter_collaboration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             roscpp
             rospy
             rosconsole
             baxter_collaboration_msgs
             baxter_core_msgs
             tf # Even if I'm not explicitly using it, adding tf is the only way
                # I found to avoid the undefined reference error to ros::init.
                # See http://goo.gl/GlbKgM
             cv_bridge
             image_transport
             trac_ik_lib
             eigen_conversions
             aruco
             aruco_ros
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
    INCLUDE_DIRS lib/include
    LIBRARIES robot_utils robot_interface robot_perception
    CATKIN_DEPENDS trac_ik_lib
    # DEPENDS system_lib
)

###########
## Build ##
###########

ADD_DEFINITIONS ( -Wall -Wextra -Wno-unused-parameter -Werror)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(lib/include
                    ${OpenCV_INCLUDE_DIRS}
                    ${eigen_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_SOURCE_DIR})

## Add C++ libraries
add_subdirectory(lib)

## Declare a C++ executable
add_executable(baxter_controller        src/baxter_controller/baxter_controller.cpp)
add_executable(flatpack_action_provider src/flatpack_furniture/artag_ctrl_impl.h
                                        src/flatpack_furniture/artag_ctrl_impl.cpp
                                        src/flatpack_furniture/hold_ctrl_impl.h
                                        src/flatpack_furniture/hold_ctrl_impl.cpp
                                        src/flatpack_furniture/action_provider.cpp)
add_executable(tower_action_provider    src/tower_building/cube_picker.h
                                        src/tower_building/cube_picker.cpp
                                        src/tower_building/action_provider.cpp)
add_executable(hsv_detector             src/modular_furniture/hsv_detector.cpp)
add_executable(modular_action_provider  src/modular_furniture/tool_picker.h
                                        src/modular_furniture/tool_picker.cpp
                                        src/modular_furniture/part_picker.h
                                        src/modular_furniture/part_picker.cpp
                                        src/modular_furniture/action_provider.cpp)
add_executable(baxter_display           src/baxter_display.cpp)

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
add_dependencies(baxter_display           ${${PROJECT_NAME}_EXPORTED_TARGETS}
                                          ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(baxter_controller         robot_interface
                                                ${catkin_LIBRARIES} )
target_link_libraries(flatpack_action_provider  robot_interface
                                                robot_perception
                                                flatpack_furniture
                                                ${catkin_LIBRARIES} )
target_link_libraries(tower_action_provider     robot_interface
                                                robot_perception
                                                ${catkin_LIBRARIES} )
target_link_libraries(hsv_detector              robot_perception
                                                ${catkin_LIBRARIES} )
target_link_libraries(modular_action_provider   robot_interface
                                                robot_perception
                                                flatpack_furniture
                                                ${catkin_LIBRARIES} )
target_link_libraries(baxter_display            robot_interface
                                                ${OpenCV_LIBS}
                                                ${catkin_LIBRARIES} )

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
# install(TARGETS baxter_collaboration flatpack_action_provider
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(utils-test test/test_utils_lib.cpp)

if(TARGET utils-test)
  target_link_libraries(utils-test robot_utils)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
