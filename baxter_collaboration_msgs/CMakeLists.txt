cmake_minimum_required(VERSION 2.8.3)
project(baxter_collaboration_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
             message_generation
             geometry_msgs)

## Generate messages in the 'msg' folder
add_message_files(FILES
                  ArmState.msg
                  GoToPose.msg
                  Object.msg
                  ObjectsArray.msg
)

## Generate services in the 'srv' folder
add_service_files(FILES
                  DoAction.srv
                  AskFeedback.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
                  geometry_msgs
)

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
#  INCLUDE_DIRS include
#  LIBRARIES baxter_collaboration_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
