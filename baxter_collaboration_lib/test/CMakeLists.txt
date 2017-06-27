# Copyright: (C) 2017 Social Robotics Lab - Yale University
# Author: Alessandro Roncone <alessandro.roncone@yale.edu>
# CopyPolicy: Released under the terms of the LGPLv2.1.

find_package(rostest            REQUIRED)
find_package(baxter_description REQUIRED)

## Robot util tests
catkin_add_gtest(test_utils_lib test_utils_lib.cpp)
target_link_libraries(test_utils_lib robot_utils)

## Robot interface tests
add_rostest_gtest(test_robot_interface test_robot_interface.test
                                       test_robot_interface.cpp)
target_link_libraries(test_robot_interface robot_interface)

## Robot gripper tests
add_rostest_gtest(test_gripper test_gripper.test
                               test_gripper.cpp)
target_link_libraries(test_gripper robot_interface)

## Gripper Keyboard tests the grippers directly on the robot
add_executable(gripper_keyboard     gripper_keyboard.cpp)
target_link_libraries(gripper_keyboard ${catkin_LIBRARIES} robot_interface)

## Robot ROS thread image test
add_rostest_gtest(test_ros_thread_image test_ros_thread_image.test
                                        test_ros_thread_image.cpp)
target_link_libraries(test_ros_thread_image robot_utils)

## Robot Arm Test
add_rostest_gtest(test_arm_ctrl test_arm_ctrl.test
                                test_arm_ctrl.cpp)
target_link_libraries(test_arm_ctrl robot_interface)
