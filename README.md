# Baxter Collaboration [![Build Status](https://img.shields.io/travis/ScazLab/baxter_collaboration/master.svg?label=Build Status)](https://travis-ci.org/ScazLab/baxter_collaboration) [![Issues](https://img.shields.io/github/issues/ScazLab/baxter_collaboration.svg?label=Issues)](https://github.com/ScazLab/baxter_collaboration/issues)


Yet another repository for the Baxter collaboration task

## Prerequisites

 * `aruco_ros`: you should use [this version of `aruco_ros`](https://github.com/ScazLab/aruco_ros) which is a fork of (https://github.com/pal-robotics/aruco_ros)
 * `nlopt`: it should be installed by from the Ubuntu repositories: `sudo apt-get install libnlopt-dev `
 * `trac_ik` : for the time being, you should use [this version of `trac_ik`](https://bitbucket.org/alecive/trac_ik). It is a fork of [the original `trac_ik`](https://bitbucket.org/traclabs/trac_ik).
 * `svox_tts` : it's a SVOX-PICO based wrapper for text-to-speech. It's not necessary, but recommended. Available [here](https://github.com/ScazLab/svox_tts).

## Compilation & Testing

We use the new Catkin Command Line Tools `catkin_tools`, a Python package that provides command line tools for working with the catkin meta-buildsystem and catkin workspaces. This package was announced in March 2015 and is still in beta, but we didn't experience any problem with it. The following instructions apply to this new package, even though the repository can be used and compile with the old `catkin_make` without issues.

 1. Compile the repo: `catkin build baxter_collaboration`
 2. Compile tests and run them (to be done after compiling the repo with command #1): `catkin build baxter_collaboration --catkin-make-args run_tests`
 3. Check the results of step #2: `catkin_test_results build/baxter_collaboration/`

## Execution

 1. Be sure that the system you're running the code has access to the Baxter robot. This is usually done by running the `baxter.sh` script that should be provided in your Baxter installation. See [here](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter#Source_ROS_Environment_Setup_Script) for more info.
 2. On one terminal, launch the `ARuco` software: `roslaunch baxter_collaboration baxter_marker_publisher.launch`
 3. On another terminal, launch the Baxter Collaboration software, e.g. `roslaunch baxter_collaboration flatpack_furniture.launch` (these two launch files should be in the same launch file, but for development purposes it is much better to separate development code and production-ready code)
 4. Request actions to either one of the two arms by using the proper service (`/action_provider/service_left` for left arm, `/action_provider/service_right` for right arm). Here are some examples to make the demo work from terminal:
  * `rosservice call /action_provider/service_right "{action: 'hand_over', object: 17}"`
  * `rosservice call /action_provider/service_left "{action: 'get', object: 17}"`
 5. Request 3D points to the cartesian controller server by using the proper topic (`/baxter_controller/limb/left/go_to_pose` for left arm, `/baxter_controller/limb/left/go_to_pose` for right arm). Here is one example: `rostopic pub /baxter_controller/limb/left/go_to_pose baxter_collaboration/GoToPose "{pose_stamp: {header:{seq: 0, stamp: {secs: 0.0, nsecs: 0.0}}, pose:{position:{ x: 0.5, y: 0.5, z: 0.5}, orientation:{ x: -100, y: -100, z: -100, w: -100}}}, ctrl_mode: 0}" --once`

### Supported actions

 * `home` (both arms): moves the arm to a specific joint configuration (i.e. it does not use IK).
 * `release` (both arms): opens the gripper (or releases the vacuum gripper).
 * `hand_over` (both arms): performs an handover from the left to the right hand. The left arm picks an object up at a specific orientation (for now it works only with the central frame, ID number `24`), and passes it to the right arm, which then holds it until further notice.
 * `get` (left arm): the robot gets an object at a random orientation, and moves it on the table (without releasing it).
 * `pass` (left arm): if the robot has an object in its vacuum gripper, it moves it toward the human partner and waits for force interaction in order to release the gripper. The force filter we applied is a low-pass filter, so in order to trigger the `release` action, the human is suggested to apply an high-frequency, spike-y interaction.
 * `hold` (right arm): the robot moves into a configuration in which it is optimal to hold an object as a supportive action for the human partner. After that, it waits for force interaction to close the gripper, and again to open the gripper when the human has finished. See above for a description of the proper force interaction that should be applied to the arm in order to trigger the behaviors.

### Misc

 * To kill an action from the terminal, you can simulate a button press on the arm's cuff: `rostopic pub --once /robot/digital_io/left_lower_button/state baxter_core_msgs/DigitalIOState "{state: 1, isInputOnly: true}"`.
 * You can also kill an action from the web interface, by pressing the ERROR button. It writes to the same topic and achieves the same behavior.
 * To go **robot-less**, simply call the `action_provider` with the argument `--no_robot`: `rosrun baxter_collaboration action_provider --no_robot`. In this mode, only the service to request actions is enabled. It will always return with a 2s delay and it will always succeed.
 * To be robot-less, add `--no_robot` to the `args` in the `baxter_collaboration.launch` file.
