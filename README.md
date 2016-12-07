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

### Initial steps (mainly for Scazlab students)

 0. Turn on the robot. Wait for the robot to finish its start-up phase.
 1. Be sure that the system you're running the code has access to the Baxter robot. This is usually done by running the `baxter.sh` script that should be provided in your Baxter installation. See [here](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter#Source_ROS_Environment_Setup_Script) for more info. **@ScazLab students** → for what concerns the Baxter robot on the ScazLab, this means that every time you have to run some ROS software to be used on the robot you should open a new terminal, and do the following: ` cd ros_devel_ws && ./baxter.sh `. A change in the terminal prompt should acknowledge that you now have access to `baxter.local`. __Please be aware of this issue when you operate the robot__.
 2. Untuck the robot. **@ScazLab students** → we have an alias for this, so you just have to type `untuck`

This repository currently allows for two modes of operation:

 A. **Cartesian Controller server** → It allows for controlling each of the arms in operational space.
 B. **High-level actions** → It enables some high-level actions to be asked to the robot, such has `hold` or `pick object`.

These two modes can be enabled concurrently, but this feature is disabled by default: in order to be able to communicate with the robot both in the high-level interface and the low-level controller, you need to create your own `action_provider`. See the `src` folder for more information on that.

### Mode A. Cartesian Controller Server

In this mode, the user can ask the robot to go to a specific _3D Position_ or _6D Pose_ (position + orientation), and the robot will simply go there (if physically possible). To guarantee safety, the robot still has the standard safety systems enabled by default. More advanced uses are allowed, but not exposed to the user: if you want to tinker with advanced features, we recommend to specialize the [RobotInterface class](https://github.com/ScazLab/baxter_collaboration/blob/master/lib/include/robot_interface/robot_interface.h).

In order to use the Cartesian Controller Server, you have to launch it with:

```
roslaunch baxter_collaboration baxter_controller.launch
```

This should create two topics (`/baxter_controller/left/go_to_pose` for left arm, `/baxter_controller/left/go_to_pose` for right arm) the user can request operational space configurations to. In the following, there are some examples on how to require them from terminal (e.g. for the left arm):

 * _[6D Pose]_ : `rostopic pub /baxter_controller/left/go_to_pose baxter_collaboration/GoToPose "{pose_stamp: {pose:{position:{ x: 0.55, y: 0.55, z: 0.2}, orientation:{ x: 0, y: 1, z: 0, w: 0}}}, ctrl_mode: 0}" --once`
 * _[3D Position]_ : `rostopic pub /baxter_controller/left/go_to_pose baxter_collaboration/GoToPose "{pose_stamp: {pose:{position:{ x: 0.55, y: 0.55, z: 0.2}, orientation:{ x: -100, y: -100, z: -100, w: -100}}}, ctrl_mode: 0}" --once`. This differs from the previous case since now every value of the orientation quaternion is set to -100. This is to communicate the Cartesian Controller to reach the desired position _while maintaining the current orientation_.

Obviously, these same messages can be sent directly _within_ your code. Please take a look at the [GoToPose.msg file](https://github.com/ScazLab/baxter_collaboration/blob/master/msg/GoToPose.msg) for further info.



#### Supported actions

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
