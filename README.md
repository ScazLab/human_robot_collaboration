# Baxter Collaboration [![Build Status](https://travis-ci.org/ScazLab/baxter_collaboration.svg?branch=master)](https://travis-ci.org/ScazLab/baxter_collaboration) [![Issues](https://img.shields.io/github/issues/ScazLab/baxter_collaboration.svg?label=Issues)](https://github.com/ScazLab/baxter_collaboration/issues) [![Codacy Badge](https://api.codacy.com/project/badge/Grade/89218b1bb7b84e6e821d689fbd5129a8)](https://www.codacy.com/app/Baxter-collaboration/baxter_collaboration?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=ScazLab/baxter_collaboration&amp;utm_campaign=Badge_Grade)

Yet another repository for the Baxter collaboration task.

![setup](https://cloud.githubusercontent.com/assets/4378663/22127940/39931bb2-de6c-11e6-864e-8c9a3da48673.jpg)

If you are using this software and or one of its components, we warmly recommend you to cite the following paper:

  > [Roncone2017] Roncone Alessandro, Mangin Olivier, Scassellati Brian **Transparent Role Assignment and Task Allocation in Human Robot Collaboration** *IEEE International Conference on Robotics and Automation (ICRA 2017)*, Singapore. [[PDF]](http://alecive.github.io/papers/[Roncone%20et%20al.%202017]%20Transparent%20Role%20Assignment%20and%20Task%20Allocation%20in%20Human%20Robot%20Collaboration.pdf) [[BIB]](http://alecive.github.io/papers/[Roncone%20et%20al.%202017]%20Transparent%20Role%20Assignment%20and%20Task%20Allocation%20in%20Human%20Robot%20Collaboration.bib)

## Prerequisites

 * `aruco_ros`: you should use [this version of `aruco_ros`](https://github.com/ScazLab/aruco_ros) which is a fork of (https://github.com/pal-robotics/aruco_ros)
 * `nlopt`: it should be installed by from the Ubuntu repositories: `sudo apt-get install libnlopt-dev `
 * `trac_ik` : for the time being, you should use [this version of `trac_ik`](https://bitbucket.org/alecive/trac_ik). It is a fork of [the original `trac_ik`](https://bitbucket.org/traclabs/trac_ik).
 * `svox_tts` : it's a SVOX-PICO based wrapper for text-to-speech. It's not necessary, but recommended. Available [here](https://github.com/ScazLab/svox_tts).
 * `baxter_description`: it is used for testing the `robot_interface` library. It can be downloaded from the `baxter_common` repository, available [here](https://github.com/RethinkRobotics/baxter_common).

## Compilation & Testing

We use the new Catkin Command Line Tools `catkin_tools`, a Python package that provides command line tools for working with the catkin meta-buildsystem and catkin workspaces. This package was announced in March 2015 and is still in beta, but we didn't experience any problem with it. The following instructions apply to this new package, even though the repository can be used and compile with the old `catkin_make` without issues.

 1. Compile the repo: `catkin build baxter_collaboration`
 2. Compile tests and run them (to be done after compiling the repo with command #1): `catkin build baxter_collaboration --catkin-make-args run_tests`
 3. Check the results of step #2: `catkin build baxter_collaboration_lib --catkin-make-args run_tests && catkin_test_results build/baxter_collaboration_lib`

## Execution

### Initial steps (mainly for Scazlab students)

 0. Turn on the robot. Wait for the robot to finish its start-up phase.
 1. Be sure that the system you're running the code has access to the Baxter robot. This is usually done by running the `baxter.sh` script that should be provided in your Baxter installation. See [here](http://sdk.rethinkrobotics.com/wiki/Hello_Baxter#Source_ROS_Environment_Setup_Script) for more info. **@ScazLab students** → for what concerns the Baxter robot on the ScazLab, this means that every time you have to run some ROS software to be used on the robot you should open a new terminal, and do the following: ` cd ros_devel_ws && ./baxter.sh `. A change in the terminal prompt should acknowledge that you now have access to `baxter.local`. __Please be aware of this issue when you operate the robot__.
 2. Untuck the robot. **@ScazLab students** → we have an alias for this, so you just have to type `untuck`

This repository currently allows for two modes of operation:

 1. A. **Cartesian Controller server** → It allows for controlling each of the arms in operational space.
 2. B. **High-level actions** → It enables some high-level actions to be asked to the robot, such has `hold` or `pick object`.

These two modes can be enabled concurrently, but this feature is disabled by default: in order to be able to communicate with the robot both in the high-level interface and the low-level controller, you need to create your own `action_provider`. See the `src` folder for more information on that.

### Mode A. Cartesian Controller Server

In this mode, the user can ask the robot to go to a specific _3D Position_ or _6D Pose_ (position + orientation), and the robot will simply go there (if physically possible). To guarantee safety, the robot _still_ has the standard safety systems enabled by default. More advanced uses are allowed, but not exposed to the user: if you want to tinker with advanced features, we recommend to specialize the [`RobotInterface` class](https://github.com/ScazLab/baxter_collaboration/blob/master/baxter_collaboration_lib/include/robot_interface/robot_interface.h).

In order to use the Cartesian Controller Server, you have to launch it with:

```
roslaunch baxter_collaboration baxter_controller.launch
```

This should create two topics the user can request operational space configurations to. They are `/baxter_controller/left/go_to_pose` for left arm, and `/baxter_controller/left/go_to_pose` for right arm. In the following, there are some examples on how to require them from terminal (e.g. for the left arm):

 * _[6D Pose]_ : `rostopic pub /baxter_controller/left/go_to_pose baxter_collaboration_msgs/GoToPose "{pose_stamp: {pose:{position:{ x: 0.55, y: 0.55, z: 0.2}, orientation:{ x: 0, y: 1, z: 0, w: 0}}}, ctrl_mode: 0}" --once`
 * _[3D Position]_ : `rostopic pub /baxter_controller/left/go_to_pose baxter_collaboration_msgs/GoToPose "{pose_stamp: {pose:{position:{ x: 0.55, y: 0.55, z: 0.2}, orientation:{ x: -100, y: -100, z: -100, w: -100}}}, ctrl_mode: 0}" --once`. This differs from the previous case since now every value of the orientation quaternion is set to -100. This is to communicate the Cartesian Controller to reach the desired position _while maintaining the current orientation_.

Obviously, these same messages can be sent directly _within_ your code. Please take a look at the [`GoToPose.msg` file](https://github.com/ScazLab/baxter_collaboration/blob/master/baxter_collaboration_msgs/msg/GoToPose.msg) for further info.

### Mode B. High-Level Actions

We implemented a low-level, state-less controller able to operate each of the arms independently (and communicate with the other one if needed). A library of high-level predefined actions (in the form of ROS services) is available for the user to choose from; such actions range from the simple, single arm `pick object` to the more complex `hold object` (which requires a physical collaboration with the human partner) or `hand over` (which demands a bi-manual interaction between the two arms).

To enable this mode, run this in two separate terminals:

 1. `roslaunch baxter_collaboration baxter_marker_publisher.launch`
 2. `roslaunch baxter_collaboration flatpack_furniture.launch` or `roslaunch baxter_collaboration tower_building.launch`. These are two predefined launch files that we use for two different experiments we ran in the lab. If you would like to create and use your own high-level actions, we suggest you to specialize the [`ArmCtrl` class](https://github.com/ScazLab/baxter_collaboration/blob/master/baxter_collaboration_lib/include/robot_interface/arm_ctrl.h). See the [`flatpack_furniture library`](https://github.com/ScazLab/baxter_collaboration/tree/master/baxter_collaboration/lib/include/flatpack_furniture) for inspiration on how to do it.

Now, the user should be able to request actions to either one of the two arms by using the proper service (`/action_provider/service_left` for left arm, `/action_provider/service_right` for right arm). Here are some examples to make the demo work from terminal:
  * `rosservice call /action_provider/service_right "{action: 'hold'}"`
  * `rosservice call /action_provider/service_left "{action: 'get', objects: [17]}"`

Similarly to Mode A, these same services can be requested directly _within_ your code. Please take a look at the [`DoAction.srv` file](https://github.com/ScazLab/baxter_collaboration/blob/master/baxter_collaboration_msgs/srv/DoAction.srv) for further info.

#### Non-exhaustive list of supported actions

 * `list_actions` (both arms): it returns a list of the available actions for the specific arm.
 * `home` (both arms): moves the arm to a specific joint configuration (i.e. it does not use IK).
 * `release` (both arms): opens the gripper (or releases the vacuum gripper).
 * `hand_over` (both arms): performs an handover from the left to the right hand. The left arm picks an object up at a specific orientation (for now it works only with the central frame, ID number `24`), and passes it to the right arm, which then holds it until further notice.
 * `get` (left arm): the robot gets an object at a random orientation, and moves it on the table (without releasing it).
 * `pass` (left arm): if the robot has an object in its vacuum gripper, it moves it toward the human partner and waits for force interaction in order to release the gripper. The force filter we applied is a low-pass filter, so in order to trigger the `release` action, the human is suggested to apply an high-frequency, spike-y interaction.
 * `hold` (right arm): the robot moves into a configuration in which it is optimal to hold an object as a supportive action for the human partner. After that, it waits for force interaction to close the gripper, and again to open the gripper when the human has finished. See above for a description of the proper force interaction that should be applied to the arm in order to trigger the behaviors.

### Misc

 * To kill an action from the terminal, you can simulate a button press on the arm's cuff: `rostopic pub --once /robot/digital_io/left_lower_button/state baxter_core_msgs/DigitalIOState "{state: 1, isInputOnly: true}"`.
 * You can also kill an action from the web interface, by pressing the ERROR button. It writes to the same topic and achieves the same behavior.
 * To go **robot-less** (that is try to execute the software without the robot, for testing purposes), you can choose one of the following options:
  * Call the `action_provider` with the argument `--no_robot`, e.g. `rosrun baxter_collaboration baxter_controller --no_robot`. In this mode, only the service to request actions is enabled. It will always return with a 2s delay and it will always succeed.
  * Change the `useRobot` flag to `false` in the `launch` file.
  * Launch the `launch` file with the argument `useRobot:=false`, e.g. `roslaunch baxter_collaboration baxter_controller.launch useRobot:=false`


# Experiments

## ICRA 2017

To reproduce the experiment from the following paper on the Baxter research robot, we provide the script `baxter_collaboration/scripts/icra_experiment`.

    [Roncone2017] Roncone Alessandro, Mangin Olivier, Scassellati Brian **Transparent Role Assignment and Task Allocation in Human Robot Collaboration** *IEEE International Conference on Robotics and Automation (ICRA 2017)*, Singapore.

It needs to be given the path to an offline computed policy, as possible with [github.com/ScazLab/task-models](https://github.com/ScazLab/task-models).
