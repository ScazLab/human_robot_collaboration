# Baxter Collaboration

Yet another repository for the Baxter collaboration task

## Prerequisites

 * `nlopt`: it should be installed by from the Ubuntu repositories: `sudo apt-get install libnlopt-dev `
 * `trac_ik` : for the time being, you should use [this version of `trac_ik`](https://bitbucket.org/alecive/trac_ik). It is a fork of [the original `trac_ik`](https://bitbucket.org/traclabs/trac_ik).

## Installation

## Execution

 1. On one terminal, launch the `ARuco` software: `roslaunch baxter_collaboration baxter_marker_publisher.launch`
 2. On another terminal, launch the Baxter Collaboration software: `roslaunch baxter_collaboration baxter_collaboration.launch` (these two commands should be in the same launch file, but for development purposes it is much better to separate development code and production code)
 3. Request actions to either one of the two arms by using the proper service (`/action_provider/service_left` for left arm, `/action_provider/service_right` for right arm). Here are some examples to make the demo work from terminal:
  * `rosservice call /action_provider/service_right "{action: 'hand_over', object: 17}"`
  * `rosservice call /action_provider/service_left "{action: 'get', object: 17}"`

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
