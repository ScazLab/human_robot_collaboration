# Baxter Collaboration
Yet another repo for the baxter collaboration task

## Prerequisites

## Installation

## Execution

 1. On one terminal, launch the `ARuco` software: `roslaunch baxter_collaboration baxter_marker_publisher.launch`
 2. On another terminal, launch the Baxter Collaboration software: `roslaunch baxter_collaboration baxter_collaboration.launch` (please note that these two commands should be together in the same launch file, but for developing and testing it is much better to keep them separate)
 3. Request actions to either one of the two arms by using the proper service (`/action_provider/service_left` for left arm, `/action_provider/service_right` for right arm). Here are some examples to make the demo work from terminal:
  * `rosservice call /action_provider/service_right "{action: 'hand_over', object: 17}"`
  * `rosservice call /action_provider/service_left "{action: 'get', object: 17}"`

### Supported actions

 * `home` (both arms): moves the arm to a specific joint configuration (i.e. it does not use IK)
 * `release` (both arms): opens the gripper (or releases the vacuum gripper)
 * ...

### Misc

 * To kill an action from the terminal, you can simulate a button press on the arm's cuff: `rostopic pub --once /robot/digital_io/left_lower_button/state baxter_core_msgs/DigitalIOState "{state: 1, isInputOnly: true}"`
