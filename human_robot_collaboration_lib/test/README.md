# Testing

## Using [Catkin Tools](https://catkin-tools.readthedocs.io/en/latest/)

1. Navigate to your workspace: `cd ros_ws`
2. `REPO_NAME=human_robot_collaboration`
3. Compile and test: `catkin build "$REPO_NAME"_lib && catkin build "$REPO_NAME"_lib --catkin-make-args run_tests`
4. Check the results of above: `catkin_test_results build/"$REPO_NAME"_lib || (cat build/"$REPO_NAME"_lib/test_results/"$REPO_NAME"_lib/*.xml ; false)`

Troubleshooting:
* If build fails, make sure prerequisites are installed: [`aruco_ros`](https://github.com/ScazLab/aruco_ros), `nlopt`, [`trac_ik`](https://bitbucket.org/alecive/trac_ik), [`baxter_common`](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup#adeedda5360641914fe9c5d681c30026).

## Manual Testing for unit tests

1. Open a terminal (T1) and run:
	```
	cd ros_ws
	roscore
	```
2. Open a second terminal (T2) and run: `roslaunch human_robot_collaboration_lib baxter_urdf.launch`
3. On T2, run the test, for example: `./devel/lib/human_robot_collaboration_lib/test_robot_interface`

Troubleshooting:
* If presented with a `roslaunch` error, navigate to `~/.bashrc` and make sure the following is appended to the file:
	```
	source /opt/ros/indigo/setup.bash
	source ~/your_ws/devel/setup.bash
	```
* If seeing unchanging results, `catkin build` may be required.

## Manual testing for ROS tests

1. Open a terminal (T1) and run:
    ```
    cd ros_ws
    roscore
    ```
2. Open a second terminal (T2) and run: `rostest --text human_robot_collaboration_lib test_arm_ctrl.test`
