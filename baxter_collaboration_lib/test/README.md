## Testing

#### Using [Catkin Tools](https://catkin-tools.readthedocs.io/en/latest/)

1. Navigate to your workspace: `cd ros_ws`
2. `REPO_NAME=baxter_collaboration`
3. Compile and test: `catkin build "$REPO_NAME"_lib && catkin build "$REPO_NAME"_lib --catkin-make-args run_tests`
4. Check the results of above: `catkin_test_results build/"$REPO_NAME"_lib || (cat build/"$REPO_NAME"_lib/test_results/"$REPO_NAME"_lib/*.xml ; false)`

Troubleshooting:
* If build fails, make sure prerequisites are installed: [`aruco_ros`](https://github.com/ScazLab/aruco_ros), `nlopt`, [`trac_ik`](https://bitbucket.org/alecive/trac_ik) and [`baxter_common`](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup#adeedda5360641914fe9c5d681c30026).

#### Manual Testing

1. Open a terminal (T1) and run: 
	```
	cd ros_ws
	roscore
	```
2. Open a second terminal (T2) and run: `roslaunch baxter_collaboration_lib baxter_urdf.launch`
3. On T2, run the test, for example: `./devel/lib/baxter_collaboration_lib/test_robot_interface`

Troubleshooting:
* If presented with a `roslaunch` error, navigate to `~/.bashrc` and make sure the following is appended to the file:
	```
	source /opt/ros/indigo/setup.bash
	source ~/ros_ws/devel/setup.bash
	```
* If seeing unchanging results, `catkin build` may be required.

#### Checklist
* /baxter_collaboration_lib/src/robot_utils
	- [x] utils.cpp
	- [ ] baxter_trac_ik.cpp
	- [ ] ros_thread.cpp
	- [ ] ros_thread_image.cpp
	- [ ] ros_thread_obj.cpp

* /baxter_collaboration_lib/src/robot_interface
	- [x] robot_interface.cpp
	- [ ] arm_ctrl.cpp
	- [ ] gripper.cpp
	
* /baxter_collaboration_lib/src/robot_perception
	- [ ] aruco_client.cpp
	- [ ] cartesian_estimator.cpp
	- [ ] cartesian_estimator_client.cpp
	- [ ] cartesian_estimator_hsv.cpp

