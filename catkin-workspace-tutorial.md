# catkin workspace start-up guide
Guide for building the catkin workspace in ROS. 

# Getting ready
* Install ROS: [Alessandro's guide to ROS installation](https://alecive.github.io/ros_installation.html#installing-and-configuring-your-ros-environment)
* Make sure you have `catkin_tools` installed:
 ```
 sudo apt-get update
 sudo apt-get install python-catkin-tools
 ```
* Create and initialize your new workspace (we'll call it `baxter_ws`, but you can call it whatever you like).
```
cd ~/code
mkdir -p baxter_ws/src
cd baxter_ws
catkin init
```

# Prequisites & more
First, the basic prerequisites as listed on guide [here](https://github.com/ScazLab/human_robot_collaboration).
Start by navigating to your workspace's `src` file (e.g. `cd ~/code/baxter_ws/src`). Now you can grab the prerequisites.
* `nlopt`: This should be installed from the Ubuntu repositories: `sudo apt-get install libnlopt-dev`. 
* `aruco_ros`: Get the recommended version via `git clone https://github.com/ScazLab/aruco_ros`
* `trac_ik`: For now, use this version of it via `git clone https://bitbucket.org/alecive/trac_ik.git`
  - Next, navigate to `trac_ik/trac_ik_kinematics_plugin` and make an empty file `CATKIN_IGNORE`.
  ```
  cd trac_ik/trac-ik_kinematics_plugin
  touch CATKIN_IGNORE
  cd ~/code/baxter_ws/src
  ```
* `svox_tts`: Not necessary, but recommended. Available via `git clone https://github.com/ScazLab/svox_tts.git`
* `baxter_description`: Contained in the `baxter_common` repository. It's easiest to grab the whole thing: `git clone https://github.com/RethinkRobotics/baxter_common.git`

Some other useful prequisites as listed in the `human_robot_collaboration/dependencies.rosinstall`:
* `rosbridge`: Available via `git clone https://github.com/RobotWebTools/rosbridge_suite.git`
* `ros_s2t`: Available via `git clone https://github.com/scazlab/ros_speech2text.git`

And finally...
* `human_robot_collaboration`: Available via `git clone https://github.com/ScazLab/human_robot_collaboration.git`

# Supporting std-c++14

Building `human_robot_collaboration` requires that you have `std-c++14` supported on your computer in some fashion. This does not come with Ubuntu 14.04, so your best bet is to get a gcc package that supports it like `gcc-4.9`. 
There's probably an easier way to install it, but here's how I did it (from a guide [here](https://gist.github.com/application2000/73fd6f4bf1be6600a2cf9f56315a2d91)):
```
sudo apt-get install build-essential software-properties-common -y
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt-get update
sudo apt-get install gcc-snapshot -y
sudo apt-get update
sudo apt-get install gcc-4.9 g++-4.9 -y 
```
If you have other versions of gcc installed already (Ubuntu 14.04 comes with gcc 4.8 by default), you might need to select 4.9 as the gcc you want to work with by default. You can do this by typing `sudo update-alternatives --config gcc` and following the instructions in the terminal.  

# Building the workspace

The easy part: just navigate back to your workspace(e.g. `cd ../`) and run `catkin build`. 
You should now have three new folders in your workspace folder: build, devel, and logs. Open your `~/.bashrc` file and add the following line:
```
source ~/code/<workspace name>/devel/setup.bash
```
where `<workspace name>` in our example is `baxter_ws`. 
