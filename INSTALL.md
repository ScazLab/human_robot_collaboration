# Installation

Guide for installing, compiling and testing the `human_robot_collaboration` package in your ROS environment. This tutorial should be useful regardless of your skill level, from a first-time ROS user to an experienced engineer.

## Prerequisites

### System Dependencies

This repository needs `nlopt` and `std-c++14`.

#### NLOPT

```
sudo apt-get install libnlopt-dev
````

#### C++14

Building `human_robot_collaboration` requires that you have `C++14` supported on your computer in some fashion. `C++14` is not available by default on Ubuntu 14.04, so your best bet is to get a `gcc` version that supports it (from `gcc-4.9` upwards).
From [here](https://gist.github.com/application2000/73fd6f4bf1be6600a2cf9f56315a2d91)):

```
sudo apt-get install build-essential software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-snapshot gcc-4.9 g++-4.9
```

If you have other versions of `gcc` installed already (Ubuntu 14.04 comes with `gcc-4.8` by default), you might need to select  `gcc-4.9` as default. You can do this by typing `sudo update-alternatives --config gcc` and following the instructions in the terminal.

### ROS Dependencies

This repository supports `ROS indigo`, although more recent versions should work just fine. [Here](https://alecive.github.io/ros_installation.html#installing-and-configuring-your-ros-environment)'s a not-so-recent guide on how to install ROS.

#### Catkin Tools

We use the new Catkin Command Line Tools `catkin_tools`, a Python package that provides command line tools for working with the catkin meta build system and catkin workspaces. This package was announced in **March 2015** and is still in beta, but we didn't experience any problem with it. The following instructions apply to this new package, even though the repository can be used and compile with the old `catkin_make` without issues.

```
sudo apt-get install python-catkin-tools
```

#### ROS Package Dependencies

* `aruco_ros`: you should use [this version of `aruco_ros`](https://github.com/ScazLab/aruco_ros) which is a fork of (https://github.com/pal-robotics/aruco_ros)
 * `trac_ik` : for the time being, you should use [this version of `trac_ik`](https://bitbucket.org/alecive/trac_ik). It is a fork of [the original `trac_ik`](https://bitbucket.org/traclabs/trac_ik).
 * `baxter_description`: it is used for testing the `robot_interface` library. It can be downloaded from the `baxter_common` repository, available [here](https://github.com/RethinkRobotics/baxter_common).
 * `svox_tts` : it's a SVOX-PICO based wrapper for text-to-speech. It's not necessary, but recommended. Available [here](https://github.com/ScazLab/svox_tts).
 * `rosbridge` : Not necessary, but recommended.
 * `ros_speech2text` : Not necessary, but recommended.

## Compilation & Testing

### Create a new ROS workspace

It is good practice to separate your development workspace from the workspace in which ROS resides. So, let's create and initialize a new workspace called `hrc_ws` into an already existing `code` folder.

```
mkdir -p ~/code/hrc_ws/src
cd ~/code/hrc_ws
catkin init
catkin build
```

The last `build` command should assure you that everything works. Now you should add the following line to your `.bashrc`:

```
source $HOME/code/hrc_ws/devel/setup.bash
```

### Download dependencies

Mandatory dependencies:

```
cd ~/code/hrc_ws/src
git clone https://github.com/ScazLab/aruco_ros
git clone https://bitbucket.org/alecive/trac_ik.git
git clone https://github.com/RethinkRobotics/baxter_common.git
```

To avoid downloading all the dependencies needed to compile `track_ik_kinematics_plugin`, we need to tell `catkin` to ignore it:

```
cd ~/code/hrc_ws/src/trac_ik/trac_ik_kinematics_plugin
touch CATKIN_IGNORE
```

Optional (but recommended) dependencies:

```
cd ~/code/hrc_ws/src
git clone https://github.com/ScazLab/svox_tts.git
git clone https://github.com/RobotWebTools/rosbridge_suite.git
git clone https://github.com/scazlab/ros_speech2text.git
```
And finally:

```
cd ~/code/hrc_ws/src
git clone https://github.com/ScazLab/human_robot_collaboration.git
```

### Compilation & Testing

Compile `human_robot_collaboration`:

```
cd ~/code/hrc_ws
catkin build human_robot_collaboration
```

Compile tests and run them:

```
catkin build human_robot_collaboration --catkin-make-args run_tests
```

Check the results of the tests:

```
catkin build human_robot_collaboration_lib --catkin-make-args run_tests && catkin_test_results build/human_robot_collaboration_lib
```
