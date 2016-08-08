#include <stdio.h>

#include <ros/ros.h>
#include <signal.h>
#include "robot_interface/artag_ctrl.h"
#include "robot_interface/hold_ctrl.h"

using namespace std;


void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "action_provider");

    ARTagCtrl  left_ctrl("action_provider","left");
    HoldCtrl  right_ctrl("action_provider","right");

    printf("\n");
    ROS_INFO("READY! Waiting for service messages..\n");
    
    //Override the default ros sigint handler.
    signal(SIGINT, mySigintHandler);
  
    ros::spin();
    return 0;
}

