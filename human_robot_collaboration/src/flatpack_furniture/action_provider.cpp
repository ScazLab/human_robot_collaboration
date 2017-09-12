#include <stdio.h>

#include <ros/ros.h>
#include <signal.h>
#include "left_ctrl.h"
#include "right_ctrl.h"

using namespace std;

void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // What the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "action_provider");
    ros::NodeHandle _n("action_provider");

    bool use_robot;
    _n.param<bool>("use_robot", use_robot, true);
    printf("\n");
    ROS_INFO("use_robot flag set to %s", use_robot==true?"true":"false");

    printf("\n");
    LeftCtrl   left_ctrl("action_provider", "left", use_robot);
    printf("\n");
    RightCtrl right_ctrl("action_provider","right", use_robot);
    printf("\n");
    ROS_INFO("READY! Waiting for service messages..\n");

    //Override the default ros sigint handler.
    signal(SIGINT, mySigintHandler);

    ros::spin();
    return 0;
}

