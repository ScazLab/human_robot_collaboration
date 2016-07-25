#include <stdio.h>

#include <ros/ros.h>

#include "actionProvider.h"

using namespace std;

int main(int argc, char ** argv)
{
    ROS_INFO("Waiting for a service message..");

    ros::init(argc, argv, "action_provider");

    actionProvider ap("action_provider","left");

    ros::spin();
    return 0;
}

// int main(int argc, char** argv)
// {
//     ROS_INFO("Picking up AR tags");
//     ros::init(argc, argv, "artag_controller");

//     ARTagController * _left_put = new ARTagController("left");
    
//     _left_put->StartInternalThread();
//     while( int(_left_put->getState() != PICK_UP )) {ros::spinOnce();}

//     delete _left_put;
//     return 0;
// }
