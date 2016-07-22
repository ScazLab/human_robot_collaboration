#include <stdio.h>

#include <ros/ros.h>

#include "artag_controller/artag_controller.h"

using namespace std;

int main(int argc, char** argv)
{
    ROS_INFO("Picking up AR tags");
    ros::init(argc, argv, "artag_controller");

    PickUpARTag * _left_put = new PickUpARTag("left");
    
    _left_put->StartInternalThread();
    while( int(_left_put->getState() != PICK_UP )) {ros::spinOnce();}

    delete _left_put;
    return 0;
}