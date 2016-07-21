#include "artag_controller/artag_controller.h"
#include <iostream>

using namespace std;

PickUpARTag::PickUpARTag(std::string limb) : ROSThread(limb)
{
    _aruco_sub = _nh.subscribe("/robot/range/left_hand_range/state", 1, &PickUpARTag::ARCallback, this);
}

PickUpARTag::~PickUpARTag() { }

// Protected
void PickUpARTag::InternalThreadEntry()
{
    // wait for IR sensor callback
    while(ros::ok())
    {
        if(!(_curr_range == 0 && _curr_min_range == 0 && _curr_max_range == 0))
        {
            break; 
        }

        ros::Rate(100).sleep();
    }

    // wait for image callback
    while(ros::ok())
    {
        if(_curr_marker_pose.position.x !=0.0) break;
    }

    // hoverAboveTokens("high");
    // gripToken();
    // hoverAboveTokens("low");

    // setState(PICK_UP);
    pthread_exit(NULL);  
}

void PickUpARTag::ARCallback(const aruco_msgs::MarkerArray& msg) 
{
    _curr_marker_pose = msg.markers[0].pose.pose;

    cout << _curr_marker_pose << endl;
    // _curr_range = msg->range; 
    // _curr_max_range = msg->max_range; 
    // _curr_min_range = msg->min_range;
}