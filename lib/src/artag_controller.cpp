#include "artag_controller/artag_controller.h"
#include <iostream>

using namespace std;

ARTagController::ARTagController(std::string limb) : ROSThread(limb)
{
    _aruco_sub = _nh.subscribe("/aruco_marker_publisher/markers", 1, &ARTagController::ARCallback, this);
}

ARTagController::~ARTagController() { }

// Protected
void ARTagController::InternalThreadEntry()
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
        if(_curr_marker_pose.position.x != 0.0) break;
    }

    hoverAboveTokens("high");
    pickARTag();
    hoverAboveTokens("low");

    ros::Duration(1.0).sleep();
    _gripper->blow();

    setState(PICK_UP);
    pthread_exit(NULL);  
}

void ARTagController::ARCallback(const aruco_msgs::MarkerArray& msg) 
{
    _curr_marker_pose = msg.markers[0].pose.pose;

    cout << _curr_marker_pose << endl;
    // _curr_range = msg->range; 
    // _curr_max_range = msg->max_range; 
    // _curr_min_range = msg->min_range;
}

void ARTagController::hoverAboveTokens(std::string height)
{
    geometry_msgs::PoseStamped req_pose_stamped;
    req_pose_stamped.header.frame_id = "base";
    setPosition(   &req_pose_stamped.pose, 0.540, 0.50, height == "high" ? 0.400 : 0.150);
    setOrientation(&req_pose_stamped.pose, VERTICAL_ORIENTATION_LEFT_ARM);
    goToPose(req_pose_stamped);
}

void ARTagController::pickARTag()
{
    geometry_msgs::PoseStamped req_pose_stamped;
    ros::Time start_time = ros::Time::now();                

    while(ros::ok())
    {
        ros::Time now_time = ros::Time::now();

        req_pose_stamped.header.frame_id = "base";

        // move incrementally towards token
        setPosition(&req_pose_stamped.pose, 
                     _curr_marker_pose.position.x,
                     _curr_marker_pose.position.y,
                     0.375 - 0.1 * (now_time - start_time).toSec());

        setOrientation(&req_pose_stamped.pose, VERTICAL_ORIENTATION_LEFT_ARM);

        vector<double> joint_angles = getJointAngles(&req_pose_stamped);

        baxter_core_msgs::JointCommand joint_cmd;
        joint_cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;

        setNames(&joint_cmd, _limb);
        joint_cmd.command.resize(7);

        for(int i = 0; i < 7; i++) {
            joint_cmd.command[i] = joint_angles[i];
        }

        _joint_cmd_pub.publish(joint_cmd);
        ros::Rate(500).sleep();

        if(hasCollided(_curr_range, _curr_max_range, _curr_min_range, "strict")) 
        {
            break;
        }
    }
    _gripper->suck();
}

