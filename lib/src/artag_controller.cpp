#include "artag_controller/artag_controller.h"
#include <iostream>

using namespace std;

ARTagController::ARTagController(std::string limb) : ROSThread(limb)
{
    _aruco_sub = _nh.subscribe("/aruco_marker_publisher/markers", SUBSCRIBER_BUFFER,
                                &ARTagController::ARCallback, this);

    elapsed_time = 0;

    _curr_marker_pose.position.x = 100;
}

ARTagController::~ARTagController() { }

// Protected
void ARTagController::InternalThreadEntry()
{
    ARTagController::hoverAboveTokens(POS_HIGH);
    pickARTag();
    ARTagController::hoverAboveTokens(POS_LOW);

    ros::Duration(1.0).sleep();
    _gripper->blow();

    setState(PICK_UP);
    pthread_exit(NULL);
}

void ARTagController::ARCallback(const aruco_msgs::MarkerArray& msg) 
{
    for (int i = 0; i < msg.markers.size(); ++i)
    {
        ROS_DEBUG("Processing marker with id %i",msg.markers[i].id);

        if (msg.markers[i].id == 24)
        {
            _curr_marker_pose = msg.markers[i].pose.pose;

            ROS_INFO("Marker is in: %g %g %g", _curr_marker_pose.position.x,
                       _curr_marker_pose.position.y, _curr_marker_pose.position.z);
        }
    }
}

void ARTagController::hoverAboveTokens(double height)
{
    geometry_msgs::PoseStamped req_pose_stamped;
    req_pose_stamped.header.frame_id = "base";
    setPosition(   &req_pose_stamped.pose, 0.60, 0.45, height);
    setOrientation(&req_pose_stamped.pose, VERTICAL_ORIENTATION_LEFT_ARM);
    ROSThread::goToPose(req_pose_stamped);
}

bool ARTagController::pickARTag()
{
    ROS_DEBUG("Start Picking up tag..");
    geometry_msgs::PoseStamped req_pose_stamped;
    ros::Time start_time = ros::Time::now();                

    if (_curr_marker_pose.position.x == 100)
    {
        ROS_ERROR("I didn't receive a callback from ARuco! Stopping.");
        return false;
    }
    else if (_curr_range == 0 || _curr_min_range == 0 || _curr_max_range == 0)
    {
        ROS_ERROR("I didn't receive a callback from the IR sensor! Stopping.");
        return false;
    }

    while(ros::ok())
    {
        ros::Time now_time = ros::Time::now();
        double new_elapsed_time = (now_time - start_time).toSec();

        req_pose_stamped.header.frame_id = "base";

        // move incrementally towards token
        setPosition(&req_pose_stamped.pose, 
                     _curr_marker_pose.position.x, _curr_marker_pose.position.y,
                     POS_HIGH - PICK_UP_SPEED * new_elapsed_time);

        setOrientation(&req_pose_stamped.pose, VERTICAL_ORIENTATION_LEFT_ARM);

        ROS_INFO("Time %g Going to: %g %g %g", new_elapsed_time, req_pose_stamped.pose.position.x,
                               req_pose_stamped.pose.position.y, req_pose_stamped.pose.position.z);

        goToPose(req_pose_stamped);

        if (new_elapsed_time - elapsed_time > 0.02)
        {
            ROS_WARN("\t\t\t\t\t\t\t\t\tTime elapsed: %g", new_elapsed_time - elapsed_time);
        }
        elapsed_time = new_elapsed_time;

        if(hasCollided(_curr_range, _curr_max_range, _curr_min_range, "strict")) 
        {
            ROS_INFO("Collision!");
            break;
        }

        ros::Rate(100).sleep();
    }
    _gripper->suck();

    return true;
}

void ARTagController::goToPose(geometry_msgs::PoseStamped req_pose_stamped)
{
    vector<double> joint_angles = getJointAngles(&req_pose_stamped);

    baxter_core_msgs::JointCommand joint_cmd;
    joint_cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    joint_cmd.command.resize(7);
    setNames(&joint_cmd, getLimb());

    for(int i = 0; i < 7; i++) {
        joint_cmd.command[i] = joint_angles[i];
    }

    _joint_cmd_pub.publish(joint_cmd);
}
