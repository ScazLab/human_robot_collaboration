#include "artag_controller/artag_controller.h"
#include <iostream>

using namespace std;

ARTagController::ARTagController(std::string limb) : ROSThread(limb), marker_id(-1), action("")
{
    _aruco_sub = _n.subscribe("/aruco_marker_publisher/markers",
                               SUBSCRIBER_BUFFER, &ARTagController::ARCallback, this);

    elapsed_time = 0;

    _curr_marker_pose.position.x = 100;
}

ARTagController::~ARTagController() { }

// Protected
void ARTagController::InternalThreadEntry()
{
    clearMarkerPose();

    if (action == ACTION_GET && (int(getState()) == START || int(getState()) == ERROR))
    {
        if (pickObject())   setState(PICK_UP);
        else                setState(ERROR);
    }
    else if (action == ACTION_PASS && int(getState()) == PICK_UP)
    {
        if(passObject())   setState(PASSED);
        else               setState(ERROR);
    }
    else
    {
        setState(ERROR);
    }

    pthread_exit(NULL);
    return;
}

bool ARTagController::goHome()
{
    ARTagController::hoverAboveTokens(POS_HIGH);
    setState(START);
    return true;
}

bool ARTagController::releaseObject()
{
    bool res = ROSThread::releaseObject();
    setState(START);
    return res;
}

bool ARTagController::pickObject()
{
    ARTagController::hoverAboveTokens(POS_HIGH);
        
    bool res = pickARTag();

    if (res)
    {
        ARTagController::hoverAboveTokens(POS_LOW);

        // ros::Duration(2.0).sleep();
        // _gripper->blow();
        // releaseObject();
    }

    return res;
}

bool ARTagController::passObject()
{
    ARTagController::moveObjectTowardHuman();
    return true;
}

void ARTagController::ARCallback(const aruco_msgs::MarkerArray& msg) 
{
    bool marker_found = false;

    for (int i = 0; i < msg.markers.size(); ++i)
    {
        ROS_DEBUG("Processing marker with id %i",msg.markers[i].id);

        if (msg.markers[i].id == marker_id)
        {
            marker_found = true;
            _curr_marker_pose = msg.markers[i].pose.pose;

            ROS_DEBUG("Marker is in: %g %g %g", _curr_marker_pose.position.x,
                                                _curr_marker_pose.position.y,
                                                _curr_marker_pose.position.z);
        }
    }

    // if (!marker_found)
    // {
    //     _curr_marker_pose.position.x = 100;
    // }
}

void ARTagController::clearMarkerPose()
{
    _curr_marker_pose.position.x = 100;
}

void ARTagController::hoverAboveTokens(double height)
{
    geometry_msgs::PoseStamped req_pose_stamped;
    req_pose_stamped.header.frame_id = "base";
    setPosition(   &req_pose_stamped.pose, 0.60, 0.45, height);
    setOrientation(&req_pose_stamped.pose, VERTICAL_ORIENTATION_LEFT_ARM);
    ROSThread::goToPose(req_pose_stamped);
}

void ARTagController::moveObjectTowardHuman()
{
    geometry_msgs::PoseStamped req_pose_stamped;
    req_pose_stamped.header.frame_id = "base";
    setPosition(   &req_pose_stamped.pose, 0.80, 0.26, 0.32);
    setOrientation(&req_pose_stamped.pose, HORIZONTAL_ORIENTATION_LEFT_ARM);
    ROSThread::goToPose(req_pose_stamped);
}

bool ARTagController::pickARTag()
{
    ROS_DEBUG("Start Picking up tag..");
    geometry_msgs::PoseStamped req_pose_stamped;
    ros::Time start_time = ros::Time::now();                

    if (_curr_marker_pose.position.x == 100)
    {
        ROS_ERROR("No callback from ARuco, or object with ID %i not found. Stopping.", marker_id);
        return false;
    }
    else if (_curr_range == 0 || _curr_min_range == 0 || _curr_max_range == 0)
    {
        ROS_ERROR("I didn't receive a callback from the IR sensor! Stopping.");
        return false;
    }

    int ik_failures = 0;
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

        ROS_DEBUG("Time %g Going to: %g %g %g", new_elapsed_time,
                                                req_pose_stamped.pose.position.x,
                                                req_pose_stamped.pose.position.y,
                                                req_pose_stamped.pose.position.z);

        if (goToPose(req_pose_stamped) == true)
        {
            ik_failures = 0;
            if (new_elapsed_time - elapsed_time > 0.02)
            {
                ROS_WARN("\t\t\t\t\t\tTime elapsed: %g", new_elapsed_time - elapsed_time);
            }
            elapsed_time = new_elapsed_time;

            if(hasCollided(_curr_range, _curr_max_range, _curr_min_range, "strict")) 
            {
                ROS_INFO("Collision!");
                break;
            }

            ros::Rate(100).sleep();
        }
        else
        {
            ik_failures++;
        }

        if (ik_failures == 10)
        {
            return false;
        }
    }
    
    return suckObject();
}

bool ARTagController::goToPose(geometry_msgs::PoseStamped req_pose_stamped)
{
    vector<double> joint_angles;
    if (getJointAngles(req_pose_stamped,joint_angles) == false)
    {
        return false;
    }

    baxter_core_msgs::JointCommand joint_cmd;
    joint_cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    joint_cmd.command.resize(7);
    setNames(&joint_cmd, getLimb());

    for(int i = 0; i < 7; i++) {
        joint_cmd.command[i] = joint_angles[i];
    }

    _joint_cmd_pub.publish(joint_cmd);

    return true;
}
