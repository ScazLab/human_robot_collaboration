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

    if (action == ACTION_GET && 
       (int(getState()) == START || int(getState()) == ERROR) || int(getState()) == PASSED)
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
        ROS_ERROR("Action %s State %i", action.c_str(), int(getState()));
        setState(ERROR);
    }

    pthread_exit(NULL);
    return;
}

bool ARTagController::goHome()
{
    ARTagController::hoverAboveTokens(POS_LOW);
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
        goHome();
        // ros::Duration(2.0).sleep();
        // _gripper->blow();
        // releaseObject();
    }

    return res;
}

bool ARTagController::passObject()
{
    ARTagController::moveObjectTowardHuman();

    bool res = waitForForceInteraction();

    if (res)
    {
        releaseObject();
        goHome();
    }
    return res;
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
    ROSThread::goToPose(0.60, 0.45, height, VERTICAL_ORIENTATION_LEFT_ARM);
}

void ARTagController::moveObjectTowardHuman()
{
    ROSThread::goToPose(0.80, 0.26, 0.32, HORIZONTAL_ORIENTATION_LEFT_ARM);
}

bool ARTagController::pickARTag()
{
    ROS_DEBUG("Start Picking up tag..");
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

        double x = _curr_marker_pose.position.x;
        double y = _curr_marker_pose.position.y;
        double z = POS_HIGH - PICK_UP_SPEED * new_elapsed_time;

        ROS_DEBUG("Time %g Going to: %g %g %g", new_elapsed_time, x, y, z);

        if (goToPose(x,y,z,VERTICAL_ORIENTATION_LEFT_ARM) == true)
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

bool ARTagController::goToPose(double px, double py, double pz,
                               double ox, double oy, double oz, double ow)
{
    geometry_msgs::PoseStamped req_pose_stamped;
    req_pose_stamped.header.frame_id = "base";
    req_pose_stamped.header.stamp    = ros::Time::now();

    setPosition(   req_pose_stamped.pose, px, py, pz);
    setOrientation(req_pose_stamped.pose, ox, oy, oz, ow);

    vector<double> joint_angles;
    if (getJointAngles(req_pose_stamped,joint_angles) == false)
    {
        return false;
    }

    baxter_core_msgs::JointCommand joint_cmd;
    joint_cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    joint_cmd.command.resize(7);
    setJointNames(joint_cmd);

    for(int i = 0; i < 7; i++) {
        joint_cmd.command[i] = joint_angles[i];
    }

    _joint_cmd_pub.publish(joint_cmd);

    return true;
}
