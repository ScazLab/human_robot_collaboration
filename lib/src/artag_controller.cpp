#include "robot_interface/artag_controller.h"

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

    int s = int(getState());

    setState(WORKING);

    if (action == ACTION_HOME)
    {
        if (goHome())   setState(START);
        else            setState(ERROR);
    }
    else if (action == ACTION_RELEASE)
    {
        if (releaseObject())   setState(START);
        else                   setState(ERROR);
    }
    else if (action == ACTION_GET && (s == START ||
                                      s == ERROR ||
                                      s == PASSED))
    {
        if (pickObject())   setState(PICK_UP);
        else                recoverFromError();
    }
    else if (action == ACTION_PASS && s == PICK_UP)
    {
        if(passObject())   setState(PASSED);
        else               recoverFromError();
    }
    else
    {
        ROS_ERROR("Invalid State %i", s);
        setState(ERROR);
    }

    pthread_exit(NULL);
    return;
}

void ARTagController::recoverFromError()
{
    releaseObject();
    goHome();
    setState(ERROR);
}

bool ARTagController::goHome()
{
    return hoverAboveTable(POS_LOW);
}

bool ARTagController::releaseObject()
{
    return ROSThread::releaseObject();
}

bool ARTagController::pickObject()
{
    if (!hoverAboveTable(POS_HIGH)) return false;
    if (!pickARTag())               return false;
    if (!hoverAboveTable(POS_LOW)) return false;

    return true;
}

bool ARTagController::passObject()
{
    if (!moveObjectTowardHuman())       return false;
    ros::Duration(1.0).sleep();
    if (!waitForForceInteraction())     return false;
    if (!releaseObject())               return false;
    if (!goHome())                      return false;

    return true;
}

void ARTagController::ARCallback(const aruco_msgs::MarkerArray& msg) 
{
    for (int i = 0; i < msg.markers.size(); ++i)
    {
        ROS_DEBUG("Processing marker with id %i",msg.markers[i].id);

        if (msg.markers[i].id == marker_id)
        {
            _curr_marker_pose = msg.markers[i].pose.pose;

            ROS_DEBUG("Marker is in: %g %g %g", _curr_marker_pose.position.x,
                                                _curr_marker_pose.position.y,
                                                _curr_marker_pose.position.z);
        }
    }
}

void ARTagController::clearMarkerPose()
{
    _curr_marker_pose.position.x = 100;
}

bool ARTagController::hoverAboveTable(double height)
{
    return ROSThread::goToPose(0.60, 0.45, height, VERTICAL_ORIENTATION_LEFT_ARM);
}

bool ARTagController::moveObjectTowardHuman()
{
    return ROSThread::goToPose(0.80, 0.26, 0.32, HORIZONTAL_ORIENTATION_LEFT_ARM);
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

        if (ARTagController::goToPose(x,y,z,VERTICAL_ORIENTATION_LEFT_ARM) == true)
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
            ros::spinOnce();
        }
        else
        {
            ik_failures++;
        }

        if (ik_failures == 20)
        {
            return false;
        }
    }
    
    return gripObject();
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
    if (!getJointAngles(req_pose_stamped,joint_angles)) return false;

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
