#include "robot_interface/robot_interface.h"

#include <tf/transform_datatypes.h>

using namespace std;
using namespace baxter_core_msgs;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace cv;

/**************************************************************************/
/*                         RobotInterface                                 */
/**************************************************************************/
RobotInterface::RobotInterface(string limb): _n("~"), _limb(limb), _state(START,0),
                                   spinner(4), ir_ok(false)
{
    _joint_cmd_pub = _n.advertise<JointCommand>("/robot/limb/" + _limb + "/joint_command", 1);
    _coll_av_pub   = _n.advertise<Empty>("/robot/limb/" + _limb + "/suppress_collision_avoidance", 1);
    
    _endpt_sub     = _n.subscribe("/robot/limb/" + _limb + "/endpoint_state",
                                    SUBSCRIBER_BUFFER, &RobotInterface::endpointCb, this);
    _ir_sub        = _n.subscribe("/robot/range/" + _limb + "_hand_range/state",
                                    SUBSCRIBER_BUFFER, &RobotInterface::IRCb, this);
    _cuff_sub      = _n.subscribe("/robot/digital_io/" + _limb + "_lower_button/state",
                                    SUBSCRIBER_BUFFER, &RobotInterface::cuffOKCb, this);
    
    _ik_client     = _n.serviceClient<SolvePositionIK>("/ExternalTools/" + _limb + 
                                                       "/PositionKinematicsNode/IKService");

    _init_time = ros::Time::now();

    _curr_max_range = 0;
    _curr_min_range = 0;
    _curr_range     = 0;

    _filt_force.push_back(0.0);
    _filt_force.push_back(0.0);
    _filt_force.push_back(0.0);

    if (getLimb()=="left")
    {
        _n.param<double>("force_threshold_left",  force_thres, FORCE_THRES_L);
    }
    else if (getLimb()=="right")
    {
        _n.param<double>("force_threshold_right", force_thres, FORCE_THRES_R);
    }

    ROS_INFO("[%s] Force Threshold : %g", getLimb().c_str(), force_thres);

    spinner.start();
}

RobotInterface::~RobotInterface() { }

void RobotInterface::cuffOKCb(const baxter_core_msgs::DigitalIOState& msg)
{
    if (msg.state == baxter_core_msgs::DigitalIOState::PRESSED)
    {
        setState(KILLED);
    }
}

bool RobotInterface::ok()
{
    bool res = ros::ok();
    res = res && getState() != KILLED;

    return res;
}

void RobotInterface::endpointCb(const baxter_core_msgs::EndpointState& msg) 
{
    ROS_DEBUG("endpointCb");
    _curr_pos      = msg.pose.position;
    _curr_ori      = msg.pose.orientation;
    _curr_wrench   = msg.wrench;

    tf::Quaternion _marker_quat;
    tf::quaternionMsgToTF(_curr_ori, _marker_quat);
    tf::Matrix3x3 _marker_mat(_marker_quat);

    // printf("Endpoint Orientation\n");
    // for (int j = 0; j < 3; ++j)
    // {
    //     printf("%g\t%g\t%g\n", _marker_mat[j][0], _marker_mat[j][1], _marker_mat[j][2]);
    // }
    // printf("\n");

    filterForces();
}

void RobotInterface::IRCb(const sensor_msgs::RangeConstPtr& msg) 
{
    ROS_DEBUG("IRCb");
    _curr_range = msg->range; 
    _curr_max_range = msg->max_range; 
    _curr_min_range = msg->min_range;

    if (!ir_ok)
    {
        ir_ok = true;
    }
}

void RobotInterface::filterForces()
{
    _filt_force[0] = (1 - FORCE_ALPHA) * _filt_force[0] + FORCE_ALPHA * _curr_wrench.force.x;
    _filt_force[1] = (1 - FORCE_ALPHA) * _filt_force[1] + FORCE_ALPHA * _curr_wrench.force.y;
    _filt_force[2] = (1 - FORCE_ALPHA) * _filt_force[2] + FORCE_ALPHA * _curr_wrench.force.z;
}

void RobotInterface::hoverAboveTokens(double height)
{
    goToPose(0.540, 0.570, height, VERTICAL_ORI_L);
}


bool RobotInterface::goToPoseNoCheck(double px, double py, double pz,
                                double ox, double oy, double oz, double ow)
{
    vector<double> joint_angles;
    if (!callIKService(px, py, pz, ox, oy, oz, ow, joint_angles)) return false;

    return goToPoseNoCheck(joint_angles);
}

bool RobotInterface::goToPoseNoCheck(vector<double> joint_angles)
{
    JointCommand joint_cmd;
    joint_cmd.mode = JointCommand::POSITION_MODE;
    
    setJointNames(joint_cmd);

    for(int i = 0; i < joint_angles.size(); i++)
    {
        joint_cmd.command.push_back(joint_angles[i]);
    }

    publish_joint_cmd(joint_cmd);

    return true;
}

bool RobotInterface::goToPose(double px, double py, double pz,
                         double ox, double oy, double oz, double ow,
                         std::string mode, bool disable_coll_av)
{
    vector<double> joint_angles;
    if (!callIKService(px, py, pz, ox, oy, oz, ow, joint_angles)) return false;

    while(RobotInterface::ok())
    {
        if (disable_coll_av)    suppressCollisionAv();

        if (!goToPoseNoCheck(joint_angles))   return false;

        if(hasPoseCompleted(px, py, pz, ox, oy, oz, ow, mode))
        {
            return true;
        }

        ros::spinOnce();
        ros::Rate(100).sleep();
    }

    return false;
}

bool RobotInterface::callIKService(double px, double py, double pz,
                              double ox, double oy, double oz, double ow,
                              std::vector<double>& joint_angles)
{
    PoseStamped pose_stamp;
    pose_stamp.header.frame_id = "base";
    pose_stamp.header.stamp    = ros::Time::now();

    setPosition(   pose_stamp.pose, px, py, pz);
    setOrientation(pose_stamp.pose, ox, oy, oz, ow);

    joint_angles.clear();
    bool got_solution = false;
    ros::Time start = ros::Time::now();
    float thresh_z = pose_stamp.pose.position.z + 0.120;

    while(!got_solution)
    {
        SolvePositionIK ik_srv;
        //ik_srv.request.seed_mode=2;       // i.e. SEED_CURRENT
        ik_srv.request.seed_mode=0;         // i.e. SEED_AUTO
        pose_stamp.header.stamp=ros::Time::now();
        ik_srv.request.pose_stamp.push_back(pose_stamp);
        
        if(_ik_client.call(ik_srv))
        {
            got_solution = ik_srv.response.isValid[0];

            if (got_solution)
            {
                joint_angles = ik_srv.response.joints[0].position;
                return true;
            }
            else
            {
                // if position cannot be reached, try a position with the same x-y coordinates
                // but higher z (useful when placing tokens)
                ROS_DEBUG("[%s] IK solution not valid: %g %g %g", getLimb().c_str(),
                                                       pose_stamp.pose.position.x,
                                                       pose_stamp.pose.position.y,
                                                       pose_stamp.pose.position.z);
                pose_stamp.pose.position.z += 0.004;
            } 
        }

        // if no solution is found within 200 milliseconds or no solution within the acceptable
        // z-coordinate threshold is found, then no solution exists and exit oufof loop
        if((ros::Time::now() - start).toSec() > 0.2 || pose_stamp.pose.position.z > thresh_z) 
        {
            ROS_WARN("[%s] Did not find a suitable IK solution! Final Position %g %g %g",
                                                                       getLimb().c_str(),
                                                            pose_stamp.pose.position.x,
                                                            pose_stamp.pose.position.y,
                                                            pose_stamp.pose.position.z);
            return false;
        }
    }

    return false;
}

bool RobotInterface::hasCollided(string mode)
{
    float thres;
    
    if     (mode == "strict") thres = 0.050;
    else if(mode ==  "loose") thres = 0.067;
    
    if(_curr_range <= _curr_max_range &&
       _curr_range >= _curr_min_range &&
       _curr_range <= thres) return true;
    else return false;
}

bool RobotInterface::hasPoseCompleted(double px, double py, double pz,
                                 double ox, double oy, double oz, double ow, string mode)
{
    ROS_DEBUG("[%s] Checking for position.. mode is %s", getLimb().c_str(), mode.c_str());
    if(mode == "strict")
    {
        if(!withinThres(_curr_pos.x, px, 0.001)) return false;
        if(!withinThres(_curr_pos.y, py, 0.001)) return false;
        if(!withinThres(_curr_pos.z, pz, 0.001)) return false;
    }
    else if(mode == "loose")
    {
        if(!withinThres(_curr_pos.x, px, 0.01)) return false;
        if(!withinThres(_curr_pos.y, py, 0.01)) return false;
        if(!withinThres(_curr_pos.z, pz, 0.01)) return false;
    }

    // ROS_INFO("[%s] Checking for orientation..", getLimb().c_str());
    if(!withinXHundredth(_curr_ori.x, ox, 2.5))  return false;
    if(!withinXHundredth(_curr_ori.y, oy, 2.5))  return false;
    if(!withinXHundredth(_curr_ori.z, oz, 2.5))  return false;
    if(!withinXHundredth(_curr_ori.w, ow, 2.5))  return false;

    return true;
}

void RobotInterface::setJointNames(JointCommand& joint_cmd)
{
    joint_cmd.names.push_back(_limb + "_s0");
    joint_cmd.names.push_back(_limb + "_s1");
    joint_cmd.names.push_back(_limb + "_e0");
    joint_cmd.names.push_back(_limb + "_e1");
    joint_cmd.names.push_back(_limb + "_w0");
    joint_cmd.names.push_back(_limb + "_w1");
    joint_cmd.names.push_back(_limb + "_w2");
}

bool RobotInterface::detectForceInteraction()
{
    double f_x = abs(_curr_wrench.force.x - _filt_force[0]);
    double f_y = abs(_curr_wrench.force.y - _filt_force[1]);
    double f_z = abs(_curr_wrench.force.z - _filt_force[2]);

    ROS_DEBUG("Interaction: %g %g %g", f_x, f_y, f_z);

    if (f_x > force_thres || f_y > force_thres || f_z > force_thres)
    {
        ROS_INFO("[%s] Interaction: %g %g %g", getLimb().c_str(), f_x, f_y, f_z);
        return true;
    }
    else
    {
        return false;
    }
}

bool RobotInterface::waitForForceInteraction(double _wait_time, bool disable_coll_av)
{
    ros::Time _init = ros::Time::now();

    while(RobotInterface::ok())
    {
        if (disable_coll_av)          suppressCollisionAv();
        if (detectForceInteraction())           return true;

        ros::spinOnce();
        ros::Rate(100).sleep();

        if ((ros::Time::now()-_init).toSec() > _wait_time)
        {
            ROS_ERROR("No force interaction has been detected in %gs!",_wait_time);
            return false;
        }
    }

    return false;
}

void RobotInterface::setState(int state)
{
    _state.state = state;
    // store the time elapsed between object initialization and state change
    _state.time = (ros::Time::now() - _init_time).toSec();
}

void RobotInterface::publish_joint_cmd(baxter_core_msgs::JointCommand _cmd)
{
    _joint_cmd_pub.publish(_cmd);
}

void RobotInterface::suppressCollisionAv()
{
    std_msgs::Empty empty_cmd;
    _coll_av_pub.publish(empty_cmd);
}

/**************************************************************************/
/*                          ROSThreadImage                                */
/**************************************************************************/

ROSThreadImage::ROSThreadImage(string limb): _img_trp(_n), RobotInterface(limb)
{
    _img_sub = _img_trp.subscribe("/cameras/"+getLimb()+"_hand_camera/image",
                           SUBSCRIBER_BUFFER, &ROSThreadImage::imageCb, this);
    pthread_mutex_init(&_mutex_img, NULL);
}

ROSThreadImage::~ROSThreadImage() {}

void ROSThreadImage::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_DEBUG("imageCb");
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("[Arm Controller] cv_bridge exception: %s", e.what());
        return;
    }

    pthread_mutex_lock(&_mutex_img);
    _curr_img  = cv_ptr->image.clone();
    _img_size  =      _curr_img.size();
    _img_empty =     _curr_img.empty();
    pthread_mutex_unlock(&_mutex_img);   
}

