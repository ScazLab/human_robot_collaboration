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
RobotInterface::RobotInterface(string name, string limb, bool no_robot) :
        _n(name), _name(name), _limb(limb), _state(START,0), spinner(4),
        ir_ok(false), _no_robot(no_robot), ik_solver(limb, no_robot)
{
    if (no_robot) return;

    _joint_cmd_pub = _n.advertise<JointCommand>("/robot/limb/" + _limb + "/joint_command", 1);
    _coll_av_pub   = _n.advertise<Empty>("/robot/limb/" + _limb + "/suppress_collision_avoidance", 1);

    _endpt_sub     = _n.subscribe("/robot/limb/" + _limb + "/endpoint_state",
                                    SUBSCRIBER_BUFFER, &RobotInterface::endpointCb, this);
    _ir_sub        = _n.subscribe("/robot/range/" + _limb + "_hand_range/state",
                                    SUBSCRIBER_BUFFER, &RobotInterface::IRCb, this);
    _cuff_sub      = _n.subscribe("/robot/digital_io/" + _limb + "_lower_button/state",
                                    SUBSCRIBER_BUFFER, &RobotInterface::cuffCb, this);

    _jntstate_sub  = _n.subscribe("/robot/joint_states",
                                    SUBSCRIBER_BUFFER, &RobotInterface::jointStatesCb, this);

    _coll_av_sub   = _n.subscribe("/robot/limb/" + _limb + "/collision_avoidance_state",
                                    SUBSCRIBER_BUFFER, &RobotInterface::collAvCb, this);

    _init_time = ros::Time::now();

    _curr_max_range = 0;
    _curr_min_range = 0;
    _curr_range     = 0;

    _filt_force.push_back(0.0);
    _filt_force.push_back(0.0);
    _filt_force.push_back(0.0);

    _filt_updated = 0;
    _filt_change.push_back(0.0);
    _filt_change.push_back(0.0);
    _filt_change.push_back(0.0);

    if (getLimb()=="left")
    {
        _n.param<double>("force_threshold_left",  force_thres, FORCE_THRES_L);
        _n.param<double>("force_filt_variance_left", filt_variance, FORCE_FILT_VAR_L);
        _n.param<double>("rel_force_threshold_left", rel_force_thres, REL_FORCE_THRES_L);
    }
    else if (getLimb()=="right")
    {
        _n.param<double>("force_threshold_right", force_thres, FORCE_THRES_R);
    }

    ROS_INFO("[%s] Force Threshold : %g", getLimb().c_str(), force_thres);
    ROS_INFO("[%s] Force Filter Variance: %g", getLimb().c_str(), filt_variance);
    ROS_INFO("[%s] Relative Force Threshold: %g", getLimb().c_str(), rel_force_thres);

    pthread_mutex_init(&_mutex_jnts, NULL);
    spinner.start();
}

bool RobotInterface::ok()
{
    bool res = ros::ok();
    res = res && getState() != KILLED;

    return res;
}

void RobotInterface::collAvCb(const baxter_core_msgs::CollisionAvoidanceState& msg)
{
    if (msg.collision_object.size()!=0)
    {
        is_colliding =  true;

        string objects = "";
        for (int i = 0; i < msg.collision_object.size(); ++i)
        {
            objects = objects + " " + msg.collision_object[i];
        }
        ROS_WARN("[%s] Collision detected with: %s", getLimb().c_str(), objects.c_str());
    }
    else is_colliding = false;
}

void RobotInterface::jointStatesCb(const sensor_msgs::JointState& msg)
{
    JointCommand joint_cmd;
    setJointNames(joint_cmd);

    if (msg.name.size() >= joint_cmd.names.size())
    {
        pthread_mutex_lock(&_mutex_jnts);
        _curr_jnts.name.clear();
        _curr_jnts.position.clear();
        for (int i = 0; i < joint_cmd.names.size(); ++i)
        {
            for (int j = 0; j < msg.name.size(); ++j)
            {
                if (joint_cmd.names[i] == msg.name[j])
                {
                    _curr_jnts.name.push_back(msg.name[j]);
                    _curr_jnts.position.push_back(msg.position[j]);
                }
            }
        }
        pthread_mutex_unlock(&_mutex_jnts);
    }

    return;
}

void RobotInterface::cuffCb(const baxter_core_msgs::DigitalIOState& msg)
{
    if (msg.state == baxter_core_msgs::DigitalIOState::PRESSED)
    {
        setState(KILLED);
    }
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
    double delta_time = ros::Time::now().toSec() - _filt_updated;

    std::vector<double> new_filt;
    std::vector<double> predicted_filt;

    new_filt[0] = (1 - FORCE_ALPHA) * _filt_force[0] + FORCE_ALPHA * _curr_wrench.force.x;
    new_filt[1] = (1 - FORCE_ALPHA) * _filt_force[1] + FORCE_ALPHA * _curr_wrench.force.y;
    new_filt[2] = (1 - FORCE_ALPHA) * _filt_force[2] + FORCE_ALPHA * _curr_wrench.force.z;

    // if the predicted force is not essentially 0, use it to calculate the
    // percentage difference between the predicted and the actual filter
    // if the predicted force is essentially 0, this is most likely the first
    // calculation, so just set the filter to the new filter value

    for (int i = 0; i < 3; ++i)
    {
        predicted_filt[i] = _filt_force[i] + (_filt_change[i] * delta_time);
        _filt_change[i] = (new_filt[i] - _filt_force[i])/delta_time;
        if (predicted_filt[i] > exp (-6))
        {
            if (abs((new_filt[i] - predicted_filt[i])/predicted_filt[i]) < filt_variance)
            {
                _filt_force[i] = new_filt[i];
            }
        }
        else
        {
            _filt_force[i] = new_filt[i];
        }
    }

    _filt_updated = ros::Time::now().toSec();

}

void RobotInterface::hoverAboveTokens(double height)
{
    goToPose(0.540, 0.570, height, VERTICAL_ORI_L);
}

bool RobotInterface::goToPoseNoCheck(double px, double py, double pz,
                                     double ox, double oy, double oz, double ow)
{
    vector<double> joint_angles;
    if (!computeIK(px, py, pz, ox, oy, oz, ow, joint_angles)) return false;

    return goToPoseNoCheck(joint_angles);
}

bool RobotInterface::goToPoseNoCheck(vector<double> joint_angles)
{
    JointCommand joint_cmd;
    joint_cmd.mode = JointCommand::POSITION_MODE;

    setJointNames(joint_cmd);

    for (int i = 0; i < joint_angles.size(); i++)
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
    if (!computeIK(px, py, pz, ox, oy, oz, ow, joint_angles)) return false;

    ros::Rate r(100);
    while (RobotInterface::ok())
    {
        if (disable_coll_av)
        {
            suppressCollisionAv();
        }
        else
        {
            if (is_colliding == true)
            {
                ROS_ERROR("Collision Occurred! Stopping.");
                return false;
            }
        }

        if (!goToPoseNoCheck(joint_angles))   return false;

        if (isPoseReached(px, py, pz, ox, oy, oz, ow, mode))
        {
            return true;
        }

        r.sleep();
    }

    return false;
}

bool RobotInterface::computeIK(double px, double py, double pz,
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
    float thresh_z = pose_stamp.pose.position.z + 0.012;

    while (!got_solution)
    {
        IK_call ik;

        pose_stamp.header.stamp=ros::Time::now();
        ik.req.pose_stamp  = pose_stamp;

        pthread_mutex_lock(&_mutex_jnts);
        ik.req.seed_angles = _curr_jnts;
        pthread_mutex_unlock(&_mutex_jnts);

        int cnt = 0;
        ros::Time tn = ros::Time::now();
        if (ik_solver.perform_ik(ik))
        {
            double te  = ros::Time::now().toSec()-tn.toSec();
            if (te>0.010)
            {
                ROS_ERROR("\t\t\tTime elapsed in computeIK: %g cnt %i",te,cnt);
            }
            cnt++;
            got_solution = ik.res.isValid;

            if (got_solution)
            {
                ROS_DEBUG("Got solution!");
                joint_angles = ik.res.joints.position;
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
                pose_stamp.pose.position.z += 0.001;
            }
        }

        // if no solution is found within 50 milliseconds or no solution within the acceptable
        // z-coordinate threshold is found, then no solution exists and exit oufof loop
        if ((ros::Time::now() - start).toSec() > 0.05 || pose_stamp.pose.position.z > thresh_z)
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
    else if (mode ==  "loose") thres = 0.067;

    if (_curr_range <= _curr_max_range &&
       _curr_range >= _curr_min_range &&
       _curr_range <= thres) return true;
    else return false;
}

bool RobotInterface::isPoseReached(double px, double py, double pz,
                                   double ox, double oy, double oz, double ow, string mode)
{
    if (!isPositionReached(px, py, pz, mode))         return false;
    if (!isOrientationReached(ox, oy, oz, ow, mode))  return false;

    return true;
}

bool RobotInterface::isPositionReached(double px, double py, double pz, string mode)
{
    ROS_DEBUG("[%s] Checking %s position. Error: %g %g %g", getLimb().c_str(),
                   mode.c_str(), px-getPos().x, py-getPos().y, pz-getPos().z);

    if (mode == "strict")
    {
        if (abs(getPos().x-px) > 0.005) return false;
        if (abs(getPos().y-py) > 0.005) return false;
        if (abs(getPos().z-pz) > 0.005) return false;
    }
    else if (mode == "loose")
    {
        if (abs(getPos().x-px) > 0.01) return false;
        if (abs(getPos().y-py) > 0.01) return false;
        if (abs(getPos().z-pz) > 0.01) return false;
    }
    else
    {
        ROS_ERROR("[%s] Please specify a mode of operation!", getLimb().c_str());
        return false;
    }

    return true;
}

bool RobotInterface::isOrientationReached(double ox, double oy, double oz, double ow, string mode)
{
    tf::Quaternion des(ox,oy,oz,ow);
    tf::Quaternion cur;
    tf::quaternionMsgToTF(getOri(), cur);

    ROS_DEBUG("[%s] Checking    orientation. Current %g %g %g %g Desired %g %g %g %g Dot %g",
                           getLimb().c_str(), getOri().x, getOri().y, getOri().z, getOri().w,
                                                                  ox,oy,oz,ow, des.dot(cur));

    if (abs(des.dot(cur)) < 0.985)  return false;

    return true;
}

bool RobotInterface::isConfigurationReached(baxter_core_msgs::JointCommand joint_cmd, std::string mode)
{
    if (_curr_jnts.position.size() < 7)
    {
        return false;
    }

    ROS_DEBUG("[%s] Checking configuration: Current %g %g %g %g %g %g %g\tDesired %g %g %g %g %g %g %g",
                                                                                      getLimb().c_str(),
         _curr_jnts.position[0], _curr_jnts.position[1], _curr_jnts.position[2], _curr_jnts.position[3],
                                 _curr_jnts.position[4], _curr_jnts.position[5], _curr_jnts.position[6],
                 joint_cmd.command[0], joint_cmd.command[1], joint_cmd.command[2], joint_cmd.command[3],
                                       joint_cmd.command[4], joint_cmd.command[5], joint_cmd.command[6]);

    bool result = false;
    for (int i = 0; i < joint_cmd.names.size(); ++i)
    {
        bool res = false;
        for (int j = 0; j < _curr_jnts.name.size(); ++j)
        {
            if (joint_cmd.names[i] == _curr_jnts.name[j])
            {
                if (mode == "strict")
                {
                    // It's approximatively half a degree
                    if (abs(joint_cmd.command[i]-_curr_jnts.position[j]) > 0.010) return false;
                }
                else if (mode == "loose")
                {
                    // It's approximatively a degree
                    if (abs(joint_cmd.command[i]-_curr_jnts.position[j]) > 0.020) return false;
                }
                res = true;
            }
        }
        if (res == false)   return false;
    }

    return true;
}

void RobotInterface::setJointNames(JointCommand& joint_cmd)
{
    joint_cmd.names.push_back(getLimb() + "_s0");
    joint_cmd.names.push_back(getLimb() + "_s1");
    joint_cmd.names.push_back(getLimb() + "_e0");
    joint_cmd.names.push_back(getLimb() + "_e1");
    joint_cmd.names.push_back(getLimb() + "_w0");
    joint_cmd.names.push_back(getLimb() + "_w1");
    joint_cmd.names.push_back(getLimb() + "_w2");
}

void RobotInterface::setJointCommands(double s0, double s1, double e0, double e1,
                                                 double w0, double w1, double w2,
                                      baxter_core_msgs::JointCommand& joint_cmd)
{
    joint_cmd.command.push_back(s0);
    joint_cmd.command.push_back(s1);
    joint_cmd.command.push_back(e0);
    joint_cmd.command.push_back(e1);
    joint_cmd.command.push_back(w0);
    joint_cmd.command.push_back(w1);
    joint_cmd.command.push_back(w2);
}

bool RobotInterface::detectForceInteraction()
{
    ROS_INFO("Filt Forces: %g, %g, %g", _filt_force[0], _filt_force[1], _filt_force[2]);
    // compare the current force to the filter force. if the percent difference is above a certain
    // value, return true
    // the following is still under construction (by Sarah)

    double small_thres = exp (-5);

    std::vector<double> curr_force;
    curr_force[0] = _curr_wrench.force.x;
    curr_force[1] = _curr_wrench.force.y;
    curr_force[2] = _curr_wrench.force.z;

    std::vector<double> curr_diff;

    for (int i = 0; i < 3; ++i)
    {
        if (abs(_filt_force[i]) > small_thres)
        {
            curr_diff[i] = abs((curr_force[i] - _filt_force[i])/_filt_force[i]);
        }
        else
        {
            curr_diff[i] = abs((curr_force[i] - _filt_force[i])/(abs(_filt_force[i]) + 0.01));
        }
        if (curr_diff[i] > rel_force_thres)
        {
            ROS_INFO("Interaction: %g %g %g", curr_force[0], curr_force[1], curr_force[2]);
            ROS_INFO("Difference relative to filter of force element %i: %g", i, curr_diff[i]);
            return true;
        }
        else
        {
            ROS_INFO("Difference relative to filter of force element %i: %g", i, curr_diff[i]);
            return false;
        }
    }
}

bool RobotInterface::waitForForceInteraction(double _wait_time, bool disable_coll_av)
{
    ros::Time _init = ros::Time::now();

    ros::Rate r(100);
    while (RobotInterface::ok())
    {
        if (disable_coll_av)          suppressCollisionAv();
        if (detectForceInteraction())           return true;

        r.sleep();

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

RobotInterface::~RobotInterface()
{
    pthread_mutex_destroy(&_mutex_jnts);
}

/**************************************************************************/
/*                          ROSThreadImage                                */
/**************************************************************************/

ROSThreadImage::ROSThreadImage(string name, string limb): _img_trp(_n), RobotInterface(name, limb)
{
    _img_sub = _img_trp.subscribe("/cameras/"+getLimb()+"_hand_camera/image",
                           SUBSCRIBER_BUFFER, &ROSThreadImage::imageCb, this);
    pthread_mutex_init(&_mutex_img, NULL);
}

ROSThreadImage::~ROSThreadImage()
{
    pthread_mutex_destroy(&_mutex_img);
}

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

