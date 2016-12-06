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
RobotInterface::RobotInterface(string name, string limb, bool no_robot, bool use_forces,
                               bool use_trac_ik, bool use_cart_ctrl) : _n(name), _name(name), _limb(limb),
                               _state(START), spinner(4), ir_ok(false), _no_robot(no_robot), is_coll_av_on(false),
                               is_ctrl_running(false), ik_solver(limb, no_robot), _use_forces(use_forces),
                               _use_trac_ik(use_trac_ik), _use_cart_ctrl(use_cart_ctrl)
{
    if (no_robot) return;

    pthread_mutexattr_t _mutex_attr;
    pthread_mutexattr_init(&_mutex_attr);
    pthread_mutexattr_settype(&_mutex_attr, PTHREAD_MUTEX_RECURSIVE_NP);

    pthread_mutex_init(&_mutex_jnts, &_mutex_attr);
    pthread_mutex_init(&_mutex_ctrl, &_mutex_attr);

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

    if (_use_cart_ctrl)
    {
        std::string topic = "/" + _name + "/" + _limb + "/go_to_pose";
        _ctrl_sub      = _n.subscribe(topic, SUBSCRIBER_BUFFER, &RobotInterface::ctrlMsgCb, this);
        ROS_INFO("[%s] Created cartesian controller that listens to : %s", getLimb().c_str(), topic.c_str());
    }

    if (!_use_trac_ik)
    {
        _ik_client = _n.serviceClient<SolvePositionIK>("/ExternalTools/" + _limb +
                                                       "/PositionKinematicsNode/IKService");
    }

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

    ROS_INFO("[%s] Cartesian Controller %s enabled", getLimb().c_str(), _use_cart_ctrl?"is":"is NOT");

    spinner.start();

    if (_use_cart_ctrl)     startThread();
}

bool RobotInterface::startThread()
{
    return _thread.start(ThreadEntryFunc, this);
}

bool RobotInterface::closeThread()
{
    return _thread.close();
}

bool RobotInterface::killThread()
{
    return _thread.kill();
}

void RobotInterface::ThreadEntry()
{
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    ros::Rate r(THREAD_FREQ);

    while (RobotInterface::ok())
    {
        // ROS_INFO("Time: %g", (ros::Time::now() - initTime).toSec());

        if (isCtrlRunning())
        {
            double time_elap = (ros::Time::now() - time_start).toSec();

            // Starting pose in terms of position and orientation
            geometry_msgs::Point      p_s =    pose_start.position;
            geometry_msgs::Quaternion o_s = pose_start.orientation;

            // Desired  pose in terms of position and orientation
            geometry_msgs::Point      p_d =      pose_des.position;
            geometry_msgs::Quaternion o_d =   pose_des.orientation;

            if (!isPoseReached(p_d, o_d, "strict"))
            {
                // Current pose to send to the IK solver.
                geometry_msgs::Pose pose_curr = pose_des;

                /* POSITIONAL PART */
                // We model the end effector as a 3D point that moves toward the
                // target with a straight trajectory and constant speed.
                geometry_msgs::Point p_c = p_s + (p_d - p_s) / norm(p_d - p_s) * ARM_SPEED * time_elap;

                // Check if the current position is overshooting the desired position
                // By checking the sign of the cosine of the angle between p_d-p_s and p_d-p_c
                // This would mean equal to 1 within some small epsilon (1e-8)
                if (dot(p_d-p_s, p_d-p_c)/(norm(p_d-p_s)*norm(p_d-p_c)) - 1 <  EPSILON &&
                    dot(p_d-p_s, p_d-p_c)/(norm(p_d-p_s)*norm(p_d-p_c)) - 1 > -EPSILON)
                {
                    pose_curr.position    = p_c;
                }

                /* ORIENTATIONAL PART */
                // We use a spherical linear interpolation between o_s and o_d. The speed of the interpolation
                // depends on ARM_ROT_SPEED, which is fixed and defined in utils.h
                tf::Quaternion o_d_TF, o_s_TF;
                tf::quaternionMsgToTF(o_d, o_d_TF);
                tf::quaternionMsgToTF(o_s, o_s_TF);
                double angle     = o_s_TF.angleShortestPath(o_d_TF);
                double traj_time =            angle / ARM_ROT_SPEED;

                if (time_elap < traj_time)
                {
                    tf::Quaternion o_c_TF = o_s_TF.slerp(o_d_TF, time_elap / traj_time);
                    o_c_TF.normalize();
                    geometry_msgs::Quaternion o_c;
                    tf::quaternionTFToMsg(o_c_TF, o_c);

                    pose_curr.orientation = o_c;
                }

                // ROS_INFO("[%s] Executing trajectory: time %g/%g pose_curr %s", getLimb.c_str(), time_elap,
                //                                                      traj_time, print(pose_curr).c_str());

                if (!goToPoseNoCheck(pose_curr)) ROS_WARN("[%s] desired configuration could not be reached.",
                                                                                          getLimb().c_str());

                if (hasCollided("strict")) ROS_INFO_THROTTLE(2, "[%s] is colliding!", getLimb().c_str());
            }
            else
            {
                ROS_INFO("[%s] Pose reached", getLimb().c_str());
                setCtrlRunning(false);
            }
        }

        r.sleep();
    }

    closeThread();
    return;
}

void * RobotInterface::ThreadEntryFunc(void * obj)
{
    ((RobotInterface *)obj)->ThreadEntry();
    return NULL;
}

bool RobotInterface::ok()
{
    bool res = ros::ok();
    res = res && getState() != KILLED && getState() != STOPPED;

    return res;
}

bool RobotInterface::getIKLimits(KDL::JntArray &ll, KDL::JntArray &ul)
{
    return ik_solver.getKDLLimits(ll,ul);
}

bool RobotInterface::setIKLimits(KDL::JntArray ll, KDL::JntArray ul)
{
    ik_solver.setKDLLimits(ll,ul);
    return true;
}

bool RobotInterface::initCtrlParams()
{
    time_start = ros::Time::now();
    pose_start = getPose();
}

void RobotInterface::ctrlMsgCb(const baxter_collaboration::GoToPose& msg)
{
    if (int(getState()) != WORKING)
    {
        pose_des.position = msg.pose_stamp.pose.position;

        if (msg.pose_stamp.pose.orientation.x != -100 &&
            msg.pose_stamp.pose.orientation.y != -100 &&
            msg.pose_stamp.pose.orientation.z != -100 &&
            msg.pose_stamp.pose.orientation.w != -100)
        {
            pose_des.orientation = msg.pose_stamp.pose.orientation;
        }
        else
        {
            pose_des.orientation = getOri();
        }

        ctrl_mode = msg.ctrl_mode;

        if (ctrl_mode != baxter_collaboration::GoToPose::POSITION_MODE )
        {
            ROS_WARN("As of now, the only accepted control mode is POSITION_MODE");
            ctrl_mode = baxter_collaboration::GoToPose::POSITION_MODE;
        }

        setCtrlRunning(true);
        initCtrlParams();

        ROS_INFO("[%s] Received new target pose: %s", getLimb().c_str(), print(pose_des).c_str());
    }
    else
    {
        ROS_ERROR_THROTTLE(1, "[%s] Received new target pose, but the controller is already"
                              " in use through the high level interface!", getLimb().c_str());
    }

    return;
}

void RobotInterface::setCtrlRunning(bool _flag)
{
    // ROS_INFO("[%s] Setting is_ctrl_running to: %i", getLimb().c_str(), _flag);

    pthread_mutex_lock(&_mutex_ctrl);
    is_ctrl_running = _flag;
    pthread_mutex_unlock(&_mutex_ctrl);

    return;
}

bool RobotInterface::isCtrlRunning()
{
    bool res;

    pthread_mutex_lock(&_mutex_ctrl);
    res = is_ctrl_running;
    pthread_mutex_unlock(&_mutex_ctrl);

    // ROS_INFO("[%s] is_ctrl_running equal to: %i", "left", res);

    return res;
}

void RobotInterface::collAvCb(const baxter_core_msgs::CollisionAvoidanceState& msg)
{
    if (msg.collision_object.size()!=0)
    {
        is_coll_av_on =  true;

        string objects = "";
        for (int i = 0; i < msg.collision_object.size(); ++i)
        {
            // Let's remove the first part of the collision object name for visualization
            // purposes, i.e. the part that says "collision_"
            objects = objects + " " + std::string(msg.collision_object[i]).erase(0,10);
        }
        ROS_WARN_THROTTLE(1, "[%s] Collision detected with: %s", getLimb().c_str(), objects.c_str());
    }
    else is_coll_av_on = false;

    return;
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

    return;
}

void RobotInterface::endpointCb(const baxter_core_msgs::EndpointState& msg)
{
    // ROS_DEBUG("endpointCb");
    _curr_pos      = msg.pose.position;
    _curr_ori      = msg.pose.orientation;

    if (_use_forces == true) _curr_wrench   = msg.wrench;

    tf::Quaternion _marker_quat;
    tf::quaternionMsgToTF(_curr_ori, _marker_quat);
    tf::Matrix3x3 _marker_mat(_marker_quat);

    if (_use_forces == true) filterForces();

    return;
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

    return;
}

void RobotInterface::filterForces()
{
    _filt_force[0] = (1 - FORCE_ALPHA) * _filt_force[0] + FORCE_ALPHA * _curr_wrench.force.x;
    _filt_force[1] = (1 - FORCE_ALPHA) * _filt_force[1] + FORCE_ALPHA * _curr_wrench.force.y;
    _filt_force[2] = (1 - FORCE_ALPHA) * _filt_force[2] + FORCE_ALPHA * _curr_wrench.force.z;

    return;
}

bool RobotInterface::goToPoseNoCheck(geometry_msgs::Pose p)
{
    return goToPoseNoCheck(p.position, p.orientation);
}

bool RobotInterface::goToPoseNoCheck(geometry_msgs::Point p, geometry_msgs::Quaternion o)
{
    return goToPoseNoCheck(p.x, p.y, p.z, o.x, o.y, o.z, o.w);
}

bool RobotInterface::goToPoseNoCheck(double px, double py, double pz,
                                     double ox, double oy, double oz, double ow)
{
    vector<double> joint_angles;
    if (!computeIK(px, py, pz, ox, oy, oz, ow, joint_angles)) return false;

    return goToJointConfNoCheck(joint_angles);
}

bool RobotInterface::goToJointConfNoCheck(vector<double> joint_angles)
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
            if (is_coll_av_on == true)
            {
                ROS_ERROR("Collision Occurred! Stopping.");
                return false;
            }
        }

        if (!goToJointConfNoCheck(joint_angles))   return false;

        if (isPoseReached(px, py, pz, ox, oy, oz, ow, mode))
        {
            return true;
        }

        r.sleep();
    }

    return false;
}

bool RobotInterface::computeIK(geometry_msgs::Pose p, std::vector<double>& j)
{
    return computeIK(p.position, p.orientation, j);
}

bool RobotInterface::computeIK(geometry_msgs::Point p, geometry_msgs::Quaternion o,
                               std::vector<double>& j)
{
    return computeIK(p.x, p.y, p.z, o.x, o.y, o.z, o.w, j);
}

bool RobotInterface::computeIK(double px, double py, double pz,
                               double ox, double oy, double oz, double ow,
                               std::vector<double>& j)
{
    PoseStamped pose_stamp;
    pose_stamp.header.frame_id = "base";
    pose_stamp.header.stamp    = ros::Time::now();

    setPosition(   pose_stamp.pose, px, py, pz);
    setOrientation(pose_stamp.pose, ox, oy, oz, ow);

    j.clear();
    bool got_solution = false;
    ros::Time start = ros::Time::now();
    float thresh_z = pose_stamp.pose.position.z + 0.01;

    while (!got_solution)
    {
        SolvePositionIK ik_srv;

        pose_stamp.header.stamp=ros::Time::now();

        //ik_srv.request.seed_mode=2;       // i.e. SEED_CURRENT
        ik_srv.request.seed_mode=0;         // i.e. SEED_AUTO

        ik_srv.request.pose_stamp.push_back(pose_stamp);
        pthread_mutex_lock(&_mutex_jnts);
        ik_srv.request.seed_angles.push_back(_curr_jnts);
        pthread_mutex_unlock(&_mutex_jnts);

        int cnt = 0;
        ros::Time tn = ros::Time::now();

        bool result = _use_trac_ik?ik_solver.perform_ik(ik_srv):_ik_client.call(ik_srv);

        if(result)
        {
            double te  = ros::Time::now().toSec()-tn.toSec();;
            if (te>0.010)
            {
                ROS_WARN_ONCE("\t\t\tTime elapsed in computing IK: %g cnt %i",te,cnt);
            }
            cnt++;

            if (ik_srv.response.isValid[0])
            {
                ROS_DEBUG("Got solution!");
                j = ik_srv.response.joints[0].position;
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
        // z-coordinate threshold is found, then no solution exists and exit out of loop
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
    double thres;

    if      (mode == "strict") thres = 0.050;
    else if (mode ==  "loose") thres = 0.067;

    if (_curr_range <= _curr_max_range &&
        _curr_range >= _curr_min_range &&
        _curr_range <= thres             ) return true;

    return false;
}

bool RobotInterface::isPoseReached(geometry_msgs::Pose p, std::string mode)
{
    return isPoseReached(p.position, p.orientation, mode);
}

bool RobotInterface::isPoseReached(geometry_msgs::Point p,
                                   geometry_msgs::Quaternion o, std::string mode)
{
    return isPoseReached(p.x, p.y, p.z, o.x, o.y, o.z, o.w, mode);
}

bool RobotInterface::isPoseReached(double px, double py, double pz,
                                   double ox, double oy, double oz, double ow, string mode)
{
    if (!   isPositionReached(px, py, pz,     mode))  return false;
    if (!isOrientationReached(ox, oy, oz, ow, mode))  return false;

    return true;
}

bool RobotInterface::isPositionReached(geometry_msgs::Point p, std::string mode)
{
    return isPositionReached(p.x, p.y, p.z, mode);
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
        if (abs(getPos().x-px) >  0.01) return false;
        if (abs(getPos().y-py) >  0.01) return false;
        if (abs(getPos().z-pz) >  0.01) return false;
    }
    else
    {
        ROS_ERROR("[%s] Please specify a mode of operation!", getLimb().c_str());
        return false;
    }

    return true;
}

bool RobotInterface::isOrientationReached(geometry_msgs::Quaternion q, std::string mode)
{
    return isOrientationReached(q.x, q.y, q.z, q.w, mode);
}

bool RobotInterface::isOrientationReached(double ox, double oy, double oz, double ow, string mode)
{
    tf::Quaternion des(ox, oy, oz, ow);
    tf::Quaternion cur;
    tf::quaternionMsgToTF(getOri(), cur);

    ROS_DEBUG("[%s] Checking    orientation. Current %g %g %g %g Desired %g %g %g %g Dot %g",
                           getLimb().c_str(), getOri().x, getOri().y, getOri().z, getOri().w,
                                                               ox, oy, oz, ow, des.dot(cur));

    if (abs(des.dot(cur)) < 0.985)  return false;

    return true;
}

bool RobotInterface::isConfigurationReached(std::vector<double> des_jnts, std::string mode)
{
    if (_curr_jnts.position.size() < 7 || des_jnts.size() < 7)
    {
        return false;
    }

    baxter_core_msgs::JointCommand dj;
    setJointNames(dj);
    setJointCommands(des_jnts[0], des_jnts[1], des_jnts[2],
                     des_jnts[3], des_jnts[4], des_jnts[5], des_jnts[6], dj);

    return isConfigurationReached(dj, mode);
}

bool RobotInterface::isConfigurationReached(baxter_core_msgs::JointCommand des_jnts, std::string mode)
{
    if (_curr_jnts.position.size() < 7)
    {
        return false;
    }

    ROS_DEBUG("[%s] Checking configuration: Current %g %g %g %g %g %g %g\tDesired %g %g %g %g %g %g %g",
                                                                                      getLimb().c_str(),
         _curr_jnts.position[0], _curr_jnts.position[1], _curr_jnts.position[2], _curr_jnts.position[3],
                                 _curr_jnts.position[4], _curr_jnts.position[5], _curr_jnts.position[6],
            des_jnts.command[0],    des_jnts.command[1],    des_jnts.command[2],    des_jnts.command[3],
                                    des_jnts.command[4],    des_jnts.command[5],    des_jnts.command[6]);

    for (int i = 0; i < des_jnts.names.size(); ++i)
    {
        bool res = false;
        for (int j = 0; j < _curr_jnts.name.size(); ++j)
        {
            if (des_jnts.names[i] == _curr_jnts.name[j])
            {
                if (mode == "strict")
                {
                    // It's approximatively half a degree
                    if (abs(des_jnts.command[i]-_curr_jnts.position[j]) > 0.010) return false;
                }
                else if (mode == "loose")
                {
                    // It's approximatively a degree
                    if (abs(des_jnts.command[i]-_curr_jnts.position[j]) > 0.020) return false;
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

geometry_msgs::Pose RobotInterface::getPose()
{
    geometry_msgs::Pose res;

    res.position    = getPos();
    res.orientation = getOri();

    return res;
}

void RobotInterface::setState(int state)
{
    _state.set(state);

    // disable the cartesian controller server
    if (state == WORKING)
    {
        setCtrlRunning(false);
    }

    return;
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
    pthread_mutex_destroy(&_mutex_ctrl);

    _thread.kill();
}
