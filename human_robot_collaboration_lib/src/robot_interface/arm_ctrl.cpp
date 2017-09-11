#include "robot_interface/arm_ctrl.h"

using namespace std;
using namespace geometry_msgs;
using namespace baxter_core_msgs;

ArmCtrl::ArmCtrl(string _name, string _limb, bool _use_robot, bool _use_forces, bool _use_trac_ik, bool _use_cart_ctrl) :
                 RobotInterface(_name,_limb, _use_robot, THREAD_FREQ, _use_forces, _use_trac_ik, _use_cart_ctrl),
                 Gripper(_limb, _use_robot), sub_state(""), action(""), prev_action(""), sel_object_id(-1),
                 home_conf(7), arm_speed(ARM_SPEED), cuff_button_pressed(false)
{
    std::string other_limb = getLimb() == "right" ? "left" : "right";

    std::string topic = "/"+getName()+"/service_"+_limb;
    service = nh.advertiseService(topic, &ArmCtrl::serviceCb, this);
    ROS_INFO("[%s] Created service server with name  : %s", getLimb().c_str(), topic.c_str());

    topic = "/"+getName()+"/service_"+_limb+"_to_"+other_limb;
    service_other_limb = nh.advertiseService(topic, &ArmCtrl::serviceOtherLimbCb, this);
    ROS_INFO("[%s] Created service server with name  : %s", getLimb().c_str(), topic.c_str());

    insertAction(ACTION_HOME,    &ArmCtrl::goHome);
    insertAction(ACTION_RELEASE, &ArmCtrl::openImpl);

    nh.param<bool>("internal_recovery",  internal_recovery, true);
    ROS_INFO("[%s] Internal_recovery flag set to %s", getLimb().c_str(),
                                internal_recovery==true?"true":"false");
}

bool ArmCtrl::startThread()
{
    // This allows to call multiple threads within the same thread object.
    // As written in http://en.cppreference.com/w/cpp/thread/thread/joinable :
    //      A thread that has finished executing code, but has not yet been joined is still considered
    //      an active thread of execution and is therefore joinable.
    // So, we need to join the thread in order to spun out a new one anyways.
    if (arm_thread.joinable())    { arm_thread.join(); };

    arm_thread = std::thread(&ArmCtrl::InternalThreadEntry, this);
    return arm_thread.joinable();
}

void ArmCtrl::InternalThreadEntry()
{
    nh.param<bool>("internal_recovery",  internal_recovery, true);

    std::string a =     getAction();
    int         s = int(getState());

    setState(WORKING);

    if (not isRobotUsed())
    {
        ros::Duration(2.0).sleep();
        setState(DONE);
    }
    else if (a == ACTION_HOME || a == ACTION_RELEASE)
    {
        if (callAction(a))   setState(DONE);
    }
    else if (s == START || s == ERROR ||
             s == DONE  || s == KILLED )
    {
        if (doAction(s, a))   setState(DONE);
        else                  setState(ERROR);
    }
    else
    {
        ROS_ERROR("[%s] Invalid Action %s in state %i", getLimb().c_str(), a.c_str(), s);
    }

    if (int(getState())==WORKING)
    {
        setState(ERROR);
    }

    if (int(getState())==ERROR)
    {
        ROS_ERROR("[%s] Action %s not successful! State %s %s", getLimb().c_str(), a.c_str(),
                                          string(getState()).c_str(), getSubState().c_str());
    }

    return;
}

bool ArmCtrl::serviceOtherLimbCb(human_robot_collaboration_msgs::AskFeedback::Request  &req,
                                 human_robot_collaboration_msgs::AskFeedback::Response &res)
{
    res.success = false;
    res.reply   = "not implemented";
    return true;
}

bool ArmCtrl::serviceCb(human_robot_collaboration_msgs::DoAction::Request  &req,
                        human_robot_collaboration_msgs::DoAction::Response &res)
{
    // Let's read the requested action and object to act upon
    setSubState("");
    object_ids.clear();
    setObjectID(-1);

    string action = req.action;
    std::vector<int> obj_ids;
    std::string objs_str = "";

    for (size_t i = 0; i < req.objects.size(); ++i)
    {
        obj_ids.push_back(req.objects[i]);
        objs_str += toString(req.objects[i]) + ", ";
    }
    objs_str = objs_str.substr(0, objs_str.size()-2); // Remove the last ", "

    ROS_INFO_COND(print_level>=1, "[%s] Service request received. Action: %s Objects: %s",
                                     getLimb().c_str(), action.c_str(), objs_str.c_str());

    // Print the action or object DB if requested by the user
    if      (action == LIST_ACTIONS)
    {
        printActionDB();
        res.success  = true;
        res.response = actionDBToString();
        return true;
    }
    else if (action == LIST_OBJECTS)
    {
        printObjectDB();
        res.success  = true;
        res.response = objectDBToString();
        return true;
    }

    res.success = false;

    setAction(action);

    if (action != ACTION_HOME && action != ACTION_RELEASE && action != ACTION_HOLD &&
        action != std::string(ACTION_HOLD) +   "_leg" && action != std::string(ACTION_HOLD) + "_top" &&
        action != "start_" + std::string(ACTION_HOLD) && action != "end_" + std::string(ACTION_HOLD))
    {
        setObjectIDs(areObjectsInDB(obj_ids));

        if      (getObjectIDs().size() == 0)
        {
            res.response = OBJ_NOT_IN_DB;
            ROS_ERROR("[%s] Requested object(s) are not in the database! Action %s",
                                                 getLimb().c_str(), action.c_str());
            return true;
        }
        else if (getObjectIDs().size() == 1)
        {
            setObjectID(getObjectIDs()[0]);
            // ROS_INFO("I will perform action %s on object with ID %i",
            //                           action.c_str(), getObjectID());
        }
        else if (getObjectIDs().size() >  1)
        {
            setObjectID(chooseObjectID(getObjectIDs()));
        }
    }
    else if (action == ACTION_HOLD || action == std::string(ACTION_HOLD) + "_leg" ||
                                      action == std::string(ACTION_HOLD) + "_top"   )
    {
        setObjectIDs(getObjectIDs());
    }

    startThread();

    // This is there for the current thread to avoid overlapping
    // with the internal thread that just started
    ros::Duration(0.5).sleep();

    ros::Rate r(THREAD_FREQ);
    while( ros::ok() && ( int(getState()) != START   &&
                          int(getState()) != ERROR   &&
                          int(getState()) != DONE      ))
    {
        if (ros::isShuttingDown())
        {
            setState(KILLED);
            return true;
        }

        if (getState() == KILLED)
        {
            res.response = ACT_FAILED;
            recoverFromError();
        }

        r.sleep();
    }

    if ( int(getState()) == START   ||
         int(getState()) == DONE      )
    {
        res.success = true;
    }

    if (getState() == ERROR)
    {
        res.response = getSubState();
    }

    ROS_INFO_COND(print_level>=1, "[%s] Service reply with success: %s\n",
                           getLimb().c_str(), res.success?"true":"false");
    return true;
}

bool ArmCtrl::notImplemented()
{
    ROS_ERROR("[%s] Action not implemented!", getLimb().c_str());
    return false;
}

bool ArmCtrl::insertObject(int id, const std::string &n)
{
    if (isObjectInDB(id))
    {
        ROS_WARN("[%s][object_db] Overwriting existing object %i with name %s",
                 getLimb().c_str(), id, n.c_str());
    }

    object_db.insert( std::make_pair( id, n ));
    return true;
}

bool ArmCtrl::insertObjects(XmlRpc::XmlRpcValue _params)
{
    ROS_ASSERT(_params.getType()==XmlRpc::XmlRpcValue::TypeStruct);

    bool res = true;

    for (XmlRpc::XmlRpcValue::iterator i=_params.begin(); i!=_params.end(); ++i)
    {
        ROS_ASSERT(i->second.getType()==XmlRpc::XmlRpcValue::TypeInt);
        res = res & insertObject(static_cast<int>(i->second), i->first.c_str());
    }

    return res;
}

bool ArmCtrl::removeObject(int id)
{
    if (isObjectInDB(id))
    {
        object_db.erase(id);
        return true;
    }

    return false;
}

string ArmCtrl::getObjectNameFromDB(int id)
{
    if (isObjectInDB(id))
    {
        return object_db[id];
    }

    return "";
}

int ArmCtrl::getObjectIDFromDB(string _name)
{
    for( map<int, string>::const_iterator it = object_db.begin(); it != object_db.end(); ++it )
    {
        if (_name == it->second) return it->first;
    }
    return -1;
}

bool ArmCtrl::isObjectInDB(int id)
{
    if (object_db.find(id) != object_db.end()) return true;

    // if (!insertAction)
    // {
    //     ROS_ERROR("[%s][object_db] Object %i is not in the database!",
    //               getLimb().c_str(), id);
    // }
    return false;
}

std::vector<int> ArmCtrl::areObjectsInDB(const std::vector<int> &_objs)
{
    std::vector<int> res;

    for (size_t i = 0; i < _objs.size(); ++i)
    {
        if (isObjectInDB(_objs[i]))
        {
            res.push_back(_objs[i]);
        }
    }

    ROS_INFO_COND(print_level>=2,"[%s] Found %lu objects in DB.",
                                  getLimb().c_str(), res.size());

    return res;
}

void ArmCtrl::printObjectDB()
{
    ROS_INFO("[%s] Available objects in the database : %s",
              getLimb().c_str(), objectDBToString().c_str());
}

string ArmCtrl::objectDBToString()
{
    string res = "";
    map<int, string>::iterator it;

    for ( it = object_db.begin(); it != object_db.end(); ++it )
    {
        res = res + "[" + toString(it->first) + "] " + it->second + ", ";
    }
    res = res.substr(0, res.size()-2); // Remove the last ", "
    return res;
}

bool ArmCtrl::insertAction(const std::string &a, ArmCtrl::f_action f)
{
    if (a == LIST_ACTIONS)
    {
        ROS_ERROR("[%s][action_db] Attempted to insert protected action key: %s",
                 getLimb().c_str(), a.c_str());
        return false;
    }

    if (isActionInDB(a, true)) // The action is in the db
    {
        ROS_WARN("[%s][action_db] Overwriting existing action with key %s",
                 getLimb().c_str(), a.c_str());
    }

    action_db.insert( std::make_pair( a, f ));
    return true;
}

bool ArmCtrl::removeAction(const std::string &a)
{
    if (isActionInDB(a)) // The action is in the db
    {
        action_db.erase(a);
        return true;
    }

    return false;
}

bool ArmCtrl::callAction(const std::string &a)
{
    if (isActionInDB(a)) // The action is in the db
    {
        f_action act = action_db[a];
        return (this->*act)();
    }

    return false;
}

bool ArmCtrl::doAction(int s, std::string a)
{
    if (isActionInDB(a))
    {
        if (callAction(a))         return true;
        else                recoverFromError();
    }
    else
    {
        setSubState(ACT_NOT_IN_DB);
        ROS_ERROR("[%s] Action %s in state %i is not in the database!",
                                      getLimb().c_str(), a.c_str(), s);
    }

    return false;
}

bool ArmCtrl::isActionInDB(const std::string &a, bool insertAction)
{
    if (action_db.find(a) != action_db.end()) return true;

    if (!insertAction)
    {
        ROS_ERROR("[%s][action_db] Action %s is not in the database!",
                  getLimb().c_str(), a.c_str());
    }
    return false;
}

void ArmCtrl::printActionDB()
{
    ROS_INFO("[%s] Available actions in the database : %s",
              getLimb().c_str(), actionDBToString().c_str());
}

string ArmCtrl::actionDBToString()
{
    string res = "";
    map<string, f_action>::iterator it;

    for ( it = action_db.begin(); it != action_db.end(); ++it )
    {
        res = res + it->first + ", ";
    }
    res = res.substr(0, res.size()-2); // Remove the last ", "
    return res;
}

void ArmCtrl::cuffUpperCb(const baxter_core_msgs::DigitalIOState& _msg)
{
    if (_msg.state == baxter_core_msgs::DigitalIOState::PRESSED)
    {
        cuff_button_pressed = true;
    }

    return;
}

bool ArmCtrl::waitForUserCuffUpperFb(double _wait_time)
{
    ROS_INFO_COND(print_level>=2, "[%s] Waiting user feedback for %g [s]",
                                           getLimb().c_str(), _wait_time);

    cuff_button_pressed = false;

    ros::Time _init = ros::Time::now();

    ros::Rate(THREAD_FREQ);
    while(RobotInterface::ok() && not isClosing())
    {
        if (cuff_button_pressed == true)        return true;

        if ((ros::Time::now()-_init).toSec() > _wait_time)
        {
            ROS_ERROR("No user feedback has been detected in %g [s]!",_wait_time);
            return false;
        }
    }

    return false;
}

bool ArmCtrl::moveArm(string dir, double dist, string mode, bool disable_coll_av)
{
    Point p_s = getPos();
    Point p_c = getPos();
    Point p_f = getPos();

    Quaternion o_f = getOri();

    if      (dir == "backward") p_f.x -= dist;
    else if (dir == "forward")  p_f.x += dist;
    else if (dir == "right")    p_f.y -= dist;
    else if (dir == "left")     p_f.y += dist;
    else if (dir == "down")     p_f.z -= dist;
    else if (dir == "up")       p_f.z += dist;
    else                         return false;

    ros::Time t_start = ros::Time::now();

    bool finish = false;

    ros::Rate r(100);
    while(RobotInterface::ok() && !isPositionReached(p_f, mode) && not isClosing())
    {
        if (disable_coll_av)    suppressCollisionAv();

        double t_elap = (ros::Time::now() - t_start).toSec();
        p_c = p_s;

        if (!finish)
        {
            if (dir == "backward" || dir == "forward")
            {
                int sgn = dir=="backward"?-1:+1;
                p_c.x = p_c.x + sgn * arm_speed * t_elap;

                if (dir == "backward")
                {
                    if (p_c.x < p_f.x) finish = true;
                }
                else if (dir == "forward")
                {
                    if (p_c.x > p_f.x) finish = true;
                }
            }
            if (dir == "right" || dir == "left")
            {
                int sgn = dir=="right"?-1:+1;
                p_c.y = p_c.y + sgn * arm_speed * t_elap;

                if (dir == "right")
                {
                    if (p_c.y < p_f.y) finish = true;
                }
                else if (dir == "left")
                {
                    if (p_c.y > p_f.y) finish = true;
                }
            }
            if (dir == "down" || dir == "up")
            {
                int sgn = dir=="down"?-1:+1;
                p_c.z = p_c.z + sgn * arm_speed * t_elap;

                if (dir == "down")
                {
                    if (p_c.z < p_f.z) finish = true;
                }
                else if (dir == "up")
                {
                    if (p_c.z > p_f.z) finish = true;
                }
            }
        }
        else
        {
            p_c = p_f;
        }

        if (!goToPoseNoCheck(p_c, o_f)) return false;

        r.sleep();
    }

    return true;
}

bool ArmCtrl::goToPose(double px, double py, double pz,
                       double ox, double oy, double oz, double ow,
                       std::string mode, bool disable_coll_av)
{
    bool res = RobotInterface::goToPose(px, py, pz,
                                        ox, oy, oz, ow, mode, disable_coll_av);

    if (res == false && getSubState() != ACT_FAILED)
    {
        setSubState(INV_KIN_FAILED);
    }

    return res;
}

bool ArmCtrl::hoverAboveTable(double _height, string _mode, bool _disable_coll_av)
{
    ROS_INFO_COND(print_level>=2, "[%s] Hovering above table..", getLimb().c_str());

    if (getLimb() == "right")
    {
        return goToPose(HOME_POS_R, _height, VERTICAL_ORI_R,
                                   _mode, _disable_coll_av);
    }
    else if (getLimb() == "left")
    {
        return goToPose(HOME_POS_L, _height, VERTICAL_ORI_L,
                                   _mode, _disable_coll_av);
    }
    else
    {
        return false;
    }
}

bool ArmCtrl::hoverAbovePool(string _mode, bool _disable_coll_av)
{
    ROS_INFO_COND(print_level>=2, "[%s] Hovering above pool..", getLimb().c_str());

    if (getLimb() == "right")
    {
        return goToPose( POOL_POS_R, POOL_ORI_R,
                        _mode, _disable_coll_av);
    }
    else if (getLimb() == "left")
    {
        return goToPose( POOL_POS_L, POOL_ORI_L,
                        _mode, _disable_coll_av);
    }
    else
    {
        return false;
    }
}

bool ArmCtrl::startHold()
{
    if (Gripper::type() == "suction")
    {
        return false;
    }

    double time=getObjectIDs().size()>=2?getObjectIDs()[0]:30.0;

    if (!goHoldPose())                  { return false; }
    ros::Duration(1.0).sleep();
    if (!waitForUserCuffUpperFb(time))  { return false; }
    if (!close())                       { return false; }
    ros::Duration(1.0).sleep();

    return true;
}

bool ArmCtrl::endHold()
{
    if (Gripper::type() == "suction")
    {
        return false;
    }

    double time=getObjectIDs().size()>=2?getObjectIDs()[1]:180.0;

    if (!waitForUserCuffUpperFb(time))  { return false; }
    if (!open())                        { return false; }
    ros::Duration(1.0).sleep();
    if (!homePoseStrict())              { return false; }

    return true;
}

bool ArmCtrl::holdObject()
{
    if (Gripper::type() == "suction")
    {
        return false;
    }

    if (!startHold()) { return false; }
    if (!endHold())   { return false; }

    return true;
}

bool ArmCtrl::goHoldPose()
{
    if (getLimb() == "left")
    {
        ROS_ERROR("[%s] %s action currently not implemented for %s arm!!",
                  getLimb().c_str(), getAction().c_str(), getLimb().c_str());

        return false;
    }

    ROS_INFO("[%s] Going to %s position..", getLimb().c_str(), getAction().c_str());

    if      (getAction() == string(ACTION_HOLD) + "_top")
    {
        return goToPose(0.72, -0.31, 0.032, 0.54, 0.75, 0.29, 0.22);
    }
    else if (getAction() == string(ACTION_HOLD) + "_leg")
    {
        return goToPose(0.80, -0.4, 0.3, HORIZONTAL_ORI_R);
    }

    return false;
}

bool ArmCtrl::homePoseStrict(bool disable_coll_av)
{
    ROS_INFO("[%s] Going to home position strict..", getLimb().c_str());

    ros::Rate r(100);
    while(RobotInterface::ok() && !isConfigurationReached(home_conf) && not isClosing())
    {
        if (disable_coll_av)    suppressCollisionAv();

        goToJointConfNoCheck(home_conf);

        r.sleep();
    }

    return true;
}

void ArmCtrl::setHomeConf(double _s0, double _s1, double _e0, double _e1,
                                      double _w0, double _w1, double _w2)
{
    home_conf << _s0, _s1, _e0, _e1, _w0, _w1, _w2;

    return;
}

void ArmCtrl::setHomeConfiguration(std::string _loc)
{
    if      (getLimb() == "left")
    {
        if      (_loc == "pool")
        {
            setHomeConf(0.7060, -1.2717, 0.3846,  1.5405,
                                -0.1273, 1.3135,  0.3206);
        }
        else if (_loc == "table")
        {
            setHomeConf(0.1967, -0.8702, -1.0531,  1.5578,
                                 0.6516,  1.2464, -0.1787);
        }
    }
    else if (getLimb() == "right")
    {
        if      (_loc == "pool")
        {
            setHomeConf(-1.6801, -1.0500, 1.1693, 1.9762,
                                 -0.5722, 1.0205, 0.5430);
        }
        else if (_loc == "table")
        {
            setHomeConf( 0.0717, -1.0009, 1.1083, 1.5520,
                                 -0.5235, 1.3468, 0.4464);
        }
    }
    return;
}

bool ArmCtrl::goHome()
{
    bool res = homePoseStrict();
    open();
    return res;
}

void ArmCtrl::recoverFromError()
{
    if (internal_recovery == true)
    {
        goHome();
    }
}

bool ArmCtrl::setState(int _state)
{
    ROS_DEBUG("[%s] Setting state to %i", getLimb().c_str(), _state);

    if (_state == KILLED && getState() != WORKING)
    {
        ROS_WARN_THROTTLE(2, "[%s] Attempted to kill a non-working controller", getLimb().c_str());
        return false;
    }

    if      (_state == DONE)
    {
        setSubState(getAction());
    }
    else if (_state == ERROR && (getSubState() == "" || getSubState() == CHECK_OBJ_IDS))
    {
        setSubState(ACT_FAILED);
    }

    return RobotInterface::setState(_state);
}

bool ArmCtrl::setArmSpeed(double _arm_speed)
{
    arm_speed = _arm_speed;

    return true;
}

void ArmCtrl::setSubState(const string& _sub_state)
{
    // ROS_DEBUG("[%s] Setting sub state to: %s", getLimb().c_str(), _sub_state.c_str());
    sub_state =  _sub_state;
}

bool ArmCtrl::setAction(const string& _action)
{
    setPrevAction(getAction());
    action = _action;
    publishState();

    return true;
}

bool ArmCtrl::setPrevAction(const string& _prev_action)
{
    prev_action = _prev_action;

    return true;
}

bool ArmCtrl::publishState()
{
    human_robot_collaboration_msgs::ArmState msg;

    msg.state  = string(getState());
    msg.action = getAction();
    msg.object = getObjectNameFromDB(getObjectID());

    state_pub.publish(msg);

    return true;
}

ArmCtrl::~ArmCtrl()
{
    setIsClosing(true);
    if (arm_thread.joinable()) { arm_thread.join(); }
}
