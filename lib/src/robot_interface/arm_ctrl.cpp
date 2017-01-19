#include "robot_interface/arm_ctrl.h"
#include <pthread.h>

using namespace std;
using namespace geometry_msgs;
using namespace baxter_core_msgs;

ArmCtrl::ArmCtrl(string _name, string _limb, bool _no_robot, bool _use_forces, bool _use_trac_ik, bool _use_cart_ctrl) :
                 RobotInterface(_name,_limb, _no_robot, _use_forces, _use_trac_ik, _use_cart_ctrl),
                 Gripper(_limb, _no_robot), sub_state(""), action(""), object_id(-1)
{
    std::string topic = "/"+getName()+"/state_"+_limb;
    state_pub = _n.advertise<baxter_collaboration::ArmState>(topic,1);
    ROS_INFO("[%s] Created state publisher with name : %s", getLimb().c_str(), topic.c_str());

    std::string other_limb = getLimb() == "right" ? "left" : "right";

    topic = "/"+getName()+"/service_"+_limb;
    service = _n.advertiseService(topic, &ArmCtrl::serviceCb, this);
    ROS_INFO("[%s] Created service server with name  : %s", getLimb().c_str(), topic.c_str());

    topic = "/"+getName()+"/service_"+_limb+"_to_"+other_limb;
    service_other_limb = _n.advertiseService(topic, &ArmCtrl::serviceOtherLimbCb,this);
    ROS_INFO("[%s] Created service server with name  : %s", getLimb().c_str(), topic.c_str());

    insertAction(ACTION_HOME,    &ArmCtrl::goHome);
    insertAction(ACTION_RELEASE, &ArmCtrl::releaseObject);

    // insertAction("recover_"+string(ACTION_HOME),    &ArmCtrl::notImplemented);
    // insertAction("recover_"+string(ACTION_RELEASE), &ArmCtrl::notImplemented);

    _n.param<bool>("internal_recovery",  internal_recovery, true);
    ROS_INFO("[%s] Internal_recovery flag set to %s", getLimb().c_str(),
                                internal_recovery==true?"true":"false");
}

void ArmCtrl::InternalThreadEntry()
{
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
    _n.param<bool>("internal_recovery",  internal_recovery, true);

    std::string a =     getAction();
    int         s = int(getState());

    setState(WORKING);

    if (is_no_robot())
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

    closeInternalThread();
    return;
}

bool ArmCtrl::serviceOtherLimbCb(baxter_collaboration::AskFeedback::Request  &req,
                                 baxter_collaboration::AskFeedback::Response &res)
{
    res.success = false;
    res.reply   = "not implemented";
    return true;
}

bool ArmCtrl::serviceCb(baxter_collaboration::DoAction::Request  &req,
                        baxter_collaboration::DoAction::Response &res)
{
    string action = req.action;
    int    obj    = req.object;

    ROS_INFO("[%s] Service request received. Action: %s object: %i", getLimb().c_str(),
                                                                   action.c_str(), obj);

    if (action == PROT_ACTION_LIST)
    {
        printActionDB();
        res.success  = true;
        res.response = actionDBToString();
        return true;
    }

    res.success = false;

    setAction(action);
    setObjectID(obj);

    startInternalThread();
    ros::Duration(0.5).sleep();

    ros::Rate r(100);
    while( ros::ok() && ( int(getState()) != START   &&
                          int(getState()) != ERROR   &&
                          int(getState()) != DONE      ))
    {
        if (ros::isShuttingDown())
        {
            setState(KILLED);
            return true;
        }

        if (getState()==KILLED)
        {
            recoverFromError();
        }

        r.sleep();
    }

    if ( int(getState()) == START   ||
         int(getState()) == DONE      )
    {
        res.success = true;
    }

    ROS_INFO("[%s] Service reply with success: %s\n", getLimb().c_str(),
                                            res.success?"true":"false");
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
        res = res && insertObject(static_cast<int>(i->second), i->first.c_str());
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

string ArmCtrl::getObjectName(int id)
{
    if (isObjectInDB(id))
    {
        return object_db[id];
    }

    return "";
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

void ArmCtrl::printObjectDB()
{
    ROS_INFO("[%s] Available objects in the database : %s",
              getLimb().c_str(), objectDBToString().c_str());
}

string ArmCtrl::objectDBToString()
{
    string res = "";
    map<int, string>::iterator it;

    for ( it = object_db.begin(); it != object_db.end(); it++ )
    {
        res = res + "[" + intToString(it->first) + "] " + it->second + ", ";
    }
    res = res.substr(0, res.size()-2); // Remove the last ", "
    return res;
}

bool ArmCtrl::insertAction(const std::string &a, ArmCtrl::f_action f)
{
    if (a == PROT_ACTION_LIST)
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
        ROS_ERROR("[%s] Invalid Action %s in state %i", getLimb().c_str(), a.c_str(), s);
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

    for ( it = action_db.begin(); it != action_db.end(); it++ )
    {
        res = res + it->first + ", ";
    }
    res = res.substr(0, res.size()-2); // Remove the last ", "
    return res;
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
    while(RobotInterface::ok() && !isPositionReached(p_f, mode))
    {
        if (disable_coll_av)    suppressCollisionAv();

        double t_elap = (ros::Time::now() - t_start).toSec();
        p_c = p_s;

        if (!finish)
        {
            if (dir == "backward" || dir == "forward")
            {
                int sgn = dir=="backward"?-1:+1;
                p_c.x = p_c.x + sgn * ARM_SPEED * t_elap;

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
                p_c.y = p_c.y + sgn * ARM_SPEED * t_elap;

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
                p_c.z = p_c.z + sgn * ARM_SPEED * t_elap;

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

bool ArmCtrl::hoverAboveTable(double height, string mode, bool disable_coll_av)
{
    if (getLimb() == "right")
    {
        return RobotInterface::goToPose(HOME_POS_R, height, VERTICAL_ORI_R,
                                                     mode, disable_coll_av);
    }
    else if (getLimb() == "left")
    {
        return RobotInterface::goToPose(HOME_POS_L, height, VERTICAL_ORI_L,
                                                     mode, disable_coll_av);
    }
    else return false;
}

bool ArmCtrl::homePoseStrict(bool disable_coll_av)
{
    ROS_INFO("[%s] Going to home position strict..", getLimb().c_str());

    ros::Rate r(100);
    while(RobotInterface::ok() && !isConfigurationReached(home_conf))
    {
        if (disable_coll_av)    suppressCollisionAv();

        goToJointConfNoCheck(home_conf);

        r.sleep();
    }

    return true;
}

void ArmCtrl::setHomeConf(double s0, double s1, double e0, double e1,
                                     double w0, double w1, double w2)
{
    home_conf.clear();
    home_conf.push_back(s0);
    home_conf.push_back(s1);
    home_conf.push_back(e0);
    home_conf.push_back(e1);
    home_conf.push_back(w0);
    home_conf.push_back(w1);
    home_conf.push_back(w2);

    return;
}

bool ArmCtrl::goHome()
{
    bool res = homePoseStrict();
    releaseObject();
    return res;
}

void ArmCtrl::recoverFromError()
{
    if (internal_recovery == true)
    {
        goHome();
    }
}

void ArmCtrl::setState(int _state)
{
    if (_state == KILLED && getState() != WORKING)
    {
        ROS_WARN("[%s] Attempted to kill a non-working controller", getLimb().c_str());
        return;
    }

    RobotInterface::setState(_state);

    // if (_state == DONE)
    // {
    //     setAction("");
    //     setMarkerID(-1);
    // }
    if (_state == DONE)
    {
        setSubState(getAction());
    }
    publishState();
}

void ArmCtrl::setAction(string _action)
{
    action = _action;
    publishState();
}

void ArmCtrl::publishState()
{
    baxter_collaboration::ArmState msg;

    msg.state  = string(getState());
    msg.action = getAction();
    msg.object = getObjectName(getObjectID());

    state_pub.publish(msg);
}

ArmCtrl::~ArmCtrl()
{
    killInternalThread();
}
