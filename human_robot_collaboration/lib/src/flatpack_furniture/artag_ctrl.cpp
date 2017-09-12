#include "flatpack_furniture/artag_ctrl.h"

#include <tf/transform_datatypes.h>

using namespace std;
using namespace human_robot_collaboration_msgs;

ARTagCtrl::ARTagCtrl(std::string _name, std::string _limb, bool _use_robot) :
                     ArmPerceptionCtrl(_name,_limb, _use_robot)
{
    setHomeConfiguration();
    setState(START);

    insertAction(ACTION_GET,       static_cast<f_action>(&ARTagCtrl::getObject));
    insertAction(ACTION_PASS,      static_cast<f_action>(&ARTagCtrl::passObject));
    insertAction(ACTION_GET_PASS,  static_cast<f_action>(&ARTagCtrl::getPassObject));

    XmlRpc::XmlRpcValue objects_db;
    if(!nh.getParam("objects_"+getLimb(), objects_db))
    {
        ROS_INFO("No objects' database found in the parameter server. "
                 "Looked up param is %s", ("objects_"+getLimb()).c_str());
    }
    else
    {
        insertObjects(objects_db);
        printObjectDB();
    }

    if (not _use_robot) return;

    // moveArm("up",0.2,"strict");
    // moveArm("down",0.2,"strict");
    // moveArm("right",0.2,"strict");
    // moveArm("left",0.2,"strict");
    // moveArm("forward",0.1,"strict");
    // moveArm("backward",0.2,"strict");
    // moveArm("forward",0.1,"strict");
}

bool ARTagCtrl::handOver()
{
    if (getPrevAction() != ACTION_GET)  return false;
    if (!prepare4HandOver())            return false;
    ros::Duration(0.2).sleep();
    if (!waitForOtherArm(30.0, true))   return false;
    ros::Duration(0.8).sleep();
    if (!open())                        return false;
    if (!moveArm("up", 0.05))           return false;
    if (!homePoseStrict())              return false;
    setSubState("");

    return true;
}

bool ARTagCtrl::prepare4HandOver()
{
    return moveArm("right", 0.32, "loose", true);
}

bool ARTagCtrl::waitForOtherArm(double _wait_time, bool disable_coll_av)
{
    ros::Time _init = ros::Time::now();

    string other_limb = getLimb() == "right" ? "left" : "right";

    ROS_INFO("[%s] Waiting for %s arm", getLimb().c_str(), other_limb.c_str());

    ros::ServiceClient _c;
    string service_name = "/"+getName()+"/service_"+other_limb+"_to_"+getLimb();
    _c = nh.serviceClient<AskFeedback>(service_name);

    AskFeedback srv;
    srv.request.ask = HAND_OVER_READY;

    ros::Rate r(100);
    while(RobotInterface::ok())
    {
        if (disable_coll_av)      suppressCollisionAv();
        if (!_c.call(srv)) break;

        ROS_DEBUG("[%s] Received: %s ", getLimb().c_str(), srv.response.reply.c_str());

        if (srv.response.reply == HAND_OVER_DONE)
        {
            return true;
        }

        r.sleep();

        if ((ros::Time::now()-_init).toSec() > _wait_time)
        {
            ROS_ERROR("[%s] No feedback from other arm has been received in %gs!",
                                                    getLimb().c_str(), _wait_time);
            return false;
        }
    }

    return false;
}

ARTagCtrl::~ARTagCtrl()
{

}
