#include "flatpack_furniture/artag_ctrl.h"

#include <tf/transform_datatypes.h>

using namespace std;
using namespace human_robot_collaboration_msgs;

ARTagCtrl::ARTagCtrl(std::string _name, std::string _limb, bool _use_robot) :
                     ArmCtrl(_name,_limb, _use_robot), ARucoClient(_name, _limb)
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

bool ARTagCtrl::getObject()
{
    if (!homePoseStrict())          return false;
    ros::Duration(0.05).sleep();

    if (getObjectIDs().size() >  1)
    {
        setSubState(CHECK_OBJ_IDS);
        int id = chooseObjectID(getObjectIDs());
        if (id == -1)       return false;
        setObjectID(id);
        ROS_INFO_COND(print_level>=1, "[%s] Chosen object with ID %i",
               getLimb().c_str(), ClientTemplate<int>::getObjectID());
    }

    if (!pickARTag())               return false;
    if (!close())                   return false;
    ros::Duration(2.0).sleep();
    if (!open())                    return false;
    if (!homePoseStrict())          return false;

    return true;
}

bool ARTagCtrl::passObject()
{
    if (getPrevAction() != ACTION_GET)  return false;
    if (!moveObjectTowardHuman())       return false;
    ros::Duration(1.0).sleep();
    if (!waitForUserCuffUpperFb())      return false;
    if (!open())                        return false;
    ros::Duration(0.2).sleep();
    if (!homePoseStrict())              return false;

    return true;
}

bool ARTagCtrl::getPassObject()
{
    if (!getObject())      return false;
    setPrevAction(ACTION_GET);
    if (!passObject())     return false;

    return true;
}

bool ARTagCtrl::recoverRelease()
{
    if (getPrevAction() != ACTION_RELEASE)  return false;
    if(!homePoseStrict())                   return false;
    ros::Duration(0.05).sleep();
    if (!pickARTag())                       return false;
    if (!close())                           return false;
    if (!moveArm("up", 0.2))                return false;
    if (!homePoseStrict())                  return false;

    return true;
}

bool ARTagCtrl::recoverGet()
{
    if (!hoverAbovePool())          return false;
    if (!open())                    return false;
    if (!homePoseStrict())          return false;

    return true;
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

bool ARTagCtrl::pickARTag()
{
    ROS_INFO_COND(print_level>=0, "[%s] Start Picking up tag..", getLimb().c_str());

    if (!isIRok())
    {
        ROS_ERROR("No callback from the IR sensor! Stopping.");
        setSubState(NO_IR_SENSOR);
        return false;
    }

    if (!waitForData())
    {
        setSubState(NO_OBJ);
        return false;
    }

    double offs_x = 0.0;
    double offs_y = 0.0;

    if (not computeOffsets(offs_x, offs_y))  { return false; }

    // Let's compute a first estimation of the joint position
    // (we reduce the z by 15 cm to start picking up from a
    // closer position)
    double x = getObjectPos().x + offs_x;
    double y = getObjectPos().y + offs_y;
    double z =       getPos().z -   0.15;

    geometry_msgs::Quaternion q;
    if (not computeOrientation(q))           { return false; }

    ROS_INFO_COND(print_level>=1, "Going to: %g %g %g", x, y, z);

    if (!goToPose(x, y, z, q.x, q.y, q.z, q.w, "loose"))
    {
        return false;
    }

    if (!waitForData())
    {
        setSubState(NO_OBJ);
        return false;
    }

    ros::Time start_time = ros::Time::now();
    double z_start       =       getPos().z;
    int cnt_ik_fail      =                0;

    ros::Rate r(THREAD_FREQ);
    while(RobotInterface::ok())
    {
        double elap_time = (ros::Time::now() - start_time).toSec();

        double x = getObjectPos().x + offs_x;
        double y = getObjectPos().y + offs_y;
        double z = z_start - getArmSpeed() * elap_time;

        ROS_INFO_COND(print_level>=3, "Time %g Going to: %g %g %g Position: %g %g %g",
                              elap_time, x, y, z, getPos().x, getPos().y, getPos().z);

        if (goToPoseNoCheck(x,y,z,q.x, q.y, q.z, q.w))
        {
            cnt_ik_fail = 0;
            // if (elap_time - old_elap_time > 0.02)
            // {
            //     ROS_WARN("\t\t\t\t\tTime elapsed: %g", elap_time - old_elap_time);
            // }
            // old_elap_time = elap_time;

            if (determineContactCondition())
            {
                return true;
            }

            r.sleep();
        }
        else
        {
            ++cnt_ik_fail;
        }

        if (cnt_ik_fail == 10)      { return false; }
    }

    return false;
}

bool ARTagCtrl::determineContactCondition()
{
    if (hasCollidedIR("strict") || hasCollidedCD())
    {
        if (hasCollidedCD())
        {
            moveArm("up", 0.002);
        }
        ROS_INFO("Collision!");
        return true;
    }
    else
    {
        if (getAction() == ACTION_GET      ||
            getAction() == ACTION_GET_PASS)
        {
            if (getPos().z < -0.28)
            {
                ROS_INFO("Object reached!");
                return true;
            }
        }
    }

    return false;
}

bool ARTagCtrl::computeOffsets(double &_x_offs, double &_y_offs)
{
    return true;
}

int ARTagCtrl::chooseObjectID(vector<int> _objs)
{
    if (getSubState() != CHECK_OBJ_IDS)
    {
        return ArmCtrl::chooseObjectID(_objs);
    }

    ROS_INFO_COND(print_level>=2, "[%s] Choosing object IDs", getLimb().c_str());

    if (!waitForOK())
    {
        setSubState(NO_OBJ);
        return -1;
    }

    if (!waitForObjsFound())
    {
        setSubState(NO_OBJ);
        return -1;
    }

    std::vector<int> av_objects = ClientTemplate<int>::getAvailableObjects(_objs);

    if (av_objects.size() == 0)
    {
        setSubState(NO_OBJ);
        return -1;
    }

    srand(time(0)); //use current time as seed

    return av_objects[rand() % av_objects.size()];
}

bool ARTagCtrl::computeOrientation(geometry_msgs::Quaternion &_ori)
{
    if      (ClientTemplate<int>::getObjectID() == 24)
    {
        // Get the rotation matrix for the object, as retrieved from
        tf::Quaternion mrk_q;
        tf::quaternionMsgToTF(getObjectOri(), mrk_q);
        tf::Matrix3x3 mrk_rot(mrk_q);

        // printf("Object Orientation\n");
        // for (int j = 0; j < 3; ++j)
        // {
        //     printf("%g\t%g\t%g\n", mrk_rot[j][0], mrk_rot[j][1], mrk_rot[j][2]);
        // }

        // Compute the transform matrix between the object's orientation
        // and the end-effector's orientation
        tf::Matrix3x3 mrk2ee;
        mrk2ee[0][0] =  1;  mrk2ee[0][1] =  0;   mrk2ee[0][2] =  0;
        mrk2ee[1][0] =  0;  mrk2ee[1][1] =  0;   mrk2ee[1][2] = -1;
        mrk2ee[2][0] =  0;  mrk2ee[2][1] =  1;   mrk2ee[2][2] =  0;

        // printf("Rotation\n");
        // for (int j = 0; j < 3; ++j)
        // {
        //     printf("%g\t%g\t%g\n", mrk2ee[j][0], mrk2ee[j][1], mrk2ee[j][2]);
        // }

        // Compute the final end-effector orientation, and convert it to a msg
        mrk2ee = mrk_rot * mrk2ee;

        tf::Quaternion ee_q;
        mrk2ee.getRotation(ee_q);
        geometry_msgs::Quaternion ee_q_msg;
        tf::quaternionTFToMsg(ee_q,ee_q_msg);

        // printf("Desired Orientation\n");
        // for (int j = 0; j < 3; ++j)
        // {
        //     printf("%g\t%g\t%g\n", mrk2ee[j][0], mrk2ee[j][1], mrk2ee[j][2]);
        // }

        _ori = ee_q_msg;
    }
    else if (getAction() == ACTION_GET || getAction() == ACTION_GET_PASS)
    {
        if      (getLimb() == "left")
        {
            quaternionFromDoubles(_ori, POOL_ORI_L);
        }
        else if (getLimb() == "right")
        {
            quaternionFromDoubles(_ori, POOL_ORI_R);
        }
    }
    else
    {
        ROS_ERROR("State is neither ACTION_GET or ACTION_GET_PASS!");
        return false;
    }

    return true;
}

void ARTagCtrl::setHomeConfiguration()
{
    ArmCtrl::setHomeConfiguration("pool");
}

bool ARTagCtrl::moveObjectTowardHuman()
{
    ROS_INFO("[%s] Moving object toward human..", getLimb().c_str());

    if      (getLimb() == "left")
    {
        return goToPose(0.80,  0.26, 0.32, HORIZONTAL_ORI_L);
    }
    else if (getLimb() == "right")
    {
        return goToPose(0.85, -0.26, 0.27, HORIZONTAL_ORI_R);
    }
    else
    {
        return false;
    }
}

void ARTagCtrl::setObjectID(int _obj)
{
    ArmCtrl::setObjectID(_obj);
    ARucoClient::setObjectID(_obj);
}

ARTagCtrl::~ARTagCtrl()
{

}
