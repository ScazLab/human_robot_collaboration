#include "tower_building/cube_picker.h"

using namespace std;
using namespace baxter_core_msgs;

CubePicker::CubePicker(std::string _name, std::string _limb, bool _no_robot) :
                       ARucoClient(_name, _limb), ArmCtrl(_name,_limb, _no_robot)
{
    setHomeConfiguration();

    setState(START);

    insertAction(ACTION_GET,       static_cast<f_action>(&CubePicker::pickObject));
    insertAction(ACTION_PASS,      static_cast<f_action>(&CubePicker::passObject));
    insertAction(ACTION_GET_PASS,  static_cast<f_action>(&CubePicker::pickPassObject));

    printDB();

    if (_no_robot) return;

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

bool CubePicker::pickObject()
{
    if (!homePoseStrict())          return false;
    ros::Duration(0.05).sleep();
    if (!pickARTag())               return false;
    if (!gripObject())              return false;
    if (!moveArm("up", 0.3))        return false;
    if (!hoverAboveTable(Z_LOW))    return false;

    return true;
}

bool CubePicker::passObject()
{
    if (getSubState() != ACTION_GET)    return false;
    if (!moveObjectTowardHuman())       return false;
    ros::Duration(1.0).sleep();
    if (!waitForForceInteraction())     return false;
    if (!releaseObject())               return false;
    if (!homePoseStrict())              return false;

    return true;
}

bool CubePicker::pickPassObject()
{
    if (!pickObject())      return false;
    setSubState(ACTION_GET);
    if (!passObject())      return false;

    return true;
}

bool CubePicker::pickARTag()
{
    ROS_INFO("[%s] Start Picking up tag..", getLimb().c_str());

    if (!is_ir_ok())
    {
        ROS_ERROR("No callback from the IR sensor! Stopping.");
        return false;
    }

    if (!waitForARucoData()) return false;

    geometry_msgs::Quaternion q;

    double x = getMarkerPos().x;
    double y = getMarkerPos().y + 0.04;
    double z =       getPos().z;

    ROS_DEBUG("Going to: %g %g %g", x, y, z);
    if (!goToPose(x, y, z, POOL_ORI_L,"loose"))
    {
        return false;
    }

    if (!waitForARucoData()) return false;

    ros::Time start_time = ros::Time::now();
    double z_start       =       getPos().z;
    int cnt_ik_fail      =                0;

    ros::Rate r(100);
    while(RobotInterface::ok())
    {
        double new_elap_time = (ros::Time::now() - start_time).toSec();

        double x = getMarkerPos().x;
        double y = getMarkerPos().y;
        double z = z_start - PICK_UP_SPEED * new_elap_time;

        ROS_DEBUG("Time %g Going to: %g %g %g", new_elap_time, x, y, z);

        if (goToPoseNoCheck(x,y,z,POOL_ORI_L))
        {
            cnt_ik_fail = 0;
            if (new_elap_time - elap_time > 0.02)
            {
                ROS_WARN("\t\t\t\t\tTime elapsed: %g", new_elap_time - elap_time);
            }
            elap_time = new_elap_time;

            if(hasCollided("strict"))
            {
                ROS_DEBUG("Collision!");
                return true;
            }

            r.sleep();
        }
        else
        {
            cnt_ik_fail++;
        }

        if (cnt_ik_fail == 10)
        {
            return false;
        }
    }

    return false;
}

bool CubePicker::moveObjectTowardHuman()
{
    ROS_INFO("[%s] Moving object toward human..", getLimb().c_str());
    return goToPose(0.80, 0.26, 0.32, VERTICAL_ORI_L);
}

bool CubePicker::doAction(int s, std::string a)
{
    if (a == ACTION_GET || a == ACTION_PASS)
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

void CubePicker::setHomeConfiguration()
{
    setHomeConf(0.7060, -1.2717, 0.3846,  1.5405,
                        -0.1273, 1.3135,  0.3206);
}

void CubePicker::setObject(int _obj)
{
    ArmCtrl::setObject(_obj);
    ARucoClient::setMarkerID(_obj);
}

CubePicker::~CubePicker()
{

}
