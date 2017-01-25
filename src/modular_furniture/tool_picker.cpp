#include "tool_picker.h"

using namespace std;
using namespace baxter_core_msgs;

ToolPicker::ToolPicker(std::string _name, std::string _limb, bool _no_robot) :
                       HoldCtrl(_name,_limb, _no_robot), CartesianEstimatorClient(_name, _limb),
                       elap_time(0)
{
    setHomeConfiguration();

    setState(START);

    insertAction(ACTION_GET,       static_cast<f_action>(&ToolPicker::getObject));
    insertAction(ACTION_PASS,      static_cast<f_action>(&ToolPicker::passObject));
    insertAction(ACTION_GET_PASS,  static_cast<f_action>(&ToolPicker::getPassObject));
    insertAction(ACTION_CLEANUP,   static_cast<f_action>(&ToolPicker::cleanUpObject));

    printActionDB();

    XmlRpc::XmlRpcValue objects_db;
    if(!_n.getParam("objects_"+getLimb(), objects_db))
    {
        ROS_INFO("No objects' database found in the parameter server. "
                 "Looked up param is %s", ("objects_"+getLimb()).c_str());
    }
    else
    {
        insertObjects(objects_db);
        printObjectDB();
    }

    if (_no_robot) return;

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

bool ToolPicker::pickUpObject()
{
    ROS_INFO("[%s] Start Picking up object %s..", getLimb().c_str(),
                  CartesianEstimatorClient::getObjectName().c_str());

    if (!is_ir_ok())
    {
        ROS_ERROR("No callback from the IR sensor! Stopping.");
        setSubState(NO_IR_SENSOR);
        return false;
    }

    if (!waitForCartEstData())
    {
        setSubState(NO_OBJ);
        return false;
    }

    double offs_x = 0.0;
    double offs_y = 0.0;

    if (!computeOffsets(offs_x, offs_y))    return false;

    double x = getObjectPos().x + offs_x;
    double y = getObjectPos().y + offs_y;
    double z =       getPos().z;

    geometry_msgs::Quaternion q;
    if (!computeOrientation(q))             return false;

    ROS_INFO("Going to: %g %g %g", x, y, z);
    if (!goToPose(x, y, z, q.x,q.y,q.z,q.w,"loose"))
    {
        return false;
    }

    if (!waitForCartEstData())
    {
        setSubState(NO_OBJ);
        return false;
    }

    ros::Time start_time = ros::Time::now();
    double z_start       =       getPos().z;
    int cnt_ik_fail = 0;

    ros::Rate r(THREAD_FREQ);

    while(RobotInterface::ok())
    {
        double new_elap_time = (ros::Time::now() - start_time).toSec();

        x = getObjectPos().x + offs_x;
        y = getObjectPos().y + offs_y;
        z = z_start - ARM_SPEED * new_elap_time / 1.3;

        ROS_DEBUG("Time %g Going to: %g %g %g Position: %g %g %g", new_elap_time, x, y, z,
                                                       getPos().x, getPos().y, getPos().z);

        if (goToPoseNoCheck(x,y,z,q.x,q.y,q.z,q.w))
        {
            cnt_ik_fail = 0;
            if (new_elap_time - elap_time > 0.02)
            {
                ROS_WARN("\t\t\t\t\tTime elapsed: %g", new_elap_time - elap_time);
            }
            elap_time = new_elap_time;

            if(hasCollidedIR("strict") || hasCollidedCD())
            {
                ROS_INFO("Collision!");
                return true;
            }

            r.sleep();
        }
        else    cnt_ik_fail++;

        if (cnt_ik_fail == 10)  return false;
    }

    return false;
}

bool ToolPicker::computeOffsets(double &_x_offs, double &_y_offs)
{
    if      (getAction() ==     ACTION_GET)
    {
        if (CartesianEstimatorClient::getObjectName() == "screwdriver")
        {
            _x_offs = +0.015;
            _y_offs = +0.020;
        }
        else if (CartesianEstimatorClient::getObjectName() == "screws_box"  ||
                 CartesianEstimatorClient::getObjectName() == "brackets_box")
        {
            _x_offs = +0.065;
        }
    }
    else if (getAction() == ACTION_CLEANUP)
    {
        if (CartesianEstimatorClient::getObjectName() == "screwdriver")
        {
            _x_offs = -0.020;
            _y_offs = -0.015;
        }
        else if (CartesianEstimatorClient::getObjectName() == "screws_box"  ||
                 CartesianEstimatorClient::getObjectName() == "brackets_box")
        {
            _y_offs = -0.065;
        }
    }
    else
    {
        return false;
    }

    return true;
}

bool ToolPicker::computeOrientation(geometry_msgs::Quaternion &_q)
{
    if      (getAction() ==     ACTION_GET)
    {
        // _q = geometry_msgs::Quaternion(POOL_ORI_R);
    }
    else if (getAction() == ACTION_CLEANUP)
    {
        // _q = geometry_msgs::Quaternion(VERTICAL_ORI_R);
    }
    else
    {
        return false;
    }

    return true;
}

int ToolPicker::chooseObjectID(std::vector<int> _objs)
{
    int res = -1;

    // if (!hoverAbovePool()) return res;
    if (!waitForCartEstOK())        return res;

    std::vector<string> objs_str;
    for (size_t i = 0; i < _objs.size(); ++i)
    {
        objs_str.push_back(getObjectNameFromDB(_objs[i]));
    }

    std::vector<string> av_objects = getAvailableObjects(objs_str);

    if (av_objects.size() == 0)     return res;

    std::srand(std::time(0)); //use current time as seed
    string res_str = av_objects[rand() % av_objects.size()];

    res = getObjectIDFromDB(res_str);

    return res;
}

bool ToolPicker::getObject()
{
    if (!homePoseStrict())          return false;
    ros::Duration(0.05).sleep();
    if (!pickUpObject())            return false;
    if (!gripObject())              return false;
    if (!moveArm("up", 0.3))        return false;
    // if (!hoverAboveTable(Z_LOW))    return false;

    return true;
}

bool ToolPicker::passObject()
{
    if (getPrevAction() != ACTION_GET)  return false;

    if (CartesianEstimatorClient::getObjectName() == "screwdriver")
    {
        if (!moveObjectTowardHuman())       return false;
        ros::Duration(1.0).sleep();
        if (!waitForForceInteraction())     return false;
        if (!releaseObject())               return false;
    }
    else if (CartesianEstimatorClient::getObjectName() == "screws_box"  ||
             CartesianEstimatorClient::getObjectName() == "brackets_box")
    {
        if (!hoverAboveTable(Z_LOW))    return false;

        if (CartesianEstimatorClient::getObjectName() == "brackets_box")
        {
            if (!goToPose(0.65, -0.00, -0.06, VERTICAL_ORI_R)) return false;
        }
        else
        {
            if (!goToPose(0.65, -0.25, -0.06, VERTICAL_ORI_R)) return false;
        }

        ros::Duration(0.5).sleep();
        if (!releaseObject())           return false;
        if (!hoverAboveTable(Z_LOW))    return false;
    }

    if (!homePoseStrict())              return false;

    return true;
}

bool ToolPicker::getPassObject()
{
    if (!getObject())      return false;
    setPrevAction(ACTION_GET);
    if (!passObject())     return false;

    return true;
}

bool ToolPicker::cleanUpObject()
{
    if (!goToPose(0.65, -0.25, 0.25, VERTICAL_ORI_R)) return false;
    return true;
}

bool ToolPicker::moveObjectTowardHuman()
{
    ROS_INFO("[%s] Moving object toward human..", getLimb().c_str());
    return goToPose(0.80, -0.26, 0.32, VERTICAL_ORI_L);
}

void ToolPicker::setHomeConfiguration()
{
    ArmCtrl::setHomeConfiguration("pool");
}

void ToolPicker::setObjectID(int _obj)
{
    ArmCtrl::setObjectID(_obj);
    CartesianEstimatorClient::setObjectName(ArmCtrl::getObjectNameFromDB(_obj));
}

ToolPicker::~ToolPicker()
{

}
