#include "tool_picker.h"

using namespace              std;
using namespace baxter_core_msgs;

#define VERTICAL_ORI_R2         0.1, 1.0, 0.0, 0.0

ToolPicker::ToolPicker(string _name, string _limb, bool _use_robot) :
                       HoldCtrl(_name,_limb, _use_robot),
                       CartesianEstimatorClient(_name, _limb)
{
    setHomeConfiguration();
    setArmSpeed(getArmSpeed() / 1.3);

    setState(START);

    insertAction(ACTION_GET,       static_cast<f_action>(&ToolPicker::getObject));
    insertAction(ACTION_PASS,      static_cast<f_action>(&ToolPicker::passObject));
    insertAction(ACTION_GET_PASS,  static_cast<f_action>(&ToolPicker::getPassObject));
    insertAction(ACTION_CLEANUP,   static_cast<f_action>(&ToolPicker::cleanUpObject));

    removeAction(ACTION_HOLD);

    insertAction(string(ACTION_HOLD) + "_leg", static_cast<f_action>(&ToolPicker::holdObject));
    insertAction(string(ACTION_HOLD) + "_top", static_cast<f_action>(&ToolPicker::holdObject));

    printActionDB();

    if (not _use_robot) return;

    // reduceSquish();

    if (!callAction(ACTION_HOME)) setState(ERROR);
}

bool ToolPicker::getObject()
{
    if (!homePoseStrict())          return false;
    ros::Duration(0.05).sleep();

    if (getObjectIDs().size() >  1)
    {
        setSubState(CHECK_OBJ_IDS);
        int id = chooseObjectID(getObjectIDs());
        if (id == -1)       return false;
        setObjectID(id);
        ROS_INFO("[%s] Chosen object with name %s", getLimb().c_str(),
                       ClientTemplate<string>::getObjectID().c_str());
    }

    if (!pickUpObject())            return false;
    if (!close())                   return false;
    if (!moveArm("up", 0.3))        return false;
    // if (!hoverAboveTable(Z_LOW))    return false;

    return true;
}

bool ToolPicker::passObject()
{
    if (getPrevAction() != ACTION_GET)  return false;

    if (ClientTemplate<string>::getObjectID() == "screwdriver")
    {
        if (!goToPose(0.85, -0.26, 0.27,
                      HORIZONTAL_ORI_R))    return false;
        if (!waitForUserCuffUpperFb())      return false;
        if (!open())                        return false;
    }
    else if (ClientTemplate<string>::getObjectID() == "screws_box"  ||
             ClientTemplate<string>::getObjectID() == "brackets_box")
    {
        if (!hoverAboveTable(Z_LOW))    return false;

        if (ClientTemplate<string>::getObjectID() == "brackets_box")
        {
            if (!goToPose(0.63, -0.10, -0.14, VERTICAL_ORI_R2)) return false;
        }
        else
        {
            if (!goToPose(0.63, -0.30, -0.14, VERTICAL_ORI_R2)) return false;
        }

        ros::Duration(0.25).sleep();
        if (!open())                    return false;
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
    ros::Duration(0.05).sleep();

    if (getObjectIDs().size() >  1)
    {
        setSubState(CHECK_OBJ_IDS);
        int id = chooseObjectID(getObjectIDs());
        if (id == -1)       return false;
        setObjectID(id);
        ROS_INFO("[%s] Chosen object with name %s", getLimb().c_str(),
                       ClientTemplate<string>::getObjectID().c_str());
    }

    if (!waitForObjFound())
    {
        setSubState(NO_OBJ);
        return false;
    }

    if (!pickUpObject())            return false;
    if (!close())                   return false;
    if (!moveArm("up", 0.3))        return false;
    if (!homePoseStrict())          return false;

    if (ClientTemplate<string>::getObjectID() == "screwdriver")
    {
        if (!goToPose( 0.20, -0.85, -0.30, POOL_ORI_R)) return false;
    }
    else if (ClientTemplate<string>::getObjectID() == "brackets_box")
    {
        if (!goToPose( 0.00, -0.85, -0.25, POOL_ORI_R)) return false;
    }
    else if (ClientTemplate<string>::getObjectID() == "screws_box")
    {
        if (!goToPose(-0.15, -0.85, -0.25, POOL_ORI_R)) return false;
    }

    ros::Duration(0.25).sleep();
    if (!open())                    return false;
    if (!homePoseStrict())          return false;

    return true;
}

bool ToolPicker::pickUpObject()
{
    ROS_INFO("[%s] Start Picking up object %s..", getLimb().c_str(),
                  ClientTemplate<string>::getObjectID().c_str());

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

    if (!computeOffsets(offs_x, offs_y))    return false;

    // Let's compute a first estimation of the joint position
    // (we reduce the z by 10 cm to start picking up from a
    // closer position)
    double x = getObjectPos().x + offs_x;
    double y = getObjectPos().y + offs_y;
    double z =       getPos().z -   0.15;

    geometry_msgs::Quaternion q;
    if (!computeOrientation(q))             return false;

    ROS_INFO("Going to: %g %g %g", x, y, z);

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
    int cnt_ik_fail = 0;

    ros::Rate r(THREAD_FREQ);

    while(RobotInterface::ok())
    {
        double new_elap_time = (ros::Time::now() - start_time).toSec();

        x = getObjectPos().x + offs_x;
        y = getObjectPos().y + offs_y;
        z = z_start - getArmSpeed() * new_elap_time;

        ROS_DEBUG("Time %g Going to: %g %g %g Position: %g %g %g", new_elap_time, x, y, z,
                                                       getPos().x, getPos().y, getPos().z);

        if (goToPoseNoCheck(x,y,z,q.x, q.y, q.z, q.w))
        {
            cnt_ik_fail = 0;
            // if (new_elap_time - elap_time > 0.02)
            // {
            //     ROS_WARN("\t\t\t\t\tTime elapsed: %g", new_elap_time - elap_time);
            // }
            // elap_time = new_elap_time;

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

        if (cnt_ik_fail == 10)  return false;
    }

    return false;
}

bool ToolPicker::determineContactCondition()
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
    else if (ClientTemplate<string>::getObjectID() != "screwdriver")
    {
        if (getAction() == ACTION_CLEANUP)
        {
            if (getPos().z < -0.17)
            {
                ROS_INFO("Object reached!");
                return true;
            }
        }
        else if (getAction() == ACTION_GET      ||
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

bool ToolPicker::computeOffsets(double &_x_offs, double &_y_offs)
{
    if      (getAction() == ACTION_GET || getAction() == ACTION_GET_PASS)
    {
        if (ClientTemplate<string>::getObjectID() == "screwdriver")
        {
            _x_offs = +0.010;
            // _y_offs = +0.017;
        }
        else if (ClientTemplate<string>::getObjectID() == "screws_box"  ||
                 ClientTemplate<string>::getObjectID() == "brackets_box")
        {
            _x_offs = +0.06;
        }
    }
    else if (getAction() == ACTION_CLEANUP)
    {
        if (ClientTemplate<string>::getObjectID() == "screwdriver")
        {
            // _x_offs = -0.020;
            _y_offs = -0.010;
        }
        else if (ClientTemplate<string>::getObjectID() == "screws_box"  ||
                 ClientTemplate<string>::getObjectID() == "brackets_box")
        {
            _x_offs = +0.020;
            _y_offs = -0.058;
        }
    }
    else
    {
        ROS_ERROR("State is neither ACTION_GET, ACTION_GET_PASS or ACTION_CLEANUP!");
        return false;
    }

    return true;
}

bool ToolPicker::computeOrientation(geometry_msgs::Quaternion &_q)
{
    if      (getAction() == ACTION_GET || getAction() == ACTION_GET_PASS)
    {
        quaternionFromDoubles(_q, POOL_ORI_R);
    }
    else if (getAction() == ACTION_CLEANUP)
    {
        quaternionFromDoubles(_q, VERTICAL_ORI_R);
    }
    else
    {
        ROS_ERROR("State is neither ACTION_GET, ACTION_GET_PASS or ACTION_CLEANUP!");
        return false;
    }

    return true;
}

int ToolPicker::chooseObjectID(vector<int> _objs)
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

    std::vector<string> objs_str;
    for (size_t i = 0; i < _objs.size(); ++i)
    {
        objs_str.push_back(getObjectNameFromDB(_objs[i]));
    }

    std::vector<string> av_objects = ClientTemplate<string>::getAvailableObjects(objs_str);

    if (av_objects.size() == 0)
    {
        setSubState(NO_OBJ);
        return -1;
    }

    srand(time(0)); //use current time as seed

    return ArmCtrl::getObjectIDFromDB(av_objects[rand() % av_objects.size()]);
}

void ToolPicker::reduceSquish()
{
    XmlRpc::XmlRpcValue squish_params;
    // store the initial squish thresholds from the parameter server
    nh.getParam("/collision/"+getLimb()+"/baxter/squish_thresholds", squish_params);

    ROS_ASSERT_MSG(squish_params.getType()==XmlRpc::XmlRpcValue::TypeArray,
                          "[%s] Squish params is not an array! Type is %i",
                               getLimb().c_str(), squish_params.getType());

    for (int i = 0; i < squish_params.size(); ++i)
    {
        // store the initial squish thresholds for reset later
        squish_thresholds.push_back(squish_params[i]);
    }

    // adjust the squish thresholds for better tool picking
    squish_params[3] = 0.5 * static_cast<double>(squish_params[3]);
    squish_params[4] = 0.5 * static_cast<double>(squish_params[4]);
    squish_params[5] = 0.5 * static_cast<double>(squish_params[5]);

    ROS_INFO("[%s] Reduced squish thresholds for joint 3, 4 and 5 to %g. %g and %g .",
                            getLimb().c_str(), static_cast<double>(squish_params[3]),
                                               static_cast<double>(squish_params[5]),
                                               static_cast<double>(squish_params[5]));

    // set the squish thresholds in the parameter server to the new values
    nh.setParam("/collision/"+getLimb()+"/baxter/squish_thresholds", squish_params);
}

void ToolPicker::resetSquish()
{
    XmlRpc::XmlRpcValue squish_params;
    for (vector<int>::size_type i = 0; i != squish_thresholds.size(); ++i)
    {
        // rewrite the squish parameters from the initial squish thresholds stored in reduceSquish()
        squish_params[i] = squish_thresholds[i];
    }

    ROS_INFO("[%s] Squish thresholds for joint 3, 4 and 5 set back to %g. %g and %g .",
                             getLimb().c_str(), static_cast<double>(squish_params[3]),
                                                static_cast<double>(squish_params[5]),
                                                static_cast<double>(squish_params[5]));

    // reset squish thresholds in the parameter server to the new values
    nh.setParam("/collision/"+getLimb()+"/baxter/squish_thresholds", squish_params);
}

bool ToolPicker::goHoldPose(double height)
{
    ROS_INFO("[%s] Going to %s position..", getLimb().c_str(), getAction().c_str());

    if (getAction() == string(ACTION_HOLD) + "_top")
    {
        return goToPose(0.72, -0.31, 0.032, 0.54, 0.75, 0.29,0.22);
    }

    return goToPose(0.80, -0.4, height, HORIZONTAL_ORI_R);
}

void ToolPicker::setHomeConfiguration()
{
    ArmCtrl::setHomeConfiguration("pool");
}

void ToolPicker::setObjectID(int _obj)
{
    ArmCtrl::setObjectID(_obj);
    ClientTemplate<string>::setObjectID(ArmCtrl::getObjectNameFromDB(_obj));
}

ToolPicker::~ToolPicker()
{
    // resetSquish();
}
