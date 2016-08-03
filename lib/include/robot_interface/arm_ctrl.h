#ifndef __ARM_CONTROLLER_H__
#define __ARM_CONTROLLER_H__

#include "robot_interface/ros_thread.h"
#include "robot_interface/gripper.h"

#include "baxter_collaboration/DoAction.h"

class ArmCtrl : public ROSThread, public Gripper
{
private:
    std::string name;

    std::string action;
    int marker_id;

    ros::ServiceServer service;

    virtual void recoverFromError();

    virtual bool goHome() = 0;

protected:
    virtual void InternalThreadEntry() = 0;

public:
    ArmCtrl(std::string _name, std::string _limb);

    ~ArmCtrl();

    bool serviceCb(baxter_collaboration::DoAction::Request  &req,
                   baxter_collaboration::DoAction::Response &res);

    void setName  (std::string _name)   { name      =   _name; };
    void setAction(std::string _action) { action    = _action; };
    void setMarkerID(int _id)           { marker_id =     _id; };

    std::string getName()     { return      name; };
    std::string getAction()   { return    action; };
    int         getMarkerID() { return marker_id; };
};

#endif
