#ifndef ACTION_PROVIDER_H
#define ACTION_PROVIDER_H

#include <string.h>

#include "robot_interface/artag_ctrl.h"
#include "robot_interface/hold_ctrl.h"

class actionProvider
{
private:
    std::string name;

    ros::NodeHandle _n;
    ros::ServiceServer service;

    ARTagCtrl*  left_ctrl;
    HoldCtrl*  right_ctrl;

public:
    actionProvider(std::string _name);
    ~actionProvider();

    bool serviceCallback(baxter_collaboration::DoAction::Request  &req, 
                         baxter_collaboration::DoAction::Response &res);
};

#endif
