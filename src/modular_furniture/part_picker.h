#ifndef __PART_PICKER_H__
#define __PART_PICKER_H__

#include <stdlib.h>
#include <time.h>

#include <robot_interface/arm_ctrl.h>
#include <robot_perception/cartesian_estimator_client.h>
#include <flatpack_furniture/artag_ctrl.h>
#include <baxter_collaboration/ObjectsArray.h>

class PartPicker : public ARTagCtrl
{
private:

protected:

    /**
     * Sets the joint-level configuration for the home position
     */
    void setHomeConfiguration();

public:
    /**
     * Constructor
     */
    PartPicker(std::string _name, std::string _limb, bool _no_robot = false);

    /**
     * Destructor
     */
    ~PartPicker();
};

#endif
