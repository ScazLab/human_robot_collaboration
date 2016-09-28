#ifndef __CUBE_PICKER_H__
#define __CUBE_PICKER_H__

#include <flatpack_furniture/artag_ctrl.h>

class CubePicker : public ARTagCtrl
{
private:
    /**
     * Sets the joint-level configuration for the home position
     */
    void setHomeConfiguration();
public:
    /**
     * Constructor
     */
    CubePicker(std::string _name, std::string _limb, bool _no_robot = false);

    /**
     * Destructor
     */
    ~CubePicker();
};

#endif
