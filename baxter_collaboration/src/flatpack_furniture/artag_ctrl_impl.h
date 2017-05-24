#ifndef __ARTAG_CTRL_IMPL_H__
#define __ARTAG_CTRL_IMPL_H__

#include "flatpack_furniture/artag_ctrl.h"

class ARTagCtrlImpl : public ARTagCtrl
{
private:

public:
    /**
     * Constructor
     */
    ARTagCtrlImpl(std::string _name, std::string _limb, bool _use_robot = true);

    /**
     * Destructor
     */
    ~ARTagCtrlImpl();
};

#endif
