#ifndef __CUBE_PICKER_H__
#define __CUBE_PICKER_H__

#include <flatpack_furniture/artag_ctrl.h>

class CubePicker : public ArmCtrl, public ARucoClient
{
private:
    double elap_time;

    /**
     * [moveObjectTowardHuman description]
     * @return true/false if success/failure
     */
    bool moveObjectTowardHuman();

    /**
     * [pickARTag description]
     * @return true/false if success/failure
     */
    bool pickARTag();

    /**
     * [pickObject description]
     * @return true/false if success/failure
     */
    bool pickObject();

    /**
     * [passObject description]
     * @return true/false if success/failure
     */
    bool passObject();

    /**
     * [pickObject description]
     * @return true/false if success/failure
     */
    bool pickPassObject();

    /**
     * [recoverPickPass description]
     * @return true/false if success/failure
     */
    bool recoverPickPass();

    /**
     * Recovers from errors during execution. It provides a basic interface,
     * but it is advised to specialize this function in the ArmCtrl's children.
     */
    void recoverFromError();

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

    void setObjectID(int _obj);
};

#endif
