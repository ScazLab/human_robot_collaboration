#ifndef __TOOL_PICKER_H__
#define __TOOL_PICKER_H__

#include <stdlib.h>
#include <time.h>

#include <robot_interface/arm_ctrl.h>
#include <robot_perception/cartesian_estimator_client.h>
#include <baxter_collaboration/ObjectsArray.h>

class ToolPicker : public ArmCtrl, public CartesianEstimatorClient
{
private:
    double elap_time;

    /**
     * [moveObjectTowardHuman description]
     * @return true/false if success/failure
     */
    bool moveObjectTowardHuman();

    /**
     * [getObject description]
     * @return true/false if success/failure
     */
    bool getObject();

    /**
     * [passObject description]
     * @return true/false if success/failure
     */
    bool passObject();

    /**
     * [getPassObject description]
     * @return true/false if success/failure
     */
    bool getPassObject();

    /**
     * Picks up the selected object by using the cartesian estimator's info
     * @return true/false if success/failure
     */
    bool pickUpObject();

    /**
     * Sets the joint-level configuration for the home position
     */
    void setHomeConfiguration();

protected:
    /**
     * Chooses the object to act upon according to some rule. This method
     * needs to be specialized in any derived class because it is dependent
     * on the type of action and the type of sensory capabilities available.
     *
     * @param _objs The list of IDs of objects to choose from
     * @return      the ID of the chosen object (by default the ID of the
     *              first object will be chosen)
     */
    int chooseObjectID(std::vector<int> _objs);

public:
    /**
     * Constructor
     */
    ToolPicker(std::string _name, std::string _limb, bool _no_robot = false);

    /**
     * Destructor
     */
    ~ToolPicker();

    /* SETTERS */
    void setObjectID(int _obj);
};

#endif
