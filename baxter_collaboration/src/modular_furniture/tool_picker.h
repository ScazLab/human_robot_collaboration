#ifndef __TOOL_PICKER_H__
#define __TOOL_PICKER_H__

#include <stdlib.h>
#include <time.h>

#include <flatpack_furniture/hold_ctrl.h>
#include <robot_perception/cartesian_estimator_client.h>
#include <baxter_collaboration_msgs/ObjectsArray.h>

#define ACTION_CLEANUP  "cleanup"

#define  CHECK_OBJ_IDS  "check_obj_ids"

class ToolPicker : public HoldCtrl, public CartesianEstimatorClient
{
private:
    double elap_time;

    std::vector<double> squish_thresholds;

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
     * Cleans up the selected obejct from the table by using the cartesian estimator's info
     * @return true/false if success/failure
     */
    bool cleanUpObject();

    /**
     * Picks up the selected object by using the cartesian estimator's info
     * @return true/false if success/failure
     */
    bool pickUpObject();

    /**
     * Determines if a contact occurred by reading the IR sensor and looking for
     * eventual squish events. Since the SDK does not allow for setting custom squish params,
     * the latter can often fail so there is a check that prevents the end-effector from going
     * too low if this happens.
     * @return true/false if success/failure
     */
    bool determineContactCondition();

    /**
     * Computes object-specific (and pose-specific) offsets in order for the robot
     * to grip the object not in the center of its coordinate system but where
     * it is most convenient for the gripper
     *
     * @param x_offs The x offset
     * @param y_offs The y offset
     *
     * @return true/false if success/failure
     */
    bool computeOffsets(double &x_offs, double &y_offs);

    /**
     * Computes action-specific orientation in order for the robot to be able to
     * be transparent with respect to different actions in different poses
     *
     * @param _ori The desired orientation as a quaternion
     * @return true/false if success/failure
     */
    bool computeOrientation(geometry_msgs::Quaternion &_q);

    /**
     * Stores initial squish thresholds to squish_thresholds, then
     * reduces and rewrites squish thresholds to aid in picking up tools
     */
    void reduceSquish();

    /**
     * Resets squish thresholds to original values stored in squish_thresholds
     */
    void resetSquish();

protected:

    /**
     * Sets the joint-level configuration for the home position
     */
    void setHomeConfiguration();

    /**
     * [goHoldPose description]
     * @param  height [description]
     * @return        true/false if success/failure
     */
    bool goHoldPose(double height);

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
    ToolPicker(std::string _name, std::string _limb, bool _use_robot = true);

    /**
     * Destructor
     */
    ~ToolPicker();

    /* SETTERS */
    void setObjectID(int _obj);
};

#endif
