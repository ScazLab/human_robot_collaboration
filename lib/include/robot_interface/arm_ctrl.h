#ifndef __ARM_CONTROLLER_H__
#define __ARM_CONTROLLER_H__

#include <map>

#include "robot_interface/ros_thread.h"
#include "robot_interface/robot_interface.h"
#include "robot_interface/gripper.h"

#include "baxter_collaboration/DoAction.h"
#include "baxter_collaboration/AskFeedback.h"
#include "baxter_collaboration/ArmState.h"

#define HAND_OVER_START  "handover_start"
#define HAND_OVER_READY  "handover_ready"
#define HAND_OVER_DONE   "handover_done"
#define HAND_OVER_WAIT   "handover_wait"

class ArmCtrl : public RobotInterface, public Gripper, public ROSThread
{
private:
    // Substate of the controller (useful to keep track of
    // long actions that need multiple internal states)
    std::string  sub_state;

    // High level action the controller is engaged in
    std::string     action;

    // Object the controller is acting upon (if the action
    // is done with respect to an object)
    int          object_id;

    // Flag to know if the robot will try to recover from an error
    // or will wait the external planner to take care of that
    bool internal_recovery;

    // Service to request actions to
    ros::ServiceServer service;

    // Internal service used for multi-arm actions
    ros::ServiceServer service_other_limb;

    // Publisher to publish the high-level state of the controller
    // (to be shown in the Baxter display)
    ros::Publisher     state_pub;

    // Home configuration. Setting it in any of the children
    // of this class is mandatory (through the virtual method
    // called setHomeConfiguration() )
    baxter_core_msgs::JointCommand home_conf;

    /**
     * Object database, which pairs an integer key, corresponding to the marker ID
     * placed on the object and read by ARuco, with a string that describes the object
     * itself in human terms.
     */
    std::map<int, std::string> object_db;

    /**
     * Provides basic functionalities for the object, such as a goHome and releaseObject.
     * For deeper, class-specific specialization, please modify doAction() instead.
     */
    void InternalThreadEntry();

protected:

    /**
     * Pointer to the action prototype function, which does not take any
     * input argument and returns true/false if success/failure
     */
    typedef bool(ArmCtrl::*f_action)();

    /**
     * Action database, which pairs a string key, corresponding to the action name,
     * with its relative action, which is an f_action.
     *
     * Please be aware that, by default, if the user calls an action with the wrong
     * key or an action that is not available, the code will segfault. By C++
     * standard: operator[] returns (*((insert(make_pair(x, T()))).first)).second
     * Which means that if we are having a map of pointers to functions, a wrong key
     * will segfault the software. A layer of protection has been put in place to
     * avoid accessing a non-existing key (so this does not happen any more, but it
     * is still worth knowing).
     */
    std::map <std::string, f_action> action_db;

    /**
     * Recovers from errors during execution. It provides a basic interface,
     * but it is advised to specialize this function in the ArmCtrl's children.
     */
    virtual void recoverFromError();

    /**
     * Hovers above table at a specific x-y position.
     * @param  height the z-axis value of the end-effector position
     * @return        true/false if success/failure
     */
    bool hoverAboveTable(double height, std::string mode="loose",
                                    bool disable_coll_av = false);

    /**
     * Home position with a specific joint configuration. This has
     * been introduced in order to force the arms to go to the home configuration
     * in always the same exact way, in order to clean the seed configuration in
     * case of subsequent inverse kinematics requests.
     *
     * @param  disable_coll_av if to disable the collision avoidance while
     *                         performing the action or not
     * @return                 true/false if success/failure
     */
    bool homePoseStrict(bool disable_coll_av = false);

    /**
     * Sets the joint-level configuration for the home position
     *
     * @param s0 First  shoulder joint
     * @param s1 Second shoulder joint
     * @param e0 First  elbow    joint
     * @param e1 Second elbow    joint
     * @param w0 First  wrist    joint
     * @param w1 Second wrist    joint
     * @param w2 Third  wrist    joint
     */
    void setHomeConf(double s0, double s1, double e0, double e1,
                                double w0, double w1, double w2);

    /**
     * Sets the joint-level configuration for the home position
     */
    virtual void setHomeConfiguration() = 0;

    /**
     * Goes to the home position, and "releases" the gripper
     *
     * @return        true/false if success/failure
     */
    bool goHome();

    /**
     * Moves arm in a direction requested by the user, relative to the current
     * end-effector position
     *
     * @param dir  the direction of motion (left right up down forward backward)
     * @param dist the distance from the end-effector starting point
     *
     * @return true/false if success/failure
     */
    bool moveArm(std::string dir, double dist, std::string mode = "loose",
                                             bool disable_coll_av = false);

    /**
     * Placeholder for an action that has not been implemented (yet)
     *
     * @return false always
     */
    bool notImplemented();

    /**
     * Adds an object to the object database
     * @param  id the id of the object as read by ARuco
     * @param  n  its name as a string
     * @return    true/false if the insertion was successful or not
     */
    bool insertObject(int id, const std::string &n);

    /**
     * Removes an object from the database. If the object is not in the
     * database, the return value will be false.
     *
     * @param   id the object to be removed
     * @return     true/false if the removal was successful or not
     */
    bool removeObject(int id);

    /**
     * Gets an object from the object database
     *
     * @param    id the requested object
     * @return      the associated string
     *              (empty string if object is not there)
     */
    std::string getObjectName(int id);

    /**
     * Checks if an object is available in the database
     * @param  id the action to check for
     * @param  insertAction flag to know if the method has been called
     *                      inside insertAction (it only removes the
     *                      ROS_ERROR if the action is not in the DB)
     * @return   true/false if the action is available in the database
     */
    bool isObjectInDB(int id);

    /**
     * Prints the object database to screen.
     */
    void printObjectDB();

    /**
     * Converts the action database to a string.
     * @return the list of allowed actions, separated by a comma.
     */
    std::string objectDBToString();

    /**
     * Adds an action to the action database
     *
     * @param   a the action to be removed
     * @param   f a pointer to the action, in the form bool action()
     * @return    true/false if the insertion was successful or not
     */
    bool insertAction(const std::string &a, ArmCtrl::f_action f);

    /**
     * Removes an action from the database. If the action is not in the
     * database, the return value will be false.
     *
     * @param   a the action to be removed
     * @return    true/false if the removal was successful or not
     */
    bool removeAction(const std::string &a);

    /**
     * Calls an action from the action database
     *
     * @param    a the action to take
     * @return     true/false if the action called was successful or failed
     */
    bool callAction(const std::string &a);

    /**
     * This function wraps the arm-specific and task-specific actions.
     * For this reason, it has been implemented as virtual because it depends on
     * the child class.
     *
     * @param  s the state of the system BEFORE starting the action (when this
     *           method is called the state has been already updated to WORKING,
     *           so there is no way for the controller to recover it a part from
     *           this)
     * @param  a the action to do
     * @return   true/false if success/failure
     */
    virtual bool doAction(int s, std::string a);

    /**
     * Checks if an action is available in the database
     * @param             a the action to check for
     * @param  insertAction flag to know if the method has been called
     *                      inside insertAction (it only removes the
     *                      ROS_ERROR if the action is not in the DB)
     * @return   true/false if the action is available in the database
     */
    bool isActionInDB(const std::string &a, bool insertAction=false);

    /**
     * Prints the action database to screen.
     */
    void printActionDB();

    /**
     * Converts the action database to a string.
     * @return the list of allowed actions, separated by a comma.
     */
    std::string actionDBToString();

public:
    /**
     * Constructor
     */
    ArmCtrl(std::string _name, std::string _limb,
            bool _no_robot = false, bool _use_forces = true, bool _use_trac_ik = true);

    /*
     * Destructor
     */
    ~ArmCtrl();

    /**
     * Callback for the service that requests actions
     * @param  req the action request
     * @param  res the action response (res.success either true or false)
     * @return     true always :)
     */
    bool serviceCb(baxter_collaboration::DoAction::Request  &req,
                   baxter_collaboration::DoAction::Response &res);

    /**
     * Callback for the service that lets the two limbs interact
     * @param  req the action request
     * @param  res the action response (res.success either true or false)
     * @return     true always :)
     */
    virtual bool serviceOtherLimbCb(baxter_collaboration::AskFeedback::Request  &req,
                                    baxter_collaboration::AskFeedback::Response &res);

    /**
     * Publishes the high-level state of the controller (to be shown in the baxter display)
     */
    void publishState();

    /* Self-explaining "setters" */
    void setSubState(std::string _state) { sub_state =  _state; };
    void setObjectID(int _obj)           { object_id =    _obj; };
    void setAction(std::string _action);

    void setState(int _state);

    /* Self-explaining "getters" */
    std::string getSubState() { return sub_state; };
    std::string getAction()   { return    action; };
    int         getObjectID() { return object_id; };
    bool        getInternalRecovery() { return internal_recovery; };
};

#endif
