#ifndef __ARM_CONTROLLER_H__
#define __ARM_CONTROLLER_H__

#include <map>

#include "robot_interface/robot_interface.h"
#include "robot_interface/gripper.h"

#include "baxter_collaboration/DoAction.h"
#include "baxter_collaboration/AskFeedback.h"
#include "baxter_collaboration/ArmState.h"

#define HAND_OVER_START  "handover_start"
#define HAND_OVER_READY  "handover_ready"
#define HAND_OVER_DONE   "handover_done"
#define HAND_OVER_WAIT   "handover_wait"

class ArmCtrl : public RobotInterface, public Gripper
{
private:
    std::string      name;
    std::string sub_state;

    std::string    action;
    int         marker_id;

    typedef bool(ArmCtrl::*f_action)();
    std::map <std::string, f_action> action_db;

    ros::ServiceServer service;
    ros::ServiceServer service_other_limb;

    ros::Publisher     state_pub;

protected:
    /**
     * Provides basic functionalities for the object, such as a goHome and releaseObject.
     * For deeper, class-specific specialization, please modify doAction() instead.
     */
    void InternalThreadEntry();

    /**
     * This function implements the action. It is child-specific, and for this reason
     * it is virtual.
     * @param  s the state of the system before starting the action
     * @param  a the action to do
     * @return   true/false if success/failure
     */
    virtual bool doAction(int s, std::string a) = 0;

    /**
     * Recovers from errors during execution. It provides a basic interface,
     * but it is advised to specialize this function in the ArmCtrl's children.
     */
    void recoverFromError();

    /**
     * Hovers above table at a specific x-y position.
     * @param  height the z-axis value of the end-effector position
     * @return        true/false if success/failure
     */
    bool hoverAboveTable(double height, std::string mode="loose",
                                    bool disable_coll_av = false);


    virtual bool hoverAboveTableStrict(bool disable_coll_av = false) = 0;

    /**
     * Goes to the home position
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
     * Adds an action to the action database
     * @param  a the action to be removed
     * @param  f a pointer to the action, in the form bool action()
     */
    void insertAction(const std::string &a, ArmCtrl::f_action f);

    /**
     * Removes an action from the database
     * @param a the action to be removed
     */
    void removeAction(const std::string &a);

    /**
     * Calls an action from the action database
     * @param    a the action to take
     * @return   true/false if the action called was successful or failed
     */
    bool callAction(const std::string &a);

public:
    // CONSTRUCTOR
    ArmCtrl(std::string _name, std::string _limb, bool no_robot = false);

    // DESTRUCTOR
    ~ArmCtrl();

    void cuffOKCb(const baxter_core_msgs::DigitalIOState& msg);

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

    void publishState();

    /* Self-explaining "setters" */
    void setName(std::string _name)      { name      =   _name; };
    void setSubState(std::string _state) { sub_state =  _state; };
    void setMarkerID(int _id)            { marker_id =     _id; };
    void setAction(std::string _action);

    void setState(int _state);

    /* Self-explaining "getters" */
    std::string getName()     { return      name; };
    std::string getSubState() { return sub_state; };
    std::string getAction()   { return    action; };
    int         getMarkerID() { return marker_id; };
    std::string getObjName();
};

#endif
