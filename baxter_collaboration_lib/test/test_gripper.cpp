#include <gtest/gtest.h>
#include "robot_interface/gripper.h"
#include <iostream>

using namespace std;

// subscribes to the grippers command
// and publishes to the grippers state
class gripperTester
{
    ros::NodeHandle             nh;
    std::string               name;
    std::string               limb;
    ros::Publisher        prop_pub;
    ros::Publisher       state_pub;
    ros::Subscriber        cmd_sub; 
    std::mutex    mutex_properties; 
    ros::AsyncSpinner      spinner;

public:
    explicit gripperTester(std::string _name, std::string _limb) : 
                                name(_name), limb(_limb), spinner(1)
    {
        prop_pub = nh.advertise<baxter_core_msgs::EndEffectorProperties>(
                                 "/robot/end_effector/" + limb + "_gripper/properties", 1, true);

        state_pub = nh.advertise<baxter_core_msgs::EndEffectorState>(
                                 "/robot/end_effector/" + limb + "_gripper/state", 1, true);
   
        cmd_sub = nh.subscribe("/robot/end_effector/" + limb + "_gripper/command",
                                 SUBSCRIBER_BUFFER, &gripperTester::commandCb, this);
        
        spinner.start();

        // sleep to wait for the publishers to be ready
        ros::Duration(.5).sleep();
    }

    void commandCb(const baxter_core_msgs::EndEffectorCommand &msg)
    {
       // std::string _msg = &msg.command;
       // std::string _cmd;


        baxter_core_msgs::EndEffectorState state;
        
        state.calibrated = baxter_core_msgs::EndEffectorState::STATE_TRUE;
        state.enabled    = baxter_core_msgs::EndEffectorState::STATE_UNKNOWN;
        state.error      = baxter_core_msgs::EndEffectorState::STATE_UNKNOWN;
        state.gripping   = baxter_core_msgs::EndEffectorState::STATE_UNKNOWN;
        state.missed     = baxter_core_msgs::EndEffectorState::STATE_UNKNOWN;
        state.ready      = baxter_core_msgs::EndEffectorState::STATE_UNKNOWN;
        state.moving     = baxter_core_msgs::EndEffectorState::STATE_UNKNOWN;
       

        baxter_core_msgs::EndEffectorCommand _cmd;
        _cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;

        if (msg.command == _cmd.command)
        {
            state.calibrated = baxter_core_msgs::EndEffectorState::STATE_TRUE;
        } 

        state_pub.publish(state);
    }

    // Publishes properties of the suction cup gripper
    void sendPropertiesSuction()
    {
        //publish the properties then make sure they are correctly set in the tests
        //std::lock_guard<std::mutex> lock(mutex_properties);
        baxter_core_msgs::EndEffectorProperties prop;
        prop.ui_type = 1u;
        prop_pub.publish(prop);   
    }

    // Publishes the properties of the electric gripper
    void sendPropertiesElectric()
    {
        baxter_core_msgs::EndEffectorProperties prop;
        prop.ui_type = 2u;
        prop_pub.publish(prop); 
    }

    // Publishes the state of the gripper
    void sendState()
    {
        //publish the state then make sure it's correctly set in the tests
        baxter_core_msgs::EndEffectorState state;

        state.calibrated = baxter_core_msgs::EndEffectorState::STATE_TRUE;
        state.enabled    = baxter_core_msgs::EndEffectorState::STATE_TRUE;
        state.error      = baxter_core_msgs::EndEffectorState::STATE_FALSE;
        state.gripping   = baxter_core_msgs::EndEffectorState::STATE_FALSE;
        state.missed     = baxter_core_msgs::EndEffectorState::STATE_FALSE;
        state.ready      = baxter_core_msgs::EndEffectorState::STATE_TRUE;
        state.moving     = baxter_core_msgs::EndEffectorState::STATE_TRUE;

        state_pub.publish(state);
    }
};

TEST(GripperTest, testPropertiesAndStateSubscriber)
{
    std::string    limb = "left";
    bool use_robot =  true;

    gripperTester gt("gripper" , "left");
    gt.sendPropertiesSuction();
    gt.sendState();

    Gripper gr(limb, use_robot);

    EXPECT_TRUE(      gr.is_enabled());
    EXPECT_TRUE(   gr.is_calibrated());  
    EXPECT_TRUE(gr.is_ready_to_grip());
    EXPECT_TRUE(      gr.is_sucking());
    EXPECT_TRUE(     gr.is_gripping());
    EXPECT_FALSE(      gr.has_error());
    
    EXPECT_EQ("left", gr.getGripperLimb());

    baxter_core_msgs::EndEffectorState _state = gr.getGripperState();

    EXPECT_EQ(_state.calibrated , baxter_core_msgs::EndEffectorState::STATE_TRUE);
    EXPECT_EQ(_state.enabled    , baxter_core_msgs::EndEffectorState::STATE_TRUE);
    EXPECT_EQ(_state.error      , baxter_core_msgs::EndEffectorState::STATE_FALSE);
    EXPECT_EQ(_state.gripping   , baxter_core_msgs::EndEffectorState::STATE_FALSE);
    EXPECT_EQ(_state.missed     , baxter_core_msgs::EndEffectorState::STATE_FALSE);
    EXPECT_EQ(_state.ready      , baxter_core_msgs::EndEffectorState::STATE_TRUE);
    EXPECT_EQ(_state.moving     , baxter_core_msgs::EndEffectorState::STATE_TRUE);    
}

TEST(GripperTest, testDefaultStates)
{ 
   std::string    limb = "left";
   bool use_robot =  true;

   Gripper gr(limb, use_robot);
   baxter_core_msgs::EndEffectorState _state;
   _state = gr.getGripperState();

  EXPECT_EQ(_state.calibrated , baxter_core_msgs::EndEffectorState::STATE_UNKNOWN);
  EXPECT_EQ(_state.enabled    , baxter_core_msgs::EndEffectorState::STATE_UNKNOWN);
  EXPECT_EQ(_state.error      , baxter_core_msgs::EndEffectorState::STATE_UNKNOWN);
  EXPECT_EQ(_state.gripping   , baxter_core_msgs::EndEffectorState::STATE_UNKNOWN);
  EXPECT_EQ(_state.missed     , baxter_core_msgs::EndEffectorState::STATE_UNKNOWN);
  EXPECT_EQ(_state.ready      , baxter_core_msgs::EndEffectorState::STATE_UNKNOWN);
  EXPECT_EQ(_state.moving     , baxter_core_msgs::EndEffectorState::STATE_UNKNOWN);
    
} 

// TEST(GripperTest, testsetGripperState)
// { 
//     std::string    limb = "left";
//     bool use_robot =  true;

//     Gripper gr(limb, use_robot);

//     baxter_core_msgs::EndEffectorState new_state;
//     new_state.calibrated = baxter_core_msgs::EndEffectorState::STATE_TRUE;
//     new_state.enabled    = baxter_core_msgs::EndEffectorState::STATE_TRUE;
//     new_state.error      = baxter_core_msgs::EndEffectorState::STATE_FALSE;
//     new_state.gripping   = baxter_core_msgs::EndEffectorState::STATE_FALSE;
//     new_state.missed     = baxter_core_msgs::EndEffectorState::STATE_FALSE;
//     new_state.ready      = baxter_core_msgs::EndEffectorState::STATE_TRUE;
//     new_state.moving     = baxter_core_msgs::EndEffectorState::STATE_TRUE;

//     gr.setGripperState(new_state);
//     baxter_core_msgs::EndEffectorState _state = gr.getGripperState();

//     EXPECT_EQ(_state.calibrated , baxter_core_msgs::EndEffectorState::STATE_TRUE);
//     EXPECT_EQ(_state.enabled    , baxter_core_msgs::EndEffectorState::STATE_TRUE);
//     EXPECT_EQ(_state.error      , baxter_core_msgs::EndEffectorState::STATE_FALSE);
//     EXPECT_EQ(_state.gripping   , baxter_core_msgs::EndEffectorState::STATE_FALSE);
//     EXPECT_EQ(_state.missed     , baxter_core_msgs::EndEffectorState::STATE_FALSE);
//     EXPECT_EQ(_state.ready      , baxter_core_msgs::EndEffectorState::STATE_TRUE);
//     EXPECT_EQ(_state.moving     , baxter_core_msgs::EndEffectorState::STATE_TRUE);

// }
TEST(GripperTest, testCalibration)
{ 
   std::string    limb = "left";
   bool       use_robot =  true;

   Gripper gr(limb, use_robot);
   gripperTester gt("gripper", "left");

   gt.sendPropertiesElectric();

   gr.calibrate();
   baxter_core_msgs::EndEffectorState _state = gr.getGripperState();

   EXPECT_TRUE(_state.calibrated == baxter_core_msgs::EndEffectorState::STATE_TRUE);

//   gr.clearCalibration();
//   _state = gr.getGripperState();
    
//   EXPECT_EQ(_state.calibrated , baxter_core_msgs::EndEffectorState::STATE_FALSE);
    
}

// TEST(GripperTest, testwithgts)
// {    
//     std::string    limb = "left";
//     bool use_robot =  true;

//     Gripper gr(limb, use_robot);
//     gripperTester gt("gripper" , "left");



// } 


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
