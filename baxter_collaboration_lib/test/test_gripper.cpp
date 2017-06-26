#include <gtest/gtest.h>
#include "robot_interface/gripper.h"
#include <iostream>

using namespace std;

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
    std::string cmd_value;

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
        cmd_value = msg.command;
    }

    // Publishes properties of the suction cup gripper
    void sendPropertiesSuction()
    {
        //publish the properties then make sure they are correctly set in the tests
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

// Waits for published command to be received
void waitTime(std::string _expected_cmd, std::string &_cmd)
{
    ros::Rate r(100);
    int flag = 0;
    while (flag == 0)
    {
        if (_expected_cmd == _cmd) 
        { 
            flag = 1; 
        }
        r.sleep();
    }
}

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
    EXPECT_FALSE(     gr.is_gripping());
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

TEST(GripperTest, testElectricCalibration)
{ 
   std::string    limb = "left";
   bool       use_robot =  true;

   gripperTester gt("gripper", "left");
   gt.sendPropertiesElectric();

   Gripper gr(limb, use_robot);
   gr.calibrate();
   
   // The expected command we want to receive
   std::string calibrate_cmd = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;

   // Wait so that the gripper has time to publish the command we want to test
   waitTime(calibrate_cmd, gt.cmd_value);

   EXPECT_EQ(calibrate_cmd, gt.cmd_value);
}

TEST(GripperTest, testElectricMethods)
{
   std::string    limb = "left";
   bool       use_robot =  true;

   gripperTester gt("gripper", "left");
   gt.sendPropertiesElectric();

   Gripper gr(limb, use_robot);

   std::string expected_cmd = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;

   // Calling close() should first calibrate the gripper, and then grip 
   gr.close();
   EXPECT_EQ(expected_cmd, gt.cmd_value);
   
   // Wait so that the gripper has time to publish the command we want to test
   waitTime(expected_cmd, gt.cmd_value);

   expected_cmd = baxter_core_msgs::EndEffectorCommand::CMD_GO;
   EXPECT_EQ(expected_cmd, gt.cmd_value);

   // Test open() method
   expected_cmd = baxter_core_msgs::EndEffectorCommand::CMD_GO;
   gr.open();

   // Wait so that the gripper has time to publish the command we want to test
   waitTime(expected_cmd, gt.cmd_value);
   EXPECT_EQ(expected_cmd, gt.cmd_value);
}

TEST(GripperTest, testSuctionMethods)
{
   std::string    limb = "left";
   bool       use_robot =  true;

   gripperTester gt("gripper", "left");
   gt.sendPropertiesSuction();

   Gripper gr(limb, use_robot);

   // The expected command we want to receive
   std::string expected_cmd = baxter_core_msgs::EndEffectorCommand::CMD_CONFIGURE;

   // Wait so that the gripper has time to publish the command we want to test
   waitTime(expected_cmd, gt.cmd_value);

   EXPECT_EQ(expected_cmd, gt.cmd_value);

   gr.close();
   expected_cmd = baxter_core_msgs::EndEffectorCommand::CMD_GO;

   // Wait so that the gripper has time to publish the command we want to test
   waitTime(expected_cmd, gt.cmd_value);

   EXPECT_EQ(expected_cmd, gt.cmd_value);

   //Test open() method and close() method (called in open() definition)
   gr.open();
   expected_cmd = baxter_core_msgs::EndEffectorCommand::CMD_RELEASE;

   // Wait so that the gripper has time to publish the command we want to test
   waitTime(expected_cmd, gt.cmd_value);

   EXPECT_EQ(expected_cmd, gt.cmd_value);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
