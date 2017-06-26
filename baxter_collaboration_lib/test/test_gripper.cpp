#include <gtest/gtest.h>
#include "robot_interface/gripper.h"
#include <iostream>

using namespace              std;
using namespace baxter_core_msgs;

class gripperTester
{
    ros::NodeHandle       nh;
    std::string         name;
    std::string         limb;
    ros::Publisher  prop_pub;
    ros::Publisher state_pub;
    ros::Subscriber  cmd_sub; 
    std::mutex       mtx_cmd; 

    std::string    cmd_value;

    ros::AsyncSpinner spinner;

    void commandCb(baxter_core_msgs::EndEffectorCommand msg)
    {
        setCmdValue(msg.command);
    }

    void setCmdValue(std::string _value)
    {
        std::lock_guard<std::mutex> lock(mtx_cmd);
        cmd_value = _value;
    }

    std::string getCmdValue()
    {
        std::lock_guard<std::mutex> lock(mtx_cmd);
        return cmd_value;
    }

public:
    explicit gripperTester(std::string _name, std::string _limb) :
                                name(_name), limb(_limb), spinner(4)
    {
        prop_pub = nh.advertise<EndEffectorProperties>("/robot/end_effector/" +
                                        limb + "_gripper/properties", 1, true);

        state_pub = nh.advertise<EndEffectorState>("/robot/end_effector/" +
                                         limb + "_gripper/state", 1, true);

        cmd_sub = nh.subscribe("/robot/end_effector/" + limb + "_gripper/command",
                              SUBSCRIBER_BUFFER, &gripperTester::commandCb, this);

        spinner.start();

        // sleep to wait for the publishers to be ready
        ros::Duration(.5).sleep();
    }

    // Publishes properties of the suction cup gripper
    void sendPropSuction()
    {
        //publish the properties then make sure they are correctly set in the tests
        EndEffectorProperties prop;
        prop.ui_type = 1u;
        prop_pub.publish(prop);
    }

    // Publishes the properties of the electric gripper
    void sendPropElectric()
    {
        EndEffectorProperties prop;
        prop.ui_type = 2u;
        prop_pub.publish(prop);
    }

    // Publishes the state of the gripper
    void sendState()
    {
        //publish the state then make sure it's correctly set in the tests
        EndEffectorState state;

        state.calibrated = EndEffectorState::STATE_TRUE;
        state.enabled    = EndEffectorState::STATE_TRUE;
        state.error      = EndEffectorState::STATE_FALSE;
        state.gripping   = EndEffectorState::STATE_FALSE;
        state.missed     = EndEffectorState::STATE_FALSE;
        state.ready      = EndEffectorState::STATE_TRUE;
        state.moving     = EndEffectorState::STATE_TRUE;

        state_pub.publish(state);
    }

    // Waits for published command to be received
    bool wait(std::string _expected_cmd)
    {
        ros::Rate r(100);

        while (ros::ok())
        {
            if (_expected_cmd == getCmdValue()) { return true; }
            r.sleep();
        }

        return false;
    }
};

TEST(GripperTest, testPropertiesAndStateSubscriber)
{
    std::string    limb = "left";

    gripperTester gt("gripper", "left");
    gt.sendPropSuction();
    gt.sendState();

    Gripper gr(limb);

    EXPECT_TRUE(      gr.is_enabled());
    EXPECT_TRUE(   gr.is_calibrated());
    EXPECT_TRUE(gr.is_ready_to_grip());
    EXPECT_TRUE(      gr.is_sucking());
    EXPECT_FALSE(    gr.is_gripping());
    EXPECT_FALSE(      gr.has_error());

    EXPECT_EQ("left", gr.getGripperLimb());

    EndEffectorState _state = gr.getGripperState();

    EXPECT_EQ(_state.calibrated, EndEffectorState::STATE_TRUE);
    EXPECT_EQ(_state.enabled   , EndEffectorState::STATE_TRUE);
    EXPECT_EQ(_state.error     , EndEffectorState::STATE_FALSE);
    EXPECT_EQ(_state.gripping  , EndEffectorState::STATE_FALSE);
    EXPECT_EQ(_state.missed    , EndEffectorState::STATE_FALSE);
    EXPECT_EQ(_state.ready     , EndEffectorState::STATE_TRUE);
    EXPECT_EQ(_state.moving    , EndEffectorState::STATE_TRUE);
}

TEST(GripperTest, testDefaultStates)
{
   std::string    limb = "left";

   Gripper gr(limb);
   EndEffectorState _state;
   _state = gr.getGripperState();

  EXPECT_EQ(_state.calibrated, EndEffectorState::STATE_UNKNOWN);
  EXPECT_EQ(_state.enabled   , EndEffectorState::STATE_UNKNOWN);
  EXPECT_EQ(_state.error     , EndEffectorState::STATE_UNKNOWN);
  EXPECT_EQ(_state.gripping  , EndEffectorState::STATE_UNKNOWN);
  EXPECT_EQ(_state.missed    , EndEffectorState::STATE_UNKNOWN);
  EXPECT_EQ(_state.ready     , EndEffectorState::STATE_UNKNOWN);
  EXPECT_EQ(_state.moving    , EndEffectorState::STATE_UNKNOWN);
}

TEST(GripperTest, testElectricCalibration)
{
   std::string    limb = "left";

   gripperTester gt("gripper", "left");
   gt.sendPropElectric();

   Gripper gr(limb);
   gr.calibrate();

   // The expected command we want to receive
   std::string calibrate_cmd = EndEffectorCommand::CMD_CALIBRATE;

   // Wait so that the gripper has time to publish the command we want to test
   EXPECT_TRUE(gt.wait(calibrate_cmd));
}

TEST(GripperTest, testElectricMethods)
{
   std::string    limb = "left";

   gripperTester gt("gripper", "left");
   gt.sendPropElectric();

   Gripper gr(limb);

   std::string expected_cmd = EndEffectorCommand::CMD_CALIBRATE;

   // Calling close() should first calibrate the gripper, and then grip
   gr.close();
   EXPECT_TRUE(gt.wait(expected_cmd));

   // Wait so that the gripper has time to publish the command we want to test
   expected_cmd = EndEffectorCommand::CMD_GO;
   EXPECT_TRUE(gt.wait(expected_cmd));

   // Test open() method
   expected_cmd = EndEffectorCommand::CMD_GO;
   gr.open();

   // Wait so that the gripper has time to publish the command we want to test
    EXPECT_TRUE(gt.wait(expected_cmd));
}

TEST(GripperTest, testSuctionMethods)
{
   std::string    limb = "left";

   gripperTester gt("gripper", "left");
   gt.sendPropSuction();

   Gripper gr(limb);

   // The expected command we want to receive
   std::string expected_cmd = EndEffectorCommand::CMD_CONFIGURE;

   // Wait so that the gripper has time to publish the command we want to test
   EXPECT_TRUE(gt.wait(expected_cmd));

   gr.close();
   expected_cmd = EndEffectorCommand::CMD_GO;

   // Wait so that the gripper has time to publish the command we want to test
   EXPECT_TRUE(gt.wait(expected_cmd));

   //Test open() method and close() method (called in open() definition)
   gr.open();
   expected_cmd = EndEffectorCommand::CMD_RELEASE;

   // Wait so that the gripper has time to publish the command we want to test
   EXPECT_TRUE(gt.wait(expected_cmd));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
