#include <gtest/gtest.h>
#include "robot_interface/gripper.h"
#include <iostream>

using namespace              std;
using namespace baxter_core_msgs;

class GripperTester
{
    ros::NodeHandle       nh;
    std::string         name;
    std::string         limb;
    ros::Publisher  prop_pub;
    ros::Publisher state_pub;
    ros::Subscriber  cmd_sub;
    std::mutex       mtx_cmd;

    std::string    cmd_value;

    void commandCb(baxter_core_msgs::EndEffectorCommand msg)
    {
        printf("[gripper_test] Received new command! %s\n", msg.command.c_str());
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
    explicit GripperTester(std::string _name = "gripper_tester",
                           std::string _limb = "left") :
                           name(_name), limb(_limb)
    {
        prop_pub = nh.advertise<EndEffectorProperties>("/robot/end_effector/" +
                                        limb + "_gripper/properties", 1, true);

        state_pub = nh.advertise<EndEffectorState>("/robot/end_effector/" +
                                         limb + "_gripper/state", 1, true);

        cmd_sub = nh.subscribe("/robot/end_effector/" + limb + "_gripper/command",
                              SUBSCRIBER_BUFFER, &GripperTester::commandCb, this);

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
        ros::Time s = ros::Time::now();

        while (ros::ok())
        {
            std::string cv = getCmdValue();

            if (cv != "")   { printf("[gripper_test] Cmd: %s\n", cv.c_str()); }

            if (_expected_cmd == cv)                  { return  true; };
            if ((ros::Time::now() - s).toSec() > 2.0) { return false; };

            ros::spinOnce();
            r.sleep();
        }

        return false;
    }
};

TEST(GripperTest, testDefaultStates)
{
    Gripper gr("left");

    EndEffectorState _state = gr.getGripperState();
    EXPECT_EQ(_state.calibrated, EndEffectorState::STATE_UNKNOWN);
    EXPECT_EQ(_state.enabled   , EndEffectorState::STATE_UNKNOWN);
    EXPECT_EQ(_state.error     , EndEffectorState::STATE_UNKNOWN);
    EXPECT_EQ(_state.gripping  , EndEffectorState::STATE_UNKNOWN);
    EXPECT_EQ(_state.missed    , EndEffectorState::STATE_UNKNOWN);
    EXPECT_EQ(_state.ready     , EndEffectorState::STATE_UNKNOWN);
    EXPECT_EQ(_state.moving    , EndEffectorState::STATE_UNKNOWN);
}

TEST(GripperTest, testPropertiesAndStateSubscriber)
{
    Gripper gr("left");
    GripperTester   gt;

    gt.sendPropSuction();
    gt.sendState();

    // Sleep to wait that ros does its job in sending the messages
    ros::Duration(0.1).sleep();

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

TEST(GripperTest, testElectricCalibration)
{
    Gripper gr("left");
    GripperTester   gt;

    gt.sendPropElectric();

    // Sleep to wait that ros does its job in sending the messages
    ros::Duration(0.1).sleep();

    gr.calibrate();
    // Wait so that the gripper has time to publish the command we want to test
    EXPECT_TRUE(gt.wait(EndEffectorCommand::CMD_CALIBRATE));
}

TEST(GripperTest, testElectricMethods)
{
    Gripper gr("left");
    GripperTester   gt;

    gt.sendPropElectric();

    // Sleep to wait that ros does its job in sending the messages
    ros::Duration(0.1).sleep();

    // Test close() method
    gr.close();
    // Calling close() should first calibrate the gripper, and then grip
    EXPECT_TRUE(gt.wait(EndEffectorCommand::CMD_CALIBRATE));

    // Wait so that the gripper has time to publish the command we want to test
    EXPECT_TRUE(gt.wait(EndEffectorCommand::CMD_GO));

    // Test open() method
    gr.open();
    // Wait so that the gripper has time to publish the command we want to test
    EXPECT_TRUE(gt.wait(EndEffectorCommand::CMD_GO));
}

TEST(GripperTest, testSuctionMethods)
{
    Gripper gr("left");
    GripperTester   gt;

    gt.sendPropSuction();

    // Sleep to wait that ros does its job in sending the messages
    ros::Duration(0.1).sleep();

    // Test close() method
    gr.close();
    // Wait so that the gripper has time to publish the command we want to test
    EXPECT_TRUE(gt.wait(EndEffectorCommand::CMD_GO));

    // Test open() method
    gr.open();
    // Wait so that the gripper has time to publish the command we want to test
    EXPECT_TRUE(gt.wait(EndEffectorCommand::CMD_RELEASE));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
