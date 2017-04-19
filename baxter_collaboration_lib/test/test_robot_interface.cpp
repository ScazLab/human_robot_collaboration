#include <gtest/gtest.h>

#include "robot_interface/robot_interface.h"

using namespace std;

// Declare a test
TEST(RobotInterfaceTest, testConstructorDefaultValues)
{
    RobotInterface ri("robot", "left");

    EXPECT_EQ(ri.isCtrlRunning(), false);
    EXPECT_EQ(START, int(ri.getState()));

    EXPECT_EQ("robot", ri.getName());
    EXPECT_EQ( "left", ri.getLimb());
    EXPECT_EQ(  false, ri.isNoRobot());
    EXPECT_EQ(  100.0, ri.getCtrlFreq());
    EXPECT_EQ(   true, ri.useForces());
    EXPECT_EQ(   true, ri.useTracIK());
    EXPECT_EQ(   true, ri.useCartCtrl());
    EXPECT_EQ(  false, ri.isExperimental());

    ri.setTracIK(false);
    EXPECT_EQ(false, ri.useTracIK());
}

TEST(RobotInterfaceTest, testConstructorCustomValues)
{
    string          name = "robot";
    string          limb =  "left";
    bool        no_robot =    true;
    double     ctrl_freq =    50.0;
    bool      use_forces =   false;
    bool     use_trac_ik =   false;
    bool   use_cart_ctrl =   false;
    bool is_experimental =   false;
    RobotInterface ri(name, limb, no_robot, ctrl_freq, use_forces,
                      use_trac_ik, use_cart_ctrl, is_experimental);

    EXPECT_EQ(ri.isCtrlRunning(), false);
    EXPECT_EQ(START, int(ri.getState()));

    EXPECT_EQ(           name, ri.getName());
    EXPECT_EQ(           limb, ri.getLimb());
    EXPECT_EQ(       no_robot, ri.isNoRobot());
    EXPECT_EQ(      ctrl_freq, ri.getCtrlFreq());
    EXPECT_EQ(     use_forces, ri.useForces());
    EXPECT_EQ(    use_trac_ik, ri.useTracIK());
    EXPECT_EQ(  use_cart_ctrl, ri.useCartCtrl());
    EXPECT_EQ(is_experimental, ri.isExperimental());

    use_trac_ik = true;
    ri.setTracIK(use_trac_ik);
    EXPECT_EQ(use_trac_ik, ri.useTracIK());

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_interface_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
