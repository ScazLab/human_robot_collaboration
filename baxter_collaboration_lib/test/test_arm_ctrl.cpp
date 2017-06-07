#include <gtest/gtest.h>
#include "robot_interface/arm_ctrl.h"

using namespace std;

TEST(ArmControlTest, testConstructorDefaultValues)
{
    ArmCtrl ac ("robot", "left");

    EXPECT_EQ   ("robot", ac.getName());
    EXPECT_EQ   ( "left", ac.getLimb());
    EXPECT_TRUE (ac.isRobotUsed());
    EXPECT_EQ   (100.0, ac.getCtrlFreq());
    EXPECT_TRUE (ac.useForces());
    EXPECT_TRUE (ac.useTracIK());
    EXPECT_FALSE(ac.useCartCtrl());
    EXPECT_FALSE(ac.isExperimental());
    EXPECT_FALSE(ac.isCtrlRunning());
    EXPECT_EQ   (START, int(ac.getState()));
}

TEST(ArmControlTest, testConstructorCustomValues)
{
    std::string   name = "robot";
    std::string   limb =  "left";
    bool    use_forces =   false;
    bool     use_robot =   false;
    bool   use_trac_ik =   false;
    bool use_cart_ctrl =   false;

    ArmCtrl ac(name, limb, use_robot, use_forces, use_trac_ik, use_cart_ctrl);

    EXPECT_EQ   (name, ac.getName());
	EXPECT_EQ   (limb, ac.getLimb());
    EXPECT_FALSE(ac.isRobotUsed());
    EXPECT_FALSE(ac.useForces());
    EXPECT_FALSE(ac.useTracIK());
    EXPECT_FALSE(ac.useCartCtrl());
    EXPECT_TRUE (ac.getInternalRecovery());
}

TEST(ArmControlTest, testPublicSetterGetterMethods)
{
    std::string name = "robot";
    std::string limb =  "left";
    //overrides default constructor values for these variables; these
    // values are the opposite of the default values
    bool    use_forces =  true;
    bool     use_robot = false;
    bool   use_trac_ik =  true;
    bool use_cart_ctrl = false;

    ArmCtrl ac(name, limb, use_robot, use_forces, use_trac_ik, use_cart_ctrl);

    //Setter Methods
    int obj = 5;               //some random obj id number
    vector<int> objs (4, 100); //some random value vector

    ac.setState     (       DONE);
    ac.setObjectID  (        obj);
    ac.setObjectIDs (       objs);
    ac.setAction    (ACTION_HOME);

    //Getter methods
    EXPECT_EQ   (       DONE, int(ac.getState()));
    EXPECT_EQ   (ACTION_HOME,     ac.getAction());
    EXPECT_EQ   (         "",     ac.getPrevAction());
    EXPECT_EQ   (        obj,     ac.getObjectID());
    EXPECT_EQ   (       objs,     ac.getObjectIDs());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_control_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
