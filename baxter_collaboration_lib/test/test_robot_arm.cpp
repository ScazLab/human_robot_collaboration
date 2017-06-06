#include <gtest/gtest.h>
#include "robot_interface/arm_ctrl.h"

using namespace std;


    //Potentially expand on the testcases; check to see that all edge values have been covered

TEST(ArmControlTest, testConstructorDefaultValues)
{
    std::string _name = "robot"; 
    std::string _limb = "left";
    bool _use_forces = true; 
    //Since we are actually not using the robot, _use_robot needs to be set to 'false' even though by default it should be true
    //Setting this to true yields a KDL chain error
    //Perhaps this could be tested using a simulation?
    bool _use_robot = false; 
    bool _use_trac_ik = true;
    bool _use_cart_ctrl = false;

    ArmCtrl ac (_name, _limb, _use_robot, _use_forces, _use_trac_ik, _use_cart_ctrl);

    EXPECT_EQ(ac.isCtrlRunning(), false);
    EXPECT_EQ(START, int(ac.getState())); 
    EXPECT_EQ("robot", ac.getName());
    EXPECT_EQ("left", ac.getLimb());
    EXPECT_FALSE(ac.isRobotUsed());
    EXPECT_TRUE(ac.useForces()); 
    EXPECT_TRUE(ac.useTracIK());
    EXPECT_FALSE(ac.useCartCtrl()); 
}

TEST(ArmControlTest, testConstructorCustomValues) 
{
    std::string _name = "robot"; 
    std::string _limb = "left";
    //overrides default constructor values for these variables; these values are the opposite of the default values bool _use_forces = false; 
    bool _use_forces = false;
    bool _use_robot = false;      
    bool _use_trac_ik = false;
    bool _use_cart_ctrl = true; 

    ArmCtrl ac (_name, _limb, _use_robot, _use_forces, _use_trac_ik, _use_cart_ctrl);

    EXPECT_EQ("robot", ac.getName());
	EXPECT_EQ("left", ac.getLimb());
    EXPECT_FALSE(ac.isRobotUsed());
    EXPECT_FALSE(ac.useForces());
    EXPECT_FALSE(ac.useTracIK()); 
    EXPECT_TRUE(ac.useCartCtrl());
    EXPECT_TRUE(ac.getInternalRecovery());
}

TEST(ArmControlTest, testPublicSetterGetterMethods)
{
    std::string _name = "robot"; 
    std::string _limb = "left";
    //overrides default constructor values for these variables; these values are the opposite of the default values
    bool _use_forces = true; 
    bool _use_robot = false;      
    bool _use_trac_ik = true;
    bool _use_cart_ctrl = false;

    ArmCtrl ac (_name, _limb, _use_robot, _use_forces, _use_trac_ik, _use_cart_ctrl);

    //Setter Methods
    std::string _state = "teststate";
    int _obj = 5; //some random obj id number
    vector<int> _objs (4, 100); //some random value vector
    std::string _action = "testaction";
    std::string _prev_action = "testprevaction";

    ac.setSubState(_state);
    ac.setObjectID(_obj);
    ac.setObjectIDs(_objs);
    ac.setAction(_action);
    ac.setPrevAction(_prev_action);

    //Getter methods
    EXPECT_EQ("teststate", ac.getSubState()); 
    EXPECT_EQ("testaction", ac.getAction());  
    EXPECT_EQ("testprevaction", ac.getPrevAction());  
    EXPECT_EQ(_obj, ac.getObjectID());
    EXPECT_EQ(_objs, ac.getObjectIDs());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_control_test"); 
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}