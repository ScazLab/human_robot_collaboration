#include <gtest/gtest.h>

#include "robot_interface/gripper.h"

using namespace std;

TEST(GripperTest, testConstructorDefaultValues)
{
	string 	limb = "left";
	bool 	use_robot = false;

	Gripper gr(limb, use_robot);

	EXPECT_FALSE(gr.is_enabled());
	EXPECT_FALSE(gr.is_calibrated());
	EXPECT_FALSE(gr.is_ready_to_grip());
	EXPECT_FALSE(gr.has_error());

	EXPECT_TRUE(gr.is_sucking());
	EXPECT_TRUE(gr.is_gripping());

	EXPECT_EQ("left", gr.getGripperLimb());
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
