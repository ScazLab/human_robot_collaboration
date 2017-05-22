#include <gtest/gtest.h>

#include "robot_interface/gripper.h"

using namespace std;

TEST(GripperTest, testConstructorDefaultValues)
{
	string 	limb = "left";
	bool 	use_robot = false;

	Gripper gr(limb, use_robot);


	EXPECT_EQ(false, gr.is_enabled());
	EXPECT_EQ(false, gr.is_calibrated());
	EXPECT_EQ(false, gr.is_ready_to_grip());
	EXPECT_EQ(false, gr.has_error());

	EXPECT_EQ(true, gr.is_sucking());
	EXPECT_EQ(true, gr.is_gripping());

	EXPECT_EQ("left", gr.getGripperLimb());

	//EXPECT_EQ(true, gr.gripObject());
	//EXPECT_EQ(true, gr.releaseObject());

}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
