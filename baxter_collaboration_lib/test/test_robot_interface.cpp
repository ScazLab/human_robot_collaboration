#include <gtest/gtest.h>

#include "robot_interface/robot_interface.h"

using namespace std;

// Declare a test
TEST(RobotInterfaceTest, testConstructor)
{
    EXPECT_TRUE(true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_interface");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
