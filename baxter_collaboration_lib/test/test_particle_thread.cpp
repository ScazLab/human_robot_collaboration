#include <gtest/gtest.h>

#include "robot_utils/particle_thread.h"

using namespace std;

// Declare a test
TEST(ParticleThreadTest, testConstructor)
{
    ParticleThreadImpl pt("pt", 50, true);

    EXPECT_EQ   (pt.getName(), "pt");
    EXPECT_EQ   (pt.getRate(),   50);
    EXPECT_TRUE (pt.isRunning());
    EXPECT_FALSE(pt.isClosing());

    ros::Duration(0.5).sleep();
    pt.stop();
    EXPECT_FALSE(pt.isRunning());
    EXPECT_FALSE(pt.isClosing());
    EXPECT_EQ   (pt.getCurrPoint(), Eigen::Vector3d(1.0, 1.0, 1.0));

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_thread_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
