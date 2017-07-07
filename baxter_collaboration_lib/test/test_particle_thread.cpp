#include <gtest/gtest.h>

#include "robot_utils/particle_thread.h"

using namespace std;

TEST(ParticleThreadTest, testConstructor)
{
    ParticleThreadImpl pt("pt", 50);

    EXPECT_EQ   (pt.getName(), "pt");
    EXPECT_EQ   (pt.getRate(),   50);

    EXPECT_FALSE(pt.isRunning());
    EXPECT_FALSE(pt.isClosing());
    EXPECT_TRUE (pt.start());
    EXPECT_FALSE(pt.start());

    EXPECT_TRUE (pt.isRunning());
    EXPECT_FALSE(pt.isClosing());

    ros::Duration(0.1).sleep();
    EXPECT_TRUE (pt.stop());
    EXPECT_FALSE(pt.isRunning());
    EXPECT_FALSE(pt.isClosing());
    EXPECT_EQ   (pt.getCurrPoint(), Eigen::Vector3d(1.0, 1.0, 1.0));
}

TEST(ParticleThreadTest, testLinearPoint)
{
    LinearPointParticle lpp;

    EXPECT_FALSE(lpp.start());
    EXPECT_TRUE(lpp.setupParticle(Eigen::Vector3d(0.0, 0.0, 0.0),
                                  Eigen::Vector3d(0.0, 0.0, 0.1), 0.2));
    EXPECT_TRUE (lpp.start());

    ros::Duration(0.51).sleep();
    EXPECT_EQ   (lpp.getCurrPoint(), Eigen::Vector3d(0.0, 0.0, 0.1));
    EXPECT_TRUE (lpp.stop());
    EXPECT_FALSE(lpp.start());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_thread_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
