#include <gtest/gtest.h>

#include "robot_utils/particle_thread.h"

using namespace std;

TEST(ParticleThreadTest, testConstructor)
{
    ros::Time::init();

    ParticleThreadImpl pt("pt", 50);

    EXPECT_EQ   (pt.getName(),"pt");
    EXPECT_EQ   (pt.getRate(),  50);

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
    ros::Time::init();

    LinearPointParticle lpp;

    EXPECT_FALSE(lpp.start());
    EXPECT_TRUE (lpp.setupParticle(Eigen::Vector3d(0.0, 0.0, 0.0),
                                   Eigen::Vector3d(0.0, 0.0, 0.1), 0.2));
    EXPECT_TRUE (lpp.start());

    ros::Duration(0.51).sleep();
    EXPECT_EQ   (lpp.getCurrPoint(), Eigen::Vector3d(0.0, 0.0, 0.1));
    EXPECT_TRUE (lpp.stop());
    EXPECT_FALSE(lpp.start());
}

TEST(ParticleThreadTest, testCircularPoint)
{
    ros::Time::init();

    CircularPointParticle cpp("cpp", 100.0, true);

    EXPECT_FALSE(cpp.start());
    EXPECT_TRUE (cpp.setupParticle(Eigen::Vector3d(0.0, 0.0, 0.0),
                                   Eigen::Vector2d(0.0, 0.0), 0.2, 1.0*M_PI));
    EXPECT_TRUE (cpp.start());

    ros::Duration(2.0).sleep();

    // With a circular trajectory, we will never know where we are,
    // because it never stops. Hence, tests should be much less specific
    // than with a LinearPointParticle class
    EXPECT_FALSE(cpp.getCurrPoint()[0] >  0.2);
    EXPECT_FALSE(cpp.getCurrPoint()[0] < -0.2);
    EXPECT_FALSE(cpp.getCurrPoint()[1] >  0.2);
    EXPECT_FALSE(cpp.getCurrPoint()[1] < -0.2);
    EXPECT_EQ   (cpp.getCurrPoint()[2],   0.0);
    EXPECT_TRUE (cpp.stop());
    EXPECT_FALSE(cpp.start());

    EXPECT_TRUE (cpp.setupParticle(Eigen::Vector3d( 0.0, 0.0, 0.25),
                                   Eigen::Vector2d( 0.0, M_PI/2), 0.25, 2.0*M_PI));
    EXPECT_TRUE (cpp.start());

    ros::Duration(1.0).sleep();

    // With a circular trajectory, we will never know where we are,
    // because it never stops. Hence, tests should be much less specific
    // than with a LinearPointParticle class
    EXPECT_TRUE (cpp.stop());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_thread_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
