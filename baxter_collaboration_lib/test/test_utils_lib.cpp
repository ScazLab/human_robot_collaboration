#include <gtest/gtest.h>

#include "robot_utils/utils.h"

using namespace std;

// Declare a test
TEST(UtilsLib, geometry_msgsPointsOperators)
{
    geometry_msgs::Point a, b;

    a.x = 0; a.y = 0; a.z = 10;
    EXPECT_EQ(norm(a), 10);

    a.x = 3; a.y = 4; a.z =  0;
    EXPECT_EQ(norm(a), 5);

    b.x = 2; b.y = 3; b.z = -1;
    a = a - 1;
    EXPECT_TRUE(a == b);
    EXPECT_EQ(norm(b), norm(a));

    a = a + 1;
    EXPECT_TRUE((a-b) == (b-a)*(-1.0));

    EXPECT_TRUE(a==a);
    EXPECT_TRUE(a==(a + 2 - 2));
    EXPECT_TRUE(a==(a - 2 + 2));
    EXPECT_TRUE(a==(a / 2 * 2));
    EXPECT_TRUE(a==(a * 2 / 2));

    // Test assignment operator
    a = b;
    EXPECT_TRUE(a==b);

    // Test dot product
    EXPECT_EQ(dot(a,a)/(norm(a)*norm(a)),1);
    EXPECT_EQ(dot(a,a*-1)/(norm(a)*norm(a)),-1);

    geometry_msgs::Point o, d, c;
    o.x = 0; o.y =  0; o.z =  0;
    d.x = 4; d.y = 12; d.z = 18;

    c = d/2;
    EXPECT_EQ(dot(d-o, d-c)/(norm(d-o)*norm(d-c)), +1);
    c = d*2;
    EXPECT_EQ(dot(d-o, d-c)/(norm(d-o)*norm(d-c)), -1);
}

TEST(UtilsLib, State)
{
    vector<int>    s_int;
    vector<string> s_str;

    s_int.push_back(WORKING);
    s_int.push_back(ERROR);
    s_int.push_back(START);
    s_int.push_back(DONE);
    s_int.push_back(KILLED);
    s_int.push_back(RECOVER);
    s_int.push_back(CTRL_RUNNING);
    s_int.push_back(CTRL_DONE);
    s_int.push_back(CTRL_FAIL);
    s_int.push_back(1e4);
    s_int.push_back(-1e3);
    s_int.push_back(-1e-3);

    s_str.push_back("WORKING");
    s_str.push_back("ERROR");
    s_str.push_back("START");
    s_str.push_back("DONE");
    s_str.push_back("KILLED");
    s_str.push_back("RECOVER");
    s_str.push_back("CTRL_RUNNING");
    s_str.push_back("CTRL_DONE");
    s_str.push_back("CTRL_FAIL");
    s_str.push_back("NONE");
    s_str.push_back("NONE");
    s_str.push_back("START");

    for (size_t i = 0; i < s_str.size(); ++i)
    {
        ros::Time::init();
        State s(s_int[i]);

        // ROS_INFO("%s", string(s).c_str());
        EXPECT_EQ(s_str[i], string(s));
    }

}

TEST(UtilsLib, toStringConversions)
{
    int testInt;
    testInt = 10;
    EXPECT_EQ("10", toString(testInt));
    EXPECT_EQ("-100", toString(-100));
    EXPECT_EQ("0", toString(0));
    EXPECT_EQ("0", toString(-0));

    std::vector<int> testIntVector;
    testIntVector.push_back(10);
    testIntVector.push_back(20);
    EXPECT_EQ("[10, 20]", toString(testIntVector));
    testIntVector.push_back(0);
    testIntVector.push_back(-0);
    testIntVector.push_back(-20);
    testIntVector.push_back(-0.00);
    testIntVector.push_back(-10.123);
    EXPECT_EQ("[10, 20, 0, 0, -20, 0, -10]", toString(testIntVector));

    double testDouble;
    testDouble = 10.123;
    EXPECT_EQ("10.123", toString(testDouble));
    EXPECT_EQ("0", toString(0.0000));
    EXPECT_EQ("0.001", toString(.001));
    EXPECT_EQ("-0.01", toString(-0.01));
    EXPECT_EQ("0", toString(-0));
    EXPECT_EQ("0", toString(-00));
    EXPECT_EQ("0", toString(-0.0));
    EXPECT_EQ("0", toString(-.0));

    std::vector<double> testDoubleVector;
    testDoubleVector.push_back(10.123);
    testDoubleVector.push_back(20.123);
    EXPECT_EQ("[10.123, 20.123]", toString(testDoubleVector));
    testDoubleVector.push_back(0);
    testDoubleVector.push_back(-0);
    testDoubleVector.push_back(-0.00);
    testDoubleVector.push_back(-0.01);
    EXPECT_EQ("[10.123, 20.123, 0, 0, 0, -0.01]", toString(testDoubleVector));

    geometry_msgs::Pose testPose;
    testPose.position.x = 0; testPose.position.y = 5; testPose.position.z = 10;
    testPose.orientation.x = 15; testPose.orientation.y = 20; testPose.orientation.z = 25; 
    testPose.orientation.w = 30;
    EXPECT_EQ("{position:{x: 0, y: 5, z: 10}, orientation:{x: 15, y: 20, z: 25, w: 30}}", toString(testPose));
    testPose.position.x = -0.00; testPose.position.y = 5.00; testPose.position.z = -10.123;
    testPose.orientation.x = -0.00; testPose.orientation.y = -0; testPose.orientation.z = 25; 
    testPose.orientation.w = 30.0;
    EXPECT_EQ("{position:{x: 0, y: 5, z: -10.123}, orientation:{x: 0, y: 0, z: 25, w: 30}}", toString(testPose));
}

TEST(UtilsLib, printFunctions)
{
    geometry_msgs::Point testPoint;
    testPoint.x = 0; testPoint.y = 5; testPoint.z = 10;
    EXPECT_EQ("[0, 5, 10]", print(testPoint));

    geometry_msgs::Quaternion testQuaternion;
    testQuaternion.x = 0; testQuaternion.y = 5; testQuaternion.z = 10; testQuaternion.w = 20;
    EXPECT_EQ("[0, 5, 10, 20]", print(testQuaternion));

    geometry_msgs::Pose testPose;
    testPose.position.x = 0; testPose.position.y = 5; testPose.position.z = 10;
    testPose.orientation.x = 15; testPose.orientation.y = 20; testPose.orientation.z = 25; 
    testPose.orientation.w = 30;
    EXPECT_EQ("[[0, 5, 10], [15, 20, 25, 30]]", print(testPose));

    testPoint.x = -10.00; testPoint.y = -0.05; testPoint.z = -0;
    EXPECT_EQ("[-10, -0.05, 0]", print(testPoint));
    testQuaternion.x = -10.00; testQuaternion.y = -.05; testQuaternion.z = -0; testQuaternion.w = 20;
    EXPECT_EQ("[-10, -0.05, 0, 20]", print(testQuaternion));
    testPose.position.x = testPoint.x;
    testPose.position.y = testPoint.y;
    testPose.position.z = testPoint.z;
    testPose.orientation.x = testQuaternion.x;
    testPose.orientation.y = testQuaternion.y;
    testPose.orientation.z = testQuaternion.z;
    testPose.orientation.w = testQuaternion.w;
    EXPECT_EQ("[[-10, -0.05, 0], [-10, -0.05, 0, 20]]", print(testPose));
}

TEST(UtilsLib, misc)
{
    geometry_msgs::Quaternion testQuaternion;
    testQuaternion.x = 0; testQuaternion.y = 5; testQuaternion.z = 10; testQuaternion.w = 20;
    quaternionFromDoubles(testQuaternion, 30, 35, 40, 45);
    EXPECT_EQ(testQuaternion.x, 30);
    EXPECT_EQ(testQuaternion.y, 35);
    EXPECT_EQ(testQuaternion.z, 40);
    EXPECT_EQ(testQuaternion.w, 45);

    quaternionFromDoubles(testQuaternion, -0.00, -10, -100.0123, 1.001);
    EXPECT_EQ(testQuaternion.x, 0);
    EXPECT_EQ(testQuaternion.x, -00.00);
    EXPECT_EQ(testQuaternion.y, -10.0);
    EXPECT_EQ(testQuaternion.z, -100.0123);
    EXPECT_EQ(testQuaternion.w, 01.001);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
