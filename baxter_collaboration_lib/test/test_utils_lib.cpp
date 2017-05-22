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

// sarim's tests
TEST(UtilsLib, toStringConversions)
{
    int testInt;
    testInt = 10;
    EXPECT_EQ("10", toString(testInt));

    std::vector<int> testIntVector;
    testIntVector.push_back(10);
    testIntVector.push_back(20);
    EXPECT_EQ("[10, 20]", toString(testIntVector)); 

    double testDouble;
    testDouble = 10.123;
    EXPECT_EQ("10.123", toString(testDouble));

    std::vector<double> testDoubleVector;
    testDoubleVector.push_back(10.123);
    testDoubleVector.push_back(20.123);
    EXPECT_EQ("[10.123, 20.123]", toString(testDoubleVector));

    geometry_msgs::Pose testPose;
    testPose.position.x = 0; testPose.position.y = 5; testPose.position.z = 10;
    testPose.orientation.x = 15; testPose.orientation.y = 20; testPose.orientation.z = 25; 
    testPose.orientation.w = 30;
    EXPECT_EQ("{position:{x: 0, y: 5, z: 10}, orientation:{x: 15, y: 20, z: 25, w: 30}}", toString(testPose));
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
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
