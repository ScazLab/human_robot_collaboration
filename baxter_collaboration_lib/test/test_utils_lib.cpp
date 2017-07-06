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
    s_int.push_back(int(-1e-3));

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
    EXPECT_EQ(  "10", toString(  10));
    EXPECT_EQ("-100", toString(-100));
    EXPECT_EQ(   "0", toString(   0));
    EXPECT_EQ(   "0", toString(  -0));

    std::vector<int> int_vec{10, 20};
    EXPECT_EQ("[10, 20]", toString(int_vec));
    int_vec = std::vector<int>{10, 20, 0, -0, -20, int(-0.00), int(-10.123)};
    EXPECT_EQ("[10, 20, 0, 0, -20, 0, -10]", toString(int_vec));

    EXPECT_EQ("10.123", toString(10.123));
    EXPECT_EQ(     "0", toString(0.0000));
    EXPECT_EQ( "0.001",   toString(.001));
    EXPECT_EQ( "-0.01",  toString(-0.01));
    EXPECT_EQ(     "0",     toString(-0));
    EXPECT_EQ(     "0",    toString(-00));
    EXPECT_EQ(     "0",   toString(-0.0));
    EXPECT_EQ(     "0",    toString(-.0));

    std::vector<double> double_vec{10.123, 20.123};
    EXPECT_EQ("[10.123, 20.123]", toString(double_vec));
    double_vec = std::vector<double>{10.123, 20.123, 0, -0, -0.00, -0.01};
    EXPECT_EQ("[10.123, 20.123, 0, 0, 0, -0.01]", toString(double_vec));

    geometry_msgs::Pose pose;
    pose.position.x     = 0; pose.position.y     = 5; pose.position.z    = 10;
    pose.orientation.x = 15; pose.orientation.y = 20; pose.orientation.z = 25;
    pose.orientation.w = 30;
    EXPECT_EQ("{position:{x: 0, y: 5, z: 10}, orientation:{x: 15, y: 20, z: 25, w: 30}}", toString(pose));
    pose.position.x    = -0.00; pose.position.y  = 5.00; pose.position.z = -10.123;
    pose.orientation.x = -0.00; pose.orientation.y = -0; pose.orientation.z   = 25;
    pose.orientation.w = 30.0;
    EXPECT_EQ("{position:{x: 0, y: 5, z: -10.123}, orientation:{x: 0, y: 0, z: 25, w: 30}}", toString(pose));
}

TEST(UtilsLib, printFunctions)
{
    geometry_msgs::Point point;
    point.x = 0; point.y = 5; point.z = 10;
    EXPECT_EQ("[0, 5, 10]", print(point));
    point.x = -10.00; point.y = -0.05; point.z = -0;
    EXPECT_EQ("[-10, -0.05, 0]", print(point));

    geometry_msgs::Quaternion quat;
    quat.x = 0; quat.y = 5; quat.z = 10; quat.w = 20;
    EXPECT_EQ("[0, 5, 10, 20]", print(quat));

    geometry_msgs::Pose pose;
    pose.position.x    =  0; pose.position.y    =  5; pose.position.z    = 10;
    pose.orientation.x = 15; pose.orientation.y = 20; pose.orientation.z = 25;
    pose.orientation.w = 30;
    EXPECT_EQ("[[0, 5, 10], [15, 20, 25, 30]]", print(pose));

    quat.x = -10.00; quat.y = -.05; quat.z = -0; quat.w = 20;
    EXPECT_EQ("[-10, -0.05, 0, 20]", print(quat));
    pose.position.x = point.x;
    pose.position.y = point.y;
    pose.position.z = point.z;
    pose.orientation.x = quat.x;
    pose.orientation.y = quat.y;
    pose.orientation.z = quat.z;
    pose.orientation.w = quat.w;
    EXPECT_EQ("[[-10, -0.05, 0], [-10, -0.05, 0, 20]]", print(pose));
}

TEST(UtilsLib, misc)
{
    geometry_msgs::Quaternion quat;
    quaternionFromDoubles(quat, 30, 35, 40, 45);
    EXPECT_EQ(quat.x, 30);
    EXPECT_EQ(quat.y, 35);
    EXPECT_EQ(quat.z, 40);
    EXPECT_EQ(quat.w, 45);

    quaternionFromDoubles(quat, -0.00, -10, -100.0123, 1.001);
    EXPECT_EQ(quat.x,         0);
    EXPECT_EQ(quat.x,    -00.00);
    EXPECT_EQ(quat.y,     -10.0);
    EXPECT_EQ(quat.z, -100.0123);
    EXPECT_EQ(quat.w,    01.001);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
