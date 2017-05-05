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

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
