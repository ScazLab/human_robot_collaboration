#include <gtest/gtest.h>

#include "robot_utils/utils.h"


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

    a = b;
    EXPECT_TRUE(a==b);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
