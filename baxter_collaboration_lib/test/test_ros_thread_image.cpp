#include <gtest/gtest.h>
#include "robot_utils/ros_thread_image.h"


class ROSImageObject: public ROSThreadImage
{
public:    
    explicit ROSImageObject(std::string namenew): ROSThreadImage(namenew) {}
    void InternalThreadEntry(); 
};


void ROSImageObject::InternalThreadEntry() {
    std::cout << "test" << std::endl;
};


TEST(rosimagetest, testsetname)
{
   ROSImageObject rt("test");
   rt.joinInternalThread();

   rt.setName("newtestname");
   EXPECT_EQ("newtestname", rt.getName());
   EXPECT_FALSE("test" == rt.getName()); 
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_thread_test"); 
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
