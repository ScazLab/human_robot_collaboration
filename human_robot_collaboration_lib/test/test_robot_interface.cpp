#include <gtest/gtest.h>

#include "robot_interface/robot_interface.h"

using namespace std;

TEST(RobotInterfaceTest, testConstructorDefaultValues)
{
    RobotInterface ri("robot", "left");

    EXPECT_FALSE(ri.isCtrlRunning());
    EXPECT_EQ(START, int(ri.getState()));

    EXPECT_EQ("robot", ri.getName());
    EXPECT_EQ( "left", ri.getLimb());
    EXPECT_TRUE(ri.isRobotUsed());
    EXPECT_EQ(  100.0, ri.getCtrlFreq());
    EXPECT_TRUE(ri.useForces());
    EXPECT_TRUE(ri.useTracIK());
    EXPECT_TRUE(ri.useCartCtrl());
    EXPECT_FALSE(ri.isExperimental());

    ri.setTracIK(false);
    EXPECT_FALSE(ri.useTracIK());
}

TEST(RobotInterfaceTest, testConstructorCustomValues)
{
    string          name = "robot";
    string          limb =  "left";
    bool       use_robot =   false;
    double     ctrl_freq =    50.0;
    bool      use_forces =   false;
    bool     use_trac_ik =   false;
    bool   use_cart_ctrl =   false;
    bool is_experimental =   false;
    RobotInterface ri(name, limb, use_robot, ctrl_freq, use_forces,
                      use_trac_ik, use_cart_ctrl, is_experimental);

    EXPECT_FALSE(ri.isCtrlRunning());
    EXPECT_EQ(START, int(ri.getState()));

    EXPECT_EQ(           name, ri.getName());
    EXPECT_EQ(           limb, ri.getLimb());
    EXPECT_EQ(      use_robot, ri.isRobotUsed());
    EXPECT_EQ(      ctrl_freq, ri.getCtrlFreq());
    EXPECT_EQ(     use_forces, ri.useForces());
    EXPECT_EQ(    use_trac_ik, ri.useTracIK());
    EXPECT_EQ(  use_cart_ctrl, ri.useCartCtrl());
    EXPECT_EQ(is_experimental, ri.isExperimental());

    use_trac_ik = true;
    ri.setTracIK(use_trac_ik);
    EXPECT_EQ(use_trac_ik, ri.useTracIK());

}

TEST(RobotInterfaceTest, testPrivateMethods)
{
    string          name = "robot";
    string          limb =  "left";
    bool       use_robot =   false;
    double     ctrl_freq =    50.0;
    bool      use_forces =   false;
    bool     use_trac_ik =   false;
    bool   use_cart_ctrl =   false;
    bool is_experimental =   false;
    RobotInterface ri(name, limb, use_robot, ctrl_freq, use_forces,
                      use_trac_ik, use_cart_ctrl, is_experimental);

    EXPECT_FALSE(ri.isPoseReached(geometry_msgs::Pose(),  "loose",         "sdf"));
    EXPECT_FALSE(ri.isPoseReached(geometry_msgs::Pose(), "strict",         "sdf"));
    EXPECT_FALSE(ri.isPoseReached(geometry_msgs::Pose(),    "sdf",        "pose"));
    EXPECT_FALSE(ri.isPoseReached(geometry_msgs::Pose(),    "sdf",    "position"));
    EXPECT_FALSE(ri.isPoseReached(geometry_msgs::Pose(),    "sdf", "orientation"));

    std::string type = "pose";
    EXPECT_FALSE(ri.setCtrlType(""));
    EXPECT_EQ(ri.getCtrlType(), type);
    EXPECT_FALSE(ri.setCtrlType(" "));
    EXPECT_EQ(ri.getCtrlType(), type);
    EXPECT_FALSE(ri.setCtrlType("asdfa"));
    EXPECT_EQ(ri.getCtrlType(), type);
    EXPECT_TRUE(ri.setCtrlType(type));
    EXPECT_EQ(ri.getCtrlType(), type);

    type = "position";
    EXPECT_TRUE(ri.setCtrlType(type));
    EXPECT_EQ(ri.getCtrlType(), type);

    type = "orientation";
    EXPECT_TRUE(ri.setCtrlType(type));
    EXPECT_EQ(ri.getCtrlType(), type);
}

using namespace baxter_core_msgs;

// Unit test for JoinStatesCb
TEST(RobotInterfaceTest, testJointStatesCb)
{
    RobotInterface ri("robot", "left");

    ros::NodeHandle nh("robot_interface_tester");
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/robot/joint_states",
                                                                  SUBSCRIBER_BUFFER);

    // Create message to publish to robot interface
    sensor_msgs::JointState msg;

    // Set random values for JointState name, position, and velocity fields
    msg.name = {"_s0", "_s1", "_e0", "_e1", "_w0", "_w1", "_w2"};

    for(size_t i = 0; i < msg.name.size(); ++i)
    {
        msg.name[i] = ri.getLimb() + msg.name[i].c_str();
    }

    msg.position = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
    msg.velocity = {1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7};


    // Publish and ensure the Subscriber has received the message
    ros::Rate loop_rate(10);
    while(ros::ok() && (ri.getJointStates().name.size() == 0))
    {
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Check that the JointState message has been correctly parsed
    // by the robot interface
    EXPECT_EQ(ri.getJointStates().name[0], ri.getLimb() + "_s0");
    EXPECT_EQ(ri.getJointStates().name[1], ri.getLimb() + "_s1");
    EXPECT_EQ(ri.getJointStates().name[2], ri.getLimb() + "_e0");
    EXPECT_EQ(ri.getJointStates().name[3], ri.getLimb() + "_e1");
    EXPECT_EQ(ri.getJointStates().name[4], ri.getLimb() + "_w0");
    EXPECT_EQ(ri.getJointStates().name[5], ri.getLimb() + "_w1");
    EXPECT_EQ(ri.getJointStates().name[6], ri.getLimb() + "_w2");

    EXPECT_EQ(ri.getJointStates().position, msg.position);
    EXPECT_EQ(ri.getJointStates().velocity, msg.velocity);
}

// Unit test for cuffLowerCb
TEST(RobotInterfaceTest, testCuffLowerCb)
{
    RobotInterface ri("robot", "left");

    ros::NodeHandle nh("robot_interface_tester");
    ros::Publisher pub = nh.advertise<DigitalIOState>("/robot/digital_io/" +
                                                      ri.getLimb() +
                                                      "_lower_button/state",
                                                      SUBSCRIBER_BUFFER);

    // Create message to publish to robot interface
    DigitalIOState msg;
    msg.state = DigitalIOState::PRESSED;

    // Publish and ensure the Subscriber has received the message
    ros::Rate loop_rate(10);
    while(ros::ok() && ri.getState() == START)
    {
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Check that the DigitalIOState message has been correctly parsed
    // by the robot interface and that the robot interface state has been
    // changed accordingly
    EXPECT_EQ(int(ri.getState()), KILLED);
}

// Unit test for cuffUpperCb
TEST(RobotInterfaceTest, testCuffUpperCb)
{
    RobotInterface ri("robot", "left");

    ros::NodeHandle nh("robot_interface_tester");
    ros::Publisher pub = nh.advertise<DigitalIOState>("/robot/digital_io/" +
                                                      ri.getLimb() +
                                                      "_upper_button/state",
                                                      SUBSCRIBER_BUFFER);

    // Create message to publish to robot interface
    DigitalIOState msg;
    msg.state = DigitalIOState::PRESSED;

    ros::Rate loop_rate(10);
    // Publish and ensure the Subscriber has received the message
    while(ros::ok() && ri.getState() == START)
    {
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Check that the DigitalIOState message has been correctly parsed
    // by the robot interface and that the robot interface state has been
    // changed accordingly
    EXPECT_EQ(int(ri.getState()), KILLED);
}

// Unit test for endpointCb
TEST(RobotInterfaceTest, testEndpointCb)
{
    RobotInterface ri("robot", "left");

    ros::NodeHandle nh("robot_interface_tester");
    ros::Publisher pub = nh.advertise<EndpointState>("/robot/limb/" +
                                                     ri.getLimb() +
                                                     "/endpoint_state",
                                                     SUBSCRIBER_BUFFER);

    // Create message to publish to robot interface
    EndpointState msg;
    // Set random values for position (Point)
    msg.pose.position.x    = 0.1;
    msg.pose.position.y    = 0.2;
    msg.pose.position.z    = 0.3;
    // Set random values for orientation (Quaternion)
    msg.pose.orientation.x = 0.4;
    msg.pose.orientation.y = 0.5;
    msg.pose.orientation.z = 0.6;
    msg.pose.orientation.w = 0.7;
    // Set random values for force fields of wrench
    msg.wrench.force.x     = 0.1;
    msg.wrench.force.y     = 0.2;
    msg.wrench.force.z     = 0.3;
    // Set random values for torque fields of wrench
    msg.wrench.torque.x    = 0.1;
    msg.wrench.torque.y    = 0.2;
    msg.wrench.torque.z    = 0.3;

    // Publish and ensure the Subscriber has received the message
    ros::Rate loop_rate(10);
    while(ros::ok() && ri.getWrench().torque.z == 0.0)
    {
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Check that position, orientation, and wrench fields have been
    // correctly parsed by the robot interface
    EXPECT_EQ(ri.getPos().x, 0.1);
    EXPECT_EQ(ri.getPos().y, 0.2);
    EXPECT_EQ(ri.getPos().z, 0.3);

    EXPECT_EQ(ri.getOri().x, 0.4);
    EXPECT_EQ(ri.getOri().y, 0.5);
    EXPECT_EQ(ri.getOri().z, 0.6);
    EXPECT_EQ(ri.getOri().w, 0.7);

    EXPECT_EQ(ri.getWrench().force.x, 0.1);
    EXPECT_EQ(ri.getWrench().force.y, 0.2);
    EXPECT_EQ(ri.getWrench().force.z, 0.3);
    EXPECT_EQ(ri.getWrench().torque.x, 0.1);
    EXPECT_EQ(ri.getWrench().torque.y, 0.2);
    EXPECT_EQ(ri.getWrench().torque.z, 0.3);

}

TEST(RobotInterfaceTest, testIRCb)
{
    RobotInterface ri("robot", "left");

    ros::NodeHandle nh("robot_interface_tester");
    ros::Publisher pub = nh.advertise<sensor_msgs::Range>("/robot/range/" +
                                                          ri.getLimb() +
                                                          "_hand_range/state",
                                                          SUBSCRIBER_BUFFER);

    // Check default valuesof robot interface
    EXPECT_EQ(ri.getCurrRange(), 0.0);
    EXPECT_EQ(ri.getCurrMinRange(), 0.0);
    EXPECT_EQ(ri.getCurrMaxRange(), 0.0);

    // Create message to publish to robot interface
    sensor_msgs::Range msg;

    // Set random values for the Range message
    msg.range = 1.0;
    msg.min_range = 2.0;
    msg.max_range = 3.0;

    // Publish and ensure the Subscriber has received the message
    ros::Rate loop_rate(10);
    while(ros::ok() && (ri.getCurrRange() == 0.0))
    {
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // Check that the Range message has been correctly parsed by the robot interface
    EXPECT_EQ(ri.getCurrRange()   , 1.0);
    EXPECT_EQ(ri.getCurrMinRange(), 2.0);
    EXPECT_EQ(ri.getCurrMaxRange(), 3.0);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_interface_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
