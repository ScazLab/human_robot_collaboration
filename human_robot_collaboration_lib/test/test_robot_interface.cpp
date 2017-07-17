#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>

#include "robot_interface/robot_interface.h"

using namespace std;

// Declare a test
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


TEST(RobotInterfaceTest, testJointStatesCallback)
{
    ros::NodeHandle       nh;
    RobotInterface ri("robot", "left");

    ros::Publisher jointStates_pub =
        nh.advertise<sensor_msgs::JointState>
        ("/robot/joint_states", SUBSCRIBER_BUFFER);

    sensor_msgs::JointState msg;
    msg.name = {"_s0", "_s1", "_e0", "_e1", "_w0", "_w1", "_w2"};
    for(size_t i = 0; i < msg.name.size(); ++i)
    {
        msg.name[i] = ri.getLimb() + msg.name[i].c_str();
    }

    msg.position = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
    msg.velocity = {1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7};

    ros::Rate loop_rate(10);

    while(ros::ok() && (ri.getJointStates().name.size() == 0))
    {
        jointStates_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

    }

    EXPECT_EQ(ri.getJointStates().name[0], ri.getLimb() + "_s0");
    EXPECT_EQ(ri.getJointStates().name[1], ri.getLimb() + "_s1");
    EXPECT_EQ(ri.getJointStates().name[2], ri.getLimb() + "_e0");
    EXPECT_EQ(ri.getJointStates().name[3], ri.getLimb() + "_e1");
    EXPECT_EQ(ri.getJointStates().name[4], ri.getLimb() + "_w0");
    EXPECT_EQ(ri.getJointStates().name[5], ri.getLimb() + "_w1");
    EXPECT_EQ(ri.getJointStates().name[6], ri.getLimb() + "_w2");

    // EXPECT_EQ(ri.getJointStates().position[0], 0.1);
    // EXPECT_EQ(ri.getJointStates().position[1], 0.2);
    // EXPECT_EQ(ri.getJointStates().position[2], 0.3);
    // EXPECT_EQ(ri.getJointStates().position[3], 0.4);
    // EXPECT_EQ(ri.getJointStates().position[4], 0.5);
    // EXPECT_EQ(ri.getJointStates().position[5], 0.6);
    // EXPECT_EQ(ri.getJointStates().position[6], 0.7);
    EXPECT_EQ(ri.getJointStates().position, msg.position);

    EXPECT_EQ(ri.getJointStates().velocity, msg.velocity);
}

TEST(RobotInterfaceTest, testCuffLowerCallback)
{
    ros::NodeHandle       nh;
    RobotInterface ri("robot", "left");

    ros::Publisher cuffLower_pub =
        nh.advertise<baxter_core_msgs::DigitalIOState>
        ("/robot/digital_io/" + ri.getLimb() + "_lower_button/state", SUBSCRIBER_BUFFER);

    baxter_core_msgs::DigitalIOState msg;
    msg.state = baxter_core_msgs::DigitalIOState::PRESSED;

    ros::Rate loop_rate(10);

    while(ros::ok() && ri.getState() == START)
    {
        cuffLower_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

    }

    EXPECT_EQ(int(ri.getState()), KILLED);

}

TEST(RobotInterfaceTest, testCuffUpperCallback)
{
    ros::NodeHandle       nh;
    RobotInterface ri("robot", "left");

    ros::Publisher cuffUpper_pub =
        nh.advertise<baxter_core_msgs::DigitalIOState>
        ("/robot/digital_io/" + ri.getLimb() + "_upper_button/state", SUBSCRIBER_BUFFER);

    baxter_core_msgs::DigitalIOState msg;
    msg.state = baxter_core_msgs::DigitalIOState::PRESSED;

    ros::Rate loop_rate(10);

    while(ros::ok() && ri.getState() == START)
    {
        cuffUpper_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

    }

    EXPECT_EQ(int(ri.getState()), KILLED);

}



TEST(RobotInterfaceTest, testEndpointCallback)
{
    ros::NodeHandle       nh;
    RobotInterface ri("robot", "left");

    ros::Publisher endpoint_pub =
        nh.advertise<baxter_core_msgs::EndpointState>
        ("/robot/limb/" + ri.getLimb() + "/endpoint_state", SUBSCRIBER_BUFFER);

    geometry_msgs::Point point;
    point.x = 0.1;
    point.y = 0.2;
    point.z = 0.3;

    geometry_msgs::Quaternion ori;
    ori.x = 0.4;
    ori.y = 0.5;
    ori.z = 0.6;
    ori.w = 0.7;

    baxter_core_msgs::EndpointState msg;
    msg.pose.position = point;
    msg.pose.orientation = ori;

    geometry_msgs::Wrench wrench;
    geometry_msgs::Vector3 force;
    geometry_msgs::Vector3 torque;

    force.x = 0.1;
    force.y = 0.2;
    force.z = 0.3;
    wrench.force = force;

    torque.x = 0.1;
    torque.y = 0.2;
    torque.z = 0.3;
    wrench.torque = torque;

    msg.wrench = wrench;


    ros::Rate loop_rate(10);

    while(ros::ok() && ri.getWrench().torque.z == 0.0)
    {
        endpoint_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

    }

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

    //filterForces() ************************************

}

TEST(RobotInterfaceTest, testIRCallback)
{
    ros::NodeHandle       nh;
    RobotInterface ri("robot", "left");

    EXPECT_EQ(ri.getCurrRange(), 0.0);
    EXPECT_EQ(ri.getCurrMinRange(), 0.0);
    EXPECT_EQ(ri.getCurrMaxRange(), 0.0);


    // Create publisher
    ros::Publisher ir_pub =
        nh.advertise<sensor_msgs::Range>("/robot/range/" + ri.getLimb() + "_hand_range/state", SUBSCRIBER_BUFFER);

    // Publish stuff
    sensor_msgs::Range msg;

    msg.range = 1.0;
    msg.min_range = 2.0;
    msg.max_range = 3.0;

    ros::Rate loop_rate(10);

    while(ros::ok() && (ri.getCurrRange() == 0.0))
    {
        ir_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

    }

    // Check that the stuff has beeen correctly parsed by a robot interface object
    EXPECT_EQ(ri.getCurrRange(), 1.0);
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


