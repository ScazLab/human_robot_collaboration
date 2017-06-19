#include "robot_interface/gripper.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_keyboard");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // gripper instantiation
    Gripper *left_gripper = NULL, *right_gripper = NULL;
    left_gripper = new Gripper("left");
    right_gripper = new Gripper("right");

    ROS_INFO("You are now controlling the robot grippers");

    // gripper keys
    while(ros::ok())
    {
        ROS_INFO("Enter your command (or press ? for help)...");

        char c = std::cin.get();
        std::cin.ignore();
        ROS_INFO("Key entered: %c", c);

        switch(c)
        {
        case '?':
            ROS_INFO("Available commands: (lowercase for left gripper, uppercase for right gripper)");
            ROS_INFO("q / Q : close gripper");
            ROS_INFO("w / W : open gripper");
            ROS_INFO("t / T : type of gripper");
            ROS_INFO("z / Z : is gripper enabled?");
            ROS_INFO("x / X : is gripper calibrated?");
            ROS_INFO("c / C : is gripper ready to grip?");
            ROS_INFO("v / V : does gripper have error?");
            ROS_INFO("b / B : is gripper sucking?");
            ROS_INFO("n / N : is gripper gripping?");
            ROS_INFO("? : help");
            ROS_INFO("e : exit");
            break;
        case 'q':
            ROS_INFO("Closing left gripper");
            left_gripper->close();
            break;
        case 'Q':
            ROS_INFO("Closing right gripper");
            right_gripper->close();
            break;
        case 'w':
            ROS_INFO("Opening left gripper");
            left_gripper->open();
            break;
        case 'W':
            ROS_INFO("Opening right gripper");
            right_gripper->open();
            break;
        case 't':
            ROS_INFO("Left gripper type: %s", left_gripper->type().c_str());
            break;
        case 'T':
            ROS_INFO("Right gripper type: %s", right_gripper->type().c_str());
            break;
        case 'z':
            ROS_INFO("Is left enabled? %s", (left_gripper->is_enabled()? "Yes" : "No"));
            break;
        case 'Z':
            ROS_INFO("Is right enabled? %s", (right_gripper->is_enabled()? "Yes" : "No"));
            break;
        case 'x':
            ROS_INFO("Is left calibrated? %s", left_gripper->is_calibrated()? "Yes" : "No");
            break;
        case 'X':
            ROS_INFO("Is right calibrated? %s", right_gripper->is_calibrated()? "Yes" : "No");
            break;
        case 'c':
            ROS_INFO("Is left ready to grip? %s", left_gripper->is_ready_to_grip()? "Yes" : "No");
            break;
        case 'C':
            ROS_INFO("Is right ready to grip? %s", right_gripper->is_ready_to_grip()? "Yes" : "No");
            break;
        case 'v':
            ROS_INFO("Does left have error? %s", left_gripper->has_error()? "Yes" : "No");
            break;
        case 'V':
            ROS_INFO("Does right have error? %s", right_gripper->has_error()? "Yes" : "No");
            break;
        case 'b':
            ROS_INFO("Is left sucking? %s", left_gripper->is_sucking()? "Yes" : "No");
            break;
        case 'B':
            ROS_INFO("Is right sucking? %s", right_gripper->is_sucking()? "Yes" : "No");
            break;
        case 'n':
            ROS_INFO("Is left gripping? %s", left_gripper->is_gripping()? "Yes" : "No");
            break;
        case 'N':
            ROS_INFO("Is right gripping? %s", right_gripper->is_gripping()? "Yes" : "No");
            break;
        case 'e':
            ROS_INFO("Exiting...");
            return 0;
        default:
            ROS_INFO("Not a valid command. Press ? for help, e to exit.");
            break;
        }
        ros::spinOnce();
    }

    delete left_gripper;
    delete right_gripper;
    left_gripper = right_gripper = 0;

    return 0;
}

