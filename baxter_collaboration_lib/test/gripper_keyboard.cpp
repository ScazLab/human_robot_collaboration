#include "robot_interface/gripper.h"
#include <thread>

using namespace std;

class GripperKeyboard
{
private:

    bool           proceed;
    char               key;
    bool           set_key;

    Gripper left_gripper;
    Gripper right_gripper;

    std::mutex mtx_set_key;
    std::thread key_thread;

    void threadFunction()
    {
        char ch;

        while(ros::ok() && proceed)
        {
            cin.get(ch);
            cin.ignore();
            ROS_INFO("Key entered: %c", ch);
            key = ch;

            std::lock_guard<std::mutex> lck(mtx_set_key);

            if(ch == 'e' || ch == 'E')
            {
                proceed = false;
            }
            else
            {
                set_key = true;
            }
        }
        return;
    }

    void startThread()
    {
        key_thread = std::thread(&GripperKeyboard::threadFunction, this);
    }

    void joinThread()
    {
        if(key_thread.joinable()) { key_thread.join(); }
    }

    void keyBindings(char _c)
    {
        switch(_c)
        {
        case '?':
            ROS_INFO("Available commands: (lowercase for left gripper, uppercase for right gripper)");
            ROS_INFO("q / Q : close gripper");
            ROS_INFO("w / W : open gripper");
            ROS_INFO("c / C : calibrate gripper");
            ROS_INFO("x / X : is gripper calibrated?");
            ROS_INFO("d / D : clear calibration");
            ROS_INFO("t / T : type of gripper");
            ROS_INFO("z / Z : is gripper enabled?");
            ROS_INFO("v / V : does gripper have error?");
            ROS_INFO("b / B : is gripper sucking?");
            ROS_INFO("n / N : is gripper gripping?");
            ROS_INFO("m / M : is gripper ready to grip?");
            ROS_INFO("? : help");
            ROS_INFO("e / E : exit");
            break;
        case 'q':
            ROS_INFO("Closing left gripper");
            left_gripper.close();
            break;
        case 'Q':
            ROS_INFO("Closing right gripper");
            right_gripper.close();
            break;
        case 'w':
            ROS_INFO("Opening left gripper");
            left_gripper.open();
            break;
        case 'W':
            ROS_INFO("Opening right gripper");
            right_gripper.open();
            break;
        case 'c':
            ROS_INFO("Calibrating left gripper");
            left_gripper.calibrate();
            break;
        case 'C':
            ROS_INFO("Calibrating right gripper");
            right_gripper.calibrate();
            break;
        case 'x':
            ROS_INFO("Is left calibrated? %s", left_gripper.is_calibrated()? "Yes" : "No");
            break;
        case 'X':
            ROS_INFO("Is right calibrated? %s", right_gripper.is_calibrated()? "Yes" : "No");
            break;
        case 'd':
            ROS_INFO("Clear left gripper calibration");
            left_gripper.clearCalibration();
            break;
        case 'D':
            ROS_INFO("Clear right gripper calibration");
            right_gripper.clearCalibration();
            break;
        case 't':
            ROS_INFO("Left gripper type: %s", left_gripper.type().c_str());
            break;
        case 'T':
            ROS_INFO("Right gripper type: %s", right_gripper.type().c_str());
            break;
        case 'z':
            ROS_INFO("Is left enabled? %s", (left_gripper.is_enabled()? "Yes" : "No"));
            break;
        case 'Z':
            ROS_INFO("Is right enabled? %s", (right_gripper.is_enabled()? "Yes" : "No"));
            break;
        case 'v':
            ROS_INFO("Does left have error? %s", left_gripper.has_error()? "Yes" : "No");
            break;
        case 'V':
            ROS_INFO("Does right have error? %s", right_gripper.has_error()? "Yes" : "No");
            break;
        case 'b':
            ROS_INFO("Is left sucking? %s", left_gripper.is_sucking()? "Yes" : "No");
            break;
        case 'B':
            ROS_INFO("Is right sucking? %s", right_gripper.is_sucking()? "Yes" : "No");
            break;
        case 'n':
            ROS_INFO("Is left gripping? %s", left_gripper.is_gripping()? "Yes" : "No");
            break;
        case 'N':
            ROS_INFO("Is right gripping? %s", right_gripper.is_gripping()? "Yes" : "No");
            break;
        case 'm':
            ROS_INFO("Is left ready to grip? %s", left_gripper.is_ready_to_grip()? "Yes" : "No");
            break;
        case 'M':
            ROS_INFO("Is right ready to grip? %s", right_gripper.is_ready_to_grip()? "Yes" : "No");
            break;
        default:
            ROS_INFO("Not a valid command. Press ? for help, e to exit.");
            break;
        }
    }

public:

    GripperKeyboard() : proceed(true), key('\0'), set_key(false),
                        left_gripper("left"), right_gripper("right")
    {
        ROS_INFO("Gripper keyboard starting...");
        ROS_INFO("You are now controlling the robot grippers");
        ROS_INFO("Enter your commands (or press ? for help)");
    }

    void run()
    {
        startThread();

        ros::Rate r(10);
        while(ros::ok() && proceed)
        {
            if(set_key)
            {
                keyBindings(key);

                std::lock_guard<std::mutex> lck(mtx_set_key);
                set_key = false;
            }
            r.sleep();
        }

        ROS_INFO("Gripper keyboard closing...");
        proceed = false;
        joinThread();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_keyboard");

    GripperKeyboard gk;
    gk.run();

    return 0;
}


/*


bool           proceed; // flag to continue the thread and main loops
char               key; // key to store the user input
bool           set_key; // flag to know if the key was set
std::mutex mtx_set_key; // mutex to protect the above flag

void keyBindings(char _c, Gripper& left_gripper, Gripper& right_gripper)
{
    switch(_c)
    {
    case '?':
        ROS_INFO("Available commands: (lowercase for left gripper, uppercase for right gripper)");
        ROS_INFO("q / Q : close gripper");
        ROS_INFO("w / W : open gripper");
        ROS_INFO("c / C : calibrate gripper");
        ROS_INFO("x / X : is gripper calibrated?");
        ROS_INFO("d / D : clear calibration");
        ROS_INFO("t / T : type of gripper");
        ROS_INFO("z / Z : is gripper enabled?");
        ROS_INFO("v / V : does gripper have error?");
        ROS_INFO("b / B : is gripper sucking?");
        ROS_INFO("n / N : is gripper gripping?");
        ROS_INFO("m / M : is gripper ready to grip?");
        ROS_INFO("? : help");
        ROS_INFO("e / E : exit");
        break;
    case 'q':
        ROS_INFO("Closing left gripper");
        left_gripper.close();
        break;
    case 'Q':
        ROS_INFO("Closing right gripper");
        right_gripper.close();
        break;
    case 'w':
        ROS_INFO("Opening left gripper");
        left_gripper.open();
        break;
    case 'W':
        ROS_INFO("Opening right gripper");
        right_gripper.open();
        break;
    case 'c':
        ROS_INFO("Calibrating left gripper");
        left_gripper.calibrate();
        break;
    case 'C':
        ROS_INFO("Calibrating right gripper");
        right_gripper.calibrate();
        break;
    case 'x':
        ROS_INFO("Is left calibrated? %s", left_gripper.is_calibrated()? "Yes" : "No");
        break;
    case 'X':
        ROS_INFO("Is right calibrated? %s", right_gripper.is_calibrated()? "Yes" : "No");
        break;
    case 'd':
        ROS_INFO("Clear left gripper calibration");
        left_gripper.clearCalibration();
        break;
    case 'D':
        ROS_INFO("Clear right gripper calibration");
        right_gripper.clearCalibration();
        break;
    case 't':
        ROS_INFO("Left gripper type: %s", left_gripper.type().c_str());
        break;
    case 'T':
        ROS_INFO("Right gripper type: %s", right_gripper.type().c_str());
        break;
    case 'z':
        ROS_INFO("Is left enabled? %s", (left_gripper.is_enabled()? "Yes" : "No"));
        break;
    case 'Z':
        ROS_INFO("Is right enabled? %s", (right_gripper.is_enabled()? "Yes" : "No"));
        break;
    case 'v':
        ROS_INFO("Does left have error? %s", left_gripper.has_error()? "Yes" : "No");
        break;
    case 'V':
        ROS_INFO("Does right have error? %s", right_gripper.has_error()? "Yes" : "No");
        break;
    case 'b':
        ROS_INFO("Is left sucking? %s", left_gripper.is_sucking()? "Yes" : "No");
        break;
    case 'B':
        ROS_INFO("Is right sucking? %s", right_gripper.is_sucking()? "Yes" : "No");
        break;
    case 'n':
        ROS_INFO("Is left gripping? %s", left_gripper.is_gripping()? "Yes" : "No");
        break;
    case 'N':
        ROS_INFO("Is right gripping? %s", right_gripper.is_gripping()? "Yes" : "No");
        break;
    case 'm':
        ROS_INFO("Is left ready to grip? %s", left_gripper.is_ready_to_grip()? "Yes" : "No");
        break;
    case 'M':
        ROS_INFO("Is right ready to grip? %s", right_gripper.is_ready_to_grip()? "Yes" : "No");
        break;
    default:
        ROS_INFO("Not a valid command. Press ? for help, e to exit.");
        break;
    }
}

void threadFunction()
{
    char ch;

    while(ros::ok() && proceed)
    {
        ROS_INFO("thread function running");
        cin.get(ch);
        cin.ignore();
        ROS_INFO("Key entered: %c", ch);
        key = ch;

        std::lock_guard<std::mutex> lck(mtx_set_key);

        if(ch == 'e' || ch == 'E')
        {
            proceed = false;
        }
        else
        {
            set_key = true;
        }
    }
    printf("thread function closing\n");
    return;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_keyboard");

    // gripper instantiation
    Gripper left_gripper("left");
    Gripper right_gripper("right");
    ROS_INFO("You are now controlling the robot grippers");
    ROS_INFO("Enter your commands (or press ? for help)...");

    proceed = true;

    std::thread thread(threadFunction);
    ros::Rate r(10);

    while(ros::ok() && proceed)
    {
        ROS_INFO("main thread running");
        if(set_key)
        {
            keyBindings(key, left_gripper, right_gripper);

            std::lock_guard<std::mutex> lck(mtx_set_key);
            set_key = false;
        }
        r.sleep();
    }

    ROS_INFO("main thread closing");
    proceed = false;
    if(thread.joinable()) { thread.join(); }

    // exit program
    return 0;
}

*/
