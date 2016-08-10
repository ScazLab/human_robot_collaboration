#include <highgui.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "baxter_collaboration/ArmState.h"

using namespace std;
using namespace baxter_collaboration;

class BaxterDisplay
{
private:
    std::string name;

    ros::NodeHandle    nh;
    ros::Subscriber l_sub;
    ros::Subscriber r_sub;

    ArmState l_state;
    ArmState r_state;

    image_transport::ImageTransport     it;
    image_transport::Publisher      im_pub;

    int h;
    int w;

    int w_delim;

    cv::Scalar red;
    cv::Scalar green;
public:

    BaxterDisplay(string _name) : name(_name), it(nh)
    {
        im_pub = it.advertise("/robot/xdisplay", 1);

        l_sub = nh.subscribe("/action_provider/state_left", 1, &BaxterDisplay::armStateCbL, this);
        r_sub = nh.subscribe("/action_provider/state_right",1, &BaxterDisplay::armStateCbR, this);

        h = 600;
        w = 1024;

        w_delim = 8;

        l_state.state  = "START";
        l_state.action =     "";
        l_state.object =     "";

        r_state.state  = "START";
        r_state.action =     ""; 
        r_state.object =     "";

        red   = cv::Scalar( 44,  48, 201);  // BGR color code
        green = cv::Scalar( 60, 160,  60);

        displayArmStates();
    };

    void armStateCbL(const ArmState& msg)
    {
        armStateCb(msg, "left");
    };

    void armStateCbR(const ArmState& msg)
    {
        armStateCb(msg, "right");
    };

    void armStateCb(const ArmState& msg, std::string _limb)
    {
        ROS_INFO("Received callback! Arm %s", _limb.c_str());

        if      (_limb == "left")
        {
            l_state = msg;

            if (l_state.state == "DONE")
            {
                l_state.action = "";
                l_state.object = "";
            }
        }
        else if (_limb == "right")
        {
            r_state = msg;

            if (r_state.state == "DONE")
            {
                r_state.action = "";
                r_state.object = "";
            }
        }

        displayArmStates();
    }

    bool displayArmStates()
    {
        cv::Mat l = createSubImage("LEFT");
        cv::Mat r = createSubImage("RIGHT");
        cv::Mat d(h,w_delim,CV_8UC3,cv::Scalar::all(80));

        cv::Mat res(h,w,CV_8UC3,cv::Scalar(255,100,255));

        // Move right boundary to the left.
        res.adjustROI(0,0,0,-(w+w_delim)/2);
        r.copyTo(res);

        // Move the left boundary to the right, right boundary to the right.
        res.adjustROI(0, 0, -(w-w_delim)/2, w_delim);
        d.copyTo(res);

        // Move the left boundary to the right, right boundary to the right.
        res.adjustROI(0, 0, -w_delim, (w-w_delim)/2);
        l.copyTo(res);

        res.adjustROI(0, 0, (w+w_delim)/2, 0);

        cv_bridge::CvImage msg;
        msg.encoding = sensor_msgs::image_encodings::BGR8;
        msg.image    = res;

        im_pub.publish(msg.toImageMsg());

        // cv::imshow("res", res);
        // cv::waitKey(20);

        return true;
    };

    cv::Mat createSubImage(std::string _limb)
    {
        ArmState state = _limb=="LEFT"?l_state:r_state;

        cv::Mat img(h,(w-w_delim)/2,CV_8UC3,cv::Scalar::all(255));
        cv::Scalar col       = cv::Scalar::all(60);
        cv::Scalar col_state = green;

        if (state.state == "ERROR"  || state.state == "TEST" ||
            state.state == "KILLED" || state.state == "DONE" ||
            state.state == "START" )
        {
            col = cv::Scalar::all(240);
            col_state = col;
            img.setTo(red);

            if (state.state == "DONE" || state.state == "TEST" || state.state == "START")
            {
                img.setTo(green);
            }
        }

        int thickness = 3;
        int baseline  = 0;
        int fontFace  = cv::FONT_HERSHEY_SIMPLEX;
        int fontScale = 2;

        // Place a centered title on top
        string title = _limb + " ARM";
        cv::Size textSize = cv::getTextSize( title, fontFace, fontScale, thickness, &baseline);
        baseline += thickness;
        cv::Point textOrg((img.cols - textSize.width)/2, (img.rows + textSize.height)/6);
        putText(img, title, textOrg, fontFace, fontScale, col, thickness, 8);

        if (state.state !="")
        {
            putText(img, "state:", cv::Point(20,300), fontFace, fontScale/2, col, 2, 8);
            putText(img, state.state, cv::Point(150,300), fontFace, fontScale, col_state, thickness, 8);
        }
        if (state.action !="")
        {
            putText(img, "action:", cv::Point(20,400), fontFace, fontScale/2, col, 2, 8);
            putText(img, state.action, cv::Point(150,400), fontFace, fontScale/1.25, col, thickness, 8);
        }
        if (state.object !="")
        {
            putText(img, "object:", cv::Point(20,500), fontFace, fontScale/2, col, 2, 8);
            putText(img, state.object, cv::Point(150,500), fontFace, fontScale/1.25, col, thickness, 8);
        }

        return img;
    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "baxter_display");

    BaxterDisplay bd("baxter_display");

    ros::Duration(0.2).sleep();
    bd.displayArmStates();
    
    ros::spin();
    return 0;
}

