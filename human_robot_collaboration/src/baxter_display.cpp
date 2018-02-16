#include <highgui.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "human_robot_collaboration_msgs/ArmState.h"

using namespace std;
using namespace human_robot_collaboration_msgs;

#define DEFAULT_DURATION 10.0  // [s]

/**
 * Class that manages the output to the baxter display. By default, it publishes an image
 */
class BaxterDisplay
{
private:
    ros::NodeHandle    nh;

    int print_level;        // Print level to be used throughout the code

    std::string name;       // Name of the node

    ros::Subscriber l_sub;  // Subscriber for the left  arm state
    ros::Subscriber r_sub;  // Subscriber for the right arm state
    ros::Subscriber s_sub;  // Subscriber for the speech output

    ArmState l_state;
    ArmState r_state;

    std::string speech;             // Text to display
    ros::Timer  speech_timer;       // Timer remove the speech pop-up after specific duration
    double      speech_duration;    // Duration of the speech pop-up

    image_transport::ImageTransport     it;
    image_transport::Publisher      im_pub;

    int h;     // height of the image to be shown (equal to the height of the baxter display)
    int w;     //  width of the image to be shown (equal to the  width of the baxter display)
    int w_d;   //  width of the delimiter between sub-screens
    int w_b;   //  width of the bottom sub-screen

    cv::Scalar   red;
    cv::Scalar green;
    cv::Scalar  blue;

    /**
     * Callback for the left arm state
     * @param msg the left arm state
     */
    void armStateCbL(const ArmState& msg)
    {
        armStateCb(msg, "left");
    };

    /**
     * Callback for the right arm state
     * @param msg the right arm state
     */
    void armStateCbR(const ArmState& msg)
    {
        armStateCb(msg, "right");
    };

    /**
     * Unified callback manager for both arm states
     * @param msg   the arm state
     * @param _limb the arm it is referred to
     */
    void armStateCb(const ArmState& msg, std::string _limb)
    {
        ROS_INFO_COND(print_level>=4, "Arm %s", _limb.c_str());

        if      (_limb == "left")
        {
            l_state = msg;
        }
        else if (_limb == "right")
        {
            r_state = msg;
        }

        displayArmStates();
    };

    /**
     * Callback from the speech
     * @param msg the speech
     */
    void speechCb(const std_msgs::String& msg)
    {
        ROS_INFO_COND(print_level>=4, "Text: %s", msg.data.c_str());

        setSpeech(msg.data);

        speech_timer = nh.createTimer(ros::Duration(speech_duration),
                                      &BaxterDisplay::deleteSpeechCb, this, true);

        displayArmStates();
    };

    /**
     * Callback to delete the speech from the screen (after t=speech_duration)
     */
    void deleteSpeechCb(const ros::TimerEvent&)
    {
        ROS_INFO_COND(print_level>=4, "Deleting speech");
        setSpeech("");
        displayArmStates();
    };

    /**
     * Function to display speech on top of the generated image
     * @param in the already generated image ready to be sent to the robot
     */
    void displaySpeech(cv::Mat& in)
    {
        if (speech !="")
        {
            ROS_INFO_COND(print_level>=5, "Displaying speech: %s", speech.c_str());

            int thickn = 3;                         // thickness
            int bsline = 0;                         // baseline
            int fontFc = cv::FONT_HERSHEY_SIMPLEX;  // fontFace
            int fontSc = 2;                         // fontScale

            int border = 20;

            int max_width = 800; // max width of a text line

            cv::Size textSize = cv::getTextSize( speech, fontFc, fontSc, thickn, &bsline);
            int numLines = int(textSize.width/max_width)+1;

            if (numLines>5)
            {
                fontSc = 1.6;
                thickn =   2;
                textSize = cv::getTextSize( speech, fontFc, fontSc, thickn, &bsline);
                numLines = int(textSize.width/max_width);
            }
            ROS_INFO_COND(print_level>=6, "Size of the text %i %i numLines %i",
                                          textSize.height, textSize.width, numLines);

            std::vector<std::string> line;
            std::vector<cv::Size>    size;

            int interline  =         20;  // Number of pixels between a line and the next one
            int rec_height = -interline;  // Height of the container (line_heigth + interline)
            int rec_width  =          0;  // Width  of the container (max width of each line)
            int line_length = int(speech.size()/numLines);

            for (int i = 0; i < numLines; ++i)
            {
                // The last line gets also the remainder of the splitting
                if (i==numLines-1)
                {
                    line.push_back(speech.substr(i*line_length,speech.size()-i*line_length));
                }
                else
                {
                    line.push_back(speech.substr(i*line_length,line_length));
                }

                size.push_back(cv::getTextSize( line.back(), fontFc, fontSc, thickn, &bsline));
                if (size.back().width>rec_width) { rec_width=size.back().width; }
                rec_height += interline + size.back().height;

                ROS_INFO_COND(print_level>=7, "   Line %i: size: %i %i\ttext: %s", i,
                              size.back().height, size.back().width, line.back().c_str());
            }
            rec_height += 2*border;
            rec_width  += 2*border;

            cv::Point rectOrg((in.cols-rec_width)/2, (in.rows-w_d/2-w_b-rec_height)/2);
            cv::Point rectEnd((in.cols+rec_width)/2, (in.rows-w_d/2-w_b+rec_height)/2);
            rectangle(in, rectOrg, rectEnd, blue, -1);

            int textOrgy = rectOrg.y + border;
            for (int i = 0; i < numLines; ++i)
            {
                textOrgy += size[i].height;
                cv::Point textOrg((in.cols - size[i].width)/2, textOrgy);
                putText(in, line[i], textOrg, fontFc, fontSc, cv::Scalar::all(255), thickn, CV_AA);
                textOrgy += interline;
            }
        }
    };

    /**
     * Function to create sub-image for either arm
     * @param  _limb the arm to create the image for
     * @return       the sub-image
     */
    cv::Mat createArmImage(std::string _limb)
    {
        ArmState state = _limb=="LEFT"?l_state:r_state;

        cv::Mat img(h-w_d/2-w_b,(w-w_d)/2,CV_8UC3,cv::Scalar::all(255));
        ROS_INFO_COND(print_level>=6, "Created %s image with size %i %i",
                                      _limb.c_str(), img.rows, img.cols);

        cv::Scalar col   = cv::Scalar::all(60);
        cv::Scalar col_s = green;

        if (state.state == "ERROR"  || state.state == "RECOVER" ||
            state.state == "KILLED" || state.state == "DONE" ||
            state.state == "START" )
        {
            col   = cv::Scalar::all(240);
            col_s = col;
            img.setTo(red);

            if (state.state == "DONE" || state.state == "TEST" || state.state == "START")
            {
                img.setTo(green);
            }
        }

        int thickn = 3;                         // thickness
        int bsline = 0;                         // baseline
        int fontFc = cv::FONT_HERSHEY_SIMPLEX;  // fontFace
        int fontSc = 2;                         // fontScale

        // Place a centered title on top
        string title = _limb + " ARM";
        cv::Size textSize = cv::getTextSize( title, fontFc, fontSc, thickn, &bsline);
        cv::Point textOrg((img.cols - textSize.width)/2, (img.rows + textSize.height)/6);
        putText(img, title, textOrg, fontFc, fontSc, col, thickn, CV_AA);

        if (state.state !=" ")
        {
            putText(img,    "state:", cv::Point( 20,300-60), fontFc, fontSc/2, col, 2, 8);
            putText(img, state.state, cv::Point(150,300-60), fontFc,   fontSc, col, thickn, CV_AA);
        }
        if (state.action !=" ")
        {
            putText(img,    "action:", cv::Point( 20,400-60), fontFc,    fontSc/2, col, 2, 8);
            putText(img, state.action, cv::Point(150,400-60), fontFc, fontSc/1.25, col, thickn, CV_AA);
        }
        if (state.object !=" ")
        {
            putText(img,    "object:", cv::Point( 20,500-60), fontFc,    fontSc/2, col, 2, 8);
            putText(img, state.object, cv::Point(150,500-60), fontFc, fontSc/1.25, col, thickn, CV_AA);
        }

        return img;
    };

    /**
     * Function to create sub-image for the bottom bar (to be filled with status icons)
     * @return the sub-image
     */
    cv::Mat createBtmImage()
    {
        cv::Mat res(w_b-w_d/2,w,CV_8UC3,cv::Scalar::all(255));
        ROS_INFO_COND(print_level>=6, "Created BOTTOM image with size %i %i", res.rows, res.cols);

        return res;
    }

public:

    /**
     * Constructor
     */
    explicit BaxterDisplay(string _name) : print_level(0), name(_name), speech(""), it(nh)
    {
        im_pub = it.advertise("/robot/xdisplay", 1);

        l_sub = nh.subscribe( "/action_provider/left/state", 1, &BaxterDisplay::armStateCbL, this);
        r_sub = nh.subscribe("/action_provider/right/state", 1, &BaxterDisplay::armStateCbR, this);

        s_sub = nh.subscribe("/svox_tts/speech_output",1, &BaxterDisplay::speechCb, this);

        nh.param<int> ("/print_level", print_level, 0);

        h = 600;
        w = 1024;

        w_d =  8;
        w_b = 80;

        l_state.state  = "START";
        l_state.action =     "";
        l_state.object =     "";

        r_state.state  = "START";
        r_state.action =     "";
        r_state.object =     "";

        red   = cv::Scalar(  44,  48, 201);  // BGR color code
        green = cv::Scalar(  60, 160,  60);
        blue  = cv::Scalar( 200, 162,  77);

        nh.param<double>("baxter_display/speech_duration", speech_duration, DEFAULT_DURATION);

        displayArmStates();

        ROS_INFO_COND(print_level>=3, "Subscribing to %s and %s", l_sub.getTopic().c_str(),
                                                                  r_sub.getTopic().c_str());
        ROS_INFO_COND(print_level>=3, "Subscribing to %s", s_sub.getTopic().c_str());
        ROS_INFO_COND(print_level>=3, "Publishing  to %s", im_pub.getTopic().c_str());
        ROS_INFO_COND(print_level>=1, "Print     Level set to %i", print_level);
        ROS_INFO_COND(print_level>=1, "Speech Duration set to %g", speech_duration);
        ROS_INFO_COND(print_level>=1, "Ready!!");
    };

    /**
     * Function to set the speech to a specific value
     * @param s the speech text
     */
    void setSpeech(const std::string &s)
    {
        speech = s;
    }

    /**
     * Function to display arm states into a single image. It combines each individual sub-image
     * (the one for the left arm, the one for the right arm, and the bottom status bar)
     * @return true/false if success/failure
     */
    bool displayArmStates()
    {
        cv::Mat l = createArmImage("LEFT");
        cv::Mat r = createArmImage("RIGHT");
        cv::Mat b = createBtmImage();
        cv::Mat d_v(r.rows,w_d,CV_8UC3,cv::Scalar::all(80));    // Vertical delimiter
        cv::Mat d_h(w_d,w,CV_8UC3,cv::Scalar::all(80));         // Horizontal delimiter

        ROS_INFO_COND(print_level>=5, "d_v size %i %i", d_v.rows, d_v.cols);
        ROS_INFO_COND(print_level>=5, "d_h size %i %i", d_h.rows, d_h.cols);

        cv::Mat res(h,w,CV_8UC3,cv::Scalar(255,100,255));

        // Draw sub-image for right arm
        r.copyTo(res(cv::Rect(0, 0, r.cols, r.rows)));

        // Draw sub-image for the vertical delimiter
        d_v.copyTo(res(cv::Rect(r.cols, 0, d_v.cols, d_v.rows)));

        // Draw sub-image for left arm
        l.copyTo(res(cv::Rect(r.cols+d_v.cols, 0, l.cols, l.rows)));

        // Draw sub-image for horizontal delimiter
        d_h.copyTo(res(cv::Rect(0, r.rows, d_h.cols, d_h.rows)));

        // Draw sub-image for bottom bar
        b.copyTo(res(cv::Rect(0, r.rows+d_h.rows, b.cols, b.rows)));

        // Eventually draw the speech on top of everything
        displaySpeech(res);

        // Publish the resulting image
        cv_bridge::CvImage msg;
        msg.encoding = sensor_msgs::image_encodings::BGR8;
        msg.image    = res;

        im_pub.publish(msg.toImageMsg());

        // cv::imshow("res", res);
        // cv::waitKey(20);

        return true;
    };

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

