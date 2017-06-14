#include <gtest/gtest.h>
#include "robot_utils/ros_thread_image.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class ROSThreadImageTester
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    std::string name;

public:
    explicit ROSThreadImageTester(std::string _name)
    : it_(nh_), name(_name)
    {
        image_pub_ = it_.advertise("/"+name+"/image", 1);
    }

    ~ROSThreadImageTester() {}

    void sendTestImage()
    {
        cv::Mat img(200, 200, CV_8UC3, cv::Scalar(0,0,0));
        cv::waitKey(30);
        cv::circle(img, cv::Point(100, 100), 20, CV_RGB(255,0,0), -1); 
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        image_pub_.publish(msg); 
    }
};

class ROSImageInstance: public ROSThreadImage
{
private:
    Point avg_coords = Point(-1, -1);

public:
    explicit ROSImageInstance(std::string namenew): ROSThreadImage(namenew) {}
    
    void InternalThreadEntry()
    {
        while(ros::ok())
        {
            cv::Mat img_in;

            if (not _img_empty)
            {
                ROS_INFO("Processing image");
                pthread_mutex_lock(&_mutex_img);
                img_in=_curr_img;
                pthread_mutex_unlock(&_mutex_img);
                cv::Mat gray;
                cv::cvtColor(img_in, gray, CV_BGR2GRAY);
                std::vector<vector<Point>> contours;
                findContours(gray, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0,0));

                //Finds the moments
                vector<Moments> mu(contours.size() );
                for(size_t i = 0; i < contours.size(); i++ )
                {
                    mu[i] = moments(contours[i], false );
                }

                //Finds the Centroid
                vector<Point2f> mc(contours.size());
                for( size_t i = 0; i < contours.size(); i++ )
                { 
                    mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
                    avg_coords.x = mu[i].m10/mu[i].m00;
                    avg_coords.y = mu[i].m01/mu[i].m00;
                }

                break;
            }
            r.sleep();
        } 
    }

//Self explaining return methods  
int returnx() { return avg_coords.x; }

int returny() { return avg_coords.y; }

Point returnpoint() {return avg_coords; }
    
};


TEST(rosimagetest, testinternalthreadentry)
{
    //Creates an object that sends a 200x200 black image with a red circle overlaid
    ROSThreadImageTester rtit("test");
    
    //Creates an object responsible for receiving an image 
    //and finding the centroid of a contour
    ROSImageInstance rtii("test");

    //Sends a 200x200 image with a red circle overlayed in the middle
    rtit.sendTestImage();
    
    //Waits so that threads can finish
    ros::Rate rate(100);

    while(ros::ok() && (rtii.returnx() == -1 || rtii.returny() == -1))
    {
        //ROS_INFO("Waiting for ROSThreadImage loop. [x y]: [%i %i]", rtii.getx(), rtii.gety());
        rate.sleep();
    }

    //Checks that x and y coordinates are correct against expected values
    EXPECT_EQ(rtii.returnpoint(), Point(100, 100));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_thread_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
