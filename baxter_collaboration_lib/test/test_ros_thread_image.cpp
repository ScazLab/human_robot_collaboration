#include <gtest/gtest.h>
#include "robot_utils/ros_thread_image.h"


class ROSThreadImageTester
{
    ros::NodeHandle nh;
    std::string   name;

    image_transport::ImageTransport   it;
    image_transport::Publisher image_pub;

public:
    explicit ROSThreadImageTester(std::string _name = "test") : it(nh), name(_name)
    {
        image_pub = it.advertise("/"+name+"/image", 1);
    }

    void sendTestImage()
    {
        // Let's create an 200x200 black image with a red circle in the center
        cv::Mat img(200, 200, CV_8UC3, cv::Scalar(0,0,0));
        cv::circle(img, cv::Point(100, 100), 20, CV_RGB(255,0,0), -1);

        // Publish the image
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        image_pub.publish(msg);
    }

    ~ROSThreadImageTester() {}
};

class ROSImageInstance: public ROSThreadImage
{
private:
    cv::Point avg_coords;

public:
    explicit ROSImageInstance(std::string _name): ROSThreadImage(_name)
    {
        avg_coords = cv::Point(-1,-1);
    }

    void InternalThreadEntry()
    {
        while(ros::ok())
        {
            cv::Mat img_in;

            if (not _img_empty)
            {
                //ROS_INFO("Processing image");
                pthread_mutex_lock(&_mutex_img);
                img_in=_curr_img;
                pthread_mutex_unlock(&_mutex_img);

                // Convert image to black and white
                cv::Mat gray;
                cv::cvtColor(img_in, gray, CV_BGR2GRAY);

                // Find contours
                std::vector<std::vector<cv::Point>> contours;
                findContours(gray, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0,0));

                //Finds the moments
                std::vector<cv::Moments> mu(contours.size() );
                for(size_t i = 0; i < contours.size(); i++ )
                {
                    mu[i] = moments(contours[i], false );
                }

                //Finds the centroid
                std::vector<cv::Point2f> mc(contours.size());
                for( size_t i = 0; i < contours.size(); i++ )
                {
                    mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
                    avg_coords.x = mu[i].m10/mu[i].m00;
                    avg_coords.y = mu[i].m01/mu[i].m00;
                }

                break;
            }
            r.sleep();
        }
    }

    //Self explaining return methods
    cv::Point centroid() {return avg_coords; }

};


TEST(rosimagetest, testinternalthreadentry)
{
    // Creates an object that sends a 200x200 black image with a red circle overlaid
    ROSThreadImageTester rtit("test");

    // Creates an object responsible for receiving an image
    // and finding the centroid of a contour
    ROSImageInstance rtii("test");

    // Sends a 200x200 image with a red circle overlayed in the middle
    rtit.sendTestImage();

    // Waits so that threads can finish
    ros::Rate rate(100);

    while(ros::ok() && (rtii.centroid() == cv::Point(-1,-1)))
    {
        // ROS_INFO("Waiting for ROSThreadImage loop. [x y]: [%i %i]",
        //                  rtii.centroid().x, rtii.centroid(),y);
        rate.sleep();
    }

    // Checks that x and y coordinates are correct against expected values
    EXPECT_EQ(rtii.centroid(), cv::Point(100, 100));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_thread_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
