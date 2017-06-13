#include <gtest/gtest.h>
#include "robot_utils/ros_thread_image.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

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
  int xcoord;
  int ycoord;

public:
    explicit ROSImageInstance(std::string namenew): ROSThreadImage(namenew), xcoord(-1), ycoord(-1) {}
    
    void InternalThreadEntry()
    {
        while(ros::ok())
        {
            cv::Mat img_in;

            if (!_img_empty)
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
                for(unsigned int i = 0; i < contours.size(); i++ )
                {
                    mu[i] = moments(contours[i], false );
                }

                //Finds the Centroid
                vector<Point2f> mc( contours.size());
                for( unsigned int i = 0; i < contours.size(); i++ )
                { 
                    mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
                    xcoord = mu[i].m10/mu[i].m00;
                    ycoord = mu[i].m01/mu[i].m00;
                }

                break;
            }
            r.sleep();
        } 
      }
    int getx() {return xcoord;}
    int gety() {return ycoord;}

    void printx() {std::cout<<"The calculated x-coord of the centroid of the cirlce is " << xcoord<< " [Expected value: 100]"<<std::endl;}
    void printy() {std::cout<<"The calculated y-coord of the centroid of the circle is " << ycoord<< " [Expected value: 100]"<<std::endl;}
};


TEST(rosimagetest, testsetname)
{
    ROSThreadImageTester rtit("test");
    ROSImageInstance rtii("test");

    rtit.sendTestImage();
    ros::Rate rate(100);

    while(ros::ok() && (rtii.getx() == -1 || rtii.gety() == -1))
    {
        ROS_INFO("Waiting for ROSThreadImage loop. [x y]: [%i %i]", rtii.getx(), rtii.gety());
        rate.sleep();
    }

    rtii.printx();
    rtii.printy();

    rtii.setName("newtestname");
    
    //Checks that x and y coordinates are correct
    EXPECT_TRUE(rtii.getx() == 100);
    EXPECT_TRUE(rtii.gety() == 100); 

    //Checks that name is correct
    EXPECT_EQ("newtestname", rtii.getName());
    EXPECT_FALSE("test" == rtii.getName());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_thread_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
