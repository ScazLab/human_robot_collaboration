#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <pthread.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/JointCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Range.h>

#include "utils.h"
#include "gripper.h"

/**
 * @brief A ROS Thread class
 * @details This class initializes overhead functions necessary to start a thread
 *          from within a class, and overhead ROS features: subscriber/publishers,
 *          services, callback functions etc.
 */
class ROSThread
{
private:
    ros::Time _init_time;
    State _state;
    std::string _limb;

    pthread_t _thread;
    static void * InternalThreadEntryFunc(void * This);

    ros::Subscriber _endpt_sub;
    ros::Subscriber _ir_sub;
    ros::Subscriber _cuff_OK_sub;  // Cuff OK button
    ros::ServiceClient _ik_client;

protected:

    ros::NodeHandle _n;

    geometry_msgs::Pose   _curr_pose;
    geometry_msgs::Point  _curr_position;
    geometry_msgs::Wrench _curr_wrench;

    std::vector<double> _filt_force;

    float _curr_range, _curr_max_range, _curr_min_range;

    ttt::Gripper   *_gripper;

    ros::Publisher  _joint_cmd_pub;

    void pause();

    /*
     * Function that will be spun out as a thread
     */
    virtual void InternalThreadEntry() = 0;

    /*
     * Uses built in IK solver to find joint angles solution for desired pose
     * 
     * @param     requested PoseStamped
     * @param     array of joint angles solution
     * @return    true/false if success/failure
     */
    bool getJointAngles(geometry_msgs::PoseStamped& pose_stamped, std::vector<double>& joint_angles);

    /*
     * Moves arm to the requested pose
     * 
     * @param  requested pose (3D position + 4D quaternion for the orientation)
     * @param  mode (either loose or strict, it checks for the final desired position)
     * @return true/false if success/failure
     */
    bool goToPose(double px, double py, double pz,
                  double ox, double oy, double oz, double ow, std::string mode="loose");

    /*
     * Sets the joint names of a JointCommand
     * 
     * @param    joint_cmd the joint command
     */
    void setJointNames(baxter_core_msgs::JointCommand& joint_cmd);

    /*
     * Detects if the force overcame a set threshold in either one of its three axis
     * 
     * @return true/false if the force overcame the threshold
     */
    bool detectForceInteraction();

    /*
     * Waits for a force interaction to occur.
     * 
     * @return true when the force interaction occurred
     * @return false if no force interaction occurred after 10s
     */
    bool waitForForceInteraction(double _wait_time = 10.0);

    /*
     * Prevents any following code from being executed before thread is exited
     * 
     * @param      N/A
     * @return     true if thread was successfully launched; false otherwise
     */      
    void WaitForInternalThreadToExit();

    /*
     * Callback function that sets the current pose to the pose received from 
     * the endpoint state topic
     * 
     * @param      N/A
     * @return     N/A
     */
    void endpointCallback(const baxter_core_msgs::EndpointState& msg);

    /*
     * Infrared sensor callback function that sets the current range to the range received
     * from the left hand range state topic
     * 
     * @param      The message
     * @return     N/A
     */
    void IRCallback(const sensor_msgs::RangeConstPtr& msg);

    /*
     * Filters the forces with a very simple low pass filter
     */
    void filterForces();

    /*
     * hover arm above tokens
     * 
     * @param      double indicating requested height of arm (z-axis)
     * return     N/A
     */
    void hoverAboveTokens(double height);

public:
    ROSThread(std::string limb);
    virtual ~ROSThread();

    /*
     * Starts thread that executes the internal thread entry function
     * 
     * @param      N/A
     * @return     true if thread was successfully launched; false otherwise
     */        
    bool startInternalThread();

    bool gripObject();

    bool releaseObject();

    /*
     * Self-explaining "setters"
     */   
    void setState(int state);

    /*
     * Self-explaining "getters"
     */
    State       getState() { return _state; };
    std::string getLimb()  { return _limb;  };
};

/**
 * @brief A ROS Thread with an image callbck
 * @details This class inherits from ROSThread, but it adds also an image callback
 *          to be overwritten by its children. Useful to to visual processing.
 */
class ROSThreadImage : public ROSThread
{
private:
    image_transport::ImageTransport _img_trp;
    image_transport::Subscriber _img_sub;

protected:
    cv::Mat _curr_img;
    cv::Size _curr_img_size;
    bool _curr_img_empty;

    pthread_mutex_t _mutex_img;

public:
    ROSThreadImage(std::string limb);
    ~ROSThreadImage();

    /*
     * image callback function that displays the image stream from the hand camera 
     * 
     * @param      The image
     * @return     N/A
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

class MoveToRest : public ROSThreadImage
{
    public:
        MoveToRest(std::string limb);
        ~MoveToRest();

    protected:

        /*
         * moves arm to rest position
         * 
         * @param      N/A
         * return     N/A
         */
        void InternalThreadEntry();
};

class PickUpToken : public ROSThreadImage
{
    public:
        PickUpToken(std::string limb);
        ~PickUpToken();

    protected:

        /*
         * picks up token
         * 
         * @param      N/A
         * return     N/A
         */
        void InternalThreadEntry();

    private:
        typedef std::vector<std::vector<cv::Point> > Contours;

        /*
         * move arm downwards and suck token upon collision
         * 
         * @param      N/A
         * return     N/A
         */
        void gripToken();

        /*
         * check if hand camera detects token 
         * 
         * @param      Point representing offset between the arm's x-y coordinates
         *            and the token
         * return     N/A
         */
        void checkForToken(cv::Point2d &offset);

        /*
         * identifies token and calculates offset distance required to move hand camera
         * to token
         * 
         * @param      Point representing offset between the arm's x-y coordinates
         *            and the token
         * return     N/A
         */
        void processImage(cv::Point2d &offset);

        /*
         * isolates blue colored object in raw image
         * 
         * @param      Mat displaying blue colored objects in raw image
         * return     N/A
         */
        void isolateBlue(cv::Mat &output);

        /*
         * isolates black colored object in raw image
         * 
         * @param      Mat displaying black colored objects in raw image
         * return     N/A
         */
        void isolateBlack(cv::Mat &output);

        /*
         * isolates board boundaries from image 
         * 
         * @param      input Mat, output Mat displaying board boundaries,
         *            and integer indicating lowest y coordinate of board boundaries
         * return     N/A
         */
        void isolateBoard(cv::Mat input, cv::Mat &output, int &board_y);

        /*
         * isolates token from image
         * 
         * @param      input Mat, output Mat displaying token,
         *            and integer indicating lowest y coordinate of board boundaries,
         *            and contours of blue-colored objects in image
         * return     N/A
         */
        void isolateToken(cv::Mat input, int board_y, cv::Mat &output, Contours &contours);

        /*
         * calculates offset distance from arm to token
         * 
         * @param      token contours, an integer indicating lowest y coordinate of board boundaries,
         *            and contours of blue-colored objects in image, and an output Mat displaying token
         * return     N/A
         */
        void setOffset(Contours contours, cv::Point2d &offset, cv::Mat &output);
};

class ScanBoard : public ROSThreadImage 
{
    public:
        ScanBoard(std::string limb);
        ~ScanBoard();

        std::vector<geometry_msgs::Point> getOffsets();

    protected:

        /*
         * scan the board 
         * 
         * @param      N/A
         * return     N/A
         */
        void InternalThreadEntry();

    private:
        typedef std::vector<std::vector<cv::Point> > Contours;
        std::vector<geometry_msgs::Point> _offsets;

        /*
         * hover arm above board
         * 
         * @param      N/A
         * return     N/A
         */
        void hoverAboveBoard();

        /*
         * scan the board and calculate cell offsets
         * 
         * @param      N/A
         * return     N/A
         */
        void scan();

        /*
         * move arm downwards until collision w/ a surface; calculate
         * distance between surface and arm starting point
         * 
         * @param      float * dist indicating the distance btw. the surface
         *            and the arm's starting point
         * return     N/A
         */
        void setDepth(float *dist);

        /*
         * calculate cell offsets; also prompts user to move board withing 'reachable zone'
         * (displayed on screen) if board is out of reach of Baxter's arm 
         * 
         * @param      string mode (test/run) indicating a test (does not exit out of scanning loop)
         *            or an actual run (quits once scanning finished)
         * return     N/A
         */
        void processImage(std::string mode, float dist);

        /*
         * isolates black colored object in raw image
         * 
         * @param      Mat displaying black colored objects in raw image
         * return     N/A
         */
        void isolateBlack(cv::Mat * output);

        /*
         * isolates board boundaries from image 
         * 
         * @param      input Mat, output Mat displaying board boundaries, output Contours
         *            storing board contours, integer indicating the area of the board,
         *            and a vector<cv::Point> of the board's four corners
         * return     N/A
         */
        void isolateBoard(Contours * contours, int * board_area,
                          std::vector<cv::Point> * board_corners, cv::Mat input, cv::Mat * output);

        /*
         * finds cell with the higher centroid
         * 
         * @param      returns true if cell i has a higher centroid than cell j; false otherwise
         * return     N/A
         */
        static bool descendingX(std::vector<cv::Point> i, std::vector<cv::Point> j);

        /*
         * calculates offset distance from arm to each board cell
         * 
         * @param      board area, cell contours, output Mat displaying cells, and height
         *            from arm to board surface
         * return     N/A
         */
        void setOffsets(int board_area, Contours contours, float dist,
                        cv::Mat *output, std::vector<cv::Point> *centroids);

        /*
         * calculates the perimeter of the area representing all points reachable to the Baxter arm
         * 
         * @param      board contours, distance between starting position and play surface, coordinates of all 
         *            4 board corners. input vector containing cell centroids, input vector representing 
         *            distance between center of corner cell and corner of corner cell
         * return     N/A
         */
        void setZone(Contours contours, float dist, std::vector<cv::Point> board_corners,
                     std::vector<cv::Point> *centroids, std::vector<cv::Point> * cell_to_corner);
       
        /*
         * checks if Baxter's arm has a joint angles solution for all the calculated cell offsets
         * 
         * @param      N/A
         * return     true if offsets are all reachable; false otherwise
         */
        bool offsetsReachable();

        /*
         * checks if Baxter's arm has a joint angles solution for a certain point on the board
         * scanning image
         * 
         * @param      N/A
         * return     true if point is reachable; false otherwise
         */
        bool pointReachable(cv::Point centroid, float dist);
};

class PutDownToken : public ROSThreadImage
{
    public:
        PutDownToken(std::string limb);     
        ~PutDownToken();

        void setCell(int cell);
        void setOffsets(std::vector<geometry_msgs::Point> offsets);

    protected:

        /*
         * puts down token in specified cell
         * 
         * @param      N/A
         * return     N/A
         */
        void InternalThreadEntry();

    private:
        int _cell;
        std::vector<geometry_msgs::Point> _offsets;

        /*
         * hover arm above specified cell
         * 
         * @param      N/A
         * return     N/A
         */
        void hoverAboveCell();

        /*
         * hover arm above the board
         * 
         * @param      N/A
         * return     N/A
         */
        void hoverAboveBoard();
};

class ArmController
{
    private:
        std::string _limb;
        MoveToRest * _rest_class;
        PickUpToken * _pick_class;
        ScanBoard * _scan_class;
        PutDownToken * _put_class; 

    public:
        ArmController(std::string limb);
        ~ArmController();

        /*
         * get most recent state change
         * 
         * @param      N/A
         * return     integer indicating arm's latest state
         */
        int getState();

        void moveToRest();

        void pickUpToken();

        void scanBoard();

        void putDownToken(int cell);     
};

#endif
