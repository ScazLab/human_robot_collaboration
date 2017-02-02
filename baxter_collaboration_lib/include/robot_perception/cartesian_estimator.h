#ifndef __CARTESIAN_ESTIMATOR__
#define __CARTESIAN_ESTIMATOR__

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/opencv.hpp>

#include <aruco/cameraparameters.h>
#include <aruco_ros/aruco_ros_utils.h>

#include "robot_utils/ros_thread_image.h"

#include "baxter_collaboration_msgs/ObjectsArray.h"

#define AREA_THRES  50      // px

/**
 * Generic class for representing a segmented object. It is a virtual class,
 * and needs to be specified in its derived children.
 */
class SegmentedObj
{
private:
    // Name of the object (for future reference)
    std::string name;

    // Flag to know if the object is there or not
    bool is_there;

public:
    // Real size of the object in meters. We consider only rectangular shapes.
    std::vector<double> size;

    // Minimum area in pixel for a blob to be considered worth evaluating.
    int area_threshold;

    // Segmented objects as a rotated rectangle
    cv::RotatedRect     rect;

    // Rotation and translation matrices with respect to the camera
    cv::Mat Rvec, Tvec;

    // Pose of the object in the root reference frame
    geometry_msgs::Pose pose;

    /* CONSTRUCTOR */
    SegmentedObj(std::vector<double> _size);
    SegmentedObj(std::string _name, std::vector<double> _size, int _area_thres);

    /* DESTRUCTOR */
    virtual ~SegmentedObj();

    /**
     * Class initializer
     */
    void init();

    /**
     * Detects the object in the image
     *
     * @param _in        Input image to detect objects from
     * @param _out       Output image to show the result of the segmentation
     *
     * @return true/false if success/failure
     */
    virtual bool detectObject(const cv::Mat& _in, cv::Mat& _out);

    /**
     * Detects the object in the image
     *
     * @param _in        Input image to detect objects from
     * @param _out       Output image to show the result of the segmentation
     * @param _out_thres Optional output image to show the thresholded image
     *
     * @return true/false if success/failure
     */
    virtual bool detectObject(const cv::Mat& _in, cv::Mat& _out, cv::Mat& _out_thres);

    /**
     * Draws a box in the image where the object is located
     *
     * @param _img      Image to draw the 3D box into
     *
     * @return true/false if success/failure
     */
    bool drawBox(cv::Mat &_img);

    /**
     * Draws a the object name in the image where the object is located
     *
     * @param _img      Image to draw the name into
     *
     * @return true/false if success/failure
     */
    bool drawName(cv::Mat &_img);

    /**
     * Draws a 3D axis in the image where the object is located
     *
     * @param _img      Image to draw the 3D axis into
     * @param _cam_mat  Camera matrix A = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{_1}
     * @param _dist_mat Input vector of distortion coefficients
     *                  (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) of 4, 5, or 8 elements.
     *                  If the vector is NULL/empty, no distortion coefficients are assumed.
     *
     * @return true/false if success/failure
     */
    bool draw3dAxis(cv::Mat &_img, const cv::Mat& _cam_mat,
                                   const cv::Mat& _dist_mat);

    /**
     * Draws a the segmented rectangle, its name and its 3D axis where
     * the object is located in the image.
     *
     * @param _img      Image to draw things into
     * @param _cam_mat  Camera matrix A = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{_1}
     * @param _dist_mat Input vector of distortion coefficients
     *                  (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) of 4, 5, or 8 elements.
     *                  If the vector is NULL/empty, no distortion coefficients are assumed.
     *
     * @return true/false if success/failure
     */
    bool draw(cv::Mat &_img, const cv::Mat& _cam_mat,
                             const cv::Mat& _dist_mat);

    /**
     * Converts the segmented object to a string.
     * @return the segmented object as a string
     */
    virtual std::string toString();

    /* GETTERS */
    bool isThere()        { return is_there; };
    std::string getName() { return     name; };

    /* SETTERS */
    void setIsThere(bool _it)           { is_there = _it; };
    void setName(const std::string &_s) {     name =  _s; };
};

/**
 * Generic helper class to estimate the cartesian position of a blob in the camera
 * reference frame (RF) and project it into any other RF (usually the base RF).
 */
class CartesianEstimator : public ROSThreadImage
{
private:
	/** INTERNAL PARAMETERS **/
    // Image publisher
    image_transport::Publisher  img_pub;

    // Image publisher for the thresholded image
    image_transport::Publisher  img_pub_thres;

    // Publisher of objects' info
    ros::Publisher objs_pub;

    // Message to send through objs_pub
    baxter_collaboration_msgs::ObjectsArray objects_msg;

    // Camera parameters
    aruco::CameraParameters cam_param;

    // Transform listener to convert reference frames
    tf::TransformListener tfListener_;

    /** EXTERNAL PARAMETERS **/
    // Name of the reference frame to transform the camera poses to
    std::string reference_frame;

    // Name of the camera frame to refer the object poses to
    std::string camera_frame;

    // Minimum area threshold in pixel for an object to be considered valid.
    // Used to avoid having erroneous detections due to noise or whatnot.
    int area_threshold;

    /**
     * [getTransform description]
     * @param  refFrame   [description]
     * @param  childFrame [description]
     * @param  transform  [description]
     * @return            true/false if success/failure
     */
    bool getTransform(const std::string& refFrame,
                      const std::string& childFrame, tf::StampedTransform& transform);

    /**
     * Converts the detected object's pose into a TF transform object
     *
     * @param idx the object's pose to convert
     *
     * @return the converted TF Transform object
     */
    tf::Transform object2Tf(int idx);

    /**
     * Class initializer
     */
    void init();

    /**
     * Publishes the array of objects on the proper topic
     *
     * @return true/false if success/failure
     */
    bool publishObjects();

protected:

    // Vector of segmented objects to track
    std::vector<SegmentedObj*>  objs;

    /*
     * Function that will be spun out as a thread
     */
    void InternalThreadEntry();

    /**
     * Adds an object to the database of objects.
     *
     * @param h Its physical height in meters
     * @param w Its physical width in meters
     *
     * @return true/false if success/failure
     */
    bool addObject(std::string _name, double _h, double _w);

    /**
     * Adds the full database of objects from a cv::Mat
     *
     * @param _o the database of objects
     * @return true/false if success/failure
     */
    bool addObjects(std::vector<std::string> _names, cv::Mat _o);

    /**
     * Creates the objects database from an XmlRpcValue::TypeStruct in the parameter server.
     * It is encoded as an entire namespace of parameters using a YAML dictionary.
     * The format is an array of arrays composed of a string (the name of the obejct),
     * and the width and heigth of the object as another array.
     * This is a valid parameter to set in your launch file:
     *
     * <rosparam param = "cartesian_estimator/objects">
     *    [["screwdriver", [0.0525, 0.015]],
     *     ["blue_box"   ,  [0.111, 0.052]]]
     * </rosparam>
     *
     * @param  _param the XmlRpcValue read from the parameter server.
     * @return        true/false if the insertion was successful or not
     */
    bool addObjects(XmlRpc::XmlRpcValue _params);

    /**
     * Detects the object in the image
     */
    bool detectObjects(const cv::Mat& _in, cv::Mat& _out);

    /**
     * Prints the object database to screen.
     */
    void printObjectDB();

    /**
     * Converts the action database to a string.
     * @return the list of allowed actions, separated by a comma.
     */
    std::string objectDBToString();

    /**
     * Calculates the cartesian pose of the segmented object in the camera frame
     * given the rotated bounding box, the camera parameters and the real physical size of the object.
     *
     * @param idx the object's index
     *
     * @return true/false if success/failure
     */
    bool poseCameraRF(int idx);

    /**
     * Projects the cartesian pose of the segmented object from the camera frame to the root frame,
     * given the kinematics of the robot, and the pose in the camera frame
     *
     * @param idx the object's index
     *
     * @return true/false if success/failure
     */
    bool cameraRFtoRootRF(int idx);

    /**
     * Calculates the cartesian pose of all the segmented objects in the root frame
     *
     * @return true/false if success/failure
     */
    bool poseRootRF();

    /**
     * Calculates the cartesian pose of the segmented object in the root frame
     * given the rotated bounding box, the camera parameters, the real physical
     * size of the object, and the kinematics of the robot
     *
     * @param idx the object's index
     *
     * @return true/false if success/failure
     */
    bool poseRootRF(int idx);

    /**
     * Draws all objects in the image where they are located
     *
     * @param _img Image to draw the objects into
     *
     * @return true/false if success/failure
     */
    bool draw(cv::Mat &_img);

    /**
     * Draws an object in the image where they are located
     *
     * @param _img Image to draw the object into
     * @param  idx The object's index
     *
     * @return true/false if success/failure
     */
    bool draw(cv::Mat &_img, int idx);

    /**
     * Computes the number of valid objects visible on the current frame
     *
     * @return the number of valid objects
     */
    int getNumValidObjects();

    /**
     * Clears the array of objects and properly deallocates memory by
     * destroying its elements one by one
     */
    void clearObjs();

public:
    /* CONSTRUCTORS */
    CartesianEstimator(std::string _name);
    CartesianEstimator(std::string _name,
                       std::vector<std::string> _objs_name, cv::Mat _objs_size);

    /* DESTRUCTOR */
    ~CartesianEstimator();

    /** GETTERS **/
    int getAreaThreshold() { return area_threshold; };

    /** SETTERS **/

};

#endif
