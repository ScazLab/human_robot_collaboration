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

class SegmentedObj
{
public:

    // Segmented objects as a rotated rectangle
    cv::RotatedRect     rect;

    // Real size of the object in meters. We consider only rectangular shapes.
    std::vector<double> size;

    // Rotation and translation matrices with respect to the camera
    cv::Mat Rvec, Tvec;

    /* CONSTRUCTOR */
    SegmentedObj(std::vector<double> _size);

    /* DESTRUCTOR */
    virtual ~SegmentedObj();

    /**
     * Detects the object in the image
     */
    virtual bool detectObject(const cv::Mat& _in, cv::Mat& _out) { return false; };
};

/**
 * Generic helper class to estimate the cartesian position of a blob in the camera
 * reference frame (RF) and project it into any other RF (usually the base RF).
 */
class CartesianEstimator : public ROSThreadImage
{
private:
    // Image publisher
    image_transport::Publisher  img_pub;

    // Camera parameters
    aruco::CameraParameters cam_param;

    // Name of the reference frame to transform the camera poses to
    std::string reference_frame_;

    // Name of the camera frame to refer the object poses to
    std::string camera_frame_;

    // Transform listener to convert reference frames
    tf::TransformListener tfListener_;

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
    bool addObject(double _h, double _w);

    /**
     * Adds the full database of objects from a cv::Mat
     *
     * @param _o the database of objects
     * @return true/false if success/failure
     */
    bool objsFromMat(cv::Mat _o);

    /**
     * Detects the object in the image
     */
    virtual bool detectObjects(const cv::Mat& _in, cv::Mat& _out) { return false; };

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
     * Draws a 3D axis in the object
     *
     * @param _in Image to draw the 3D axis into
     * @param idx The object's index
     *
     * @return true/false if success/failure
     */
    bool draw3dAxis(cv::Mat &_in, int idx);

    /**
     * Clears the array of objects and properly deallocates memory by
     * destroying its elements one by one
     */
    void clearObjs();

public:
    /* CONSTRUCTORS */
    CartesianEstimator(std::string _name);
    CartesianEstimator(std::string _name, cv::Mat _objs_size);

    /* DESTRUCTOR */
    ~CartesianEstimator();

    /** GETTERS **/
    // cv::RotatedRect getSegmentedObject() { return obj_segm; };

    /** SETTERS **/
    // void setSegmentedObject(cv::RotatedRect _os) { obj_segm = _os; return; };
};

#endif
