#include <arm_controller/arm_controller.h>
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>

class PickUpARTag : public ROSThread
{
private:
    ros::NodeHandle _nh;
    ros::Subscriber _aruco_sub;

    void ARCallback(const aruco_msgs::MarkerArray& msg);

    geometry_msgs::Pose _curr_marker_pose;

protected:
    /*
     * picks up token
     * 
     * param      N/A
     * return     N/A
     */
    void InternalThreadEntry();

public:
    PickUpARTag(std::string limb);
    ~PickUpARTag();
};