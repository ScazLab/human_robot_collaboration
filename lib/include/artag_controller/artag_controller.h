#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <aruco_msgs/MarkerArray.h>
#include <arm_controller/arm_controller.h>

class ARTagController : public ROSThread
{
private:
    ros::NodeHandle _nh;
    ros::Subscriber _aruco_sub;

    geometry_msgs::Pose _curr_marker_pose;

    void ARCallback(const aruco_msgs::MarkerArray& msg);

    void hoverAboveTokens(std::string height);

    void pickARTag();

protected:
    /*
     * picks up token
     * 
     * param      N/A
     * return     N/A
     */
    void InternalThreadEntry();

public:
    ARTagController(std::string limb);
    ~ARTagController();
};