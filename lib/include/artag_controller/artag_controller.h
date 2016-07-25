#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <aruco_msgs/MarkerArray.h>
#include <arm_controller/arm_controller.h>

#define POS_HIGH        0.400
#define POS_LOW         0.150
#define PICK_UP_SPEED   0.1

class ARTagController : public ROSThread
{
private:
    double elapsed_time;

    ros::NodeHandle _nh;
    ros::Subscriber _aruco_sub;

    geometry_msgs::Pose _curr_marker_pose;

    void goToPose(geometry_msgs::PoseStamped req_pose_stamped);

    void ARCallback(const aruco_msgs::MarkerArray& msg);

    void hoverAboveTokens(double height);

    bool pickARTag();

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
