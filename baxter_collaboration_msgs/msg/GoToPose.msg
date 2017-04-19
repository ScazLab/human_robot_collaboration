# Type of command to send. It can be either pose (to request a 6D pose),
# position (to request a 3D position and keeping the current orientation),
# or relative (to request an increment relative to the current pose, see below)
string type

# Control mode to control the robot with. For now, only position mode is
# allowed. In the near future, also velocity mode will be allowed, in order
# to directly control the robot's joints in velocity.
int32     POSITION_MODE = 1
int32     VELOCITY_MODE = 2
int32 RAW_POSITION_MODE = 4

int32 ctrl_mode

# POSE AND POSITION GOTOPOSE MESSAGE TYPE
geometry_msgs/Point      position
geometry_msgs/Quaternion orientation

# RELATIVE GOTOPOSE MESSAGE TYPE
# direction can be one of the following:
# backward, forward, right, left, down, up
string direction
# increment is in meters
float64 increment

# Endpoint Pose(s) to request Inverse-Kinematics joint solutions for.
# If pose_stamp.pose.orientation is set to -100, the end effector will
# be moved to the desired position with the current orientation.
geometry_msgs/PoseStamped pose_stamp
