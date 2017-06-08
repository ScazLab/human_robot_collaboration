# Type of command to send. It can be one of the following:
# pose        to request a 6D pose
# position    to request a 3D position and keeping the current orientation
# relative_x  to request an increment relative to the current pose in the x axis
# relative_y  to request an increment relative to the current pose in the y axis
# relative_z  to request an increment relative to the current pose in the z axis
# stop        to stop the motion of the robot altogether (if it was moving)
string type

# Control mode to control the robot with. For now, only position mode is
# allowed. In the near future, also velocity mode will be allowed, in order
# to directly control the robot's joints in velocity.
int32     POSITION_MODE = 1
int32     VELOCITY_MODE = 2
int32 RAW_POSITION_MODE = 4

int32 ctrl_mode

# "POSE" AND "POSITION" GOTOPOSE MESSAGE TYPE
# Endpoint Pose to request Inverse-Kinematics joint solutions for.
# If type is set to position, orientation is ignored and the end effector
# will be moved to the desired position with the current orientation.
geometry_msgs/Point      position
geometry_msgs/Quaternion orientation

# "RELATIVE" GOTOPOSE MESSAGE TYPE
# increment is in meters, and will move the end effector in the desired
# direction, specified by the type
float64 increment

# Mode to check if the position/pose has been reached successfully.
# Either "strict" or "loose"
string STRICT_CHECK = strict
string  LOOSE_CHECK = loose

string check_mode
