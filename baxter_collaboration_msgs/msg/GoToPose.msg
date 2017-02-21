# Endpoint Pose(s) to request Inverse-Kinematics joint solutions for.
# If pose_stamp.pose.orientation is set to -100, the end effector will
# be moved to the desired position with the current orientation.
geometry_msgs/PoseStamped pose_stamp

# Control mode to control the robot with. For now, only position mode is
# allowed. In the near future, also velocity mode will be allowed, in order
# to directly control the robot's joints in velocity.
uint8 POSITION_MODE = 1
uint8 VELOCITY_MODE = 2

uint8 ctrl_mode
