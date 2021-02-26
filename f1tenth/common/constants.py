"""
Share-able constants.
"""
# Frame ID shouldn't change - it is defined within the F1/10 ROS simulator
FRAME_ID = "/map"

# Drive topics (closely related to ROS technological stack)
LASER_SCAN_TOPIC = "/scan"
ODOMETRY_TOPIC = "/odom"
DRIVE_TOPIC = "/drive"

# Front and rear wheel distances, wheelbase = lr + lf
LR = 0.17145
LF = 0.15875
WHEELBASE_LENGTH = 0.3302