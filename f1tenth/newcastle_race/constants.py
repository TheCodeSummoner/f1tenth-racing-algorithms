"""
Newcastle race re-implementation constants.
"""
import math

# Some mathematical or control-related constants
MAX_SPEED_SELECTION_ANGLE = 15 * math.pi / 180
MIN_SPEED_SELECTION_ANGLE = 20 * math.pi / 180
MAX_SPEED = 7.0
MIN_SPEED = 6.0
AVERAGE_SPEED = 6.5

# Subscriber and publisher topics
LASER_SCAN_TOPIC = "/scan"
ODOMETRY_TOPIC = "/odom"
DRIVE_TOPIC = "/drive"

# PID control parameters
KP = 1.115
