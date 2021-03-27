"""
Algorithm-relevant and tuning constants.
"""
from ..common.constants import LIDAR_MAX_INDEX

# Define parameters here for easier configurability
HORIZON_LENGTH = 6
TIME_STEP = 0.027

# Limits for deciding left and right halves (counter-clockwise)
RIGHT_DIVERGENCE_INDEX = 350
MID_INDEX = LIDAR_MAX_INDEX // 2
LEFT_DIVERGENCE_INDEX = LIDAR_MAX_INDEX - RIGHT_DIVERGENCE_INDEX

# Values deciding the car's behaviour in case of either left or right half has no valid FTG points
# This it to avoid index errors on trying to access empty point arrays
DEFAULT_TARGET_INDEX_DIVERGENCE = 135
DEFAULT_LEFT_TARGET_INDEX = MID_INDEX + DEFAULT_TARGET_INDEX_DIVERGENCE
DEFAULT_RIGHT_TARGET_INDEX = MID_INDEX - DEFAULT_TARGET_INDEX_DIVERGENCE
DEFAULT_RANGE = 1

# Limit for recognising lidar point - any points further than the limit will be replaced with a different value
FTG_DISTANCE_LIMIT = 9

# Any points with this range will be avoided at all cost
FTG_IGNORE_RANGE = 0

# Limiting area for ignoring points next to the closest point
FTG_AREA_RADIUS_SQUARED = 16

# Lidar angle increment and minimum angle (max angle is max index * increment + min angle, which gives pi)
LIDAR_MINIMUM_ANGLE = -3.1415927410125732
LIDAR_ANGLE_INCREMENT = 0.005823155865073204

# Need to store value for computing most likely position (for better MPC predictions)
POSITION_PREDICTION_TIME = 0.028
