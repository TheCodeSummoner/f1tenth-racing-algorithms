"""
Algorithm-relevant and tuning constants.
"""
# Define parameters here for easier configurability
HORIZON_LENGTH = 8
TIME_STEP = 0.027

# Limits for deciding left and right halves (counter-clockwise)
MAX_INDEX = 1080
RIGHT_DIVERGENCE_INDEX = 350
MID_INDEX = MAX_INDEX // 2
LEFT_DIVERGENCE_INDEX = MAX_INDEX - RIGHT_DIVERGENCE_INDEX

# Values deciding the car's behaviour in case of either left or right half has no valid FTG points
# This it to avoid index errors on trying to access empty point arrays
DEFAULT_TARGET_INDEX_DIVERGENCE = 135
DEFAULT_LEFT_TARGET_INDEX = MID_INDEX + DEFAULT_TARGET_INDEX_DIVERGENCE
DEFAULT_RIGHT_TARGET_INDEX = MID_INDEX - DEFAULT_TARGET_INDEX_DIVERGENCE
DEFAULT_RANGE = 2.5

# Limit for recognising lidar point - any points further than the limit will be ignored
FTG_DISTANCE_LIMIT = 9

# Limiting area for ignoring points next to the closest point
FTG_AREA_RADIUS_SQUARED = 16

# Lidar angle increment and minimum angle (max angle is max index * increment + min angle, which gives pi)
LIDAR_MINIMUM_ANGLE = -3.1415927410125732
LIDAR_ANGLE_INCREMENT = 0.005823155865073204
