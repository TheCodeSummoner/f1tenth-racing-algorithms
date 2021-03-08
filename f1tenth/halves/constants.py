"""
Algorithm-relevant and tuning constants.
"""
# Define parameters here for easier configurability
HORIZON_LENGTH = 4
TIME_STEP = 0.1

# Limits for deciding left and right halves (counter-clockwise)
MAX_INDEX = 1080
RIGHT_DIVERGENCE_INDEX = 300
MID_INDEX = MAX_INDEX // 2
LEFT_DIVERGENCE_INDEX = MAX_INDEX - RIGHT_DIVERGENCE_INDEX

# Values deciding the car's behaviour in case of either left or right half not resulting in any valid FTG points
DEFAULT_TARGET_INDEX_DIVERGENCE = 135
DEFAULT_LEFT_TARGET_INDEX = MID_INDEX + DEFAULT_TARGET_INDEX_DIVERGENCE
DEFAULT_RIGHT_TARGET_INDEX = MID_INDEX - DEFAULT_TARGET_INDEX_DIVERGENCE
DEFAULT_RANGE = 2.5

# Limit for recognising lidar point - any points further than the limit will be ignored
FTG_DISTANCE_LIMIT = 9

# Limiting area for ignoring points next to the closest point
FTG_AREA_RADIUS_SQUARED = 5

# Lidar angle increment and minimum angle
LIDAR_MINIMUM_ANGLE = -3.1415927410125732
LIDAR_ANGLE_INCREMENT = 0.005823155865073204

