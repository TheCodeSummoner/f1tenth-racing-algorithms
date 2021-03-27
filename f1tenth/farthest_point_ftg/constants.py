"""
Algorithm-relevant and tuning constants.
"""
from ..common.constants import LIDAR_MAX_INDEX

# Define parameters here for easier configurability
HORIZON_LENGTH = 6
TIME_STEP = 0.027

# FTG scan angle limits
RIGHT_DIVERGENCE_INDEX = 350
LEFT_DIVERGENCE_INDEX = LIDAR_MAX_INDEX - RIGHT_DIVERGENCE_INDEX

# Any points with this range will be avoided at all cost
FTG_IGNORE_RANGE = 0

# Need to store value for computing most likely position (for better MPC predictions)
POSITION_PREDICTION_TIME = 0.020

# Limiting area for ignoring points next to the closest point
SAFETY_RADIUS_SQUARED = 16
