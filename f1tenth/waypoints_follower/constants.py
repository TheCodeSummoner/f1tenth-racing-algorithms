"""
Algorithm-relevant and tuning constants.
"""
import os

# Define parameters here for easier configurability
HORIZON_LENGTH = 4
TIME_STEP = 0.1

# Cost function Tuning parameters
DISTANCE_IMPORTANCE = 2.5
VELOCITY_IMPORTANCE = 1

# CSV waypoints file location
WAYPOINTS_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                                   "assets", "waypoints", "skirk-waypoints.csv")

# Threshold deciding what the minimum distance should be to select the next waypoint
NEXT_WAYPOINT_THRESHOLD = 3
