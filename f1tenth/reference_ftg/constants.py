"""
Tuning constants.
"""
import math

# Values required to determine the car's target velocity
MAX_SPEED_SELECTION_THRESHOLD = 15 * math.pi / 180
MIN_SPEED_SELECTION_THRESHOLD = 20 * math.pi / 180
MAX_SPEED = 7.0
MIN_SPEED = 6.0
AVERAGE_SPEED = 6.5

# PID control parameters (this is a simple version of the algorithm, so only the proportional parameter is specified)
KP = 1.115
