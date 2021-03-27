"""
Collection of F1TENTH-compatible autonomous control algorithms.
"""
from . import reference_ftg
from . import waypoints_follower
from . import halves_ftg
from . import middle_point_ftg
from . import farthest_point_ftg

__all__ = [
    "reference_ftg",
    "waypoints_follower",
    "halves_ftg",
    "middle_point_ftg",
    "farthest_point_ftg",
]
