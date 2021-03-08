"""
Waypoints-based trajectory follower MPC.
"""
import rospy
from .racer import WaypointsFollowerRacer
from .constants import HORIZON_LENGTH, TIME_STEP
from ..common import PointFollowerMPC


def run():
    """
    Start the racer (blocking call).
    """
    rospy.init_node("time")
    mpc = PointFollowerMPC(horizon_length=HORIZON_LENGTH, time_step=TIME_STEP)
    mpc.setup()
    WaypointsFollowerRacer(mpc=mpc).start()


__all__ = [
    "WaypointsFollowerRacer",
    "run"
]
