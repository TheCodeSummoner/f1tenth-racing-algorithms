"""
Waypoints-based trajectory follower MPC.
"""
import rospy
from .racer import WaypointsFollowerRacer, WaypointsFollowerMPC


def run():
    """
    Start the racer (blocking call).
    """
    rospy.init_node("time")
    mpc = WaypointsFollowerMPC()
    mpc.setup()
    WaypointsFollowerRacer(mpc).start()


__all__ = [
    "WaypointsFollowerRacer",
    "WaypointsFollowerMPC",
    "run"
]
