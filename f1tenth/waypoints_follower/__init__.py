"""
Waypoints-based trajectory follower MPC.
"""
import rospy
from .racer import WaypointsFollowerRacer


def run():
    """
    Start the racer (blocking call).
    """
    rospy.init_node("time")
    waypoints_follower_racer = WaypointsFollowerRacer()
    waypoints_follower_racer.start()


__all__ = [
    "WaypointsFollowerRacer",
    "run"
]
