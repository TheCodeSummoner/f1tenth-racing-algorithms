"""
Simple follow the gap algorithm package.

Transferred from on https://github.com/pastankaitis/newcastle_race.
"""
import rospy
from .racer import FollowTheGapRacer


def run():
    """
    Start the racer (blocking call).
    """
    rospy.init_node("time")
    FollowTheGapRacer().start()


__all__ = [
    "FollowTheGapRacer",
    "run"
]
