"""
Simple follow the gap algorithm package.

Transferred from on https://github.com/pastankaitis/newcastle_race.
"""
import rospy
from .racer import Racer


def run():
    """
    Start the racer (blocking call).
    """
    rospy.init_node("time")
    Racer().start()


__all__ = [
    "Racer",
    "run"
]
