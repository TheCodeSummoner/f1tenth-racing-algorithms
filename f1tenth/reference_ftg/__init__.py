"""
Simple follow the gap algorithm package.

Transferred from on https://github.com/pastankaitis/newcastle_race.
"""
import rospy
from .racer import ReferenceRacer


def run():
    """
    Start the racer (blocking call).
    """
    rospy.init_node("time")
    ReferenceRacer().start()


__all__ = [
    "ReferenceRacer",
    "run"
]
