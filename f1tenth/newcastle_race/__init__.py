"""
Newcastle race source code.

Based on https://github.com/pastankaitis/newcastle_race.
"""
import rospy
from .lib import Racer


def run():
    """
    Start the racer (blocks).
    """
    rospy.init_node("time")
    racer = Racer()
    racer.race()


__all__ = [
    "Racer",
    "run"
]
