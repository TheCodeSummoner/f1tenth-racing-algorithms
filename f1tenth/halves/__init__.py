"""
Modified Follow The Gap with MPC.
"""
import rospy
from .racer import PointFollowerMPC, HalvesRacer


def run():
    """
    Start the racer (blocking call).
    """
    rospy.init_node("time")
    mpc = PointFollowerMPC()
    mpc.setup()
    HalvesRacer(mpc).start()


__all__ = [
    "PointFollowerMPC",
    "HalvesRacer",
    "run"
]
