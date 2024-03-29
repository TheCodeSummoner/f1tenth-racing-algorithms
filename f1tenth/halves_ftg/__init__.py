"""
Modified FTG with MPC.
"""
import rospy
from ..common import PointFollowerMPC
from .racer import HalvesRacer
from .constants import HORIZON_LENGTH, TIME_STEP


def run():
    """
    Start the racer (blocking call).
    """
    rospy.init_node("time")
    mpc = PointFollowerMPC(horizon_length=HORIZON_LENGTH, time_step=TIME_STEP)
    mpc.setup()
    HalvesRacer(mpc=mpc).start()


__all__ = [
    "HalvesRacer",
    "run"
]
