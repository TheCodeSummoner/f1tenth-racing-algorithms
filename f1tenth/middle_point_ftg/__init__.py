"""
Middle-point FTG with MPC.
"""
import rospy
from ..common import PointFollowerMPC
from .racer import MiddlePointRacer
from .constants import HORIZON_LENGTH, TIME_STEP


def run():
    """
    Start the racer (blocking call).
    """
    rospy.init_node("time")
    mpc = PointFollowerMPC(horizon_length=HORIZON_LENGTH, time_step=TIME_STEP)
    mpc.setup()
    MiddlePointRacer(mpc=mpc).start()


__all__ = [
    "MiddlePointRacer",
    "run"
]
