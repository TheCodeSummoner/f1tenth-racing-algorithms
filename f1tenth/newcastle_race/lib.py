"""
Newcastle race re-implementation library code.
"""
from typing import Optional
import rospy
import numpy as np
from rospy import Subscriber, Publisher, Time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from .constants import KP
from .constants import DRIVE_TOPIC, LASER_SCAN_TOPIC, ODOMETRY_TOPIC
from .constants import AVERAGE_SPEED, MAX_SPEED, MIN_SPEED, MAX_SPEED_SELECTION_ANGLE, MIN_SPEED_SELECTION_ANGLE


def _calculate_angle(data: LaserScan) -> float:
    """
    Calculate angle to set.
    """
    angle_to_dist = 135 - _find_angle(data) / 4

    pid_angle = - KP * angle_to_dist

    if (angle_to_dist > 40) or (angle_to_dist < -40):
        pid_angle = np.clip(pid_angle, -0.4, 0.4).item()
    else:
        pid_angle /= 100

    return pid_angle


def _find_angle(data: LaserScan) -> float:
    """
    Find a new steering angle based on the lidar data.
    """
    max_index = 540
    i = 0
    k = 0
    reading_old = 0
    reading = 0

    while i < len(data.ranges):
        if (i <= 300) or (i >= 780):
            k = 0
            reading = 0
        elif data.ranges[i] <= 5.5:
            k = 0
            reading = 0
        else:
            reading += data.ranges[i] - 0.005 * abs(540 - i)
            k += 1
            if k > 10 and reading / k ** 0.3 > reading_old:
                reading_old = reading / k ** 0.3
                max_index = i - k / 2
        i += 1

    return max_index


def _calculate_speed(angle: float) -> float:
    """
    Calculate speed to set.
    """
    angle = abs(angle)

    if angle <= MAX_SPEED_SELECTION_ANGLE:
        return MAX_SPEED

    if angle <= MIN_SPEED_SELECTION_ANGLE:
        return MIN_SPEED

    return AVERAGE_SPEED


class _EnvironmentData:
    """
    Environment measurement data required by the algorithm.

    Note: Odometry data is not actively used, but fetching it is required to trigger the next drive command. It can be
    used for visualisation purposes.
    """

    def __init__(self):
        self.lidar: Optional[LaserScan] = None
        self.pose: Optional[Odometry] = None


class Racer:
    """
    Newcastle racer drives by setting an appropriate angle at each stage.

    It listens to laser scan and odometry data, and publishes steering angle and velocity to the "drive" topic.
    """

    def __init__(self):
        """
        Set up the listener and publisher nodes, as well as initial measurement data and control parameters.
        """
        self._scan_topic = Subscriber(LASER_SCAN_TOPIC, LaserScan, self._scan_callback, queue_size=1)
        self._odom_topic = Subscriber(ODOMETRY_TOPIC, Odometry, self._pose_callback, queue_size=1)
        self._drive_topic = Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self._data = _EnvironmentData()

    @property
    def _command(self):
        """
        Build a new drive command.
        """
        command = AckermannDriveStamped()
        command.header.stamp = Time.now()
        command.header.frame_id = "drive"
        command.drive.steering_angle = _calculate_angle(self._data.lidar)
        command.drive.speed = _calculate_speed(command.drive.steering_angle)
        return command

    def _scan_callback(self, data: LaserScan):
        """
        Update the lidar data.
        """
        self._data.lidar = data

    def _pose_callback(self, data: Odometry):
        """
        Update the pose data and send the next drive command.
        """
        self._data.pose = data
        self._trigger_drive()

    def _trigger_drive(self):
        """
        Publish new data to the "drive" topic, but only if an updated lidar scan is available.
        """
        if self._data.lidar is None:
            return
        self._drive_topic.publish(self._command)

        # Reset lidar data (mark as consumed)
        self._data.lidar = None

    @staticmethod
    def race():
        """
        Start listening to the subscribed topics and running associated callbacks.
        """
        rospy.spin()
