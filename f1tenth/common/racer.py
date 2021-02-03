"""
Racer base class module.

Relevant algorithms should derive from the base class defined below.
"""
import math
from typing import Optional, Tuple
from abc import ABC, abstractmethod
import rospy
from rospy import Subscriber, Publisher, Time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

# Drive topics are closely related to ROS technological stack
LASER_SCAN_TOPIC = "/scan"
ODOMETRY_TOPIC = "/odom"
DRIVE_TOPIC = "/drive"


class Racer(ABC):
    """
    Abstract class allowing interacting with ROS through reading the environment data and driving the vehicle.

    Most generally, public methods of this class can be overridden, while the private ones should be left untouched.

    The `prepare_drive_command` must be implemented and within its body the deriving class should populate relevant
    vehicle steering fields.

    To start a racer (and hence connect to ROS master node) simply call:

        racer.start()

    where `racer` is an instance of the deriving class. This is a blocking call. Remember that some rospy node must be
    initialised before the default `start` method functionality will work.
    """

    def __init__(self):
        self._lidar_topic = Subscriber(LASER_SCAN_TOPIC, LaserScan, self.on_lidar_update, queue_size=1)
        self._odometry_topic = Subscriber(ODOMETRY_TOPIC, Odometry, self.on_odometry_update, queue_size=1)
        self._drive_topic = Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
        self._lidar_data: Optional[LaserScan] = None
        self._odometry_data: Optional[Odometry] = None
        self._command: AckermannDriveStamped = None

    def _trigger_drive(self):
        """
        Publish new data to the "drive" topic, but only if an updated lidar scan is available.
        """
        if self._odometry_data is not None and self._lidar_data is not None:
            self._initialise_drive_command()
            self.prepare_drive_command()
            self._publish_drive_command()

    def _initialise_drive_command(self):
        """
        Build the drive command and populating some essential header fields.
        """
        self._command = AckermannDriveStamped()
        self._command.header.stamp = Time.now()
        self._command.header.frame_id = "drive"

    def _publish_drive_command(self):
        """
        Publish the drive command to the relevant topic (and hence drive the vehicle in the simulator).
        """
        self._drive_topic.publish(self._command)

    def _retrieve_position(self) -> Tuple[float, float]:
        """
        Get current vehicle's position in (x, y) coordinates.
        """
        return self._odometry_data.pose.pose.position.x, self._odometry_data.pose.pose.position.y

    def _retrieve_heading_angle(self) -> float:
        """
        Get current vehicle's heading angle (yaw) with respect to the y-axis.
        """
        quaternion_z = self._odometry_data.pose.pose.orientation.z
        quaternion_w = self._odometry_data.pose.pose.orientation.w
        return math.atan2(2 * (quaternion_z * quaternion_w), 1 - 2 * (quaternion_z * quaternion_z))

    def on_lidar_update(self, data: LaserScan):
        """
        Update the lidar data.

        Override this method for additional functionality. Keep in mind that calling super().on_lidar_update is
        recommended for deriving classes.
        """
        self._lidar_data = data

    def on_odometry_update(self, data: Odometry):
        """
        Update the pose data and send the next drive command.

        Override this method for additional functionality. Keep in mind that calling super().on_odometry_update is
        recommended for deriving classes.
        """
        self._odometry_data = data
        self._trigger_drive()

    @abstractmethod
    def prepare_drive_command(self):
        """
        Abstract method which should specify how the vehicle will be driven.

        Most generally you should be setting the following values:

            - command.steering_angle
            - command.speed

        Other AckermannDriveStamped fields can be populated, but may not work with ROS as expected (or at all).
        """

    @staticmethod
    def start():
        """
        Start listening to the subscribed topics and calling associated callbacks.
        """
        rospy.spin()
