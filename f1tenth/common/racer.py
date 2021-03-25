"""
Racer base class module.

Relevant algorithms should derive from the base class defined below.
"""
import math
import timeit
from typing import Optional, Tuple, Iterable, List
from abc import ABC, abstractmethod
import numpy as np
import rospy
from rospy import Subscriber, Publisher, Time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from .point import CartesianPoint
from . import marker
from .marker import MarkerColour
from .constants import LASER_SCAN_TOPIC, DRIVE_TOPIC, ODOMETRY_TOPIC
from .constants import LR, WHEELBASE_LENGTH
from .constants import LIDAR_MINIMUM_ANGLE, LIDAR_ANGLE_INCREMENT, LIDAR_MAX_INDEX
from .constants import LAP_FINISH_DISTANCE, LAP_CHECK_DISTANCE


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
        self._velocity: float = 0
        self._steering_angle: float = 0

        # Need to record lap-related information for measuring approximated lap time
        self._lap_start_timestamp: Optional[float] = None
        self._race_start_position: Optional[Tuple[CartesianPoint, CartesianPoint]] = None
        self._check_lap_finished: bool = False

    @property
    def velocity(self):
        """
        Get current vehicle's velocity.
        """
        return self._velocity

    @velocity.setter
    def velocity(self, value: float):
        """
        Set current velocity and mark lap timer start if the vehicle has just started moving.
        """
        self._velocity = value
        self._command.drive.speed = value
        if value > 0 and self._lap_start_timestamp is None:
            self._lap_start_timestamp = timeit.default_timer()

    @property
    def steering_angle(self):
        """
        Get current steering angle.
        """
        return self._steering_angle

    @steering_angle.setter
    def steering_angle(self, value: float):
        """
        Set the steering angle.
        """
        self._steering_angle = value
        self._command.drive.steering_angle = value

    def _mark_race_start(self):
        """
        Store the lap line position upon starting the race.

        The lap line is created by finding points immediately to the left and to the right of the vehicle (at 90 and 270
        degrees). The points are then stored in cartesian coordinate format.
        """
        position_x, position_y = self._retrieve_position()
        heading_angle = self._retrieve_heading_angle()

        # Find out which indexes correspond to the left and right hand side of the vehicle
        right_index = round(math.pi / 2 / LIDAR_ANGLE_INCREMENT)
        left_index = LIDAR_MAX_INDEX - right_index

        # Find lidar point to the left and to the right of the vehicles
        race_start_point_right = self.lidar_to_cartesian(
            ranges=[self._lidar_data.ranges[right_index]],
            position_x=position_x,
            position_y=position_y,
            heading_angle=heading_angle,
            starting_index=right_index
        )[0]
        race_start_point_left = self.lidar_to_cartesian(
            ranges=[self._lidar_data.ranges[left_index]],
            position_x=position_x,
            position_y=position_y,
            heading_angle=heading_angle,
            starting_index=left_index
        )[0]

        # Lap line (or rather, lap segment) is defined by two points
        self._race_start_position = [race_start_point_left, race_start_point_right]
        marker.mark_array(
            positions=self._race_start_position,
            scale=0.2,
            colour=MarkerColour(1, 1, 0),
            duration=0,
        )

    def _lap_passed(self):
        """
        Check whether a new lap should be started.
        """
        position_x, position_y = self._retrieve_position()
        point_a, point_b = self._race_start_position

        # Compute distance of car position to the lap line (segment)
        length = (point_a.x - point_b.x) ** 2 + (point_a.y - point_b.y) ** 2
        a_difference = np.array([position_x - point_a.x, position_y - point_a.y])
        b_difference = np.array([point_b.x - point_a.x, point_b.y - point_a.y])
        clamped = max(0, min(1, (np.dot(a_difference, b_difference)) / length))
        projected = CartesianPoint(
            point_a.x + clamped * (point_b.x - point_a.x),
            point_a.y + clamped * (point_b.y - point_a.y)
        )
        distance_squared = (position_x - projected.x) ** 2 + (position_y - projected.y) ** 2

        # If the vehicle was previously far away from the starting point and now is close
        if self._check_lap_finished and distance_squared <= LAP_FINISH_DISTANCE:
            return True

        # If the vehicle was previously close to the starting point and is now far away
        if not self._check_lap_finished and distance_squared >= LAP_CHECK_DISTANCE:
            self._check_lap_finished = True

        return False

    def _mark_new_lap(self):
        """
        Start a new lap.
        """
        lap_end_time = timeit.default_timer()
        print("Lap passed in: ", lap_end_time - self._lap_start_timestamp)
        self._lap_start_timestamp = lap_end_time
        self._check_lap_finished = False

    def _trigger_drive(self):
        """
        Publish new data to the "drive" topic, but only if an updated lidar scan is available.

        Additionally, handle against lap-related checks.
        """
        if self._odometry_data is not None and self._lidar_data is not None:
            if self._race_start_position is None:
                self._mark_race_start()
            if self._lap_passed():
                self._mark_new_lap()
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

    def predict_trajectory(self, velocity: float, steering_angle: float, steps_count: int = 10,
                           time_step: float = 0.025, position_x: float = None, position_y: float = None,
                           heading_angle: float = None) -> Tuple[List[CartesianPoint], List[float]]:
        """
        Generate predicted trajectory points.
        """
        predicted_positions, predicted_heading_angles = list(), list()
        if position_x is None or position_y is None:
            position_x, position_y = self._retrieve_position()
        heading_angle = self._retrieve_heading_angle() if heading_angle is None else heading_angle

        # Need this to compute the new heading angle after each step
        predicted_heading_angle = heading_angle

        for step_counter in range(steps_count):
            time_delta = time_step + step_counter * time_step
            slip_factor = math.atan(LR * math.tan(steering_angle) / WHEELBASE_LENGTH)
            predicted_position_x = position_x + time_delta * velocity * math.cos(predicted_heading_angle + slip_factor)
            predicted_position_y = position_y + time_delta * velocity * math.sin(predicted_heading_angle + slip_factor)
            predicted_heading_angle = heading_angle + time_delta * velocity * math.tan(steering_angle) \
                * math.cos(slip_factor) / WHEELBASE_LENGTH
            predicted_heading_angles.append(predicted_heading_angle)
            predicted_positions.append(CartesianPoint(predicted_position_x, predicted_position_y))

        return predicted_positions, predicted_heading_angles

    @staticmethod
    def lidar_to_cartesian(ranges: Iterable, position_x: float, position_y: float, heading_angle: float,
                           starting_index: int = 0) -> List[CartesianPoint]:
        """
        Convert lidar points to a collection of cartesian coordinates.

        Starting index can be used to start counting lidar points from a non-zero starting point.
        """
        points = []
        for index, lidar_range in enumerate(ranges):
            laser_beam_angle = ((starting_index + index) * LIDAR_ANGLE_INCREMENT) + LIDAR_MINIMUM_ANGLE
            rotated_angle = laser_beam_angle + heading_angle
            x_coordinate = lidar_range * math.cos(rotated_angle) + position_x
            y_coordinate = lidar_range * math.sin(rotated_angle) + position_y
            points.append(CartesianPoint(x_coordinate, y_coordinate))

        return points

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
