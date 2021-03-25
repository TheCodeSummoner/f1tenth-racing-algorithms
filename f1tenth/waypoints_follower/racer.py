"""
Waypoints-based trajectory MPC.
"""
import csv
from typing import List
import numpy as np
from .constants import WAYPOINTS_FILE_PATH, NEXT_WAYPOINT_THRESHOLD
from ..common import Racer, PointFollowerMPC, marker, CartesianPoint
from ..common.marker import MarkerColour, MarkerPublisherChannel


class WaypointsFollowerRacer(Racer):
    """
    Waypoints racer drives by following some chosen point within its reference trajectory.
    """

    def __init__(self, mpc: PointFollowerMPC, waypoints_file_path: str = WAYPOINTS_FILE_PATH):
        super().__init__()
        self._mpc = mpc

        # Need iteration over waypoints to adjust the target positions when needed
        self._waypoints: List[CartesianPoint] = []
        self._read_waypoints(waypoints_file_path)
        self._waypoints_iterator = 0

        # Mark initial target points to avoid going into (0, 0) point at first
        self._mpc.target_x, self._mpc.target_y = self._waypoints[self._waypoints_iterator]

        # Mark the reference trajectory for better visibility
        marker.mark(
            positions=self._waypoints,
            colour=MarkerColour(0, 1, 0),
            scale=0.05,
            duration=0,
        )

    def _read_waypoints(self, file_path: str):
        """
        Cast data points from the csv file to a collection of relevant named tuple instances.
        """
        with open(file_path) as waypoints_fd:
            reader = csv.reader(waypoints_fd, delimiter=",")
            for position_x, position_y in reader:
                self._waypoints.append(CartesianPoint(float(position_x.strip()), float(position_y.strip())))

    def _adjust_target_position(self, position_x: float, position_y: float):
        """
        Override the target position if the vehicle is "close enough" (threshold check passes).
        """
        distance = (self._mpc.target_x - position_x) ** 2 + (self._mpc.target_y - position_y) ** 2
        if distance < NEXT_WAYPOINT_THRESHOLD:
            self._waypoints_iterator = (self._waypoints_iterator + 1) % len(self._waypoints)
            self._mpc.target_x, self._mpc.target_y = self._waypoints[self._waypoints_iterator]

    def prepare_drive_command(self):
        """
        Each iteration the steering angle and the velocity must be computed.
        """
        # Retrieve the vehicle's state
        position_x, position_y = self._retrieve_position()
        heading_angle = self._retrieve_heading_angle()
        state = np.array([position_x, position_y, heading_angle])

        # Change target waypoints if needed
        self._adjust_target_position(position_x, position_y)

        # Mark where the vehicle is going
        marker.mark(
            positions=[(self._mpc.target_x, self._mpc.target_y)],
            channel=MarkerPublisherChannel.SECOND
        )

        # Compute inputs and visualise predicted trajectory
        velocity, steering_angle = self._mpc.make_step(state)
        marker.mark(
            positions=self._mpc.get_prediction_coordinates(),
            colour=MarkerColour(0, 1, 1),
            scale=0.12,
            channel=MarkerPublisherChannel.THIRD
        )

        # Finally, embed the inputs into the ackermann message
        self.steering_angle = steering_angle
        self.velocity = velocity
