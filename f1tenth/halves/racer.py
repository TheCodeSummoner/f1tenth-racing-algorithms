"""
Modified Follow The Gap with MPC.
"""
import math
from typing import Tuple, List
import dataclasses
import numpy as np
from laser_geometry.laser_geometry import LaserProjection
from sensor_msgs import point_cloud2
from .constants import HORIZON_LENGTH, TIME_STEP
from ..common import Racer, marker, PointFollowerMPC
from ..common.marker import MarkerColour, MarkerArrayPublisherChannel, MarkerPublisherChannel
from .constants import MAX_INDEX, MID_INDEX, LEFT_DIVERGENCE_INDEX, RIGHT_DIVERGENCE_INDEX
from .constants import FTG_DISTANCE_LIMIT, FTG_AREA_RADIUS_SQUARED
from .constants import DEFAULT_RANGE, DEFAULT_RIGHT_TARGET_INDEX, DEFAULT_LEFT_TARGET_INDEX
from .constants import LIDAR_MINIMUM_ANGLE, LIDAR_ANGLE_INCREMENT


class HalvesRacer(Racer):
    """
    The racer solves a follow-the-gap algorithm twice, for each side of the car.

    Consider the following ascii graphic:

    |     |     |
    |     ^     |
    |     |     |
    | L [car] R |
    |     |     |
    |     |     |

    As you can (maybe) see, the area is divided into two halves - the Left half, and the Right half (the car is assumed
    to be going north). In this case, Follow The Gap algorithm will run for each half separately, which will result in
    a total of two points. Then, the resulting points are combined into a single point, which is used as a target point
    to be followed.
    """

    # Describe a lidar / cloud point data together
    Point = dataclasses.make_dataclass("Point", [("index", int), ("range", float),
                                                 ("cloud_point_x", float), ("cloud_point_y", float)])

    def __init__(self, mpc: PointFollowerMPC):
        super().__init__()
        self._mpc = mpc

    def _adjust_target_position(self, position_x: float, position_y: float, heading_angle: float):
        """
        Given current position and heading angle, find the target reference point.
        """
        lidar_data = self._lidar_data
        ranges = lidar_data.ranges

        # Cloud points will not be placed correctly with respect to the car's position and heading, but we only care
        # about the distances between each pair of points, and these will be true even without positional corrections
        laser_projection = LaserProjection().projectLaser(lidar_data)
        cloud_points = list(point_cloud2.read_points(laser_projection, skip_nans=True, field_names=("x", "y")))

        # Build a list of relevant Point instances for each half
        left_points = list()
        right_points = list()
        for i in range(MAX_INDEX):
            if LEFT_DIVERGENCE_INDEX >= i >= RIGHT_DIVERGENCE_INDEX:
                cloud_point = cloud_points[i]
                if i > MID_INDEX:
                    left_points.append(self.Point(i, ranges[i], cloud_point[0], cloud_point[1]))
                else:
                    right_points.append(self.Point(i, ranges[i], cloud_point[0], cloud_point[1]))

        # Avoid driving into points around the closest point
        self._mark_safety_radius(left_points)
        self._mark_safety_radius(right_points)

        # Find the longest non-zero sequences in each half
        left_point_space = self._find_longest_non_zero_sequence(left_points)
        right_point_space = self._find_longest_non_zero_sequence(right_points)

        # Fix the range and the index if either of the spaces is empty (no valid drive points detected)
        left_target_index, left_target_range = self._get_target_index_and_range(left_point_space,
                                                                                DEFAULT_LEFT_TARGET_INDEX)
        right_target_index, right_target_range = self._get_target_index_and_range(right_point_space,
                                                                                  DEFAULT_RIGHT_TARGET_INDEX)

        # Find the coordinates of FTG result for each half
        left_x, left_y = self._get_target_point(left_target_index, left_target_range,
                                                position_x, position_y, heading_angle)
        right_x, right_y = self._get_target_point(right_target_index, right_target_range,
                                                  position_x, position_y, heading_angle)

        # Visualise resulting coordinates and find the final drive-to-point
        marker.mark(left_x, left_y, colour=MarkerColour(0, 1, 0), channel=MarkerPublisherChannel.SECOND)
        marker.mark(right_x, right_y, colour=MarkerColour(0, 0, 1), channel=MarkerPublisherChannel.THIRD)
        self._mpc.target_x, self._mpc.target_y = (left_x + right_x) / 2, (left_y + right_y) / 2

    @staticmethod
    def _mark_safety_radius(points: List[Point]):
        """
        Mark too far points, the closest point, and all points 'next' to it as distance 0 (FTG algorithm).

        Closest point's distance will also be marked as 0 because it's 0 measurement units away from itself.
        """
        closest_point = min(points, key=lambda p: p.range)
        for point in points:
            if point.range >= FTG_DISTANCE_LIMIT:
                point.range = 0
            elif (point.cloud_point_x - closest_point.cloud_point_x) ** 2 \
                    + (point.cloud_point_y - closest_point.cloud_point_y) ** 2 \
                    <= FTG_AREA_RADIUS_SQUARED:
                point.range = 0

    @staticmethod
    def _find_longest_non_zero_sequence(points: List[Point]) -> List[Point]:
        """
        Very "manual" approach to finding such a sequence.

        Rather than using complicated Python mechanisms (zipping, filtering, etc.), a more verbose algorithm is used for
        the benefit of understanding the methodology of this method.

        When encountering a non-zero number the sub-sequence is either lengthened if it already exists, or a new one
        is created, whereas encountering a zero ends the current sub-sequence.
        """
        current_left_index = 0
        current_right_index = 0
        current_longest_sequence = 0
        final_left_index = 0
        final_right_index = 0
        is_sequence_started = False

        for i, p in enumerate(points):

            # Lengthen the sub-sequence or start a new one if non-zero number found
            if p != 0:
                if is_sequence_started:
                    current_right_index += 1
                else:
                    is_sequence_started = True
                    current_left_index = i
                    current_right_index = i

            # End current sub-sequence and see if it was any bigger than the previous one
            else:
                if is_sequence_started:
                    length = current_right_index - current_left_index
                    if length > current_longest_sequence:
                        final_left_index = current_left_index
                        final_right_index = current_right_index
                        current_longest_sequence = length
                    is_sequence_started = False

        # Finally, make sure that if the last point was also within the sub-sequence, the sub-sequence is also counted
        if is_sequence_started:
            length = current_right_index - current_left_index
            if length > current_longest_sequence:
                final_left_index = current_left_index
                final_right_index = current_right_index

        return points[final_left_index:final_right_index]

    @staticmethod
    def _get_target_point(target_index: int, target_range: float,
                          position_x: float, position_y: float, heading_angle: float):
        """
        Compute the target point given the current car's state, and the pre-fetched lidar index and range.
        """
        laser_beam_angle = (target_index * LIDAR_ANGLE_INCREMENT) + LIDAR_MINIMUM_ANGLE
        rotated_angle = laser_beam_angle + heading_angle
        target_x = target_range * math.cos(rotated_angle) + position_x
        target_y = target_range * math.sin(rotated_angle) + position_y
        return target_x, target_y

    @staticmethod
    def _get_target_index_and_range(points: List[Point], default_index: int) -> Tuple[int, float]:
        """
        Find target point by selecting either the default values, or the farthest point in current sample.

        Lidar index and range are returned to pass them to the next method.
        """
        if not points:
            target_range = DEFAULT_RANGE
            target_index = default_index
        else:
            farthest_point = max(points, key=lambda p: p.range)
            target_range = farthest_point.range
            target_index = farthest_point.index

        return target_index, target_range

    def prepare_drive_command(self):
        """
        Modified Follow The Gap with MPC.

        Each iteration the steering angle and the velocity must be computed.
        """
        # Retrieve the vehicle's state
        position_x, position_y = self._retrieve_position()
        heading_angle = self._retrieve_heading_angle()
        state = np.array([position_x, position_y, heading_angle])

        # Change target waypoints if needed
        self._adjust_target_position(position_x, position_y, heading_angle)

        # Mark where the vehicle is going
        marker.mark(self._mpc.target_x, self._mpc.target_y)

        # Compute inputs and visualise predicted trajectory
        velocity, steering_angle = self._mpc.make_step(state)
        marker.mark_array(
            self.predict_trajectory(velocity, steering_angle, steps_count=HORIZON_LENGTH, time_step=TIME_STEP),
            colour=MarkerColour(0, 1, 1),
            scale=0.12,
            channel=MarkerArrayPublisherChannel.SECOND
        )

        # Finally, embed the inputs into the ackermann message
        self._command.drive.steering_angle = steering_angle
        self._command.drive.speed = velocity
