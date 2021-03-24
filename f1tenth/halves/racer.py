"""
Modified Follow The Gap with MPC.
"""
import math
from typing import List
import dataclasses
import numpy as np
from ..common import Racer, marker, PointFollowerMPC, CartesianPoint
from ..common.marker import MarkerColour, MarkerArrayPublisherChannel, MarkerPublisherChannel
from .constants import MID_INDEX, LEFT_DIVERGENCE_INDEX, RIGHT_DIVERGENCE_INDEX
from .constants import FTG_DISTANCE_LIMIT, FTG_AREA_RADIUS_SQUARED, FTG_IGNORE_VALUE
from .constants import DEFAULT_RANGE, DEFAULT_RIGHT_TARGET_INDEX, DEFAULT_LEFT_TARGET_INDEX
from .constants import LIDAR_MINIMUM_ANGLE, LIDAR_ANGLE_INCREMENT, POSITION_PREDICTION_TIME


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
        self._velocity = 0
        self._steering_angle = 0

    def _adjust_target_position(self, position_x: float, position_y: float, heading_angle: float):
        """
        Given current position and heading angle, find the target reference point.
        """
        lidar_data = self._lidar_data
        ranges = lidar_data.ranges[RIGHT_DIVERGENCE_INDEX:(LEFT_DIVERGENCE_INDEX+1)]
        cartesian_points = self.lidar_to_cartesian(
            ranges=ranges,
            position_x=position_x,
            position_y=position_y,
            heading_angle=heading_angle,
            starting_index=RIGHT_DIVERGENCE_INDEX
        )

        # Visualise cartesian points cloud as a polygon
        # TODO: Measure time impact
        marker.mark_line_strips(
            positions=cartesian_points,
            channel=MarkerPublisherChannel.FOURTH,
            colour=MarkerColour(0.4, 1, 1),
            scale=0.1
        )

        # Build a list of relevant Point instances for each half
        left_points, right_points = list(), list()
        for i in range(LEFT_DIVERGENCE_INDEX - RIGHT_DIVERGENCE_INDEX + 1):
            cartesian_point = cartesian_points[i]
            lidar_index = i + RIGHT_DIVERGENCE_INDEX
            lidar_range = ranges[i]

            # Need to adjust the distances early to accommodate for computing safety radii later on
            if lidar_range >= FTG_DISTANCE_LIMIT:
                lidar_range = FTG_DISTANCE_LIMIT_REPLACEMENT

            if lidar_index > MID_INDEX:
                left_points.append(self.Point(lidar_index, lidar_range, cartesian_point.x, cartesian_point.y))
            else:
                right_points.append(self.Point(lidar_index, lidar_range, cartesian_point.x, cartesian_point.y))

        # Avoid driving into points around the closest point
        self._mark_safety_radius(left_points)
        self._mark_safety_radius(right_points)

        # Visualise cartesian points cloud as a polygon
        visualisation_points = [CartesianPoint(point.cloud_point_x, point.cloud_point_y) for point in left_points + right_points if point.range != 0]
        if len(visualisation_points) >= 3:
            marker.mark_line_strips(
                positions=visualisation_points,
                channel=MarkerPublisherChannel.FOURTH,
                colour=MarkerColour(0.4, 1, 1),
                scale=0.1
            )

        # Find the coordinates of FTG result for each half
        left_x, left_y = self._get_target_point(
            points=self._find_longest_non_zero_sequence(left_points),
            default_index=DEFAULT_LEFT_TARGET_INDEX,
            position_x=position_x,
            position_y=position_y,
            heading_angle=heading_angle
        )
        right_x, right_y = self._get_target_point(
            points=self._find_longest_non_zero_sequence(right_points),
            default_index=DEFAULT_RIGHT_TARGET_INDEX,
            position_x=position_x,
            position_y=position_y,
            heading_angle=heading_angle
        )

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
            if (point.cloud_point_x - closest_point.cloud_point_x) ** 2 \
                    + (point.cloud_point_y - closest_point.cloud_point_y) ** 2 \
                    <= FTG_AREA_RADIUS_SQUARED:
                point.range = FTG_IGNORE_VALUE

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

        for i, point in enumerate(points):

            # Lengthen the sub-sequence or start a new one if non-zero number found
            if point.range != 0:
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
    def _get_target_point(points: List[Point], default_index: int,
                          position_x: float, position_y: float, heading_angle: float):
        """
        Compute the target reference point.
        """
        # Find target point by selecting either the default values, or the farthest point in current sample
        if not points:
            target_range = DEFAULT_RANGE
            target_index = default_index
            laser_beam_angle = (target_index * LIDAR_ANGLE_INCREMENT) + LIDAR_MINIMUM_ANGLE
            rotated_angle = laser_beam_angle + heading_angle
            target_x = target_range * math.cos(rotated_angle) + position_x
            target_y = target_range * math.sin(rotated_angle) + position_y
        else:
            farthest_point = max(points, key=lambda p: p.range)
            target_x = farthest_point.cloud_point_x
            target_y = farthest_point.cloud_point_y

        return target_x, target_y

    def prepare_drive_command(self):
        """
        Follow The Gap with MPC based on running FTG for left and right hand side of the vehicle separately.

        Each iteration the steering angle and the velocity must be computed.
        """
        position_x, position_y = self._retrieve_position()
        heading_angle = self._retrieve_heading_angle()

        # Change target waypoints if needed
        self._adjust_target_position(position_x, position_y, heading_angle)

        # Mark where the vehicle is going
        marker.mark(self._mpc.target_x, self._mpc.target_y)

        # Predict the car's position in which it's likely to be after the computations are done
        positions, heading_angles = self.predict_trajectory(
            velocity=self._velocity,
            steering_angle=self._steering_angle,
            steps_count=1,
            time_step=POSITION_PREDICTION_TIME,
            position_x=position_x,
            position_y=position_y,
            heading_angle=heading_angle
        )
        position_x, position_y = positions[0]
        heading_angle = heading_angles[0]

        # Define the state used for MPC
        state = np.array([position_x, position_y, heading_angle])

        # Compute inputs and visualise predicted trajectory
        self._velocity, self._steering_angle = self._mpc.make_step(state)
        marker.mark_array(
            self._mpc.get_prediction_coordinates(),
            colour=MarkerColour(0, 1, 1),
            scale=0.12,
            channel=MarkerArrayPublisherChannel.SECOND
        )

        # Finally, embed the inputs into the ackermann message
        self._command.drive.steering_angle = self._steering_angle
        self._command.drive.speed = self._velocity
