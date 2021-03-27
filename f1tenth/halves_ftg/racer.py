"""
Halves FTG with MPC.
"""
import math
from typing import List
from ..common import FollowTheGapRacer, marker, CartesianPoint, LidarPoint
from ..common.marker import MarkerColour, MarkerPublisherChannel, MarkerType
from ..common.constants import LIDAR_MINIMUM_ANGLE, LIDAR_ANGLE_INCREMENT
from .constants import MID_INDEX, LEFT_DIVERGENCE_INDEX, RIGHT_DIVERGENCE_INDEX
from .constants import FTG_DISTANCE_LIMIT, FTG_AREA_RADIUS_SQUARED, FTG_IGNORE_RANGE
from .constants import DEFAULT_RANGE, DEFAULT_RIGHT_TARGET_INDEX, DEFAULT_LEFT_TARGET_INDEX
from .constants import POSITION_PREDICTION_TIME


class HalvesRacer(FollowTheGapRacer):
    """
    The racer solves a farthest-point follow-the-gap algorithm twice, for each side of the car.

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

    @property
    def _position_prediction_time(self) -> float:
        """
        Overridden to allow position predictions.
        """
        return POSITION_PREDICTION_TIME

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

        # Build a list of relevant Point instances for each half
        left_points, right_points = list(), list()
        for i in range(LEFT_DIVERGENCE_INDEX - RIGHT_DIVERGENCE_INDEX + 1):
            cartesian_point = cartesian_points[i]
            lidar_index = i + RIGHT_DIVERGENCE_INDEX
            lidar_range = ranges[i]

            if lidar_index > MID_INDEX:
                left_points.append(LidarPoint(lidar_index, lidar_range, cartesian_point))
            else:
                right_points.append(LidarPoint(lidar_index, lidar_range, cartesian_point))

        # Avoid driving into points around the closest point
        self._mark_safety_radius(left_points)
        self._mark_safety_radius(right_points)

        # Visualise cartesian points cloud as a polygon
        visualisation_points = [point.cartesian for point in left_points + right_points
                                if point.range != FTG_IGNORE_RANGE]
        if len(visualisation_points) >= 3:
            marker.mark(
                positions=visualisation_points,
                colour=MarkerColour(0.4, 1, 1),
                scale=0.1,
                marker_type=MarkerType.LINE_STRIPS,
            )

        # Find the coordinates of FTG result for each half
        left_x, left_y = self._get_target_point(
            points=self._find_longest_sequence(left_points, FTG_IGNORE_RANGE),
            default_index=DEFAULT_LEFT_TARGET_INDEX,
            position_x=position_x,
            position_y=position_y,
            heading_angle=heading_angle
        )
        right_x, right_y = self._get_target_point(
            points=self._find_longest_sequence(right_points, FTG_IGNORE_RANGE),
            default_index=DEFAULT_RIGHT_TARGET_INDEX,
            position_x=position_x,
            position_y=position_y,
            heading_angle=heading_angle
        )

        # Visualise resulting coordinates and find the final drive-to-point
        marker.mark(
            positions=[(left_x, left_y), (right_x, right_y)],
            colour=MarkerColour(0, 0, 1),
            channel=MarkerPublisherChannel.SECOND,
            scale=0.3
        )
        self._mpc.target_x, self._mpc.target_y = (left_x + right_x) / 2, (left_y + right_y) / 2

    @staticmethod
    def _mark_safety_radius(points: List[LidarPoint]):
        """
        Mark too far points, the closest point, and all points 'next' to it as ignore distance (FTG algorithm).

        Closest point's distance will also be ignored because it's 0 measurement units away from itself.
        """
        closest_point = min(points, key=lambda p: p.range)
        for point in points:
            if point.range >= FTG_DISTANCE_LIMIT:
                point.range = FTG_IGNORE_RANGE
            elif (point.x - closest_point.x) ** 2 + (point.y - closest_point.y) ** 2 <= FTG_AREA_RADIUS_SQUARED:
                point.range = FTG_IGNORE_RANGE

    def _get_target_point(self, points: List[LidarPoint], **kwargs) -> CartesianPoint:
        """
        Compute the target reference point (either default point at a reasonable angle from the car, or farthest point).
        """
        default_index: int = kwargs.pop("default_index")
        position_x: float = kwargs.pop("position_x")
        position_y: float = kwargs.pop("position_y")
        heading_angle: float = kwargs.pop("heading_angle")

        if not points:
            target_range = DEFAULT_RANGE
            target_index = default_index
            laser_beam_angle = (target_index * LIDAR_ANGLE_INCREMENT) + LIDAR_MINIMUM_ANGLE
            rotated_angle = laser_beam_angle + heading_angle
            target_x = target_range * math.cos(rotated_angle) + position_x
            target_y = target_range * math.sin(rotated_angle) + position_y
            return CartesianPoint(target_x, target_y)

        # Points not empty so can return farthest point
        return max(points, key=lambda p: p.range).cartesian
