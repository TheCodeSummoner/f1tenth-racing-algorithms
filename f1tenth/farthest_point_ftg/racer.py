"""
Farthest-point FTG with MPC.
"""
from typing import List
from ..common import FollowTheGapRacer, marker, LidarPoint, CartesianPoint
from ..common.marker import MarkerColour, MarkerType
from .constants import LEFT_DIVERGENCE_INDEX, RIGHT_DIVERGENCE_INDEX
from .constants import FTG_IGNORE_RANGE, SAFETY_RADIUS_SQUARED
from .constants import POSITION_PREDICTION_TIME


class FarthestPointFollower(FollowTheGapRacer):
    """
    Drive towards the farthest point within the largest gap.

    More specifically, by filtering the values using safety radius (only the values N measurement units away from the
    closest point to the vehicle are considered) a collection of gaps is retrieved. Then, the largest gap is picked,
    and the vehicle will drive towards the middle of it.

    This implementation considers a static (constant) safety radius.
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

        # Build a list of relevant Point instances
        points = list()
        for i in range(LEFT_DIVERGENCE_INDEX - RIGHT_DIVERGENCE_INDEX + 1):
            cartesian_point = cartesian_points[i]
            lidar_index = i + RIGHT_DIVERGENCE_INDEX
            lidar_range = ranges[i]
            points.append(LidarPoint(lidar_index, lidar_range, cartesian_point))

        # Avoid driving into points around the closest point
        self._mark_safety_radius(points)

        # Visualise cartesian points cloud as a polygon
        visualisation_points = [point.cartesian for point in points if point.range != FTG_IGNORE_RANGE]
        if len(visualisation_points) >= 3:
            marker.mark(
                positions=visualisation_points,
                colour=MarkerColour(0.4, 1, 1),
                scale=0.1,
                marker_type=MarkerType.LINE_STRIPS,
            )

        self._mpc.target_x, self._mpc.target_y = self._get_target_point(
            self._find_longest_sequence(points, FTG_IGNORE_RANGE)
        )

    @staticmethod
    def _mark_safety_radius(points: List[LidarPoint]):
        """
        Mark all values within the safety radius as ignored.
        """
        closest_point = min(points, key=lambda p: p.range)
        for point in points:
            if (point.x - closest_point.x) ** 2 + (point.y - closest_point.y) ** 2 <= SAFETY_RADIUS_SQUARED:
                point.range = FTG_IGNORE_RANGE

    def _get_target_point(self, points: List[LidarPoint], **_) -> CartesianPoint:
        """
        Compute the target reference point (the farthest point within the largest gap).
        """
        if not points:
            return CartesianPoint(self._mpc.target_x, self._mpc.target_y)

        # Points not empty so can pick farthest point
        return max(points, key=lambda point: point.range).cartesian
