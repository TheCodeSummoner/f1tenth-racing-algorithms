"""
Farthest-point FTG with MPC.
"""
from typing import List
import dataclasses
import numpy as np
from ..common import Racer, marker, PointFollowerMPC, CartesianPoint
from ..common.marker import MarkerColour, MarkerPublisherChannel, MarkerType
from .constants import LEFT_DIVERGENCE_INDEX, RIGHT_DIVERGENCE_INDEX, FTG_IGNORE_RANGE
from .constants import SAFETY_RADIUS_SQUARED, POSITION_PREDICTION_TIME


class FarthestPointFollower(Racer):
    """
    Drive towards the farthest point within the largest gap.

    More specifically, by filtering the values using safety radius (only the values N measurement units away from the
    closest point to the vehicle are considered) a collection of gaps is retrieved. Then, the largest gap is picked,
    and the vehicle will drive towards the middle of it.

    This implementation considers a static (constant) safety radius.
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
            points.append(self.Point(lidar_index, lidar_range, cartesian_point.x, cartesian_point.y))

        # Avoid driving into points around the closest point
        self._mark_safety_radius(points)

        # Visualise cartesian points cloud as a polygon
        visualisation_points = [CartesianPoint(point.cloud_point_x, point.cloud_point_y)
                                for point in points if point.range != FTG_IGNORE_RANGE]
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
    def _mark_safety_radius(points: List[Point]):
        """
        Compute the target reference point (find the farthest point within the largest gap).
        """
        closest_point = min(points, key=lambda p: p.range)
        for point in points:
            if (point.cloud_point_x - closest_point.cloud_point_x) ** 2 \
                    + (point.cloud_point_y - closest_point.cloud_point_y) ** 2 \
                    <= SAFETY_RADIUS_SQUARED:
                point.range = FTG_IGNORE_RANGE

    @staticmethod
    def _find_longest_sequence(points: List[Point], ignore_range: float) -> List[Point]:
        """
        Very "manual" approach to finding such a sequence.

        Rather than using complicated Python mechanisms (zipping, filtering, etc.), a more verbose algorithm is used for
        the benefit of understanding the methodology of this method.

        When encountering a non-ignore range the sub-sequence is either lengthened if it already exists, or a new one
        is created, whereas encountering the ignore range ends the current sub-sequence.
        """
        current_left_index = 0
        current_right_index = 0
        current_longest_sequence = 0
        final_left_index = 0
        final_right_index = 0
        is_sequence_started = False

        for i, point in enumerate(points):

            # Lengthen the sub-sequence or start a new one if non-ignore number found
            if point.range != ignore_range:
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

    def _get_target_point(self, points: List[Point]) -> CartesianPoint:
        """
        Compute the target reference point.
        """
        if not points:
            return CartesianPoint(self._mpc.target_x, self._mpc.target_y)

        # Points not empty so can pick farthest point
        target = max(points, key=lambda point: point.range)

        return CartesianPoint(target.cloud_point_x, target.cloud_point_y)

    def prepare_drive_command(self):
        """
        Follow The Gap (specifically, follow the farthest of the largest gap) with MPC.
        """
        position_x, position_y = self._retrieve_position()
        heading_angle = self._retrieve_heading_angle()

        # Change target waypoints if needed
        self._adjust_target_position(position_x, position_y, heading_angle)

        # Mark where the vehicle is going
        marker.mark(
            positions=[(self._mpc.target_x, self._mpc.target_y)],
            channel=MarkerPublisherChannel.SECOND
        )

        # Predict the car's position in which it's likely to be after the computations are done
        positions, heading_angles = self.predict_trajectory(
            velocity=self.velocity,
            steering_angle=self.steering_angle,
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
        self.velocity, self.steering_angle = self._mpc.make_step(state)
        marker.mark(
            positions=self._mpc.get_prediction_coordinates(),
            colour=MarkerColour(0, 1, 1),
            scale=0.12,
            channel=MarkerPublisherChannel.THIRD
        )
