"""
Follow the gap implementation.
"""
import numpy as np
from ..common import Racer, marker
from .constants import KP
from .constants import AVERAGE_SPEED, MAX_SPEED, MIN_SPEED, MAX_SPEED_SELECTION_THRESHOLD, MIN_SPEED_SELECTION_THRESHOLD


class ReferenceRacer(Racer):
    """
    Follow the gap racer drives by steering towards the largest gap in front of it.
    """

    def prepare_drive_command(self):
        """
        Follow-the-gap control (used as a reference algorithm when implementing the MPC ones).

        Each iteration the steering angle and the velocity must be computed. Additionally, predicted trajectory
        is plotted for more pleasing visuals and understanding of the steering angles.
        """
        angle = self._calculate_angle()
        speed = self._calculate_speed()
        marker.mark(
            positions=self.predict_trajectory(speed, angle)[0]
        )
        self.steering_angle = angle
        self.velocity = speed

    def _calculate_angle(self) -> float:
        """
        Calculate the target steering angle.

        TODO: Constants from this method should be interpreted and transferred to the constants.py file
        """
        angle_to_dist = 135 - self._find_angle() / 4

        pid_angle = - KP * angle_to_dist

        if (angle_to_dist > 40) or (angle_to_dist < -40):
            pid_angle = np.clip(pid_angle, -0.4, 0.4).item()
        else:
            pid_angle /= 100

        return pid_angle

    def _find_angle(self) -> float:
        """
        Find a new steering angle based on the lidar data.

        TODO: Constants from this method should be interpreted and transferred to the constants.py file
        """
        data = self._lidar_data
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

    def _calculate_speed(self) -> float:
        """
        Calculate target speed based on the current steering angle.
        """
        angle = abs(self.steering_angle)

        if angle <= MAX_SPEED_SELECTION_THRESHOLD:
            return MAX_SPEED

        if angle <= MIN_SPEED_SELECTION_THRESHOLD:
            return MIN_SPEED

        return AVERAGE_SPEED
