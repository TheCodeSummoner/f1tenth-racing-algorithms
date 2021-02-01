"""
Waypoints-based trajectory MPC.
"""
import csv
from typing import Tuple
from collections import namedtuple
import casadi
import numpy as np
from .constants import HORIZON_LENGTH, TIME_STEP
from .constants import DISTANCE_IMPORTANCE, VELOCITY_IMPORTANCE
from .constants import WAYPOINTS_FILE_PATH, NEXT_WAYPOINT_THRESHOLD
from ..common import Racer, ModelPredictiveControl


class WaypointsFollowerMPC(ModelPredictiveControl):
    """
    Model predictive control for minimising distance from some precomputed reference point.

    An additional constraint on velocity is present to prefer higher speeds.
    """

    def __init__(self):
        super().__init__(
            horizon_length=HORIZON_LENGTH,
            time_step=TIME_STEP,
            plot_results=False
        )

        # Additional states will be needed to record target position data
        self._target_x = None
        self._target_y = None

        # Need to store actual target point values as well (so they can be retrieved in the cost function)
        self._tx = 0
        self._ty = 0

    @property
    def target_x(self) -> float:
        return self._tx

    @target_x.setter
    def target_x(self, value: float):
        self._tx = value

    @property
    def target_y(self) -> float:
        return self._ty

    @target_y.setter
    def target_y(self, value: float):
        self._ty = value

    def _prepare_target_position_template(self):
        """
        Following the docs of do_mpc, an approach to populate the target position variables with values, at any given
        point.
        """
        template = self._controller.get_tvp_template()

        for k in range(HORIZON_LENGTH + 1):
            template["_tvp", k, "target_x"] = self._tx
            template["_tvp", k, "target_y"] = self._ty

        return template

    @property
    def stage_cost(self):
        """
        No stage cost is specified in this approach.
        """
        return casadi.DM.zeros()

    @property
    def terminal_cost(self):
        """
        Terminal cost is the distance from the target joint with the difference from the target (max) speed.

        Both values are parametrised and may have different importance.
        """
        return DISTANCE_IMPORTANCE * ((self._target_x - self._position_x) ** 2 +
                                      (self._target_y - self._position_y) ** 2) \
            + VELOCITY_IMPORTANCE * ((self._constraints.max_velocity - self._velocity) ** 2)

    def configure_model(self):
        """
        Additionally to the base class variables, two time varying parameters must be specified, to allow changing
        the target position with time.
        """
        super().configure_model()

        # Create time-varying-parameters, these will be populated with (potentially) different data at each call
        self._target_x = self._model.set_variable(
            var_type="_tvp",
            var_name="target_x",
            shape=(1, 1)
        )
        self._target_y = self._model.set_variable(
            var_type="_tvp",
            var_name="target_y",
            shape=(1, 1)
        )

    def configure_controller(self):
        """
        Extension of the base class method simply considers setting the time varying parameters' update function.
        """
        super().configure_controller()

        # Setup time varying params - target position can change with time
        self._controller.set_tvp_fun(self._prepare_target_position_template)


class WaypointsFollowerRacer(Racer):
    """
    Waypoints racer drives by following some chosen point within its reference trajectory.
    """

    # Simple named tuple to better describe (x, y) tuples representing reference trajectory waypoints
    Waypoint = namedtuple("Waypoint", ["x", "y"])

    def __init__(self, waypoints_file_path: str = WAYPOINTS_FILE_PATH):
        super().__init__()
        self._mpc = WaypointsFollowerMPC()
        self._mpc.setup()

        # Need iteration over waypoints to adjust the target positions when needed
        self._waypoints: Tuple[WaypointsFollowerRacer.Waypoint, ...]
        self._read_waypoints(waypoints_file_path)
        self._waypoints_iterator = 0

    def _read_waypoints(self, file_path: str):
        """
        Cast data points from the csv file to a collection of relevant named tuple instances.
        """
        waypoints = []
        with open(file_path) as waypoints_fd:
            reader = csv.reader(waypoints_fd, delimeter=",")
            for x, y in reader:
                waypoints.append(self.Waypoint(float(x.strip()), float(y.strip())))

        self._waypoints = tuple(waypoints)

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
        Simple follow-the-gap control.

        Each iteration the steering angle and the velocity must be computed.
        """
        # Retrieve the vehicle's state
        position_x, position_y = self._retrieve_position()
        heading_angle = self._retrieve_heading_angle()
        state = np.array([position_x, position_y, heading_angle])

        # Change target waypoints if needed
        self._adjust_target_position(position_x, position_y)

        # Compute inputs and embed them into the ackermann message
        velocity, steering_angle = self._mpc.make_step(state)
        self._command.drive.steering_angle = steering_angle
        self._command.drive.speed = velocity
