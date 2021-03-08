"""
Model Predictive Control base class module.

Relevant algorithms should derive from the base class defined below.
"""
from typing import Optional
from abc import ABC, abstractmethod
from dataclasses import dataclass
import matplotlib
import matplotlib.pyplot as plt
import casadi
from do_mpc.model import Model
from do_mpc.controller import MPC
from do_mpc.graphics import Graphics
import numpy as np
from .constants import LR, WHEELBASE_LENGTH


@dataclass
class ControlConstraints:
    """
    Parametrisable collection of control constraints.
    """
    min_velocity: float = 0
    max_velocity: float = 7
    min_steering_angle: float = -0.4189
    max_steering_angle: float = 0.4189


class ModelPredictiveControl(ABC):
    """
    Abstract class allowing performing MPC-based approach to steering the vehicle.

    Configuration and abstract methods can (or should) be overridden. All other methods are recommended to be left
    untouched.

    The stage (lagrange term) and terminal (meyer term) costs must be specified by implementing the abstract methods.

    To ensure the `ModelPredictiveControl` instance can function properly simply call:

        mpc.setup()

    where `mpc` is an instance of the deriving class. This operation may take a while, as the internal `do_mpc`
    configuration must be performed.

    By default all results will be plotted to a graph, and no command-line output will be allowed.

    Each instance of this class will consists of three key elements:

        - model
        - controller
        - plotter

    Most generally, model describes the physical world, controller uses the model to obtain an optimal solution to some
    problem, and the plotter prepares a graphical representation of the algorithm's results.

    F1/10 MPC consists of 3 state variables (position and the heading angle) and two inputs (velocity and the steering
    angle) which are used within the above elements. Extensions are possible by rewriting most of the base class' code.
    """

    def __init__(self, horizon_length: int = 5, time_step: float = 0.1, suppress_outputs: bool = True,
                 plot_results: bool = True):
        self._horizon_length = horizon_length
        self._time_step = time_step
        self._suppress_outputs = suppress_outputs
        self._plot_results = plot_results

        # Model, the controller, and plotting must be prepared via the `setup` method call
        self._model: Optional[Model] = None
        self._controller: Optional[MPC] = None
        self._plotter: Optional[Graphics] = None
        self._ready: bool = False

        # States - x, y, Ψ
        self._state = np.array([0, 0, 0])
        self._position_x = None
        self._position_y = None
        self._heading_angle = None

        # Inputs - v, 𝛿
        self._inputs = np.array([0, 0])
        self._velocity = None
        self._steering_angle = None

        # Configurable constraints
        self._constraints = ControlConstraints()

    @property
    @abstractmethod
    def stage_cost(self):
        """
        This is the cost that will get added on each iteration.

        Also known as the Lagrange term.
        """

    @property
    @abstractmethod
    def terminal_cost(self):
        """
        This is the cost that will get added once, at the end of the iterations.

        Also known as the Meyer term.
        """

    def setup(self):
        """
        Configure the model, the controller, and the graphics-related setup.

        The MPC class is ready to perform operations once this method is called.
        """
        self.configure_model()
        self._model.setup()

        # Configuring the controller additionally requires setting the initial guess (to help the optimiser)
        self.configure_controller()
        self._controller.setup()
        self._controller.x0 = self._state
        self._controller.set_initial_guess()

        # Only configure graphics if result plotting is turned on
        if self._plot_results:
            self.configure_graphics()

        self._ready = True

    def configure_model(self):
        """
        Create state and input variables, as well as describe the vehicle's physical properties by using approximative
        motion's equations.
        """
        self._model = Model("continuous")

        # Create state and input variables
        self._position_x = self._model.set_variable(
            var_type="_x",
            var_name="position_x",
            shape=(1, 1)
        )
        self._position_y = self._model.set_variable(
            var_type="_x",
            var_name="position_y",
            shape=(1, 1)
        )
        self._heading_angle = self._model.set_variable(
            var_type="_x",
            var_name="heading_angle",
            shape=(1, 1)
        )
        self._velocity = self._model.set_variable(
            var_type="_u",
            var_name="velocity",
            shape=(1, 1)
        )
        self._steering_angle = self._model.set_variable(
            var_type="_u",
            var_name="steering_angle",
            shape=(1, 1)
        )

        # Next state equations - these mathematical formulas are the core of the model
        slip_factor = casadi.arctan(LR * casadi.tan(self._steering_angle) / WHEELBASE_LENGTH)
        self._model.set_rhs("position_x", self._velocity * casadi.cos(self._heading_angle + slip_factor))
        self._model.set_rhs("position_y", self._velocity * casadi.sin(self._heading_angle + slip_factor))
        self._model.set_rhs("heading_angle", self._velocity * casadi.tan(self._steering_angle)
                            * casadi.cos(slip_factor) / WHEELBASE_LENGTH)

    def configure_controller(self):
        """
        Set the model's parameters, the constraints, and the objective (cost) function.
        """
        self._controller = MPC(self._model)

        # Pass MPC and other parameters to the controller
        nlpsol_opts = {
            "ipopt.print_level": 0,
            "ipopt.sb": "yes",
            "print_time": 0
        } if self._suppress_outputs else {

        }
        self._controller.set_param(
            n_horizon=self._horizon_length,
            t_step=self._time_step,
            nlpsol_opts=nlpsol_opts
        )

        # lterm -> lagrange term (stage cost); mterm -> meyer term (terminal cost)
        self._controller.set_objective(lterm=self.stage_cost, mterm=self.terminal_cost)

        # Control bounds stored in the configurable dataclass instance
        self._controller.bounds["lower", "_u", "velocity"] = ControlConstraints.min_velocity
        self._controller.bounds["upper", "_u", "velocity"] = ControlConstraints.max_velocity
        self._controller.bounds["lower", "_u", "steering_angle"] = ControlConstraints.min_steering_angle
        self._controller.bounds["upper", "_u", "steering_angle"] = ControlConstraints.max_steering_angle

    def configure_graphics(self):
        """
        Setup matplotlib-based plotter and connect relevant data points to it.

        Additional styling is added for more pleasing visuals and can be extended for custom plotting.
        """
        self._plotter = Graphics(self._controller.data)

        # Add some nice styling
        matplotlib.rcParams["font.size"] = 18
        matplotlib.rcParams["lines.linewidth"] = 3
        matplotlib.rcParams["axes.grid"] = True

        # Create the figure and the axis
        fig, ax = plt.subplots(3, sharex="all", figsize=(16, 9))
        fig.align_ylabels()

        # Draw relevant state and inputs
        self._plotter.add_line(var_type="_x", var_name="position_x", axis=ax[0], color="green")
        self._plotter.add_line(var_type="_x", var_name="position_y", axis=ax[0], color="blue")
        self._plotter.add_line(var_type="_x", var_name="heading_angle", axis=ax[1], color="red")
        self._plotter.add_line(var_type="_u", var_name="steering_angle", axis=ax[1], color="green")
        self._plotter.add_line(var_type="_u", var_name="velocity", axis=ax[2], color="red")

        # Set X and Y labels
        ax[0].set_ylabel("Position")
        ax[1].set_ylabel("Angles")
        ax[2].set_ylabel("Velocity")
        ax[2].set_xlabel("Time")

    def make_step(self, state: np.array) -> np.array:
        """
        Derive next collection of inputs.

        The method call will fail if no call to the `setup` method has been made yet.
        """
        if not self._ready:
            raise RuntimeError("MPC has not been configured, make sure to call the \"setup\" method before attempting"
                               "to use the controller")

        return self._controller.make_step(state)

    def plot(self):
        """
        Plot the results onto the graph and display the figure.

        The method call will fail if no call to the `setup` method has been made yet, or the plotting has been disabled.
        """
        if not self._ready:
            raise RuntimeError("MPC has not been configured, make sure to call the \"setup\" method before attempting"
                               "to use the controller")

        if self._plotter is None:
            raise RuntimeError("Plotting has not been turned on upon instantiation of the class")

        self._plotter.plot_results()
        self._plotter.reset_axes()
        plt.show()


class PointFollowerMPC(ModelPredictiveControl):
    """
    Model predictive control for minimising distance from some precomputed reference point.

    An additional constraint on velocity is present to prefer higher speeds.
    """

    def __init__(self, horizon_length: int, time_step: float):
        super().__init__(
            horizon_length=horizon_length,
            time_step=time_step,
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

    def _prepare_target_position_template(self, _):
        """
        Following the docs of do_mpc, an approach to populate the target position variables with values, at any given
        point.
        """
        template = self._controller.get_tvp_template()

        for k in range(self._horizon_length + 1):
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
        return (self._target_x - self._position_x) ** 2 + (self._target_y - self._position_y) ** 2

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
