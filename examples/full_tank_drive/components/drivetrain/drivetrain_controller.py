from wpilib import Joystick, RobotState

from components.drivetrain.drivetrain import Drivetrain
from components.drivetrain.drivetrain_constants import DriveConstants

from adaptive_robot import AdaptiveComponent, BasicPriority


class DrivetrainController(AdaptiveComponent):
    """
    Controls the Drivetrain via teleoperation.
    """
    def __init__(
        self,
        drivetrain: Drivetrain, 
        controller: Joystick
    ) -> None:
        super().__init__()

        self.drivetrain = drivetrain
        self.controller = controller

    def _safe_defaults(self) -> None:
        """
        Requests safe values to the Drivetrain.
        """
        self.drivetrain.request_linear_velocity(0.0, BasicPriority.SAFETY, "safety")
        self.drivetrain.request_angular_velocity(0.0, BasicPriority.SAFETY, "safety")

    def on_faulted_init(self) -> None:
        """
        Method called once if the component becomes unhealthy.
        """
        self._safe_defaults()

    def on_faulted_periodic(self) -> None:
        """
        Method called each iteration once the component becomes unhealthy.
        """
        self._safe_defaults()

    def on_enabled(self) -> None:
        """
        Method called once when the robot enters a enabled state.
        """
        self._safe_defaults()

    def on_disabled(self) -> None:
        """
        Method called once when the robot enters a disabled state.
        """
        self._safe_defaults()

    def execute(self) -> None:
        """
        Method called each iteration if the component is healthy.
        """
        if not RobotState.isTeleop():
            return

        linear_percent = self.controller.getX()
        angular_percent = self.controller.getY()

        linear_velocity = linear_percent * DriveConstants.MAX_LINEAR_SPEED
        angular_velocity = angular_percent * DriveConstants.MAX_ANGULAR_SPEED

        self.drivetrain.request_linear_velocity(linear_velocity, BasicPriority.TELEOP, "teleop")
        self.drivetrain.request_angular_velocity(angular_velocity, BasicPriority.TELEOP, "teleop")
