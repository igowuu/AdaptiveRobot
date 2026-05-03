from wpilib import Joystick, RobotState

from adaptive_robot import Schedulable, BasicPriority
from drivetrain import Drivetrain


class DrivetrainController(Schedulable):
    """
    Represents a drivetrain controlled via teleoperation.
    Inherits from Schedulable to access health and lifecycle methods.
    """
    def __init__(self, drivetrain: Drivetrain, controller: Joystick) -> None:
        self.drivetrain = drivetrain
        self.controller = controller
    
    def _safe_defaults(self) -> None:
        """
        Requests safe defaults to the Drivetrain.
        """
        self.drivetrain.request_linear_percent(0.0, BasicPriority.TELEOP, "teleop")
        self.drivetrain.request_angular_percent(0.0, BasicPriority.TELEOP, "teleop")

    def on_enabled(self) -> None:
        """
        Hook that will be called once when the robot transitions to an enabled state.
        """
        self._safe_defaults()
    
    def on_disabled(self) -> None:
        """
        Hook that will be called once when the robot transitions to a disabled state.
        """
        self._safe_defaults()
    
    def on_faulted_init(self) -> None:
        """
        Hook that will be called once when the Schedulable enters an unhealthy state.
        """
        self._safe_defaults()
    
    def on_faulted_periodic(self) -> None:
        """
        Hook that will be called each iteration when the Schedulable enters an unhealthy state.
        """
        self._safe_defaults()

    def execute(self) -> None:
        """
        Hook that will be called each iteration if the Schedulable is healthy.
        """
        if not RobotState.isTeleop():
            return

        linear_percent = self.controller.getRawAxis(0)
        angular_percent = self.controller.getRawAxis(1)

        self.drivetrain.request_linear_percent(linear_percent, BasicPriority.TELEOP, "teleop")
        self.drivetrain.request_angular_percent(angular_percent, BasicPriority.TELEOP, "teleop")
