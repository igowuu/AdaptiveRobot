from wpilib import Joystick, RobotState

from components.drivetrain.drivetrain import Drivetrain
from components.drivetrain.drivetrain_constants import DriveConstants

from adaptive_robot import Schedulable, BasicPriority

from constants import JoystickAxis


class DrivetrainController(Schedulable):
    """
    Controls the Drivetrain via teleoperation.
    """
    def __init__(self, drivetrain: Drivetrain, controller: Joystick) -> None:
        self.drivetrain = drivetrain
        self.controller = controller

    def _safe_defaults(self) -> None:
        """
        Requests safe values to the Drivetrain.
        """
        self.drivetrain.request_linear_velocity(0.0, BasicPriority.SAFETY, "safety")
        self.drivetrain.request_angular_velocity(0.0, BasicPriority.SAFETY, "safety")

    def on_enabled(self) -> None:
        """
        Runs once when the robot enters an enabled mode.
        """
        self._safe_defaults()

    def on_disabled(self) -> None:
        """
        Runs once when the robot enters a disabled mode.
        """
        self._safe_defaults()
    
    def on_faulted_init(self) -> None:
        """
        Runs once when the Schedulable first becomes unhealthy.
        """
        self._safe_defaults()
    
    def on_faulted_periodic(self) -> None:
        """
        Runs each iteration when the Schedulable is unhealthy.
        """
        self._safe_defaults()

    def execute(self) -> None:
        """
        Method called each iteration if the Schedulable is healthy.
        """
        if not RobotState.isTeleop():
            return

        linear_percent = self.controller.getRawAxis(JoystickAxis.DRIVE_LINEAR)
        angular_percent = self.controller.getRawAxis(JoystickAxis.DRIVE_ANGULAR)

        linear_velocity = linear_percent * DriveConstants.MAX_LINEAR_SPEED
        angular_velocity = angular_percent * DriveConstants.MAX_ANGULAR_SPEED

        self.drivetrain.request_linear_velocity(linear_velocity, BasicPriority.TELEOP, "teleop")
        self.drivetrain.request_angular_velocity(angular_velocity, BasicPriority.TELEOP, "teleop")
