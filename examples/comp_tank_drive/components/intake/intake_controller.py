from wpilib import Joystick, RobotState

from components.intake.intake import Intake

from adaptive_robot import Schedulable, BasicPriority

from constants import JoystickButton


class IntakeController(Schedulable):
    """
    Controls the intake via teleoperation.
    """
    def __init__(self, intake: Intake, controller: Joystick) -> None:
        self.intake = intake
        self.controller = controller

    def _safe_defaults(self) -> None:
        """
        Requests safe values to the Intake.
        """
        self.intake.request_percent(0.0, BasicPriority.SAFETY, "safety")

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

        if self.controller.getRawButton(JoystickButton.INTAKE_GRABBING):
            self.intake.request_percent(1.0, BasicPriority.TELEOP, "teleop")
        elif self.controller.getRawButton(JoystickButton.INTAKE_RELEASING):
            self.intake.request_percent(-1.0, BasicPriority.TELEOP, "teleop")
        else:
            self.intake.request_percent(0.0, BasicPriority.TELEOP, "teleop")
