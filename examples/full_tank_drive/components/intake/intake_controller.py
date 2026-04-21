from wpilib import Joystick, RobotState

from components.intake.intake import Intake

from adaptive_robot import AdaptiveComponent, BasicPriority


class IntakeController(AdaptiveComponent):
    def __init__(self, intake: Intake, controller: Joystick) -> None:
        super().__init__()
        self.controller = controller

        self.intake = intake
    
    def _safe_defaults(self) -> None:
        """
        Requests safe values to the Intake.
        """
        self.intake.request_percent(0.0, BasicPriority.SAFETY, "safety")

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

        if self.controller.getRawButton(4):
            self.intake.request_percent(1.0, BasicPriority.TELEOP, "teleop")
        elif self.controller.getRawButton(5):
            self.intake.request_percent(-1.0, BasicPriority.TELEOP, "teleop")
        else:
            self.intake.request_percent(0.0, BasicPriority.TELEOP, "teleop")
