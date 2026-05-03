from adaptive_robot import Schedulable, BasicPriority

from wpilib import Joystick, RobotState

from components.intake_arm.intake_arm import IntakeArm
from components.intake_arm.intake_arm_constants import IntakeArmConstants

from constants import JoystickButton


class IntakeArmController(Schedulable):
    """
    Controls the intake arm via teleoperation.
    """
    def __init__(self, intake_arm: IntakeArm, controller: Joystick) -> None:
        self.intake_arm = intake_arm
        self.controller = controller
        self.sticky_angle = 0.0

    def _safe_defaults(self) -> None:
        """
        Requests safe values to the IntakeArm.
        """
        self.intake_arm.request_angle(0.0, BasicPriority.SAFETY, "safety")

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

        if self.controller.getRawButton(JoystickButton.INTAKE_ARM_UP):
            self.sticky_angle = IntakeArmConstants.MAX_ANGLE
        elif self.controller.getRawButton(JoystickButton.INTAKE_ARM_DOWN):
            self.sticky_angle = IntakeArmConstants.MIN_ANGLE
        
        self.intake_arm.request_angle(self.sticky_angle, BasicPriority.TELEOP, "teleop")