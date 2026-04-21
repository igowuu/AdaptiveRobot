from adaptive_robot import AdaptiveComponent, BasicPriority

from wpilib import Joystick, RobotState

from components.arm.arm import Arm
from components.arm.arm_constants import ArmConstants


class ArmController(AdaptiveComponent):
    """
    Controls the arm via teleoperation.
    """
    def __init__(
        self,
        arm: Arm, 
        controller: Joystick
    ) -> None:
        super().__init__()

        self.arm = arm
        self.controller = controller

        self.sticky_angle = 0.0

    def _safe_defaults(self) -> None:
        """
        Requests safe values to the Arm.
        """
        self.arm.request_angle(0.0, BasicPriority.SAFETY, "safety")

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

        if self.controller.getRawButton(1):
            self.sticky_angle = ArmConstants.MAX_ANGLE
        elif self.controller.getRawButton(2):
            self.sticky_angle = ArmConstants.MAX_ANGLE / 2
        elif self.controller.getRawButton(3):
            self.sticky_angle = ArmConstants.MIN_ANGLE
        
        self.arm.request_angle(self.sticky_angle, BasicPriority.TELEOP, "teleop")