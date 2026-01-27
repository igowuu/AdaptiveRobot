from enum import Enum

from phoenix6.hardware import TalonFX
from phoenix6.configs import MotorOutputConfigs
from phoenix6.signals import NeutralModeValue

from wpimath import applyDeadband
from wpimath.units import percent

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent
from adaptive_robot.requests import AxisRequest, resolve_axis

from utils.math_utils import clamp
from config.constants import JoystickConst


class IntakePriority(Enum):
    SAFETY = 3
    AUTO = 2
    TELEOP = 1


class Intake(AdaptiveComponent):
    """
    This class declares the intake component to grab and release FRC items. The intake component
    is attatched to the arm and acts similarly to a conveyor belt.
    """

    def __init__(self, robot: "AdaptiveRobot") -> None:
        super().__init__(robot)

        self.intake_motor = TalonFX(23)

        self.motor_config = MotorOutputConfigs()
        self.motor_config.neutral_mode = NeutralModeValue.BRAKE

        self.intake_motor.configurator.apply(self.motor_config)

        self.max_pct_output = self.tunable("Tunables/IntakePCTOUT", 0.8)

        self._move_requests: list[AxisRequest] = []

    def stop(self) -> None:
        self._move_requests.append(
            AxisRequest(0.0, IntakePriority.SAFETY.value, 0.2, "safety")
        )

    def move_intake(self, desired_pct_output: percent) -> None:
        """
        Move the intake with a desired pct output.
        
        Inputs are in the range [-1.0, 1.0].
        """
        desired_pct_output = applyDeadband(desired_pct_output, JoystickConst.DEADBAND)

        desired_pct_output = clamp(
            desired_pct_output, 
            -self.max_pct_output.value, 
            self.max_pct_output.value
        )

        self._move_requests.append(
            AxisRequest(desired_pct_output, IntakePriority.TELEOP.value, 0.2, "teleop")
        )

    def execute(self) -> None:
        pct_out = resolve_axis(self._move_requests)
        self.intake_motor.set(pct_out)

        self._move_requests.clear()
