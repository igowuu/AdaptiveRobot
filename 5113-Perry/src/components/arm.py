from enum import Enum

from phoenix5 import TalonSRX, ControlMode

from wpilib import DutyCycleEncoder

from wpimath import applyDeadband
from wpimath.units import percent, radians

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent
from adaptive_robot.requests import AxisRequest, resolve_axis

from config.constants import JoystickConst, RobotConst, ArmConst
from utils.math_utils import clamp, rotations_to_radians


class ArmPriority(Enum):
    SAFETY = 3
    AUTO = 2
    TELEOP = 1


class Arm(AdaptiveComponent):
    """
    This class declares and controls the components of our bot's arm.
    """
    
    def __init__(self, robot: "AdaptiveRobot") -> None:
        super().__init__(robot)

        self.left_arm_motor = TalonSRX(21)
        self.right_arm_motor = TalonSRX(22)

        self.right_arm_motor.setInverted(True)

        self.right_arm_motor.follow(self.left_arm_motor)

        self.left_arm_motor.enableVoltageCompensation(True)
        self.left_arm_motor.configVoltageCompSaturation(RobotConst.MAX_VOLTAGE)

        self.right_arm_motor.enableVoltageCompensation(True)
        self.right_arm_motor.configVoltageCompSaturation(RobotConst.MAX_VOLTAGE)

        self.left_arm_encoder = DutyCycleEncoder(0)
        self.right_arm_encoder = DutyCycleEncoder(1)

        
        self.max_pct_output = self.tunable("Tunables/ArmPCTOUT", 0.8)

        self._move_requests: list[AxisRequest] = []

    def get_angle(self) -> radians:
        """Returns current arm angle in radians."""
        left_arm_position = self.left_arm_encoder.get()
        right_arm_position = self.right_arm_encoder.get()
        avg_arm_position = (left_arm_position + right_arm_position) / 2.0

        return rotations_to_radians(avg_arm_position)

    def stop(self) -> None:
        """Sets the arm motor voltage to zero."""
        self._move_requests.append(
            AxisRequest(0.0, ArmPriority.SAFETY.value, 0.2, "safety")
        )

    def move_arm(self, desired_pct_output: percent) -> None:
        """
        Move the arm with a desired pct output.
        
        Inputs are in the range [-1.0, 1.0].
        """
        current_angle = self.get_angle()

        desired_pct_output = applyDeadband(desired_pct_output, JoystickConst.DEADBAND)

        # Set voltage to zero if voltage will push the arm below its safety limit.
        if desired_pct_output < 0 and current_angle <= ArmConst.MIN_ANGLE:
            self.stop()
            return
        
        # Set voltage to zero if voltage will push the arm beyond its safety limit.
        if desired_pct_output > 0 and current_angle >= ArmConst.MAX_ANGLE:
            self.stop()
            return
        
        desired_pct_output = clamp(
            desired_pct_output, 
            -self.max_pct_output.value, 
            self.max_pct_output.value
        )

        self._move_requests.append(
            AxisRequest(desired_pct_output, ArmPriority.TELEOP.value, 0.2, "teleop")
        )
    
    def publish_telemetry(self) -> None:
        """
        Publishes all arm-specific data before execute() is run.
        """
        arm_angle = self.get_angle()

        self.publish_value("Arm/Angle (radians)", arm_angle)
        self.publish_value(
            "Arm/MinLimitReached",
            arm_angle <= ArmConst.MIN_ANGLE,
        )
        self.publish_value(
            "Arm/MaxLimitReached",
            arm_angle >= ArmConst.MAX_ANGLE,
        )

        self.publish_value(
            "Arm/MotorOutput [-1, 1]",
            self.left_arm_motor.getMotorOutputPercent(),
        )
        self.publish_value(
            "Arm/EncoderPos (rotations)",
            self.left_arm_encoder.get(),
        )

    def execute(self) -> None:
        pct_out = resolve_axis(self._move_requests)

        self.left_arm_motor.set(ControlMode.PercentOutput, pct_out)

        self._move_requests.clear()
