from adaptive_robot.utils.math_utils import rotations_to_radians, rps_to_radps, clamp

from wpilib import RobotController

from wpimath.units import volts, radians, radians_per_second

from arm_io.io_base import ArmIOBase
from arm_constants import ArmConstants, ArmCAN

from adaptive_robot.utils.talon_faults import TalonFaultLogger

from phoenix6.hardware import TalonFXS
from phoenix6.controls import Follower
from phoenix6.signals import MotorAlignmentValue


class RealArmIO(ArmIOBase):
    """
    IO that allows the robot to run with real hardware.
    """
    def __init__(self) -> None:
        self.left_motor = TalonFXS(ArmCAN.LEFT)
        self.right_motor = TalonFXS(ArmCAN.RIGHT)

        self.right_motor.set_control(Follower(ArmCAN.LEFT, MotorAlignmentValue.OPPOSED))

        self._check_motor_faults()

    def _check_motor_faults(self) -> None:
        """
        Checks for all motor faults and logs them.
        """
        talonfx_logger = TalonFaultLogger()

        for motor in (self.left_motor, self.right_motor):
            talonfx_logger.report_talon_faults(motor, False, f"{motor.device_id}")
            talonfx_logger.report_talon_faults(motor, True, f"{motor.device_id}")

    def _average(self, values: list[float]) -> float:
        """
        Returns the average of a list of floats.
        """
        return sum(values) / len(values)

    def get_position(self) -> radians:
        averaged_positions = self._average(
            list(motor.get_position().value for motor in (self.left_motor, self.right_motor))
        )
        return rotations_to_radians(averaged_positions)
    
    def get_velocity(self) -> radians_per_second:
        averaged_velocities = self._average(
            list(motor.get_velocity().value for motor in (self.left_motor, self.right_motor))
        )
        return rps_to_radps(averaged_velocities)

    def get_voltage(self) -> volts:
        return self.left_motor.get() * RobotController.getBatteryVoltage()

    def set_voltage(self, voltage: volts) -> None:
        self.left_motor.setVoltage(
            clamp(voltage, -ArmConstants.MAX_VOLTAGE, ArmConstants.MAX_VOLTAGE)
        )

    def update(self) -> None:
        pass
