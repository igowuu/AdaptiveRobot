from components.drivetrain.drivetrain_io.io_base import DrivetrainIOBase
from components.drivetrain.drivetrain_constants import DriveConstants, DriveCAN

from navx import AHRS

from phoenix6.hardware import TalonFX
from phoenix6.controls import Follower
from phoenix6.signals import MotorAlignmentValue

from wpilib import RobotController

from wpimath.units import volts, meters, meters_per_second
from wpimath.geometry import Rotation2d

from adaptive_robot.utils.math_utils import clamp, rotations_to_meters, rps_to_mps
from adaptive_robot.utils.talon_faults import TalonFaultLogger


class RealDrivetrainIO(DrivetrainIOBase):
    """
    IO that allows the robot to run with real hardware.
    """
    def __init__(self) -> None:
        self.front_left = TalonFX(DriveCAN.FRONT_LEFT)
        self.front_right = TalonFX(DriveCAN.FRONT_RIGHT)
        self.back_left = TalonFX(DriveCAN.BACK_LEFT)
        self.back_right = TalonFX(DriveCAN.BACK_RIGHT)

        self.front_right.set_control(Follower(DriveCAN.FRONT_LEFT, MotorAlignmentValue.OPPOSED))
        self.back_right.set_control(Follower(DriveCAN.BACK_LEFT, MotorAlignmentValue.OPPOSED))

        self._gyro = AHRS.create_spi()
        self._gyro.reset()

        self._check_motor_faults()
    
    def _check_motor_faults(self) -> None:
        """
        Checks for all motor faults, raising them as FaultExceptions if caught.
        """
        talonfx_logger = TalonFaultLogger()

        for motor in (self.front_left, self.front_right, self.back_left, self.back_right):
            talonfx_logger.report_talon_faults(motor, False, f"{motor.device_id}")

    def _average(self, values: list[float]) -> float:
        """
        Returns the average of a list of floats.
        """
        return sum(values) / len(values)

    def get_left_velocity(self) -> meters_per_second:
        return rps_to_mps(
            motor_rps=self._average([motor.get_velocity().value for motor in (self.front_left, self.back_left)]),
            wheel_diameter=DriveConstants.WHEEL_DIAMETER,
            gear_ratio=DriveConstants.GEAR_RATIO
        )

    def get_right_velocity(self) -> meters_per_second:
        return rps_to_mps(
            motor_rps=self._average([motor.get_velocity().value for motor in (self.front_right, self.back_right)]),
            wheel_diameter=DriveConstants.WHEEL_DIAMETER,
            gear_ratio=DriveConstants.GEAR_RATIO
        )

    def get_left_distance(self) -> meters:
        return rotations_to_meters(
            motor_rotations=self._average([motor.get_position().value for motor in (self.front_left, self.back_left)]),
            wheel_diameter=DriveConstants.WHEEL_DIAMETER,
            gear_ratio=DriveConstants.GEAR_RATIO
        )

    def get_right_distance(self) -> meters:
        return rotations_to_meters(
            motor_rotations=self._average([motor.get_position().value for motor in (self.front_right, self.back_right)]),
            wheel_diameter=DriveConstants.WHEEL_DIAMETER,
            gear_ratio=DriveConstants.GEAR_RATIO
        )
    
    def get_left_voltage(self) -> volts:
        return self.front_left.get() * RobotController.getBatteryVoltage()

    def get_right_voltage(self) -> volts:
        return self.front_right.get() * RobotController.getBatteryVoltage()
    
    def get_angle(self) -> Rotation2d:
        return self._gyro.getRotation2d()

    def set_left_voltage(self, voltage: volts) -> None:
        self.front_left.setVoltage(
            clamp(voltage, -DriveConstants.MAX_VOLTAGE, DriveConstants.MAX_VOLTAGE)
        )
    
    def set_right_voltage(self, voltage: volts) -> None:
        self.front_right.setVoltage(
            clamp(voltage, -DriveConstants.MAX_VOLTAGE, DriveConstants.MAX_VOLTAGE)
        )

    def update(self) -> None:
        pass
