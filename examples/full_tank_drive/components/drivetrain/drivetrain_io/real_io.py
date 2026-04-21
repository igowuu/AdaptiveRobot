from components.drivetrain.drivetrain_io.io_base import DrivetrainIOBase
from components.drivetrain.drivetrain_constants import DriveConstants, DriveCAN

from utils.sparkmax_faults import SparkMaxFaultLogger

from rev import SparkMax, SparkMaxConfig, ResetMode, PersistMode

from navx import AHRS

from wpilib import RobotController

from wpimath.units import volts, meters, meters_per_second
from wpimath.geometry import Rotation2d

from adaptive_robot.utils.math_utils import rotations_to_meters, rps_to_mps, clamp


class RealDrivetrainIO(DrivetrainIOBase):
    """
    IO that allows the robot to run with real hardware.
    """
    def __init__(self) -> None:
        self.left_motors = (
            SparkMax(DriveCAN.FRONT_LEFT, SparkMax.MotorType.kBrushless), # Front left
            SparkMax(DriveCAN.BACK_LEFT, SparkMax.MotorType.kBrushless)   # Back left
        )
        self.right_motors = (
            SparkMax(DriveCAN.FRONT_RIGHT, SparkMax.MotorType.kBrushless), # Front right
            SparkMax(DriveCAN.BACK_RIGHT, SparkMax.MotorType.kBrushless)   # Back right
        )

        self.left_motors[0].configure(
            SparkMaxConfig().inverted(False)
            .voltageCompensation(DriveConstants.NOMINAL_VOLTAGE)
            .setIdleMode(SparkMaxConfig.IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )
        self.left_motors[1].configure(
            SparkMaxConfig().inverted(False).follow(DriveCAN.FRONT_LEFT)
            .voltageCompensation(DriveConstants.NOMINAL_VOLTAGE)
            .setIdleMode(SparkMaxConfig.IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )
        self.right_motors[0].configure(
            SparkMaxConfig().inverted(True)
            .voltageCompensation(DriveConstants.NOMINAL_VOLTAGE)
            .setIdleMode(SparkMaxConfig.IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )
        self.right_motors[1].configure(
            SparkMaxConfig().inverted(True).follow(DriveCAN.FRONT_RIGHT)
            .voltageCompensation(DriveConstants.NOMINAL_VOLTAGE)
            .setIdleMode(SparkMaxConfig.IdleMode.kBrake),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )

        self._left_encoders = tuple(motor.getEncoder() for motor in self.left_motors)
        self._right_encoders = tuple(motor.getEncoder() for motor in self.right_motors)

        self._gyro = AHRS.create_spi()
        self._gyro.reset()

        self._check_motor_faults()
    
    def _check_motor_faults(self) -> None:
        """
        Checks for all motor faults, raising them as FaultExceptions if caught.
        """
        sparkmax_logger = SparkMaxFaultLogger()

        for motor in self.left_motors + self.right_motors:
            if motor.hasActiveFault():
                sparkmax_logger.report_sparkmax_faults(motor, False, f"{motor.getDeviceId()}")
            if motor.hasStickyFault():
                sparkmax_logger.report_sparkmax_faults(motor, True, f"{motor.getDeviceId()}")
            if motor.hasActiveWarning():
                sparkmax_logger.report_sparkmax_warnings(motor, False, f"{motor.getDeviceId()}")
            if motor.hasStickyWarning():
                sparkmax_logger.report_sparkmax_faults(motor, True, f"{motor.getDeviceId()}")

    def _average(self, values: list[float]) -> float:
        """
        Returns the average of a list of floats.
        """
        return sum(values) / len(values)

    def get_left_velocity(self) -> meters_per_second:
        return rps_to_mps(
            motor_rps=self._average([encoder.getVelocity() for encoder in self._left_encoders]),
            wheel_diameter=DriveConstants.WHEEL_DIAMETER,
            gear_ratio=DriveConstants.GEAR_RATIO
        )

    def get_right_velocity(self) -> meters_per_second:
        return rps_to_mps(
            motor_rps=self._average([encoder.getVelocity() for encoder in self._right_encoders]),
            wheel_diameter=DriveConstants.WHEEL_DIAMETER,
            gear_ratio=DriveConstants.GEAR_RATIO
        )

    def get_left_distance(self) -> meters:
        return rotations_to_meters(
            motor_rotations=self._average([encoder.getPosition() for encoder in self._left_encoders]),
            wheel_diameter=DriveConstants.WHEEL_DIAMETER,
            gear_ratio=DriveConstants.GEAR_RATIO
        )

    def get_right_distance(self) -> meters:
        return rotations_to_meters(
            motor_rotations=self._average([encoder.getPosition() for encoder in self._right_encoders]),
            wheel_diameter=DriveConstants.WHEEL_DIAMETER,
            gear_ratio=DriveConstants.GEAR_RATIO
        )
    
    def get_left_voltage(self) -> volts:
        return self.left_motors[0].get() * RobotController.getBatteryVoltage()

    def get_right_voltage(self) -> volts:
        return self.right_motors[0].get() * RobotController.getBatteryVoltage()
    
    def get_angle(self) -> Rotation2d:
        return self._gyro.getRotation2d()

    def set_left_voltage(self, voltage: volts) -> None:
        self.left_motors[0].setVoltage(
            clamp(voltage, -DriveConstants.MAX_VOLTAGE, DriveConstants.MAX_VOLTAGE)
        )
    
    def set_right_voltage(self, voltage: volts) -> None:
        self.right_motors[0].setVoltage(
            clamp(voltage, -DriveConstants.MAX_VOLTAGE, DriveConstants.MAX_VOLTAGE)
        )

    def update(self) -> None:
        pass
