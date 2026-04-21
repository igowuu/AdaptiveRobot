from wpimath.geometry import Rotation2d
from wpimath.units import volts, meters, meters_per_second
from wpimath.system.plant import DCMotor

from components.drivetrain.drivetrain_io.io_base import DrivetrainIOBase
from components.drivetrain.drivetrain_constants import DriveConstants

from wpilib import Timer
from wpilib.simulation import DifferentialDrivetrainSim

from adaptive_robot.utils.math_utils import clamp


class SimulatedDrivetrainIO(DrivetrainIOBase):
    """
    IO that allows the robot to run without real hardware.
    """
    def __init__(self) -> None:
        self._diff_drive_sim = DifferentialDrivetrainSim(
            driveMotor=DCMotor.NEO(2),
            gearing=DriveConstants.GEAR_RATIO,
            J=DriveConstants.MOI,
            mass=DriveConstants.ROBOT_MASS,
            wheelRadius=DriveConstants.WHEEL_DIAMETER / 2,
            trackWidth=DriveConstants.TRACK_WIDTH
        )

        self._desired_left_voltage = 0.0
        self._desired_right_voltage = 0.0

        self._previous_timestamp = Timer.getFPGATimestamp()
    
    def get_left_distance(self) -> meters:
        return self._diff_drive_sim.getLeftPosition()

    def get_right_distance(self) -> meters:
        return self._diff_drive_sim.getRightPosition()

    def get_left_velocity(self) -> meters_per_second:
        return self._diff_drive_sim.getLeftVelocity()

    def get_right_velocity(self) -> meters_per_second:
        return self._diff_drive_sim.getRightVelocity()

    def get_left_voltage(self) -> volts:
        return self._desired_left_voltage

    def get_right_voltage(self) -> volts:
        return self._desired_right_voltage

    def get_angle(self) -> Rotation2d:
        return self._diff_drive_sim.getHeading()
    
    def set_left_voltage(self, voltage: volts) -> None:
        self._desired_left_voltage = voltage
    
    def set_right_voltage(self, voltage: volts) -> None:
        self._desired_right_voltage = voltage
    
    def update(self) -> None:
        self._diff_drive_sim.setInputs(
            clamp(self._desired_left_voltage, -DriveConstants.MAX_VOLTAGE, DriveConstants.MAX_VOLTAGE), 
            clamp(self._desired_right_voltage, -DriveConstants.MAX_VOLTAGE, DriveConstants.MAX_VOLTAGE)
        )

        new_timestamp = Timer.getFPGATimestamp()
        self._diff_drive_sim.update(new_timestamp - self._previous_timestamp)
        self._previous_timestamp = new_timestamp
