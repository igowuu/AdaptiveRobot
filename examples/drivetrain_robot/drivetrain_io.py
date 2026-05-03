from wpilib import Spark

from wpimath.units import percent, volts
from wpilib.drive import DifferentialDrive


class DrivetrainIO:
    """
    Represents a simple real IO that interacts directly with Spark hardware.
    """
    def __init__(self) -> None:
        self.left_motor = Spark(0)
        self.right_motor = Spark(1)

        self.diff_drive = DifferentialDrive(self.left_motor, self.right_motor)

    def get_left_voltage(self) -> volts:
        """
        Returns the commanded voltage to the left motor this iteration.
        """
        return self.left_motor.getVoltage()

    def get_right_voltage(self) -> volts:
        """
        Returns the commanded voltage to the right motor this iteration.
        """
        return self.right_motor.getVoltage()

    def drive(self, linear_percent: percent, angular_percent: percent) -> None:
        """
        Drives the robot by converting linear and angular percents into voltages for the motors.
        """
        self.diff_drive.arcadeDrive(linear_percent, angular_percent)
