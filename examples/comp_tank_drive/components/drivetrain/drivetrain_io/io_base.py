from abc import ABC, abstractmethod

from wpimath.geometry import Rotation2d
from wpimath.units import volts, meters, meters_per_second

from adaptive_robot import Faultable


class DrivetrainIOBase(Faultable, ABC):
    """
    Holds the required methods for all drivetrain IOs.
    """
    @abstractmethod
    def get_left_velocity(self) -> meters_per_second:
        """
        Returns the current drivetrain left linear velocity in meters per second.
        """
        ...

    @abstractmethod
    def get_right_velocity(self) -> meters_per_second:
        """
        Returns the current drivetrain right linear velocity in meters per second.
        """
        ...

    @abstractmethod
    def get_left_distance(self) -> meters:
        """
        Returns the total drivetrain left linear distance traveled in meters.
        """
        ...

    @abstractmethod
    def get_right_distance(self) -> meters:
        """
        Returns the total drivetrain right linear distance traveled in meters.
        """
        ...
    
    @abstractmethod
    def get_left_voltage(self) -> volts:
        """
        Returns the left applied voltage.
        """
        ...
    
    @abstractmethod
    def get_right_voltage(self) -> volts:
        """
        Returns the right applied voltage.
        """
        ...

    @abstractmethod
    def get_angle(self) -> Rotation2d:
        """
        Returns the current estimated robot angle.
        """
        ...

    @abstractmethod
    def set_left_voltage(self, voltage: volts) -> None:
        """
        Commands a voltage to all left motors.
        """
        ...

    @abstractmethod
    def set_right_voltage(self, voltage: volts) -> None:
        """
        Commands a voltage to all right motors.
        """
        ...

    @abstractmethod
    def update(self) -> None:
        """
        Updates the state of the IO, if necessary.
        """
        ...
