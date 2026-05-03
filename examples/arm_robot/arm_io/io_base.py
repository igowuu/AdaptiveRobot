from abc import ABC, abstractmethod

from wpimath.units import volts, radians, radians_per_second

from adaptive_robot import Faultable


class ArmIOBase(Faultable, ABC):
    """
    Holds the required methods for all arm IOs.
    """
    @abstractmethod
    def get_position(self) -> radians:
        """
        Returns the current arm position in radians.
        """
        ...

    @abstractmethod
    def get_velocity(self) -> radians_per_second:
        """
        Returns the current arm velocity in radians per second.
        """
        ...
    
    @abstractmethod
    def get_voltage(self) -> volts:
        """
        Returns the current applied voltage.
        """
        ...
    
    @abstractmethod
    def set_voltage(self, voltage: volts) -> None:
        """
        Directly sets a voltage to the arm mechanism.
        """
        ...

    @abstractmethod
    def update(self) -> None:
        """
        Updates the state of the IO if necessary.
        """
        ...
