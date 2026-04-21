from abc import ABC, abstractmethod

from wpimath.units import volts

from adaptive_robot import Faultable


class IntakeIOBase(Faultable, ABC):
    """
    Holds the required methods for all intake IOs.
    """
    @abstractmethod
    def get_voltage(self) -> volts:
        """
        Returns the applied voltage to the intake.
        """
        ...

    @abstractmethod
    def set_voltage(self, voltage: volts) -> None:
        """
        Directly sets a voltage to the intake mechanism.
        """
        ...

    @abstractmethod
    def update(self) -> None:
        """
        Updates the state of the IO if necessary.
        """
        ...
