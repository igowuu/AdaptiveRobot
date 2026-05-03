from abc import ABC, abstractmethod

from wpimath.units import volts

from adaptive_robot import Faultable


class ShooterIOBase(Faultable, ABC):
    """
    Holds the required methods for all shooter IOs.
    """
    @abstractmethod
    def get_voltage(self) -> volts:
        """
        Returns the applied voltage to the shooter.
        """
        ...

    @abstractmethod
    def set_voltage(self, voltage: volts) -> None:
        """
        Directly sets a voltage to the shooter mechanism.
        """
        ...

    @abstractmethod
    def update(self) -> None:
        """
        Updates the state of the IO if necessary.
        """
        ...
