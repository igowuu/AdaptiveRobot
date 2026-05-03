from phoenix6.hardware import TalonFX

from wpimath.units import volts


class SimpleIO:
    """
    Represents a simple real IO that interacts directly with Talon hardware.
    """
    def __init__(self) -> None:
        self.motor = TalonFX(0)
    
    def set_voltage(self, voltage: volts) -> None:
        self.motor.setVoltage(voltage)
