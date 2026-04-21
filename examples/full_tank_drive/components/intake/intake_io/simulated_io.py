from wpimath.units import volts

from components.intake.intake_io.io_base import IntakeIOBase
from components.intake.intake_constants import IntakeConstants

from adaptive_robot.utils.math_utils import clamp


class SimulatedIntakeIO(IntakeIOBase):
    """
    IO that allows the robot to run without real hardware.

    Fully simulating Intake is not necessarily useful, nor is it supported with wpilib,
    so a voltage field is used for simplicity (rather than a physics simulation).
    """
    def __init__(self) -> None:
        self._voltage = 0.0
    
    def get_voltage(self) -> volts:
        return self._voltage

    def set_voltage(self, voltage: volts) -> None:
        self._voltage = clamp(voltage, -IntakeConstants.MAX_VOLTAGE, IntakeConstants.MAX_VOLTAGE)

    def update(self) -> None:
        pass
