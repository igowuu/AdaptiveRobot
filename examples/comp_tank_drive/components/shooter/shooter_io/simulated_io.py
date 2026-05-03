from wpilib.simulation import DCMotorSim

from wpimath.system.plant import LinearSystemId, DCMotor

from wpimath.units import volts

from components.shooter.shooter_io.io_base import ShooterIOBase
from components.shooter.shooter_constants import ShooterConstants

from adaptive_robot.utils.math_utils import clamp


class SimulatedShooterIO(ShooterIOBase):
    """
    IO that allows the robot to run without real hardware.
    """
    def __init__(self) -> None:
        self._motor_model = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                DCMotor.falcon500(1),
                0.001,
                ShooterConstants.GEAR_RATIO
            ),
            DCMotor.falcon500(1)
        )

    def get_voltage(self) -> volts:
        return self._motor_model.getInputVoltage()

    def set_voltage(self, voltage: volts) -> None:
        clamped = clamp(voltage, -ShooterConstants.MAX_VOLTAGE, ShooterConstants.MAX_VOLTAGE)
        self._motor_model.setInputVoltage(clamped)

    def update(self) -> None:
        self._motor_model.update(0.02)
