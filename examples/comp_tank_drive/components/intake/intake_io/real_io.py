from wpimath.units import volts

from components.intake.intake_io.io_base import IntakeIOBase
from components.intake.intake_constants import IntakeConstants, IntakeCAN

from adaptive_robot.utils.math_utils import clamp
from adaptive_robot.utils.talon_faults import TalonFaultLogger

from phoenix6.hardware import TalonFXS



class RealIntakeIO(IntakeIOBase):
    """
    IO that allows the robot to run with real hardware.
    """
    def __init__(self) -> None:
        self.intake_motor = TalonFXS(IntakeCAN.INTAKE_MOTOR)

        self._check_motor_faults()
    
    def _check_motor_faults(self) -> None:
        """
        Checks for all motor faults and logs them.
        """
        talonfx_logger = TalonFaultLogger()

        motor = self.intake_motor
        talonfx_logger.report_talon_faults(motor, False, f"{motor.device_id}")
        talonfx_logger.report_talon_faults(motor, True, f"{motor.device_id}")

    def get_voltage(self) -> volts:
        return self.intake_motor.get_motor_voltage().value

    def set_voltage(self, voltage: volts) -> None:
        self.intake_motor.setVoltage(
            clamp(voltage, -IntakeConstants.MAX_VOLTAGE, IntakeConstants.MAX_VOLTAGE)
        )
    
    def update(self) -> None:
        pass
