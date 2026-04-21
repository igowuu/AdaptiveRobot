from adaptive_robot import Faultable, FaultSeverity

from rev import SparkMax


class SparkMaxFaultLogger(Faultable):
    """
    Handles and reports all SparkMax faults and warnings, raising them as Fault objects if caught.
    """
    FAULT_DEFINITIONS = [
        ('firmware', 'FIRMWARE', FaultSeverity.ERROR),
        ('can', 'CAN', FaultSeverity.ERROR),
        ('escEeprom', 'ESC_EEPROM', FaultSeverity.ERROR),
        ('motorType', 'MOTOR_TYPE', FaultSeverity.ERROR),
        ('sensor', 'SENSOR', FaultSeverity.ERROR),
        ('temperature', 'TEMPERATURE', FaultSeverity.ERROR),
        ('gateDriver', 'GATE_DRIVER', FaultSeverity.ERROR),
        ('other', 'OTHER', FaultSeverity.ERROR)
    ]

    WARNING_DEFINITIONS = [
        ('brownout', 'BROWNOUT', FaultSeverity.WARNING),
        ('escEeprom', 'ESC_EEPROM', FaultSeverity.WARNING),
        ('extEeprom', 'EXT_EEPROM', FaultSeverity.WARNING),
        ('hasReset', 'HAS_RESET', FaultSeverity.WARNING),
        ('other', 'OTHER', FaultSeverity.WARNING), 
        ('overcurrent', 'OVERCURRENT', FaultSeverity.WARNING),
        ('sensor', 'SENSOR', FaultSeverity.WARNING),
        ('stall', 'STALL', FaultSeverity.WARNING)
    ]

    def report_sparkmax_faults(self, motor: SparkMax, sticky: bool, name: str = "Unknown") -> None:
        """
        Raises a Fault object if any SparkMax contains any faults or sticky faults.

        :param motor: The motor to report faults from.
        :param sticky: Determines whether to retrive sticky faults or normal faults.
        :param module_name: The motor name, used for logging.
        """
        faults = motor.getStickyFaults() if sticky else motor.getFaults()
        
        for fault_attr, display_name, severity in self.FAULT_DEFINITIONS:
            if getattr(faults, fault_attr, False):
                self.raise_fault(
                    None,
                    severity,
                    f"SparkMax fault [{display_name}] on motor '{name}'"
                )

    def report_sparkmax_warnings(self, motor: SparkMax, sticky: bool, name: str = "Unknown") -> None:
        """
        Raises a Fault object if any SparkMax contains any warnings or sticky warnings.

        :param motor: The motor to report warnings from.
        :param sticky: Determines whether to retrive sticky warnings or normal warnings.
        :param module_name: The motor name, used for logging.
        """
        warnings = motor.getStickyWarnings() if sticky else motor.getWarnings()
        
        for warning_attr, display_name, severity in self.WARNING_DEFINITIONS:
            if getattr(warnings, warning_attr, False):
                self.raise_fault(
                    None,
                    severity,
                    f"SparkMax warning [{display_name}] on motor '{name}'"
                )
