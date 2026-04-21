from wpimath.units import volts


class IntakeConstants:
    MAX_VOLTAGE: volts = 12.0


class IntakeCAN:
    """
    Holds the CAN ids for each motor used in the component.
    """
    INTAKE_MOTOR = 7
