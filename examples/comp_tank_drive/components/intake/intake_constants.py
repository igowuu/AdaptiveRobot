from wpimath.units import volts, radians_per_second, meters


class IntakeConstants:
    MAX_VOLTAGE: volts = 12.0
    MAX_VELOCITY: radians_per_second = 65.0

    INTAKE_LENGTH: meters = 0.6604 # Span of robot (26 in.)
    INTAKE_WIDTH: meters = 0.254   # Amount of space balls have to be fed into the intake.

    GEAR_RATIO = 1.5


class IntakeCAN:
    """
    Holds the CAN ids for each motor used in the component.
    """
    INTAKE_MOTOR = 7
