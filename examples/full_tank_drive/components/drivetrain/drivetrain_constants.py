from wpimath.units import (
    meters, 
    meters_per_second, 
    radians_per_second, 
    kilograms, 
    kilogram_square_meters,
    volts
)


class DriveConstants:
    """
    Holds all constants that are used throughout the drivetrain component.
    """
    MAX_VOLTAGE: volts = 12.0
    NOMINAL_VOLTAGE: volts = 12.0

    ROBOT_MASS: kilograms = 45.2

    MAX_LINEAR_SPEED: meters_per_second = 4.2
    MAX_ANGULAR_SPEED: radians_per_second = 9.6

    WHEEL_DIAMETER: meters = 0.161
    TRACK_WIDTH: meters = 0.6

    GEAR_RATIO = 10.71
    MOI: kilogram_square_meters = 6.0


class DriveCAN:
    """
    Holds the CAN ids for each motor used in the component.
    """
    FRONT_LEFT = 1
    FRONT_RIGHT = 2
    BACK_LEFT = 3
    BACK_RIGHT = 4


class DriveLeftPID:
    """
    Holds the left velocity PID gains for the drivetrain component.
    """
    KP = 0.0
    KI = 0.13
    KD = 0.07


class DriveRightPID:
    """
    Holds the right velocity PID gains for the drivetrain component.
    """
    KP = 0.0
    KI = 0.13
    KD = 0.07


class DriveLeftFF:
    """
    Holds the left velocity FF gains for the drivetrain component.
    """
    KS = 0.0
    KV = 2.8571
    KA = 0.0


class DriveRightFF:
    """
    Holds the right velocity FF gains for the drivetrain component.
    """
    KS = 0.0
    KV = 2.8572
    KA = 0.0
