from wpimath.units import (
    meters, 
    volts, 
    radians, 
    radians_per_second, 
    kilograms, 
    kilogram_square_meters
)
from wpimath.geometry import Translation2d


class ShooterConstants:
    """
    Holds all of the constants for the shooter mechanism.
    """
    MAX_VOLTAGE: volts = 12.0
    MAX_VELOCITY: radians_per_second = 400.0

    FLYWHEEL_RADIUS: meters = 0.0508
    GEAR_RATIO = 1.5

    SHOOTER_HEIGHT: meters = 0.635  # Height of the shooter relative to the ground
    HUB_HEIGHT: meters = 2.64       # Height of the hub from the ground
    SHOOTER_HOOD_ANGLE: radians = 1.04
    SHOOTER_MOI: kilogram_square_meters = 0.00586

    FUEL_WEIGHT: kilograms = 0.1134
    FUEL_DIAMETER: meters = 5.91

    HUB_POSITION = Translation2d(4.035, 4.03)


class ShooterCAN:
    """
    Holds the CAN ids for each motor used in the component.
    """
    LEFT_SHOOTER_MOTOR = 8
    RIGHT_SHOOTER_MOTOR = 9
